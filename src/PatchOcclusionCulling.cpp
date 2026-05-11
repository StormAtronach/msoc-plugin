#include "PatchOcclusionCulling.h"

#include "Log.h"
#include "MemoryUtil.h"
#include "Config.h"
// LAYER-A-HORIZON: 1D horizon → curtain occluder used by the Horizon
// mode of rasterizeAggregateTerrain. See src/HorizonOccluder.h.
#include "HorizonOccluder.h"
// Freeze-forensics watchdog. Owns the watchdog thread, its stage-name
// table, and the spawn gate. This TU implements the read accessor
// (forensics::captureSnapshot) it calls back into.
#include "PatchForensicsWatchdog.h"

#include "TES3Cell.h"
#include "TES3DataHandler.h"
#include "TES3WorldController.h"

#include "NIAVObject.h"
#include "NICamera.h"
#include "NIColor.h"
#include "NIDefines.h"
#include "NIDynamicEffect.h"
#include "NIGeometryData.h"
#include "NILight.h"
#include "NINode.h"
#include "NIProperty.h"
#include "NITArray.h"
#include "NITransform.h"
#include "NITriShape.h"
#include "NITriShapeData.h"

#include "CullingThreadpool.h"
#include "MaskedOcclusionCulling.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <ostream>
#include <thread>
#include <unordered_map>
#include <vector>

namespace msoc::patch::occlusion {

	// `log::getLog()` call sites resolved to mwse::log in MWSE proper;
	// this alias keeps them unchanged.
	namespace log = ::msoc::log;

	// MSOC tile-buffer resolution. Decoupled from the game viewport.
	// Latched from Configuration at install time; restart-only.
	unsigned int kMsocWidth  = 512;
	unsigned int kMsocHeight = 256;

	// Clip-space w floor; MOC's near plane. 1.0 sits below the engine's
	// own near plane and above the numerical noise that explodes NDC
	// after perspective divide. Lowering risks NaN in projection math.
	constexpr float kNearClipW = 1.0f;

	// NI::Camera accessors that read fields the upstream MWSE NICamera.h
	// labels `unknown_*`. A proposed (unmerged) header adds named fields
	// (cullingPlanePtrs at 0x148, countCullingPlanes at 0x160,
	// usedCullingPlanesBitfield at 0x1C4); these accessors derive the
	// same data from the upstream-decoded cullingPlanes[6]:
	//   - countCullingPlanes is always 6 for the main world camera.
	//   - cullingPlanePtrs[i] is a TArray<NiPlane*> whose first 6 entries
	//     mirror &cullingPlanes[i] (NiCamera::UpdateWorldData copies via
	//     those pointers).
	//   - usedCullingPlanesBitfield lives at &cullingPlanes[6] — past
	//     the inline array. Computed via offset arithmetic to avoid the
	//     past-end-deref UB of &cullingPlanes[6].x.
	// Swap bodies to `cam->countCullingPlanes` etc. when upstream merges.
	static inline int cameraCountCullingPlanes(const NI::Camera* /*cam*/) {
		return 6;
	}

	static inline const TES3::Vector4* cameraCullingPlane(NI::Camera* cam, int i) {
		return &cam->cullingPlanes[i];
	}

	static inline uint32_t* cameraUsedPlanesMask(NI::Camera* cam) {
		auto* base = reinterpret_cast<char*>(&cam->cullingPlanes[0]);
		return reinterpret_cast<uint32_t*>(base + sizeof(TES3::Vector4) * 6);
	}

	// ============================================================
	// MSOC instance & per-frame camera state
	// ============================================================

	// Heap-allocated via Create()/Destroy(). Leaked on process exit.
	static ::MaskedOcclusionCulling* g_msoc = nullptr;
	// Double-buffer. g_msoc_prev holds the PREVIOUS frame's completed
	// mask — what external consumers (MGE-XE) read from. We rasterize
	// into g_msoc each frame and swap at drain-complete so consumers
	// always see "the mask as of the end of the previous frame".
	static ::MaskedOcclusionCulling* g_msoc_prev = nullptr;
	// Matrix + NDC constants snapshotted at the swap, so external queries
	// project through the same view the depth data was built with.
	static float g_worldToClip_prev[16] = {};
	static float g_ndcRadiusX_prev = 0.0f;
	static float g_ndcRadiusY_prev = 0.0f;
	static float g_wGradMag_prev   = 0.0f;

	// Freshness gate. External queries reject snapshots older than
	// kSnapshotMaxAgeMs (alt-tab, menu, loading screen). 0 = never captured.
	static unsigned long long g_snapshotTickMs = 0;
	static constexpr unsigned long long kSnapshotMaxAgeMs = 200;

	// External occluder injection. Each PendingOccluder is a self-
	// contained copy of a consumer's submission; plugin owns the memory.
	// Populated on the consumer's thread, drained on the render thread
	// before native near-scene occluders rasterize. Mutex contention is
	// trivial in practice (consumers submit a handful of batches per
	// frame; plugin drains once per frame).
	//
	// g_externalOccluderTrisQueued enforces external-first rejection
	// against OcclusionOccluderMaxTriangles, preserving the native budget.
	struct PendingOccluder {
		std::vector<float> verts;
		std::vector<std::uint32_t> tris;
		int stride;
		int offY;
		int offW;
		int vtxCount;
		int triCount;
		std::optional<std::array<float, 16>> modelMatrix;
		// True: verts are clip-space (x, y, w); drain passes nullptr to
		// RenderTriangles. Used by screen-space submitters (horizon curtains).
		bool preTransformed = false;
	};

	static std::vector<PendingOccluder> g_pendingOccluders;
	static std::mutex g_pendingOccludersMutex;
	static int g_externalOccluderTrisQueued = 0;

	static void drainPendingOccluders();

	// Transposed from NI's row-major M*v into Intel's column-major v*M
	// layout (consecutive memory = one column). Refreshed at top-level
	// CullShow_detour entry.
	static float g_worldToClip[16];

	// Reusable rasterizeTriShape buffers, grown on demand, never shrunk.
	// MSOC is single-threaded from the worldCamera pass.
	static std::vector<float> g_occluderVerts;
	static std::vector<unsigned int> g_occluderIndices;

	// Per-frame matrix metrics for testSphereVisible:
	//   g_ndcRadiusX/Y: L2 norm of clip.x/y coefficients. NDC half-extent
	//                   of a sphere of radius r at clip-w cw is r*X/cw.
	//   g_wGradMag:     L2 norm of clip.w coefficients. Worst-case clip-w
	//                   offset from sphere center to near surface is r*mag.
	//                   1.0 for standard perspective on pure-rotation view;
	//                   computed for safety against scaled views.
	static float g_ndcRadiusX = 0.0f;
	static float g_ndcRadiusY = 0.0f;
	static float g_wGradMag = 0.0f;

	// DataHandler::worldLandscapeRoot, captured per top-level frame.
	// Drain uses it to short-circuit occludee queries on terrain patches
	// (25v/32t, ~always visible). Null before the world exists.
	static NI::Node* g_worldLandscapeRoot = nullptr;

	// True only while renderMainScene (0x41C400) is on the stack. Gates
	// MSOC so Click trees outside the main scene — load splash, UI
	// targets, chargen preview, MGE water reflection — run vanilla.
	// Those cameras aren't validated for occlusion, and main-scene is
	// the only place MSOC's cost pays back.
	static bool g_inRenderMainScene = false;

	// Diagnostics for engines that fire multiple main-camera CullShow
	// passes per renderMainScene. Each pass would run ClearBuffer + its
	// own subtree, so the LAST pass clobbers earlier ones. attempts
	// counts raw entries; fires counts those that survived the
	// alreadyBuiltThisScene guard. Both reset at renderMainScene_wrapper.
	static unsigned int g_isTopLevelFiresThisScene = 0;
	static unsigned int g_maxIsTopLevelFiresSession = 0;
	static unsigned int g_mainCamCullShowAttemptsThisScene = 0;
	static unsigned int g_maxMainCamCullShowAttemptsSession = 0;

	// True only while the worldCamera main pass is being traversed.
	// Gates MSOC so shadow-manager, water-refraction, armCamera, and
	// other non-main Clicks inside renderMainScene run vanilla.
	static bool g_msocActive = false;

	// True while the depth buffer reflects the complete vanilla
	// main-scene occluder set. Set after drainPendingDisplays in the
	// top-level cullShow_detour, cleared at the next ClearBuffer().
	// External consumers gate TestRect on this; queries before drain
	// would hit a partial buffer and falsely report VISIBLE.
	static bool g_maskReady = false;

	// Per-frame diagnostics. Reset at the top of each worldCamera traversal.
	static uint64_t g_recursiveCalls = 0;
	static uint64_t g_recursiveAppCulled = 0;
	static uint64_t g_recursiveFrustumCulled = 0;
	static uint64_t g_rasterizedAsOccluder = 0;
	// Sum of outTri across successful rasterizeTriShape. Used to size the
	// threadpool queue: occluderTriangles/TRIS_PER_JOB + rasterizedAsOccluder
	// is the upper bound on queue writes per frame.
	static uint64_t g_occluderTriangles = 0;
	static uint64_t g_skippedInside = 0;
	static uint64_t g_skippedThin = 0;
	static uint64_t g_skippedAlpha = 0;
	static uint64_t g_skippedStencil = 0;
	static uint64_t g_queryTested = 0;
	static uint64_t g_queryOccluded = 0;
	static uint64_t g_queryViewCulled = 0;
	// Atomic so phase-1 drain workers can race the increment safely.
	// Relaxed: read for diagnostic logging only, never control flow.
	static std::atomic<uint64_t> g_queryNearClip{0};
	static uint64_t g_deferred = 0;
	// Inline (during traversal) vs deferred (drain-phase) test counts —
	// helps attribute queryOccluded between "whole subtree culled early"
	// and "leaf shape hidden behind a fully-populated buffer."
	static uint64_t g_inlineTested = 0;
	// Phase 2.2 gate counters. Non-zero at default config means a gate
	// is mis-placed (defaults keep these at 0 on typical scenes).
	static uint64_t g_skippedTriCount = 0;
	static uint64_t g_skippedTesteeTiny = 0;
	static uint64_t g_skippedSceneGate = 0;
	// CullShow_detour entries while menu mode is set. Cumulative; the
	// per-frame reset doesn't run on these frames.
	static uint64_t g_skippedMenuMode = 0;
	// Terrain occludees bypassed from TestRect — patches under the
	// camera are nearly always visible.
	static uint64_t g_skippedTerrain = 0;
	// Aggregate terrain submissions: one combined RenderTriangles per
	// visible Land. 0 unless OcclusionAggregateTerrain is on.
	static uint64_t g_aggregateTerrainLands = 0;
	static uint64_t g_aggregateTerrainTris = 0;
	// Wall-clock in rasterizeAggregateTerrain (walk + transform + submit).
	// Includes async enqueue; worker rasterization happens in parallel
	// and is not counted here.
	static uint64_t g_aggregateTerrainUs = 0;

	// Horizon-mode counters (active when g_aggregateTerrainEffective == 2):
	//   g_horizonBuildUs:  end-to-end (project + simplify + emit + submit).
	//   g_horizonRasterUs: inner subset — only the curtain RenderTriangles.
	// Pure construction = build - raster. Curtain raster also accumulates
	// into g_rasterizeTimeUs alongside other occluders.
	static uint64_t g_horizonBuildUs        = 0;
	static uint64_t g_horizonRasterUs       = 0;
	static uint64_t g_horizonLandsFed       = 0;
	static uint64_t g_horizonVertsFed       = 0;
	static uint64_t g_horizonColumnsTouched = 0;
	static uint64_t g_horizonCurtainTris    = 0;
	static float    g_horizonAdaptiveEpsD   = 0;

	// ============================================================
	// Cache types & globals (land / drain / light)
	// ============================================================

	// Per-Land merged occluder cache. Key = per-Land NiNode under
	// WorldLandscape; value = world-space vertex+index buffers prebuilt
	// on first sight. Mark-and-sweep evicted per frame.
	//
	// nodePtr (NI::Pointer) refcounts the NiNode so the engine can't
	// free a key we still hold. Sweep eviction releases the ref.
	struct LandCacheEntry {
		NI::Pointer<NI::Node> nodePtr;
		std::vector<float> verts;
		std::vector<unsigned int> indices;
		unsigned int triCount = 0;
		bool seen = false;
		// Mismatch with current OcclusionTerrainResolution forces a
		// rebuild on hit, so MCM changes take effect lazily without
		// a global cache flush. 0xff = uninitialised.
		uint8_t builtForResolution = 0xff;

		// Per-subcell ranges into `indices` for per-subcell frustum-cull
		// at submit time. `node` is the subcell NiNode used per frame
		// as the frustumCulledSphere argument.
		struct SubcellRange {
			NI::Node* node;
			unsigned int firstIdx;
			unsigned int triCount;
		};
		std::vector<SubcellRange> subcellRanges;
	};
	static std::unordered_map<NI::Node*, LandCacheEntry> g_landCache;
	static uint64_t g_landCacheHits = 0;
	static uint64_t g_landCacheMisses = 0;
	static uint64_t g_landCacheEvictions = 0;

	// Temporal coherence cache for drain-phase TestRect verdicts. Keyed
	// by NI::AVObject*. OcclusionTemporalCoherenceFrames N: 0 disables;
	// N>0 reuses a fresh entry for up to N frames.
	//
	// Caches only OCCLUDED verdicts — VISIBLE and VIEW_CULLED fall
	// through to display() anyway, so caching them adds stale-pointer
	// surface for no hit-path win.
	//
	// Move detection compares worldBound origin + radius against the
	// snapshot taken at query time.
	//
	// shapePtr (NI::Pointer) refcounts the AVObject key. Without it,
	// raising N widens the window where a destroyed-but-not-yet-pruned
	// entry could collide with a recycled pointer. Age-prune at 2*N
	// frames bounds memory.
	struct DrainCacheEntry {
		NI::Pointer<NI::AVObject> shapePtr;
		::MaskedOcclusionCulling::CullingResult result;
		uint32_t lastQueryFrame;
		float boundOriginX, boundOriginY, boundOriginZ;
		float boundRadius;
	};
	static std::unordered_map<NI::AVObject*, DrainCacheEntry> g_drainCache;
	static uint64_t g_drainCacheHits = 0;
	static uint64_t g_drainCacheMisses = 0;

	// Terrain-descendant membership cache. Replaces a 7-deep parent-
	// chain walk (called twice per deferred leaf) with a flat lookup.
	// On a peak Vivec frame: 2354 calls × 7.3 avg = 17k pointer chases
	// per frame, dominated by L2 misses on cold scene-graph nodes.
	//
	// Wiped on cell change. Within a cell, scene-graph parentage and
	// g_worldLandscapeRoot are stable. Mid-cell reparenting is rare;
	// the failure mode is benign (wrong "is terrain" → harmless under-
	// occlusion for one item).
	struct TerrainMembershipEntry {
		NI::Pointer<NI::AVObject> objPtr;
		bool isDescendant;
	};
	static std::unordered_map<NI::AVObject*, TerrainMembershipEntry> g_terrainMembership;
	static uint64_t g_terrainMembershipHits   = 0;
	static uint64_t g_terrainMembershipMisses = 0;

	// Per-light occlusion cache for updateLights (gated by
	// OcclusionCullLights). Tracks last query frame, last verdict,
	// consecutive-occluded count for hysteresis, and worldBound snapshot
	// for move detection.
	//
	// lightPtr (NI::Pointer) refcounts the NiLight so a freed-then-
	// reallocated address can't collide with a stale entry mid-cell.
	struct LightCullEntry {
		NI::Pointer<NI::Light> lightPtr;
		uint32_t lastQueryFrame;
		uint8_t consecOccluded;
		bool cullActive;
		float boundOriginX, boundOriginY, boundOriginZ;
		float boundRadius;
	};
	static std::unordered_map<NI::Light*, LightCullEntry> g_lightCullCache;
	static uint64_t g_lightsTested = 0;
	static uint64_t g_lightsOccluded = 0;
	static uint64_t g_lightCullCacheHits = 0;
	static uint64_t g_lightCullCacheMisses = 0;

	// Light observers — registered by external consumers (MGE-XE) that
	// want the live renderer-iterated light list without re-patching
	// 0x6BB7D4. Iterated lock-free from the render thread.
	static std::vector<LightObservedCallback> g_lightObservers;

	// ============================================================
	// Frame counters & diagnostic state
	// ============================================================

	// File-scope frame counter; incremented once per top-level frame.
	// Used by the drain loop for cache-freshness checks.
	static uint32_t g_frameCounter = 0;
	// Cell pointer across frames. Cell change → wipe g_landCache and
	// g_drainCache (NI::Node*/NI::AVObject* recycle in the new cell).
	static TES3::Cell* g_lastCell = nullptr;
	static uint64_t g_cellChanges = 0;

	// Phase timers (µs). steady_clock is QPC-backed on MSVC. Rasterize
	// accumulates each RenderTriangles call; drainPhase wraps the whole
	// drain body; classify wraps phase 1 (the verdict pass) on the main
	// thread — single writer in both serial and parallel modes.
	static uint64_t g_rasterizeTimeUs = 0;
	static uint64_t g_drainPhaseTimeUs = 0;
	static uint64_t g_classifyUs = 0;
	static uint64_t g_drainDisplayUs = 0;

	// Hybrid phase budgeting (Configuration::Occlusion*BudgetUs):
	//   Layer 1 — predictive skip: if EMA(last frames) > 2× budget,
	//     skip the whole phase this frame. Self-regulating, no in-loop
	//     cost. 2× multiplier so single spikes don't trigger skipping.
	//   Layer 2 — spike clip: rasterize checks cumulative time at the
	//     top of each rasterizeTriShape; classify samples every 32
	//     iterations of classifyDrainRange. Bails when budget is hit;
	//     remaining work is conservative (occluders skipped, testees
	//     marked Visible).
	//
	// Both target MSOC-only work, not vanilla rendering: rasterize
	// budget bounds RenderTriangles SIMD only (not cullShowBody
	// traversal); classify bounds TestRect only (not phase 2's
	// display() vanilla D3D8 submissions, which run regardless).
	//
	// EMA = (prev * 7 + sample) >> 3 — ~6-frame half-life.
	static uint64_t g_rasterizeEmaUs = 0;
	static uint64_t g_classifyEmaUs  = 0;
	static uint64_t g_rasterizeBudgetUsEffective = 0;
	static uint64_t g_classifyBudgetUsEffective  = 0;
	static bool     g_skipRasterizeThisFrame = false;
	static bool     g_skipClassifyThisFrame  = false;
	static std::chrono::steady_clock::time_point g_classifyPhaseStart;
	static uint32_t g_rasterizeBudgetTrips = 0;
	static uint32_t g_classifyBudgetTrips  = 0;
	static uint64_t g_rasterizeBudgetTripsSession = 0;
	static uint64_t g_classifyBudgetTripsSession  = 0;

	static inline uint64_t elapsedUsSince(std::chrono::steady_clock::time_point t0) {
		return static_cast<uint64_t>(
			std::chrono::duration_cast<std::chrono::microseconds>(
				std::chrono::steady_clock::now() - t0).count());
	}

	static inline uint64_t emaUpdate(uint64_t prev, uint64_t sample) {
		return (prev * 7 + sample) >> 3;
	}
	// Phase 3.2: time spent in threadpool Flush() barrier before drain.
	// Only populated when async mode is active; zero otherwise.
	static uint64_t g_asyncFlushTimeUs = 0;
	// Time inside the threadpool's WakeThreads() spin. Should be ~0 every
	// frame; non-trivial values mean a worker didn't reach its suspended
	// state from the previous SuspendThreads(). max-seen kept lifetime
	// so a single outlier shows up in the next periodic log.
	static uint64_t g_wakeThreadsTimeUs = 0;
	static uint64_t g_maxWakeThreadsUsSession = 0;
	// Recursion diagnostic: a debugger during a freeze can read g_callDepth
	// directly to spot a runaway tree (cycle, broken sentinel) before
	// stack overflow.
	static uint32_t g_callDepth = 0;
	static uint32_t g_maxCallDepthThisFrame = 0;
	static uint32_t g_maxCallDepthSession = 0;

	// RAII for g_callDepth. Used exclusively by CullShow_detour.
	struct CallDepthGuard {
		CallDepthGuard() {
			++g_callDepth;
			if (g_callDepth > g_maxCallDepthThisFrame) {
				g_maxCallDepthThisFrame = g_callDepth;
			}
		}
		~CallDepthGuard() { --g_callDepth; }
	};
	// Last-checkpoint marker. Stable numbering — DO NOT renumber:
	//   0 idle, 1 entered top-level, 2 wakeThreads, 3 clearBuffer,
	//   4 cellWipe, 5 ageprune, 6 uploadCamera, 7 setMatrix,
	//   8 aggTerrain, 9 cullShowBody, 10 flush, 11 drain,
	//   12 suspendThreads, 13 exitedClean,
	//   14 drainClassify, 15 drainAction.
	// Mirrored in PatchForensicsWatchdog::stageName().
	static volatile uint32_t g_lastStage = 0;

	// Published at end of each top-level frame (after g_lastStage = 13).
	// Forensics watchdog reads this for "time since last clean exit."
	static std::atomic<uint64_t> g_lastFrameEndTimeMs{0};

	static bool ensureMSOCResourcesMatchConfig();

	// ============================================================
	// Threadpool state & per-frame config cache
	// ============================================================

	// g_threadpool lives for the process. Runtime-toggle of
	// OcclusionAsyncOccluders works without restart — Wake/Suspend is
	// driven per-frame off that flag, suspended threadpool sleeps at ~0%.
	//
	// g_asyncThisFrame latches the flag at top-level entry so mid-frame
	// MCM edits don't mix sync/async submissions within one traversal.
	//
	// Arena vectors own per-submission triangle data until Flush() retires
	// the job. CullingThreadpool requires caller-owned buffers to stay
	// stable between submission and completion; the rasterize path moves
	// g_occluderVerts/Indices into the arena and allocates fresh ones for
	// the next call. Cleared at top-level entry after ClearBuffer's
	// implicit Flush.
	static ::CullingThreadpool* g_threadpool = nullptr;
	static bool g_asyncThisFrame = false;
	static std::vector<std::vector<float>> g_asyncOccluderVerts;
	static std::vector<std::vector<unsigned int>> g_asyncOccluderIndices;

	// Scene-type-resolved occluder thresholds. Written once per top-level
	// entry after isInterior is known; collapses interior/exterior pair-
	// selection out of the hot path.
	static float g_occluderRadiusMinEffective     = 0.0f;
	static float g_occluderRadiusMaxEffective     = 0.0f;
	static float g_occluderMinDimensionEffective  = 0.0f;
	static float g_insideOccluderMarginEffective  = 0.0f;

	// Per-frame cache of loop-invariant Configuration reads. Configuration
	// statics have external linkage; the optimizer must reload them after
	// each opaque MOC/NI call. Caching into internal-linkage statics lets
	// the compiler hoist reads out of hot loops and register-allocate
	// across opaque calls. Contract: MCM only commits between frames, so
	// one refresh per top-level CullShow is sound. Defaults match
	// Config.cpp so pre-refresh reads (init, off-frame light callback)
	// stay consistent.
	static float        g_depthSlackWorldUnitsEffective      = 128.0f;
	static unsigned int g_occluderMaxTrianglesEffective      = 4096;
	static float        g_occludeeMinRadiusEffective         = 1.0f;
	static bool         g_skipTerrainOccludeesEffective      = true;
	// 0=Off, 1=Raster, 2=Horizon.
	static int          g_aggregateTerrainEffective          = 1;
	static unsigned int g_terrainResolutionEffective         = 1;
	static unsigned int g_temporalCoherenceFramesEffective   = 4;
	static bool         g_cullLightsEffective                = true;
	static unsigned int g_lightCullHysteresisFramesEffective = 3;
	static bool         g_tintOccluderEffective              = false;
	static bool         g_tintOccludedEffective              = false;
	static bool         g_tintTestedEffective                = false;
	static bool         g_logEnabledEffective                = false;

	// Drain-parallel worker pool — DISABLED. Serial drain is the only
	// active path. To re-enable, restore: Configuration::OcclusionParallelDrain
	// (Config.h/.cpp + Lua config/mcm), drainWorkerMain, the worker-pool
	// spawn in installPatches, and the runParallel branch in
	// drainPendingDisplays. Sizing reference: ~840 TestRect/frame in dense
	// scenes, 2-way split (hiZ cache traffic dominates beyond that).
	//
	// constexpr unsigned int kDrainWorkerCount = 2;
	// constexpr size_t kParallelDrainMin = 100;
	// struct DrainWorkerRange { size_t lo; size_t hi; };
	// static std::jthread g_drainWorkers[kDrainWorkerCount];
	// static DrainWorkerRange g_drainRanges[kDrainWorkerCount];
	// static std::mutex g_drainMtx;
	// static std::condition_variable_any g_drainStartCv;
	// static std::condition_variable g_drainDoneCv;
	// static unsigned int g_drainStartTickets = 0;
	// static unsigned int g_drainDoneTickets = 0;

	// ============================================================
	// Forensics watchdog — read accessor
	// ============================================================

	// Sole bridge to PatchForensicsWatchdog.cpp. Defined here so the
	// hot-path statics stay internal-linkage. Called once per ~250ms;
	// no synchronisation — single loads, post-mortem-grade coherence.
	forensics::Snapshot forensics::captureSnapshot() {
		forensics::Snapshot s{};
		s.stage               = g_lastStage;
		s.callDepth           = g_callDepth;
		s.maxCallDepthSession = g_maxCallDepthSession;
		s.frame               = g_frameCounter;
		s.lastFrameEndMs      = g_lastFrameEndTimeMs.load(
			std::memory_order_relaxed);
		s.asyncThisFrame      = g_asyncThisFrame;
		s.threadpoolAlive     = g_threadpool != nullptr;
		return s;
	}

	// ============================================================
	// Deferred-display queue & timing helpers
	// ============================================================

	// Gated on the log channels — when both are off the timer is a no-op
	// (nullptr target, no clock reads). Read-once at construction so a
	// mid-scope MCM toggle can't flip the dtor onto a still-zero start.
	struct ScopedUsAccumulator {
		uint64_t* target;
		std::chrono::steady_clock::time_point start;
		explicit ScopedUsAccumulator(uint64_t& t)
			: target(g_logEnabledEffective ? &t : nullptr) {
			if (target) start = std::chrono::steady_clock::now();
		}
		~ScopedUsAccumulator() {
			if (target) {
				*target += static_cast<uint64_t>(
					std::chrono::duration_cast<std::chrono::microseconds>(
						std::chrono::steady_clock::now() - start).count());
			}
		}
	};

	// Deferred-display queue for small NiTriShape leaves. Queued during
	// traversal and drained AFTER all occluder rasterisation, so TestRect
	// runs against a fully-populated depth buffer. Single traversal vs
	// OpenMW's two-pass.
	//
	// Only NiTriShape leaves defer; NiNodes stay inline so their subtree
	// keeps contributing occluders during the main pass.
	struct PendingDisplay {
		NI::AVObject* shape;
		NI::Camera* camera;
		// Skip the drain-phase tint write so the occluder (yellow) tint
		// from rasterizeTriShape isn't overwritten by Tested/Occluded.
		// TestRect still runs.
		bool rasterisedAsOccluder;
	};
	static std::vector<PendingDisplay> g_pendingDisplays;

	// Drain phase-1 verdict slots, populated by classifyDrainRange and
	// consumed by phase 2. Phase 1 stays read-only on shared state —
	// prerequisite for moving it to worker threads.
	enum class DrainVerdict : uint8_t {
		Visible,        // VISIBLE; call display()
		Occluded,       // OCCLUDED; skip display (or tint+display in debug)
		ViewCulled,     // rect collapsed; treat like Visible for display()
		SkipTerrain,    // bypassed TestRect (terrain descendant); call display()
		SkipTiny,       // bypassed TestRect (radius < threshold); call display()
		CachedOccluded, // g_drainCache hit; verdict OCCLUDED (only cached verdict)
	};

	struct DrainSlot {
		DrainVerdict verdict;
		// True only when testSphereVisible actually ran. False for
		// Skip*/CachedOccluded. Phase 2 uses this to gate counter
		// increments that fired only on the !reused branch pre-refactor.
		bool ranTestRect;
	};

	static std::vector<DrainSlot> g_drainSlots;

	// ============================================================
	// Debug tinting & occluder property classification
	// ============================================================

	// Debug tint overlay. DebugOcclusionTint* flags select hues for
	// classified leaves:
	//   red    — OCCLUDED (kept visible for visual compare)
	//   green  — survived the MSOC query
	//   yellow — rasterised as an occluder
	//
	// Persistent-clone lifetime: first tint on a shape clones the
	// effective MaterialProperty, detaches any own material, attaches
	// the clone. Subsequent frames only overwrite emissive + bump
	// revisionID. Avoids the per-frame allocator churn that previously
	// crashed the DX8 per-material state cache (~1.7k new/delete per
	// frame; recycled addresses returned dangling D3D state).
	//
	// End of each main-scene frame: reset every clone to the source
	// material's current emissive so non-classified frames render
	// untinted. Shapes that never get tinted never allocate.
	//
	// Morrowind shares NiMaterialProperty via inheritance (parent NiNode
	// material covers descendants without their own) AND via NIF
	// streamable dedup. Cloning avoids both stains. Only NiTriBasedGeom
	// leaves tint — a NiNode tint propagates to its whole subtree.
	static const NI::Color kTintOccluded(1.0f, 0.0f, 0.0f);
	static const NI::Color kTintTested(0.0f, 1.0f, 0.0f);
	static const NI::Color kTintOccluder(1.0f, 1.0f, 0.0f);

	// NiMaterialProperty vtable. MWSE exposes no public ctor; we
	// open-code the engine pattern (Property::Property sets base vtable;
	// concrete subclasses overwrite).
	constexpr uintptr_t kNiMaterialPropertyVTable = 0x75036C;

	struct TintClone {
		// Refcounts the shape so map key + attached clone stay valid
		// across cell unload.
		NI::Pointer<NI::AVObject> shape;
		// Refcounts the source so end-of-frame reset can read its
		// current emissive. For own-material shapes we detached it;
		// this Pointer is now the sole owner.
		NI::Pointer<NI::MaterialProperty> source;
		// Owned by the shape's property list (attachProperty refs it).
		// Safe raw-ptr because `shape` keeps the list alive.
		NI::MaterialProperty* clone;
	};
	static std::unordered_map<NI::AVObject*, TintClone> g_tintClones;

	// Walk obj → ancestors, return first MaterialProperty (engine's
	// property-inheritance rule).
	static NI::MaterialProperty* findInheritedMaterial(NI::AVObject* obj) {
		for (NI::AVObject* cur = obj; cur; cur = cur->parentNode) {
			for (auto* node = &cur->propertyNode; node && node->data; node = node->next) {
				if (node->data->getType() == NI::PropertyType::Material) {
					return static_cast<NI::MaterialProperty*>(node->data);
				}
			}
		}
		return nullptr;
	}

	// Single-pass occluder property classifier. Walks ancestors once,
	// resolving first-of-type-wins NiAlphaProperty and NiStencilProperty
	// in the same scan. Nearest-ancestor-wins applies independently per
	// type — alpha and stencil may live on different ancestors.
	//
	// Why these can't be occluders:
	//   alpha   — blended/alpha-tested shapes (fences, banners, grates,
	//             vines, leaves, glass) are partially transparent. A
	//             solid-occluder rasterise fills the full quad and
	//             falsely occludes whatever sits behind the holes.
	//   stencil — fills only where the test passes (shadow volumes,
	//             reflection clip masks, UI cutouts). Same hazard.
	// Both still participate as occludees; only the occluder rasterise
	// pass skips them.
	struct OccluderPropertyFlags {
		bool alpha;
		bool stencil;
	};
	static OccluderPropertyFlags classifyOccluderProperties(NI::AVObject* obj) {
		OccluderPropertyFlags out = { false, false };
		bool alphaResolved = false;
		bool stencilResolved = false;
		for (NI::AVObject* cur = obj; cur; cur = cur->parentNode) {
			for (auto* node = &cur->propertyNode; node && node->data; node = node->next) {
				const auto type = node->data->getType();
				if (!alphaResolved && type == NI::PropertyType::Alpha) {
					const unsigned short flags = node->data->flags;
					out.alpha = (flags & (NI::AlphaProperty::ALPHA_MASK
						| NI::AlphaProperty::TEST_ENABLE_MASK)) != 0;
					alphaResolved = true;
				}
				else if (!stencilResolved && type == NI::PropertyType::Stencil) {
					out.stencil = static_cast<NI::StencilProperty*>(node->data)->enabled;
					stencilResolved = true;
				}
				if (alphaResolved && stencilResolved) return out;
			}
		}
		return out;
	}

	// True iff obj sits under DataHandler's worldLandscapeRoot. Terrain
	// hits resolve in ~4 levels; non-terrain runs to scene root (~6-8).
	// Pre-cache profile (Vivec exterior peak): 2354 calls × 7.3 avg =
	// ~17k pointer chases per frame, dominated by L2 misses on cold
	// scene-graph nodes. Cached because parent-chain is stable within a
	// cell; wiped on cell change.
	static bool isLandscapeDescendant(NI::AVObject* obj, NI::Node* root) {
		if (!root || !obj) return false;
		auto it = g_terrainMembership.find(obj);
		if (it != g_terrainMembership.end()) {
			++g_terrainMembershipHits;
			return it->second.isDescendant;
		}
		++g_terrainMembershipMisses;
		bool result = false;
		for (NI::AVObject* cur = obj; cur; cur = cur->parentNode) {
			if (cur == root) { result = true; break; }
		}
		g_terrainMembership.emplace(
			obj, TerrainMembershipEntry{ NI::Pointer<NI::AVObject>(obj), result });
		return result;
	}

	// Standalone NiMaterialProperty seeded from source (if any) with the
	// tint forced into ambient + diffuse + emissive. All three channels:
	// a sibling NiVertexColorProperty may replace one of them from baked
	// per-vertex colors (SOURCE_IGNORE / SOURCE_EMISSIVE /
	// SOURCE_AMBIENT_DIFFUSE). World statics use SOURCE_EMISSIVE; skinned
	// actors use IGNORE/AMBIENT_DIFFUSE — tinting all three covers both.
	static NI::MaterialProperty* cloneMaterialProperty(NI::MaterialProperty* source, const NI::Color& tint) {
		// Returns refCount=1 (caller-owned, matching NiObject::Clone).
		// attachProperty's wrapper at 0x405840 has a Pointer cycle
		// around AddHead that net-zeros the input refcount; refCount=0
		// would 0→1→0 → DELETE, leaving the property list dangling.
		NI::MaterialProperty* mat;
		if (source) {
			// Engine's NiObject::Clone (0x6E9910): stream-based deep
			// copy, returns refCount=1. Preferred — picks up every
			// inherited field, even those we don't enumerate below.
			mat = static_cast<NI::MaterialProperty*>(source->createClone());
		}
		else {
			// Fresh allocation. NiObject::ctor sets refCount=0; bump to
			// match the createClone path.
			mat = new NI::MaterialProperty();
			mat->refCount = 1;
			mat->vTable.asProperty = reinterpret_cast<NI::Property_vTable*>(kNiMaterialPropertyVTable);
			mat->flags = 1;
			mat->index = 0;
			mat->specular = NI::Color(0.0f, 0.0f, 0.0f);
			mat->shininess = 10.0f;
			mat->alpha = 1.0f;
		}
		mat->ambient = tint;
		mat->diffuse = tint;
		mat->emissive = tint;
		mat->revisionID = 0;
		return mat;
	}

	static void tintEmissive(NI::AVObject* shape, const NI::Color& color) {
		// Only leaves: tinting a NiNode would stain every inheriting child.
		if (!shape->isInstanceOfType(NI::RTTIStaticPtr::NiTriBasedGeom)) {
			return;
		}

		// Subsequent tint on an already-cloned shape: overwrite emissive
		// only. No allocations, no property-list churn.
		auto it = g_tintClones.find(shape);
		if (it != g_tintClones.end()) {
			it->second.clone->emissive = color;
			it->second.clone->incrementRevisionId();
			return;
		}

		// First tint: clone the effective material, detach any own
		// material, attach the clone. findInheritedMaterial walks from
		// shape upward so it returns the own material when present,
		// falling back to an ancestor's.
		NI::MaterialProperty* source = findInheritedMaterial(shape);
		NI::MaterialProperty* clone = cloneMaterialProperty(source, color);

		// Detach returns null if the shape had no own material (pure
		// inheritance). Either way our clone becomes the leading (and
		// only) Material property on the shape.
		NI::Pointer<NI::Property> detachedOwn = shape->detachPropertyByType(NI::PropertyType::Material);
		shape->attachProperty(clone);
		// attachProperty mutates the raw list only; the renderer reads
		// from a per-Geometry effective-material cache that
		// updateProperties() rebuilds. Without this, draws keep using
		// the cached material until something else updates properties.
		shape->updateProperties();

		// Keep the source alive. If the own material was detached,
		// detachedOwn holds the sole ref now — promote it to our map
		// entry. Otherwise source points into an ancestor's property
		// list; take a fresh Pointer.
		NI::Pointer<NI::MaterialProperty> sourcePtr;
		if (detachedOwn) {
			sourcePtr = static_cast<NI::MaterialProperty*>(detachedOwn.get());
		}
		else {
			sourcePtr = source;
		}

		g_tintClones.emplace(shape, TintClone{ shape, sourcePtr, clone });
	}

	// End-of-frame tint reset: restore each tracked source's ambient +
	// diffuse + emissive into its clone (all three channels — tintEmissive
	// sets all three to handle NiVertexColorProperty SOURCE_*). Pointer-
	// stable; updateProperties() is required for the DX8 effective-
	// material cache to pick up the new colors.
	static void resetFrameTints() {
		const NI::Color zero(0.0f, 0.0f, 0.0f);
		for (auto& [key, entry] : g_tintClones) {
			if (entry.source) {
				entry.clone->ambient = entry.source->ambient;
				entry.clone->diffuse = entry.source->diffuse;
				entry.clone->emissive = entry.source->emissive;
			}
			else {
				entry.clone->ambient = zero;
				entry.clone->diffuse = zero;
				entry.clone->emissive = zero;
			}
			entry.clone->incrementRevisionId();
			entry.shape->updateProperties();
		}
	}

	// ============================================================
	// Camera & projection math
	// ============================================================

	// Transpose NI::Camera::worldToCamera into Intel's column-major v*M
	// layout. NI stores row-major M*v: clip[r] = Σ_c ni[r*4+c]*v[c].
	// Intel reads mtx[c*4+r] for the same out[r]. So mtx[c*4+r] = ni[r*4+c].
	static void uploadCameraTransform(NI::Camera* cam) {
		const float* ni = reinterpret_cast<const float*>(&cam->worldToCamera);
		for (int row = 0; row < 4; ++row) {
			for (int col = 0; col < 4; ++col) {
				g_worldToClip[col * 4 + row] = ni[row * 4 + col];
			}
		}

		// Per-frame sphere-projection metrics. ndcRadiusX/Y are operator
		// norms of the x/y rows (translation excluded) — upper bounds on
		// |d(clip)/d(pos)|. wGradMag is the same for clip-w. After the
		// transpose, row-r coefficients live at m[0+r], m[4+r], m[8+r].
		const float* m = g_worldToClip;
		g_ndcRadiusX = std::sqrt(m[0] * m[0] + m[4] * m[4] + m[8] * m[8]);
		g_ndcRadiusY = std::sqrt(m[1] * m[1] + m[5] * m[5] + m[9] * m[9]);
		g_wGradMag = std::sqrt(m[3] * m[3] + m[7] * m[7] + m[11] * m[11]);
	}

	// Project a world-space point through the cached matrix. Returns
	// clip-space (x, y, w); z is unused by Intel's rasterizer. Same
	// formula as Intel's TransformVerts so sphere rects stay aligned
	// with rasterised occluder depth.
	struct ClipXYW {
		float x, y, w;
	};
	static inline ClipXYW projectWorld(float wx, float wy, float wz) {
		const float* m = g_worldToClip;
		return {
			wx * m[0] + wy * m[4] + wz * m[8] + m[12],
			wx * m[1] + wy * m[5] + wz * m[9] + m[13],
			wx * m[3] + wy * m[7] + wz * m[11] + m[15],
		};
	}

	// Sphere → NDC-rect + wmin → TestRect. Projects only the center and
	// derives the NDC half-extent + near-surface clip-w from per-frame
	// matrix metrics. Tighter than an 8-corner AABB project by ~√3 and
	// stable under camera rotation — small-mesh queries don't flicker
	// across TestRect's hiZ thresholds.
	//
	// Sphere straddling the near plane bails as VISIBLE; TestRect can't
	// project a straddling rect safely.
	static ::MaskedOcclusionCulling::CullingResult testSphereVisible(
		const TES3::Vector3& center, float radius) {
		const ClipXYW c = projectWorld(center.x, center.y, center.z);

		const float wMin = c.w - (radius + g_depthSlackWorldUnitsEffective) * g_wGradMag;
		if (wMin <= kNearClipW) {
			g_queryNearClip.fetch_add(1, std::memory_order_relaxed);
			return ::MaskedOcclusionCulling::VISIBLE;
		}

		const float invW = 1.0f / c.w;
		const float cxNdc = c.x * invW;
		const float cyNdc = c.y * invW;
		const float rxNdc = radius * g_ndcRadiusX * invW;
		const float ryNdc = radius * g_ndcRadiusY * invW;

		float ndcMinX = std::max(cxNdc - rxNdc, -1.0f);
		float ndcMinY = std::max(cyNdc - ryNdc, -1.0f);
		float ndcMaxX = std::min(cxNdc + rxNdc, 1.0f);
		float ndcMaxY = std::min(cyNdc + ryNdc, 1.0f);
		if (ndcMinX >= ndcMaxX || ndcMinY >= ndcMaxY) {
			return ::MaskedOcclusionCulling::VIEW_CULLED;
		}

		return g_msoc->TestRect(ndcMinX, ndcMinY, ndcMaxX, ndcMaxY, wMin);
	}

	// ============================================================
	// Light culling
	// ============================================================

	// Dedup on register so DLL reload doesn't double-dispatch. Startup
	// only, not hot path.
	void registerLightObservedCallback(LightObservedCallback cb) {
		if (cb == nullptr) return;
		const auto it = std::find(g_lightObservers.begin(), g_lightObservers.end(), cb);
		if (it != g_lightObservers.end()) return;
		g_lightObservers.push_back(cb);
	}

	void unregisterLightObservedCallback(LightObservedCallback cb) {
		const auto it = std::find(g_lightObservers.begin(), g_lightObservers.end(), cb);
		if (it != g_lightObservers.end()) {
			g_lightObservers.erase(it);
		}
	}

	// Called from the naked hook at 0x6bb7d4 (replacing the original
	// `mov al, [ebx+0x90]` enabled-read in NiDX8LightManager::updateLights).
	// Returns the effective enabled byte: real flag, unless
	// OcclusionCullLights is on AND the worldBound sphere has been
	// OCCLUDED for at least OcclusionLightCullHysteresisFrames consecutive
	// frames, in which case 0 (disabled). Pre-empts D3D8 SetLight/
	// LightEnable without touching the scene graph.
	//
	// Hysteresis: VISIBLE resets and unlatches immediately (lights
	// reappear same-frame). OCCLUDED only latches after N consecutive,
	// so transient misfires don't flicker. Default N=3 (~50ms at 60Hz).
	// Counter is uint8_t — slider values above 255 effectively disable
	// the latch.
	extern "C" bool __cdecl shouldLightBeEnabled(NI::Light* light) {
		// Observer dispatch runs unconditionally — observers see every
		// iterated NiLight regardless of cache hit / MSOC state.
		for (const auto cb : g_lightObservers) {
			cb(light);
		}

		// The true enabled flag — always fetched, since a disabled
		// light stays disabled regardless of occlusion.
		const bool realEnabled = light->enabled;
		if (!realEnabled) {
			return false;
		}
		if (!g_cullLightsEffective) {
			return true;
		}
		if (!g_msoc) {
			// Hook installed but MSOC backend failed to init — bypass.
			return true;
		}

		// Captured on the AVObject base so worldBound comparisons are
		// offsetof-stable across NiLight subclasses (Point/Spot/
		// Directional/Ambient all share this layout).
		NI::AVObject* const av = light;
		const float cx = av->worldBoundOrigin.x;
		const float cy = av->worldBoundOrigin.y;
		const float cz = av->worldBoundOrigin.z;
		const float cr = av->worldBoundRadius;

		// Degenerate bound: zero-radius lights are ambients or fresh
		// instances whose worldBound hasn't propagated yet. Pass through.
		if (cr <= 0.0f) {
			return true;
		}

		auto it = g_lightCullCache.find(light);
		const bool haveValidEntry = (it != g_lightCullCache.end())
			&& it->second.boundOriginX == cx
			&& it->second.boundOriginY == cy
			&& it->second.boundOriginZ == cz
			&& it->second.boundRadius == cr;

		// Same-frame cache hit — return latched verdict without re-testing.
		if (haveValidEntry && it->second.lastQueryFrame == g_frameCounter) {
			++g_lightCullCacheHits;
			return !it->second.cullActive;
		}

		// Miss (or stale) — run MSOC test.
		++g_lightCullCacheMisses;
		++g_lightsTested;
		const TES3::Vector3 center{ cx, cy, cz };
		const auto verdict = testSphereVisible(center, cr);
		const bool occluded = (verdict == ::MaskedOcclusionCulling::OCCLUDED);
		if (occluded) ++g_lightsOccluded;

		if (haveValidEntry) {
			auto& e = it->second;
			e.lastQueryFrame = g_frameCounter;
			if (occluded) {
				if (e.consecOccluded < 0xFF) ++e.consecOccluded;
				if (e.consecOccluded >= g_lightCullHysteresisFramesEffective) {
					e.cullActive = true;
				}
			}
			else {
				e.consecOccluded = 0;
				e.cullActive = false;
			}
			return !e.cullActive;
		}

		// Fresh insert: not-culled-yet, so hysteresis ramps up before
		// first darkening even when the first verdict is OCCLUDED.
		LightCullEntry e;
		e.lightPtr = light;
		e.lastQueryFrame = g_frameCounter;
		e.consecOccluded = occluded ? 1 : 0;
		e.cullActive = false;
		e.boundOriginX = cx;
		e.boundOriginY = cy;
		e.boundOriginZ = cz;
		e.boundRadius = cr;
		g_lightCullCache[light] = e;
		return true;
	}

	// Naked trampoline replacing `mov al, [ebx+NiLight.super.enabled]`
	// at 0x6bb7d4. Entry: ebx = NiLight*. Exit: AL = effective enabled
	// byte. Upper EAX is don't-care (matches the mov-al semantics; the
	// followup `test al, al` only reads AL).
	__declspec(naked) void updateLights_enabledRead_hook() {
		__asm {
			// ecx/edx are nominally caller-save but the surrounding
			// loop doesn't preserve them across our site.
			push ecx
			push edx
			push ebx                 // arg: NI::Light*
			call shouldLightBeEnabled
			add esp, 4
			movzx eax, al            // zero-extend so upper bits are defined
			pop edx
			pop ecx
			retn
		}
	}

	// ============================================================
	// Occluder rasterisation
	// ============================================================

	// Rasterise a shape's real triangles (world-space) as an occluder.
	// Using real triangles instead of the AABB prevents "sign on wall
	// falsely occluded because it lives inside the wall's AABB."
	// Returns true on rasterise, false on skip (no data, thin axis,
	// camera inside tight AABB, or budget gate).
	static bool rasterizeTriShape(NI::TriBasedGeometry* shape, const TES3::Vector3& eye) {
		// Phase budget gate. Compares against g_rasterizeTimeUs (sum
		// of completed RenderTriangles SIMD time), NOT wall-clock-since-
		// frame-start. The latter would include cullShowBody traversal
		// between calls and falsely trip every frame.
		if (g_skipRasterizeThisFrame) {
			g_rasterizeBudgetTrips = 1;
			return false;
		}
		if (g_rasterizeBudgetUsEffective > 0
			&& g_rasterizeTimeUs > g_rasterizeBudgetUsEffective) {
			g_rasterizeBudgetTrips = 1;
			return false;
		}

		// Skinned meshes have bind-pose vertices that need
		// skinInstance->deform() to position correctly; rasterising raw
		// would draw a T-pose at world origin. Moving actors are poor
		// occluders anyway.
		if (shape->skinInstance) {
			return false;
		}

		auto data = shape->getModelData();
		if (!data) {
			return false;
		}
		// getActive*() returns the logically-valid subset (post-LOD).
		// Raw fields are allocation sizes — Intel's gather crashes on
		// stale tail entries.
		const unsigned short vertexCount = data->getActiveVertexCount();
		if (vertexCount == 0 || data->vertex == nullptr) {
			return false;
		}
		const unsigned short triCount = data->getActiveTriangleCount();
		const NI::Triangle* tris = data->getTriList();
		if (triCount == 0 || tris == nullptr) {
			return false;
		}
		// Dense meshes pay rasterisation cost ∝ triCount but rarely
		// occlude proportionally better. Still tested as occludees.
		if (triCount > g_occluderMaxTrianglesEffective) {
			++g_skippedTriCount;
			return false;
		}

		const auto& xf = shape->worldTransform;
		const auto& R = xf.rotation;
		const auto& T = xf.translation;
		const float s = xf.scale;

		// Transform every vertex into the reusable world-space buffer
		// once and accumulate the tight AABB for the inside/thin-axis
		// gates. Not cached across frames — TriShapeData pointers
		// can be freed and reused.
		g_occluderVerts.resize(static_cast<size_t>(vertexCount) * 3);
		float* out = g_occluderVerts.data();
		float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
		float maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;
		for (unsigned short i = 0; i < vertexCount; ++i) {
			const auto& v = data->vertex[i];
			const float rx = R.m0.x * v.x + R.m0.y * v.y + R.m0.z * v.z;
			const float ry = R.m1.x * v.x + R.m1.y * v.y + R.m1.z * v.z;
			const float rz = R.m2.x * v.x + R.m2.y * v.y + R.m2.z * v.z;
			const float wx = rx * s + T.x;
			const float wy = ry * s + T.y;
			const float wz = rz * s + T.z;
			out[i * 3 + 0] = wx;
			out[i * 3 + 1] = wy;
			out[i * 3 + 2] = wz;
			if (wx < minX) minX = wx;
			if (wx > maxX) maxX = wx;
			if (wy < minY) minY = wy;
			if (wy > maxY) maxY = wy;
			if (wz < minZ) minZ = wz;
			if (wz > maxZ) maxZ = wz;
		}

		// Reject pencil shapes — tiny silhouette area, near-zero
		// occlusion. Walls/floors (single thin axis) still qualify.
		const float minDim = g_occluderMinDimensionEffective;
		const float dx = maxX - minX;
		const float dy = maxY - minY;
		const float dz = maxZ - minZ;
		const int thinAxes = (dx < minDim ? 1 : 0)
			+ (dy < minDim ? 1 : 0)
			+ (dz < minDim ? 1 : 0);
		if (thinAxes >= 2) {
			++g_skippedThin;
			return false;
		}

		// Inside-guard: a camera inside the mesh would cover the screen
		// with near-face depths and falsely occlude everything behind
		// the far face. Tight AABB + margin suffices since the real
		// triangles are strictly inside it.
		const float m = g_insideOccluderMarginEffective;
		if (eye.x >= minX - m && eye.x <= maxX + m &&
			eye.y >= minY - m && eye.y <= maxY + m &&
			eye.z >= minZ - m && eye.z <= maxZ + m) {
			++g_skippedInside;
			return false;
		}

		// Expand 16-bit indices into MSOC's 32-bit list and drop any
		// out-of-bounds entries (Intel's gather would crash).
		g_occluderIndices.resize(static_cast<size_t>(triCount) * 3);
		unsigned int* idx = g_occluderIndices.data();
		unsigned int outTri = 0;
		for (unsigned short i = 0; i < triCount; ++i) {
			const unsigned short a = tris[i].vertices[0];
			const unsigned short b = tris[i].vertices[1];
			const unsigned short c = tris[i].vertices[2];
			if (a >= vertexCount || b >= vertexCount || c >= vertexCount) {
				continue;
			}
			idx[outTri * 3 + 0] = a;
			idx[outTri * 3 + 1] = b;
			idx[outTri * 3 + 2] = c;
			++outTri;
		}
		if (outTri == 0) {
			return false;
		}

		// VertexLayout(12, 4, 8): stride 12, y@4, z@8 — packed float[3].
		// BACKFACE_NONE: NIF winding isn't guaranteed consistent.
		if (g_asyncThisFrame) {
			// Move owning buffers into the per-frame arena so workers
			// have stable pointers until Flush() retires the job.
			// Inner vectors keep their data pointer through outer-vector
			// relocations (move ctor steals).
			g_asyncOccluderVerts.emplace_back(std::move(g_occluderVerts));
			g_asyncOccluderIndices.emplace_back(std::move(g_occluderIndices));
			const auto& vtx = g_asyncOccluderVerts.back();
			const auto& tri = g_asyncOccluderIndices.back();
			ScopedUsAccumulator t(g_rasterizeTimeUs);
			g_threadpool->RenderTriangles(vtx.data(), tri.data(),
				static_cast<int>(outTri),
				::MaskedOcclusionCulling::BACKFACE_NONE,
				::MaskedOcclusionCulling::CLIP_PLANE_ALL);
		}
		else {
			ScopedUsAccumulator t(g_rasterizeTimeUs);
			g_msoc->RenderTriangles(g_occluderVerts.data(), idx,
				static_cast<int>(outTri), g_worldToClip,
				::MaskedOcclusionCulling::BACKFACE_NONE,
				::MaskedOcclusionCulling::CLIP_PLANE_ALL,
				::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
		}
		g_occluderTriangles += outTri;
		return true;
	}

	// ============================================================
	// Terrain aggregation
	// ============================================================

	// Frustum check for the aggregate-terrain walk. No mask bookkeeping —
	// runs before cullShowBody so usedCullingPlanesBitfield stays clean.
	static bool frustumCulledSphere(NI::AVObject* obj, NI::Camera* camera) {
		const int n = cameraCountCullingPlanes(camera);
		const float r = obj->worldBoundRadius;
		for (int i = 0; i < n; ++i) {
			const auto* plane = cameraCullingPlane(camera, i);
			const float d = plane->x * obj->worldBoundOrigin.x
				+ plane->y * obj->worldBoundOrigin.y
				+ plane->z * obj->worldBoundOrigin.z
				- plane->w;
			if (d <= -r) return true;
		}
		return false;
	}

	// Append one terrain TriShape (25v/32t) to the aggregate buffers,
	// transforming to world space and offsetting indices. Mirrors
	// rasterizeTriShape's vertex math without the gates/skinning/thin-
	// axis paths.
	static void appendTerrainShape(std::vector<float>& aggVerts,
		std::vector<unsigned int>& aggIdx, NI::TriShape* shape,
		unsigned int step) {
		auto data = shape->getModelData();
		if (!data) return;
		const unsigned short vcount = data->getActiveVertexCount();
		if (vcount == 0 || data->vertex == nullptr) return;
		const unsigned short tcount = data->getActiveTriangleCount();
		const NI::Triangle* tris = data->getTriList();
		if (tcount == 0 || tris == nullptr) return;

		const auto& xf = shape->worldTransform;
		const auto& R = xf.rotation;
		const auto& T = xf.translation;
		const float s = xf.scale;

		// Downsample fast path. Terrain subcells are 5×5 row-major grids
		// (25v/32t). step=2 → 3×3 (8 tris); step=4 → 2×2 (2 tris). Sample
		// every step-th vertex; min-z over the dropped neighbours so the
		// silhouette can only shrink (conservative under-occlude). Other
		// vcount/step combos fall through to full-resolution. Only 2/4
		// divide the 4-quad edge cleanly without dropping the seam.
		if (vcount == 25 && (step == 2 || step == 4)) {
			// Coarse-grid axis count: step=2 -> 3 verts/edge, step=4 -> 2.
			const unsigned int n = (4u / step) + 1u;
			const unsigned int baseVert = static_cast<unsigned int>(aggVerts.size() / 3);
			aggVerts.resize(aggVerts.size() + static_cast<size_t>(n * n) * 3);
			float* out = aggVerts.data() + baseVert * 3;

			// For each kept (cr, cc), take min-world-z over the source-
			// grid neighbourhood. Right/bottom seam verts are only
			// covered by the last-row/column kept vertex — the seam
			// already matches the next subcell exactly.
			for (unsigned int cr = 0; cr < n; ++cr) {
				for (unsigned int cc = 0; cc < n; ++cc) {
					const unsigned int sr0 = cr * step;
					const unsigned int sc0 = cc * step;
					const unsigned int sr1 = (cr + 1 == n) ? sr0 : sr0 + (step - 1);
					const unsigned int sc1 = (cc + 1 == n) ? sc0 : sc0 + (step - 1);

					float minWz = std::numeric_limits<float>::infinity();
					float keepX = 0, keepY = 0;
					// XY anchored at (sr0, sc0); only Z is min-folded.
					for (unsigned int sr = sr0; sr <= sr1; ++sr) {
						for (unsigned int sc = sc0; sc <= sc1; ++sc) {
							const auto& v = data->vertex[sr * 5 + sc];
							const float rx = R.m0.x * v.x + R.m0.y * v.y + R.m0.z * v.z;
							const float ry = R.m1.x * v.x + R.m1.y * v.y + R.m1.z * v.z;
							const float rz = R.m2.x * v.x + R.m2.y * v.y + R.m2.z * v.z;
							const float wx = rx * s + T.x;
							const float wy = ry * s + T.y;
							const float wz = rz * s + T.z;
							if (sr == sr0 && sc == sc0) { keepX = wx; keepY = wy; }
							if (wz < minWz) minWz = wz;
						}
					}
					float* dst = out + (cr * n + cc) * 3;
					dst[0] = keepX;
					dst[1] = keepY;
					dst[2] = minWz;
				}
			}

			// (n-1)*(n-1)*2 triangles. BACKFACE_NONE so winding is moot.
			const unsigned int qN = n - 1;
			aggIdx.reserve(aggIdx.size() + static_cast<size_t>(qN * qN) * 6);
			for (unsigned int qr = 0; qr < qN; ++qr) {
				for (unsigned int qc = 0; qc < qN; ++qc) {
					const unsigned int v00 = baseVert + (qr * n + qc);
					const unsigned int v01 = baseVert + (qr * n + (qc + 1));
					const unsigned int v10 = baseVert + ((qr + 1) * n + qc);
					const unsigned int v11 = baseVert + ((qr + 1) * n + (qc + 1));
					aggIdx.push_back(v00); aggIdx.push_back(v10); aggIdx.push_back(v01);
					aggIdx.push_back(v10); aggIdx.push_back(v11); aggIdx.push_back(v01);
				}
			}
			return;
		}

		// Full-resolution path: copy every source vert + every source tri.
		const unsigned int baseVert = static_cast<unsigned int>(aggVerts.size() / 3);
		aggVerts.resize(aggVerts.size() + static_cast<size_t>(vcount) * 3);
		float* out = aggVerts.data() + baseVert * 3;
		for (unsigned short i = 0; i < vcount; ++i) {
			const auto& v = data->vertex[i];
			const float rx = R.m0.x * v.x + R.m0.y * v.y + R.m0.z * v.z;
			const float ry = R.m1.x * v.x + R.m1.y * v.y + R.m1.z * v.z;
			const float rz = R.m2.x * v.x + R.m2.y * v.y + R.m2.z * v.z;
			out[i * 3 + 0] = rx * s + T.x;
			out[i * 3 + 1] = ry * s + T.y;
			out[i * 3 + 2] = rz * s + T.z;
		}

		aggIdx.reserve(aggIdx.size() + static_cast<size_t>(tcount) * 3);
		for (unsigned short i = 0; i < tcount; ++i) {
			const unsigned short a = tris[i].vertices[0];
			const unsigned short b = tris[i].vertices[1];
			const unsigned short c = tris[i].vertices[2];
			if (a >= vcount || b >= vcount || c >= vcount) continue;
			aggIdx.push_back(baseVert + a);
			aggIdx.push_back(baseVert + b);
			aggIdx.push_back(baseVert + c);
		}
	}

	// 0=Full (5x5, 32 tris), 1=Half (3x3, 8 tris), 2=Corners (2x2, 2 tris).
	// Out-of-range clamps to Full so malformed JSON doesn't disable terrain.
	static unsigned int currentTerrainStep() {
		switch (g_terrainResolutionEffective) {
		case 1: return 2;
		case 2: return 4;
		default: return 1;
		}
	}

	// Build VB+IB for one per-Land NiNode (cache-miss path). Walks every
	// subcell unconditionally — result must be valid for any camera angle.
	// MSOC clips internally; extra off-frustum triangles cost negligibly
	// vs the per-frame walk + transform they replace.
	static void buildLandCacheEntry(LandCacheEntry& entry, NI::Node* landNode) {
		entry.verts.clear();
		entry.indices.clear();
		entry.subcellRanges.clear();
		const unsigned int step = currentTerrainStep();
		const auto& subcells = landNode->children;
		for (size_t j = 0; j < subcells.endIndex; ++j) {
			auto* sub = subcells.storage[j].get();
			if (!sub) continue;
			if (!sub->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;
			auto* subNode = static_cast<NI::Node*>(sub);

			// Bracket this subcell's index-buffer range so the submit
			// path can frustum-cull at subcell granularity.
			const unsigned int firstIdxBefore = static_cast<unsigned int>(entry.indices.size());

			const auto& shapes = subNode->children;
			for (size_t k = 0; k < shapes.endIndex; ++k) {
				auto* shape = shapes.storage[k].get();
				if (!shape) continue;
				if (!shape->isInstanceOfType(NI::RTTIStaticPtr::NiTriShape)) continue;
				const auto p = classifyOccluderProperties(shape);
				if (p.alpha || p.stencil) continue;
				appendTerrainShape(entry.verts, entry.indices, static_cast<NI::TriShape*>(shape), step);
			}

			const unsigned int firstIdxAfter = static_cast<unsigned int>(entry.indices.size());
			if (firstIdxAfter > firstIdxBefore) {
				LandCacheEntry::SubcellRange r;
				r.node     = subNode;
				r.firstIdx = firstIdxBefore;
				r.triCount = (firstIdxAfter - firstIdxBefore) / 3;
				entry.subcellRanges.push_back(r);
			}
		}
		entry.triCount = static_cast<unsigned int>(entry.indices.size() / 3);
		entry.builtForResolution = static_cast<uint8_t>(g_terrainResolutionEffective);
	}

	// Horizon-mode terrain occluder. Walks WorldLandscape, projects each
	// vertex to clip space, bins into a 1D max-y/max-w horizon, simplifies
	// to ~60 adaptive samples, emits ~120 curtain triangles, submits those
	// to MOC via sync RenderTriangles. No land cache (fresh projection
	// each frame). Frustum-cull at land granularity only. epsD is adaptive
	// (do NOT pass 1e30 like MGE-XE's distant path — see HorizonOccluder.h).
	static void rasterizeAggregateTerrainHorizon(NI::Camera* camera) {
		if (!g_worldLandscapeRoot) return;
		if (g_worldLandscapeRoot->getAppCulled()) return;

		// Outer bucket: build = project + simplify + emit + submit.
		// The inner RenderTriangles also accumulates into g_horizonRasterUs
		// and g_rasterizeTimeUs.
		ScopedUsAccumulator timer(g_horizonBuildUs);

		auto& horizon = msoc::horizon::HorizonOccluder::getInstance();
		// 512 cols / 60 samples matches MGE-XE's distant-land path.
		horizon.init(512, 60);
		horizon.reset();

		const int   resolution = horizon.resolution();
		const float halfSpan   = 0.5f * static_cast<float>(resolution - 1);

		uint64_t landsContributed = 0;
		uint64_t vertsProjected   = 0;

		const auto& landChildren = g_worldLandscapeRoot->children;
		for (size_t i = 0; i < landChildren.endIndex; ++i) {
			auto* land = landChildren.storage[i].get();
			if (!land) continue;
			if (!land->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;
			if (land->getAppCulled()) continue;
			if (frustumCulledSphere(land, camera)) continue;

			auto* landNode = static_cast<NI::Node*>(land);
			bool landFedAny = false;

			const auto& subcells = landNode->children;
			for (size_t j = 0; j < subcells.endIndex; ++j) {
				auto* sub = subcells.storage[j].get();
				if (!sub) continue;
				if (!sub->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;
				auto* subNode = static_cast<NI::Node*>(sub);

				// Per-subcell frustum cull. A Land covers 8192×8192;
				// typically ~5–8 of its 16 subcells are in-frustum.
				// Skipping invisible ones avoids the per-vertex
				// transform — the dominant cost. Quality is unchanged
				// (the c.w / ndcX guards below would've rejected these
				// post-projection anyway).
				if (frustumCulledSphere(subNode, camera)) continue;

				const auto& shapes = subNode->children;
				for (size_t k = 0; k < shapes.endIndex; ++k) {
					auto* shape = shapes.storage[k].get();
					if (!shape) continue;
					if (!shape->isInstanceOfType(NI::RTTIStaticPtr::NiTriShape)) continue;

					// Skip alpha/stencil shapes — same gate as the raster
					// path. A transparent terrain patch shouldn't write
					// occlusion regardless of mode.
					const auto props = classifyOccluderProperties(shape);
					if (props.alpha || props.stencil) continue;

					auto* tri = static_cast<NI::TriShape*>(shape);
					// getModelData returns a smart Pointer<TriShapeData>,
					// not a raw pointer — use auto, not auto*. Matches
					// appendTerrainShape's pattern at line ~1614.
					auto data = tri->getModelData();
					if (!data) continue;
					const unsigned short vcount = data->getActiveVertexCount();
					if (vcount == 0 || data->vertex == nullptr) continue;

					const auto& xf = tri->worldTransform;
					const auto& R  = xf.rotation;
					const auto& T  = xf.translation;
					const float s  = xf.scale;

					// Mirrors appendTerrainShape's transform.
					for (unsigned short v = 0; v < vcount; ++v) {
						const auto& vp = data->vertex[v];
						const float rx = R.m0.x * vp.x + R.m0.y * vp.y + R.m0.z * vp.z;
						const float ry = R.m1.x * vp.x + R.m1.y * vp.y + R.m1.z * vp.z;
						const float rz = R.m2.x * vp.x + R.m2.y * vp.y + R.m2.z * vp.z;
						const float wx = rx * s + T.x;
						const float wy = ry * s + T.y;
						const float wz = rz * s + T.z;

						const ClipXYW c = projectWorld(wx, wy, wz);
						// Near-plane straddling — projection unstable.
						if (c.w <= kNearClipW) continue;

						const float invW = 1.0f / c.w;
						const float ndcX = c.x * invW;
						const float ndcY = c.y * invW;

						// Off-screen X clamps would contaminate column-
						// boundary sentinels; drop. Below-screen doesn't
						// occlude anything; drop. Above-screen clamps up
						// (terrain past the top still defines silhouette).
						if (ndcX < -1.0f || ndcX > 1.0f) continue;
						if (ndcY < -1.0f) continue;
						const float clampedY = (ndcY > 1.0f) ? 1.0f : ndcY;

						int col = static_cast<int>((ndcX + 1.0f) * halfSpan);
						if (col < 0) col = 0;
						if (col >= resolution) col = resolution - 1;

						horizon.update(col, clampedY, c.w);
						++vertsProjected;
						landFedAny = true;
					}
				}
			}
			if (landFedAny) ++landsContributed;
		}

		if (vertsProjected == 0) return;

		// Simplify → margin → emit → fixup → submit.
		constexpr int   kMaxSamples    = 60;
		constexpr float kEpsH          = 0.01f;
		constexpr int   kTileAlign     = 16;
		constexpr float kNdcYBottom    = -1.1f;
		constexpr float kYSafetyMargin = 0.04f;
		constexpr float kEpsDFraction  = 0.05f;
		constexpr float kEpsDFloor     = 100.0f;

		// D12: adaptive epsD computed from this frame's depth range.
		// computeAdaptiveEpsD returns +infinity if fewer than 2 cols are
		// active — equivalent to disabling the depth term, safe fallback.
		const float adaptiveEpsD = horizon.computeAdaptiveEpsD(kEpsDFraction, kEpsDFloor);

		static msoc::horizon::Sample samples[kMaxSamples];
		const int nSamples = horizon.simplify(samples, kMaxSamples, kEpsH, adaptiveEpsD, kTileAlign);
		if (nSamples < 2) return;

		// Pull the curtain top down by the safety margin. Skip sentinel
		// rows (those columns will be skipped by emit anyway).
		for (int i = 0; i < nSamples; ++i) {
			if (samples[i].h > -1.0e29f) samples[i].h -= kYSafetyMargin;
		}

		// Per-frame scratch — function-static so allocations amortise.
		// Sized for the worst case (every segment emits 6 verts).
		static std::vector<msoc::horizon::CurtainVertex> curtainVerts;
		curtainVerts.resize(static_cast<size_t>(6 * (nSamples - 1)));
		const int triCount = horizon.emitCurtainNDC(samples, nSamples, kNdcYBottom,
			curtainVerts.data(), static_cast<int>(curtainVerts.size()));
		if (triCount <= 0) return;

		// Convert NDC layout (x, y, z=depth, w=1) → MOC pre-transformed
		// layout (x*d, y*d, _, d) in place. Submit with VertexLayout
		// matching that layout: stride=16, offY=4, offW=12.
		msoc::horizon::HorizonOccluder::fixupForMOC(curtainVerts.data(), triCount * 3);

		static std::vector<unsigned int> curtainIdx;
		const int vtxCount = triCount * 3;
		curtainIdx.resize(static_cast<size_t>(vtxCount));
		for (int i = 0; i < vtxCount; ++i) {
			curtainIdx[i] = static_cast<unsigned int>(i);
		}

		// Sync submit; bypasses the threadpool — ~120 tris is below the
		// dispatch-overhead break-even. Times into both buckets so total
		// rasterize cost stays comparable across modes AND the curtain's
		// own raster cost stays measurable.
		{
			ScopedUsAccumulator t1(g_rasterizeTimeUs);
			ScopedUsAccumulator t2(g_horizonRasterUs);
			g_msoc->RenderTriangles(
				reinterpret_cast<const float*>(curtainVerts.data()),
				curtainIdx.data(), triCount,
				/*modelToClip=*/nullptr,
				::MaskedOcclusionCulling::BACKFACE_NONE,
				::MaskedOcclusionCulling::CLIP_PLANE_ALL,
				::MaskedOcclusionCulling::VertexLayout(16, 4, 12));
		}

		// columnsTouched is a linear scan but runs once per frame.
		g_horizonCurtainTris    = static_cast<uint64_t>(triCount);
		g_horizonLandsFed       = landsContributed;
		g_horizonVertsFed       = vertsProjected;
		g_horizonColumnsTouched = static_cast<uint64_t>(horizon.columnsTouched());
		g_horizonAdaptiveEpsD   = adaptiveEpsD;
	}

	// Aggregate terrain rasteriser. Walks WorldLandscape (root → Land →
	// 16 subcells → N NiTriShapes) and submits one combined occluder per
	// visible Land. Individual 25v/32t patches fail the thin-axis gate;
	// merging gives the hill/horizon silhouette that actually occludes
	// distant architecture.
	//
	// Must run inside isTopLevel — after ClearBuffer + uploadCameraTransform,
	// before cullShowBody — so the drain sees terrain in the depth buffer.
	//
	// Uses g_landCache to amortise the per-Land walk + vertex transform.
	// Mark-and-sweep evicts entries whose NiNode wasn't seen this frame.
	static void rasterizeAggregateTerrain(NI::Camera* camera) {
		if (!g_worldLandscapeRoot) return;
		if (g_worldLandscapeRoot->getAppCulled()) return;

		ScopedUsAccumulator timer(g_aggregateTerrainUs);

		// Mark all cached entries unseen; we'll flip this for entries we
		// touch this frame and evict the remainder below.
		for (auto& kv : g_landCache) kv.second.seen = false;

		const auto& landChildren = g_worldLandscapeRoot->children;
		for (size_t i = 0; i < landChildren.endIndex; ++i) {
			auto* land = landChildren.storage[i].get();
			if (!land) continue;
			if (!land->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;

			auto* landNode = static_cast<NI::Node*>(land);

			// Get-or-build the cached buffer. Keyed by NiNode pointer;
			// a cell-change realloc produces a new key and rebuilds.
			auto [it, inserted] = g_landCache.try_emplace(landNode);
			LandCacheEntry& entry = it->second;
			// Resolution-mismatch on hit forces a rebuild — MCM dropdown
			// changes propagate lazily. nodePtr stays valid (NI::Pointer
			// pins the NiNode); only verts/indices redo.
			const uint8_t curRes = static_cast<uint8_t>(g_terrainResolutionEffective);
			if (inserted) {
				entry.nodePtr = landNode;
				buildLandCacheEntry(entry, landNode);
				++g_landCacheMisses;
			}
			else if (entry.builtForResolution != curRes) {
				buildLandCacheEntry(entry, landNode);
				++g_landCacheMisses;
			}
			else {
				++g_landCacheHits;
			}
			entry.seen = true;

			// Per-frame submit gates. Shape-level filters are intentionally
			// omitted from cache build — they'd bake view-specific state
			// into a reusable buffer.
			if (land->getAppCulled()) continue;
			if (frustumCulledSphere(land, camera)) continue;
			if (entry.triCount == 0) continue;

			// Per-subcell frustum cull. ~5–8 of a Land's 16 subcells are
			// typically in-frustum; skipping the rest trades one big
			// submission for N small ones with fewer total triangles —
			// net win on per-tile cost and async queue pressure.
			//
			// Empty subcellRanges fallback (stale cache from before the
			// field landed) submits the whole entry as one range.
			unsigned int submittedTris = 0;
			if (!entry.subcellRanges.empty()) {
				for (const auto& range : entry.subcellRanges) {
					if (range.triCount == 0) continue;
					if (range.node && frustumCulledSphere(range.node, camera)) continue;

					if (g_asyncThisFrame) {
						ScopedUsAccumulator t(g_rasterizeTimeUs);
						g_threadpool->RenderTriangles(
							entry.verts.data(),
							entry.indices.data() + range.firstIdx,
							static_cast<int>(range.triCount),
							::MaskedOcclusionCulling::BACKFACE_NONE,
							::MaskedOcclusionCulling::CLIP_PLANE_ALL);
					} else {
						ScopedUsAccumulator t(g_rasterizeTimeUs);
						g_msoc->RenderTriangles(
							entry.verts.data(),
							entry.indices.data() + range.firstIdx,
							static_cast<int>(range.triCount),
							g_worldToClip,
							::MaskedOcclusionCulling::BACKFACE_NONE,
							::MaskedOcclusionCulling::CLIP_PLANE_ALL,
							::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
					}
					submittedTris += range.triCount;
				}
			} else {
				// Fallback path — single submission for the whole Land.
				if (g_asyncThisFrame) {
					ScopedUsAccumulator t(g_rasterizeTimeUs);
					g_threadpool->RenderTriangles(entry.verts.data(), entry.indices.data(),
						static_cast<int>(entry.triCount),
						::MaskedOcclusionCulling::BACKFACE_NONE,
						::MaskedOcclusionCulling::CLIP_PLANE_ALL);
				} else {
					ScopedUsAccumulator t(g_rasterizeTimeUs);
					g_msoc->RenderTriangles(entry.verts.data(), entry.indices.data(),
						static_cast<int>(entry.triCount), g_worldToClip,
						::MaskedOcclusionCulling::BACKFACE_NONE,
						::MaskedOcclusionCulling::CLIP_PLANE_ALL,
						::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
				}
				submittedTris = entry.triCount;
			}

			if (submittedTris > 0) {
				++g_aggregateTerrainLands;
				g_aggregateTerrainTris += submittedTris;
			}
		}

		// Sweep stale entries. Safe to free now because ClearBuffer() at
		// top-level entry already flushed previous-frame async work that
		// referenced these pointers.
		for (auto it = g_landCache.begin(); it != g_landCache.end(); ) {
			if (!it->second.seen) {
				it = g_landCache.erase(it);
				++g_landCacheEvictions;
			}
			else {
				++it;
			}
		}
	}

	// ============================================================
	// Main scene-graph traversal
	// ============================================================

	// Reimplements CullShow at 0x6EB480 with MSOC query + occluder
	// rasterisation wedged between the frustum test and Display. The
	// detour JMPs here. Behaviour matches the engine 1:1 when
	// g_msocActive is false.
	static void __fastcall cullShowBody(NI::AVObject* self, void* /*edx*/, NI::Camera* camera) {
		if (g_msocActive) ++g_recursiveCalls;
		if (self->getAppCulled()) {
			if (g_msocActive) ++g_recursiveAppCulled;
			return;
		}

		// Hierarchical frustum test, mirroring the engine's loop at
		// 0x6EB4B7. setBits tracks bits flipped in
		// usedCullingPlanesBitfield so they're unflipped before return
		// (engine's LABEL_10 cleanup).
		uint32_t setBits[4] = { 0, 0, 0, 0 };
		const int nPlanes = cameraCountCullingPlanes(camera);
		auto* mask = cameraUsedPlanesMask(camera);

		auto restoreIgnoreBits = [&]() {
			for (int j = 0; j < nPlanes; ++j) {
				const uint32_t jbit = 1u << (j & 0x1F);
				if (jbit & setBits[j >> 5]) {
					mask[j >> 5] &= ~jbit;
				}
			}
		};

		const float boundRadius = self->worldBoundRadius;
		for (int i = nPlanes - 1; i >= 0; --i) {
			const uint32_t bit = 1u << (i & 0x1F);
			const int word = i >> 5;
			if ((bit & mask[word]) != 0) {
				continue;
			}
			const auto* plane = cameraCullingPlane(camera, i);
			const float d = plane->x * self->worldBoundOrigin.x
				+ plane->y * self->worldBoundOrigin.y
				+ plane->z * self->worldBoundOrigin.z
				- plane->w;
			if (d <= -boundRadius) {
				if (g_msocActive) ++g_recursiveFrustumCulled;
				restoreIgnoreBits();
				return;
			}
			if (d >= boundRadius) {
				mask[word] |= bit;
				setBits[word] |= bit;
			}
		}

		if (g_msocActive) {
			// Two-pass: every NiTriBasedGeom leaf defers its visibility
			// test until after the main traversal finishes populating the
			// depth buffer. Eliminates the false positive where a leaf is
			// tested against an occluder its parent/sibling rasterised
			// moments before (door behind its own wall, leaf in front of
			// its own branch).
			//
			// Large NiTriShapes rasterise inline as occluders WITHOUT an
			// own-visibility test — MSOC's "closest 1/w wins" means a
			// shape that's itself occluded just overdraws tiles with
			// farther depths (harmless). Removing the own-test breaks the
			// cycle where a shape fails against its own sibling's raster
			// and still produces usable occluder depth.
			//
			// NiNodes are NOT MSOC-tested — only frustum-culled. Testing
			// them inline against a partial buffer was a major false-
			// positive source (a ref's root NiNode could fail against an
			// earlier sibling and suppress the whole subtree).
			const bool isGeom = self->isInstanceOfType(NI::RTTIStaticPtr::NiTriBasedGeom);
			if (isGeom) {
				bool didRasterise = false;
				// Only opaque shapes occlude. Alpha/blended geometry
				// (fences, banners, leaves) would falsely occlude things
				// behind the transparent parts.
				//
				// When aggregate-terrain is on, terrain descendants are
				// already in the buffer as merged per-Land submissions —
				// re-rasterising would be dup work.
				const bool skipAsAggregated = (g_aggregateTerrainEffective != 0)
					&& isLandscapeDescendant(self, g_worldLandscapeRoot);
				if (!skipAsAggregated
					&& boundRadius >= g_occluderRadiusMinEffective
					&& boundRadius <= g_occluderRadiusMaxEffective) {
					const auto p = classifyOccluderProperties(self);
					if (p.alpha) {
						++g_skippedAlpha;
					}
					else if (p.stencil) {
						++g_skippedStencil;
					}
					else {
						const auto& eye = camera->worldTransform.translation;
						if (rasterizeTriShape(static_cast<NI::TriBasedGeometry*>(self), eye)) {
							++g_rasterizedAsOccluder;
							didRasterise = true;
							if (g_tintOccluderEffective) {
								tintEmissive(self, kTintOccluder);
							}
						}
					}
				}
				g_pendingDisplays.push_back({ self, camera, didRasterise });
				++g_deferred;
				restoreIgnoreBits();
				return;
			}
		}

		self->vTable.asAVObject->display(self, camera);

		restoreIgnoreBits();
	}

	// ============================================================
	// Deferred-display drain pipeline
	// ============================================================

	// Drain phase 1 — classify [lo, hi) of g_pendingDisplays into
	// g_drainSlots. Read-only on shared state (no g_drainCache writes,
	// no counter increments except the atomic g_queryNearClip, no
	// display(), no tints) — phase 2 owns all writes. Read-only-ness
	// is what lets this run on workers once parallel dispatch lands.
	static void classifyDrainRange(size_t lo, size_t hi) {
		// Captured here, not at drainPendingDisplays entry, so the
		// spike-clip measures classify-only elapsed (excluding phase 2's
		// vanilla D3D8 display() calls).
		g_classifyPhaseStart = std::chrono::steady_clock::now();

		// Predictive skip: EMA over 2× budget → skip TestRect entirely
		// this frame, mark all Visible. Equivalent to EnableMSOC=false
		// for one frame, except mask build still happens so recovery
		// momentum is preserved once the EMA drops back.
		if (g_skipClassifyThisFrame) {
			g_classifyBudgetTrips = 1;
			for (size_t i = lo; i < hi; ++i) {
				g_drainSlots[i].verdict     = DrainVerdict::Visible;
				g_drainSlots[i].ranTestRect = false;
			}
			return;
		}

		const unsigned int tcFrames = g_temporalCoherenceFramesEffective;
		const bool skipTerrainEnabled = g_skipTerrainOccludeesEffective;
		const float tinyThreshold = g_occludeeMinRadiusEffective;

		// Spike-clip armed when budget > 0. Timer is sampled every 32nd
		// iteration; per-iter overhead is ~one branch. On trip, remaining
		// slots become Visible (safe-fallback MOC invariant).
		const bool budgetActive = (g_classifyBudgetUsEffective > 0);
		constexpr size_t kCheckMask = 31;

		for (size_t i = lo; i < hi; ++i) {
			const auto& p = g_pendingDisplays[i];
			auto& slot = g_drainSlots[i];
			slot.ranTestRect = false;

			if (budgetActive && i > lo && (i & kCheckMask) == 0) {
				if (elapsedUsSince(g_classifyPhaseStart) > g_classifyBudgetUsEffective) {
					g_classifyBudgetTrips = 1;
					for (size_t j = i; j < hi; ++j) {
						g_drainSlots[j].verdict   = DrainVerdict::Visible;
						g_drainSlots[j].ranTestRect = false;
					}
					return;
				}
			}

			if (skipTerrainEnabled
				&& isLandscapeDescendant(p.shape, g_worldLandscapeRoot)) {
				slot.verdict = DrainVerdict::SkipTerrain;
				continue;
			}

			if (p.shape->worldBoundRadius < tinyThreshold) {
				slot.verdict = DrainVerdict::SkipTiny;
				continue;
			}

			// Temporal cache: read-only lookup. Inserts happen in phase 2.
			if (tcFrames > 0) {
				auto it = g_drainCache.find(p.shape);
				if (it != g_drainCache.end()) {
					const auto& e = it->second;
					const auto& o = p.shape->worldBoundOrigin;
					const float r = p.shape->worldBoundRadius;
					const bool stationary =
						e.boundOriginX == o.x &&
						e.boundOriginY == o.y &&
						e.boundOriginZ == o.z &&
						e.boundRadius == r;
					const bool fresh =
						(g_frameCounter - e.lastQueryFrame) <= tcFrames;
					if (stationary && fresh) {
						slot.verdict = DrainVerdict::CachedOccluded;
						continue;
					}
				}
			}

			// Phase-1 wall time is bracketed once on the main thread; no
			// per-call timing here (worker CPU time ≠ wall time).
			::MaskedOcclusionCulling::CullingResult r;
			r = testSphereVisible(
				p.shape->worldBoundOrigin, p.shape->worldBoundRadius);
			slot.ranTestRect = true;
			switch (r) {
			case ::MaskedOcclusionCulling::VISIBLE:
				slot.verdict = DrainVerdict::Visible; break;
			case ::MaskedOcclusionCulling::OCCLUDED:
				slot.verdict = DrainVerdict::Occluded; break;
			case ::MaskedOcclusionCulling::VIEW_CULLED:
				slot.verdict = DrainVerdict::ViewCulled; break;
			default:
				slot.verdict = DrainVerdict::Visible;  // conservative fallback
				break;
			}
		}
	}

	// Parallel-drain worker body — DISABLED. See the worker-pool block
	// higher in this file for the re-enable checklist.
	//
	// static void drainWorkerMain(std::stop_token stop, unsigned int workerId) {
	// 	assert(workerId < kDrainWorkerCount);
	//
	// 	while (!stop.stop_requested()) {
	// 		{
	// 			std::unique_lock<std::mutex> lock(g_drainMtx);
	// 			g_drainStartCv.wait(lock, stop, [] {
	// 				return g_drainStartTickets > 0;
	// 				});
	// 			if (stop.stop_requested()) break;
	// 			--g_drainStartTickets;
	// 		}
	//
	// 		const auto range = g_drainRanges[workerId];
	// 		classifyDrainRange(range.lo, range.hi);
	//
	// 		{
	// 			std::lock_guard<std::mutex> lock(g_drainMtx);
	// 			++g_drainDoneTickets;
	// 		}
	// 		g_drainDoneCv.notify_one();
	// 	}
	// }

	// Drain the deferred-display queue. Re-tests each entry against the
	// now-complete depth buffer and displays the visible ones. Must run
	// before g_msocActive clears so counters land in this frame's log.
	//
	// Two-phase: classifyDrainRange does read-only verdict computation;
	// the loop below applies all writes (counters, cache, tints,
	// display()). Bit-exact counter parity with the pre-refactor serial
	// loop is the acceptance criterion.
	static void drainPendingDisplays() {
		ScopedUsAccumulator t(g_drainPhaseTimeUs);
		// Fast path: zero occluders this frame → depth buffer is cleared
		// → every TestRect would return VISIBLE. Skip the loop. Aggregate-
		// terrain submissions count as occluders too — 40k hill triangles
		// can still cull deferred leaves.
		if (g_rasterizedAsOccluder == 0 && g_aggregateTerrainLands == 0) {
			ScopedUsAccumulator tt(g_drainDisplayUs);
			for (const auto& p : g_pendingDisplays) {
				p.shape->vTable.asAVObject->display(p.shape, p.camera);
			}
			g_pendingDisplays.clear();
			return;
		}

		const size_t n = g_pendingDisplays.size();

		// Parallel drain DISABLED. Only the serial classify path
		// is active. The worker-pool gate, dispatch, and done-barrier are
		// preserved below as commented reference; see the worker-pool
		// block higher in this file for the re-enable checklist.
		//
		// bool poolReady = true;
		// for (unsigned int i = 0; i < kDrainWorkerCount; ++i) {
		// 	if (!g_drainWorkers[i].joinable()) { poolReady = false; break; }
		// }
		// const bool runParallel =
		// 	Configuration::OcclusionParallelDrain
		// 	&& n >= kParallelDrainMin
		// 	&& poolReady;

		// Phase 1: classify (currently serial).
		g_drainSlots.resize(n);
		// if (runParallel) {
		// 	g_lastStage = 14;
		// 	const size_t half = n / 2;
		// 	g_drainRanges[0] = { 0, half };
		// 	g_drainRanges[1] = { half, n };
		// }

		const auto classifyT0 = std::chrono::steady_clock::now();
		// if (runParallel) {
		// 	{
		// 		std::lock_guard<std::mutex> lock(g_drainMtx);
		// 		g_drainStartTickets = kDrainWorkerCount;
		// 		g_drainDoneTickets = 0;
		// 	}
		// 	g_drainStartCv.notify_all();
		// 	{
		// 		std::unique_lock<std::mutex> lock(g_drainMtx);
		// 		g_drainDoneCv.wait(lock, [] {
		// 			return g_drainDoneTickets >= kDrainWorkerCount;
		// 			});
		// 	}
		// }
		// else {
			classifyDrainRange(0, n);
		// }
		g_classifyUs = static_cast<uint64_t>(
			std::chrono::duration_cast<std::chrono::microseconds>(
				std::chrono::steady_clock::now() - classifyT0).count());

		// Shared handler for OCCLUDED verdicts (fresh + cached paths).
		auto handleOccluded = [&](const PendingDisplay& p) {
			if (g_tintOccludedEffective) {
				tintEmissive(p.shape, kTintOccluded);
				ScopedUsAccumulator tt(g_drainDisplayUs);
				p.shape->vTable.asAVObject->display(p.shape, p.camera);
			}
		};

		// Phase 2: serial action pass — counter semantics match the
		// pre-refactor loop exactly.
		g_lastStage = 15;
		for (size_t i = 0; i < n; ++i) {
			const auto& p = g_pendingDisplays[i];
			const auto& s = g_drainSlots[i];

			if (s.verdict == DrainVerdict::SkipTerrain) {
				++g_skippedTerrain;
				ScopedUsAccumulator tt(g_drainDisplayUs);
				p.shape->vTable.asAVObject->display(p.shape, p.camera);
				continue;
			}
			if (s.verdict == DrainVerdict::SkipTiny) {
				++g_skippedTesteeTiny;
				ScopedUsAccumulator tt(g_drainDisplayUs);
				p.shape->vTable.asAVObject->display(p.shape, p.camera);
				continue;
			}
			if (s.verdict == DrainVerdict::CachedOccluded) {
				++g_drainCacheHits;
				handleOccluded(p);
				continue;
			}

			// Fresh TestRect path (Visible / Occluded / ViewCulled).
			if (s.ranTestRect) {
				++g_queryTested;
				if (g_temporalCoherenceFramesEffective > 0) {
					++g_drainCacheMisses;
					// Cache only OCCLUDED — VISIBLE/VIEW_CULLED still call
					// display() this frame, so caching them adds stale-
					// pointer surface for no hit-path win.
					if (s.verdict == DrainVerdict::Occluded) {
						auto& e = g_drainCache[p.shape];
						e.shapePtr = p.shape;
						e.result = ::MaskedOcclusionCulling::OCCLUDED;
						e.lastQueryFrame = g_frameCounter;
						e.boundOriginX = p.shape->worldBoundOrigin.x;
						e.boundOriginY = p.shape->worldBoundOrigin.y;
						e.boundOriginZ = p.shape->worldBoundOrigin.z;
						e.boundRadius = p.shape->worldBoundRadius;
					}
				}
			}

			if (s.verdict == DrainVerdict::Occluded) {
				if (s.ranTestRect) ++g_queryOccluded;
				handleOccluded(p);
				continue;
			}
			if (s.verdict == DrainVerdict::ViewCulled) {
				if (s.ranTestRect) ++g_queryViewCulled;
			}
			// Skip Tested-tint when the main pass already applied
			// Occluder; last-write-wins would flip yellow → green.
			else if (g_tintTestedEffective && !p.rasterisedAsOccluder) {
				tintEmissive(p.shape, kTintTested);
			}
			{
				ScopedUsAccumulator tt(g_drainDisplayUs);
				p.shape->vTable.asAVObject->display(p.shape, p.camera);
			}
		}

		g_pendingDisplays.clear();
	}

	// ============================================================
	// CullShow detour & frame lifecycle
	// ============================================================

	// Function-level detour at NiAVObject::CullShow (0x6EB480). All 7
	// direct engine callers (NiNode::Display, 4× NiBSPNode quadrants,
	// NiSwitchNode::Display, NiCamera::Click) land here. Top-level entry
	// for the main-scene worldCamera pass drives MSOC setup/teardown;
	// every other entry just runs the body.
	static void __fastcall CullShow_detour(NI::AVObject* self, void* edx, NI::Camera* camera) {
		CallDepthGuard depthGuard;
		bool isTopLevel = false;
		TES3::Cell* activeCell = nullptr;
		if (!g_msocActive && g_inRenderMainScene) {
			auto wc = TES3::WorldController::get();
			NI::Camera* mainCamera = wc ? wc->worldCamera.cameraData.camera.get() : nullptr;
			// Skip MSOC entirely in menu mode. The threadpool's spin-
			// locks have no timeout, and menu mode is the observed
			// trigger for the worker-not-suspended race. FPS doesn't
			// matter behind a menu anyway. By not flipping isTopLevel,
			// we neither WakeThreads nor SuspendThreads while the menu
			// is up — sidesteps the race wholesale.
			const bool inMenuMode = wc && wc->flagMenuMode;
			if (inMenuMode) ++g_skippedMenuMode;
			// Multi-pass guard. The engine may fire more than one main-
			// camera CullShow per renderMainScene (shadow caster, main,
			// 1st-person subtree). Each would ClearBuffer and clobber
			// the last, so we only open isTopLevel on the FIRST entry
			// per scene. Later passes still render correctly (vanilla
			// body), they just don't re-build the mask.
			const bool alreadyBuiltThisScene = g_isTopLevelFiresThisScene > 0;
			// Outer-entry counter: ungated, this would tick on every
			// recursive descent (hundreds per frame). The !g_msocActive
			// gate restricts to true outer entries — value > 1 means
			// the engine genuinely issued multiple main-camera passes.
			if (camera == mainCamera && !inMenuMode && !g_msocActive) {
				++g_mainCamCullShowAttemptsThisScene;
			}
			if (camera == mainCamera && !inMenuMode && !alreadyBuiltThisScene) {
				// dh null on main menu / load screen — no scene to cull.
				auto dh = TES3::DataHandler::get();
				if (dh) {
					const bool isInterior = dh->currentCell
						&& dh->currentCell->getIsInterior();
					// EnableMSOC is the master runtime gate. Reconcile
					// resource state first: a fresh toggle-on allocates
					// g_msoc + threadpool now; toggle-off tears them
					// down (joins workers, frees ~57MB). Returns false
					// on alloc failure → vanilla cullShowBody this frame.
					const bool resourcesLive = ensureMSOCResourcesMatchConfig();
					const bool sceneEnabled = resourcesLive
						&& (isInterior
							? Configuration::OcclusionEnableInterior
							: Configuration::OcclusionEnableExterior);
					if (sceneEnabled) {
						isTopLevel = true;
						// Resolve scene-type occluder thresholds once
						// per top-level frame; hot path stays
						// branch-free on isInterior.
						if (isInterior) {
							g_occluderRadiusMinEffective    = Configuration::OcclusionOccluderRadiusMinInterior;
							g_occluderRadiusMaxEffective    = Configuration::OcclusionOccluderRadiusMaxInterior;
							g_occluderMinDimensionEffective = Configuration::OcclusionOccluderMinDimensionInterior;
							g_insideOccluderMarginEffective = Configuration::OcclusionInsideOccluderMarginInterior;
						} else {
							g_occluderRadiusMinEffective    = Configuration::OcclusionOccluderRadiusMinExterior;
							g_occluderRadiusMaxEffective    = Configuration::OcclusionOccluderRadiusMaxExterior;
							g_occluderMinDimensionEffective = Configuration::OcclusionOccluderMinDimensionExterior;
							g_insideOccluderMarginEffective = Configuration::OcclusionInsideOccluderMarginExterior;
						}
						// Refresh the rest of the hot-path Configuration
						// cache. See g_*Effective for the contract.
						g_depthSlackWorldUnitsEffective      = Configuration::OcclusionDepthSlackWorldUnits;
						g_occluderMaxTrianglesEffective      = Configuration::OcclusionOccluderMaxTriangles;
						g_occludeeMinRadiusEffective         = static_cast<float>(Configuration::OcclusionOccludeeMinRadius);
						g_skipTerrainOccludeesEffective      = Configuration::OcclusionSkipTerrainOccludees;
						g_aggregateTerrainEffective          = Configuration::OcclusionAggregateTerrain;
						g_terrainResolutionEffective         = Configuration::OcclusionTerrainResolution;
						g_temporalCoherenceFramesEffective   = Configuration::OcclusionTemporalCoherenceFrames;
						g_cullLightsEffective                = Configuration::OcclusionCullLights;
						g_lightCullHysteresisFramesEffective = Configuration::OcclusionLightCullHysteresisFrames;
						g_tintOccluderEffective              = Configuration::DebugOcclusionTintOccluder;
						g_tintOccludedEffective              = Configuration::DebugOcclusionTintOccluded;
						g_tintTestedEffective                = Configuration::DebugOcclusionTintTested;
						g_logEnabledEffective                = Configuration::OcclusionLogPerFrame
						                                    || Configuration::OcclusionLogAggregate;
						g_worldLandscapeRoot = dh->worldLandscapeRoot;
						// Stored only on active frames so a disabled-
						// scene span doesn't spuriously "change cell".
						activeCell = dh->currentCell;
					}
					else {
						++g_skippedSceneGate;
					}
				}
			}
		}

		if (isTopLevel) {
			++g_isTopLevelFiresThisScene;
			g_lastStage = 1;
			// Latch async mode once per frame so a mid-frame Lua toggle
			// can't split submissions between threadpool and direct MOC.
			g_asyncThisFrame = g_threadpool
				&& Configuration::OcclusionAsyncOccluders;
			// Mask about to be wiped. Consumers gate on g_maskReady —
			// clear so mid-frame queries get conservative VISIBLE
			// instead of reading a partial buffer.
			g_maskReady = false;
			if (g_asyncThisFrame) {
				// Wake workers ~100µs before the first RenderTriangles.
				// ClearBuffer's implicit Flush retires last frame's
				// stragglers, so it's safe to clear the arena after.
				g_lastStage = 2;
				{
					ScopedUsAccumulator t(g_wakeThreadsTimeUs);
					g_threadpool->WakeThreads();
				}
				g_lastStage = 3;
				g_threadpool->ClearBuffer();
				g_asyncOccluderVerts.clear();
				g_asyncOccluderIndices.clear();
			}
			else {
				g_lastStage = 3;
				g_msoc->ClearBuffer();
			}
			// Cell-change cache wipe. Must come AFTER ClearBuffer/Flush
			// so the threadpool has consumed any cached buffers
			// referenced by last-frame work — only then is it safe to
			// free them. Both caches key off pointers a cell load can
			// recycle (per-Land NiNode*, shape AVObject*).
			g_lastStage = 4;
			if (activeCell != g_lastCell) {
				g_landCache.clear();
				g_drainCache.clear();
				g_lightCullCache.clear();
				g_terrainMembership.clear();
				// Releases our NI::Pointer refs on outgoing-cell shapes
				// so they can actually be freed. Surviving shapes
				// (player, inventory) re-enter on next tint.
				g_tintClones.clear();
				g_lastCell = activeCell;
				++g_cellChanges;
			}
			g_recursiveCalls = 0;
			g_recursiveAppCulled = 0;
			g_recursiveFrustumCulled = 0;
			g_rasterizedAsOccluder = 0;
			g_occluderTriangles = 0;
			g_skippedInside = 0;
			g_skippedThin = 0;
			g_skippedAlpha = 0;
			g_skippedStencil = 0;
			g_queryTested = 0;
			g_queryOccluded = 0;
			g_queryViewCulled = 0;
			g_queryNearClip.store(0, std::memory_order_relaxed);
			g_deferred = 0;
			g_inlineTested = 0;
			g_skippedTriCount = 0;
			g_skippedTesteeTiny = 0;
			g_skippedSceneGate = 0;
			g_skippedTerrain = 0;
			g_aggregateTerrainLands = 0;
			g_aggregateTerrainTris = 0;
			g_aggregateTerrainUs = 0;
			// LAYER-A-HORIZON: per-frame reset for the Horizon-mode counters.
			g_horizonBuildUs        = 0;
			g_horizonRasterUs       = 0;
			g_horizonLandsFed       = 0;
			g_horizonVertsFed       = 0;
			g_horizonColumnsTouched = 0;
			g_horizonCurtainTris    = 0;
			g_horizonAdaptiveEpsD   = 0.0f;
			g_terrainMembershipHits   = 0;
			g_terrainMembershipMisses = 0;
			g_landCacheHits = 0;
			g_landCacheMisses = 0;
			g_landCacheEvictions = 0;
			g_drainCacheHits = 0;
			g_drainCacheMisses = 0;
			g_lightsTested = 0;
			g_lightsOccluded = 0;
			g_lightCullCacheHits = 0;
			g_lightCullCacheMisses = 0;
			++g_frameCounter;
			g_lastStage = 5;
			// Age-prune the drain cache. Window 2*N frames — older
			// entries can't be reused anyway. N=0 drops the whole map.
			{
				const unsigned int tcFrames = Configuration::OcclusionTemporalCoherenceFrames;
				if (tcFrames == 0) {
					if (!g_drainCache.empty()) g_drainCache.clear();
				}
				else {
					const uint32_t maxAge = tcFrames * 2;
					for (auto it = g_drainCache.begin(); it != g_drainCache.end(); ) {
						if (g_frameCounter - it->second.lastQueryFrame > maxAge) {
							it = g_drainCache.erase(it);
						}
						else {
							++it;
						}
					}
				}
			}
			// Age-prune the light cache. Same pattern — transient lights
			// (spell effects, projectile glows) otherwise accumulate
			// pinned by their NI::Pointer keys until cell change.
			{
				if (!Configuration::OcclusionCullLights) {
					if (!g_lightCullCache.empty()) g_lightCullCache.clear();
				}
				else {
					const uint32_t maxAge =
						Configuration::OcclusionLightCullHysteresisFrames * 2;
					for (auto it = g_lightCullCache.begin(); it != g_lightCullCache.end(); ) {
						if (g_frameCounter - it->second.lastQueryFrame > maxAge) {
							it = g_lightCullCache.erase(it);
						}
						else {
							++it;
						}
					}
				}
			}
			g_rasterizeTimeUs = 0;
			g_drainPhaseTimeUs = 0;
			g_classifyUs = 0;
			g_drainDisplayUs = 0;
			g_asyncFlushTimeUs = 0;
			// Per-frame counters reset; g_max*Session are lifetime peaks.
			g_wakeThreadsTimeUs = 0;
			g_maxCallDepthThisFrame = 0;

			// Read budgets once per frame so a mid-frame configure()
			// doesn't split a phase across two regimes. 2× threshold so
			// one-shot spikes don't trigger sustained skipping. 0 = off.
			g_rasterizeBudgetUsEffective = Configuration::OcclusionRasterizeBudgetUs;
			g_classifyBudgetUsEffective  = Configuration::OcclusionClassifyBudgetUs;
			g_skipRasterizeThisFrame =
				(g_rasterizeBudgetUsEffective > 0)
				&& (g_rasterizeEmaUs > 2 * g_rasterizeBudgetUsEffective);
			g_skipClassifyThisFrame =
				(g_classifyBudgetUsEffective > 0)
				&& (g_classifyEmaUs > 2 * g_classifyBudgetUsEffective);
			g_rasterizeBudgetTrips = 0;
			g_classifyBudgetTrips  = 0;
			g_lastStage = 6;
			uploadCameraTransform(camera);
			if (g_asyncThisFrame) {
				g_lastStage = 7;
				// Must run after uploadCameraTransform — copies the
				// matrix into the threadpool's state ring buffer.
				g_threadpool->SetMatrix(g_worldToClip);
			}
			g_msocActive = true;

			// External occluder drain. Must run AFTER ClearBuffer (so
			// it isn't stomped), AFTER uploadCameraTransform (so
			// g_worldToClip is live for direct-MOC), and BEFORE any
			// threadpool work queues.
			drainPendingOccluders();

			// Aggregate terrain. Submit merged per-Land occluders so
			// hill silhouettes are in the buffer before non-terrain
			// leaves reach the drain. Individual 25v/32t patches fail
			// the thin-axis gate; merging reclaims terrain as a useful
			// occluder. Reads the latched g_aggregateTerrainEffective
			// so the mode can't change mid-frame.
			switch (g_aggregateTerrainEffective) {
			case 1:
				g_lastStage = 8;
				rasterizeAggregateTerrain(camera);
				break;
			case 2:
				g_lastStage = 8;
				rasterizeAggregateTerrainHorizon(camera);
				break;
			case 0:
			default:
				break;
			}
		}

		g_lastStage = 9;
		cullShowBody(self, edx, camera);

		if (isTopLevel) {
			// Aggregate-terrain also goes through the threadpool, so
			// the Flush gate must include it. Otherwise a terrain-only
			// frame would SuspendThreads() with queued work in the
			// ring — suspected cause of a freeze on menu entry.
			const bool hadAsyncWork = g_asyncThisFrame
				&& (g_rasterizedAsOccluder > 0 || g_aggregateTerrainLands > 0);
			if (hadAsyncWork) {
				g_lastStage = 10;
				// Barrier: blocks until every queued RenderTriangles is
				// fully rasterised. Only after return is it safe to
				// TestRect.
				ScopedUsAccumulator t(g_asyncFlushTimeUs);
				g_threadpool->Flush();
			}
			g_lastStage = 11;
			drainPendingDisplays();
			// Snapshot swap — hands this frame's completed mask to
			// external consumers and rotates the ex-prev buffer in as
			// next frame's write target. After:
			//   g_msoc_prev = completed mask (external reads)
			//   g_msoc      = stale, will be cleared next frame
			// Freshness tick guards against culling against a frozen
			// mask during pauses / loading / alt-tab.
			std::swap(g_msoc, g_msoc_prev);
			// CRITICAL: CullingThreadpool caches the target buffer at
			// init via SetBuffer; it does NOT follow g_msoc. Without
			// this re-point, the threadpool keeps rasterising into the
			// init-time buffer and the snapshot ends up empty every
			// other frame.
			if (g_threadpool) {
				g_threadpool->SetBuffer(g_msoc);
			}
			std::memcpy(g_worldToClip_prev, g_worldToClip, sizeof(g_worldToClip));
			g_ndcRadiusX_prev = g_ndcRadiusX;
			g_ndcRadiusY_prev = g_ndcRadiusY;
			g_wGradMag_prev   = g_wGradMag;
			g_snapshotTickMs  = GetTickCount64();

			g_maskReady = true;
			if (g_asyncThisFrame) {
				g_lastStage = 12;
				// Workers back to low-overhead sleep until next frame.
				g_threadpool->SuspendThreads();
				g_asyncThisFrame = false;
			}
			g_msocActive = false;
			// Lifetime peaks survive across frames so single outliers
			// show up in the next periodic log.
			if (g_wakeThreadsTimeUs > g_maxWakeThreadsUsSession) {
				g_maxWakeThreadsUsSession = g_wakeThreadsTimeUs;
			}
			if (g_maxCallDepthThisFrame > g_maxCallDepthSession) {
				g_maxCallDepthSession = g_maxCallDepthThisFrame;
			}

			// EMAs for next frame's predictive-skip gate. Samples are
			// MSOC-only — including non-MSOC work would have the gate
			// trigger on vanilla render slowness.
			g_rasterizeEmaUs = emaUpdate(g_rasterizeEmaUs, g_rasterizeTimeUs);
			g_classifyEmaUs  = emaUpdate(g_classifyEmaUs,  g_classifyUs);
			g_rasterizeBudgetTripsSession += g_rasterizeBudgetTrips;
			g_classifyBudgetTripsSession  += g_classifyBudgetTrips;
			g_lastStage = 13;
			// Publish frame-end timestamp for the watchdog. If the next
			// frame freezes mid-body, MSOC.forensics.txt shows
			// sinceFrameEndMs growing — freeze is inside MSOC. If it
			// stays near 250ms during a freeze, the freeze is upstream.
			g_lastFrameEndTimeMs.store(
				static_cast<uint64_t>(
					std::chrono::duration_cast<std::chrono::milliseconds>(
						std::chrono::steady_clock::now()
							.time_since_epoch()).count()),
				std::memory_order_relaxed);

			// Cumulative counters survive across frames so rare OCCLUDED
			// events — scenes where one building sits squarely behind
			// another — show up even when the 300-frame sampling misses
			// them. Logged alongside per-frame counts so we can spot when
			// the ratio totalOccluded/totalTested is nonzero.
			static uint64_t totalTested = 0;
			static uint64_t totalOccluded = 0;
			static uint64_t totalViewCulled = 0;
			totalTested += g_queryTested;
			totalOccluded += g_queryOccluded;
			totalViewCulled += g_queryViewCulled;

			// Two independently-toggled log channels:
			//   - OcclusionLogAggregate: periodic 300-frame sample. Steady
			//     baseline of cumulative counters; useful for "is the
			//     culler doing anything" at a glance.
			//   - OcclusionLogPerFrame: any frame that produced an
			//     OCCLUDED verdict. Reconciliation channel for "I see
			//     culling but the counters say 0" — every culling event
			//     gets a line.
			// Both default off. Identical line format on both channels.
			const bool baselineTick = Configuration::OcclusionLogAggregate
				&& (g_frameCounter % 300) == 0;
			const bool hadOccluded = Configuration::OcclusionLogPerFrame
				&& g_queryOccluded > 0;
			if (baselineTick || hadOccluded) {
				log::getLog() << "MSOC: frame " << g_frameCounter
					<< " rasterized=" << g_rasterizedAsOccluder
					<< " occluderTris=" << g_occluderTriangles
					<< " queryOccluded=" << g_queryOccluded
					<< "/" << g_queryTested
					<< " viewCulled=" << g_queryViewCulled
					<< " nearClip=" << g_queryNearClip.load(std::memory_order_relaxed)
					<< " deferred=" << g_deferred
					<< " inlineTested=" << g_inlineTested
					<< " recursive=" << g_recursiveCalls
					<< " appCulled=" << g_recursiveAppCulled
					<< " frustumCulled=" << g_recursiveFrustumCulled
					<< " insideSkipped=" << g_skippedInside
					<< " thinSkipped=" << g_skippedThin
					<< " alphaSkipped=" << g_skippedAlpha
					<< " stencilSkipped=" << g_skippedStencil
					<< " triSkipped=" << g_skippedTriCount
					<< " tinySkipped=" << g_skippedTesteeTiny
					<< " terrainSkipped=" << g_skippedTerrain
					<< " aggTerrainLands=" << g_aggregateTerrainLands
					<< " aggTerrainTris=" << g_aggregateTerrainTris
					<< " aggTerrainUs=" << g_aggregateTerrainUs
					// Horizon-mode counters; zero unless g_aggregateTerrainEffective == 2.
					<< " horizonBuildUs=" << g_horizonBuildUs
					<< " horizonRasterUs=" << g_horizonRasterUs
					<< " horizonLandsFed=" << g_horizonLandsFed
					<< " horizonVertsFed=" << g_horizonVertsFed
					<< " horizonColumnsTouched=" << g_horizonColumnsTouched
					<< " horizonCurtainTris=" << g_horizonCurtainTris
					<< " horizonAdaptiveEpsD=" << g_horizonAdaptiveEpsD
					<< " landMembershipHit=" << g_terrainMembershipHits
					<< " landMembershipMiss=" << g_terrainMembershipMisses
					<< " landMembershipSize=" << g_terrainMembership.size()
					<< " landCacheHit=" << g_landCacheHits
					<< " landCacheMiss=" << g_landCacheMisses
					<< " landCacheEvict=" << g_landCacheEvictions
					<< " tcHit=" << g_drainCacheHits
					<< " tcMiss=" << g_drainCacheMisses
					<< " tcSize=" << g_drainCache.size()
					<< " cellChanges=" << g_cellChanges
					<< " sceneGateSkipped=" << g_skippedSceneGate
					<< " menuModeSkipped=" << g_skippedMenuMode // cumulative
					<< " rasterizeUs=" << g_rasterizeTimeUs
					<< " drainUs=" << g_drainPhaseTimeUs
					<< " classifyUs=" << g_classifyUs // phase-1 wall (serial = work; parallel = barrier)
					<< " displayUs=" << g_drainDisplayUs // audit
					<< " asyncFlushUs=" << g_asyncFlushTimeUs
					// Hybrid budget diagnostics. *Trip is per-frame (0/1),
					// *TripsSess is lifetime sum, *Ema is the predictive-skip metric
					// (compared against 2× *BudgetUs). *BudgetUs=0 means the gate
					// is disabled for that phase.
					//
					// Names changed in 0.0.10: was drain*BudgetUs/Ema/Trip; renamed
					// to class*BudgetUs/Ema/Trip because the phase being budgeted
					// is classifyDrainRange (TestRect work), not the whole drain
					// (whose phase 2 display() calls are vanilla render work).
					<< " rastBudgetUs=" << g_rasterizeBudgetUsEffective
					<< " rastEmaUs=" << g_rasterizeEmaUs
					<< " rastTrip=" << g_rasterizeBudgetTrips
					<< " rastTripsSess=" << g_rasterizeBudgetTripsSession
					<< " classBudgetUs=" << g_classifyBudgetUsEffective
					<< " classEmaUs=" << g_classifyEmaUs
					<< " classTrip=" << g_classifyBudgetTrips
					<< " classTripsSess=" << g_classifyBudgetTripsSession
					<< " wakeUs=" << g_wakeThreadsTimeUs // this frame's WakeThreads spin
					<< " maxWakeUsSess=" << g_maxWakeThreadsUsSession // lifetime peak
					<< " maxDepthFrame=" << g_maxCallDepthThisFrame // this frame's recursion peak
					<< " maxDepthSess=" << g_maxCallDepthSession // lifetime peak
					<< " tp=" << (g_threadpool ? 1 : 0) // threadpool liveness
					<< " cfgAsync=" << (Configuration::OcclusionAsyncOccluders ? 1 : 0) // async fires iff tp && cfgAsync
					<< " topLvlThisScene=" << g_isTopLevelFiresThisScene
					<< " maxTopLvlSess=" << g_maxIsTopLevelFiresSession
					<< " mainAttemptsThisScene=" << g_mainCamCullShowAttemptsThisScene
					<< " maxMainAttemptsSess=" << g_maxMainCamCullShowAttemptsSession
					<< " cumul=" << totalOccluded << "/" << totalTested
					<< "(vc=" << totalViewCulled << ")" << std::endl;
			}
		}
	}

	// ============================================================
	// renderMainScene wrapper & MSOC resource management
	// ============================================================

	// Wraps TES3Game_static::renderMainScene (0x41C400) at its 3 known
	// call sites (renderNextFrame, takeScreenshot, createSaveScreenshot).
	// Sets g_inRenderMainScene so CullShow_detour knows the main scene is
	// active. wasActive save/restore is reentry-safe (reentry not expected
	// but harmless if it happens).
	using RenderMainSceneFn = void(__cdecl*)();
	static const auto renderMainScene_original = reinterpret_cast<RenderMainSceneFn>(0x41C400);

	static void __cdecl renderMainScene_wrapper() {
		const bool wasActive = g_inRenderMainScene;
		g_inRenderMainScene = true;
		if (!wasActive) {
			g_isTopLevelFiresThisScene = 0;
			g_mainCamCullShowAttemptsThisScene = 0;
		}
		renderMainScene_original();
		if (!wasActive) {
			if (g_isTopLevelFiresThisScene > g_maxIsTopLevelFiresSession) {
				g_maxIsTopLevelFiresSession = g_isTopLevelFiresThisScene;
			}
			if (g_mainCamCullShowAttemptsThisScene > g_maxMainCamCullShowAttemptsSession) {
				g_maxMainCamCullShowAttemptsSession = g_mainCamCullShowAttemptsThisScene;
			}
		}
		g_inRenderMainScene = wasActive;

		// Reset tint clones at end-of-frame. Draws happen inside
		// renderMainScene_original after the cull pass populates the
		// render list, so by the time we return all material reads for
		// this frame are done. Runs unconditionally (no-op when
		// g_tintClones empty) so a mid-frame Lua flag toggle can't
		// leave state inconsistent.
		resetFrameTints();
	}

	// Allocate g_msoc + g_threadpool. Idempotent. Returns false on
	// allocation failure (callers treat as permanent disable). Called
	// from installPatches at startup and from the detour's ensure-helper
	// on MCM toggle-on.
	static bool createMSOCResources(std::ostream& log) {
		if (g_msoc && g_threadpool) return true;

		if (!g_msoc) {
			g_msoc = ::MaskedOcclusionCulling::Create();
			if (!g_msoc) {
				log << "MSOC: MaskedOcclusionCulling::Create() returned null; occlusion disabled." << std::endl;
				return false;
			}
			g_msoc->SetResolution(kMsocWidth, kMsocHeight);
			g_msoc->SetNearClipPlane(kNearClipW);
		}

		if (!g_msoc_prev) {
			// Snapshot buffer — external consumers read this. Same config
			// as g_msoc so the drain-complete swap preserves rasterizer
			// state. Pre-cleared so the first query (before any frame
			// has completed) sees a defined all-far mask.
			g_msoc_prev = ::MaskedOcclusionCulling::Create();
			if (!g_msoc_prev) {
				log << "MSOC: snapshot buffer Create() returned null; occlusion disabled." << std::endl;
				::MaskedOcclusionCulling::Destroy(g_msoc);
				g_msoc = nullptr;
				return false;
			}
			g_msoc_prev->SetResolution(kMsocWidth, kMsocHeight);
			g_msoc_prev->SetNearClipPlane(kNearClipW);
			g_msoc_prev->ClearBuffer();
			g_snapshotTickMs = 0;
		}

		if (!g_threadpool) {
			const unsigned int binCount = Configuration::OcclusionThreadpoolBinsW
				* Configuration::OcclusionThreadpoolBinsH;
			// Intel's CullingThreadpool uses hand-rolled spin-semaphores
			// in worker dispatch and livelocks at ~98% CPU when too many
			// workers spin against the same bin queues. A 28-thread CPU
			// freeze was traced to 26 spinning workers saturating the
			// scheduler. 12 is well above hot-path parallelism needs and
			// well under the saturation threshold on many-core CPUs.
			constexpr unsigned int kAutoMax = 12;
			unsigned int threadCount = Configuration::OcclusionThreadpoolThreadCount;
			if (threadCount == 0) {
				// Reserve 2 cores when hw > 4 (main render + slack); 1
				// when hw ≤ 4. The old plain hw-2 stranded half the CPU
				// on 4-thread parts (i5-2400: hw=4 → hwBudget=2 wasted
				// binCap=4 and gave only 2 workers).
				// binCap = binCount/2 keeps Intel's work-stealing path
				// enabled (binCount > threadCount) and avoids the 1:1
				// case where the busiest bin sets frame latency.
				const unsigned hw = std::thread::hardware_concurrency();
				const unsigned int hwBudget = (hw <= 4)
					? (hw > 1 ? hw - 1 : 1)
					: hw - 2;
				const unsigned int binCap = binCount / 2 > 0 ? binCount / 2 : 1;
				threadCount = std::min({ hwBudget, binCap, kAutoMax });
			}
			// Manual overrides above kAutoMax — auto path already caps
			// itself, so this only fires for explicit user values.
			if (threadCount > kAutoMax) {
				log << "MSOC: clamping threadCount from " << threadCount
					<< " to kAutoMax=" << kAutoMax
					<< " (Intel CullingThreadpool livelocks at high"
					<< " worker counts on saturated CPUs)." << std::endl;
				threadCount = kAutoMax;
			}
			// Threads > bins crashes inside RenderTrilist (surplus
			// workers contend for the same bin mutex).
			if (threadCount > binCount) {
				log << "MSOC: clamping threadCount from " << threadCount
					<< " to binCount=" << binCount
					<< " (threads > bins causes worker crash)." << std::endl;
				threadCount = binCount;
			}
			// One worker is strictly worse than the synchronous path —
			// pays the full WakeThreads/Flush/dispatch tax for zero
			// parallelism gain. g_threadpool stays nullptr; the
			// per-frame gate (g_threadpool && cfgAsync) silently
			// downgrades async to sync.
			if (threadCount <= 1) {
				log << "MSOC: threadCount=" << threadCount
					<< " — skipping threadpool allocation. Async path"
					   " would pay dispatch overhead with no parallelism"
					   " gain; using direct serial submission instead."
					   " Set OcclusionThreadpoolThreadCount=2+ to override."
					<< std::endl;
				g_threadpool = nullptr;
				return true;
			}

			constexpr unsigned int kMaxJobs = 64;
			// Realistic throw site: threadpool ctor's ~57MB ring-buffer
			// new[]. On any exception we have a half-built world — tear
			// it all down so the reconciler sees disabled-state next frame.
			try {
				g_threadpool = new ::CullingThreadpool(threadCount,
					Configuration::OcclusionThreadpoolBinsW,
					Configuration::OcclusionThreadpoolBinsH,
					kMaxJobs);
				g_threadpool->SetBuffer(g_msoc);
				g_threadpool->SetResolution(kMsocWidth, kMsocHeight);
				g_threadpool->SetNearClipPlane(kNearClipW);
				g_threadpool->SetVertexLayout(::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
				g_threadpool->SuspendThreads();
			}
			catch (const std::exception& e) {
				log << "MSOC: threadpool creation threw: " << e.what()
					<< "; freeing partial state, occlusion disabled this session." << std::endl;
				if (g_threadpool) {
					delete g_threadpool;
					g_threadpool = nullptr;
				}
				::MaskedOcclusionCulling::Destroy(g_msoc);
				g_msoc = nullptr;
				return false;
			}
			catch (...) {
				log << "MSOC: threadpool creation threw non-std exception; "
					"freeing partial state, occlusion disabled this session." << std::endl;
				if (g_threadpool) {
					delete g_threadpool;
					g_threadpool = nullptr;
				}
				::MaskedOcclusionCulling::Destroy(g_msoc);
				g_msoc = nullptr;
				return false;
			}

			log << "MSOC: threadpool created; threads=" << threadCount
				<< " binsW=" << Configuration::OcclusionThreadpoolBinsW
				<< " binsH=" << Configuration::OcclusionThreadpoolBinsH
				<< " maxJobs=" << kMaxJobs
				<< " tp=" << (g_threadpool ? 1 : 0)
				<< " cfgAsync=" << (Configuration::OcclusionAsyncOccluders ? 1 : 0)
				<< std::endl;
		}
		return true;
	}

	// Free g_msoc + g_threadpool and clear the async arenas. Caller must
	// ensure no async work is in flight (between frames, g_msocActive
	// false). Threadpool dtor joins all workers — bounded but blocking,
	// up to a few ms.
	static void destroyMSOCResources(std::ostream& log) {
		if (!g_msoc && !g_threadpool) return;

		if (g_threadpool) {
			delete g_threadpool;
			g_threadpool = nullptr;
		}
		if (g_msoc) {
			::MaskedOcclusionCulling::Destroy(g_msoc);
			g_msoc = nullptr;
		}
		if (g_msoc_prev) {
			::MaskedOcclusionCulling::Destroy(g_msoc_prev);
			g_msoc_prev = nullptr;
		}
		g_snapshotTickMs = 0;
		g_asyncOccluderVerts.clear();
		g_asyncOccluderVerts.shrink_to_fit();
		g_asyncOccluderIndices.clear();
		g_asyncOccluderIndices.shrink_to_fit();
		// External occluder queue: drop pending submissions so re-enable
		// doesn't replay stale ones against the fresh mask.
		{
			std::lock_guard<std::mutex> lock(g_pendingOccludersMutex);
			g_pendingOccluders.clear();
			g_externalOccluderTrisQueued = 0;
		}
		g_asyncThisFrame = false;
		g_maskReady = false;

		log << "MSOC: resources freed (threadpool joined, mask buffer destroyed)." << std::endl;
	}

	// Idempotent reconciler called at the safe top-of-frame point.
	// Returns true when resources are live and MSOC can run; false
	// when disabled or creation failed.
	static bool ensureMSOCResourcesMatchConfig() {
		auto& log = log::getLog();
		if (Configuration::EnableMSOC) {
			return createMSOCResources(log);
		}
		else {
			destroyMSOCResources(log);
			return false;
		}
	}

	// ============================================================
	// Install & public query API
	// ============================================================

	void installPatches() {
		auto& log = log::getLog();

		log << "MSOC: installPatches entered; Configuration::EnableMSOC="
			<< (Configuration::EnableMSOC ? "true" : "false")
			<< std::endl;

		// Must run before createMSOCResources so the snapshot buffer
		// and threadpool see the tier-resolved size. Aligns to MOC's
		// SUB_TILE_WIDTH=8 / SUB_TILE_HEIGHT=4 and clamps — tiny
		// resolutions trip MOC's tile math, huge ones blow out the
		// ~57MB ring buffer.
		{
			unsigned int w = Configuration::OcclusionMaskWidth;
			unsigned int h = Configuration::OcclusionMaskHeight;
			w = (w / 8) * 8;
			h = (h / 4) * 4;
			if (w < 64)   w = 64;
			if (h < 32)   h = 32;
			if (w > 2048) w = 2048;
			if (h > 1024) h = 1024;
			kMsocWidth  = w;
			kMsocHeight = h;
			log << "MSOC: mask resolution latched at " << kMsocWidth
				<< "x" << kMsocHeight
				<< " (cfg requested " << Configuration::OcclusionMaskWidth
				<< "x" << Configuration::OcclusionMaskHeight << ")."
				<< std::endl;
		}

		// Hooks always install. Resources are allocated here when
		// EnableMSOC starts on, lazily on first MCM toggle-on otherwise.
		if (Configuration::EnableMSOC) {
			createMSOCResources(log);
		}
		else {
			log << "MSOC: starting with EnableMSOC=false; resources will be allocated on first MCM toggle-on." << std::endl;
		}

		// Restart-only — see PatchForensicsWatchdog.h. Reads the gate
		// before configure() runs, so MCM edits only take effect on
		// next launch.
		forensics::spawnIfEnabled(log);

		// Drain-parallel worker pool spawn — DISABLED. See the
		// worker-pool block higher in this file for the re-enable
		// checklist.
		//
		// bool drainPoolStarted = true;
		// try {
		// 	for (unsigned int i = 0; i < kDrainWorkerCount; ++i) {
		// 		g_drainWorkers[i] = std::jthread(drainWorkerMain, i);
		// 	}
		// }
		// catch (const std::exception& e) {
		// 	log << "MSOC: drain worker spawn failed: " << e.what()
		// 		<< "; tearing down partial state, parallel drain disabled."
		// 		<< std::endl;
		// 	drainPoolStarted = false;
		// 	for (unsigned int i = 0; i < kDrainWorkerCount; ++i) {
		// 		g_drainWorkers[i] = std::jthread{};
		// 	}
		// }
		// if (drainPoolStarted) {
		// 	log << "MSOC: drain worker pool spawned; workers="
		// 		<< kDrainWorkerCount
		// 		<< " (gated by OcclusionParallelDrain)." << std::endl;
		// }

		// 5-byte prologue overwrite — we reimplement the body end-to-end
		// so no trampoline is needed. Replaces the previous 7-call-site
		// patch (equivalent coverage; all 7 direct callers land here).
		mwse::genJumpUnprotected(0x6EB480, reinterpret_cast<DWORD>(CullShow_detour));

		// Call-site wrappers for renderMainScene. We need to call the
		// original, so call-site wrapping (vs prologue trampoline) is
		// the simpler path. Enforcement check catches address drift.
		unsigned renderMainSceneInstalled = 0;
		const DWORD wrapperAddr = reinterpret_cast<DWORD>(renderMainScene_wrapper);
		static const uintptr_t kRenderMainSceneCallSites[3] = {
			0x41C08E, // TES3Game::renderNextFrame
			0x42E655, // WorldControllerRenderTarget::takeScreenshot
			0x4B50FF, // TES3File_static::createSaveScreenshot
		};
		for (auto site : kRenderMainSceneCallSites) {
			if (mwse::genCallEnforced(site, 0x41C400, wrapperAddr)) {
				++renderMainSceneInstalled;
			}
			else {
				log << "MSOC: failed to wrap renderMainScene call at 0x"
					<< std::hex << site << std::dec << std::endl;
			}
		}

		log << "MSOC: CullShow detour installed at 0x6EB480; wrapped "
			<< renderMainSceneInstalled << " / 3 renderMainScene call sites ("
			<< kMsocWidth << "x" << kMsocHeight << " tile buffer)." << std::endl;

		// Replaces the 6-byte `mov al, [ebx+NiLight.super.enabled]` at
		// 0x6bb7d4 with `call shouldLightBeEnabled; nop`. Installed
		// unconditionally; the detour short-circuits when the feature
		// is off or MSOC failed to init.
		mwse::genCallUnprotected(0x6bb7d4,
			reinterpret_cast<DWORD>(updateLights_enabledRead_hook), 6);
		log << "MSOC: light cull hook installed at 0x6bb7d4 (gated by "
			<< "Configuration::OcclusionCullLights, default off)." << std::endl;
	}

	// External-consumer mask query API. Reads the SNAPSHOT (g_msoc_prev)
	// captured at drain-complete — race-free regardless of when the
	// consumer calls. The freshness gate (kSnapshotMaxAgeMs) rejects
	// snapshots older than ~200ms so paused/load/alt-tab states don't
	// serve a frozen mask.
	bool isOcclusionMaskReady() {
		if (!g_msoc_prev || g_snapshotTickMs == 0) {
			return false;
		}
		const unsigned long long now = GetTickCount64();
		return (now - g_snapshotTickMs) <= kSnapshotMaxAgeMs;
	}

	// Sphere test against the SNAPSHOT buffer using the _prev matrix +
	// NDC constants captured at swap time. Math mirrors testSphereVisible;
	// kept separate so the in-progress-mask path stays untouched.
	static ::MaskedOcclusionCulling::CullingResult testSphereVisiblePrev(
		const TES3::Vector3& center, float radius)
	{
		// Project center through last frame's world-to-clip.
		const float* m = g_worldToClip_prev;
		const float wx = center.x, wy = center.y, wz = center.z;
		const float cx = wx * m[0] + wy * m[4] + wz * m[8] + m[12];
		const float cy = wx * m[1] + wy * m[5] + wz * m[9] + m[13];
		const float cw = wx * m[3] + wy * m[7] + wz * m[11] + m[15];

		const float wMin = cw - (radius + g_depthSlackWorldUnitsEffective) * g_wGradMag_prev;
		if (wMin <= kNearClipW) {
			return ::MaskedOcclusionCulling::VISIBLE;
		}

		const float invW = 1.0f / cw;
		const float cxNdc = cx * invW;
		const float cyNdc = cy * invW;
		const float rxNdc = radius * g_ndcRadiusX_prev * invW;
		const float ryNdc = radius * g_ndcRadiusY_prev * invW;

		float ndcMinX = std::max(cxNdc - rxNdc, -1.0f);
		float ndcMinY = std::max(cyNdc - ryNdc, -1.0f);
		float ndcMaxX = std::min(cxNdc + rxNdc, 1.0f);
		float ndcMaxY = std::min(cyNdc + ryNdc, 1.0f);
		if (ndcMinX >= ndcMaxX || ndcMinY >= ndcMaxY) {
			return ::MaskedOcclusionCulling::VIEW_CULLED;
		}

		return g_msoc_prev->TestRect(ndcMinX, ndcMinY, ndcMaxX, ndcMaxY, wMin);
	}

	MaskQueryResult testOcclusionSphere(float worldX, float worldY, float worldZ, float radius) {
		if (!isOcclusionMaskReady()) {
			return kMaskQueryNotReady;
		}
		const TES3::Vector3 center(worldX, worldY, worldZ);
		const auto result = testSphereVisiblePrev(center, radius);
		switch (result) {
		case ::MaskedOcclusionCulling::VISIBLE:     return kMaskQueryVisible;
		case ::MaskedOcclusionCulling::OCCLUDED:    return kMaskQueryOccluded;
		case ::MaskedOcclusionCulling::VIEW_CULLED: return kMaskQueryViewCulled;
		}
		return kMaskQueryVisible;
	}

	MaskQueryResult testOcclusionAABB(
		float minX, float minY, float minZ,
		float maxX, float maxY, float maxZ)
	{
		const float cx = (minX + maxX) * 0.5f;
		const float cy = (minY + maxY) * 0.5f;
		const float cz = (minZ + maxZ) * 0.5f;
		const float dx = maxX - cx;
		const float dy = maxY - cy;
		const float dz = maxZ - cz;
		const float r = std::sqrt(dx * dx + dy * dy + dz * dz);
		return testOcclusionSphere(cx, cy, cz, r);
	}

	void getSnapshotViewProj(float outMatrix[16]) {
		if (!outMatrix) return;
		std::memcpy(outMatrix, g_worldToClip_prev, sizeof(g_worldToClip_prev));
	}

	unsigned long long getSnapshotAgeMs() {
		if (g_snapshotTickMs == 0) return 0;
		return GetTickCount64() - g_snapshotTickMs;
	}

	void getMaskResolution(int* outWidth, int* outHeight) {
		if (outWidth)  *outWidth  = static_cast<int>(kMsocWidth);
		if (outHeight) *outHeight = static_cast<int>(kMsocHeight);
	}

	void testOcclusionSphereBatch(
		const float* centersAndRadii, int count, int* resultsOut)
	{
		if (!resultsOut || count <= 0) {
			return;
		}
		if (!centersAndRadii) {
			for (int i = 0; i < count; ++i) {
				resultsOut[i] = kMaskQueryNotReady;
			}
			return;
		}

		// Single readiness check — Morrowind's render-adjacent work is
		// single-threaded so the mask can't flip mid-batch.
		if (!isOcclusionMaskReady()) {
			for (int i = 0; i < count; ++i) {
				resultsOut[i] = kMaskQueryNotReady;
			}
			return;
		}

		for (int i = 0; i < count; ++i) {
			const float* s = centersAndRadii + i * 4;
			const TES3::Vector3 center(s[0], s[1], s[2]);
			const auto result = testSphereVisiblePrev(center, s[3]);
			switch (result) {
			case ::MaskedOcclusionCulling::VISIBLE:     resultsOut[i] = kMaskQueryVisible;    break;
			case ::MaskedOcclusionCulling::OCCLUDED:    resultsOut[i] = kMaskQueryOccluded;   break;
			case ::MaskedOcclusionCulling::VIEW_CULLED: resultsOut[i] = kMaskQueryViewCulled; break;
			default:                                    resultsOut[i] = kMaskQueryVisible;    break;
			}
		}
	}

	// out = a * b. Column-major (MOC/Intel layout: m[col*4 + row]).
	static void mat4MulColumnMajor(const float* a, const float* b, float* out) {
		for (int col = 0; col < 4; ++col) {
			for (int row = 0; row < 4; ++row) {
				float sum = 0.0f;
				for (int k = 0; k < 4; ++k) {
					sum += a[k * 4 + row] * b[col * 4 + k];
				}
				out[col * 4 + row] = sum;
			}
		}
	}

	// Drain external occluders into g_msoc. Must run AFTER
	// uploadCameraTransform (g_worldToClip valid) and BEFORE threadpool
	// work starts (no race on the shared MOC instance). Uses the direct
	// path — submissions are small enough that serializing on the main
	// thread is fine, and it keeps the threadpool's single-matrix
	// invariant intact for native occluders.
	static void drainPendingOccluders() {
		// Swap-and-release so a slow rasterise doesn't block consumer
		// threads in mwse_addOccluder.
		std::vector<PendingOccluder> localQueue;
		int drainedTris = 0;
		{
			std::lock_guard<std::mutex> lock(g_pendingOccludersMutex);
			if (g_pendingOccluders.empty()) {
				return;
			}
			localQueue = std::move(g_pendingOccluders);
			g_pendingOccluders.clear();
			drainedTris = g_externalOccluderTrisQueued;
			g_externalOccluderTrisQueued = 0;
		}

		for (const auto& p : localQueue) {
			// Matrix selection:
			//   preTransformed=true  → nullptr (MOC skips the transform,
			//                           consumes clip-space vertices as-is)
			//   otherwise             → g_worldToClip [* p.modelMatrix if set]
			float combinedMatrix[16];
			const float* modelToClip = nullptr;
			if (!p.preTransformed) {
				modelToClip = g_worldToClip;
				if (p.modelMatrix.has_value()) {
					mat4MulColumnMajor(g_worldToClip, p.modelMatrix->data(), combinedMatrix);
					modelToClip = combinedMatrix;
				}
			}

			const ::MaskedOcclusionCulling::VertexLayout layout(p.stride, p.offY, p.offW);
			// BACKFACE_NONE: consumers typically submit closed convex
			// hulls. Hi-Z tiles store minimum depth so back faces of a
			// convex shape can't tighten the mask. Bonus: removes the
			// winding-direction footgun.
			g_msoc->RenderTriangles(
				p.verts.data(), p.tris.data(), p.triCount,
				modelToClip,
				::MaskedOcclusionCulling::BACKFACE_NONE,
				::MaskedOcclusionCulling::CLIP_PLANE_ALL,
				layout);
			// Folds into g_occluderTriangles so the per-frame log line
			// reflects the external contribution.
			g_occluderTriangles += p.triCount;
			++g_rasterizedAsOccluder;
		}
	}

	bool addOccluder(
		const float* verts, int vtxCount, int stride, int offY, int offW,
		const unsigned int* tris, int triCount,
		const float* modelMatrix16)
	{
		if (!verts || !tris || vtxCount <= 0 || triCount <= 0) {
			return false;
		}
		// stride/offY/offW: stride positive, offsets fit one vertex.
		if (stride <= 0 || offY < 0 || offW < 0
			|| offY + static_cast<int>(sizeof(float)) > stride
			|| offW + static_cast<int>(sizeof(float)) > stride) {
			return false;
		}

		// Mask not live (EnableMSOC off, init failed, teardown). Drop
		// silently — soft-feature semantics.
		if (!g_msoc) {
			return false;
		}

		// External submissions share OcclusionOccluderMaxTriangles with
		// native occluders. External-first rejection preserves the
		// native mask under contention.
		const int cap = static_cast<int>(Configuration::OcclusionOccluderMaxTriangles);
		std::lock_guard<std::mutex> lock(g_pendingOccludersMutex);
		if (g_externalOccluderTrisQueued + triCount > cap) {
			// Rate-limit the log so a chatty consumer can't flood it.
			static bool warnOnce = true;
			if (warnOnce) {
				log::getLog() << "MSOC addOccluder: triangle budget exceeded ("
					<< g_externalOccluderTrisQueued << " queued + " << triCount
					<< " requested > cap " << cap
					<< ") — rejecting external submission" << std::endl;
				warnOnce = false;
			}
			return false;
		}

		// Copy into plugin-owned storage; consumer can free after return.
		PendingOccluder p;
		p.stride = stride;
		p.offY = offY;
		p.offW = offW;
		p.vtxCount = vtxCount;
		p.triCount = triCount;

		// stride is bytes per vertex — copy as raw bytes to preserve
		// whatever layout the caller picked.
		const size_t vtxBytes = static_cast<size_t>(vtxCount) * static_cast<size_t>(stride);
		p.verts.resize(vtxBytes / sizeof(float));
		std::memcpy(p.verts.data(), verts, vtxBytes);

		p.tris.assign(tris, tris + static_cast<size_t>(triCount) * 3);

		if (modelMatrix16) {
			std::array<float, 16> m;
			std::memcpy(m.data(), modelMatrix16, sizeof(m));
			p.modelMatrix = m;
		}

		g_externalOccluderTrisQueued += triCount;
		g_pendingOccluders.emplace_back(std::move(p));
		return true;
	}

	bool addPreTransformedOccluder(
		const float* verts, int vtxCount, int stride, int offY, int offW,
		const unsigned int* tris, int triCount)
	{
		// Same validation + budget as addOccluder, plus preTransformed.
		// Validation is duplicated rather than factored — keeps both
		// public paths readable in isolation.
		if (!verts || !tris || vtxCount <= 0 || triCount <= 0) {
			return false;
		}
		if (stride <= 0 || offY < 0 || offW < 0
			|| offY + static_cast<int>(sizeof(float)) > stride
			|| offW + static_cast<int>(sizeof(float)) > stride) {
			return false;
		}
		if (!g_msoc) {
			return false;
		}

		const int cap = static_cast<int>(Configuration::OcclusionOccluderMaxTriangles);
		std::lock_guard<std::mutex> lock(g_pendingOccludersMutex);
		if (g_externalOccluderTrisQueued + triCount > cap) {
			static bool warnOnce = true;
			if (warnOnce) {
				log::getLog() << "MSOC addPreTransformedOccluder: triangle budget exceeded ("
					<< g_externalOccluderTrisQueued << " queued + " << triCount
					<< " requested > cap " << cap
					<< ") — rejecting external submission" << std::endl;
				warnOnce = false;
			}
			return false;
		}

		PendingOccluder p;
		p.stride = stride;
		p.offY = offY;
		p.offW = offW;
		p.vtxCount = vtxCount;
		p.triCount = triCount;
		p.preTransformed = true;

		const size_t vtxBytes = static_cast<size_t>(vtxCount) * static_cast<size_t>(stride);
		p.verts.resize(vtxBytes / sizeof(float));
		std::memcpy(p.verts.data(), verts, vtxBytes);

		p.tris.assign(tris, tris + static_cast<size_t>(triCount) * 3);

		g_externalOccluderTrisQueued += triCount;
		g_pendingOccluders.emplace_back(std::move(p));
		return true;
	}

	bool dumpOcclusionMask(const char* path) {
		if (path == nullptr) {
			log::getLog() << "MSOC dump: null path" << std::endl;
			return false;
		}
		if (!isOcclusionMaskReady()) {
			const unsigned long long ageMs = g_snapshotTickMs
				? (GetTickCount64() - g_snapshotTickMs) : 0;
			log::getLog() << "MSOC dump: mask not ready (snapshotTick="
				<< g_snapshotTickMs << ", ageMs=" << ageMs
				<< ", prevPtr=" << (g_msoc_prev ? "ok" : "null") << ")" << std::endl;
			return false;
		}

		// Reads the SNAPSHOT. ComputePixelDepthBuffer takes a caller-
		// owned float[width*height].
		std::vector<float> depth(static_cast<size_t>(kMsocWidth) * kMsocHeight, 0.0f);
		g_msoc_prev->ComputePixelDepthBuffer(depth.data(), /*flipY*/ true);

		// Diagnostic: dump the LIVE buffer alongside the snapshot —
		// lets us tell whether native occluders end up in the raster
		// target but not the swapped snapshot (would imply a broken
		// swap contract). Saved with "_live" suffix.
		if (g_msoc) {
			std::vector<float> liveDepth(static_cast<size_t>(kMsocWidth) * kMsocHeight, 0.0f);
			g_msoc->ComputePixelDepthBuffer(liveDepth.data(), /*flipY*/ true);

			// Normalize live copy (same tone-map as snapshot below)
			float lMinPos = FLT_MAX, lMaxPos = -FLT_MAX;
			int lPosCount = 0;
			for (float v : liveDepth) {
				if (v > 0.0f) { ++lPosCount; if (v < lMinPos) lMinPos = v; if (v > lMaxPos) lMaxPos = v; }
			}
			const float lRange = (lPosCount > 0 && lMaxPos > lMinPos) ? (lMaxPos - lMinPos) : 1.0f;
			for (float& v : liveDepth) {
				if (v <= 0.0f) v = 0.0f;
				else v = 0.2f + 0.8f * ((v - lMinPos) / lRange);
			}

			char livePath[512];
			std::snprintf(livePath, sizeof(livePath), "%s_live.pfm", path);
			if (FILE* fl = std::fopen(livePath, "wb")) {
				std::fprintf(fl, "Pf\n%u %u\n-1.0\n", kMsocWidth, kMsocHeight);
				std::fwrite(liveDepth.data(), sizeof(float), liveDepth.size(), fl);
				std::fclose(fl);
				log::getLog() << "MSOC: live mask also dumped to " << livePath
					<< " (occluderPx=" << lPosCount
					<< " rawRange=[" << lMinPos << ".." << lMaxPos << "])" << std::endl;
			}
		}

		// Tone-map for [0, 1] viewers. MOC writes -1.0 for unwritten
		// tiles and tiny positive 1/w for occluder depth — both
		// collapse to black under auto-stretch. Remap unwritten → 0,
		// occluder → [0.2, 1.0] (lightest = nearest). Raw stats stay
		// in the log line so tooling has exact numbers.
		float minPos = FLT_MAX, maxPos = -FLT_MAX;
		int posCount = 0;
		for (float v : depth) {
			if (v > 0.0f) {
				++posCount;
				if (v < minPos) minPos = v;
				if (v > maxPos) maxPos = v;
			}
		}
		const float range = (posCount > 0 && maxPos > minPos) ? (maxPos - minPos) : 1.0f;
		for (float& v : depth) {
			if (v <= 0.0f) {
				v = 0.0f;
			} else {
				v = 0.2f + 0.8f * ((v - minPos) / range);
			}
		}

		FILE* f = std::fopen(path, "wb");
		if (!f) {
			log::getLog() << "MSOC dump: fopen failed for '" << path
				<< "' errno=" << errno << std::endl;
			return false;
		}
		std::fprintf(f, "Pf\n%u %u\n-1.0\n", kMsocWidth, kMsocHeight);
		std::fwrite(depth.data(), sizeof(float), depth.size(), f);
		std::fclose(f);

		log::getLog() << "MSOC: occlusion mask dumped to " << path
			<< " (" << kMsocWidth << "x" << kMsocHeight
			<< ", occluderPx=" << posCount
			<< ", rawRange=[" << minPos << ".." << maxPos << "])" << std::endl;
		return true;
	}

	MaskQueryResult testOcclusionOBB(
		float cx, float cy, float cz,
		float vxX, float vxY, float vxZ,
		float vyX, float vyY, float vyZ,
		float vzX, float vzY, float vzZ)
	{
		if (!isOcclusionMaskReady()) {
			return kMaskQueryNotReady;
		}

		// 8 corners = center + (±vx ±vy ±vz). (x, y, z, w) stride 16.
		float corners[8 * 4];
		const float sx[8] = { -1, +1, +1, -1, -1, +1, +1, -1 };
		const float sy[8] = { -1, -1, +1, +1, -1, -1, +1, +1 };
		const float sz[8] = { -1, -1, -1, -1, +1, +1, +1, +1 };
		for (int i = 0; i < 8; ++i) {
			corners[i * 4 + 0] = cx + sx[i] * vxX + sy[i] * vyX + sz[i] * vzX;
			corners[i * 4 + 1] = cy + sx[i] * vxY + sy[i] * vyY + sz[i] * vzY;
			corners[i * 4 + 2] = cz + sx[i] * vxZ + sy[i] * vyZ + sz[i] * vzZ;
			corners[i * 4 + 3] = 1.0f;
		}

		static const unsigned int tris[12 * 3] = {
			0, 1, 2,  0, 2, 3,   // -Z face (0-1-2-3)
			4, 6, 5,  4, 7, 6,   // +Z face (4-5-6-7)
			0, 5, 1,  0, 4, 5,   // -Y face (0-1-5-4)
			3, 2, 6,  3, 6, 7,   // +Y face (3-2-6-7)
			0, 3, 7,  0, 7, 4,   // -X face (0-3-7-4)
			1, 5, 6,  1, 6, 2,   // +X face (1-2-6-5)
		};

		const ::MaskedOcclusionCulling::VertexLayout layout(16, 4, 12); // stride, offY, offW

		// Test against the SNAPSHOT buffer with the SNAPSHOT matrix.
		const auto result = g_msoc_prev->TestTriangles(
			corners, tris, 12,
			g_worldToClip_prev,
			::MaskedOcclusionCulling::BACKFACE_NONE,
			::MaskedOcclusionCulling::CLIP_PLANE_ALL,
			layout);

		switch (result) {
		case ::MaskedOcclusionCulling::VISIBLE:     return kMaskQueryVisible;
		case ::MaskedOcclusionCulling::OCCLUDED:    return kMaskQueryOccluded;
		case ::MaskedOcclusionCulling::VIEW_CULLED: return kMaskQueryViewCulled;
		}
		return kMaskQueryVisible;
	}

}

// ============================================================
// C-ABI exports (out-of-tree consumers: MGE-XE etc.)
// ============================================================

// C-ABI shims. void*/NI::Light* share representation on x86 and both
// callback pointers are __cdecl, so the reinterpret_cast is an ABI
// formality. Consumers look up by name (not ordinal) — names are
// frozen.
extern "C" __declspec(dllexport)
void __cdecl mwse_registerLightObservedCallback(void(__cdecl* cb)(void* niLight)) {
	using Typed = msoc::patch::occlusion::LightObservedCallback;
	msoc::patch::occlusion::registerLightObservedCallback(
		reinterpret_cast<Typed>(cb));
}

extern "C" __declspec(dllexport)
void __cdecl mwse_unregisterLightObservedCallback(void(__cdecl* cb)(void* niLight)) {
	using Typed = msoc::patch::occlusion::LightObservedCallback;
	msoc::patch::occlusion::unregisterLightObservedCallback(
		reinterpret_cast<Typed>(cb));
}

// Mask query exports. Returns MWSE_OCC_* codes (0=Visible, 1=Occluded,
// 2=ViewCulled, 3=NotReady) — frozen ABI independent of Intel's enum.
// Gate with mwse_isOcclusionMaskReady() if unsure about timing.
extern "C" __declspec(dllexport)
int __cdecl mwse_isOcclusionMaskReady() {
	return msoc::patch::occlusion::isOcclusionMaskReady() ? 1 : 0;
}

extern "C" __declspec(dllexport)
int __cdecl mwse_testOcclusionSphere(float worldX, float worldY, float worldZ, float radius) {
	return static_cast<int>(
		msoc::patch::occlusion::testOcclusionSphere(worldX, worldY, worldZ, radius));
}

extern "C" __declspec(dllexport)
int __cdecl mwse_testOcclusionAABB(
	float minX, float minY, float minZ,
	float maxX, float maxY, float maxZ)
{
	return static_cast<int>(
		msoc::patch::occlusion::testOcclusionAABB(minX, minY, minZ, maxX, maxY, maxZ));
}

extern "C" __declspec(dllexport)
int __cdecl mwse_testOcclusionOBB(
	float cx, float cy, float cz,
	float vxX, float vxY, float vxZ,
	float vyX, float vyY, float vyZ,
	float vzX, float vzY, float vzZ)
{
	return static_cast<int>(
		msoc::patch::occlusion::testOcclusionOBB(
			cx, cy, cz,
			vxX, vxY, vxZ,
			vyX, vyY, vyZ,
			vzX, vzY, vzZ));
}

extern "C" __declspec(dllexport)
void __cdecl mwse_testOcclusionSphereBatch(
	const float* centersAndRadii, int count, int* resultsOut)
{
	msoc::patch::occlusion::testOcclusionSphereBatch(
		centersAndRadii, count, resultsOut);
}

extern "C" __declspec(dllexport)
void __cdecl mwse_getSnapshotViewProj(float outMatrix[16]) {
	msoc::patch::occlusion::getSnapshotViewProj(outMatrix);
}

extern "C" __declspec(dllexport)
unsigned long long __cdecl mwse_getSnapshotAgeMs() {
	return msoc::patch::occlusion::getSnapshotAgeMs();
}

extern "C" __declspec(dllexport)
void __cdecl mwse_getMaskResolution(int* outWidth, int* outHeight) {
	msoc::patch::occlusion::getMaskResolution(outWidth, outHeight);
}

extern "C" __declspec(dllexport)
int __cdecl mwse_dumpOcclusionMask(const char* path) {
	return msoc::patch::occlusion::dumpOcclusionMask(path) ? 1 : 0;
}

extern "C" __declspec(dllexport)
int __cdecl mwse_addOccluder(
	const float* verts, int vtxCount, int stride, int offY, int offW,
	const unsigned int* tris, int triCount,
	const float* modelMatrix16)
{
	return msoc::patch::occlusion::addOccluder(
		verts, vtxCount, stride, offY, offW,
		tris, triCount,
		modelMatrix16) ? 1 : 0;
}

extern "C" __declspec(dllexport)
int __cdecl mwse_addPreTransformedOccluder(
	const float* verts, int vtxCount, int stride, int offY, int offW,
	const unsigned int* tris, int triCount)
{
	return msoc::patch::occlusion::addPreTransformedOccluder(
		verts, vtxCount, stride, offY, offW,
		tris, triCount) ? 1 : 0;
}
