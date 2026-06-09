#include "PatchOcclusionCulling.h"

#include "Log.h"
#include "MemoryUtil.h"
#include "Config.h"
// Engine-free clip-space / projection math (matrix transpose, projectWorld,
// row-norm metrics, column-major mat4 multiply). Header-only + unit-tested.
#include "ClipMath.h"
// Engine-free phase-budget math (EMA, predictive-skip / spike-clip
// decisions). Header-only + unit-tested.
#include "Profiling.h"
// Debug-only occlusion tint overlay (recolors classified leaves). Engine-
// coupled; lives in its own TU.
#include "DebugTint.h"
// Per-frame snapshot of the Configuration knobs the hot path reads.
#include "FrameConfig.h"
// Cross-TU shared state (g_frame, g_snapshot, g_msoc_prev, mask consts) for
// the extracted subsystem TUs (QueryApi.cpp, ...).
#include "OcclusionInternal.h"
// LAYER-A-HORIZON: 1D horizon -> curtain occluder used by the Horizon
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
#include "NIPoint3.h"
#include "NIPoint4.h"
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

	// kNearClipW (MOC near-plane w floor) now lives in OcclusionInternal.h,
	// shared with the subsystem TUs.

	// NI::Camera accessors that read fields the upstream MWSE NICamera.h
	// labels `unknown_*`. A proposed (unmerged) header adds named fields
	// (cullingPlanePtrs at 0x148, countCullingPlanes at 0x160,
	// usedCullingPlanesBitfield at 0x1C4); these accessors derive the
	// same data from the upstream-decoded cullingPlanes[6]:
	//   - countCullingPlanes is always 6 for the main world camera.
	//   - cullingPlanePtrs[i] is a TArray<NiPlane*> whose first 6 entries
	//     mirror &cullingPlanes[i] (NiCamera::UpdateWorldData copies via
	//     those pointers).
	//   - usedCullingPlanesBitfield lives at &cullingPlanes[6] - past
	//     the inline array. Computed via offset arithmetic to avoid the
	//     past-end-deref UB of &cullingPlanes[6].x.
	// Swap bodies to `cam->countCullingPlanes` etc. when upstream merges.
	static inline int cameraCountCullingPlanes(const NI::Camera* /*cam*/) {
		return 6;
	}

	static inline const NI::Point4* cameraCullingPlane(NI::Camera* cam, int i) {
		return &cam->cullingPlanes[i];
	}

	static inline uint32_t* cameraUsedPlanesMask(NI::Camera* cam) {
		auto* base = reinterpret_cast<char*>(&cam->cullingPlanes[0]);
		return reinterpret_cast<uint32_t*>(base + sizeof(NI::Point4) * 6);
	}

	// ============================================================
	// MSOC instance & per-frame camera state
	// ============================================================

	// Heap-allocated via Create()/Destroy(). Leaked on process exit.
	::MaskedOcclusionCulling* g_msoc = nullptr;  // extern in OcclusionInternal.h
	// Double-buffer. g_msoc_prev holds the PREVIOUS frame's completed
	// mask - what external consumers (MGE-XE) read from. We rasterize
	// into g_msoc each frame and swap at drain-complete so consumers
	// always see "the mask as of the end of the previous frame".
	// Defined here, declared extern in OcclusionInternal.h (read by QueryApi).
	::MaskedOcclusionCulling* g_msoc_prev = nullptr;
	// Snapshot published at the drain-complete buffer swap. The MaskSnapshot
	// type + kSnapshotMaxAgeMs live in OcclusionInternal.h; the query API
	// projects through g_snapshot, not the live per-frame matrix.
	MaskSnapshot g_snapshot;

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
	// MSOC so Click trees outside the main scene - load splash, UI
	// targets, chargen preview, MGE water reflection - run vanilla.
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
	// Inline (during traversal) vs deferred (drain-phase) test counts -
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
	// Terrain occludees bypassed from TestRect - patches under the
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

	// Horizon-mode counters (active when g_frame.aggregateTerrain == 2):
	//   g_horizonBuildUs:  end-to-end (project + simplify + emit + submit).
	//   g_horizonRasterUs: inner subset - only the curtain RenderTriangles.
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





	// Miss-path workload probes: count walked ancestor steps and transformed
	// verts on the cache-miss path. Together with hit/miss above these
	// confirm the cache is amortising the work it's supposed to.
	static uint64_t g_classifyOccluderCalls = 0;
	static uint64_t g_classifyOccluderSteps = 0;
	static uint64_t g_occluderVertexCalls   = 0;
	static uint64_t g_occluderVertexVerts   = 0;



	OcclusionCaches g_caches;  // extern in OcclusionCaches.h


	// Visible-geom observers - registered by external consumers (MGE-XE)
	// to receive the MSOC-culled visible set before display() is called.
	static std::vector<VisibleGeomCallback> g_visibleGeomObservers;
	static std::vector<void*>  g_visCallbackNodes;   // reused each frame, avoids alloc
	static std::vector<float>  g_visCallbackBounds;  // xyzr per node, reused each frame

	// ============================================================
	// Frame counters & diagnostic state
	// ============================================================

	// File-scope frame counter; incremented once per top-level frame.
	// Used by the drain loop for cache-freshness checks.
	uint32_t g_frameCounter = 0;  // extern in OcclusionInternal.h
	// Cell pointer across frames. Cell change -> wipe g_caches.land and
	// g_caches.drain (NI::Node*/NI::AVObject* recycle in the new cell).
	static TES3::Cell* g_lastCell = nullptr;
	static uint64_t g_cellChanges = 0;
	// Cell-cross profiling (Configuration::OcclusionLogCellCross). Set to a
	// small frame budget when the cell changes; the per-frame stats line is
	// emitted while it counts down, capturing the cross + the re-population
	// frames where the caches refill on the miss path. g_cellWipeUs times the
	// cache-wipe itself; g_lastFrameDeltaUs is this frame's wall time.
	static int g_cellCrossLogFrames = 0;
	static uint64_t g_cellWipeUs = 0;
	static uint64_t g_lastFrameDeltaUs = 0;

	// Phase timers (us). steady_clock is QPC-backed on MSVC. Rasterize
	// accumulates each RenderTriangles call; drainPhase wraps the whole
	// drain body; classify wraps phase 1 (the verdict pass) on the main
	// thread - single writer in both serial and parallel modes.
	static uint64_t g_rasterizeTimeUs = 0;
	static uint64_t g_drainPhaseTimeUs = 0;
	static uint64_t g_classifyUs = 0;
	static uint64_t g_drainDisplayUs = 0;
	// Occluder world-space vertex transform (the cache-miss rebuild loop in
	// rasterizeTriShape). Separate from rasterizeUs (which is the MOC
	// RenderTriangles call): this is the R*v*s+T + AABB pass that re-runs for
	// every occluder when the occluder cache is wiped on a cell cross.
	static uint64_t g_occluderTransformUs = 0;

	// Hybrid phase budgeting (Configuration::Occlusion*BudgetUs):
	//   Layer 1 - predictive skip: if EMA(last frames) > 2x budget,
	//     skip the whole phase this frame. Self-regulating, no in-loop
	//     cost. 2x multiplier so single spikes don't trigger skipping.
	//   Layer 2 - spike clip: rasterize checks cumulative time at the
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
	// EMA = (prev * 7 + sample) >> 3 - ~6-frame half-life.
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

	// EMA + budget decisions live in profiling:: now; alias keeps the local
	// call sites terse.
	using profiling::emaUpdate;
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
	// Last-checkpoint marker. Stable numbering - DO NOT renumber:
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

	// Frame-time sampling for per-window FPS logging. Microsecond precision
	// (the existing watchdog timestamp is millisecond, too coarse for a
	// per-frame delta). `g_windowFrameTimeUs` is the running sum since the
	// last log emission, `g_windowFrameCount` the matching frame count;
	// divide for the window average and zero both inside the log block.
	static uint64_t g_prevFrameEndUs    = 0;
	static uint64_t g_windowFrameTimeUs = 0;
	static uint64_t g_windowFrameCount  = 0;

	static bool ensureMSOCResourcesMatchConfig();

	// ============================================================
	// Threadpool state & per-frame config cache
	// ============================================================

	// g_threadpool lives for the process. Runtime-toggle of
	// OcclusionAsyncOccluders works without restart - Wake/Suspend is
	// driven per-frame off that flag, suspended threadpool sleeps at ~0%.
	//
	// g_asyncThisFrame latches the flag at top-level entry so mid-frame
	// MCM edits don't mix sync/async submissions within one traversal.
	//
	// Buffer stability for async submission: occluder cache entries live in
	// unordered_map nodes whose addresses are stable across insertions, and
	// the entries themselves persist until cell-change wipe. So the buffer
	// pointers handed to the threadpool stay valid until Flush() - no
	// per-submission arena copy needed.
	static ::CullingThreadpool* g_threadpool = nullptr;
	static bool g_asyncThisFrame = false;

	// Per-top-level-frame snapshot of the Configuration:: knobs the hot path
	// reads (FrameConfig::snapshot). Resolved once after isInterior is known;
	// collapses interior/exterior selection AND the Configuration reloads
	// (external-linkage statics the optimizer must otherwise re-read after
	// every opaque MOC/NI call) out of the inner loops. MCM only commits
	// between frames, so one snapshot per top-level CullShow is sound. The
	// field defaults in FrameConfig match Config.cpp so pre-snapshot reads
	// (init, off-frame light callback) stay consistent.
	// Defined here, declared extern in OcclusionInternal.h (read by QueryApi).
	FrameConfig g_frame;

	// Drain-parallel worker pool - DISABLED. Serial drain is the only
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
	// Forensics watchdog - read accessor
	// ============================================================

	// Sole bridge to PatchForensicsWatchdog.cpp. Defined here so the
	// hot-path statics stay internal-linkage. Called once per ~250ms;
	// no synchronisation - single loads, post-mortem-grade coherence.
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

	// Gated on the log channels - when both are off the timer is a no-op
	// (nullptr target, no clock reads). Read-once at construction so a
	// mid-scope MCM toggle can't flip the dtor onto a still-zero start.
	struct ScopedUsAccumulator {
		uint64_t* target;
		std::chrono::steady_clock::time_point start;
		explicit ScopedUsAccumulator(uint64_t& t)
			: target(g_frame.logEnabled ? &t : nullptr) {
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
	// consumed by phase 2. Phase 1 stays read-only on shared state -
	// prerequisite for moving it to worker threads.
	enum class DrainVerdict : uint8_t {
		Visible,        // VISIBLE; call display()
		Occluded,       // OCCLUDED; skip display (or tint+display in debug)
		ViewCulled,     // rect collapsed; treat like Visible for display()
		SkipTerrain,    // bypassed TestRect (terrain descendant); call display()
		SkipTiny,       // bypassed TestRect (radius < threshold); call display()
		CachedOccluded, // g_caches.drain hit; verdict OCCLUDED (only cached verdict)
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
	// Occluder property classification & terrain membership
	// ============================================================
	// (Debug tinting moved to DebugTint.{h,cpp}.)

	// Single-pass occluder property classifier. Walks ancestors once,
	// resolving first-of-type-wins NiAlphaProperty and NiStencilProperty
	// in the same scan. Nearest-ancestor-wins applies independently per
	// type - alpha and stencil may live on different ancestors.
	//
	// Why these can't be occluders:
	//   alpha   - blended/alpha-tested shapes (fences, banners, grates,
	//             vines, leaves, glass) are partially transparent. A
	//             solid-occluder rasterise fills the full quad and
	//             falsely occludes whatever sits behind the holes.
	//   stencil - fills only where the test passes (shadow volumes,
	//             reflection clip masks, UI cutouts). Same hazard.
	// Both still participate as occludees; only the occluder rasterise
	// pass skips them.
	struct OccluderPropertyFlags {
		bool alpha;
		bool stencil;
	};
	static OccluderPropertyFlags classifyOccluderProperties(NI::AVObject* obj) {
		// Probes gated on g_frame.logEnabled - same contract as
		// ScopedUsAccumulator. Off-path: predicted-not-taken branch + no
		// counter store; on-path: 4 atomic-free uint64 inc per frame's
		// miss-set.
		const bool logOn = g_frame.logEnabled;
		if (logOn) ++g_classifyOccluderCalls;
		OccluderPropertyFlags out = { false, false };
		bool alphaResolved = false;
		bool stencilResolved = false;
		for (NI::AVObject* cur = obj; cur; cur = cur->parentNode) {
			if (logOn) ++g_classifyOccluderSteps;
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
	// Pre-cache profile (Vivec exterior peak): 2354 calls x 7.3 avg =
	// ~17k pointer chases per frame, dominated by L2 misses on cold
	// scene-graph nodes. Cached because parent-chain is stable within a
	// cell; wiped on cell change.
	static bool isLandscapeDescendant(NI::AVObject* obj, NI::Node* root) {
		if (!root || !obj) return false;
		auto it = g_caches.terrainMembership.find(obj);
		if (it != g_caches.terrainMembership.end()) {
			++g_caches.terrainMembershipHits;
			return it->second.isDescendant;
		}
		++g_caches.terrainMembershipMisses;
		bool result = false;
		for (NI::AVObject* cur = obj; cur; cur = cur->parentNode) {
			if (cur == root) { result = true; break; }
		}
		g_caches.terrainMembership.emplace(
			obj, TerrainMembershipEntry{ NI::Pointer<NI::AVObject>(obj), result });
		return result;
	}

	// ============================================================
	// Camera & projection math
	// ============================================================

	// Transpose NI::Camera::worldToCamera into Intel's column-major v*M
	// layout. NI stores row-major M*v: clip[r] = sum_c ni[r*4+c]*v[c].
	// Intel reads mtx[c*4+r] for the same out[r]. So mtx[c*4+r] = ni[r*4+c].
	static void uploadCameraTransform(NI::Camera* cam) {
		const float* ni = reinterpret_cast<const float*>(&cam->worldToCamera);
		clipmath::transposeRowToColumnMajor(ni, g_worldToClip);

		// Per-frame sphere-projection metrics (upper bounds on
		// |d(clip)/d(pos)|), derived from the transposed matrix rows.
		const clipmath::RowNorms norms = clipmath::clipRowNorms(g_worldToClip);
		g_ndcRadiusX = norms.ndcRadiusX;
		g_ndcRadiusY = norms.ndcRadiusY;
		g_wGradMag = norms.wGradMag;
	}

	// Project a world-space point through the cached live matrix. Thin
	// forwarder over clipmath::projectWorld bound to g_worldToClip; kept so
	// the hot-path call sites stay terse. (Definition is header-inline, so
	// this still inlines fully.)
	using ClipXYW = clipmath::ClipXYW;
	static inline ClipXYW projectWorld(float wx, float wy, float wz) {
		return clipmath::projectWorld(g_worldToClip, wx, wy, wz);
	}

	// Sphere -> NDC-rect + wmin -> TestRect. Projects only the center and
	// derives the NDC half-extent + near-surface clip-w from per-frame
	// matrix metrics. Tighter than an 8-corner AABB project by ~sqrt3 and
	// stable under camera rotation - small-mesh queries don't flicker
	// across TestRect's hiZ thresholds.
	//
	// Sphere straddling the near plane bails as VISIBLE; TestRect can't
	// project a straddling rect safely.
	::MaskedOcclusionCulling::CullingResult testSphereVisible(
		const NI::Point3& center, float radius) {
		const ClipXYW c = projectWorld(center.x, center.y, center.z);

		const float wMin = c.w - (radius + g_frame.depthSlackWorldUnits) * g_wGradMag;
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
	// Visible-geom observer registration
	// ============================================================

	// Dedup on register so DLL reload doesn't double-dispatch. Startup
	// only, not hot path.


	void registerVisibleGeomCallback(VisibleGeomCallback cb) {
		if (!cb) return;
		if (std::find(g_visibleGeomObservers.begin(), g_visibleGeomObservers.end(), cb)
				!= g_visibleGeomObservers.end()) return;
		g_visibleGeomObservers.push_back(cb);
	}
	void unregisterVisibleGeomCallback(VisibleGeomCallback cb) {
		auto it = std::find(g_visibleGeomObservers.begin(), g_visibleGeomObservers.end(), cb);
		if (it != g_visibleGeomObservers.end()) g_visibleGeomObservers.erase(it);
	}


	// ============================================================
	// Occluder rasterisation
	// ============================================================

	// Rasterise a shape's real triangles (world-space) as an occluder.
	// Using real triangles instead of the AABB prevents "sign on wall
	// falsely occluded because it lives inside the wall's AABB."
	// World-space verts + indices + AABB live in the supplied cache entry
	// (g_caches.occluder); only re-derived when worldTransform changes.
	// Returns true on rasterise, false on skip (no data, thin axis,
	// inside-guard if enabled, or budget gate).
	static bool rasterizeTriShape(NI::TriBasedGeometry* shape, const NI::Point3& eye, OccluderCacheEntry& cache) {
		// Phase budget gate. Compares against g_rasterizeTimeUs (sum
		// of completed RenderTriangles SIMD time), NOT wall-clock-since-
		// frame-start. The latter would include cullShowBody traversal
		// between calls and falsely trip every frame.
		if (g_skipRasterizeThisFrame) {
			g_rasterizeBudgetTrips = 1;
			return false;
		}
		if (profiling::spikeClipTripped(g_rasterizeTimeUs, g_rasterizeBudgetUsEffective)) {
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
		// Raw fields are allocation sizes - Intel's gather crashes on
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
		// Dense meshes pay rasterisation cost ~ triCount but rarely
		// occlude proportionally better. Still tested as occludees.
		if (triCount > g_frame.occluderMaxTriangles) {
			++g_skippedTriCount;
			return false;
		}

		const auto& xf = shape->worldTransform;

		// Geom cache: world-space verts + indices + AABB are invariant
		// across frames for static cell meshes. Recompute only when
		// worldTransform changes (catches moved pickables) or on first
		// touch. Cache entries live until cell-change wipe.
		// unordered_map node addresses are stable across insertions, so
		// the buffer data pointers we pass to threadpool/g_msoc remain
		// valid until cell change - no per-frame arena copy needed.
		const bool xfStale = !cache.geomResolved
			|| std::memcmp(cache.xfData, &xf, sizeof(cache.xfData)) != 0;
		if (xfStale) {
			// Measure the vertex transform + index expansion: dominates the
			// occluder re-population on a cell cross (occVertVerts).
			ScopedUsAccumulator transformTimer(g_occluderTransformUs);
			const auto& R = xf.rotation;
			const auto& T = xf.translation;
			const float s = xf.scale;

			if (g_frame.logEnabled) {
				++g_occluderVertexCalls;
				g_occluderVertexVerts += vertexCount;
			}
			cache.worldVerts.resize(static_cast<size_t>(vertexCount) * 3);
			float* out = cache.worldVerts.data();
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
			cache.minX = minX; cache.minY = minY; cache.minZ = minZ;
			cache.maxX = maxX; cache.maxY = maxY; cache.maxZ = maxZ;
			cache.vertexCount = vertexCount;

			// Expand 16-bit indices into MSOC's 32-bit list and drop any
			// out-of-bounds entries (Intel's gather would crash).
			cache.indices.resize(static_cast<size_t>(triCount) * 3);
			unsigned int* idx = cache.indices.data();
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
			cache.indices.resize(static_cast<size_t>(outTri) * 3);
			cache.outTriCount = outTri;

			cache.geomResolved = true;
			std::memcpy(cache.xfData, &xf, sizeof(cache.xfData));
		}

		// Reject pencil shapes - tiny silhouette area, near-zero
		// occlusion. Walls/floors (single thin axis) still qualify.
		const float minDim = g_frame.occluderMinDimension;
		const float dx = cache.maxX - cache.minX;
		const float dy = cache.maxY - cache.minY;
		const float dz = cache.maxZ - cache.minZ;
		const int thinAxes = (dx < minDim ? 1 : 0)
			+ (dy < minDim ? 1 : 0)
			+ (dz < minDim ? 1 : 0);
		if (thinAxes >= 2) {
			++g_skippedThin;
			return false;
		}

		// Inside-guard: opt-in via OcclusionInsideOccluderGuard. The
		// original rationale (eye-inside mesh writes near-face depths
		// that falsely occlude things behind the far face) did not hold
		// up empirically - with BACKFACE_NONE rasterising both sides,
		// MOC's tile semantics handle the concave-shell case fine, and
		// the AABB+margin gate was eating close-up walls (the wall's
		// AABB+64 wu contains the player whenever they're near it).
		// Default off; flip OcclusionInsideOccluderGuard=true in
		// msoc.json to restore the old rejection.
		if (g_frame.insideOccluderGuard) {
			const float m = g_frame.insideOccluderMargin;
			if (eye.x >= cache.minX - m && eye.x <= cache.maxX + m &&
				eye.y >= cache.minY - m && eye.y <= cache.maxY + m &&
				eye.z >= cache.minZ - m && eye.z <= cache.maxZ + m) {
				++g_skippedInside;
				return false;
			}
		}

		if (cache.outTriCount == 0) {
			return false;
		}

		// VertexLayout(12, 4, 8): stride 12, y@4, z@8 - packed float[3].
		// BACKFACE_NONE: NIF winding isn't guaranteed consistent.
		if (g_asyncThisFrame) {
			ScopedUsAccumulator t(g_rasterizeTimeUs);
			g_threadpool->RenderTriangles(cache.worldVerts.data(), cache.indices.data(),
				static_cast<int>(cache.outTriCount),
				::MaskedOcclusionCulling::BACKFACE_NONE,
				::MaskedOcclusionCulling::CLIP_PLANE_ALL);
		}
		else {
			ScopedUsAccumulator t(g_rasterizeTimeUs);
			g_msoc->RenderTriangles(cache.worldVerts.data(), cache.indices.data(),
				static_cast<int>(cache.outTriCount), g_worldToClip,
				::MaskedOcclusionCulling::BACKFACE_NONE,
				::MaskedOcclusionCulling::CLIP_PLANE_ALL,
				::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
		}
		g_occluderTriangles += cache.outTriCount;
		return true;
	}

	// ============================================================
	// Terrain aggregation
	// ============================================================

	// Frustum check for the aggregate-terrain walk. No mask bookkeeping -
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

		// Downsample fast path. Terrain subcells are 5x5 row-major grids
		// (25v/32t). step=2 -> 3x3 (8 tris); step=4 -> 2x2 (2 tris). Sample
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
			// covered by the last-row/column kept vertex - the seam
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
		switch (g_frame.terrainResolution) {
		case 1: return 2;
		case 2: return 4;
		default: return 1;
		}
	}

	// Build VB+IB for one per-Land NiNode (cache-miss path). Walks every
	// subcell unconditionally - result must be valid for any camera angle.
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
		entry.builtForResolution = static_cast<uint8_t>(g_frame.terrainResolution);
	}

	// Horizon-mode terrain occluder. Walks WorldLandscape, projects each
	// vertex to clip space, bins into a 1D max-y/max-w horizon, simplifies
	// to ~60 adaptive samples, emits ~120 curtain triangles, submits those
	// to MOC via sync RenderTriangles. No land cache (fresh projection
	// each frame). Frustum-cull at land granularity only. epsD is adaptive
	// (do NOT pass 1e30 like MGE-XE's distant path - see HorizonOccluder.h).
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

				// Per-subcell frustum cull. A Land covers 8192x8192;
				// typically ~5-8 of its 16 subcells are in-frustum.
				// Skipping invisible ones avoids the per-vertex
				// transform - the dominant cost. Quality is unchanged
				// (the c.w / ndcX guards below would've rejected these
				// post-projection anyway).
				if (frustumCulledSphere(subNode, camera)) continue;

				const auto& shapes = subNode->children;
				for (size_t k = 0; k < shapes.endIndex; ++k) {
					auto* shape = shapes.storage[k].get();
					if (!shape) continue;
					if (!shape->isInstanceOfType(NI::RTTIStaticPtr::NiTriShape)) continue;

					// Skip alpha/stencil shapes - same gate as the raster
					// path. A transparent terrain patch shouldn't write
					// occlusion regardless of mode.
					const auto props = classifyOccluderProperties(shape);
					if (props.alpha || props.stencil) continue;

					auto* tri = static_cast<NI::TriShape*>(shape);
					// getModelData returns a smart Pointer<TriShapeData>,
					// not a raw pointer - use auto, not auto*. Matches
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
						// Near-plane straddling - projection unstable.
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

		// Simplify -> margin -> emit -> fixup -> submit.
		constexpr int   kMaxSamples    = 60;
		constexpr float kEpsH          = 0.01f;
		constexpr int   kTileAlign     = 16;
		constexpr float kNdcYBottom    = -1.1f;
		constexpr float kYSafetyMargin = 0.04f;
		constexpr float kEpsDFraction  = 0.05f;
		constexpr float kEpsDFloor     = 100.0f;

		// D12: adaptive epsD computed from this frame's depth range.
		// computeAdaptiveEpsD returns +infinity if fewer than 2 cols are
		// active - equivalent to disabling the depth term, safe fallback.
		const float adaptiveEpsD = horizon.computeAdaptiveEpsD(kEpsDFraction, kEpsDFloor);

		static msoc::horizon::Sample samples[kMaxSamples];
		const int nSamples = horizon.simplify(samples, kMaxSamples, kEpsH, adaptiveEpsD, kTileAlign);
		if (nSamples < 2) return;

		// Pull the curtain top down by the safety margin. Skip sentinel
		// rows (those columns will be skipped by emit anyway).
		for (int i = 0; i < nSamples; ++i) {
			if (samples[i].h > -1.0e29f) samples[i].h -= kYSafetyMargin;
		}

		// Per-frame scratch - function-static so allocations amortise.
		// Sized for the worst case (every segment emits 6 verts).
		static std::vector<msoc::horizon::CurtainVertex> curtainVerts;
		curtainVerts.resize(static_cast<size_t>(6 * (nSamples - 1)));
		const int triCount = horizon.emitCurtainNDC(samples, nSamples, kNdcYBottom,
			curtainVerts.data(), static_cast<int>(curtainVerts.size()));
		if (triCount <= 0) return;

		// Convert NDC layout (x, y, z=depth, w=1) -> MOC pre-transformed
		// layout (x*d, y*d, _, d) in place. Submit with VertexLayout
		// matching that layout: stride=16, offY=4, offW=12.
		msoc::horizon::HorizonOccluder::fixupForMOC(curtainVerts.data(), triCount * 3);

		static std::vector<unsigned int> curtainIdx;
		const int vtxCount = triCount * 3;
		curtainIdx.resize(static_cast<size_t>(vtxCount));
		for (int i = 0; i < vtxCount; ++i) {
			curtainIdx[i] = static_cast<unsigned int>(i);
		}

		// Sync submit; bypasses the threadpool - ~120 tris is below the
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

	// Aggregate terrain rasteriser. Walks WorldLandscape (root -> Land ->
	// 16 subcells -> N NiTriShapes) and submits one combined occluder per
	// visible Land. Individual 25v/32t patches fail the thin-axis gate;
	// merging gives the hill/horizon silhouette that actually occludes
	// distant architecture.
	//
	// Must run inside isTopLevel - after ClearBuffer + uploadCameraTransform,
	// before cullShowBody - so the drain sees terrain in the depth buffer.
	//
	// Uses g_caches.land to amortise the per-Land walk + vertex transform.
	// Mark-and-sweep evicts entries whose NiNode wasn't seen this frame.
	static void rasterizeAggregateTerrain(NI::Camera* camera) {
		if (!g_worldLandscapeRoot) return;
		if (g_worldLandscapeRoot->getAppCulled()) return;

		ScopedUsAccumulator timer(g_aggregateTerrainUs);

		// Mark all cached entries unseen; we'll flip this for entries we
		// touch this frame and evict the remainder below.
		for (auto& kv : g_caches.land) kv.second.seen = false;

		const auto& landChildren = g_worldLandscapeRoot->children;
		for (size_t i = 0; i < landChildren.endIndex; ++i) {
			auto* land = landChildren.storage[i].get();
			if (!land) continue;
			if (!land->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;

			auto* landNode = static_cast<NI::Node*>(land);

			// Get-or-build the cached buffer. Keyed by NiNode pointer;
			// a cell-change realloc produces a new key and rebuilds.
			auto [it, inserted] = g_caches.land.try_emplace(landNode);
			LandCacheEntry& entry = it->second;
			// Resolution-mismatch on hit forces a rebuild - MCM dropdown
			// changes propagate lazily. nodePtr stays valid (NI::Pointer
			// pins the NiNode); only verts/indices redo.
			const uint8_t curRes = static_cast<uint8_t>(g_frame.terrainResolution);
			if (inserted) {
				entry.nodePtr = landNode;
				buildLandCacheEntry(entry, landNode);
				++g_caches.landMisses;
			}
			else if (entry.builtForResolution != curRes) {
				buildLandCacheEntry(entry, landNode);
				++g_caches.landMisses;
			}
			else {
				++g_caches.landHits;
			}
			entry.seen = true;

			// Per-frame submit gates. Shape-level filters are intentionally
			// omitted from cache build - they'd bake view-specific state
			// into a reusable buffer.
			if (land->getAppCulled()) continue;
			if (frustumCulledSphere(land, camera)) continue;
			if (entry.triCount == 0) continue;

			// Per-subcell frustum cull. ~5-8 of a Land's 16 subcells are
			// typically in-frustum; skipping the rest trades one big
			// submission for N small ones with fewer total triangles -
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
				// Fallback path - single submission for the whole Land.
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
		for (auto it = g_caches.land.begin(); it != g_caches.land.end(); ) {
			if (!it->second.seen) {
				it = g_caches.land.erase(it);
				++g_caches.landEvictions;
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
			// own-visibility test - MSOC's "closest 1/w wins" means a
			// shape that's itself occluded just overdraws tiles with
			// farther depths (harmless). Removing the own-test breaks the
			// cycle where a shape fails against its own sibling's raster
			// and still produces usable occluder depth.
			//
			// NiNodes are NOT MSOC-tested - only frustum-culled. Testing
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
				// already in the buffer as merged per-Land submissions -
				// re-rasterising would be dup work.
				const bool skipAsAggregated = (g_frame.aggregateTerrain != 0)
					&& isLandscapeDescendant(self, g_worldLandscapeRoot);
				if (!skipAsAggregated
					&& boundRadius >= g_frame.occluderRadiusMin
					&& boundRadius <= g_frame.occluderRadiusMax) {
					auto& cacheEntry = g_caches.occluderEntry(self);
					if (!cacheEntry.propsResolved) {
						if (g_frame.logEnabled) ++g_caches.occluderMisses;
						const auto p = classifyOccluderProperties(self);
						cacheEntry.alpha = p.alpha;
						cacheEntry.stencil = p.stencil;
						cacheEntry.propsResolved = true;
					} else {
						if (g_frame.logEnabled) ++g_caches.occluderHits;
					}
					if (cacheEntry.alpha) {
						++g_skippedAlpha;
					}
					else if (cacheEntry.stencil) {
						++g_skippedStencil;
					}
					else {
						const auto& eye = camera->worldTransform.translation;
						if (rasterizeTriShape(static_cast<NI::TriBasedGeometry*>(self), eye, cacheEntry)) {
							++g_rasterizedAsOccluder;
							didRasterise = true;
							if (g_frame.tintOccluder) {
								debugtint::tintOccluder(self);
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

	// Drain phase 1 - classify [lo, hi) of g_pendingDisplays into
	// g_drainSlots. Read-only on shared state (no g_caches.drain writes,
	// no counter increments except the atomic g_queryNearClip, no
	// display(), no tints) - phase 2 owns all writes. Read-only-ness
	// is what lets this run on workers once parallel dispatch lands.
	static void classifyDrainRange(size_t lo, size_t hi) {
		// Captured here, not at drainPendingDisplays entry, so the
		// spike-clip measures classify-only elapsed (excluding phase 2's
		// vanilla D3D8 display() calls).
		g_classifyPhaseStart = std::chrono::steady_clock::now();

		// Predictive skip: EMA over 2x budget -> skip TestRect entirely
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

		const unsigned int tcFrames = g_frame.temporalCoherenceFrames;
		const bool skipTerrainEnabled = g_frame.skipTerrainOccludees;
		const float tinyThreshold = g_frame.occludeeMinRadius;

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
				auto it = g_caches.drain.find(p.shape);
				if (it != g_caches.drain.end()) {
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
			// per-call timing here (worker CPU time != wall time).
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

	// Parallel-drain worker body - DISABLED. See the worker-pool block
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
		// Fast path: zero occluders this frame -> depth buffer is cleared
		// -> every TestRect would return VISIBLE. Skip the loop. Aggregate-
		// terrain submissions count as occluders too - 40k hill triangles
		// can still cull deferred leaves.
		if (g_rasterizedAsOccluder == 0 && g_aggregateTerrainLands == 0) {
			if (!g_visibleGeomObservers.empty()) {
				const size_t nFast = g_pendingDisplays.size();
				g_visCallbackNodes.clear();
				g_visCallbackBounds.clear();
				g_visCallbackNodes.reserve(nFast);
				g_visCallbackBounds.reserve(nFast * 4);
				for (size_t i = 0; i < nFast; ++i) {
					const auto& p = g_pendingDisplays[i];
					g_visCallbackNodes.push_back(p.shape);
					g_visCallbackBounds.push_back(p.shape->worldBoundOrigin.x);
					g_visCallbackBounds.push_back(p.shape->worldBoundOrigin.y);
					g_visCallbackBounds.push_back(p.shape->worldBoundOrigin.z);
					g_visCallbackBounds.push_back(p.shape->worldBoundRadius);
				}
				for (const auto cb : g_visibleGeomObservers)
					cb(g_visCallbackNodes.data(), g_visCallbackBounds.data(),
					   (int)g_visCallbackNodes.size());
			}
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

		// Fire visible-geom callback: give external consumers (e.g. MGE-XE) the exact
		// set of nodes Morrowind will draw, before any display() calls.
		if (!g_visibleGeomObservers.empty()) {
			g_visCallbackNodes.clear();
			g_visCallbackBounds.clear();
			g_visCallbackNodes.reserve(n);
			g_visCallbackBounds.reserve(n * 4);
			for (size_t i = 0; i < n; ++i) {
				const auto& s = g_drainSlots[i];
				if (s.verdict == DrainVerdict::Occluded ||
				    s.verdict == DrainVerdict::CachedOccluded) continue;
				const auto& p = g_pendingDisplays[i];
				g_visCallbackNodes.push_back(p.shape);
				g_visCallbackBounds.push_back(p.shape->worldBoundOrigin.x);
				g_visCallbackBounds.push_back(p.shape->worldBoundOrigin.y);
				g_visCallbackBounds.push_back(p.shape->worldBoundOrigin.z);
				g_visCallbackBounds.push_back(p.shape->worldBoundRadius);
			}
			for (const auto cb : g_visibleGeomObservers)
				cb(g_visCallbackNodes.data(), g_visCallbackBounds.data(),
				   (int)g_visCallbackNodes.size());
		}

		// Shared handler for OCCLUDED verdicts (fresh + cached paths).
		auto handleOccluded = [&](const PendingDisplay& p) {
			if (g_frame.tintOccluded) {
				debugtint::tintOccluded(p.shape);
				ScopedUsAccumulator tt(g_drainDisplayUs);
				p.shape->vTable.asAVObject->display(p.shape, p.camera);
			}
		};

		// Phase 2: serial action pass - counter semantics match the
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
				++g_caches.drainHits;
				handleOccluded(p);
				continue;
			}

			// Fresh TestRect path (Visible / Occluded / ViewCulled).
			if (s.ranTestRect) {
				++g_queryTested;
				if (g_frame.temporalCoherenceFrames > 0) {
					++g_caches.drainMisses;
					// Cache only OCCLUDED - VISIBLE/VIEW_CULLED still call
					// display() this frame, so caching them adds stale-
					// pointer surface for no hit-path win.
					if (s.verdict == DrainVerdict::Occluded) {
						auto& e = g_caches.drain[p.shape];
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
			// Occluder; last-write-wins would flip yellow -> green.
			else if (g_frame.tintTested && !p.rasterisedAsOccluder) {
				debugtint::tintTested(p.shape);
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
	// direct engine callers (NiNode::Display, 4x NiBSPNode quadrants,
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
			// is up - sidesteps the race wholesale.
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
			// gate restricts to true outer entries - value > 1 means
			// the engine genuinely issued multiple main-camera passes.
			if (camera == mainCamera && !inMenuMode && !g_msocActive) {
				++g_mainCamCullShowAttemptsThisScene;
			}
			if (camera == mainCamera && !inMenuMode && !alreadyBuiltThisScene) {
				// dh null on main menu / load screen - no scene to cull.
				auto dh = TES3::DataHandler::get();
				if (dh) {
					const bool isInterior = dh->currentCell
						&& dh->currentCell->getIsInterior();
					// EnableMSOC is the master runtime gate. Reconcile
					// resource state first: a fresh toggle-on allocates
					// g_msoc + threadpool now; toggle-off tears them
					// down (joins workers, frees ~57MB). Returns false
					// on alloc failure -> vanilla cullShowBody this frame.
					const bool resourcesLive = ensureMSOCResourcesMatchConfig();
					const bool sceneEnabled = resourcesLive
						&& (isInterior
							? Configuration::OcclusionEnableInterior
							: Configuration::OcclusionEnableExterior);
					if (sceneEnabled) {
						isTopLevel = true;
						// Resolve every hot-path Configuration knob once per
						// top-level frame; inner loops stay branch-free on
						// isInterior and immune to a mid-frame configure().
						g_frame.snapshot(isInterior);
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
			// Mask about to be wiped. Consumers gate on g_maskReady -
			// clear so mid-frame queries get conservative VISIBLE
			// instead of reading a partial buffer.
			g_maskReady = false;
			if (g_asyncThisFrame) {
				// Wake workers ~100us before the first RenderTriangles.
				// ClearBuffer's implicit Flush retires last frame's
				// stragglers.
				g_lastStage = 2;
				{
					ScopedUsAccumulator t(g_wakeThreadsTimeUs);
					g_threadpool->WakeThreads();
				}
				g_lastStage = 3;
				g_threadpool->ClearBuffer();
			}
			else {
				g_lastStage = 3;
				g_msoc->ClearBuffer();
			}
			// Cell-change cache wipe. Must come AFTER ClearBuffer/Flush
			// so the threadpool has consumed any cached buffers
			// referenced by last-frame work - only then is it safe to
			// free them. Both caches key off pointers a cell load can
			// recycle (per-Land NiNode*, shape AVObject*).
			g_lastStage = 4;
			g_cellWipeUs = 0;
			if (activeCell != g_lastCell) {
				{
					// Time just the wipe - the one cross cost the per-frame
					// timers don't already capture.
					ScopedUsAccumulator wipeTimer(g_cellWipeUs);
					g_caches.wipeForCellChange();
					// Releases our NI::Pointer refs on outgoing-cell shapes
					// so they can actually be freed. Surviving shapes
					// (player, inventory) re-enter on next tint.
					debugtint::clearClones();
				}
				g_lastCell = activeCell;
				++g_cellChanges;
				// Profile this cross + the next few frames as the caches
				// refill on the miss path (the spike isn't just frame 0).
				g_cellCrossLogFrames = 8;
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
			g_caches.terrainMembershipHits   = 0;
			g_caches.terrainMembershipMisses = 0;
			g_classifyOccluderCalls   = 0;
			g_classifyOccluderSteps   = 0;
			g_occluderVertexCalls     = 0;
			g_occluderVertexVerts     = 0;
			g_caches.occluderHits       = 0;
			g_caches.occluderMisses     = 0;
			g_caches.landHits = 0;
			g_caches.landMisses = 0;
			g_caches.landEvictions = 0;
			g_caches.drainHits = 0;
			g_caches.drainMisses = 0;
			g_caches.lightsTested = 0;
			g_caches.lightsOccluded = 0;
			g_caches.lightCullHits = 0;
			g_caches.lightCullMisses = 0;
			++g_frameCounter;
			g_lastStage = 5;
			// Age-prune the drain cache. Window 2*N frames - older
			// entries can't be reused anyway. N=0 drops the whole map.
			{
				const unsigned int tcFrames = Configuration::OcclusionTemporalCoherenceFrames;
				if (tcFrames == 0) {
					if (!g_caches.drain.empty()) g_caches.drain.clear();
				}
				else {
					const uint32_t maxAge = tcFrames * 2;
					for (auto it = g_caches.drain.begin(); it != g_caches.drain.end(); ) {
						if (g_frameCounter - it->second.lastQueryFrame > maxAge) {
							it = g_caches.drain.erase(it);
						}
						else {
							++it;
						}
					}
				}
			}
			// Age-prune the light cache. Same pattern - transient lights
			// (spell effects, projectile glows) otherwise accumulate
			// pinned by their NI::Pointer keys until cell change.
			{
				if (!Configuration::OcclusionCullLights) {
					if (!g_caches.lightCull.empty()) g_caches.lightCull.clear();
				}
				else {
					const uint32_t maxAge =
						Configuration::OcclusionLightCullHysteresisFrames * 2;
					for (auto it = g_caches.lightCull.begin(); it != g_caches.lightCull.end(); ) {
						if (g_frameCounter - it->second.lastQueryFrame > maxAge) {
							it = g_caches.lightCull.erase(it);
						}
						else {
							++it;
						}
					}
				}
			}
			g_rasterizeTimeUs = 0;
			g_occluderTransformUs = 0;
			g_drainPhaseTimeUs = 0;
			g_classifyUs = 0;
			g_drainDisplayUs = 0;
			g_asyncFlushTimeUs = 0;
			// Per-frame counters reset; g_max*Session are lifetime peaks.
			g_wakeThreadsTimeUs = 0;
			g_maxCallDepthThisFrame = 0;

			// Read budgets once per frame so a mid-frame configure()
			// doesn't split a phase across two regimes. 2x threshold so
			// one-shot spikes don't trigger sustained skipping. 0 = off.
			g_rasterizeBudgetUsEffective = Configuration::OcclusionRasterizeBudgetUs;
			g_classifyBudgetUsEffective  = Configuration::OcclusionClassifyBudgetUs;
			g_skipRasterizeThisFrame =
				profiling::predictiveSkip(g_rasterizeEmaUs, g_rasterizeBudgetUsEffective);
			g_skipClassifyThisFrame =
				profiling::predictiveSkip(g_classifyEmaUs, g_classifyBudgetUsEffective);
			g_rasterizeBudgetTrips = 0;
			g_classifyBudgetTrips  = 0;
			g_lastStage = 6;
			uploadCameraTransform(camera);
			if (g_asyncThisFrame) {
				g_lastStage = 7;
				// Must run after uploadCameraTransform - copies the
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
			// occluder. Reads the latched g_frame.aggregateTerrain
			// so the mode can't change mid-frame.
			switch (g_frame.aggregateTerrain) {
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
			// ring - suspected cause of a freeze on menu entry.
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
			// Snapshot swap - hands this frame's completed mask to
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
			std::memcpy(g_snapshot.worldToClip, g_worldToClip, sizeof(g_worldToClip));
			g_snapshot.ndcRadiusX = g_ndcRadiusX;
			g_snapshot.ndcRadiusY = g_ndcRadiusY;
			g_snapshot.wGradMag   = g_wGradMag;
			g_snapshot.tickMs  = GetTickCount64();

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
			// MSOC-only - including non-MSOC work would have the gate
			// trigger on vanilla render slowness.
			g_rasterizeEmaUs = emaUpdate(g_rasterizeEmaUs, g_rasterizeTimeUs);
			g_classifyEmaUs  = emaUpdate(g_classifyEmaUs,  g_classifyUs);
			g_rasterizeBudgetTripsSession += g_rasterizeBudgetTrips;
			g_classifyBudgetTripsSession  += g_classifyBudgetTrips;
			g_lastStage = 13;
			// Publish frame-end timestamp for the watchdog. If the next
			// frame freezes mid-body, MSOC.forensics.txt shows
			// sinceFrameEndMs growing - freeze is inside MSOC. If it
			// stays near 250ms during a freeze, the freeze is upstream.
			const auto frameEndNowUs = static_cast<uint64_t>(
				std::chrono::duration_cast<std::chrono::microseconds>(
					std::chrono::steady_clock::now()
						.time_since_epoch()).count());
			g_lastFrameEndTimeMs.store(frameEndNowUs / 1000,
				std::memory_order_relaxed);
			// Accumulate frame-to-frame delta into the per-window window
			// so the log emission can publish an average FPS. Skip the
			// first frame (no prior sample) to avoid a giant first delta.
			// Gated on the log channels for symmetry with the rest of
			// the diagnostic surface; first window after a mid-session
			// toggle reports avgFrameUs=0 until samples accumulate.
			if (g_frame.logEnabled) {
				if (g_prevFrameEndUs != 0) {
					g_lastFrameDeltaUs = frameEndNowUs - g_prevFrameEndUs;
					g_windowFrameTimeUs += g_lastFrameDeltaUs;
					++g_windowFrameCount;
				}
				g_prevFrameEndUs = frameEndNowUs;
			}
			else {
				// Reset so a flip-on later doesn't fold in a stale delta
				// spanning the off-period.
				g_prevFrameEndUs = 0;
			}

			// Cumulative counters survive across frames so rare OCCLUDED
			// events - scenes where one building sits squarely behind
			// another - show up even when the 300-frame sampling misses
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
			//     culling but the counters say 0" - every culling event
			//     gets a line.
			// Both default off. Identical line format on both channels.
			const bool baselineTick = Configuration::OcclusionLogAggregate
				&& (g_frameCounter % 300) == 0;
			const bool hadOccluded = Configuration::OcclusionLogPerFrame
				&& g_queryOccluded > 0;
			// Cell-cross channel: fire on the cross frame + the re-population
			// frames. cellCross=<age> (0 = cross frame). Same line format.
			const bool cellCrossTick = Configuration::OcclusionLogCellCross
				&& g_cellCrossLogFrames > 0;
			if (baselineTick || hadOccluded || cellCrossTick) {
				log::getLog() << "MSOC: frame " << g_frameCounter
					<< " cellCross=" << (cellCrossTick ? (8 - g_cellCrossLogFrames) : -1)
					<< " cellWipeUs=" << g_cellWipeUs
					<< " frameDeltaUs=" << g_lastFrameDeltaUs
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
					// Horizon-mode counters; zero unless g_frame.aggregateTerrain == 2.
					<< " horizonBuildUs=" << g_horizonBuildUs
					<< " horizonRasterUs=" << g_horizonRasterUs
					<< " horizonLandsFed=" << g_horizonLandsFed
					<< " horizonVertsFed=" << g_horizonVertsFed
					<< " horizonColumnsTouched=" << g_horizonColumnsTouched
					<< " horizonCurtainTris=" << g_horizonCurtainTris
					<< " horizonAdaptiveEpsD=" << g_horizonAdaptiveEpsD
					<< " landMembershipHit=" << g_caches.terrainMembershipHits
					<< " landMembershipMiss=" << g_caches.terrainMembershipMisses
					<< " landMembershipSize=" << g_caches.terrainMembership.size()
					<< " classOccCalls=" << g_classifyOccluderCalls
					<< " classOccSteps=" << g_classifyOccluderSteps
					<< " occVertCalls=" << g_occluderVertexCalls
					<< " occVertVerts=" << g_occluderVertexVerts
					<< " occCacheHit=" << g_caches.occluderHits
					<< " occCacheMiss=" << g_caches.occluderMisses
					<< " occCacheSize=" << g_caches.occluder.size()
					<< " landCacheHit=" << g_caches.landHits
					<< " landCacheMiss=" << g_caches.landMisses
					<< " landCacheEvict=" << g_caches.landEvictions
					// Light-cull A/B counters. lightCullMiss == lightsTested
					// (every miss runs the MSOC test); kept separate for
					// symmetry with other *Hit/Miss pairs.
					<< " lightCullHit=" << g_caches.lightCullHits
					<< " lightCullMiss=" << g_caches.lightCullMisses
					<< " lightOccluded=" << g_caches.lightsOccluded
					<< " lightCacheSize=" << g_caches.lightCull.size()
					// Average per-frame time across this sample window.
					// 1e6 / avgFrameUs = average FPS. Reset right after.
					<< " avgFrameUs=" << (g_windowFrameCount
						? (g_windowFrameTimeUs / g_windowFrameCount) : 0)
					<< " framesInWin=" << g_windowFrameCount
					<< " tcHit=" << g_caches.drainHits
					<< " tcMiss=" << g_caches.drainMisses
					<< " tcSize=" << g_caches.drain.size()
					<< " cellChanges=" << g_cellChanges
					<< " sceneGateSkipped=" << g_skippedSceneGate
					<< " menuModeSkipped=" << g_skippedMenuMode // cumulative
					<< " rasterizeUs=" << g_rasterizeTimeUs
					<< " occXformUs=" << g_occluderTransformUs
					<< " drainUs=" << g_drainPhaseTimeUs
					<< " classifyUs=" << g_classifyUs // phase-1 wall (serial = work; parallel = barrier)
					<< " displayUs=" << g_drainDisplayUs // audit
					<< " asyncFlushUs=" << g_asyncFlushTimeUs
					// Hybrid budget diagnostics. *Trip is per-frame (0/1),
					// *TripsSess is lifetime sum, *Ema is the predictive-skip metric
					// (compared against 2x *BudgetUs). *BudgetUs=0 means the gate
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
				// Reset the per-window frame-time accumulator so the next
				// log line reflects its own window only.
				g_windowFrameTimeUs = 0;
				g_windowFrameCount  = 0;
			}

			// Count down the cell-cross profiling budget (set to 8 on the
			// cross); runs every frame so cellCross ages 0..7 then stops.
			if (g_cellCrossLogFrames > 0) {
				--g_cellCrossLogFrames;
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
		// this frame are done. Runs unconditionally (no-op when no clones
		// exist) so a mid-frame Lua flag toggle can't leave state
		// inconsistent.
		debugtint::resetFrameTints();
	}

	static void destroyMSOCResources(std::ostream& log);  // complete teardown, used by the create-failure paths

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
			// Snapshot buffer - external consumers read this. Same config
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
			g_snapshot.tickMs = 0;
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
				// when hw <= 4. The old plain hw-2 stranded half the CPU
				// on 4-thread parts (i5-2400: hw=4 -> hwBudget=2 wasted
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
			// Manual overrides above kAutoMax - auto path already caps
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
			// One worker is strictly worse than the synchronous path -
			// pays the full WakeThreads/Flush/dispatch tax for zero
			// parallelism gain. g_threadpool stays nullptr; the
			// per-frame gate (g_threadpool && cfgAsync) silently
			// downgrades async to sync.
			if (threadCount <= 1) {
				log << "MSOC: threadCount=" << threadCount
					<< " - skipping threadpool allocation. Async path"
					   " would pay dispatch overhead with no parallelism"
					   " gain; using direct serial submission instead."
					   " Set OcclusionThreadpoolThreadCount=2+ to override."
					<< std::endl;
				g_threadpool = nullptr;
				return true;
			}

			constexpr unsigned int kMaxJobs = 64;
			// Realistic throw site: threadpool ctor's ~57MB ring-buffer
			// new[]. On any exception we have a half-built world - tear
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
				destroyMSOCResources(log);
				return false;
			}
			catch (...) {
				log << "MSOC: threadpool creation threw non-std exception; "
					"freeing partial state, occlusion disabled this session." << std::endl;
				destroyMSOCResources(log);
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

	// Free g_msoc + g_threadpool and release cached state. Caller must
	// ensure no async work is in flight (between frames, g_msocActive
	// false). Threadpool dtor joins all workers - bounded but blocking,
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
		g_snapshot.tickMs = 0;
		// Release NI::Pointer refcounts on every cached static, drop the
		// world-vert/index buffers; re-enable rebuilds from scratch.
		g_caches.occluder.clear();
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
		// SUB_TILE_WIDTH=8 / SUB_TILE_HEIGHT=4 and clamps - tiny
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

		// Restart-only - see PatchForensicsWatchdog.h. Reads the gate
		// before configure() runs, so MCM edits only take effect on
		// next launch.
		forensics::spawnIfEnabled(log);

		// Drain-parallel worker pool spawn - DISABLED. See the
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

		// 5-byte prologue overwrite - we reimplement the body end-to-end
		// so no trampoline is needed. Replaces the previous 7-call-site
		// patch (equivalent coverage; all 7 direct callers land here).
		se::memory::genJumpUnprotected(0x6EB480, reinterpret_cast<DWORD>(CullShow_detour));

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
			if (se::memory::genCallEnforced(site, 0x41C400, wrapperAddr)) {
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
		se::memory::genCallUnprotected(0x6bb7d4,
			reinterpret_cast<DWORD>(updateLights_enabledRead_hook), 6);
		log << "MSOC: light cull hook installed at 0x6bb7d4 (gated by "
			<< "Configuration::OcclusionCullLights, default off)." << std::endl;
	}

	// Drain external occluders into g_msoc. Must run AFTER
	// uploadCameraTransform (g_worldToClip valid) and BEFORE threadpool
	// work starts (no race on the shared MOC instance). Uses the direct
	// path - submissions are small enough that serializing on the main
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
			//   preTransformed=true  -> nullptr (MOC skips the transform,
			//                           consumes clip-space vertices as-is)
			//   otherwise             -> g_worldToClip [* p.modelMatrix if set]
			float combinedMatrix[16];
			const float* modelToClip = nullptr;
			if (!p.preTransformed) {
				modelToClip = g_worldToClip;
				if (p.modelMatrix.has_value()) {
					clipmath::mat4MulColumnMajor(g_worldToClip, p.modelMatrix->data(), combinedMatrix);
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
		// silently - soft-feature semantics.
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
					<< ") - rejecting external submission" << std::endl;
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

		// stride is bytes per vertex - copy as raw bytes to preserve
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
		// Validation is duplicated rather than factored - keeps both
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
					<< ") - rejecting external submission" << std::endl;
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
			const unsigned long long ageMs = g_snapshot.tickMs
				? (GetTickCount64() - g_snapshot.tickMs) : 0;
			log::getLog() << "MSOC dump: mask not ready (snapshotTick="
				<< g_snapshot.tickMs << ", ageMs=" << ageMs
				<< ", prevPtr=" << (g_msoc_prev ? "ok" : "null") << ")" << std::endl;
			return false;
		}

		// Reads the SNAPSHOT. ComputePixelDepthBuffer takes a caller-
		// owned float[width*height].
		std::vector<float> depth(static_cast<size_t>(kMsocWidth) * kMsocHeight, 0.0f);
		g_msoc_prev->ComputePixelDepthBuffer(depth.data(), /*flipY*/ true);

		// Diagnostic: dump the LIVE buffer alongside the snapshot -
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
		// tiles and tiny positive 1/w for occluder depth - both
		// collapse to black under auto-stretch. Remap unwritten -> 0,
		// occluder -> [0.2, 1.0] (lightest = nearest). Raw stats stay
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


}
