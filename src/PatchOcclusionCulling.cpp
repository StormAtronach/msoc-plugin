#include "PatchOcclusionCulling.h"

#include "Log.h"
#include "MemoryUtil.h"
#include "Config.h"

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

	// _Claude_ Plugin migration alias: bare `log::getLog()` used by this
	// translation unit resolved to `mwse::log` in MWSE proper. In the
	// plugin the logger lives at msoc::log (separate file, separate
	// implementation), so alias it here for source-level compatibility.
	// Three call sites otherwise; this one line keeps them unchanged.
	namespace log = ::msoc::log;

	// MSOC tile-buffer resolution. Decoupled from the game viewport: the
	// rasterizer works in NDC via the camera's world-to-clip matrix, then maps
	// NDC to this framebuffer via its own internal scale/offset.
	// _Claude_ No longer constexpr — latched from Configuration at install
	// time (see installPatches::latchMaskResolution). Values are otherwise
	// treated as immutable for the session: changing
	// Configuration::OcclusionMaskWidth/Height after install is a no-op
	// (and the comment above each Lua-side default flags it as restart-
	// only). Hot-path uses (depth-buffer sizing, dump-to-PFM headers,
	// SetResolution / GetResolution accessors) all read these, but none
	// in tight per-frame loops where the lost compile-time folding would
	// matter — MOC's tile loop reads its own internal mWidth/mHeight at
	// runtime regardless, so the rasterizer wasn't const-folding these
	// to begin with.
	unsigned int kMsocWidth  = 512;
	unsigned int kMsocHeight = 256;

	// Clip-space w floor. Intel MSOC treats this as the near plane. In
	// Morrowind units (1 unit ~= 1.4 cm) 1.0 is a safe floor below the
	// engine's own near plane and well above the numerical noise that
	// explodes NDC after perspective divide. Not exposed as a knob —
	// numerical floor, lowering it risks NaN in projection math.
	constexpr float kNearClipW = 1.0f;

	// All other occluder/occludee thresholds (radius bounds, thin-axis
	// rejection, inside-occluder margin, testee depth slack, max triangles,
	// min occludee radius, scene-type gates) are Configuration::Occlusion*
	// knobs loaded from MWSE.json; see MWSEConfig.{h,cpp} for defaults and
	// PatchOcclusionCulling-plan.md for the rationale behind each default.

	// _Claude_ NI::Camera field accessors that adapt to the upstream MWSE
	// NICamera.h layout. Upstream only decodes `cullingPlanes[6]` directly
	// and labels the surrounding slots as `unknown_*`; the proposed (but
	// unmerged) NICamera.h adds named fields for `cullingPlanePtrs` (0x148),
	// `countCullingPlanes` (0x160), and `usedCullingPlanesBitfield[4]`
	// (0x1C4). Rather than depend on the unmerged proposal we read those
	// slots through the upstream `cullingPlanes[6]` storage:
	//
	//   - countCullingPlanes is always 6 for Morrowind's main world camera
	//     (frustum planes only — no user clip planes are ever set up by
	//     the engine for CullShow). The proposed header's comment confirms
	//     this. Hardcoded here.
	//
	//   - cullingPlanePtrs[i] in the engine is a TArray of NiPlane* whose
	//     first 6 entries point at the inline cullingPlanes[6] mirror
	//     (NiCamera::UpdateWorldData copies plane data into the inline
	//     array via those pointers). Reading `&cullingPlanes[i]` directly
	//     yields the same plane data — confirmed by the proposed header's
	//     comment "the first 6 pointers target the inline cullingPlanes[6]".
	//
	//   - usedCullingPlanesBitfield[4] lives at offset 0x1C4, which is
	//     exactly `&cullingPlanes[6]` — the byte right past the 6-element
	//     inline array. We compute that address via offset arithmetic
	//     from `&cullingPlanes[0]` (legitimate base) + 6 * sizeof(Vector4)
	//     to avoid the past-end-deref UB of `&cullingPlanes[6].x`.
	//
	// If/when the upstream header adds these fields, swap the bodies for
	// `cam->countCullingPlanes` etc.; call sites stay unchanged.
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

	// Intel MSOC is allocated on the heap via Create()/Destroy(). We leak
	// this on process exit (same pattern as other long-lived MWSE globals).
	static ::MaskedOcclusionCulling* g_msoc = nullptr;
	// Double-buffer: g_msoc_prev holds the PREVIOUS frame's completed
	// mask and is what external consumers (MGE-XE via mwse_testOcclusion*)
	// read from. The plugin's own render pipeline rasterizes into g_msoc
	// each frame; at drain-complete we swap the pointers so the freshly-
	// built buffer becomes the next snapshot available to external queries
	// and the (now stale) ex-prev buffer becomes the write target for the
	// next frame's rasterization. Gives consumers a stable, race-free view:
	// they always see "the mask as of the end of the previous frame".
	static ::MaskedOcclusionCulling* g_msoc_prev = nullptr;
	// Matrix + NDC constants snapshotted at the same instant as the mask
	// swap — external queries project sphere centers through LAST frame's
	// view matrix, matching the depth data in g_msoc_prev.
	static float g_worldToClip_prev[16] = {};
	static float g_ndcRadiusX_prev = 0.0f;
	static float g_ndcRadiusY_prev = 0.0f;
	static float g_wGradMag_prev   = 0.0f;

	// Freshness gate. Wall-clock tick recorded at swap time; external
	// queries reject the snapshot if it's older than kSnapshotMaxAgeMs.
	// Guards against consuming a very stale mask when the plugin hasn't
	// rendered for a while (alt-tab, pause menu that skips the hook,
	// loading screen, cell-load stall). 0 here means "never captured".
	static unsigned long long g_snapshotTickMs = 0;
	static constexpr unsigned long long kSnapshotMaxAgeMs = 200;

	// External occluder injection (Phase B). Each PendingOccluder is a
	// self-contained copy of a consumer's submission: vertex data, index
	// data, optional model-to-world matrix, and MOC VertexLayout hints.
	// Plugin owns the memory; consumer can free their input buffers as
	// soon as mwse_addOccluder returns.
	//
	// The queue is populated on whatever thread the consumer calls us
	// from and drained on the render thread at the top of the per-frame
	// rasterization window, before native near-scene occluders rasterize.
	// g_pendingOccludersMutex covers both operations; contention is
	// trivial in practice (MGE-XE submits a handful of batches once per
	// frame, plugin drains once per frame).
	//
	// g_externalOccluderTrisQueued tracks the sum of triCount values
	// currently in the queue, used for budget enforcement against the
	// existing OcclusionOccluderMaxTriangles cap. External-first
	// rejection policy: if adding a batch would push past the cap, we
	// reject it and log, preserving the plugin's native-occluder budget.
	struct PendingOccluder {
		std::vector<float> verts;              // copy of consumer vtx data
		std::vector<std::uint32_t> tris;       // copy of consumer index data
		int stride;
		int offY;
		int offW;
		int vtxCount;
		int triCount;
		std::optional<std::array<float, 16>> modelMatrix;
		// When true: verts are already in clip space (homogeneous x,y,w);
		// drain path passes nullptr to MOC::RenderTriangles to skip the
		// world-to-clip multiply. Used by screen-space submitters such
		// as horizon-curtain terrain occluders.
		bool preTransformed = false;
	};

	static std::vector<PendingOccluder> g_pendingOccluders;
	static std::mutex g_pendingOccludersMutex;
	static int g_externalOccluderTrisQueued = 0;

	// Forward decl — definition sits with the other mask-pipeline
	// helpers further down the file; call site is inside the render
	// hook body which appears earlier.
	static void drainPendingOccluders();

	// World-to-clip matrix, transposed from NI's row-major M*v layout into
	// Intel's column-major v*M layout (consecutive memory = one column).
	// Intel reads clip.x = v.x*m[0] + v.y*m[4] + v.z*m[8] + m[12].
	// Refreshed each frame at the top-level entry of CullShow_detour.
	static float g_worldToClip[16];

	// Reusable buffers for rasterizeTriShape. Grown on demand, never shrunk;
	// MSOC is invoked single-threaded from the worldCamera pass so a single
	// module-level buffer is sufficient.
	static std::vector<float> g_occluderVerts;
	static std::vector<unsigned int> g_occluderIndices;

	// Per-frame matrix metrics used by testSphereVisible.
	// g_ndcRadiusX/Y: L2 norm of the world->clip.x / clip.y coefficients, so
	// the NDC half-extent of a sphere of radius r at clip-w cw is
	// r * g_ndcRadiusX / cw (resp. Y). Reads the leading 3 entries of each
	// row; the trailing entry is translation and doesn't contribute to the
	// gradient.
	// g_wGradMag: L2 norm of clip.w coefficients, so the worst-case clip-w
	// offset from the sphere center to its near surface is r * g_wGradMag.
	// For a standard perspective projection after a pure-rotation world->view
	// this is 1.0; computing it from the matrix handles any scaling.
	static float g_ndcRadiusX = 0.0f;
	static float g_ndcRadiusY = 0.0f;
	static float g_wGradMag = 0.0f;

	// _Claude_ DataHandler::worldLandscapeRoot captured at top-level frame
	// entry. Used by the drain loop to short-circuit occludee queries on
	// terrain patches (25v/32t, ~always visible under the camera). Plain
	// NiNode named "WorldLandscape" at DH+0x94, persists across cell
	// changes — so we could cache once, but re-reading per frame is a
	// single pointer load and stays robust to teardown / new-game reloads.
	// Null before the game world exists (load screen, menu).
	static NI::Node* g_worldLandscapeRoot = nullptr;

	// True only while TES3Game_static::renderMainScene (0x41C400) is on the
	// stack. Gates MSOC activation so every Click tree that fires outside
	// the per-frame main scene — load-screen splash, offscreen UI targets,
	// chargen race preview, MGE water reflection weather passes, etc. (see
	// rendering-engine-notes §1.6) — runs the pure engine-equivalent path.
	// Those callers wield cameras with setups we haven't validated for
	// occlusion culling, and the main-scene frame is the only place where
	// MSOC's cost is paid back.
	static bool g_inRenderMainScene = false;

	// _Claude_ Diagnostic: count of isTopLevel fires within the current
	// renderMainScene call. Reset to 0 at the top of renderMainScene_wrapper,
	// incremented each time CullShow_detour opens an isTopLevel block. If this
	// ends up > 1 per game-frame, it means the engine issues multiple main-
	// camera CullShow passes per frame (e.g. shadow-caster + main + 1st-person
	// subtree). Each one would run ClearBuffer + its own subtree, so the
	// LAST pass clobbers the previous passes' mask content — explaining why
	// the snapshot ends up with only whatever the last pass traverses
	// (typically: aggregateTerrain only, since that reads worldLandscapeRoot
	// unconditionally).
	static unsigned int g_isTopLevelFiresThisScene = 0;
	static unsigned int g_maxIsTopLevelFiresSession = 0;
	// _Claude_ Companion to g_isTopLevelFiresThisScene: counts the RAW
	// number of main-camera CullShow detour entries per renderMainScene,
	// BEFORE the alreadyBuiltThisScene guard. Lets us distinguish
	// "engine only fires one pass" (attempt=1) from "engine fires multiple
	// passes and the guard is suppressing them" (attempt>1). Both reset
	// at the top of renderMainScene_wrapper.
	static unsigned int g_mainCamCullShowAttemptsThisScene = 0;
	static unsigned int g_maxMainCamCullShowAttemptsSession = 0;

	// True only while the scene graph is being traversed for the worldCamera
	// main pass. CullShow_detour gates MSOC work on this so shadow-manager,
	// water-refraction, armCamera and other non-main Clicks inside
	// renderMainScene still run the pure engine-equivalent path.
	static bool g_msocActive = false;

	// _Claude_ True while the MSOC depth buffer reflects the vanilla main
	// scene's complete occluder set — set after drainPendingDisplays() in
	// the top-level cullShow_detour, cleared at the next frame's
	// ClearBuffer(). Out-of-tree consumers (MGE-XE) gate TestRect queries
	// on this: queries issued before the drain hit a partial/empty buffer
	// and would report false-VISIBLE. Never true outside the main-scene
	// worldCamera pass.
	static bool g_maskReady = false;

	// Per-frame diagnostics. Reset at the top of each worldCamera traversal.
	static uint64_t g_recursiveCalls = 0;
	static uint64_t g_recursiveAppCulled = 0;
	static uint64_t g_recursiveFrustumCulled = 0;
	static uint64_t g_rasterizedAsOccluder = 0;
	// _Claude_ Per-frame sum of outTri across successful rasterizeTriShape
	// calls. Used to size the threadpool queue (maxJobs): each submission
	// with outTri <= TRIS_PER_JOB (1024) is one queue slot, so this number
	// divided by TRIS_PER_JOB + rasterizedAsOccluder gives us the upper
	// bound on queue writes per frame.
	static uint64_t g_occluderTriangles = 0;
	static uint64_t g_skippedInside = 0;
	static uint64_t g_skippedThin = 0;
	static uint64_t g_skippedAlpha = 0;
	static uint64_t g_skippedStencil = 0;
	static uint64_t g_queryTested = 0;
	static uint64_t g_queryOccluded = 0;
	static uint64_t g_queryViewCulled = 0;
	// _Claude_ atomic so phase-1 drain workers can race on the
	// near-plane increment safely. Relaxed memory order is fine: the value
	// is only read for diagnostic logging, never for control flow.
	static std::atomic<uint64_t> g_queryNearClip{0};
	static uint64_t g_deferred = 0;
	// Diagnostic: NiNodes / non-geom AVObjects tested inline during
	// traversal (as opposed to deferred NiTriBasedGeom leaves). Helps
	// attribute queryOccluded between "whole subtree culled early"
	// (inline) and "leaf shape hidden behind fully-populated buffer"
	// (drain-phase).
	static uint64_t g_inlineTested = 0;
	// Phase 2.2 gate counters. Non-zero at default config means a
	// gate is mis-placed (defaults are tuned to keep all three at 0
	// on typical scenes).
	static uint64_t g_skippedTriCount = 0;
	static uint64_t g_skippedTesteeTiny = 0;
	static uint64_t g_skippedSceneGate = 0;
	// _Claude_ Frames where a menu was open and we skipped MSOC entirely.
	// Cumulative across the session (not reset per top-level frame, since
	// these are exactly the frames where the per-frame reset doesn't run).
	// Incremented per top-level CullShow_detour entry while flagMenuMode
	// is set, so on a busy menu pass it can grow by several per actual
	// game frame — the rate matters less than the fact that it's non-zero.
	static uint64_t g_skippedMenuMode = 0;
	// _Claude_ Deferred terrain leaves bypassed from TestRect. Ground
	// patches sit under the camera and are nearly always visible;
	// skipping the query saves depth-buffer bandwidth + ~1 µs/leaf.
	static uint64_t g_skippedTerrain = 0;
	// _Claude_ Aggregate terrain occluder counters. Each visible Land node
	// produces one combined RenderTriangles submission; triCount is the
	// total sum across Lands that frame. Zero unless
	// Configuration::OcclusionAggregateTerrain is on.
	static uint64_t g_aggregateTerrainLands = 0;
	static uint64_t g_aggregateTerrainTris = 0;
	// _Claude_ Wall-clock time spent in rasterizeAggregateTerrain per
	// frame (tree walk + vertex transform + RenderTriangles submit).
	// Includes the async enqueue path when the threadpool is active —
	// actual worker rasterisation happens in parallel and is not counted.
	static uint64_t g_aggregateTerrainUs = 0;

	// ============================================================
	// Cache types & globals (land / drain / light)
	// ============================================================

	// _Claude_ Per-Land merged occluder cache. Key = per-Land NiNode
	// pointer under the WorldLandscape root. Value = world-space vertex
	// buffer + index buffer, prebuilt on first sight. Scene-graph walk
	// and vertex transform become O(1) hits after that. Entries are
	// mark-and-sweep evicted each frame: any key not touched this frame
	// is stale and dropped. Eviction is safe because previous-frame async
	// queues are consumed by ClearBuffer()/implicit Flush before the
	// next aggregate pass runs.
	//
	// _Claude_ Lifetime: nodePtr (NI::Pointer) holds a refcount on the
	// NiNode key so the engine cannot tear it down while it lives in
	// the map. Without this, an engine-side cell unload between two
	// frames could free the NiNode and leave us with a dangling key
	// to dereference on the next pass. Sweep eviction releases the
	// ref; if we were the last owner the NiNode is destroyed then.
	struct LandCacheEntry {
		NI::Pointer<NI::Node> nodePtr;
		std::vector<float> verts;
		std::vector<unsigned int> indices;
		unsigned int triCount = 0;
		bool seen = false;
		// _Claude_ Records the OcclusionTerrainResolution value the entry
		// was built under. On cache hit, a mismatch forces a rebuild so
		// MCM dropdown changes take effect lazily without a global cache
		// flush. Sentinel 0xff means "uninitialised" (any real value
		// triggers rebuild).
		uint8_t builtForResolution = 0xff;
	};
	static std::unordered_map<NI::Node*, LandCacheEntry> g_landCache;
	static uint64_t g_landCacheHits = 0;
	static uint64_t g_landCacheMisses = 0;
	static uint64_t g_landCacheEvictions = 0;

	// _Claude_ Temporal coherence cache for drain-phase TestRect results.
	// Keyed by NI::AVObject* (the deferred shape pointer). Each entry
	// records the last TestRect verdict and the frame it was taken at.
	// Configuration::OcclusionTemporalCoherenceFrames (N) controls reuse:
	// N=0 disables the cache entirely; N>0 reuses a fresh entry for up
	// to N intervening frames before re-querying.
	//
	// _Claude_ We cache only OCCLUDED verdicts. VISIBLE and VIEW_CULLED
	// fall through to display() anyway, which dereferences the live
	// p.shape — caching them would add stale-pointer surface area while
	// gaining nothing on the hit path. OCCLUDED is the only verdict
	// where the hit lets us skip both TestRect and display().
	//
	// Move detection: we compare worldBoundOrigin + worldBoundRadius
	// against the snapshot taken at query time. Any meaningful change
	// (animated shape, physics movement) invalidates the entry.
	//
	// _Claude_ Lifetime: shapePtr (NI::Pointer) holds a refcount on the
	// AVObject key so the engine cannot free it while we hold a cache
	// entry. Without this, raising OcclusionTemporalCoherenceFrames
	// widened the window in which a destroyed-then-prune-pending entry
	// could be looked up against a recycled pointer (and crash on the
	// dereference if the page got unmapped). Age-prune at 2*N frames
	// bounds memory; on prune the NI::Pointer destructor releases the
	// ref and the object is freed if we were the last owner.
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

	// _Claude_ Per-light occlusion cache for the updateLights hook (gated
	// by Configuration::OcclusionCullLights). Keyed by NI::Light*. Entry
	// records the last frame the light was queried, the most recent MSOC
	// verdict, consecutive-occluded frame counter for hysteresis, and
	// the worldBound snapshot used for move detection.
	//
	// _Claude_ Lifetime: lightPtr (NI::Pointer) holds a refcount on the
	// NiLight key so the engine cannot free it mid-cell while our entry
	// lives. Cell-change wipe (g_lightCullCache.clear) releases all refs
	// at once. Without this, a light freed mid-cell whose address was
	// reused by a different allocation could be looked up and matched
	// against a stale entry — at best a wrong cull verdict, at worst a
	// crash on the dereference.
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

	// _Claude_ Observer callbacks. Populated at startup by external
	// consumers (e.g. MGE-XE) that want to tap into the live
	// renderer-iterated NiLight list without re-patching 0x6BB7D4.
	// Iterated lock-free from the render thread inside
	// shouldLightBeEnabled; see header for the full contract.
	static std::vector<LightObservedCallback> g_lightObservers;

	// ============================================================
	// Frame counters & diagnostic state
	// ============================================================

	// File-scope frame counter. Needed by the drain loop to decide
	// cache freshness; incremented once per top-level frame.
	static uint32_t g_frameCounter = 0;
	// _Claude_ Cached currentCell pointer across frames. When it changes
	// (cell transition: ext→int, int→ext, int→int, ext→ext chunk swap),
	// we wipe both g_landCache and g_drainCache. Without this, destroyed
	// NI::Node* / NI::AVObject* pointers can be reused by new allocations
	// in the fresh cell, and a stale cache entry might dereference freed
	// memory. Pointer-compare is cheap; full clear happens at most once
	// per cell load.
	static TES3::Cell* g_lastCell = nullptr;
	static uint64_t g_cellChanges = 0;
	// Phase 2.2 timers (microseconds). steady_clock is QPC-backed
	// on MSVC so no QPC wrapper is needed. Rasterize accumulates
	// each RenderTriangles call; drain wraps the whole drain body.
	static uint64_t g_rasterizeTimeUs = 0;
	static uint64_t g_drainPhaseTimeUs = 0;
	// _Claude_ Wall-clock time of phase 1 (the classify pass that
	// produces every shape's verdict). Bracketed on the main thread
	// around either the serial classifyDrainRange call or the
	// parallel-barrier dispatch — single writer in both modes, no race.
	// Replaces the per-call testRectUs/testRectNs sums that under-counted
	// in parallel mode (CPU time across workers ≠ wall time) and the
	// dedicated parallelDrainUs that duplicated this measurement on the
	// parallel side. One number, one meaning, comparable across modes.
	static uint64_t g_classifyUs = 0;
	static uint64_t g_drainDisplayUs = 0;
	// Phase 3.2: time spent in threadpool Flush() barrier before drain.
	// Only populated when async mode is active; zero otherwise.
	static uint64_t g_asyncFlushTimeUs = 0;
	// _Claude_ Freeze diagnostic: time spent inside the threadpool's
	// WakeThreads() spin (`while (mNumSuspendedThreads < mNumThreads)
	// yield()` in Intel's CullingThreadpool). Should be ~0 every frame;
	// any non-trivial value here means a worker didn't reach its
	// suspended state from the previous frame's SuspendThreads(), and a
	// permanently-stuck worker would peg this at infinity (i.e. freeze).
	// Reset per frame; max-seen kept for the lifetime so a single
	// outlier shows up in the next periodic log even if the average
	// stays low.
	static uint64_t g_wakeThreadsTimeUs = 0;
	static uint64_t g_maxWakeThreadsUsSession = 0;
	// _Claude_ Recursion diagnostic. CullShow descends NiNode children
	// recursively via display(); a runaway tree (cycle, broken sentinel)
	// would manifest as ever-growing depth before a stack overflow.
	// g_callDepth is incremented at CullShow_detour entry / decremented
	// at exit, so a debugger attached during a freeze can read it
	// directly. g_maxCallDepthThisFrame tracks the per-frame peak;
	// g_maxCallDepthSession is the lifetime peak for the periodic log.
	static uint32_t g_callDepth = 0;
	static uint32_t g_maxCallDepthThisFrame = 0;
	static uint32_t g_maxCallDepthSession = 0;

	// _Claude_ RAII guard for the recursion depth counter. Incremented at
	// CullShow_detour entry, decremented on every return path (including
	// future early-exits) via destructor. Lifted out of the detour body
	// for readability; state lives in g_callDepth / g_maxCallDepthThisFrame
	// above. Used exclusively by CullShow_detour.
	struct CallDepthGuard {
		CallDepthGuard() {
			++g_callDepth;
			if (g_callDepth > g_maxCallDepthThisFrame) {
				g_maxCallDepthThisFrame = g_callDepth;
			}
		}
		~CallDepthGuard() { --g_callDepth; }
	};
	// _Claude_ Last-checkpoint marker. Updated as the top-level frame
	// passes each major stage, so a debugger attached to a frozen
	// process can identify which stage the main thread is stuck in
	// without needing symbols. Numbering is intentionally stable
	// (don't renumber): 0 idle, 1 entered top-level, 2 wakeThreads,
	// 3 clearBuffer, 4 cellWipe, 5 ageprune, 6 uploadCamera,
	// 7 setMatrix, 8 aggTerrain, 9 cullShowBody, 10 flush, 11 drain,
	// 12 suspendThreads, 13 exited top-level cleanly,
	// 14 drainClassify (parallel phase 1 in flight),
	// 15 drainAction (phase 2 in flight).
	static volatile uint32_t g_lastStage = 0;

	// _Claude_ Freeze-forensics watchdog. A background thread polls
	// the checkpoint globals every ~250ms and writes an atomic
	// snapshot to disk (trunc mode, so the file is always the latest
	// state — no parse needed). When the game hard-freezes and
	// Windows kills it, the file left behind shows which stage the
	// main thread was in, the recursion depth at freeze time, and
	// how long it has been since the last clean frame end. Without
	// this, a freeze with no log tail and no crash dump is opaque.
	//
	// g_lastFrameEndTimeMs is published by the main thread at the
	// end of each top-level frame (after g_lastStage = 13). The
	// watchdog reads it to compute "time since last clean exit".
	// Using steady_clock::now().time_since_epoch().count() in ms;
	// std::atomic<uint64_t> gives cheap wait-free publish on x86.
	static std::atomic<uint64_t> g_lastFrameEndTimeMs{0};
	// _Claude_ Set true on orderly shutdown. There's no such path for
	// MSOC today (we detach the watchdog), but if one is added later
	// the thread will observe this and exit its sleep loop.
	static std::atomic<bool> g_watchdogStop{false};

	// _Claude_ Human-readable stage label so the forensics file can
	// be read without cross-referencing the numeric table above.
	// Kept in sync with the g_lastStage numbering comment.
	static const char* stageName(uint32_t s) {
		switch (s) {
			case 0:  return "idle";
			case 1:  return "entered";
			case 2:  return "wakeThreads";
			case 3:  return "clearBuffer";
			case 4:  return "cellWipe";
			case 5:  return "agePrune";
			case 6:  return "uploadCamera";
			case 7:  return "setMatrix";
			case 8:  return "aggTerrain";
			case 9:  return "cullShowBody";
			case 10: return "flush";
			case 11: return "drain";
			case 12: return "suspendThreads";
			case 13: return "exitedClean";
			case 14: return "drainClassify";
			case 15: return "drainAction";
			default: return "unknown";
		}
	}

	// Forward decl — body defined after Phase 3.2 globals so it can
	// reference g_threadpool / g_asyncThisFrame.
	static void watchdogTick();

	// Forward decl — body defined alongside installPatches() so it can
	// share the lazy create/destroy helpers. Called from CullShow_detour
	// at the safe top-of-frame point.
	static bool ensureMSOCResourcesMatchConfig();

	// ============================================================
	// Threadpool state & per-frame config cache
	// ============================================================

	// Phase 3.2 async-rasterization state.
	//
	// g_threadpool is created once in installPatches and lives for the
	// process. Runtime-toggle of OcclusionAsyncOccluders works without a
	// restart because the threadpool's lifecycle (Wake/Suspend) is driven
	// per-frame off that flag — a suspended threadpool sleeps at ~0% CPU.
	//
	// g_asyncThisFrame latches the async flag at top-level entry so any
	// mid-frame Configuration edits from Lua don't mix sync and async
	// submissions within one traversal.
	//
	// The two arena vectors own the per-submission triangle data until
	// Flush() retires the corresponding render job. CullingThreadpool
	// requires caller-owned buffers to stay unchanged between submission
	// and completion; we enforce that by moving g_occluderVerts /
	// g_occluderIndices into the arena per submission (next rasterize call
	// allocates fresh vectors). Cleared at top-level entry, after
	// ClearBuffer's implicit Flush has retired the previous frame's jobs.
	static ::CullingThreadpool* g_threadpool = nullptr;
	static bool g_asyncThisFrame = false;
	static std::vector<std::vector<float>> g_asyncOccluderVerts;
	static std::vector<std::vector<unsigned int>> g_asyncOccluderIndices;

	// _Claude_ Scene-type-resolved occluder thresholds. Written once per
	// top-level CullShow_detour entry after isInterior is known, then
	// read by cullShowBody / rasterizeTriShape without branching on
	// scene type per shape. The four source knobs
	// (RadiusMin/Max/MinDimension/InsideOccluderMargin) have separate
	// interior and exterior Configuration:: fields; this collapses the
	// pair-selection into a single value for the hot path.
	static float g_occluderRadiusMinEffective     = 0.0f;
	static float g_occluderRadiusMaxEffective     = 0.0f;
	static float g_occluderMinDimensionEffective  = 0.0f;
	static float g_insideOccluderMarginEffective  = 0.0f;

	// _Claude_ Per-frame cache of loop-invariant Configuration:: reads.
	// The hot paths (rasterizeTriShape, cullShowBody recursion, drain
	// phase 2, light callback, ScopedUsAccumulator ctor) bracket opaque
	// cross-TU calls into MaskedOcclusionCulling and NI::display. Because
	// Configuration:: statics have external linkage, the optimiser assumes
	// those calls may mutate them and reloads after each call; caching
	// into these internal-linkage (static) copies lets the compiler hoist
	// the reads out of hot loops and register-allocate them across the
	// opaque calls. Contract: MCM never rewrites Configuration:: mid-frame
	// (Lua side only pushes between frames), so a single refresh at the
	// top of each active top-level CullShow call is sound. Defaults match
	// Config.cpp so reads before the first refresh (plugin init, light
	// callback fired on a non-active frame) still see consistent values.
	static float        g_depthSlackWorldUnitsEffective      = 128.0f;
	static unsigned int g_occluderMaxTrianglesEffective      = 4096;
	static float        g_occludeeMinRadiusEffective         = 1.0f;
	static bool         g_skipTerrainOccludeesEffective      = true;
	static bool         g_aggregateTerrainEffective          = true;
	static unsigned int g_terrainResolutionEffective         = 1;
	static unsigned int g_temporalCoherenceFramesEffective   = 4;
	static bool         g_cullLightsEffective                = true;
	static unsigned int g_lightCullHysteresisFramesEffective = 3;
	static bool         g_tintOccluderEffective              = false;
	static bool         g_tintOccludedEffective              = false;
	static bool         g_tintTestedEffective                = false;
	static bool         g_logEnabledEffective                = false;

	// _Claude_ Drain-parallel worker pool — DISABLED. The serial drain
	// is the only active path; the infrastructure below is preserved as
	// commented-out reference for a future re-enable. If you bring this
	// back, also restore: Configuration::OcclusionParallelDrain (Config.h/
	// .cpp + Lua config/mcm), drainWorkerMain, the worker-pool spawn in
	// installPatches, and the runParallel branch in drainPendingDisplays.
	//
	// Sizing rationale kept for reference: ~840 TestRect queries/frame in
	// dense scenes, 2-way split (hiZ cache traffic dominates beyond that),
	// C++17 condvar/mutex fallback for std::counting_semaphore, workers
	// parked on start cv at ~0% CPU when idle.
	//
	// constexpr unsigned int kDrainWorkerCount = 2;
	// constexpr size_t kParallelDrainMin = 100;
	//
	// struct DrainWorkerRange {
	// 	size_t lo;
	// 	size_t hi;
	// };
	//
	// static std::jthread g_drainWorkers[kDrainWorkerCount];
	// static DrainWorkerRange g_drainRanges[kDrainWorkerCount];
	//
	// static std::mutex g_drainMtx;
	// static std::condition_variable_any g_drainStartCv;
	// static std::condition_variable g_drainDoneCv;
	// static unsigned int g_drainStartTickets = 0;
	// static unsigned int g_drainDoneTickets = 0;

	// ============================================================
	// Watchdog implementation
	// ============================================================

	// _Claude_ Watchdog loop (body — forward-declared higher up).
	// 250ms cadence is a compromise: fast enough to catch a freeze
	// before the user alt-tabs out, slow enough not to thrash the
	// disk (one small trunc-write per tick). File lives in CWD
	// (= modlist root under MO2), alongside MWSE.log, so the user
	// can attach both when reporting.
	static void watchdogTick() {
		using namespace std::chrono;
		while (!g_watchdogStop.load(std::memory_order_relaxed)) {
			std::this_thread::sleep_for(milliseconds(250));

			const uint64_t nowMs = static_cast<uint64_t>(
				duration_cast<milliseconds>(
					steady_clock::now().time_since_epoch()).count());
			const uint64_t lastEnd = g_lastFrameEndTimeMs.load(
				std::memory_order_relaxed);
			const uint64_t sinceFrameEndMs = lastEnd == 0
				? 0 : (nowMs - lastEnd);

			// Snapshot volatile/racy globals. Reads may tear on
			// 64-bit counters but torn values still bracket the
			// real value well enough for post-mortem.
			const uint32_t stage = g_lastStage;
			const uint32_t depth = g_callDepth;
			const uint32_t maxDepth = g_maxCallDepthSession;
			const uint64_t frame = g_frameCounter;
			const bool asyncNow = g_asyncThisFrame;
			const bool tpAlive = g_threadpool != nullptr;

			// Trunc mode: each tick overwrites — the file is
			// always the freshest snapshot. If the process
			// hard-freezes, whatever the watchdog last wrote is
			// what Windows has on disk at kill time.
			std::ofstream f("MSOC.forensics.txt",
				std::ios::out | std::ios::trunc);
			if (!f.is_open()) continue;
			f << "frame=" << frame
			  << " stage=" << stage << "(" << stageName(stage) << ")"
			  << " depth=" << depth
			  << " maxDepthSess=" << maxDepth
			  << " sinceFrameEndMs=" << sinceFrameEndMs
			  << " asyncThisFrame=" << (asyncNow ? 1 : 0)
			  << " threadpoolAlive=" << (tpAlive ? 1 : 0)
			  << " nowMs=" << nowMs
			  << " lastEndMs=" << lastEnd
			  << std::endl;
		}
	}

	// ============================================================
	// Deferred-display queue & timing helpers
	// ============================================================

	// _Claude_ Gated on either log channel — when both are off the timer is
	// a no-op (nullptr target, no clock reads), saving the QPC overhead in
	// production. Read-once at construction so a mid-scope MCM toggle can't
	// flip the destructor onto a still-zero `start`.
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

	// Deferred-display queue for small NiTriShape leaves. Small shapes are
	// queued during the main traversal and drained *after* all occluder
	// rasterisation is complete, so their MSOC visibility test runs
	// against the fully-populated depth buffer instead of whatever
	// occluders happened to precede them in scene-graph order. Avoids the
	// OpenMW two-pass cost (single traversal) and still gives big shapes
	// priority for rasterisation.
	//
	// Only NiTriShape leaves are deferred. NiNodes must stay inline so
	// their subtree keeps contributing occluders during the main pass;
	// other leaf types (Particles, etc.) are rare enough to keep inline.
	struct PendingDisplay {
		NI::AVObject* shape;
		NI::Camera* camera;
		// _Claude_ True when the main pass rasterised this shape as an
		// occluder. The drain loop uses this to skip re-tinting so the
		// occluder (yellow) classification is not overwritten by the
		// drain's Tested (green) / Occluded (red) tints. Draw-side
		// TestRect still runs — this only gates the debug tint write.
		bool rasterisedAsOccluder;
	};
	static std::vector<PendingDisplay> g_pendingDisplays;

	// _Claude_ Drain phase-1 verdict slots. Phase 1 (classifyDrainRange)
	// fills g_drainSlots[i] with a verdict per g_pendingDisplays[i].
	// Phase 2 (the serial action pass in drainPendingDisplays) walks the
	// slots in order and applies counter increments, cache writes, tints,
	// and display() calls. Splitting the work this way keeps phase 1
	// read-only on shared state, which is the prerequisite for moving
	// it onto worker threads in a later commit.
	enum class DrainVerdict : uint8_t {
		Visible,        // TestRect returned VISIBLE; call display()
		Occluded,       // TestRect returned OCCLUDED; skip display (or tint+display in debug)
		ViewCulled,     // rect collapsed; treat like Visible for display()
		SkipTerrain,    // bypassed TestRect (terrain descendant); call display()
		SkipTiny,       // bypassed TestRect (radius < threshold); call display()
		CachedOccluded, // hit g_drainCache; verdict is OCCLUDED (we only cache OCCLUDED)
	};

	struct DrainSlot {
		DrainVerdict verdict;
		// _Claude_ True only when testSphereVisible actually ran (fresh
		// Visible/Occluded/ViewCulled verdicts). False for the Skip* paths
		// and for CachedOccluded. Phase 2 uses this to gate counter
		// increments that fire only on the !reused branch in the
		// pre-refactor serial loop.
		bool ranTestRect;
	};

	static std::vector<DrainSlot> g_drainSlots;

	// ============================================================
	// Debug tinting & occluder property classification
	// ============================================================

	// Debug tint overlay. When any of the three DebugOcclusionTint* flags
	// in Configuration are set, classified leaves render in a distinct hue:
	//   red    — OCCLUDED (would have been culled; kept visible for compare)
	//   green  — test passed (survived the MSOC query)
	//   yellow — rasterised as an occluder (driving the depth buffer)
	//
	// Lifetime model (persistent-clone): the first time we tint a shape we
	// clone its effective MaterialProperty (ancestor-inherited or own),
	// detach any own material, and attach the clone. The clone stays
	// attached for the shape's lifetime; subsequent frames only overwrite
	// emissive + bump revisionID. This avoids the per-frame allocator churn
	// that caused a crash earlier (~1.7k new/delete per frame × hundreds of
	// frames drove the DX8 renderer's per-material state cache stale —
	// freed clones' addresses got recycled and the cache returned dangling
	// D3D state).
	//
	// At end of each main-scene frame every tracked clone is reset to the
	// source material's current emissive, so the default render state shows
	// the shape's real color; the next frame's classifications re-apply the
	// tint. Shapes that never get tinted never allocate.
	//
	// Morrowind's art shares NiMaterialProperty instances two ways:
	//   - via inheritance (a parent NiNode's material applies to every
	//     descendant without their own),
	//   - via NIF streamable dedup (two leaf shapes point at the same own
	//     MaterialProperty object),
	// and mutating either in place stains every sibling. Cloning avoids
	// both. Only NiTriBasedGeom leaves are tinted — tinting a NiNode would
	// propagate via inheritance to every child in its subtree.
	static const NI::Color kTintOccluded(1.0f, 0.0f, 0.0f);
	static const NI::Color kTintTested(0.0f, 1.0f, 0.0f);
	static const NI::Color kTintOccluder(1.0f, 1.0f, 0.0f);

	// NiMaterialProperty vtable address (see NIDefines.h VTableAddress
	// block). Property::Property() sets the base Property vtable; every
	// concrete subclass overwrites with its own (see AlphaProperty ctor
	// in NIProperty.cpp). MWSE exposes no public NiMaterialProperty ctor
	// so we open-code the same pattern here.
	constexpr uintptr_t kNiMaterialPropertyVTable = 0x75036C;

	struct TintClone {
		// Holds the shape alive so the map key + the clone (attached to
		// the shape's property list) stay valid as long as our map entry
		// exists. Without this, cell-unload would delete the shape and
		// leave us with a dangling key + freed clone.
		NI::Pointer<NI::AVObject> shape;
		// Holds the source material alive so we can read its current
		// emissive during end-of-frame reset. For own-material shapes we
		// detached it from the list; this Pointer is now the sole owner.
		// For purely-inheriting shapes source is the ancestor's material
		// (still alive via ancestor); keeping a Pointer is cheap insurance.
		// Null only if the shape had no reachable MaterialProperty at all.
		NI::Pointer<NI::MaterialProperty> source;
		// Owned by the shape's property list (attachProperty adds the ref).
		// Safe to raw-ptr because the shape Pointer above keeps the list —
		// and thus this clone — alive for the entry's lifetime.
		NI::MaterialProperty* clone;
	};
	static std::unordered_map<NI::AVObject*, TintClone> g_tintClones;

	// Walk from obj up through ancestors, returning the first
	// MaterialProperty reached. Matches the engine's property-inheritance
	// rule: a shape with no own MaterialProperty uses its ancestor's.
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

	// _Claude_ Single-pass property classifier for occluder filtering.
	// Walks ancestor->root once, checking each property list for the
	// first-of-type-wins NiAlphaProperty and NiStencilProperty in the
	// same scan. Replaced two separate helpers (hasTransparency +
	// hasStencil) that each walked the same chain end to end; combining
	// halves the ancestor-walk cost for the common "no stencil anywhere"
	// case, which hits every shape at ~50-100 calls/frame in typical
	// scenes.
	//
	// Why filter these as occluders at all:
	//   alpha   — blended / alpha-tested shapes are visually transparent
	//             in parts (fences, banners, grates, vines, tree leaves,
	//             window glass). Rasterising one as a solid occluder
	//             fills the depth buffer across the full quad footprint
	//             and falsely occludes whatever's behind the "holes".
	//   stencil — stencil-enabled shapes only fill where the test
	//             passes (shadow volumes, reflection clip masks, UI
	//             cutouts). Same phantom-occlusion hazard.
	// Both still participate as occludees in the visibility query pass;
	// only the occluder rasterisation pass skips them.
	//
	// Nearest-ancestor-wins applies INDEPENDENTLY per property type: a
	// shape's own stencil takes precedence over a grandparent's stencil,
	// but its own alpha is unrelated and may live on a different
	// ancestor. Each type's first hit latches and subsequent hits of
	// that type are ignored.
	struct OccluderPropertyFlags {
		bool alpha;    // effective NiAlphaProperty has blend or test enabled
		bool stencil;  // effective NiStencilProperty has enabled == true
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

	// _Claude_ Ancestor-walk: true iff obj sits under DataHandler's
	// worldLandscapeRoot. The terrain tree is shallow — leaf trishape →
	// subcell NiNode → per-Land NiNode → WorldLandscape root is exactly
	// four levels, so terrain hits return at iteration 4. Non-terrain
	// shapes run the loop to scene root (~5–8 levels) and return false;
	// that's the cost we pay for the terrain skip, measured in cache-hot
	// pointer chases so it stays under ~1 µs even at 500 deferred leaves.
	static bool isLandscapeDescendant(NI::AVObject* obj, NI::Node* root) {
		if (!root) return false;
		for (NI::AVObject* cur = obj; cur; cur = cur->parentNode) {
			if (cur == root) return true;
		}
		return false;
	}

	// Allocate a standalone NiMaterialProperty seeded from source (if any)
	// and force the tint into ambient, diffuse, and emissive. All three
	// channels are set because a sibling NiVertexColorProperty may replace
	// one of them from baked per-vertex colors (SOURCE_IGNORE /
	// SOURCE_EMISSIVE / SOURCE_AMBIENT_DIFFUSE); with all three tinted at
	// most one is overridden and the rest carry the color. World statics
	// in Morrowind use SOURCE_EMISSIVE so an emissive-only tint is
	// invisible on them while skinned actors (IGNORE/AMBIENT_DIFFUSE) show
	// it — the asymmetry we originally saw.
	static NI::MaterialProperty* cloneMaterialProperty(NI::MaterialProperty* source, const NI::Color& tint) {
		// _Claude_ Returns a NiMaterialProperty with refCount=1 ("caller-
		// owned" convention, matching the engine's own NiObject::Clone
		// semantics). attachProperty's wrapper at 0x405840 has a temp-
		// Pointer cycle around AddHead (claim+release) that net-zeros
		// the input refcount; if we returned refCount=0, the cycle
		// would be 0→1→0 → DELETE, leaving the property list with a
		// dangling pointer that the heap free-list keeps readable for
		// a while before reuse triggers a delayed crash.
		NI::MaterialProperty* mat;
		if (source) {
			// Engine's NiObject::Clone (0x6E9910) does a stream-based
			// deep copy and returns refCount=1. Preferred path because
			// it picks up every inherited field (incl. anything we
			// don't enumerate manually below).
			mat = static_cast<NI::MaterialProperty*>(source->createClone());
		}
		else {
			// No source to clone — fresh allocation. NiObject::ctor sets
			// refCount=0; bump to 1 to match the createClone path.
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
		// _Claude_ Required: attachProperty only mutates the raw list; the
		// renderer reads from a per-Geometry effective-material cache that
		// updateProperties() rebuilds. Without this call the clone sits in
		// the list but draws keep using the previously-cached material —
		// tints appear only when something else triggers a property update.
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

	// End-of-frame tint reset: restore each tracked source's ambient,
	// diffuse, and emissive into its clone so shapes that weren't re-
	// tinted next frame render with their real color. All three channels
	// must be reset because tintEmissive sets all three (see
	// cloneMaterialProperty for the NiVertexColorProperty SOURCE rationale)
	// — restoring only emissive leaves IGNORE/AMBIENT_DIFFUSE shapes
	// permanently green. Pointer-stable (no detach/re-attach), but
	// updateProperties() is required so the per-Geometry effective-material
	// cache the DX8 renderer reads picks up the new colors.
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

	// Fill g_worldToClip by transposing NI::Camera::worldToCamera into
	// Intel's column-major v*M layout. NI stores row-major M*v — clip[r] =
	// Σ_c ni[r*4+c]*v[c] (verified against NiCamera::ScreenSpaceBoundBound
	// at 0x6CCEC0). Intel's TransformVerts computes out[r] =
	// Σ_c v[c]*mtx[r*4+c] with mtx indexed column-major as mtx[col*4+row],
	// so the effective formula is out[r] = Σ_c v[c]*mtx[c*4+r]. For Intel
	// to produce the engine's clip coords we need mtx[c*4+r] = ni[r*4+c].
	static void uploadCameraTransform(NI::Camera* cam) {
		const float* ni = reinterpret_cast<const float*>(&cam->worldToCamera);
		for (int row = 0; row < 4; ++row) {
			for (int col = 0; col < 4; ++col) {
				g_worldToClip[col * 4 + row] = ni[row * 4 + col];
			}
		}

		// Per-frame metrics for sphere projection. g_ndcRadiusX is the
		// operator norm of the x-row coefficients (ignoring translation):
		// an upper bound on |d(x_clip)/d(pos)| used to scale the sphere
		// radius in clip space. Similarly for y and w. After the transpose
		// into Intel's column-major layout, NI's row-r coefficients live at
		// strided positions m[0+r], m[4+r], m[8+r] (column 0..2, row r).
		const float* m = g_worldToClip;
		g_ndcRadiusX = std::sqrt(m[0] * m[0] + m[4] * m[4] + m[8] * m[8]);
		g_ndcRadiusY = std::sqrt(m[1] * m[1] + m[5] * m[5] + m[9] * m[9]);
		g_wGradMag = std::sqrt(m[3] * m[3] + m[7] * m[7] + m[11] * m[11]);
	}

	// Project a world-space point using the cached Intel-layout matrix.
	// Produces clip-space (x, y, w); z is unused by Intel's rasterizer.
	// Uses the same column-major v*M formula as Intel's TransformVerts so
	// sphere screen rects stay aligned with rasterised occluder depth.
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

	// Direct sphere -> NDC-rect + wmin projection, then Intel's TestRect.
	// Projects only the sphere center and derives the NDC half-extent and
	// near-surface clip-w from per-frame matrix metrics (see
	// uploadCameraTransform). Tighter than an 8-corner world AABB projection
	// by ~sqrt(3) and stable under camera rotation — neither the NDC radius
	// nor wMin depend on how the sphere's bbox aligns with camera axes, so
	// small-mesh queries don't flicker across TestRect's hiZ thresholds.
	//
	// If the sphere's near surface straddles the near plane we bail as
	// VISIBLE: TestRect can't project a straddling rect safely.
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

	// _Claude_ Registry management. Dedup on register so double-registration
	// (e.g. DLL reload) doesn't cause duplicate observer dispatch. Called
	// at startup; not on any hot path.
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

	// _Claude_ Called from the naked hook at 0x6bb7d4 (replaces the original
	// `mov al, [ebx+0x90]` enabled-read inside NiDX8LightManager::updateLights).
	// Returns the effective enabled byte: the light's real enabled flag
	// unless Configuration::OcclusionCullLights is on AND the light's
	// worldBound sphere has been occluded for at least
	// Configuration::OcclusionLightCullHysteresisFrames consecutive frames,
	// in which case we return 0 (disabled). This pre-empts the D3D8
	// SetLight/LightEnable pair without touching the scene graph.
	//
	// Hysteresis avoids flicker on light/dark boundary crossings: VISIBLE
	// verdicts immediately reset the counter and unlatch the cull, so
	// lights that swing back into view reappear the same frame. OCCLUDED
	// verdicts only latch a cull after N frames in a row, so a transient
	// misfire (e.g. a frame where the depth buffer briefly says the sphere
	// is hidden) doesn't darken the scene for even one frame. Default N
	// is 3 (~50 ms at 60 FPS — below the perceptual flicker threshold but
	// short enough to pay back on "turn away from a lamp row" motions).
	// The counter is uint8_t (max 255), so any slider value above that
	// effectively disables the latch.
	extern "C" bool __cdecl shouldLightBeEnabled(NI::Light* light) {
		// _Claude_ Observer dispatch runs first, unconditionally. Observers
		// see every iterated NiLight — disabled ones, cache-hit path, MSOC
		// backend missing, whatever. Empty-vector path is one predicted
		// branch; single-observer path is one indirect call. See header
		// for the contract observers must honour.
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

		// Identity fingerprints. Captured on the NI::AVObject base
		// (Light inherits AVObject) so worldBound comparisons are
		// offsetof-stable across subclasses (NiPoint/Spot/Ambient/
		// DirectionalLight all share this layout).
		NI::AVObject* const av = light;
		const float cx = av->worldBoundOrigin.x;
		const float cy = av->worldBoundOrigin.y;
		const float cz = av->worldBoundOrigin.z;
		const float cr = av->worldBoundRadius;

		// Degenerate bound: no sphere to test. Pass through as enabled
		// (we can't make a reliable verdict, and a zero-radius light
		// is likely an ambient or freshly-created instance whose
		// worldBound hasn't propagated yet).
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

		// Fresh insert — starting state is "not culled yet," even
		// on an occluded first verdict, so the hysteresis counter
		// has to ramp up before the first darkening.
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

	// _Claude_ Naked trampoline installed at 0x6bb7d4 in place of
	// `mov al, [ebx+NiLight.super.enabled]` (6 bytes). Writes `call rel32`
	// + NOP across those 6 bytes; we enter here with ebx = NiLight* and
	// must return with AL holding the effective enabled byte. The
	// instruction after our patch (`mov [esp+...], ebx` at 0x6bb7da,
	// then `test al, al` at 0x6bb7de) only cares about AL; upper EAX
	// bits are free to trash, same as the original mov-al semantics.
	__declspec(naked) void updateLights_enabledRead_hook() {
		__asm {
			// Preserve volatile regs we clobber. ebx/esi/edi/ebp are
			// non-volatile and the C function honours that, but ecx/edx
			// are nominally caller-save and the surrounding loop doesn't
			// save them before our site, so we guard them ourselves.
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

	// Rasterise a shape's actual triangles (in world space) as an occluder.
	// Using real triangles instead of the shape's bounding box prevents the
	// classic "sign hanging off a wall gets falsely occluded because it
	// lives inside the wall's AABB volume" failure — small meshes embedded
	// in a large mesh's bbox no longer flicker across tile boundaries.
	// Returns true if we actually rasterised; false if the shape was skipped
	// (no data, too thin on some axis, or camera inside the tight AABB).
	static bool rasterizeTriShape(NI::TriShape* shape, const TES3::Vector3& eye) {
		// Skip skinned meshes (NPCs, creatures). Their vertices live in
		// bind-pose space and need skinInstance->deform() to produce the
		// current animated positions — rasterising the bind-pose verts
		// would draw the mesh in its T-pose at world origin, far from
		// where the engine actually renders it. Moving actors are also
		// poor occluders in practice.
		if (shape->skinInstance) {
			return false;
		}

		auto data = shape->getModelData();
		if (!data) {
			return false;
		}
		// Virtual accessors: the raw vertexCount / triangleListLength
		// fields are allocation sizes, while getActive*() returns the
		// logically-valid subset (e.g. after LOD trimming). Using the
		// raw values over-reads and Intel's gather crashes on the stale
		// tail entries.
		const unsigned short vertexCount = data->getActiveVertexCount();
		if (vertexCount == 0 || data->vertex == nullptr) {
			return false;
		}
		const unsigned short triCount = data->getActiveTriangleCount();
		const NI::Triangle* tris = data->getTriList();
		if (triCount == 0 || tris == nullptr) {
			return false;
		}
		// Skip absurdly dense meshes as occluders: per-frame vertex transform
		// and Intel's binning cost scale with triCount, and huge meshes rarely
		// produce proportionally better occlusion. Still visibility-tested via
		// their bounding sphere on the drain pass.
		if (triCount > g_occluderMaxTrianglesEffective) {
			++g_skippedTriCount;
			return false;
		}

		const auto& xf = shape->worldTransform;
		const auto& R = xf.rotation;
		const auto& T = xf.translation;
		const float s = xf.scale;

		// Transform every vertex once into the reusable world-space buffer.
		// We also accumulate the tight AABB for the inside-guard and
		// thin-axis gate. Not cached across frames: TriShapeData pointers
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

		// Reject pencil-shaped meshes: their silhouette area is tiny and
		// the per-frame rasterisation cost isn't worth the near-zero
		// occlusion contribution. Walls/floors with a single thin axis
		// still qualify.
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

		// Inside-guard: rasterising a mesh the camera sits inside would
		// cover the screen with near-face depths and falsely occlude
		// everything behind the far face. Tight AABB + small margin is
		// enough here since the real triangles are strictly inside it.
		const float m = g_insideOccluderMarginEffective;
		if (eye.x >= minX - m && eye.x <= maxX + m &&
			eye.y >= minY - m && eye.y <= maxY + m &&
			eye.z >= minZ - m && eye.z <= maxZ + m) {
			++g_skippedInside;
			return false;
		}

		// Expand NI's 16-bit triangle list into MSOC's 32-bit index
		// buffer. Indices beyond vertexCount-1 would out-of-bounds read
		// g_occluderVerts inside Intel's gather; clamp defensively.
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

		// VertexLayout(12, 4, 8): stride 12 bytes, y at offset 4, z at
		// offset 8 — tightly-packed float[3] per vertex.
		// BACKFACE_NONE because NIF winding is not guaranteed consistent
		// and we want every face to contribute to occluder depth.
		if (g_asyncThisFrame) {
			// Move our owning buffers into the per-frame arena so the worker
			// threads have stable pointers until Flush() retires the job.
			// The next rasterizeTriShape call allocates fresh vectors
			// (~100us/frame of heap churn, acceptable per plan §3.2).
			g_asyncOccluderVerts.emplace_back(std::move(g_occluderVerts));
			g_asyncOccluderIndices.emplace_back(std::move(g_occluderIndices));
			// Inner vectors hold their buffer pointer across outer-vector
			// relocations (std::vector move ctor steals the pointer), so
			// .back().data() is stable for the submission's lifetime.
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
		g_occluderTriangles += outTri; // _Claude_ frame-wide triangle sum
		return true;
	}

	// ============================================================
	// Terrain aggregation
	// ============================================================

	// _Claude_ Simple frustum check for aggregate-terrain walk. No mask
	// bookkeeping — the aggregate pass runs before cullShowBody so
	// usedCullingPlanesBitfield is still clean and we don't mutate it.
	// Matches the plane-vs-sphere sign convention at cullShowBody L661.
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

	// _Claude_ Append one terrain TriShape's 25v/32t mesh to the aggregate
	// buffers, transforming vertices to world space and offsetting indices
	// by the current vertex base. Mirrors rasterizeTriShape's vertex math
	// and OOB-index guard, minus the gates/skinning/thin-axis paths that
	// don't apply to terrain.
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

		// _Claude_ Downsample fast path: Morrowind terrain subcells are
		// canonically a 5x5 grid in row-major order (vertex index = row*5 + col,
		// 25v/32t). When step ∈ {2, 4} and vcount == 25, sample every `step`-th
		// vertex on each axis, taking min-z over the dropped neighbours so the
		// silhouette can only shrink (conservative-as-occluder: under-occlude
		// rather than over-occlude). step values: 2 -> 3x3 grid (8 tris),
		// 4 -> 2x2 grid (2 tris). Any other vcount or step (3, 5+, etc.)
		// falls through to the full-resolution path — non-grid terrain shapes
		// (rare; possibly mod content) and unsupported steps are submitted
		// intact, which is correct-but-unoptimised. Only 2 and 4 divide the
		// 4-quad edge cleanly without dropping the seam row.
		if (vcount == 25 && (step == 2 || step == 4)) {
			// Coarse-grid axis count: step=2 -> 3 verts/edge, step=4 -> 2.
			const unsigned int n = (4u / step) + 1u;
			const unsigned int baseVert = static_cast<unsigned int>(aggVerts.size() / 3);
			aggVerts.resize(aggVerts.size() + static_cast<size_t>(n * n) * 3);
			float* out = aggVerts.data() + baseVert * 3;

			// For each kept vertex (cr, cc) in coarse grid, take min-world-z
			// over the source verts in the source-grid neighbourhood
			// [cr*step .. cr*step+(step-1)] x [cc*step .. cc*step+(step-1)],
			// clipped to [0,4]. Source verts at the right/bottom seam are
			// only covered by the last-row/column kept vertex itself, which
			// is fine — the seam already matches the next subcell exactly.
			for (unsigned int cr = 0; cr < n; ++cr) {
				for (unsigned int cc = 0; cc < n; ++cc) {
					const unsigned int sr0 = cr * step;
					const unsigned int sc0 = cc * step;
					const unsigned int sr1 = (cr + 1 == n) ? sr0 : sr0 + (step - 1);
					const unsigned int sc1 = (cc + 1 == n) ? sc0 : sc0 + (step - 1);

					float minWz = std::numeric_limits<float>::infinity();
					float keepX = 0, keepY = 0;
					// Track the kept (x,y) at the canonical anchor (sr0, sc0)
					// so the coarse XY grid stays regular — only Z is min-folded.
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

			// Build (n-1)*(n-1)*2 triangles. BACKFACE_NONE is used downstream,
			// so winding doesn't matter — both per-quad triangles are emitted
			// in CCW relative to a top-down view, but the rasterizer accepts
			// either.
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

	// _Claude_ Build the world-space VB+IB for one per-Land NiNode.
	// Called on cache miss only. Walks all subcells and trishapes
	// unconditionally (no per-shape frustum/appCulled filter) because the
	// result must be valid for any future camera angle. MSOC handles
	// view-frustum clipping internally during rasterisation, so the
	// "extra" triangles we include cost negligible compared to the
	// per-frame walk + matrix multiply they replace.
	// _Claude_ Map MCM dropdown value (0/1/2) to subsample step (1/2/4).
	// 0=Full (5x5, 32 tris/subcell), 1=Half (3x3, 8 tris/subcell),
	// 2=Corners (2x2, 2 tris/subcell). Out-of-range values clamp to Full
	// so a malformed JSON edit doesn't disable terrain entirely.
	static unsigned int currentTerrainStep() {
		switch (g_terrainResolutionEffective) {
		case 1: return 2;
		case 2: return 4;
		default: return 1;
		}
	}

	static void buildLandCacheEntry(LandCacheEntry& entry, NI::Node* landNode) {
		entry.verts.clear();
		entry.indices.clear();
		const unsigned int step = currentTerrainStep();
		const auto& subcells = landNode->children;
		for (size_t j = 0; j < subcells.endIndex; ++j) {
			auto* sub = subcells.storage[j].get();
			if (!sub) continue;
			if (!sub->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;
			auto* subNode = static_cast<NI::Node*>(sub);
			const auto& shapes = subNode->children;
			for (size_t k = 0; k < shapes.endIndex; ++k) {
				auto* shape = shapes.storage[k].get();
				if (!shape) continue;
				if (!shape->isInstanceOfType(NI::RTTIStaticPtr::NiTriShape)) continue;
				const auto p = classifyOccluderProperties(shape);
				if (p.alpha || p.stencil) continue;
				appendTerrainShape(entry.verts, entry.indices, static_cast<NI::TriShape*>(shape), step);
			}
		}
		entry.triCount = static_cast<unsigned int>(entry.indices.size() / 3);
		entry.builtForResolution = static_cast<uint8_t>(g_terrainResolutionEffective);
	}

	// _Claude_ Aggregate terrain rasteriser. Walks the WorldLandscape
	// subtree (root → per-Land NiNode → 16 subcell NiNodes → N NiTriShapes)
	// and submits one combined occluder per visible Land. Individual
	// 25v/32t patches fail the thin-axis gate; merging them gives the
	// hill/horizon silhouette that actually occludes distant architecture.
	//
	// Must run inside the isTopLevel block — after ClearBuffer and
	// uploadCameraTransform, before cullShowBody traversal so the depth
	// buffer has terrain by the time the drain tests leaves against it.
	//
	// Uses g_landCache to amortise the per-Land walk + vertex transform:
	// first-frame sighting builds the entry, subsequent frames pass the
	// cached pointers straight to RenderTriangles. Cache lifetime is
	// tied to the per-Land NiNode pointer — cell-change teardown frees
	// the old NiNode and a new one is allocated; mark-and-sweep evicts
	// the stale entry at frame end.
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
			// _Claude_ Treat a resolution-mismatch as a miss so MCM
			// dropdown changes propagate lazily — the entry already
			// in the map gets rebuilt at the new resolution on its
			// next visit. nodePtr stays valid (the NI::Pointer keeps
			// the engine's NiNode pinned), so we only need to redo
			// the verts/indices.
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

			// Per-frame submit decisions still apply. appCulled can toggle
			// on the per-Land root (game-driven distance culling) and the
			// frustum test remains a cheap pre-filter before MSOC's own
			// clipping. Shape-level culling is intentionally omitted — any
			// filter applied during cache build would bake view-specific
			// state into a reusable buffer.
			if (land->getAppCulled()) continue;
			if (frustumCulledSphere(land, camera)) continue;
			if (entry.triCount == 0) continue;

			if (g_asyncThisFrame) {
				// Cached buffers outlive this submission (persistent in
				// g_landCache until evicted), so we pass their pointers
				// directly. MSOC queues work for worker threads which
				// consume it during the drain-phase Flush; by the time
				// we'd ever evict, that Flush has completed.
				ScopedUsAccumulator t(g_rasterizeTimeUs);
				g_threadpool->RenderTriangles(entry.verts.data(), entry.indices.data(),
					static_cast<int>(entry.triCount),
					::MaskedOcclusionCulling::BACKFACE_NONE,
					::MaskedOcclusionCulling::CLIP_PLANE_ALL);
			}
			else {
				ScopedUsAccumulator t(g_rasterizeTimeUs);
				g_msoc->RenderTriangles(entry.verts.data(), entry.indices.data(),
					static_cast<int>(entry.triCount), g_worldToClip,
					::MaskedOcclusionCulling::BACKFACE_NONE,
					::MaskedOcclusionCulling::CLIP_PLANE_ALL,
					::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
			}

			++g_aggregateTerrainLands;
			g_aggregateTerrainTris += entry.triCount;
		}

		// Sweep: drop entries whose per-Land NiNode isn't a child of
		// WorldLandscape this frame. Safe to free now because
		// ClearBuffer() at top-level entry already flushed previous-
		// frame async work that referenced these pointers.
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

	// Body of the engine's CullShow, with MSOC query + occluder rasterisation
	// wedged between the frustum test and the Display call. Replaces the
	// engine's function at 0x6EB480 completely — our detour JMPs here.
	// Behaviour matches the engine 1:1 when g_msocActive is false.
	static void __fastcall cullShowBody(NI::AVObject* self, void* /*edx*/, NI::Camera* camera) {
		if (g_msocActive) ++g_recursiveCalls;
		if (self->getAppCulled()) {
			if (g_msocActive) ++g_recursiveAppCulled;
			return;
		}

		// Hierarchical frustum test, mirroring the engine's loop at 0x6EB4B7.
		// setBits tracks bits WE flipped in usedCullingPlanesBitfield so we
		// can unflip them before returning (the engine's LABEL_10 cleanup).
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
			// Two-pass: every NiTriBasedGeom leaf (NiTriShape + NiTriStrips,
			// the bulk of Morrowind's decorative/static meshes) defers its
			// visibility test until after the main traversal has finished
			// populating the depth buffer. This eliminates the single-pass
			// false positive where a leaf is tested against an occluder its
			// parent or earlier sibling rasterised moments before (door
			// behind its own wall, leaf in front of its own branch) — by
			// the time the drain phase runs, every occluder that can
			// contribute is already in the buffer, so the test result only
			// depends on geometry and camera, not on scene-graph order.
			//
			// Large NiTriShapes still rasterise inline as occluders, but
			// *without* an own-visibility test — MSOC's "closest 1/w wins"
			// means rasterising a shape that's itself occluded just
			// overdraws tiles with farther depths (harmless, wastes a few
			// triangles). Removing the own-test breaks the cycle where a
			// shape could fail the test against its own sibling's
			// rasterisation and still contribute usable occluder depth.
			//
			// NiNodes are *not* MSOC-tested — only frustum-culled (above).
			// Testing them inline against a partially-populated depth buffer
			// was a major source of false positives: a ref's root NiNode
			// tested before the rest of the scene rasterised could fail
			// against an earlier sibling's occluders and suppress the whole
			// subtree. Skipping the NiNode test means every leaf that
			// survives frustum reaches the drain and is tested once against
			// the final depth buffer — correctness first, at the cost of
			// losing the "skip whole subtree" shortcut.
			const bool isGeom = self->isInstanceOfType(NI::RTTIStaticPtr::NiTriBasedGeom);
			if (isGeom) {
				bool didRasterise = false;
				// Only opaque NiTriShapes rasterise as occluders. Alpha-
				// tested/blended geometry (fences, banners, tree leaves)
				// would fill the depth buffer across their whole quad and
				// falsely occlude things behind the transparent parts.
				//
				// _Claude_ When aggregate-terrain is on, skip per-patch
				// rasterise for terrain descendants — they're already in
				// the depth buffer as merged per-Land submissions, and
				// re-rasterising them is duplicate work (harmless depth-
				// wise but wastes triangles / threadpool queue slots).
				const bool skipAsAggregated = g_aggregateTerrainEffective
					&& isLandscapeDescendant(self, g_worldLandscapeRoot);
				if (!skipAsAggregated
					&& boundRadius >= g_occluderRadiusMinEffective
					&& boundRadius <= g_occluderRadiusMaxEffective
					&& self->isInstanceOfType(NI::RTTIStaticPtr::NiTriShape)) {
					const auto p = classifyOccluderProperties(self);
					if (p.alpha) {
						++g_skippedAlpha;
					}
					else if (p.stencil) {
						++g_skippedStencil;
					}
					else {
						const auto& eye = camera->worldTransform.translation;
						if (rasterizeTriShape(static_cast<NI::TriShape*>(self), eye)) {
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

	// _Claude_ Phase-1 of the two-phase drain. Classifies entries in the
	// half-open range [lo, hi) of g_pendingDisplays into g_drainSlots.
	// Read-only on shared state: no writes to g_drainCache, no counter
	// increments (except the atomic g_queryNearClip inside testSphereVisible),
	// no display() calls, no tints. All write-bearing logic moves into
	// phase 2 in drainPendingDisplays so this body is safe to invoke from
	// worker threads once the parallel dispatch path lands. Until then it
	// runs serially on the main thread; sharing one body between serial
	// and parallel modes lets the serial-only refactor commit validate
	// the phase-1 logic before threading is introduced.
	static void classifyDrainRange(size_t lo, size_t hi) {
		const unsigned int tcFrames = g_temporalCoherenceFramesEffective;
		const bool skipTerrainEnabled = g_skipTerrainOccludeesEffective;
		const float tinyThreshold = g_occludeeMinRadiusEffective;

		for (size_t i = lo; i < hi; ++i) {
			const auto& p = g_pendingDisplays[i];
			auto& slot = g_drainSlots[i];
			slot.ranTestRect = false;

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

			::MaskedOcclusionCulling::CullingResult r;
			// _Claude_ No per-call timing here. Phase-1 wall time is
			// bracketed once on the main thread (see g_classifyUs at
			// the runParallel decision in drainPendingDisplays); per-call
			// sums across workers measured CPU time, not wall time.
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

	// _Claude_ Parallel-drain worker body — DISABLED along with the rest
	// of the parallel path. Kept as commented reference; see the worker-
	// pool block higher in this file for the full checklist if re-enabled.
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

	// Drain the deferred-display queue built during the main traversal.
	// Each entry is re-tested against the now-complete depth buffer and
	// displayed if still visible. Must run before g_msocActive is cleared
	// so any counters it updates land in the current frame's log line.
	//
	// _Claude_ Two-phase split. Phase 1 (classifyDrainRange) classifies
	// every entry into a DrainSlot using only read-only access to shared
	// state. Phase 2 (the loop below) walks the slots in original index
	// order and applies all writes — counter increments, cache inserts,
	// tints, display() calls. Counter accounting matches the pre-refactor
	// serial loop exactly: ranTestRect is the single-bit flag that lets
	// phase 2 reproduce the !reused gate from the original code (true
	// only when testSphereVisible actually ran, false for skip and cache
	// paths). Bit-exact counter parity vs. the pre-refactor build is the
	// acceptance criterion for the refactor.
	static void drainPendingDisplays() {
		ScopedUsAccumulator t(g_drainPhaseTimeUs);
		// Fast path: nothing rasterised this frame means the depth buffer
		// is still cleared, so every TestRect would return VISIBLE for
		// shapes in frustum. Skip the query loop entirely and just display
		// each deferred shape. Saves ~1 µs/entry plus the async Flush cost
		// (handled at the caller) on occluder-free frames — rare at
		// defaults, but common in tiny rooms / dense fog / or after a
		// knob tweak that rejects every candidate.
		// _Claude_ Aggregate-terrain submissions count as occluders too —
		// a pass that put 40k+ hill/horizon triangles in the buffer can
		// still cull deferred leaves behind it, so we must not short-circuit
		// just because no main-pass leaf qualified.
		if (g_rasterizedAsOccluder == 0 && g_aggregateTerrainLands == 0) {
			ScopedUsAccumulator tt(g_drainDisplayUs); // _Claude_ audit (whole-loop)
			for (const auto& p : g_pendingDisplays) {
				p.shape->vTable.asAVObject->display(p.shape, p.camera);
			}
			g_pendingDisplays.clear();
			return;
		}

		const size_t n = g_pendingDisplays.size();

		// _Claude_ Parallel drain DISABLED. Only the serial classify path
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

		// Phase 1: classify. Serial over the whole queue.
		g_drainSlots.resize(n);
		// if (runParallel) {
		// 	g_lastStage = 14;
		// 	const size_t half = n / 2;
		// 	g_drainRanges[0] = { 0, half };
		// 	g_drainRanges[1] = { half, n };
		// }

		// _Claude_ Phase-1 wall-time bracket. Kept so classifyUs still
		// reports into MSOC.log.
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

		// _Claude_ Shared handler for OCCLUDED verdicts (from both fresh
		// and cached paths). Lambda because the codebase forbids goto;
		// compilers inline single-call-site lambdas. Takes p by parameter
		// rather than capturing it so the body is independent of any
		// loop-local state.
		auto handleOccluded = [&](const PendingDisplay& p) {
			if (g_tintOccludedEffective) {
				tintEmissive(p.shape, kTintOccluded);
				ScopedUsAccumulator tt(g_drainDisplayUs);
				p.shape->vTable.asAVObject->display(p.shape, p.camera);
			}
		};

		// Phase 2: serial action pass. Counter-increment semantics match
		// the pre-refactor loop exactly — see the function comment.
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
					// _Claude_ Cache only OCCLUDED. VISIBLE and VIEW_CULLED
					// still call display() this frame, which dereferences
					// p.shape anyway — caching them gains nothing on the
					// hit path while widening the stale-pointer attack
					// surface.
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
			// _Claude_ Skip Tested-tint for shapes that already got the
			// Occluder tint in the main pass; last-write-wins would flip
			// yellow → green and hide the occluder classification.
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

	// Function-level detour installed at NiAVObject::CullShow (0x6EB480).
	// Every direct caller in the engine — NiNode::Display, all 4
	// NiBSPNode::Display quadrants, NiSwitchNode::Display, NiCamera::Click —
	// now lands here (verified via IDA get_callers: exactly 7 direct
	// callers, no indirect dispatch). The top-level entry for the
	// main-scene worldCamera pass (g_inRenderMainScene set by
	// renderMainScene_wrapper, camera matches mainCamera, not already
	// inside an MSOC traversal) drives the per-frame MSOC setup/teardown.
	// Every other entry — recursive descent inside an active traversal,
	// non-main-camera Clicks like shadow/water/arm, Clicks from outside
	// renderMainScene (load screen, screenshots, etc.) — just runs the
	// body, matching the engine 1:1.
	static void __fastcall CullShow_detour(NI::AVObject* self, void* edx, NI::Camera* camera) {
		// _Claude_ Depth tracking for freeze diagnostic. Fires on every
		// entry (top-level + recursive descents through display()); the
		// per-frame max is surfaced in the periodic log. RAII so the
		// decrement happens on every return path. See CallDepthGuard.
		CallDepthGuard depthGuard;
		bool isTopLevel = false;
		TES3::Cell* activeCell = nullptr;
		if (!g_msocActive && g_inRenderMainScene) {
			auto wc = TES3::WorldController::get();
			NI::Camera* mainCamera = wc ? wc->worldCamera.cameraData.camera.get() : nullptr;
			// _Claude_ Skip MSOC entirely while a menu is open (MCM, inventory,
			// dialogue, journal, console, etc). Two reasons:
			//   1. The threadpool's WakeThreads / Flush spin-locks have no
			//      timeout — a worker that doesn't reach the suspended state
			//      between frames freezes the main thread. Menu mode is the
			//      observed trigger for that race.
			//   2. FPS doesn't matter while a menu is open; the world behind
			//      the menu can render with vanilla frustum culling only.
			// The threadpool stays in whatever state the previous top-level
			// frame left it (suspended, since SuspendThreads runs at the end
			// of every active frame). Skipping isTopLevel here means we
			// neither WakeThreads nor SuspendThreads while the menu is up,
			// which sidesteps the race wholesale.
			const bool inMenuMode = wc && wc->flagMenuMode;
			if (inMenuMode) ++g_skippedMenuMode;
			// _Claude_ Multi-pass guard (snapshot-only-terrain hypothesis).
			// The engine may issue more than one main-camera CullShow per
			// renderMainScene (e.g. shadow-caster pass, main pass, 1st-
			// person subtree) and each one would ClearBuffer + run its own
			// subtree, so the LAST pass clobbers everything the earlier
			// passes wrote. This explains the observed pattern where the
			// snapshot only contains aggregateTerrain (which reads the
			// global worldLandscapeRoot on every pass) while cantons
			// rasterized during the first pass are wiped.
			//
			// Gate: only open isTopLevel on the FIRST main-camera entry
			// per renderMainScene call. Subsequent main-camera CullShows
			// fall through to the vanilla body path — they still render
			// correctly (engine path), they just don't re-build the mask.
			const bool alreadyBuiltThisScene = g_isTopLevelFiresThisScene > 0;
			// Pre-guard OUTER-ENTRY counter — counts main-camera CullShow
			// detour entries that are NOT recursive (i.e. the engine is not
			// already inside an MSOC traversal). Without the !g_msocActive
			// gate this would also count every NiAVObject child visited
			// during scene-graph descent through the detour, which balloons
			// into the hundreds per frame just from a normal traversal.
			// Restricted here so a value > 1 genuinely means the engine
			// issued multiple independent main-camera passes in one
			// renderMainScene.
			if (camera == mainCamera && !inMenuMode && !g_msocActive) {
				++g_mainCamCullShowAttemptsThisScene;
			}
			if (camera == mainCamera && !inMenuMode && !alreadyBuiltThisScene) {
				// Scene-type gate: let users disable MSOC in interiors or
				// exteriors independently. Cheap flag bit test on the active
				// cell; falls back to exterior behaviour if currentCell isn't
				// yet populated (main-menu preview, load screen).
				// _Claude_ dh null on load screen / main menu — no scene to
				// cull, so we stay out of isTopLevel entirely rather than
				// null-check at every use inside the block.
				auto dh = TES3::DataHandler::get();
				if (dh) {
					const bool isInterior = dh->currentCell
						&& dh->currentCell->getIsInterior();
					// _Claude_ EnableMSOC is the master runtime gate.
					// Reconcile resource state with the live config first:
					// if the user just toggled MSOC on, allocate g_msoc +
					// threadpool now; if they just toggled it off, tear
					// them down (joins workers, frees ~57MB). The reconciler
					// is idempotent in steady state. Returns true only when
					// MSOC is enabled AND resources are live; on failure
					// (alloc returned null) it returns false and we fall
					// through to vanilla cullShowBody for this frame.
					const bool resourcesLive = ensureMSOCResourcesMatchConfig();
					const bool sceneEnabled = resourcesLive
						&& (isInterior
							? Configuration::OcclusionEnableInterior
							: Configuration::OcclusionEnableExterior);
					if (sceneEnabled) {
						isTopLevel = true;
						// _Claude_ Resolve scene-type-split occluder thresholds
						// once per top-level frame. Recursive cullShowBody calls
						// read the g_*Effective copies; keeps the hot-path free
						// of per-shape isInterior branching.
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
						// _Claude_ Refresh the rest of the hot-path config
						// cache. See the g_*Effective comment block for the
						// mid-frame-change contract these rely on.
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
						// Captured for drain-loop terrain-skip.
						g_worldLandscapeRoot = dh->worldLandscapeRoot;
						// Forwarded to the isTopLevel block below for
						// cell-change cache invalidation. Stored only on
						// active frames so a disabled-scene span doesn't
						// spuriously "change cell".
						activeCell = dh->currentCell;
					}
					else {
						// Counter accumulates only on skipped frames; no
						// top-level fires then so the reset at the top of
						// an active frame zeroes it out. Visible only on
						// the first active frame after a skipped stretch
						// — this is intended, per Phase 2.2 semantics.
						++g_skippedSceneGate;
					}
				}
			}
		}

		if (isTopLevel) {
			++g_isTopLevelFiresThisScene; // _Claude_ multi-pass hypothesis diagnostic
			g_lastStage = 1; // _Claude_ entered top-level
			// Latch async mode once per frame so a mid-frame Lua toggle
			// can't split submissions between threadpool and direct MOC.
			g_asyncThisFrame = g_threadpool
				&& Configuration::OcclusionAsyncOccluders;
			// _Claude_ Mask is about to be wiped. Out-of-tree consumers
			// (MGE-XE) gate queries on g_maskReady; clear it here so any
			// query that arrives mid-frame before the drain completes
			// gets a conservative VISIBLE instead of reading a partial
			// depth buffer.
			g_maskReady = false;
			if (g_asyncThisFrame) {
				// Wake up the worker threads ~100us before the first
				// RenderTriangles call. ClearBuffer() does an implicit
				// Flush() which retires any stragglers from the previous
				// frame — safe to clear the arena once it returns.
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
			// _Claude_ Cell-change cache invalidation. Must run AFTER the
			// ClearBuffer()/Flush above so the threadpool has finished
			// consuming any cached buffers still referenced by last
			// frame's queued work — only then is it safe to free them.
			// Both caches key off pointers that a cell load can free and
			// reuse (per-Land NiNode*, shape NI::AVObject*), so a wipe
			// here eliminates the stale-pointer hazard wholesale.
			g_lastStage = 4; // _Claude_ cellWipe
			if (activeCell != g_lastCell) {
				g_landCache.clear();
				g_drainCache.clear();
				g_lightCullCache.clear();
				// _Claude_ Debug-tint map: releases our NI::Pointer refs
				// on shapes from the outgoing cell so they can actually
				// be freed. Shapes that survive the cell change (player,
				// inventory items) get re-entered on their next tint
				// call and end-of-frame reset continues working.
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
			g_lastStage = 5; // _Claude_ ageprune (drain + light caches below)
			// _Claude_ Age-prune the temporal drain cache so entries for
			// shapes we haven't seen in a while (unloaded cell, destroyed
			// reference) don't accumulate. Window is 2*N frames — any
			// entry older than that can't be reused anyway. When the
			// feature is disabled (N=0) the whole map is dropped so it
			// doesn't linger from a previous session.
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
			// _Claude_ Age-prune the light-cull cache on the same pattern.
			// Without this, transient lights (spell effects, thrown-item
			// glows, light-projectile spells) accumulate entries until the
			// next cell change — each one pinned alive via the NI::Pointer
			// key. Window is 2*hysteresis frames: any entry not queried
			// within that span isn't being iterated by the renderer anymore
			// and can't affect hysteresis state. When light culling is
			// disabled the whole map is dropped so a previously-enabled
			// session doesn't linger.
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
			g_classifyUs = 0; // _Claude_ phase-1 wall time, comparable across serial/parallel
			g_drainDisplayUs = 0; // _Claude_ audit
			g_asyncFlushTimeUs = 0;
			// _Claude_ Freeze diagnostics: per-frame counters reset here
			// alongside the rest. Session maxes (g_max*Session) are NOT
			// reset — they're lifetime peaks for the periodic log.
			g_wakeThreadsTimeUs = 0;
			g_maxCallDepthThisFrame = 0;
			g_lastStage = 6; // _Claude_ uploadCamera
			uploadCameraTransform(camera);
			if (g_asyncThisFrame) {
				g_lastStage = 7; // _Claude_ setMatrix
				// SetMatrix copies into the threadpool's state ring buffer;
				// must be called AFTER uploadCameraTransform populated
				// g_worldToClip. Passed once per frame (all submissions
				// share the same world-to-clip transform).
				g_threadpool->SetMatrix(g_worldToClip);
			}
			g_msocActive = true;

			// _Claude_ External occluder drain (Phase B). Rasterizes any
			// triangles consumers queued via mwse_addOccluder into the
			// freshly-cleared mask, before the native near-scene / terrain
			// passes. Must run here — AFTER ClearBuffer so we don't get
			// stomped, AFTER uploadCameraTransform so g_worldToClip is
			// live for the direct-MOC path, and BEFORE any threadpool
			// work queues (so direct g_msoc writes don't race workers).
			drainPendingOccluders();

			// _Claude_ Aggregate terrain pass: submit merged per-Land
			// occluders before the main traversal so the depth buffer
			// has hill/horizon silhouettes by the time non-terrain
			// leaves reach the drain. Individual 25v/32t patches would
			// fail the thin-axis gate; merging them reclaims terrain
			// as a useful occluder. Gated for A/B profiling.
			if (Configuration::OcclusionAggregateTerrain) {
				g_lastStage = 8; // _Claude_ aggTerrain
				rasterizeAggregateTerrain(camera);
			}
		}

		g_lastStage = 9; // _Claude_ cullShowBody (or non-top-level body)
		cullShowBody(self, edx, camera);

		if (isTopLevel) {
			// _Claude_ Aggregate-terrain submissions also go through the
			// threadpool, so the Flush gate must account for them too —
			// otherwise a frame with terrain but no main-pass occluders
			// SuspendThreads() with queued work still in the ring, which
			// can leave the threadpool in an inconsistent state next
			// frame (suspected cause of a freeze on menu entry). Same
			// shape as the drain fast-path gate.
			const bool hadAsyncWork = g_asyncThisFrame
				&& (g_rasterizedAsOccluder > 0 || g_aggregateTerrainLands > 0);
			if (hadAsyncWork) {
				g_lastStage = 10; // _Claude_ flush
				// Barrier: Flush() blocks until every queued RenderTriangles
				// is fully rasterised into the shared buffer. Only after
				// this return is it safe to TestRect (drainPendingDisplays
				// runs on the main thread but queries the shared buffer).
				// Skipped when nothing was queued — no work to wait for and
				// drainPendingDisplays takes the no-TestRect fast path.
				ScopedUsAccumulator t(g_asyncFlushTimeUs);
				g_threadpool->Flush();
			}
			g_lastStage = 11; // _Claude_ drain
			drainPendingDisplays();
			// _Claude_ Snapshot swap — hands the just-completed mask to
			// external consumers (MGE-XE) and rotates the ex-prev buffer
			// in as the next frame's write target. The ex-prev buffer
			// still carries the previous snapshot's depth data at this
			// point; the existing ClearBuffer() at the TOP of the next
			// frame's renderMainScene hook handles that reset (MOC's
			// RenderTriangles is depth-min, so a clear is mandatory to
			// prevent occluder depths from accumulating across frames).
			// Memory-wise nothing grows — two fixed-size MOC buffers, a
			// 16-float matrix copy, 3 floats, and a 64-bit tick.
			//
			// After the swap:
			//   g_msoc_prev = this frame's completed mask (external reads)
			//   g_msoc      = stale data, cleared next frame, re-written
			//
			// Freshness timestamp tags the snapshot with its wall-clock
			// capture time; queries more than kSnapshotMaxAgeMs stale are
			// rejected by isOcclusionMaskReady() — guards against culling
			// against a frozen mask during pauses / loading / alt-tab.
			std::swap(g_msoc, g_msoc_prev);
			// CRITICAL: CullingThreadpool caches the target buffer via
			// SetBuffer() at init; it does NOT follow the g_msoc pointer
			// variable. Without this re-point, the threadpool keeps
			// rasterizing into whatever buffer g_msoc was at init — the
			// snapshot side ends up alternating between populated and
			// empty every frame depending on parity, and external
			// consumers reading g_msoc_prev see garbage or nothing.
			// Pointed to the new g_msoc so next frame's rasterization
			// hits the correct (about-to-be-cleared, then written) buffer.
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
				g_lastStage = 12; // _Claude_ suspendThreads
				// Put the workers back to low-overhead sleep between frames.
				// Next frame's top-level entry will WakeThreads again.
				g_threadpool->SuspendThreads();
				g_asyncThisFrame = false;
			}
			g_msocActive = false;
			// _Claude_ Update lifetime peaks for the periodic log. These
			// stay across frames so a single outlier shows up in the next
			// baseline tick even if subsequent frames are quiet.
			if (g_wakeThreadsTimeUs > g_maxWakeThreadsUsSession) {
				g_maxWakeThreadsUsSession = g_wakeThreadsTimeUs;
			}
			if (g_maxCallDepthThisFrame > g_maxCallDepthSession) {
				g_maxCallDepthSession = g_maxCallDepthThisFrame;
			}
			g_lastStage = 13; // _Claude_ exited top-level cleanly
			// _Claude_ Publish the frame-end timestamp for the
			// watchdog. If the next frame freezes mid-body, the
			// forensics file shows "sinceFrameEndMs" growing
			// without bound — telling us the freeze is inside
			// MSOC code. If sinceFrameEndMs stays small (near
			// 250ms) while the game is frozen, the freeze is
			// upstream of CullShow_detour (game loop, MGE-XE,
			// D3D present, etc.) because frames are still
			// completing MSOC cleanly.
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

			// _Claude_ Two independently-toggled log channels:
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
					<< " occluderTris=" << g_occluderTriangles // _Claude_
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
					<< " terrainSkipped=" << g_skippedTerrain // _Claude_
					<< " aggTerrainLands=" << g_aggregateTerrainLands // _Claude_
					<< " aggTerrainTris=" << g_aggregateTerrainTris // _Claude_
					<< " aggTerrainUs=" << g_aggregateTerrainUs // _Claude_
					<< " landCacheHit=" << g_landCacheHits // _Claude_
					<< " landCacheMiss=" << g_landCacheMisses // _Claude_
					<< " landCacheEvict=" << g_landCacheEvictions // _Claude_
					<< " tcHit=" << g_drainCacheHits // _Claude_
					<< " tcMiss=" << g_drainCacheMisses // _Claude_
					<< " tcSize=" << g_drainCache.size() // _Claude_
					<< " cellChanges=" << g_cellChanges // _Claude_
					<< " sceneGateSkipped=" << g_skippedSceneGate
					<< " menuModeSkipped=" << g_skippedMenuMode // _Claude_ cumulative
					<< " rasterizeUs=" << g_rasterizeTimeUs
					<< " drainUs=" << g_drainPhaseTimeUs
					<< " classifyUs=" << g_classifyUs // _Claude_ phase-1 wall (serial = work; parallel = barrier)
					<< " displayUs=" << g_drainDisplayUs // _Claude_ audit
					<< " asyncFlushUs=" << g_asyncFlushTimeUs
					<< " wakeUs=" << g_wakeThreadsTimeUs // _Claude_ this frame's WakeThreads spin
					<< " maxWakeUsSess=" << g_maxWakeThreadsUsSession // _Claude_ lifetime peak
					<< " maxDepthFrame=" << g_maxCallDepthThisFrame // _Claude_ this frame's recursion peak
					<< " maxDepthSess=" << g_maxCallDepthSession // _Claude_ lifetime peak
					<< " tp=" << (g_threadpool ? 1 : 0) // _Claude_ threadpool liveness
					<< " cfgAsync=" << (Configuration::OcclusionAsyncOccluders ? 1 : 0) // _Claude_ JSON-propagated flag (async fires iff tp && cfgAsync)
					<< " topLvlThisScene=" << g_isTopLevelFiresThisScene // _Claude_ multi-pass hypothesis (post-guard)
					<< " maxTopLvlSess=" << g_maxIsTopLevelFiresSession // _Claude_ lifetime peak (post-guard)
					<< " mainAttemptsThisScene=" << g_mainCamCullShowAttemptsThisScene // _Claude_ pre-guard raw attempts
					<< " maxMainAttemptsSess=" << g_maxMainCamCullShowAttemptsSession // _Claude_ lifetime peak (pre-guard)
					<< " cumul=" << totalOccluded << "/" << totalTested
					<< "(vc=" << totalViewCulled << ")" << std::endl;
			}
		}
	}

	// ============================================================
	// renderMainScene wrapper & MSOC resource management
	// ============================================================

	// Wrapper for TES3Game_static::renderMainScene (0x41C400). Installed at
	// its 3 known call sites (renderNextFrame, takeScreenshot,
	// createSaveScreenshot; verified via IDA get_callers). Sets
	// g_inRenderMainScene while the function is on the stack so
	// CullShow_detour knows the per-frame main scene is active. The
	// wasActive save/restore makes this reentry-safe even though reentry
	// isn't expected — nesting would just run the inner call with the
	// flag already set, and the outer restore is a no-op.
	using RenderMainSceneFn = void(__cdecl*)();
	static const auto renderMainScene_original = reinterpret_cast<RenderMainSceneFn>(0x41C400);

	static void __cdecl renderMainScene_wrapper() {
		const bool wasActive = g_inRenderMainScene;
		g_inRenderMainScene = true;
		// _Claude_ Diagnostic: zero the per-scene isTopLevel counter so we
		// can observe whether the engine fires multiple main-camera CullShow
		// passes per renderMainScene (snapshot-only-terrain hypothesis).
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

		// Tint-debug reset: the batched renderer reads material state at
		// draw time, which happens *inside* renderMainScene_original after
		// the cull pass has populated the render list. By the time we
		// return here, all draws for this frame are done, so we flip every
		// tracked clone back to its source's current emissive. Next frame's
		// classifications re-apply the tint. No allocations, no property-
		// list churn — just a field write per tracked shape.
		//
		// Runs unconditionally and is a no-op when g_tintClones is empty,
		// saving us from gating on the three flags (which may toggle mid-
		// frame if the user edits from Lua, leaving state inconsistent).
		resetFrameTints();
	}

	// _Claude_ Allocate g_msoc + g_threadpool. Idempotent: returns true
	// without doing anything if both already exist. Returns false if
	// MSOC creation failed (g_msoc null) — callers should treat that as
	// a permanent disable. Called from installPatches() at startup when
	// EnableMSOC is on, and from the top-of-detour ensure-helper when
	// the user toggles MSOC on at runtime.
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
			// Snapshot buffer — external consumers read this one. Same
			// configuration as g_msoc so the pointer swap at drain-complete
			// preserves all rasterizer state. Pre-cleared so the very first
			// external query sees a defined (all-far) mask even before a
			// single frame has completed.
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
			// _Claude_ Hard cap on worker count. Intel's CullingThreadpool
			// uses hand-rolled spin-semaphores in its worker dispatch and
			// livelocks at ~98% CPU when too many workers spin against the
			// same bin queues; one user-reported freeze on a
			// 28-logical-thread CPU was traced to 26 spinning workers
			// saturating the scheduler so no individual worker could be
			// rescheduled to make progress. 12 is comfortably above the
			// hot-path workload's parallelism and stays well below the
			// saturation threshold even on many-core consumer CPUs and
			// future-proof against absurd reports (dual-socket Xeon ~80).
			// Applied as a safety gate to BOTH auto-sizing and manual
			// OcclusionThreadpoolThreadCount values — defence in depth
			// against the livelock failure mode regardless of how
			// threadCount got picked.
			constexpr unsigned int kAutoMax = 12;
			unsigned int threadCount = Configuration::OcclusionThreadpoolThreadCount;
			if (threadCount == 0) {
				// _Claude_ Auto-sizing. Take hw-2 (leave one full core for the
				// main thread) but cap at binCount/2 so each worker owns ~2 bins
				// — this keeps Intel's work-stealing path enabled (gated on
				// binCount > threadCount) and avoids the 1:1 case where the
				// busiest bin sets frame latency. Floor of 1 covers tiny CPUs
				// and binCount=1 edge cases. kAutoMax applied via std::min
				// so auto path naturally respects the livelock cap; the
				// gate below catches manual overrides.
				const unsigned hw = std::thread::hardware_concurrency();
				// _Claude_ Reservation policy: hw > 4 reserves 2 cores
				// (main render thread + scheduling slack), hw ≤ 4 only
				// reserves 1. The old hw-2 was a many-core heuristic that
				// stranded half the CPU on 4-thread parts (i5-2400 etc.):
				// hw=4 → hwBudget=2 left binCap=4 unused and gave only 2
				// workers when the spare hardware threads could happily
				// take 3. The clamp below (binCap, kAutoMax) still prevents
				// over-subscription and the 12-thread livelock cap.
				const unsigned int hwBudget = (hw <= 4)
					? (hw > 1 ? hw - 1 : 1)
					: hw - 2;
				const unsigned int binCap = binCount / 2 > 0 ? binCount / 2 : 1;
				threadCount = std::min({ hwBudget, binCap, kAutoMax });
			}
			// _Claude_ Safety gate: clamp manual overrides to kAutoMax.
			// Auto-sizing already incorporates the cap, so this only fires
			// for manual OcclusionThreadpoolThreadCount values exceeding
			// the livelock threshold. Logged so a user who set a high
			// manual value sees why they got fewer workers than they asked
			// for.
			if (threadCount > kAutoMax) {
				log << "MSOC: clamping threadCount from " << threadCount
					<< " to kAutoMax=" << kAutoMax
					<< " (Intel CullingThreadpool livelocks at high"
					<< " worker counts on saturated CPUs)." << std::endl;
				threadCount = kAutoMax;
			}
			// _Claude_ Safety gate: clamp threadCount to binCount unconditionally.
			// Intel's CullingThreadpool assigns work by bin — surplus threads
			// contend for the same bin mutexes and crash inside RenderTrilist.
			// Auto-sizing already respects this via binCap, but a manual
			// OcclusionThreadpoolThreadCount value can still trip it.
			if (threadCount > binCount) {
				log << "MSOC: clamping threadCount from " << threadCount
					<< " to binCount=" << binCount
					<< " (threads > bins causes worker crash)." << std::endl;
				threadCount = binCount;
			}
			constexpr unsigned int kMaxJobs = 64;
			// _Claude_ Partial-failure cleanup. The realistic throw site is
			// the threadpool ctor's per-thread state ring buffer (~57 MB
			// new[]). On bad_alloc — or any exception out of Set* /
			// SuspendThreads — we have a half-built world: g_msoc allocated,
			// g_threadpool either null (ctor threw before assignment) or
			// partially configured. Destroy both, return false; the
			// reconciler treats this frame as MSOC-disabled and the caller
			// (detour top-of-frame) falls through to vanilla cullShowBody.
			// Cost on the no-throw path is zero — try-block entry is a
			// few ns and this code only runs at install / on MCM toggle-on,
			// not per-frame (createMSOCResources short-circuits at the top
			// once both globals are live).
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

	// _Claude_ Free g_msoc + g_threadpool and clear the async arenas.
	// MUST be called only when no async work is in flight: between
	// frames on the render thread, with g_msocActive == false. The
	// threadpool destructor calls WakeThreads + sets mKillThreads +
	// joins all workers — bounded but blocking; up to a few ms while
	// any unparked worker reaches its suspend point.
	//
	// Mask-ready state and pending displays are also cleared so a
	// subsequent re-enable starts from a clean slate. Out-of-tree
	// query consumers (MGE-XE) will see g_maskReady=false and treat
	// queries as VISIBLE until the first re-enabled frame populates
	// the mask.
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
		// External occluder queue — drop any pending submissions so a
		// subsequent re-enable doesn't replay stale occluders against
		// the fresh mask on the first frame.
		{
			std::lock_guard<std::mutex> lock(g_pendingOccludersMutex);
			g_pendingOccluders.clear();
			g_externalOccluderTrisQueued = 0;
		}
		g_asyncThisFrame = false;
		g_maskReady = false;

		log << "MSOC: resources freed (threadpool joined, mask buffer destroyed)." << std::endl;
	}

	// _Claude_ Idempotent reconciler called at the safe top-of-frame
	// point in CullShow_detour. Brings live resource state into agreement
	// with Configuration::EnableMSOC. Returns false if creation failed
	// (treat as permanently disabled this frame) or if disabled — caller
	// should skip MSOC entirely. Returns true if resources are live and
	// the frame may proceed with MSOC.
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

		// _Claude_ Latch mask resolution from Configuration. Must run
		// BEFORE createMSOCResources (which calls SetResolution with
		// kMsocWidth/Height) so the snapshot buffer, threadpool, and
		// every later reader sees the tier-resolved size. Round down
		// to MOC's required alignment (width % SUB_TILE_WIDTH == 0,
		// height % SUB_TILE_HEIGHT == 0; SUB_TILE_WIDTH=8, SUB_TILE_HEIGHT=4
		// per MaskedOcclusionCullingCommon.inl) and clamp to a sane
		// range — pathological tiny resolutions trip MOC's edge-case
		// tile math (assert on width=0), and pathological huge ones
		// blow out the ~57MB threadpool ring buffer.
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

		// _Claude_ Hooks always install. The detour body is free when the
		// runtime gate (sceneEnabled at top-of-frame) is false — it just
		// calls vanilla cullShowBody. Resources (g_msoc + threadpool) are
		// allocated eagerly here when EnableMSOC starts on, or lazily on
		// the first MCM toggle-on via ensureMSOCResourcesMatchConfig() at
		// the top of the detour. When the user toggles MSOC off at
		// runtime, the next safe top-of-frame tears the threadpool down
		// (joins workers, frees the ~57MB ring buffer, destroys g_msoc).

		if (Configuration::EnableMSOC) {
			createMSOCResources(log);
		}
		else {
			log << "MSOC: starting with EnableMSOC=false; resources will be allocated on first MCM toggle-on." << std::endl;
		}

		// _Claude_ Spawn the freeze-forensics watchdog. Detached so
		// it survives until process exit without needing a join — we
		// have no orderly shutdown path for occlusion patches, and
		// the thread is read-only on racy globals so a torn read at
		// exit is harmless. If the user reports a freeze, the last
		// state snapshot will be sitting in MSOC.forensics.txt next
		// to MWSE.log.
		try {
			std::thread(watchdogTick).detach();
			log << "MSOC: forensics watchdog thread spawned (250ms tick; "
				"writes MSOC.forensics.txt)." << std::endl;
		}
		catch (const std::exception& e) {
			log << "MSOC: failed to spawn forensics watchdog: "
				<< e.what() << std::endl;
		}

		// _Claude_ Drain-parallel worker pool spawn — DISABLED. The
		// serial drain is the only active path. Preserved as commented
		// reference; to re-enable, restore this block plus the worker
		// globals, drainWorkerMain, the runParallel branch in
		// drainPendingDisplays, and Configuration::OcclusionParallelDrain.
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

		// Function-level detour at NiAVObject::CullShow. The 5-byte
		// prologue (sub esp,14h; push ebx; push esi) gets overwritten
		// with a JMP to our detour, which reimplements the body
		// end-to-end (no trampoline needed — we never call back into
		// 0x6EB480). This replaces the previous 7-call-site patch and
		// is equivalent in coverage: IDA get_callers shows exactly 7
		// direct callers, all Display methods + NiCamera::Click.
		mwse::genJumpUnprotected(0x6EB480, reinterpret_cast<DWORD>(CullShow_detour));

		// Call-site wrappers for renderMainScene. Three known callers
		// (IDA get_callers on 0x41C400): the per-frame path, screenshot
		// capture, save-thumbnail capture. Call-site wrapping avoids
		// the function-prologue trampoline problem — we need to
		// actually call the original — and the enforcement check
		// catches address drift on new game builds.
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

		// _Claude_ Light cull hook. Replaces the 6-byte
		// `mov al, [ebx+NiLight.super.enabled]` at 0x6bb7d4 inside
		// NiDX8LightManager::updateLights with `call shouldLightBeEnabled;
		// nop`. Installed unconditionally so Configuration::OcclusionCullLights
		// can be toggled at runtime without a restart; the detour itself
		// short-circuits to the real enabled flag when the feature is off
		// or MSOC failed to initialise.
		mwse::genCallUnprotected(0x6bb7d4,
			reinterpret_cast<DWORD>(updateLights_enabledRead_hook), 6);
		log << "MSOC: light cull hook installed at 0x6bb7d4 (gated by "
			<< "Configuration::OcclusionCullLights, default off)." << std::endl;
	}

	// _Claude_ Mask query API for out-of-tree consumers (MGE-XE etc.).
	// Queries read the SNAPSHOT (g_msoc_prev) — the mask from the
	// previous frame, frozen at drain-complete. This makes external
	// queries race-free: no matter where in the game frame the consumer
	// calls us, we always serve a stable, completely-built mask with
	// no mid-frame transitions.
	//
	// Freshness gate (kSnapshotMaxAgeMs) rejects snapshots older than
	// ~200ms so pauses / loading screens / alt-tab don't hand consumers
	// a stale frozen mask. The tick is recorded at each swap.
	bool isOcclusionMaskReady() {
		if (!g_msoc_prev || g_snapshotTickMs == 0) {
			return false;
		}
		const unsigned long long now = GetTickCount64();
		return (now - g_snapshotTickMs) <= kSnapshotMaxAgeMs;
	}

	// Project a world-space sphere against the SNAPSHOT buffer using
	// the _prev matrix + NDC constants captured at swap time. Mirrors
	// testSphereVisible's math exactly; kept separate so the plugin's
	// internal (in-progress mask) path stays untouched.
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

		// Single up-front readiness check — staleness is a per-frame
		// property; the mask can't flip mid-batch because swap happens
		// on the render thread at drain complete, and external callers
		// run serialized against that by construction (Morrowind is
		// single-threaded for render-adjacent work).
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

	// Multiply two column-major 4x4 matrices: `out = a * b`. Used only
	// by drainPendingOccluders when a consumer provides a model-to-world
	// matrix — `a` is the world-to-clip already cached in g_worldToClip,
	// `b` is the consumer's model matrix. Both in MOC/Intel column-major
	// layout (`m[col*4 + row]`).
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

	// Drain the external-occluder queue, rasterizing every queued
	// PendingOccluder into the live g_msoc. Called once per frame from
	// the render hook AFTER uploadCameraTransform (g_worldToClip is
	// valid) and BEFORE any threadpool work starts (no race against
	// async workers on the shared MOC instance).
	//
	// Uses the direct (non-threadpool) RenderTriangles path — external
	// submissions are tiny (tens to a few thousand tris) and serialize
	// cleanly on the main render thread, keeping the threadpool's single
	// "current matrix" invariant intact for the native occluder pipeline.
	static void drainPendingOccluders() {
		// Swap the queue out under the lock, then release — rasterization
		// can be slow and we don't want to block a mwse_addOccluder caller
		// on it. The externalOccluderTrisQueued counter resets too so the
		// next frame's budget accounting starts clean.
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
			// BACKFACE_NONE for external submissions: consumers typically
			// hand us closed convex hulls (box corners, simplified silhou-
			// ettes). Rasterizing both faces produces the same mask depth
			// as front-only because MOC stores minimum (closest) depth per
			// Hi-Z tile — back faces of a convex shape are always behind,
			// can't tighten the mask. Side benefit: winding doesn't matter,
			// which removes a common footgun when consumers build geometry.
			g_msoc->RenderTriangles(
				p.verts.data(), p.tris.data(), p.triCount,
				modelToClip,
				::MaskedOcclusionCulling::BACKFACE_NONE,
				::MaskedOcclusionCulling::CLIP_PLANE_ALL,
				layout);
			// Rolled into g_occluderTriangles so the per-frame stats
			// line already in MSOC.log reflects the external contribution
			// without needing a separate counter surface.
			g_occluderTriangles += p.triCount;
			++g_rasterizedAsOccluder;
		}
	}

	bool addOccluder(
		const float* verts, int vtxCount, int stride, int offY, int offW,
		const unsigned int* tris, int triCount,
		const float* modelMatrix16)
	{
		// Input validation — reject obviously malformed submissions
		// rather than silently copying garbage the rasterizer would trip
		// on. stride/offY/offW sanity: stride must be positive, Y and W
		// offsets must sit inside one vertex's worth of bytes.
		if (!verts || !tris || vtxCount <= 0 || triCount <= 0) {
			return false;
		}
		if (stride <= 0 || offY < 0 || offW < 0
			|| offY + static_cast<int>(sizeof(float)) > stride
			|| offW + static_cast<int>(sizeof(float)) > stride) {
			return false;
		}

		// If mask resources aren't live (EnableMSOC off, not yet inited,
		// teardown in progress), drop the submission — it would have no
		// mask to rasterize into. Not an error for the caller; just a
		// no-op consistent with the "soft feature" semantics.
		if (!g_msoc) {
			return false;
		}

		// Budget check. External submissions share the configured
		// OcclusionOccluderMaxTriangles cap with native occluders, and
		// external-first rejection keeps the plugin's own mask intact
		// under contention.
		const int cap = static_cast<int>(Configuration::OcclusionOccluderMaxTriangles);
		std::lock_guard<std::mutex> lock(g_pendingOccludersMutex);
		if (g_externalOccluderTrisQueued + triCount > cap) {
			// Rate-limit the log: a chatty consumer could otherwise
			// flood MSOC.log. Once per contention burst is enough.
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

		// Copy the input arrays into plugin-owned storage. The consumer
		// is free to reuse or free their buffers as soon as we return.
		PendingOccluder p;
		p.stride = stride;
		p.offY = offY;
		p.offW = offW;
		p.vtxCount = vtxCount;
		p.triCount = triCount;

		// stride is bytes per vertex; verts is sized by total bytes, not
		// by a per-vertex float count. Copy as raw bytes to preserve
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
		// Same validation + budget semantics as addOccluder; only difference
		// is the preTransformed flag on the queued entry and no model-matrix
		// path. Intentionally duplicates the validation block rather than
		// pulling in a shared helper — keeps both public paths readable in
		// isolation and the code is small.
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

		// ComputePixelDepthBuffer wants a caller-owned float buffer
		// sized for the full mask. kMsocWidth/Height are fixed at
		// construction (512x256 in this build). Reads the SNAPSHOT.
		std::vector<float> depth(static_cast<size_t>(kMsocWidth) * kMsocHeight, 0.0f);
		g_msoc_prev->ComputePixelDepthBuffer(depth.data(), /*flipY*/ true);

		// DIAGNOSTIC: also dump the LIVE buffer (g_msoc) alongside the
		// snapshot so we can tell whether native near-scene occluders
		// (cantons, walls) end up in the rasterization target but not
		// in the swapped snapshot — would imply the swap is breaking
		// our contract somewhere. File sits next to the main PFM with
		// "_live" suffix.
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

		// Normalise for immediate display in viewers that tone-map
		// against [0, 1]. MOC's native output uses two ranges that
		// don't display well straight:
		//   -1.0                           → "unwritten" tile sentinel
		//    tiny positive 1/w (~1e-5..1e-3) → occluder depth
		// Both collapse to black when GIMP / IrfanView auto-stretch
		// against [0, 1]. Remap so:
		//   unwritten → 0.0 (black)
		//   occluder  → [0.2, 1.0] (lightest = closest to camera)
		// This preserves the relative ordering so the silhouette
		// remains depth-correct, just with a display-friendly scale.
		// Log the raw stats so tooling still has exact numbers.
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
				const float t = (v - minPos) / range;   // 0..1 within positive range
				v = 0.2f + 0.8f * t;                    // map to [0.2, 1.0]
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

		// Expand 8 corners as center + (±vx ±vy ±vz). Layout is
		// (x, y, z, w) per vertex, stride 16 — matches VertexLayout below.
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

// _Claude_ Stable C-ABI shims. Named for GetProcAddress lookup from
// out-of-tree consumers (MGE-XE etc.). NI::Light* and void* share
// representation on x86; the reinterpret_cast is an ABI formality. If
// MWSE is renamed or moved behind a different DLL name, the export
// names here stay stable — consumers look up by name, not ordinal.
extern "C" __declspec(dllexport)
void __cdecl mwse_registerLightObservedCallback(void(__cdecl* cb)(void* niLight)) {
	// Safe: void* and NI::Light* have identical representation on x86 and
	// both function pointers use __cdecl, so the cast is an ABI formality.
	using Typed = msoc::patch::occlusion::LightObservedCallback;
	msoc::patch::occlusion::registerLightObservedCallback(
		reinterpret_cast<Typed>(cb));
}

extern "C" __declspec(dllexport)
void __cdecl mwse_unregisterLightObservedCallback(void(__cdecl* cb)(void* niLight)) {
	// Same ABI-formality cast as the register shim above.
	using Typed = msoc::patch::occlusion::LightObservedCallback;
	msoc::patch::occlusion::unregisterLightObservedCallback(
		reinterpret_cast<Typed>(cb));
}

// _Claude_ Mask query exports. Return values are the MWSE_OCC_* codes
// (0=Visible, 1=Occluded, 2=ViewCulled, 3=NotReady) — frozen in ABI
// independent of Intel's internal enum. Cheap to call; safe from the
// render thread at any point after renderMainScene's drain completes
// (gate with mwse_isOcclusionMaskReady() if unsure).
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
