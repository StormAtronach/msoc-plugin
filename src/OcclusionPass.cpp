#include "OcclusionApi.h"

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
// Cross-TU shared state (g_frame, the caches, mask consts) for
// the extracted subsystem TUs (QueryApi.cpp, ...).
#include "OcclusionInternal.h"
// LAYER-A-HORIZON: 1D horizon -> curtain occluder used by the Horizon
// mode of rasterizeAggregateTerrain. See src/HorizonOccluder.h.
#include "HorizonOccluder.h"
// Freeze-forensics watchdog. Owns the watchdog thread, its stage-name
// table, and the spawn gate. This TU implements the read accessor
// (forensics::captureSnapshot) it calls back into.
#include "ForensicsWatchdog.h"
// Debug mask overlay (engine texture mirror of the finished mask) + the
// PFM dump that shares its buffer. Both read the live mask post-drain.
#include "MaskOverlay.h"

#include "TES3Cell.h"
#include "TES3DataHandler.h"
#include "TES3WorldController.h"

#include "NIAVObject.h"
#include "NICamera.h"
#include "NIColor.h"
#include "NIDefines.h"
#include "NIGeometryData.h"
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
#include <atomic>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <ostream>
#include <unordered_map>
#include <vector>

namespace msoc::patch::occlusion {

// `log::getLog()` call sites resolved to mwse::log in MWSE proper;
// this alias keeps them unchanged.
namespace log = ::msoc::log;

// MSOC tile-buffer resolution. Decoupled from the game viewport.
// Latched from Configuration at install time; restart-only.
unsigned int kMsocWidth = 512;
unsigned int kMsocHeight = 256;

// kNearClipW (MOC near-plane w floor) now lives in OcclusionInternal.h,
// shared with the subsystem TUs.

// ============================================================
// MSOC instance & per-frame camera state
// ============================================================

// Heap-allocated via Create()/Destroy(). Leaked on process exit.
::MaskedOcclusionCulling* g_msoc = nullptr;  // extern in OcclusionInternal.h

// Transposed from NI's row-major M*v into Intel's column-major v*M
// layout (consecutive memory = one column). Refreshed at top-level
// CullShow_detour entry.
float g_worldToClip[16];  // extern in OcclusionInternal.h

// Per-frame matrix metrics for testSphereVisible:
//   g_ndcRadiusX/Y: L2 norm of clip.x/y coefficients. NDC half-extent
//                   of a sphere of radius r at clip-w cw is r*X/cw.
//   g_wGradMag:     L2 norm of clip.w coefficients. Worst-case clip-w
//                   offset from sphere center to near surface is r*mag.
//                   1.0 for standard perspective on pure-rotation view;
//                   computed for safety against scaled views.
float g_ndcRadiusX = 0.0f;  // extern in OcclusionInternal.h
float g_ndcRadiusY = 0.0f;  // extern in OcclusionInternal.h
float g_wGradMag = 0.0f;    // extern in OcclusionInternal.h

// DataHandler::worldLandscapeRoot, captured per top-level frame.
// Drain uses it to short-circuit occludee queries on terrain patches
// (25v/32t, ~always visible). Null before the world exists.
NI::Node* g_worldLandscapeRoot = nullptr;  // extern in OcclusionInternal.h

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

// True only while the worldCamera main pass is being traversed.
// Gates MSOC so shadow-manager, water-refraction, armCamera, and
// other non-main Clicks inside renderMainScene run vanilla.
static bool g_msocActive = false;

// Per-frame diagnostic counters + phase timers (FrameStats.h), reset at
// the top of each worldCamera traversal. Defined here; declared extern
// in FrameStats.h for the subsystem TUs.
FrameStats g_stats;

// ============================================================
// Cache types & globals (land / drain / light)
// ============================================================

OcclusionCaches g_caches;  // extern in OcclusionCaches.h

// ============================================================
// Frame counters & diagnostic state
// ============================================================

// File-scope frame counter; incremented once per top-level frame.
// Used by the drain loop for cache-freshness checks.
uint32_t g_frameCounter = 0;  // extern in OcclusionInternal.h
FrameDiag g_diag;             // per-frame + session diagnostics (FrameDiag.h)
// Cell pointer across frames. Cell change -> wipe g_caches.land and
// g_caches.drain (NI::Node*/NI::AVObject* recycle in the new cell).
static TES3::Cell* g_lastCell = nullptr;
// Cell-cross profiling (Configuration::OcclusionLogCellCross). Set to a
// small frame budget when the cell changes; the per-frame stats line is
// emitted while it counts down, capturing the cross + the re-population
// frames where the caches refill on the miss path. g_diag.cellWipeUs times the
// cache-wipe itself; g_diag.lastFrameDeltaUs is this frame's wall time.

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
// EMA = (prev * 7 + sample) >> 3 - ~6-frame half-life. Fields in BudgetState.h.
BudgetState g_budget;

static inline uint64_t elapsedUsSince(std::chrono::steady_clock::time_point t0) {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0)
            .count());
}

// EMA + budget decisions live in profiling:: now; alias keeps the local
// call sites terse.
using profiling::emaUpdate;
// Phase 3.2: time spent in threadpool Flush() barrier before drain.
// Only populated when async mode is active; zero otherwise.
// Time inside the threadpool's WakeThreads() spin. Should be ~0 every
// frame; non-trivial values mean a worker didn't reach its suspended
// state from the previous SuspendThreads(). max-seen kept lifetime
// so a single outlier shows up in the next periodic log.
// Recursion diagnostic: a debugger during a freeze can read g_callDepth
// directly to spot a runaway tree (cycle, broken sentinel) before
// stack overflow.
static uint32_t g_callDepth = 0;

// RAII for g_callDepth. Used exclusively by CullShow_detour.
struct CallDepthGuard {
    CallDepthGuard() {
        ++g_callDepth;
        if (g_callDepth > g_diag.maxCallDepthThisFrame) {
            g_diag.maxCallDepthThisFrame = g_callDepth;
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
// Mirrored in ForensicsWatchdog::stageName().
static volatile uint32_t g_lastStage = 0;

// Published at end of each top-level frame (after g_lastStage = 13).
// Forensics watchdog reads this for "time since last clean exit."
static std::atomic<uint64_t> g_lastFrameEndTimeMs{0};

// Frame-time sampling for per-window FPS logging. Microsecond precision
// (the existing watchdog timestamp is millisecond, too coarse for a
// per-frame delta). `g_diag.windowFrameTimeUs` is the running sum since the
// last log emission, `g_diag.windowFrameCount` the matching frame count;
// divide for the window average and zero both inside the log block.
static uint64_t g_prevFrameEndUs = 0;

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
::CullingThreadpool* g_threadpool = nullptr;  // extern in OcclusionInternal.h
bool g_asyncThisFrame = false;                // extern in OcclusionInternal.h

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

// The drain runs serially. A parallel drain was built, measured and
// abandoned; lessons/parallel-drain-lessons-learned.md in the moreFPS docs
// records why, and is the thing to read before considering it again. The
// commented-out declarations that used to sit here were a revival checklist
// for a procedure that is no longer safe to follow, so they are gone.

// ============================================================
// Forensics watchdog - read accessor
// ============================================================

// Sole bridge to ForensicsWatchdog.cpp. Defined here so the
// hot-path statics stay internal-linkage. Called once per ~250ms;
// no synchronisation - single loads, post-mortem-grade coherence.
forensics::Snapshot forensics::captureSnapshot() {
    forensics::Snapshot s{};
    s.stage = g_lastStage;
    s.callDepth = g_callDepth;
    s.maxCallDepthSession = g_diag.maxCallDepthSession;
    s.frame = g_frameCounter;
    s.lastFrameEndMs = g_lastFrameEndTimeMs.load(
        std::memory_order_relaxed);
    s.asyncThisFrame = g_asyncThisFrame;
    s.threadpoolAlive = g_threadpool != nullptr;
    return s;
}

// ============================================================
// Deferred-display queue & timing helpers
// ============================================================

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
    // Resolved at deferral, where the traversal has already asked the same
    // question to decide occluder eligibility. Carrying the answer saves the
    // drain a hash lookup per occludee per frame, and keeps drain phase 1
    // read-only against the caches.
    bool isTerrain;
};
static std::vector<PendingDisplay> g_pendingDisplays;

// Deferred-occluder queue for the optional front-to-back submission order
// (OcclusionOccluderFrontToBack). When enabled, rasterizeTriShape builds the
// world geometry during traversal but records the ready-to-submit occluder here
// instead of rasterising inline; the detour then sorts near-to-far and submits
// after traversal. Front-to-back lets MOC early-reject occluded-occluder
// triangles against the accumulating HiZ. Trade-off: it forfeits the async
// traverse/rasterize overlap (workers can only start after traversal), so it is
// a measure-it in async mode and a clear win in sync mode. cache points into
// g_caches.occluder (unordered_map node addresses are stable across insertions,
// valid until cell-change wipe - same lifetime guarantee the threadpool relies
// on). Reused across frames; cleared after the submit loop.
struct PendingOccluder {
    const OccluderCacheEntry* cache;
    float dist2;  // squared eye->worldBoundOrigin distance, sort key
};
static std::vector<PendingOccluder> g_pendingOccluders;

// Drain phase-1 verdict slots, populated by classifyDrainRange and
// consumed by phase 2.
enum class DrainVerdict : uint8_t {
    Visible,         // VISIBLE; call display()
    Occluded,        // OCCLUDED; skip display (or tint+display in debug)
    ViewCulled,      // rect collapsed; treat like Visible for display()
    SkipTerrain,     // bypassed TestRect (terrain descendant); call display()
    SkipTiny,        // bypassed TestRect (radius < threshold); call display()
    CachedOccluded,  // g_caches.drain hit; verdict OCCLUDED (only cached verdict)
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
        if (cur == root) {
            result = true;
            break;
        }
    }
    g_caches.terrainMembership.emplace(
        obj, TerrainMembershipEntry{NI::Pointer<NI::AVObject>(obj), result});
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

// Sphere -> NDC-rect + wmin -> TestRect. Projects only the center and
// derives the NDC half-extent + near-surface clip-w from per-frame
// matrix metrics. Tighter than an 8-corner AABB project by ~sqrt3 and
// stable under camera rotation - small-mesh queries don't flicker
// across TestRect's hiZ thresholds.
//

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
    // Phase budget gate. Compares against g_stats.rasterizeTimeUs (sum
    // of completed RenderTriangles SIMD time), NOT wall-clock-since-
    // frame-start. The latter would include cullShowBody traversal
    // between calls and falsely trip every frame.
    if (g_budget.skipRasterizeThisFrame) {
        g_budget.rasterizeBudgetTrips = 1;
        return false;
    }
    if (profiling::spikeClipTripped(g_stats.rasterizeTimeUs, g_budget.rasterizeBudgetUsEffective)) {
        g_budget.rasterizeBudgetTrips = 1;
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
        ++g_stats.skippedTriCount;
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
    const bool xfStale = !cache.geomResolved || std::memcmp(cache.xfData, &xf, sizeof(cache.xfData)) != 0;
    if (xfStale) {
        // Measure the vertex transform + index expansion: dominates the
        // occluder re-population on a cell cross (occVertVerts).
        ScopedUsAccumulator transformTimer(g_stats.occluderTransformUs);
        const auto& R = xf.rotation;
        const auto& T = xf.translation;
        const float s = xf.scale;

        if (g_frame.logEnabled) {
            ++g_stats.occluderVertexCalls;
            g_stats.occluderVertexVerts += vertexCount;
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
        cache.minX = minX;
        cache.minY = minY;
        cache.minZ = minZ;
        cache.maxX = maxX;
        cache.maxY = maxY;
        cache.maxZ = maxZ;

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
    const int thinAxes = (dx < minDim ? 1 : 0) + (dy < minDim ? 1 : 0) + (dz < minDim ? 1 : 0);
    if (thinAxes >= 2) {
        ++g_stats.skippedThin;
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
            ++g_stats.skippedInside;
            return false;
        }
    }

    if (cache.outTriCount == 0) {
        return false;
    }

    // Front-to-back: geometry is built (above); defer the submit so the detour
    // can sort near-to-far and rasterise after traversal. occluderTriangles is
    // counted at submit time so the budget-clipped tail isn't over-counted.
    if (g_frame.occluderFrontToBack) {
        const auto& o = shape->worldBoundOrigin;
        const float dx = o.x - eye.x, dy = o.y - eye.y, dz = o.z - eye.z;
        g_pendingOccluders.push_back({&cache, dx * dx + dy * dy + dz * dz});
        return true;
    }

    // VertexLayout(12, 4, 8): stride 12, y@4, z@8 - packed float[3].
    // Winding from g_frame.occluderWinding, which is BACKFACE_CW whenever
    // OcclusionOccluderCCWOnly is set - the config's default since 1.4.0. The
    // struct default below it is BACKFACE_NONE because
    // NIF winding isn't guaranteed consistent; OcclusionOccluderCCWOnly
    // trades the ~1% CW-wound meshes (dropped, safe) for half the raster.
    if (g_asyncThisFrame) {
        ScopedUsAccumulator t(g_stats.rasterizeTimeUs);
        g_threadpool->RenderTriangles(cache.worldVerts.data(), cache.indices.data(),
                                      static_cast<int>(cache.outTriCount),
                                      g_frame.occluderWinding,
                                      ::MaskedOcclusionCulling::CLIP_PLANE_ALL);
        ++g_stats.asyncJobsQueued;
    } else {
        ScopedUsAccumulator t(g_stats.rasterizeTimeUs);
        g_msoc->RenderTriangles(cache.worldVerts.data(), cache.indices.data(),
                                static_cast<int>(cache.outTriCount), g_worldToClip,
                                g_frame.occluderWinding,
                                ::MaskedOcclusionCulling::CLIP_PLANE_ALL,
                                ::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
    }
    g_stats.occluderTriangles += cache.outTriCount;
    ++g_stats.rasterizedAsOccluder;
    g_stats.maskHasOccluders = true;
    return true;
}

// Drain the deferred front-to-back occluder queue: sort near-to-far and submit.
// Called by the detour after cullShowBody, before the threadpool Flush. Mirrors
// rasterizeTriShape's submit (winding, layout, async vs direct) exactly; the
// budget spike-clip still applies so a dense frame bails the tail. Occluders
// are counted here (not at record time) so the counter matches what rasterised.
static void submitPendingOccluders() {
    if (g_pendingOccluders.empty()) return;
    std::sort(g_pendingOccluders.begin(), g_pendingOccluders.end(),
              [](const PendingOccluder& a, const PendingOccluder& b) { return a.dist2 < b.dist2; });
    for (const auto& po : g_pendingOccluders) {
        if (g_budget.skipRasterizeThisFrame) break;
        if (profiling::spikeClipTripped(g_stats.rasterizeTimeUs, g_budget.rasterizeBudgetUsEffective)) {
            g_budget.rasterizeBudgetTrips = 1;
            break;
        }
        const OccluderCacheEntry& c = *po.cache;
        if (g_asyncThisFrame) {
            ScopedUsAccumulator t(g_stats.rasterizeTimeUs);
            g_threadpool->RenderTriangles(c.worldVerts.data(), c.indices.data(),
                                          static_cast<int>(c.outTriCount),
                                          g_frame.occluderWinding,
                                          ::MaskedOcclusionCulling::CLIP_PLANE_ALL);
            ++g_stats.asyncJobsQueued;
        } else {
            ScopedUsAccumulator t(g_stats.rasterizeTimeUs);
            g_msoc->RenderTriangles(c.worldVerts.data(), c.indices.data(),
                                    static_cast<int>(c.outTriCount), g_worldToClip,
                                    g_frame.occluderWinding,
                                    ::MaskedOcclusionCulling::CLIP_PLANE_ALL,
                                    ::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
        }
        g_stats.occluderTriangles += c.outTriCount;
        ++g_stats.rasterizedAsOccluder;
        g_stats.maskHasOccluders = true;
    }
    g_pendingOccluders.clear();
}

// ============================================================
// Main scene-graph traversal
// ============================================================

// Reimplements CullShow at 0x6EB480 with MSOC query + occluder
// rasterisation wedged between the frustum test and Display. The
// detour JMPs here. Behaviour matches the engine 1:1 when
// g_msocActive is false.
static void __fastcall cullShowBody(NI::AVObject* self, void* /*edx*/, NI::Camera* camera) {
    if (g_msocActive) ++g_stats.recursiveCalls;
    if (self->getAppCulled()) {
        if (g_msocActive) ++g_stats.recursiveAppCulled;
        return;
    }

    // Hierarchical frustum test, mirroring the engine's loop at
    // 0x6EB4B7. setBits tracks bits flipped in
    // usedCullingPlanesBitfield so they're unflipped before return
    // (engine's LABEL_10 cleanup).
    uint32_t setBits[4] = {0, 0, 0, 0};
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
        const float d = plane->x * self->worldBoundOrigin.x + plane->y * self->worldBoundOrigin.y + plane->z * self->worldBoundOrigin.z - plane->w;
        if (d <= -boundRadius) {
            if (g_msocActive) ++g_stats.recursiveFrustumCulled;
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
            // Terrain leaves are never per-leaf occluders. In Raster/Horizon
            // mode they're already in the buffer as merged per-Land
            // submissions (re-rasterising here would duplicate them); in Off
            // mode terrain is intentionally absent from the mask (the MCM
            // "Off: no terrain in the occlusion mask"). Either way, skip them.
            const bool isTerrainLeaf = isLandscapeDescendant(self, g_worldLandscapeRoot);
            if (!isTerrainLeaf && boundRadius >= g_frame.occluderRadiusMin && boundRadius <= g_frame.occluderRadiusMax) {
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
                    ++g_stats.skippedAlpha;
                } else if (cacheEntry.stencil) {
                    ++g_stats.skippedStencil;
                } else {
                    const auto& eye = camera->worldTransform.translation;
                    // rasterizedAsOccluder is incremented at the submit
                    // sites, not here: with front-to-back the call below only
                    // queues, and a budget bail can drop the tail. didRasterise
                    // stays record-time - the tint marks what was chosen as an
                    // occluder, which is what a debugger wants to see.
                    if (rasterizeTriShape(static_cast<NI::TriBasedGeometry*>(self), eye, cacheEntry)) {
                        didRasterise = true;
                        if (g_frame.tintOccluder) {
                            debugtint::tintOccluder(self);
                        }
                    }
                }
            }
            g_pendingDisplays.push_back({self, camera, didRasterise, isTerrainLeaf});
            ++g_stats.deferred;
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

// Drain phase 1 - classify [lo, hi) of g_pendingDisplays into g_drainSlots.
// Phase 2 owns every write to g_caches.drain, the counters, display() and the
// tints, and the only counter touched here is the atomic g_stats.queryNearClip.
//
// One deliberate exception: the occludee box cache inserts here, on miss. It is
// keyed per geometry and computing an object-space AABB is not something to do
// twice, so it stays lazy. Phase 1 is therefore "writes nothing phase 2 depends
// on", not "read-only" - the earlier comment claimed the latter, which was
// never true.
// Optional tighter occludee test (OcclusionOccludeeBoxTest). Returns true if
// the occludee's object-space vertex AABB, transformed to world, reads fully
// behind the live mask. The object AABB is computed once per geometry and
// cached (shared across instances via NIF dedup); per instance we just
// transform the 8 corners. Called only on sphere-VISIBLE occludees, so it
// only ever upgrades Visible -> Occluded - never the reverse.
static bool occludeeBoxOccluded(NI::AVObject* shape) {
    // Deferred occludees are always NiTriBasedGeom (see the cullShowBody
    // deferral, which casts the same shape for rasterizeTriShape).
    auto* geom = static_cast<NI::TriBasedGeometry*>(shape);
    // Skinned meshes carry bind-pose vertices that skinInstance->deform()
    // repositions per frame from the bone palette. The model AABB *
    // worldTransform would bound the BIND POSE, not the animated geometry, so
    // the box could sit somewhere the limb no longer is and wrongly occlude it
    // (missing hands/legs on NPCs and creatures). Skip them and keep the
    // sphere verdict - the same guard rasterizeTriShape uses to refuse skinned
    // occluders.
    if (geom->skinInstance) {
        return false;
    }
    auto data = geom->getModelData();
    if (!data) {
        return false;
    }
    const void* key = data.get();

    float mn[3], mx[3];
    auto it = g_caches.occludeeBox.find(key);
    if (it != g_caches.occludeeBox.end()) {
        if (g_frame.logEnabled) ++g_caches.occludeeBoxHits;
        const auto& e = it->second;
        mn[0] = e.minX;
        mn[1] = e.minY;
        mn[2] = e.minZ;
        mx[0] = e.maxX;
        mx[1] = e.maxY;
        mx[2] = e.maxZ;
    } else {
        if (g_frame.logEnabled) ++g_caches.occludeeBoxMisses;
        const unsigned short n = data->getActiveVertexCount();
        if (n == 0 || data->vertex == nullptr) {
            return false;
        }
        float a[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
        float b[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        for (unsigned short i = 0; i < n; ++i) {
            const auto& v = data->vertex[i];
            if (v.x < a[0]) a[0] = v.x;
            if (v.x > b[0]) b[0] = v.x;
            if (v.y < a[1]) a[1] = v.y;
            if (v.y > b[1]) b[1] = v.y;
            if (v.z < a[2]) a[2] = v.z;
            if (v.z > b[2]) b[2] = v.z;
        }
        // `data` is an NI::Pointer, so storing it here is what keeps the key
        // address alive and unique for as long as the entry exists.
        g_caches.occludeeBox[key] = {a[0], a[1], a[2], b[0], b[1], b[2], data};
        mn[0] = a[0];
        mn[1] = a[1];
        mn[2] = a[2];
        mx[0] = b[0];
        mx[1] = b[1];
        mx[2] = b[2];
    }

    // Transform the 8 object-AABB corners to world via the shape's transform
    // (same R*v*s+T as rasterizeTriShape).
    const auto& xf = shape->worldTransform;
    const auto& R = xf.rotation;
    const auto& T = xf.translation;
    const float s = xf.scale;
    float corners[24];
    for (int c = 0; c < 8; ++c) {
        const float ox = (c & 1) ? mx[0] : mn[0];
        const float oy = (c & 2) ? mx[1] : mn[1];
        const float oz = (c & 4) ? mx[2] : mn[2];
        const float rx = R.m0.x * ox + R.m0.y * oy + R.m0.z * oz;
        const float ry = R.m1.x * ox + R.m1.y * oy + R.m1.z * oz;
        const float rz = R.m2.x * ox + R.m2.y * oy + R.m2.z * oz;
        corners[c * 3 + 0] = rx * s + T.x;
        corners[c * 3 + 1] = ry * s + T.y;
        corners[c * 3 + 2] = rz * s + T.z;
    }
    return testBoxVisible(corners) == ::MaskedOcclusionCulling::OCCLUDED;
}

static void classifyDrainRange(size_t lo, size_t hi) {
    // Captured here, not at drainPendingDisplays entry, so the
    // spike-clip measures classify-only elapsed (excluding phase 2's
    // vanilla D3D8 display() calls).
    g_budget.classifyPhaseStart = std::chrono::steady_clock::now();

    // Predictive skip: EMA over 2x budget -> skip TestRect entirely
    // this frame, mark all Visible. Equivalent to EnableMSOC=false
    // for one frame, except mask build still happens so recovery
    // momentum is preserved once the EMA drops back.
    if (g_budget.skipClassifyThisFrame) {
        g_budget.classifyBudgetTrips = 1;
        for (size_t i = lo; i < hi; ++i) {
            g_drainSlots[i].verdict = DrainVerdict::Visible;
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
    const bool budgetActive = (g_budget.classifyBudgetUsEffective > 0);
    constexpr size_t kCheckMask = 31;

    for (size_t i = lo; i < hi; ++i) {
        const auto& p = g_pendingDisplays[i];
        auto& slot = g_drainSlots[i];
        slot.ranTestRect = false;

        if (budgetActive && i > lo && (i & kCheckMask) == 0) {
            if (elapsedUsSince(g_budget.classifyPhaseStart) > g_budget.classifyBudgetUsEffective) {
                g_budget.classifyBudgetTrips = 1;
                for (size_t j = i; j < hi; ++j) {
                    g_drainSlots[j].verdict = DrainVerdict::Visible;
                    g_drainSlots[j].ranTestRect = false;
                }
                return;
            }
        }

        if (skipTerrainEnabled && p.isTerrain) {
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
                // Loose sphere says visible; optionally refine with the
                // tighter object-space box before committing to display().
                if (g_frame.occludeeBoxTest && occludeeBoxOccluded(p.shape)) {
                    slot.verdict = DrainVerdict::Occluded;
                    ++g_stats.boxOccluded;
                } else {
                    slot.verdict = DrainVerdict::Visible;
                }
                break;
            case ::MaskedOcclusionCulling::OCCLUDED:
                slot.verdict = DrainVerdict::Occluded;
                break;
            case ::MaskedOcclusionCulling::VIEW_CULLED:
                slot.verdict = DrainVerdict::ViewCulled;
                break;
            default:
                slot.verdict = DrainVerdict::Visible;  // conservative fallback
                break;
        }
    }
}

// Drain the deferred-display queue. Re-tests each entry against the
// now-complete depth buffer and displays the visible ones. Must run
// before g_msocActive clears so counters land in this frame's log.
//
// Two-phase: classifyDrainRange does read-only verdict computation;
// the loop below applies all writes (counters, cache, tints,
// display()). Bit-exact counter parity with the pre-refactor serial
// loop is the acceptance criterion.
static void drainPendingDisplays() {
    ScopedUsAccumulator t(g_stats.drainPhaseTimeUs);
    // Fast path: nothing in the mask this frame -> depth buffer is cleared
    // -> every TestRect would return VISIBLE. Skip the loop.
    //
    // maskHasOccluders is set by every submit path, which is the point of it.
    // This test used to be "rasterizedAsOccluder == 0 && aggregateTerrainLands
    // == 0", and the horizon curtain increments neither - so in Horizon mode,
    // where the curtain is usually the only thing in the mask, the drain
    // skipped every frame and the curtain culled nothing at all.
    if (!g_stats.maskHasOccluders) {
        ScopedUsAccumulator tt(g_stats.drainDisplayUs);
        for (const auto& p : g_pendingDisplays) {
            p.shape->vTable.asAVObject->display(p.shape, p.camera);
        }
        g_pendingDisplays.clear();
        return;
    }

    const size_t n = g_pendingDisplays.size();

    // The drain is serial; see the note beside g_frame for why the
    // parallel version was abandoned.

    // Phase 1: classify (currently serial).
    g_drainSlots.resize(n);

    const auto classifyT0 = std::chrono::steady_clock::now();
    classifyDrainRange(0, n);
    g_stats.classifyUs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - classifyT0)
            .count());

    // Shared handler for OCCLUDED verdicts (fresh + cached paths).
    auto handleOccluded = [&](const PendingDisplay& p) {
        if (g_frame.tintOccluded) {
            debugtint::tintOccluded(p.shape);
            ScopedUsAccumulator tt(g_stats.drainDisplayUs);
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
            ++g_stats.skippedTerrain;
            ScopedUsAccumulator tt(g_stats.drainDisplayUs);
            p.shape->vTable.asAVObject->display(p.shape, p.camera);
            continue;
        }
        if (s.verdict == DrainVerdict::SkipTiny) {
            ++g_stats.skippedTesteeTiny;
            ScopedUsAccumulator tt(g_stats.drainDisplayUs);
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
            ++g_stats.queryTested;
            if (g_frame.temporalCoherenceFrames > 0) {
                ++g_caches.drainMisses;
                // Cache only OCCLUDED - VISIBLE/VIEW_CULLED still call
                // display() this frame, so caching them adds stale-
                // pointer surface for no hit-path win.
                if (s.verdict == DrainVerdict::Occluded) {
                    auto& e = g_caches.drain[p.shape];
                    e.shapePtr = p.shape;
                    e.lastQueryFrame = g_frameCounter;
                    e.boundOriginX = p.shape->worldBoundOrigin.x;
                    e.boundOriginY = p.shape->worldBoundOrigin.y;
                    e.boundOriginZ = p.shape->worldBoundOrigin.z;
                    e.boundRadius = p.shape->worldBoundRadius;
                }
            }
        }

        if (s.verdict == DrainVerdict::Occluded) {
            if (s.ranTestRect) ++g_stats.queryOccluded;
            handleOccluded(p);
            continue;
        }
        if (s.verdict == DrainVerdict::ViewCulled) {
            if (s.ranTestRect) ++g_stats.queryViewCulled;
        }
        // Skip Tested-tint when the main pass already applied
        // Occluder; last-write-wins would flip yellow -> green.
        else if (g_frame.tintTested && !p.rasterisedAsOccluder) {
            debugtint::tintTested(p.shape);
        }
        {
            ScopedUsAccumulator tt(g_stats.drainDisplayUs);
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
        if (inMenuMode) ++g_stats.skippedMenuMode;
        // Multi-pass guard. The engine may fire more than one main-
        // camera CullShow per renderMainScene (shadow caster, main,
        // 1st-person subtree). Each would ClearBuffer and clobber
        // the last, so we only open isTopLevel on the FIRST entry
        // per scene. Later passes still render correctly (vanilla
        // body), they just don't re-build the mask.
        const bool alreadyBuiltThisScene = g_diag.isTopLevelFiresThisScene > 0;
        // Outer-entry counter: ungated, this would tick on every
        // recursive descent (hundreds per frame). The !g_msocActive
        // gate restricts to true outer entries - value > 1 means
        // the engine genuinely issued multiple main-camera passes.
        if (camera == mainCamera && !inMenuMode && !g_msocActive) {
            ++g_diag.mainCamCullShowAttemptsThisScene;
        }
        if (camera == mainCamera && !inMenuMode && !alreadyBuiltThisScene) {
            // dh null on main menu / load screen - no scene to cull.
            auto dh = TES3::DataHandler::get();
            if (dh) {
                const bool isInterior = dh->currentCell && dh->currentCell->getIsInterior();
                // EnableMSOC is the master runtime gate. Reconcile
                // resource state first: a fresh toggle-on allocates
                // g_msoc + threadpool now; toggle-off tears them
                // down (joins workers, frees ~57MB). Returns false
                // on alloc failure -> vanilla cullShowBody this frame.
                const bool resourcesLive = ensureMSOCResourcesMatchConfig();
                const bool sceneEnabled = resourcesLive && (isInterior
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
                } else {
                    ++g_stats.skippedSceneGate;
                }
            }
        }
    }

    if (isTopLevel) {
        ++g_diag.isTopLevelFiresThisScene;
        g_lastStage = 1;
        // Latch async mode once per frame so a mid-frame Lua toggle
        // can't split submissions between threadpool and direct MOC.
        g_asyncThisFrame = g_threadpool && Configuration::OcclusionAsyncOccluders;
        if (g_asyncThisFrame) {
            // Wake workers ~100us before the first RenderTriangles.
            // ClearBuffer's implicit Flush retires last frame's
            // stragglers.
            g_lastStage = 2;
            {
                ScopedUsAccumulator t(g_diag.wakeThreadsTimeUs);
                g_threadpool->WakeThreads();
            }
            g_lastStage = 3;
            g_threadpool->ClearBuffer();
        } else {
            g_lastStage = 3;
            g_msoc->ClearBuffer();
        }
        // Cell-change cache wipe. Must come AFTER ClearBuffer/Flush
        // so the threadpool has consumed any cached buffers
        // referenced by last-frame work - only then is it safe to
        // free them. Both caches key off pointers a cell load can
        // recycle (per-Land NiNode*, shape AVObject*).
        g_lastStage = 4;
        g_diag.cellWipeUs = 0;
        if (activeCell != g_lastCell) {
            {
                // Time just the wipe - the one cross cost the per-frame
                // timers don't already capture.
                ScopedUsAccumulator wipeTimer(g_diag.cellWipeUs);
                g_caches.wipeForCellChange();
                // Releases our NI::Pointer refs on outgoing-cell shapes
                // so they can actually be freed. Surviving shapes
                // (player, inventory) re-enter on next tint.
                debugtint::clearClones();
            }
            g_lastCell = activeCell;
            ++g_diag.cellChanges;
            // Profile this cross + the next few frames as the caches
            // refill on the miss path (the spike isn't just frame 0).
            g_diag.cellCrossLogFrames = 8;
        }
        g_stats.recursiveCalls = 0;
        g_stats.recursiveAppCulled = 0;
        g_stats.recursiveFrustumCulled = 0;
        g_stats.rasterizedAsOccluder = 0;
        g_stats.occluderTriangles = 0;
        g_stats.maskHasOccluders = false;
        g_stats.asyncJobsQueued = 0;
        g_stats.skippedInside = 0;
        g_stats.skippedThin = 0;
        g_stats.skippedAlpha = 0;
        g_stats.skippedStencil = 0;
        g_stats.queryTested = 0;
        g_stats.queryOccluded = 0;
        g_stats.queryViewCulled = 0;
        g_stats.boxOccluded = 0;
        g_stats.queryNearClip.store(0, std::memory_order_relaxed);
        g_stats.deferred = 0;
        g_stats.skippedTriCount = 0;
        g_stats.skippedTesteeTiny = 0;
        g_stats.skippedSceneGate = 0;
        g_stats.skippedTerrain = 0;
        g_stats.aggregateTerrainLands = 0;
        g_stats.aggregateTerrainTris = 0;
        g_stats.aggregateTerrainUs = 0;
        // LAYER-A-HORIZON: per-frame reset for the Horizon-mode counters.
        g_stats.horizonBuildUs = 0;
        g_stats.horizonRasterUs = 0;
        g_stats.horizonLandsFed = 0;
        g_stats.horizonVertsFed = 0;
        g_stats.horizonColumnsTouched = 0;
        g_stats.horizonCurtainTris = 0;
        g_stats.horizonAdaptiveEpsD = 0.0f;
        g_caches.terrainMembershipHits = 0;
        g_caches.terrainMembershipMisses = 0;
        g_stats.classifyOccluderCalls = 0;
        g_stats.classifyOccluderSteps = 0;
        g_stats.occluderVertexCalls = 0;
        g_stats.occluderVertexVerts = 0;
        g_caches.occluderHits = 0;
        g_caches.occluderMisses = 0;
        g_caches.occludeeBoxHits = 0;
        g_caches.occludeeBoxMisses = 0;
        g_caches.landHits = 0;
        g_caches.landMisses = 0;
        g_caches.landEvictions = 0;
        g_caches.drainHits = 0;
        g_caches.drainMisses = 0;
        ++g_frameCounter;
        g_lastStage = 5;
        // Age-prune the drain cache. Window 2*N frames - older
        // entries can't be reused anyway. N=0 drops the whole map.
        {
            const unsigned int tcFrames = Configuration::OcclusionTemporalCoherenceFrames;
            if (tcFrames == 0) {
                if (!g_caches.drain.empty()) g_caches.drain.clear();
            } else {
                const uint32_t maxAge = tcFrames * 2;
                for (auto it = g_caches.drain.begin(); it != g_caches.drain.end();) {
                    if (g_frameCounter - it->second.lastQueryFrame > maxAge) {
                        it = g_caches.drain.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
        }
        g_stats.rasterizeTimeUs = 0;
        g_stats.occluderTransformUs = 0;
        g_stats.drainPhaseTimeUs = 0;
        g_stats.classifyUs = 0;
        g_stats.drainDisplayUs = 0;
        g_diag.asyncFlushTimeUs = 0;
        // Per-frame counters reset; g_max*Session are lifetime peaks.
        g_diag.wakeThreadsTimeUs = 0;
        g_diag.maxCallDepthThisFrame = 0;

        // Read budgets once per frame so a mid-frame configure()
        // doesn't split a phase across two regimes. 2x threshold so
        // one-shot spikes don't trigger sustained skipping. 0 = off.
        g_budget.rasterizeBudgetUsEffective = Configuration::OcclusionRasterizeBudgetUs;
        g_budget.classifyBudgetUsEffective = Configuration::OcclusionClassifyBudgetUs;
        g_budget.skipRasterizeThisFrame =
            profiling::predictiveSkip(g_budget.rasterizeEmaUs, g_budget.rasterizeBudgetUsEffective);
        g_budget.skipClassifyThisFrame =
            profiling::predictiveSkip(g_budget.classifyEmaUs, g_budget.classifyBudgetUsEffective);
        g_budget.rasterizeBudgetTrips = 0;
        g_budget.classifyBudgetTrips = 0;
        g_lastStage = 6;
        uploadCameraTransform(camera);
        if (g_asyncThisFrame) {
            g_lastStage = 7;
            // Must run after uploadCameraTransform - copies the
            // matrix into the threadpool's state ring buffer.
            g_threadpool->SetMatrix(g_worldToClip);
        }
        g_msocActive = true;

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
        // Front-to-back: submit the occluders deferred during traversal, sorted
        // near-to-far, before the Flush. No-op unless OcclusionOccluderFrontToBack
        // is on (the queue stays empty otherwise).
        submitPendingOccluders();

        // Aggregate-terrain also goes through the threadpool, so
        // the Flush gate must include it. Otherwise a terrain-only
        // frame would SuspendThreads() with queued work in the
        // ring - suspected cause of a freeze on menu entry.
        const bool hadAsyncWork = g_asyncThisFrame && g_stats.asyncJobsQueued > 0;
        if (hadAsyncWork) {
            g_lastStage = 10;
            // Barrier: blocks until every queued RenderTriangles is
            // fully rasterised. Only after return is it safe to
            // TestRect.
            ScopedUsAccumulator t(g_diag.asyncFlushTimeUs);
            g_threadpool->Flush();
        }
        g_lastStage = 11;
        drainPendingDisplays();
        // Debug overlay: mirror the finished mask into the engine texture the
        // HUD element samples. Gated on the MCM toggle, and internally a no-op
        // until Lua has asked for the texture, so an off overlay pays nothing.
        if (g_frame.maskOverlay) {
            updateMaskOverlay();
        }
        if (g_asyncThisFrame) {
            g_lastStage = 12;
            // Workers back to low-overhead sleep until next frame.
            g_threadpool->SuspendThreads();
            g_asyncThisFrame = false;
        }
        g_msocActive = false;
        // Lifetime peaks survive across frames so single outliers
        // show up in the next periodic log.
        if (g_diag.wakeThreadsTimeUs > g_diag.maxWakeThreadsUsSession) {
            g_diag.maxWakeThreadsUsSession = g_diag.wakeThreadsTimeUs;
        }
        if (g_diag.maxCallDepthThisFrame > g_diag.maxCallDepthSession) {
            g_diag.maxCallDepthSession = g_diag.maxCallDepthThisFrame;
        }

        // EMAs for next frame's predictive-skip gate. Samples are
        // MSOC-only - including non-MSOC work would have the gate
        // trigger on vanilla render slowness.
        // Async frames pay for rasterization twice over on this thread: the
        // enqueue, and the Flush barrier waiting for the workers. Sampling only
        // the enqueue made the EMA read a few hundred microseconds on a frame
        // that actually cost a millisecond, so predictive skip never tripped
        // under async no matter how dense the scene. The mid-phase spike clip
        // still sees main-thread time only - it cannot observe worker time.
        g_budget.rasterizeEmaUs = emaUpdate(
            g_budget.rasterizeEmaUs,
            g_stats.rasterizeTimeUs + (g_asyncThisFrame ? g_diag.asyncFlushTimeUs : 0));
        g_budget.classifyEmaUs = emaUpdate(g_budget.classifyEmaUs, g_stats.classifyUs);
        g_budget.rasterizeBudgetTripsSession += g_budget.rasterizeBudgetTrips;
        g_budget.classifyBudgetTripsSession += g_budget.classifyBudgetTrips;
        g_lastStage = 13;
        // Publish frame-end timestamp for the watchdog. If the next
        // frame freezes mid-body, MSOC.forensics.txt shows
        // sinceFrameEndMs growing - freeze is inside MSOC. If it
        // stays near 250ms during a freeze, the freeze is upstream.
        const auto frameEndNowUs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now()
                    .time_since_epoch())
                .count());
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
                g_diag.lastFrameDeltaUs = frameEndNowUs - g_prevFrameEndUs;
                g_diag.windowFrameTimeUs += g_diag.lastFrameDeltaUs;
                ++g_diag.windowFrameCount;
            }
            g_prevFrameEndUs = frameEndNowUs;
        } else {
            // Reset so a flip-on later doesn't fold in a stale delta
            // spanning the off-period.
            g_prevFrameEndUs = 0;
        }

        emitPerFrameStatsLine();
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
        g_diag.isTopLevelFiresThisScene = 0;
        g_diag.mainCamCullShowAttemptsThisScene = 0;
    }
    renderMainScene_original();
    if (!wasActive) {
        if (g_diag.isTopLevelFiresThisScene > g_diag.maxIsTopLevelFiresSession) {
            g_diag.maxIsTopLevelFiresSession = g_diag.isTopLevelFiresThisScene;
        }
        if (g_diag.mainCamCullShowAttemptsThisScene > g_diag.maxMainCamCullShowAttemptsSession) {
            g_diag.maxMainCamCullShowAttemptsSession = g_diag.mainCamCullShowAttemptsThisScene;
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
        if (w < 64) w = 64;
        if (h < 32) h = 32;
        if (w > 2048) w = 2048;
        if (h > 1024) h = 1024;
        kMsocWidth = w;
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
    } else {
        log << "MSOC: starting with EnableMSOC=false; resources will be allocated on first MCM toggle-on." << std::endl;
    }

    // Restart-only - see ForensicsWatchdog.h. Reads the gate
    // before configure() runs, so MCM edits only take effect on
    // next launch.
    forensics::spawnIfEnabled(log);

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
        0x41C08E,  // TES3Game::renderNextFrame
        0x42E655,  // WorldControllerRenderTarget::takeScreenshot
        0x4B50FF,  // TES3File_static::createSaveScreenshot
    };
    for (auto site : kRenderMainSceneCallSites) {
        if (se::memory::genCallEnforced(site, 0x41C400, wrapperAddr)) {
            ++renderMainSceneInstalled;
        } else {
            log << "MSOC: failed to wrap renderMainScene call at 0x"
                << std::hex << site << std::dec << std::endl;
        }
    }

    log << "MSOC: CullShow detour installed at 0x6EB480; wrapped "
        << renderMainSceneInstalled << " / 3 renderMainScene call sites ("
        << kMsocWidth << "x" << kMsocHeight << " tile buffer)." << std::endl;
}

}  // namespace msoc::patch::occlusion
