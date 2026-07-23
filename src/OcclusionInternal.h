#pragma once

// Internal shared state for the occlusion patch, exposed so subsystem
// translation units (QueryApi.cpp, ...) can link against the core state that
// OcclusionPass.cpp defines. This is the plugin's private seam, not a
// public API - the public C surface lives in OcclusionApi.h.
//
// The single definitions of the extern globals below live in
// OcclusionPass.cpp.

#include "MaskedOcclusionCulling.h"
#include "CullingThreadpool.h"  // ::CullingThreadpool for g_threadpool
#include "FrameConfig.h"
#include "ClipMath.h"
#include "OcclusionCaches.h"  // cache types + extern g_caches
#include "FrameStats.h"       // per-frame diagnostic counters/timers + g_stats
#include "BudgetState.h"      // phase-budget controller + g_budget
#include "FrameDiag.h"        // per-frame + session diagnostic bookkeeping + g_diag

#include "NIPoint3.h"  // NI::Point3 for the live::testSphere decl
#include "NICamera.h"  // NI::Camera (cullingPlanes) for the accessors
#include "NIPoint4.h"  // NI::Point4

#include <chrono>
#include <cstdint>
#include <iosfwd>

namespace msoc::occlusion {

// NI::Camera accessors for fields upstream MWSE NICamera.h labels
// `unknown_*` (countCullingPlanes is always 6 for the main world camera;
// usedCullingPlanesBitfield sits just past the inline cullingPlanes[6]).
// Shared by the core-TU occluder frustum test and TerrainAggregation.cpp.
inline int cameraCountCullingPlanes(const NI::Camera* /*cam*/) {
    return 6;
}
inline const NI::Point4* cameraCullingPlane(NI::Camera* cam, int i) {
    return &cam->cullingPlanes[i];
}
inline uint32_t* cameraUsedPlanesMask(NI::Camera* cam) {
    auto* base = reinterpret_cast<char*>(&cam->cullingPlanes[0]);
    return reinterpret_cast<uint32_t*>(base + sizeof(NI::Point4) * 6);
}

// First-of-type alpha/stencil flags from an occluder's ancestor chain;
// alpha/stencil meshes are excluded from the occluder rasterise pass.
// Defined in OccluderClassify.cpp (leaf); shared by core-rasterize + Terrain.
namespace classify {
struct PropertyFlags {
    bool alpha;
    bool stencil;
};
PropertyFlags occluderProperties(NI::AVObject* obj);
}  // namespace classify

// Live per-frame state shared with the subsystem TUs (defined in
// OcclusionPass.cpp).
extern float g_worldToClip[16];  // live world-to-clip (column-major)
extern float g_ndcRadiusX;       // live sphere-projection metrics, set by
extern float g_ndcRadiusY;       // uploadCameraTransform; read by LiveQuery
extern float g_wGradMag;
extern ::CullingThreadpool* g_threadpool;
extern bool g_asyncThisFrame;           // async submit latched this frame
extern NI::Node* g_worldLandscapeRoot;  // DataHandler terrain root

// Snapshot published at the drain-complete buffer swap: the matrix + NDC
// constants the depth data was built with, plus the capture time. The
// external query API projects through these, not the live per-frame matrix.
struct MaskSnapshot {
    float worldToClip[16] = {};
    float ndcRadiusX = 0.0f;
    float ndcRadiusY = 0.0f;
    float wGradMag = 0.0f;
    unsigned long long tickMs = 0;  // GetTickCount64 at swap; 0 = none yet
};

// MOC near-clip w floor; sits below the engine near plane and above the
// numerical noise that explodes NDC after the perspective divide.
inline constexpr float kNearClipW = 1.0f;

// External queries reject snapshots older than this (alt-tab, menu, load).
inline constexpr unsigned long long kSnapshotMaxAgeMs = 200;

// Cross-TU shared state. Defined once in OcclusionPass.cpp.
extern FrameConfig g_frame;                    // per-frame Configuration snapshot
extern MaskSnapshot g_snapshot;                // published snapshot metadata
extern ::MaskedOcclusionCulling* g_msoc_prev;  // swapped snapshot buffer
extern ::MaskedOcclusionCulling* g_msoc;       // live frame buffer
extern unsigned int kMsocWidth;                // mask resolution (latched at install)
extern unsigned int kMsocHeight;
extern uint32_t g_frameCounter;  // top-level frame counter

// Queries against the LIVE mask (g_msoc, current frame's projection).
// The snapshot equivalents - which read g_msoc_prev through the
// published projection - live in QueryApi.cpp; keeping the two behind
// distinct namespaces is deliberate, since confusing them means
// culling against the wrong buffer.
namespace live {
// Sphere test against g_msoc. Defined in LiveQuery.cpp (leaf); called
// by the drain (core) and LightCulling. Returns Intel's CullingResult
// (VISIBLE / OCCLUDED / VIEW_CULLED).
::MaskedOcclusionCulling::CullingResult testSphere(
    const NI::Point3& center, float radius);

// OBB test against g_msoc: corners is 8 world-space (x, y, z) triples.
// Used by the drain's optional occludee box test.
::MaskedOcclusionCulling::CullingResult testBox(const float* corners);
}  // namespace live

// Naked trampoline (NiDX8LightManager::updateLights enabled-read hook),
// defined in LightCulling.cpp; installPatches() takes its address.
void updateLights_enabledRead_hook();

// Terrain aggregation entry points (TerrainAggregation.cpp), called by the
// detour. Raster mode merges each near Land into one submission; Horizon
// mode rasterizes a 1D silhouette curtain.
namespace terrain {
void rasterizeAggregate(NI::Camera* camera);
void rasterizeAggregateHorizon(NI::Camera* camera);
}  // namespace terrain

// Mask resource lifecycle (MaskResources.cpp). create/ensure are called by
// installPatches and the detour's top-of-frame reconcile; destroy by the
// create-failure paths and a toggle-off.
namespace resources {
bool create(std::ostream& log);
void destroy(std::ostream& log);
bool ensureMatchesConfig();
}  // namespace resources

// External-occluder injection (ExternalOccluders.cpp). drain() rasterizes
// queued consumer submissions into the mask (called by the detour);
// clearQueue() drops them on teardown (called by MaskResources). The
// public addOccluder / addPreTransformedOccluder in OcclusionApi.h are
// thin adapters over enqueue* in this namespace.
namespace external {
void drain();
void clearQueue();
}  // namespace external

// Emit the per-frame MSOC diagnostic line (DiagnosticsLog.cpp). Called at the
// tail of the detour; gated internally on the log channels (cold path).
namespace diag {
void emitPerFrameLine();
}  // namespace diag

// Live mask readiness: true once the depth buffer reflects the complete
// vanilla main-scene occluder set; cleared at the next ClearBuffer.
extern bool g_maskReady;

// Live projection forwarder + clip type alias, shared by the query, drain,
// and terrain paths. Header-inline so the hot path still inlines fully.
// (Declared after the externs above since it binds g_worldToClip.)
using ClipXYW = clipmath::ClipXYW;
inline ClipXYW projectWorld(float wx, float wy, float wz) {
    return clipmath::projectWorld(g_worldToClip, wx, wy, wz);
}

// Scoped microsecond accumulator (RAII timer). No-op unless logging is on;
// start is read once at construction so a mid-scope MCM toggle can't flip
// the dtor onto a still-zero start. Binds g_frame, declared above.
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
                    std::chrono::steady_clock::now() - start)
                    .count());
        }
    }
};

}  // namespace msoc::occlusion
