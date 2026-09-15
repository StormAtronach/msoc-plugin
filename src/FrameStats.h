#pragma once

// Per-frame diagnostic counters + phase timers, produced across the occlusion
// subsystems (rasterization, terrain, drain) and consumed by the per-frame
// stats logging in the core TU. Grouped into one owner so the subsystem TUs
// reference a single extern instead of dozens of loose globals.
//
// NOT included here: the budget / EMA / call-depth / frame-window control
// state, which is detour-internal and stays in OcclusionPass.cpp.
//
// The single g_stats instance is defined in OcclusionPass.cpp. Reset
// to zero each top-level frame by the detour; no synchronization beyond the
// render thread except queryNearClip (touched from the threadpool path).

#include <atomic>
#include <cstdint>

namespace msoc::occlusion {

struct FrameStats {
    // Scene-graph traversal.
    uint64_t recursiveCalls = 0;
    uint64_t recursiveAppCulled = 0;
    uint64_t recursiveFrustumCulled = 0;

    // Occluder rasterization.
    uint64_t rasterizedAsOccluder = 0;
    uint64_t occluderTriangles = 0;
    // Set by every path that submits triangles into the mask: per-instance
    // occluders and aggregate terrain. The drain's fast path reads this
    // rather than inferring emptiness from the counters below.
    bool maskHasOccluders = false;
    // Threadpool submits only. The Flush barrier is worth paying for only if
    // something was actually queued; a front-to-back frame that recorded
    // occluders and then bailed on budget has queued nothing.
    uint64_t asyncJobsQueued = 0;
    uint64_t skippedInside = 0;
    uint64_t skippedThin = 0;
    uint64_t skippedAlpha = 0;
    uint64_t skippedStencil = 0;
    uint64_t skippedZTestOccluder = 0;  // depth test off: never an occluder

    // Occludee queries.
    uint64_t queryTested = 0;
    uint64_t queryOccluded = 0;
    uint64_t queryViewCulled = 0;
    uint64_t boxOccluded = 0;  // sphere VISIBLE but tighter box test culled it
    std::atomic<uint64_t> queryNearClip{0};
    uint64_t deferred = 0;
    uint64_t skippedTriCount = 0;
    uint64_t skippedTesteeTiny = 0;
    uint64_t skippedSceneGate = 0;
    uint64_t skippedMenuMode = 0;
    uint64_t skippedTerrain = 0;
    uint64_t skippedNoZTest = 0;  // depth test off: bypassed the occludee test
    // Depth-test-off leaves that drew an OCCLUDED verdict anyway. Only
    // nonzero with OcclusionSkipNoZTestOccludees off: it is the number of
    // x-ray leaves the mask hid that frame, i.e. what the bypass prevents.
    uint64_t zTestOffOccluded = 0;

    // Terrain aggregation.
    uint64_t aggregateTerrainLands = 0;
    uint64_t aggregateTerrainTris = 0;
    uint64_t aggregateTerrainUs = 0;

    // Occluder cache miss-path probes.
    uint64_t classifyOccluderCalls = 0;
    uint64_t classifyOccluderSteps = 0;
    uint64_t occluderVertexCalls = 0;
    uint64_t occluderVertexVerts = 0;

    // Phase timers (microseconds).
    uint64_t rasterizeTimeUs = 0;
    uint64_t drainPhaseTimeUs = 0;
    uint64_t classifyUs = 0;
    uint64_t drainDisplayUs = 0;
    uint64_t occluderTransformUs = 0;
};

extern FrameStats g_stats;

}  // namespace msoc::occlusion
