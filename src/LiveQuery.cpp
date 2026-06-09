// Live sphere-vs-mask query: projects a world sphere through the live
// per-frame matrix and tests it against g_msoc. Used by the drain (core) and
// LightCulling - extracted to a leaf TU so neither depends back into the core.
// Shared state via OcclusionInternal.h.

#include "OcclusionInternal.h"

#include "NIPoint3.h"

#include <algorithm>
#include <atomic>
#include <cfloat>

namespace msoc::patch::occlusion {

// Sphere straddling the near plane bails as VISIBLE; TestRect can't
// project a straddling rect safely.
::MaskedOcclusionCulling::CullingResult testSphereVisible(
    const NI::Point3& center, float radius) {
    const ClipXYW c = projectWorld(center.x, center.y, center.z);

    const float wMin = c.w - (radius + g_frame.depthSlackWorldUnits) * g_wGradMag;
    if (wMin <= kNearClipW) {
        g_stats.queryNearClip.fetch_add(1, std::memory_order_relaxed);
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

// Live OBB-vs-mask query: projects the 8 world-space box corners through the
// live matrix, tests their covered screen rect against g_msoc. Same shape as
// testOcclusionSphere's rect path + testOcclusionOBB, but against the live
// (this-frame) buffer. corners is 8 (x, y, z) triples. Used by the drain to
// refine a sphere-VISIBLE occludee with its tighter box.
::MaskedOcclusionCulling::CullingResult testBoxVisible(const float* corners) {
    float ndcMinX = FLT_MAX, ndcMinY = FLT_MAX;
    float ndcMaxX = -FLT_MAX, ndcMaxY = -FLT_MAX;
    float wMin = FLT_MAX;

    for (int i = 0; i < 8; ++i) {
        const ClipXYW c = projectWorld(corners[i * 3 + 0], corners[i * 3 + 1], corners[i * 3 + 2]);
        // Any corner straddling the near plane: bail VISIBLE (can't project
        // the rect safely), matching the sphere path.
        if (c.w <= kNearClipW) {
            return ::MaskedOcclusionCulling::VISIBLE;
        }
        const float invW = 1.0f / c.w;
        const float ndcX = c.x * invW;
        const float ndcY = c.y * invW;
        ndcMinX = std::min(ndcMinX, ndcX);
        ndcMinY = std::min(ndcMinY, ndcY);
        ndcMaxX = std::max(ndcMaxX, ndcX);
        ndcMaxY = std::max(ndcMaxY, ndcY);
        wMin = std::min(wMin, c.w);
    }

    // Same depth-slack pull-in as the sphere test (conservative near surface).
    wMin -= g_frame.depthSlackWorldUnits * g_wGradMag;
    if (wMin <= kNearClipW) {
        return ::MaskedOcclusionCulling::VISIBLE;
    }

    ndcMinX = std::max(ndcMinX, -1.0f);
    ndcMinY = std::max(ndcMinY, -1.0f);
    ndcMaxX = std::min(ndcMaxX, 1.0f);
    ndcMaxY = std::min(ndcMaxY, 1.0f);
    if (ndcMinX >= ndcMaxX || ndcMinY >= ndcMaxY) {
        return ::MaskedOcclusionCulling::VIEW_CULLED;
    }

    return g_msoc->TestRect(ndcMinX, ndcMinY, ndcMaxX, ndcMaxY, wMin);
}

}  // namespace msoc::patch::occlusion
