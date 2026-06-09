// Live sphere-vs-mask query: projects a world sphere through the live
// per-frame matrix and tests it against g_msoc. Used by the drain (core) and
// LightCulling - extracted to a leaf TU so neither depends back into the core.
// Shared state via OcclusionInternal.h.

#include "OcclusionInternal.h"

#include "NIPoint3.h"

#include <algorithm>
#include <atomic>

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

}  // namespace msoc::patch::occlusion
