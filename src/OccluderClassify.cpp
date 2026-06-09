// Occluder property classification: first-of-type alpha/stencil flags from
// a node's ancestor chain (alpha/stencil meshes are excluded from the
// occluder rasterise pass). Used by the rasterizer (core) and
// TerrainAggregation - extracted to a leaf TU to break the back-edge into
// the core. Shared state via OcclusionInternal.h.

#include "OcclusionInternal.h"

#include "NIAVObject.h"
#include "NIProperty.h"

namespace msoc::patch::occlusion {

// Why these can't be occluders:
//   alpha   - blended/alpha-tested shapes (fences, banners, grates,
//             vines, leaves, glass) are partially transparent. A
//             solid-occluder rasterise fills the full quad and
//             falsely occludes whatever sits behind the holes.
//   stencil - fills only where the test passes (shadow volumes,
//             reflection clip masks, UI cutouts). Same hazard.
// Both still participate as occludees; only the occluder rasterise
// pass skips them.
OccluderPropertyFlags classifyOccluderProperties(NI::AVObject* obj) {
    // Probes gated on g_frame.logEnabled - same contract as
    // ScopedUsAccumulator. Off-path: predicted-not-taken branch + no
    // counter store; on-path: 4 atomic-free uint64 inc per frame's
    // miss-set.
    const bool logOn = g_frame.logEnabled;
    if (logOn) ++g_stats.classifyOccluderCalls;
    OccluderPropertyFlags out = {false, false};
    bool alphaResolved = false;
    bool stencilResolved = false;
    for (NI::AVObject* cur = obj; cur; cur = cur->parentNode) {
        if (logOn) ++g_stats.classifyOccluderSteps;
        for (auto* node = &cur->propertyNode; node && node->data; node = node->next) {
            const auto type = node->data->getType();
            if (!alphaResolved && type == NI::PropertyType::Alpha) {
                const unsigned short flags = node->data->flags;
                out.alpha = (flags & (NI::AlphaProperty::ALPHA_MASK | NI::AlphaProperty::TEST_ENABLE_MASK)) != 0;
                alphaResolved = true;
            } else if (!stencilResolved && type == NI::PropertyType::Stencil) {
                out.stencil = static_cast<NI::StencilProperty*>(node->data)->enabled;
                stencilResolved = true;
            }
            if (alphaResolved && stencilResolved) return out;
        }
    }
    return out;
}

}  // namespace msoc::patch::occlusion
