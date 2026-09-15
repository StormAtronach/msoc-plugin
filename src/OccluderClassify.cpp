// Property classification: first-of-type alpha/stencil/z-buffer flags from
// a node's ancestor chain (alpha/stencil meshes are excluded from the
// occluder rasterise pass; depth-test-off meshes from the occluder pass and
// the occludee test). Used by the rasterizer (core) and TerrainAggregation -
// extracted to a leaf TU to break the back-edge into the core. Shared state
// via OcclusionInternal.h.

#include "OcclusionInternal.h"

#include "NIAVObject.h"
#include "NIProperty.h"

namespace msoc::occlusion::classify {

// Why these can't be occluders:
//   alpha    - blended/alpha-tested shapes (fences, banners, grates,
//              vines, leaves, glass) are partially transparent. A
//              solid-occluder rasterise fills the full quad and
//              falsely occludes whatever sits behind the holes.
//   stencil  - fills only where the test passes (shadow volumes,
//              reflection clip masks, UI cutouts). Same hazard.
//   zTestOff - drawn with the depth test disabled: x-ray effects such
//              as Enhanced Detection's OJ/ED/*.nif (flags 0x0000) and
//              see-through architecture inserts (window panes, grates,
//              flags 0x0002). The renderer ignores depth for them, so
//              they hide nothing. The same flag exempts the leaf from
//              the occludee test in OcclusionPass: the renderer draws
//              it through walls, and a depth verdict that flips with
//              the hi-Z seams showed as flicker on moving refs.
// Alpha and stencil shapes still participate as occludees; only the
// occluder rasterise pass skips them.
//
// NiZBufferProperty flags, per the NIF format and the D3D8 pipeline note
// in moreFPS/docs (nif-alpha-zbuffer-d3d8-pipeline.md): bit 0 = depth test
// enable, bit 1 = depth write enable. SharedSE's NI::ZBufferProperty names
// neither, so the mask lives here.
constexpr unsigned short kZBufferTestEnableMask = 0x0001;

OccluderPropertyFlags occluderProperties(NI::AVObject* obj) {
    // Probes gated on g_frame.logEnabled - same contract as
    // ScopedUsAccumulator. Off-path: predicted-not-taken branch + no
    // counter store; on-path: 4 atomic-free uint64 inc per frame's
    // miss-set.
    const bool logOn = g_frame.logEnabled;
    if (logOn) ++g_stats.classifyOccluderCalls;
    OccluderPropertyFlags out = {false, false, false};
    bool alphaResolved = false;
    bool stencilResolved = false;
    bool zBufferResolved = false;
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
            } else if (!zBufferResolved && type == NI::PropertyType::ZBuffer) {
                // First ZBuffer property up the chain is the effective one
                // (Gamebryo child-overrides-parent accumulation), so a
                // test-off parent with a test-on child resolves test-on.
                const auto* z = static_cast<const NI::ZBufferProperty*>(node->data);
                out.zTestOff = (z->flags & kZBufferTestEnableMask) == 0 ||
                               z->testFunction == NI::ZBufferProperty::TestFunction::ALWAYS;
                zBufferResolved = true;
            }
            if (alphaResolved && stencilResolved && zBufferResolved) return out;
        }
    }
    return out;
}

}  // namespace msoc::occlusion::classify
