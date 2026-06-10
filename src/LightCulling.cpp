// CPU light culling: NiDX8LightManager::updateLights enabled-read hook.
// Tests each NiPointLight bounding sphere against the live mask and latches
// off fully-occluded lights (hysteresis), pre-empting D3D8 SetLight/
// LightEnable without touching the scene graph. Split from
// OcclusionPass.cpp; shared state via OcclusionInternal.h.

#include "OcclusionApi.h"
#include "OcclusionInternal.h"

#include "NILight.h"
#include "NIPoint3.h"
#include "NIRTTI.h"

#include <algorithm>
#include <vector>

namespace msoc::patch::occlusion {

// Light observers - external consumers (MGE-XE) snapshotting the live
// renderer-iterated light list. Iterated lock-free from the render thread.
// Light observers - registered by external consumers (MGE-XE) that
// want the live renderer-iterated light list without re-patching
// 0x6BB7D4. Iterated lock-free from the render thread.
static std::vector<LightObservedCallback> g_lightObservers;

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
// Counter is uint8_t - slider values above 255 effectively disable
// the latch.
extern "C" bool __cdecl shouldLightBeEnabled(NI::Light* light) {
    // Observer dispatch runs unconditionally - observers see every
    // iterated NiLight regardless of cache hit / MSOC state.
    for (const auto cb : g_lightObservers) {
        cb(light);
    }

    // The true enabled flag - always fetched, since a disabled
    // light stays disabled regardless of occlusion.
    const bool realEnabled = light->enabled;
    if (!realEnabled) {
        return false;
    }
    if (!g_frame.cullLights) {
        return true;
    }
    if (!g_msoc) {
        // Hook installed but MSOC backend failed to init - bypass.
        return true;
    }

    // Only NiPointLight is spatially cullable. Directional / ambient
    // lights affect the scene globally and must never be tested. The
    // specular.r overload below is point-light-specific (Bethesda's
    // modder-set fade radius); other light types may carry non-zero
    // specular for actual specular colour, so we can't filter by
    // that alone - we'd dim the menu / sun light at unlucky angles.
    if (!light->isInstanceOfType(NI::RTTIStaticPtr::NiPointLight)) {
        return true;
    }

    // Position + radius for the occlusion test:
    //   - worldTransform.translation: live engine-computed transform.
    //     worldBoundOrigin / worldBoundRadius look tempting but they
    //     stay uncomputed for most NiLights (the engine never asks
    //     for their bounding volume), so they degenerate to (0,0,0)/0.
    //     Same trap MGE-XE's many-lights producer hit (see
    //     occlusion-and-rendering.md sec 12).
    //   - specular.r: Bethesda overloaded NI::Light::specular.r as the
    //     modder-set fade radius. The engine's per-object light
    //     selection at 0x4D2F40 uses this and culls lights with
    //     radius=0 from every object - match its behavior here so we
    //     never test a light the engine itself wouldn't render.
    const NI::Point3& t = light->worldTransform.translation;
    const float cx = t.x;
    const float cy = t.y;
    const float cz = t.z;
    const float cr = light->specular.r;

    // PointLight with no modder-set radius - engine wouldn't render
    // it on most objects anyway, so don't bother testing.
    if (cr <= 0.0f) {
        return true;
    }

    auto it = g_caches.lightCull.find(light);
    const bool haveValidEntry = (it != g_caches.lightCull.end()) && it->second.boundOriginX == cx && it->second.boundOriginY == cy && it->second.boundOriginZ == cz && it->second.boundRadius == cr;

    // Same-frame cache hit - return latched verdict without re-testing.
    if (haveValidEntry && it->second.lastQueryFrame == g_frameCounter) {
        ++g_caches.lightCullHits;
        return !it->second.cullActive;
    }

    // Miss (or stale) - run MSOC test.
    ++g_caches.lightCullMisses;
    ++g_caches.lightsTested;
    const NI::Point3 center{cx, cy, cz};
    const auto verdict = testSphereVisible(center, cr);
    const bool occluded = (verdict == ::MaskedOcclusionCulling::OCCLUDED);
    if (occluded) ++g_caches.lightsOccluded;

    if (haveValidEntry) {
        auto& e = it->second;
        e.lastQueryFrame = g_frameCounter;
        if (occluded) {
            if (e.consecOccluded < 0xFF) ++e.consecOccluded;
            if (e.consecOccluded >= g_frame.lightCullHysteresisFrames) {
                e.cullActive = true;
            }
        } else {
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
    g_caches.lightCull[light] = e;
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
			push ebx  // arg: NI::Light*
			call shouldLightBeEnabled
			add esp, 4
			movzx eax, al  // zero-extend so upper bits are defined
			pop edx
			pop ecx
			retn
    }
}

}  // namespace msoc::patch::occlusion
