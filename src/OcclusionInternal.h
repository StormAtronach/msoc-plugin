#pragma once

// Internal shared state for the occlusion patch, exposed so subsystem
// translation units (QueryApi.cpp, ...) can link against the core state that
// PatchOcclusionCulling.cpp defines. This is the plugin's private seam, not a
// public API - the public C surface lives in PatchOcclusionCulling.h.
//
// The single definitions of the extern globals below live in
// PatchOcclusionCulling.cpp.

#include "MaskedOcclusionCulling.h"
#include "FrameConfig.h"
#include "ClipMath.h"

namespace msoc::patch::occlusion {

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

    // Cross-TU shared state. Defined once in PatchOcclusionCulling.cpp.
    extern FrameConfig g_frame;        // per-frame Configuration snapshot
    extern MaskSnapshot g_snapshot;    // published snapshot metadata
    extern ::MaskedOcclusionCulling* g_msoc_prev;  // swapped snapshot buffer
    extern unsigned int kMsocWidth;    // mask resolution (latched at install)
    extern unsigned int kMsocHeight;

} // namespace msoc::patch::occlusion
