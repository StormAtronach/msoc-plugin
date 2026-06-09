#pragma once

// Per-frame + session diagnostic bookkeeping reported by the per-frame stats
// log (DiagnosticsLog.cpp). This is the remaining frame/session state that
// isn't config (FrameConfig), per-frame work counters (FrameStats), or budget
// control (BudgetState): cell-cross + frame-window timing, threadpool timing,
// recursion/multi-pass depth trackers, and lifetime cumulative query totals.
//
// Reset semantics are the detour's: per-frame fields are cleared at the top of
// each frame, *Session fields accumulate for the process lifetime, and the
// window* fields reset per log window. The single g_diag instance is defined
// in PatchOcclusionCulling.cpp.

#include <cstdint>

namespace msoc::patch::occlusion {

struct FrameDiag {
    // Cell-cross diagnostics.
    int cellCrossLogFrames = 0;
    uint64_t cellWipeUs = 0;
    uint64_t lastFrameDeltaUs = 0;
    uint64_t cellChanges = 0;

    // Frame-window average timing (reset per log window).
    uint64_t windowFrameTimeUs = 0;
    uint64_t windowFrameCount = 0;

    // Threadpool timing.
    uint64_t asyncFlushTimeUs = 0;
    uint64_t wakeThreadsTimeUs = 0;
    uint64_t maxWakeThreadsUsSession = 0;

    // Recursion depth.
    uint32_t maxCallDepthThisFrame = 0;
    uint32_t maxCallDepthSession = 0;

    // Multi-pass guards / per-scene attempt counters.
    unsigned int isTopLevelFiresThisScene = 0;
    unsigned int maxIsTopLevelFiresSession = 0;
    unsigned int mainCamCullShowAttemptsThisScene = 0;
    unsigned int maxMainCamCullShowAttemptsSession = 0;

    // Lifetime cumulative query totals.
    uint64_t totalTested = 0;
    uint64_t totalOccluded = 0;
    uint64_t totalViewCulled = 0;
};

extern FrameDiag g_diag;

}  // namespace msoc::patch::occlusion
