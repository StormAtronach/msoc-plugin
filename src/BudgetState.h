#pragma once

// Two-layer phase-budget controller for the MSOC-only phases (occluder
// rasterization, occludee classification). Grouped into one owner so the
// detour's control flow and the diagnostics log reference a single g_budget
// instead of loose globals. The math lives in Profiling.h: emaUpdate,
// predictiveSkip (Layer 1: EMA vs 2x budget), spikeClipTripped (Layer 2:
// elapsed vs budget) all operate on these fields.
//
// The single g_budget instance is defined in PatchOcclusionCulling.cpp.

#include <chrono>
#include <cstdint>

namespace msoc::patch::occlusion {

struct BudgetState {
    uint64_t rasterizeEmaUs = 0;
    uint64_t classifyEmaUs = 0;
    uint64_t rasterizeBudgetUsEffective = 0;  // resolved from Configuration each frame
    uint64_t classifyBudgetUsEffective = 0;
    bool skipRasterizeThisFrame = false;  // Layer 1 predictive-skip latch
    bool skipClassifyThisFrame = false;
    std::chrono::steady_clock::time_point classifyPhaseStart;
    uint32_t rasterizeBudgetTrips = 0;  // Layer 2 trip, per-frame (0/1)
    uint32_t classifyBudgetTrips = 0;
    uint64_t rasterizeBudgetTripsSession = 0;  // lifetime trip count
    uint64_t classifyBudgetTripsSession = 0;
};

extern BudgetState g_budget;

}  // namespace msoc::patch::occlusion
