#pragma once

// Engine-free profiling / phase-budget math, extracted from
// OcclusionPass.cpp. Pure integer logic - no engine or clock types -
// so it compiles into both the plugin DLL and the unit-test target and inlines
// on the hot path.
//
// The plugin bounds two MSOC-only phases (occluder rasterization, occludee
// classification) with a two-layer budget:
//   Layer 1 - predictive skip: if the EMA of recent frames exceeds 2x the
//             budget, skip the whole phase this frame (self-regulating, no
//             in-loop cost; the 2x guard keeps a single spike from latching).
//   Layer 2 - spike clip: bail mid-phase once elapsed time exceeds the budget.
// A budget of 0 means "unlimited" and disables both layers for that phase.

#include <cstdint>

namespace msoc::profiling {

// Exponential moving average with a ~6-frame half-life: (prev*7 + sample) / 8.
inline uint64_t emaUpdate(uint64_t prev, uint64_t sample) {
    return (prev * 7 + sample) >> 3;
}

// Layer 1. True when this frame's phase should be skipped wholesale because
// recent frames have consistently blown the budget. Disabled when budget == 0.
inline bool predictiveSkip(uint64_t emaUs, uint64_t budgetUs) {
    return budgetUs > 0 && emaUs > 2 * budgetUs;
}

// Layer 2. True when the running in-phase elapsed time has exceeded the
// budget and the phase should bail now. Disabled when budget == 0.
inline bool spikeClipTripped(uint64_t elapsedUs, uint64_t budgetUs) {
    return budgetUs > 0 && elapsedUs > budgetUs;
}

}  // namespace msoc::profiling
