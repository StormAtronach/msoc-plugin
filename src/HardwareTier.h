#pragma once

// Hardware tier classification - pure logic, no Configuration or Lua
// dependency, so it is unit-testable on its own (see tests/hardware_tier_tests).
// applyHardwareTierDefaults (which writes Configuration:: statics) stays in
// Config.cpp; only the pure classifier + name helper live here.

namespace msoc {

// On weaker CPUs (no AVX2, <=4 threads) the async occluder path can be
// net-negative, so the tier maps to a different default set of threadpool /
// mask knobs. simdImpl values match MaskedOcclusionCulling::Implementation
// (SSE2=0, SSE41=1, AVX2=2, AVX512=3); negative = probe failed, treat as Low.
enum class HardwareTier { Low = 0, Mid = 1, High = 2 };

HardwareTier classifyHardwareTier(int simdImpl, unsigned hwConcurrency);
const char* hardwareTierName(HardwareTier tier);

}  // namespace msoc
