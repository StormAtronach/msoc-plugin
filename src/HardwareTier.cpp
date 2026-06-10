#include "HardwareTier.h"

namespace msoc {

HardwareTier classifyHardwareTier(int simdImpl, unsigned hwConcurrency) {
    // Sandy/Ivy Bridge fall back to SSE4.1 (4-wide) - half the per-tile
    // throughput of Haswell+ AVX2. Combined with their 4-thread ceiling,
    // async occluders cost more in sync than they save in parallelism.
    const bool noAvx2 = (simdImpl < 2);
    if (noAvx2 || hwConcurrency <= 4) return HardwareTier::Low;
    if (hwConcurrency <= 8) return HardwareTier::Mid;
    return HardwareTier::High;
}

const char* hardwareTierName(HardwareTier tier) {
    switch (tier) {
        case HardwareTier::Low:
            return "low";
        case HardwareTier::Mid:
            return "mid";
        case HardwareTier::High:
            return "high";
    }
    return "unknown";
}

}  // namespace msoc
