// Unit tests for msoc::classifyHardwareTier / hardwareTierName - the pure
// hardware-tier decision logic split out of Config.cpp (which is Lua-coupled).
// simdImpl: SSE2=0, SSE41=1, AVX2=2, AVX512=3, negative = probe failed.

#include "doctest.h"

#include "HardwareTier.h"

#include <string>

using msoc::classifyHardwareTier;
using msoc::HardwareTier;

// Compare as int so doctest never needs to stringify the enum class.
static int tier(int simd, unsigned threads) {
    return static_cast<int>(classifyHardwareTier(simd, threads));
}

TEST_CASE("classifyHardwareTier: no AVX2 is always Low") {
    CHECK(tier(0, 16) == static_cast<int>(HardwareTier::Low));   // SSE2
    CHECK(tier(1, 16) == static_cast<int>(HardwareTier::Low));   // SSE4.1
    CHECK(tier(-1, 32) == static_cast<int>(HardwareTier::Low));  // probe failed
}

TEST_CASE("classifyHardwareTier: <=4 threads is Low even with AVX2/AVX512") {
    CHECK(tier(2, 1) == static_cast<int>(HardwareTier::Low));
    CHECK(tier(2, 4) == static_cast<int>(HardwareTier::Low));
    CHECK(tier(3, 4) == static_cast<int>(HardwareTier::Low));
}

TEST_CASE("classifyHardwareTier: AVX2+ with 5..8 threads is Mid") {
    CHECK(tier(2, 5) == static_cast<int>(HardwareTier::Mid));
    CHECK(tier(2, 8) == static_cast<int>(HardwareTier::Mid));
    CHECK(tier(3, 6) == static_cast<int>(HardwareTier::Mid));  // AVX512 same as AVX2 here
}

TEST_CASE("classifyHardwareTier: AVX2+ with >8 threads is High") {
    CHECK(tier(2, 9) == static_cast<int>(HardwareTier::High));
    CHECK(tier(2, 16) == static_cast<int>(HardwareTier::High));
    CHECK(tier(3, 32) == static_cast<int>(HardwareTier::High));
}

TEST_CASE("classifyHardwareTier: thread-count boundaries (AVX2)") {
    CHECK(tier(2, 4) == static_cast<int>(HardwareTier::Low));   // <=4
    CHECK(tier(2, 5) == static_cast<int>(HardwareTier::Mid));   // first Mid
    CHECK(tier(2, 8) == static_cast<int>(HardwareTier::Mid));   // last Mid
    CHECK(tier(2, 9) == static_cast<int>(HardwareTier::High));  // first High
}

TEST_CASE("hardwareTierName maps each tier") {
    CHECK(std::string(msoc::hardwareTierName(HardwareTier::Low)) == "low");
    CHECK(std::string(msoc::hardwareTierName(HardwareTier::Mid)) == "mid");
    CHECK(std::string(msoc::hardwareTierName(HardwareTier::High)) == "high");
}
