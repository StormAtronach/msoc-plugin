// Unit tests for msoc::profiling - the engine-free phase-budget math
// (EMA + predictive-skip / spike-clip decisions) extracted from
// OcclusionPass.cpp.

#include "doctest.h"

#include "Profiling.h"

using msoc::profiling::emaUpdate;
using msoc::profiling::predictiveSkip;
using msoc::profiling::spikeClipTripped;

TEST_CASE("emaUpdate is the (prev*7 + sample)/8 moving average") {
    // A steady stream of the same value converges to that value.
    CHECK(emaUpdate(0, 0) == 0);
    CHECK(emaUpdate(800, 800) == 800);

    // One step toward a new sample: (1000*7 + 0)/8 = 875.
    CHECK(emaUpdate(1000, 0) == 875);
    // (0*7 + 800)/8 = 100.
    CHECK(emaUpdate(0, 800) == 100);

    // Repeated application drifts monotonically toward the sample.
    uint64_t ema = 0;
    for (int i = 0; i < 100; ++i) ema = emaUpdate(ema, 1000);
    CHECK(ema >= 990);  // converged near the target
    CHECK(ema <= 1000);
}

TEST_CASE("predictiveSkip trips only above 2x budget, and never when disabled") {
    SUBCASE("budget 0 disables the layer") {
        CHECK_FALSE(predictiveSkip(0, 0));
        CHECK_FALSE(predictiveSkip(1'000'000, 0));  // huge EMA, still off
    }
    SUBCASE("strictly greater than 2x budget") {
        CHECK_FALSE(predictiveSkip(200, 100));  // exactly 2x -> not >, no skip
        CHECK(predictiveSkip(201, 100));        // just over 2x -> skip
        CHECK_FALSE(predictiveSkip(150, 100));  // within budget band -> no skip
    }
}

TEST_CASE("spikeClipTripped trips above budget, and never when disabled") {
    SUBCASE("budget 0 disables the layer") {
        CHECK_FALSE(spikeClipTripped(0, 0));
        CHECK_FALSE(spikeClipTripped(1'000'000, 0));
    }
    SUBCASE("strictly greater than budget") {
        CHECK_FALSE(spikeClipTripped(100, 100));  // at budget -> not >, no trip
        CHECK(spikeClipTripped(101, 100));        // over budget -> trip
        CHECK_FALSE(spikeClipTripped(50, 100));
    }
}
