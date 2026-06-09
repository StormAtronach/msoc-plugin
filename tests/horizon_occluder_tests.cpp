// Unit tests for msoc::horizon::HorizonOccluder.
//
// This is the engine-free half of the plugin's math: the 1D silhouette
// buffer, its adaptive-epsilon depth threshold, the budget-bounded
// Douglas-Peucker simplifier, and the NDC -> MOC vertex fixup. None of it
// touches the Morrowind engine, so it tests standalone with no MWSE/LuaJIT.
//
// Assertions are grounded in the documented contracts in HorizonOccluder.h
// and the implementation in HorizonOccluder.cpp - not guessed.

#include "doctest.h"

#include "HorizonOccluder.h"

#include <cmath>
#include <limits>
#include <vector>

using msoc::horizon::CurtainVertex;
using msoc::horizon::HorizonOccluder;
using msoc::horizon::kYBelow;
using msoc::horizon::Sample;

// A column counts as "touched" when its height clears the sentinel by the
// same epsilon the implementation uses.
static bool isSentinel(float h) {
    return h <= kYBelow + 1.0e-29f;
}

TEST_CASE("init validates args and is idempotent") {
    HorizonOccluder h;

    SUBCASE("rejects degenerate sizes") {
        CHECK_FALSE(h.init(1, 8));   // resolution < 2
        CHECK_FALSE(h.init(64, 1));  // maxSamples < 2
    }

    SUBCASE("same args are a no-op, different args reallocate") {
        CHECK(h.init(64, 16));
        CHECK(h.resolution() == 64);
        CHECK(h.init(64, 16));  // idempotent
        CHECK(h.resolution() == 64);
        CHECK(h.init(128, 16));  // realloc
        CHECK(h.resolution() == 128);
    }
}

TEST_CASE("reset returns every column to the sentinel") {
    HorizonOccluder h;
    REQUIRE(h.init(32, 8));
    h.updateRange(0, 31, 0.5f, 100.0f);
    REQUIRE(h.columnsTouched() == 32);

    h.reset();
    CHECK(h.columnsTouched() == 0);
    for (int c = 0; c < 32; ++c) {
        CHECK(h.heightAt(c) == kYBelow);  // exact: reset does std::fill
    }
}

TEST_CASE("update writes only when yUpper STRICTLY exceeds current height") {
    HorizonOccluder h;
    REQUIRE(h.init(16, 8));
    h.reset();

    h.update(5, 0.5f, 1000.0f);
    CHECK(h.heightAt(5) == doctest::Approx(0.5f));
    CHECK(h.depthAt(5) == doctest::Approx(1000.0f));

    // Lower height is ignored; depth stays the winner's.
    h.update(5, 0.2f, 2000.0f);
    CHECK(h.heightAt(5) == doctest::Approx(0.5f));
    CHECK(h.depthAt(5) == doctest::Approx(1000.0f));

    // Equal height is NOT strictly greater -> ignored.
    h.update(5, 0.5f, 3000.0f);
    CHECK(h.depthAt(5) == doctest::Approx(1000.0f));

    // Strictly higher overwrites both height and depth.
    h.update(5, 0.9f, 4000.0f);
    CHECK(h.heightAt(5) == doctest::Approx(0.9f));
    CHECK(h.depthAt(5) == doctest::Approx(4000.0f));
}

TEST_CASE("update is order-independent: max height wins, depth follows it") {
    HorizonOccluder a;
    HorizonOccluder b;
    REQUIRE(a.init(16, 8));
    REQUIRE(b.init(16, 8));
    a.reset();
    b.reset();

    // Same three contributions, applied in different orders.
    a.update(3, 0.1f, 100.0f);
    a.update(3, 0.7f, 700.0f);
    a.update(3, 0.4f, 400.0f);

    b.update(3, 0.4f, 400.0f);
    b.update(3, 0.1f, 100.0f);
    b.update(3, 0.7f, 700.0f);

    CHECK(a.heightAt(3) == doctest::Approx(b.heightAt(3)));
    CHECK(a.depthAt(3) == doctest::Approx(b.depthAt(3)));
    CHECK(a.heightAt(3) == doctest::Approx(0.7f));
    CHECK(a.depthAt(3) == doctest::Approx(700.0f));
}

TEST_CASE("columnsTouched counts distinct non-sentinel columns") {
    HorizonOccluder h;
    REQUIRE(h.init(16, 8));
    h.reset();
    CHECK(h.columnsTouched() == 0);

    h.update(1, 0.1f, 1.0f);
    h.update(2, 0.1f, 1.0f);
    h.update(2, 0.2f, 1.0f);  // same column again - not double-counted
    CHECK(h.columnsTouched() == 2);
}

TEST_CASE("updateRange covers an inclusive range and clamps to bounds") {
    HorizonOccluder h;
    REQUIRE(h.init(16, 8));
    h.reset();

    h.updateRange(4, 7, 0.3f, 500.0f);
    for (int c = 4; c <= 7; ++c) {
        CHECK(h.heightAt(c) == doctest::Approx(0.3f));
        CHECK(h.depthAt(c) == doctest::Approx(500.0f));
    }
    CHECK(isSentinel(h.heightAt(3)));  // just outside the range
    CHECK(isSentinel(h.heightAt(8)));
    CHECK(h.columnsTouched() == 4);

    SUBCASE("out-of-range endpoints are clamped, not UB") {
        h.reset();
        h.updateRange(-5, 100, 0.2f, 50.0f);  // clamps to [0, resolution-1]
        CHECK(h.columnsTouched() == 16);
    }
}

TEST_CASE("computeAdaptiveEpsD") {
    HorizonOccluder h;
    REQUIRE(h.init(16, 8));
    h.reset();

    SUBCASE("fewer than two data columns disables the depth term (+inf)") {
        CHECK(std::isinf(h.computeAdaptiveEpsD()));  // zero columns
        h.update(0, 0.1f, 100.0f);
        CHECK(std::isinf(h.computeAdaptiveEpsD()));  // one column
    }

    SUBCASE("returns clamp_low(fraction * depthRange, floor)") {
        // Depth range [100, 1100] -> span 1000.
        h.update(0, 0.1f, 100.0f);
        h.update(1, 0.1f, 1100.0f);

        // 0.05 * 1000 = 50, below the floor of 100 -> floor wins.
        CHECK(h.computeAdaptiveEpsD(0.05f, 100.0f) == doctest::Approx(100.0f));
        // 0.5 * 1000 = 500, above the floor -> candidate wins.
        CHECK(h.computeAdaptiveEpsD(0.5f, 100.0f) == doctest::Approx(500.0f));
    }
}

TEST_CASE("fixupForMOC converts NDC layout to pre-transformed clip space") {
    // (x, y, z=depth, w=1) -> (x*d, y*d, z unchanged, w=d).
    CurtainVertex verts[2] = {
        {2.0f, 3.0f, 5.0f, 1.0f},
        {-1.0f, 0.5f, 10.0f, 1.0f},
    };
    HorizonOccluder::fixupForMOC(verts, 2);

    CHECK(verts[0].x == doctest::Approx(2.0f * 5.0f));
    CHECK(verts[0].y == doctest::Approx(3.0f * 5.0f));
    CHECK(verts[0].w == doctest::Approx(5.0f));  // w := depth

    CHECK(verts[1].x == doctest::Approx(-1.0f * 10.0f));
    CHECK(verts[1].y == doctest::Approx(0.5f * 10.0f));
    CHECK(verts[1].w == doctest::Approx(10.0f));
}

TEST_CASE("simplify keeps endpoints and stays within the sample budget") {
    HorizonOccluder h;
    REQUIRE(h.init(16, 8));
    h.reset();

    SUBCASE("a flat horizon collapses to the two endpoints") {
        h.updateRange(0, 15, 0.0f, 100.0f);  // uniform height and depth
        Sample out[8];
        const int n = h.simplify(out, 8, /*epsH*/ 0.01f, /*epsD*/ 100.0f, /*tileAlign*/ 1);
        CHECK(n == 2);
        CHECK(out[0].col == 0);
        CHECK(out[1].col == 15);
    }

    SUBCASE("a sharp step forces interior splits but respects the budget") {
        h.updateRange(0, 7, 0.0f, 100.0f);
        h.updateRange(8, 15, 1.0f, 100.0f);  // a one-unit height cliff at col 8
        Sample out[8];
        const int n = h.simplify(out, 8, /*epsH*/ 0.05f, /*epsD*/ 100.0f, /*tileAlign*/ 1);
        CHECK(n > 2);
        CHECK(n <= 8);
        CHECK(out[0].col == 0);       // first kept is always the left endpoint
        CHECK(out[n - 1].col == 15);  // last kept is always the right endpoint
    }

    SUBCASE("guards: null out or degenerate budget returns 0") {
        CHECK(h.simplify(nullptr, 8, 1.0f, 1.0f, 1) == 0);
        Sample out[8];
        CHECK(h.simplify(out, 1, 1.0f, 1.0f, 1) == 0);  // maxSamples < 2
    }
}

TEST_CASE("emitCurtainNDC uses conservative top/depth and the given bottom") {
    // Two explicit samples; one segment -> 2 triangles, 6 vertices.
    Sample s[2] = {
        {0, -1.0f, /*h*/ 0.5f, /*d*/ 100.0f},
        {15, 1.0f, /*h*/ 0.8f, /*d*/ 200.0f},
    };
    CurtainVertex out[6];
    HorizonOccluder h;  // emitCurtainNDC reads only its arguments

    const int tris = h.emitCurtainNDC(s, 2, /*ndcYBottom*/ -1.1f, out, 6);
    CHECK(tris == 2);

    // Conservative silhouette: top = min(h), depth = max(d).
    const float expectTop = 0.5f;
    const float expectZ = 200.0f;
    // First emitted vertex is the top-left corner (TL).
    CHECK(out[0].x == doctest::Approx(-1.0f));
    CHECK(out[0].y == doctest::Approx(expectTop));
    CHECK(out[0].z == doctest::Approx(expectZ));
    // Second is bottom-left (BL): same depth, the caller-supplied bottom y.
    CHECK(out[1].y == doctest::Approx(-1.1f));
    CHECK(out[1].z == doctest::Approx(expectZ));

    SUBCASE("too-small output buffer is rejected") {
        CurtainVertex tiny[3];
        CHECK(h.emitCurtainNDC(s, 2, -1.1f, tiny, 3) == 0);  // needs 6
    }
}
