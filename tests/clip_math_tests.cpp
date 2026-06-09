// Unit tests for msoc::clipmath - the engine-free clip-space / projection
// math extracted from OcclusionPass.cpp. All pure float helpers;
// no NI/TES3/MOC dependency.

#include "doctest.h"

#include "ClipMath.h"

#include <array>

using msoc::clipmath::ClipXYW;
using msoc::clipmath::RowNorms;

namespace {

// Column-major 4x4 identity, element [col*4 + row].
constexpr std::array<float, 16> kIdentity = {
    1,
    0,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    0,
    1,
};

}  // namespace

TEST_CASE("mat4MulColumnMajor: identity is the multiplicative unit") {
    // A populated, non-symmetric matrix so a transpose bug would show.
    std::array<float, 16> a = {
        1,
        2,
        3,
        4,
        5,
        6,
        7,
        8,
        9,
        10,
        11,
        12,
        13,
        14,
        15,
        16,
    };
    std::array<float, 16> out{};

    SUBCASE("A * I == A") {
        msoc::clipmath::mat4MulColumnMajor(a.data(), kIdentity.data(), out.data());
        for (int i = 0; i < 16; ++i) {
            CHECK(out[i] == doctest::Approx(a[i]));
        }
    }
    SUBCASE("I * A == A") {
        msoc::clipmath::mat4MulColumnMajor(kIdentity.data(), a.data(), out.data());
        for (int i = 0; i < 16; ++i) {
            CHECK(out[i] == doctest::Approx(a[i]));
        }
    }
}

TEST_CASE("mat4MulColumnMajor: composition matches hand computation") {
    // Two pure scales: diag(2) then diag(3) -> diag(6).
    std::array<float, 16> s2 = kIdentity;
    std::array<float, 16> s3 = kIdentity;
    for (int d = 0; d < 3; ++d) {
        s2[d * 4 + d] = 2.0f;
        s3[d * 4 + d] = 3.0f;
    }
    std::array<float, 16> out{};
    msoc::clipmath::mat4MulColumnMajor(s2.data(), s3.data(), out.data());
    CHECK(out[0] == doctest::Approx(6.0f));
    CHECK(out[5] == doctest::Approx(6.0f));
    CHECK(out[10] == doctest::Approx(6.0f));
    CHECK(out[15] == doctest::Approx(1.0f));
}

TEST_CASE("transposeRowToColumnMajor swaps the index convention") {
    // Row-major source filled with its own flat index.
    std::array<float, 16> rowMajor{};
    for (int i = 0; i < 16; ++i) rowMajor[i] = static_cast<float>(i);

    std::array<float, 16> colMajor{};
    msoc::clipmath::transposeRowToColumnMajor(rowMajor.data(), colMajor.data());

    // out[col*4 + row] == in[row*4 + col]
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            CHECK(colMajor[col * 4 + row] == doctest::Approx(rowMajor[row * 4 + col]));
        }
    }
    // Spot-check a known off-diagonal: col0,row1 == in[1*4+0] == 4.
    CHECK(colMajor[1] == doctest::Approx(4.0f));
}

TEST_CASE("projectWorld through identity is the point with w=1") {
    const ClipXYW c = msoc::clipmath::projectWorld(kIdentity.data(), 2.0f, 3.0f, 4.0f);
    CHECK(c.x == doctest::Approx(2.0f));
    CHECK(c.y == doctest::Approx(3.0f));
    CHECK(c.w == doctest::Approx(1.0f));  // identity row 3 contributes only m[15]
}

TEST_CASE("projectWorld applies translation and perspective rows") {
    // Column-major: translation in m[12..15], a w that depends on z via m[11].
    std::array<float, 16> m = kIdentity;
    m[12] = 10.0f;  // +x translation
    m[13] = -5.0f;  // +y translation
    m[11] = 0.5f;   // w picks up 0.5 * z  (m[11] is row 3, col 2)

    const ClipXYW c = msoc::clipmath::projectWorld(m.data(), 1.0f, 2.0f, 8.0f);
    // x = 1*1 + ... + m[12] = 11; y = 2*1 + m[13] = -3; w = 8*m[11] + m[15] = 5.
    CHECK(c.x == doctest::Approx(11.0f));
    CHECK(c.y == doctest::Approx(-3.0f));
    CHECK(c.w == doctest::Approx(5.0f));
}

TEST_CASE("clipRowNorms are the per-row operator norms (translation excluded)") {
    SUBCASE("identity gives unit x/y radii and zero w-gradient") {
        const RowNorms n = msoc::clipmath::clipRowNorms(kIdentity.data());
        CHECK(n.ndcRadiusX == doctest::Approx(1.0f));
        CHECK(n.ndcRadiusY == doctest::Approx(1.0f));
        CHECK(n.wGradMag == doctest::Approx(0.0f));
    }
    SUBCASE("a 3-4-5 row yields norm 5") {
        std::array<float, 16> m = kIdentity;
        // x-row coefficients live at m[0], m[4], m[8].
        m[0] = 3.0f;
        m[4] = 4.0f;
        m[8] = 0.0f;
        const RowNorms n = msoc::clipmath::clipRowNorms(m.data());
        CHECK(n.ndcRadiusX == doctest::Approx(5.0f));
    }
}
