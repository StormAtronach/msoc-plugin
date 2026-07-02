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

TEST_CASE("conservativeSphereNdcRect bounds every sphere surface point") {
    // Perspective matrix, column-major: clip.x = fx*x, clip.y = fy*y,
    // clip.w = z (view depth). fx/fy approximate a wide Morrowind FOV.
    const float fx = 1.2f;
    const float fy = 1.6f;
    std::array<float, 16> m = {};
    m[0] = fx;   // x row
    m[5] = fy;   // y row
    m[11] = 1;   // w row picks up view z

    const RowNorms norms = msoc::clipmath::clipRowNorms(m.data());

    // Sphere centers across the frustum, including hard off-axis close-range
    // cases (the regime where dividing by the center w under-covers).
    const float centers[][3] = {
        {0, 0, 100},   {50, 30, 100},  {-70, 45, 110}, {80, -60, 120},
        {30, 20, 40},  {-25, 18, 35},  {15, -12, 25},  {-9, 8, 20},
    };
    const float radii[] = {1.0f, 5.0f, 12.0f};

    for (const auto& ctr : centers) {
        for (const float r : radii) {
            if (ctr[2] - r * norms.wGradMag <= 1.0f) continue;  // near bail, as callers do

            const ClipXYW c = msoc::clipmath::projectWorld(m.data(), ctr[0], ctr[1], ctr[2]);
            const auto rect = msoc::clipmath::conservativeSphereNdcRect(
                c.x, c.y, c.w, r * norms.ndcRadiusX, r * norms.ndcRadiusY, r * norms.wGradMag);

            // Sample the sphere surface; every projected point must be inside.
            const float kEps = 1e-4f;
            for (int i = 0; i < 400; ++i) {
                const float t = 3.8832221f * static_cast<float>(i);   // golden angle
                const float z = 1.0f - 2.0f * (static_cast<float>(i) + 0.5f) / 400.0f;
                const float s = std::sqrt(1.0f - z * z);
                const float px = ctr[0] + r * s * std::cos(t);
                const float py = ctr[1] + r * s * std::sin(t);
                const float pz = ctr[2] + r * z;
                const ClipXYW p = msoc::clipmath::projectWorld(m.data(), px, py, pz);
                REQUIRE(p.w > 0.0f);
                const float nx = p.x / p.w;
                const float ny = p.y / p.w;
                REQUIRE(nx >= rect.minX - kEps);
                REQUIRE(nx <= rect.maxX + kEps);
                REQUIRE(ny >= rect.minY - kEps);
                REQUIRE(ny <= rect.maxY + kEps);
            }
        }
    }

    // Regression: the pre-fix rect (clip extents divided by the CENTER w)
    // fails to cover an off-axis close sphere - the bug this helper replaces.
    {
        const float ctr[3] = {30, 20, 40};
        const float r = 12.0f;
        const ClipXYW c = msoc::clipmath::projectWorld(m.data(), ctr[0], ctr[1], ctr[2]);
        const float invW = 1.0f / c.w;
        const float naiveMaxX = (c.x + r * norms.ndcRadiusX) * invW;
        // Nearest-and-outermost surface point exceeds the naive bound.
        const ClipXYW p = msoc::clipmath::projectWorld(
            m.data(), ctr[0] + r * 0.7071f, ctr[1], ctr[2] - r * 0.7071f);
        CHECK(p.x / p.w > naiveMaxX);
    }
}
