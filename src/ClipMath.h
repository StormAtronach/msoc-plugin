#pragma once

// Engine-free clip-space / projection math, extracted from
// OcclusionPass.cpp. Pure float helpers with no NI/TES3/MOC types, so
// they compile into both the plugin DLL and the unit-test target. Header-only
// `inline` so the hot-path call sites (per-occludee projectWorld) inline fully
// in every config, without depending on the Release-only LTO pass.
//
// Matrix layout: Intel/MOC column-major (v*M), element [col*4 + row]. NI stores
// its matrices row-major (M*v); transposeRowToColumnMajor converts between them.

#include <cmath>

namespace msoc::clipmath {

// Column-major 4x4 multiply: out = a * b, with out[col*4 + row].
inline void mat4MulColumnMajor(const float* a, const float* b, float* out) {
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += a[k * 4 + row] * b[col * 4 + k];
            }
            out[col * 4 + row] = sum;
        }
    }
}

// Transpose a row-major M*v matrix (NI layout, clip[r] = sum_c ni[r*4+c]*v[c])
// into Intel's column-major v*M layout: out[col*4 + row] = rowMajor[row*4 + col].
inline void transposeRowToColumnMajor(const float* rowMajor, float* outColMajor) {
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            outColMajor[col * 4 + row] = rowMajor[row * 4 + col];
        }
    }
}

// Clip-space (x, y, w) of a projected point; z is unused by the rasterizer.
struct ClipXYW {
    float x, y, w;
};

// Project a point through a column-major clip matrix. Same formula as Intel's
// TransformVerts, so sphere rects stay aligned with rasterised occluder depth.
inline ClipXYW projectWorld(const float* m, float wx, float wy, float wz) {
    return {
        wx * m[0] + wy * m[4] + wz * m[8] + m[12],
        wx * m[1] + wy * m[5] + wz * m[9] + m[13],
        wx * m[3] + wy * m[7] + wz * m[11] + m[15],
    };
}

// Per-frame sphere-projection metrics: operator norms of the x / y / w rows
// (translation excluded) of a column-major clip matrix. These are upper bounds
// on |d(clip)/d(pos)|. After transpose, row-r coefficients live at m[0+r],
// m[4+r], m[8+r].
struct RowNorms {
    float ndcRadiusX;
    float ndcRadiusY;
    float wGradMag;
};
inline RowNorms clipRowNorms(const float* m) {
    return {
        std::sqrt(m[0] * m[0] + m[4] * m[4] + m[8] * m[8]),
        std::sqrt(m[1] * m[1] + m[5] * m[5] + m[9] * m[9]),
        std::sqrt(m[3] * m[3] + m[7] * m[7] + m[11] * m[11]),
    };
}

}  // namespace msoc::clipmath
