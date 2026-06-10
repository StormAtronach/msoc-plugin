#pragma once

// Conservative screen-space terrain occluder built from a 1D horizon buffer.
//
// Per-frame pipeline:
//   reset()                              once at frame start
//   update(c, y_upper, far_depth) ...    once per terrain vertex contribution
//   computeAdaptiveEpsD(...)             once after the build phase
//   simplify(out, max, epsH, epsD, ...)  once before emit
//   emitCurtainNDC(samples, ...)         once to produce curtain triangles
//   HorizonOccluder::fixupForMOC(...)    once to convert NDC verts to MOC's
//                                        pre-transformed clip-space layout
//
// Output is ~120 triangles submitted to MOC via RenderTriangles with
// modelToClip=nullptr - replaces a much larger raw terrain rasterization
// while preserving conservative occlusion semantics.
//
// Coordinate conventions:
//   x, y in clip-space NDC after perspective divide. y is up.
//   Depth d is a scalar where LARGER = FARTHER (clip-w in this plugin).
//
// Adapted from MGE-XE's terrain_horizon_occluder. Notable adaptations:
//   - Order-independent update (no front-to-back prune); near-terrain
//     iteration order is not guaranteed front-to-back.
//   - d[c] is updated only when y_upper STRICTLY exceeds h[c] - the depth
//     written tracks the silhouette winner, matching the FAR-depth
//     occluder semantic required for correctness.
//   - simplify() takes an ADAPTIVE epsD from computeAdaptiveEpsD(), not
//     a fixed 1e30 like MGE-XE's distant path. Near-terrain depth
//     heterogeneity makes a disabled-depth-term setup silently wrong.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace msoc::horizon {

// "No terrain seen in this column yet." Well below any plausible clip-space y.
inline constexpr float kYBelow = -1.0e30f;

// Curtain vertex. Written first in NDC layout (x, y, z=depth, w=1.0),
// then converted in place by fixupForMOC to MOC's pre-transformed
// clip-space layout (x*d, y*d, z=unused, w=d).
struct CurtainVertex {
    float x, y, z, w;
};

// One simplified horizon sample. col is the source column; ndcX/h/d are
// pre-computed by the simplifier so the emit phase doesn't need the
// column->ndc map.
struct Sample {
    int col;
    float ndcX;
    float h;
    float d;
};

class HorizonOccluder {
public:
    HorizonOccluder() = default;
    ~HorizonOccluder() = default;
    HorizonOccluder(const HorizonOccluder&) = delete;
    HorizonOccluder& operator=(const HorizonOccluder&) = delete;

    // One instance per process, lazy-init on first call. Construction
    // is cheap; init() is idempotent. Used in pipeline order:
    //   auto& horizon = HorizonOccluder::getInstance();
    //   horizon.reset();
    //   ... per-vertex horizon.update(...)
    //   const float adaptiveEpsD = horizon.computeAdaptiveEpsD(...);
    //   const int n = horizon.simplify(samples, ...);
    //   horizon.emitCurtainNDC(samples, n, ...);
    //   HorizonOccluder::fixupForMOC(verts, n*3*2);
    static HorizonOccluder& getInstance();

    // Allocate horizon arrays + simplify workspace. Idempotent - same
    // args are a no-op, different args reallocate. Returns true on success.
    bool init(int resolution = 512, int maxSamples = 60);

    // Reset every column to the sentinel state. Call once per frame
    // before any update() calls.
    void reset();

    // Update column c with (y_upper, far_depth). Writes both h[c] and
    // d[c] only when y_upper > h[c]; depth tracks the silhouette winner.
    // Order-independent. Caller ensures c in [0, resolution).
    void update(int c, float yUpper, float farDepth);

    // Apply (yUpper, farDepth) to an inclusive column range [c0, c1].
    void updateRange(int c0, int c1, float yUpper, float farDepth);

    // Budget-bounded Douglas-Peucker simplifier. Writes up to maxSamples
    // into out[]; returns actual count (>= 2 unless resolution < 2).
    // Endpoints (col 0 and resolution-1) are always included.
    //
    // Error metric: max(|delta_h|/epsH, |delta_d|/epsD). Splits a
    // subinterval when its max-error column exceeds 1; stops at budget
    // or when no interval has error > 1.
    //
    // tile_align > 1 snaps non-endpoint columns to the nearest multiple
    // (mirrors MOC's HiZ tile width).
    //
    // CRITICAL: pass an ADAPTIVE epsD from computeAdaptiveEpsD(), not
    // a fixed value. See header comment.
    int simplify(Sample* out, int maxSamples,
                 float epsH, float epsD, int tileAlign);

    // Adaptive epsD threshold from the current horizon's depth range:
    // clamp_low(fraction * (maxD - minD), floor) over non-sentinel
    // columns. Returns +infinity when fewer than 2 columns have data
    // (falls back to height-only simplification).
    float computeAdaptiveEpsD(float fraction = 0.05f, float floor_ = 100.0f) const;

    // Emit curtain triangles in NDC layout. Per segment [s_i, s_{i+1}]:
    //   y_top    = min(s_i.h, s_{i+1}.h)   conservative - below silhouette
    //   z        = max(s_i.d, s_{i+1}.d)   conservative - at/behind terrain
    //   y_bottom = ndcYBottom              typically -1.1f, extends past screen
    //
    // Output: w=1.0, depth in z slot. Caller MUST run fixupForMOC()
    // before submitting to MOC's RenderTriangles. outVerts must be
    // >= 6 * (nSamples - 1). Returns triangle count (segments with both
    // endpoints at the sentinel are skipped, so triCount can be less).
    int emitCurtainNDC(const Sample* samples, int nSamples,
                       float ndcYBottom,
                       CurtainVertex* outVerts, int outVertsCapacity);

    // Convert NDC-layout verts to MOC's pre-transformed clip-space
    // layout in place: reads (x, y, z=depth, w=1), writes (x*d, y*d,
    // z=unused, w=d). After this, submit with VertexLayout(stride=16,
    // offY=4, offW=12) and modelToClip=nullptr.
    static void fixupForMOC(CurtainVertex* verts, int vertCount);

    int resolution() const { return m_resolution; }

    // Count of columns with non-sentinel data after build. Linear scan;
    // call at most once per frame.
    int columnsTouched() const;

    // Diagnostic accessors - validation use only, not for hot path.
    float heightAt(int c) const { return m_h[static_cast<size_t>(c)]; }
    float depthAt(int c) const { return m_d[static_cast<size_t>(c)]; }

private:
    int m_resolution = 0;
    int m_maxSamples = 0;
    std::vector<float> m_h;
    std::vector<float> m_d;
    std::vector<uint8_t> m_workspace;  // simplify scratch (keep[] + heap[])
};

}  // namespace msoc::horizon
