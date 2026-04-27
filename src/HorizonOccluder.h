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
// The output curtain is a tiny set of triangles (~120 max) submitted to MOC
// via RenderTriangles with modelToClip=nullptr. Replaces a much larger raw
// terrain rasterization while preserving conservative occlusion semantics.
//
// Coordinate conventions (matches the reference impl this is ported from):
//   - x, y in clip-space NDC after perspective divide. y is up.
//   - Depth d is a scalar where LARGER = FARTHER (clip-w in this plugin).
//
// Adapted from MGE-XE's terrain_horizon_occluder.{h,c} (same author).
// Adaptations vs the reference (per LAYER-A handoff doc decisions):
//   D2: order-independent update (the reference's `test_and_update` had a
//       front-to-back prune semantic; near-terrain iteration order is not
//       guaranteed front-to-back, so the prune is dropped).
//   D3: d[c] is updated only when y_upper STRICTLY exceeds h[c] — the depth
//       written is "depth of the silhouette winner at this column," matching
//       the FAR-depth occluder semantic that the morefps lessons-learned
//       (§5.3) call out as the fundamental correctness requirement.
//   D4: C++ class instead of C ABI.
//   D5: std::vector for h, d, and simplify workspace; no allocator callbacks.
//   D12: depth-driven simplifier splits are content-driven via
//        computeAdaptiveEpsD — DO NOT pass kEpsD = 1e30 like MGE-XE's distant
//        path does; near-terrain depth heterogeneity makes that wrong.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace msoc::horizon {

// Sentinel for "no terrain seen in this column yet." Well below any plausible
// clip-space y so that update()'s "max" comparison reliably bumps out of it.
inline constexpr float kYBelow = -1.0e30f;

// One vertex of the curtain output, written first in NDC layout
// (x = ndc_x, y = ndc_y, z = depth, w = 1.0), then converted in-place by
// HorizonOccluder::fixupForMOC to MOC's pre-transformed clip-space layout
// (x = ndc_x * d, y = ndc_y * d, z = unused, w = d).
struct CurtainVertex {
    float x, y, z, w;
};

// One simplified horizon sample picked by simplify(). col is the source
// column; ndcX/h/d are the corresponding clip-space values pre-computed by
// the simplifier so the emit phase doesn't need the column-to-ndc map.
struct Sample {
    int   col;
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

    // Singleton accessor. Matches the global-state style of g_msoc /
    // g_msoc_prev / g_threadpool elsewhere in the plugin — one instance
    // per process, lazy-init on first call. Construction is cheap (no
    // allocation until init() is called); init() is idempotent.
    //
    // Call site: rasterizeAggregateTerrainHorizon does
    //   auto& horizon = HorizonOccluder::getInstance();
    //   horizon.reset();
    //   ... per-vertex horizon.update(...)
    //   const float adaptiveEpsD = horizon.computeAdaptiveEpsD(...);
    //   const int n = horizon.simplify(samples, ...);
    //   horizon.emitCurtainNDC(samples, n, ...);
    //   HorizonOccluder::fixupForMOC(verts, n*3*2);
    static HorizonOccluder& getInstance();

    // Allocate horizon arrays + simplify workspace. Idempotent — calling a
    // second time with the same args is a no-op; calling with different
    // args reallocates. Call once during plugin init or lazily on first
    // reset(). Returns true on success.
    bool init(int resolution = 512, int maxSamples = 60);

    // Reset every column to the sentinel state. Call once per frame BEFORE
    // any update() calls. Cheap: a single pass over the resolution-sized h/d
    // arrays.
    void reset();

    // Update the horizon at column c with a contribution (y_upper, far_depth).
    // Per D3: writes BOTH h[c] and d[c] only when y_upper > h[c]. The depth
    // tracks the winner of the silhouette comparison, not the max of all
    // contributors. Order-independent.
    //
    // Caller is responsible for c being in [0, resolution).
    void update(int c, float yUpper, float farDepth);

    // Same as update(), but applies the same (yUpper, farDepth) pair to an
    // inclusive column range [c0, c1]. Useful when a node projects to a
    // multi-column bbox with a single y_upper.
    void updateRange(int c0, int c1, float yUpper, float farDepth);

    // Run the budget-bounded Douglas-Peucker simplifier. Writes up to
    // maxSamples into out[]. Returns the actual sample count (>= 2 unless
    // resolution < 2). Endpoints (col 0 and resolution-1) are always
    // included.
    //
    // Error metric: max(|delta_h|/epsH, |delta_d|/epsD). A subinterval is
    // split when its max-error column exceeds the threshold; stops when the
    // budget fills or no interval has error > 1.
    //
    // tile_align > 1 snaps non-endpoint columns to the nearest multiple
    // (mirrors MOC's HiZ tile width — 32 px, but a horizon resolution of
    // 512 with tile_align=16 gives ~32-px granularity in a 1024-wide mask).
    //
    // CRITICAL: pass an ADAPTIVE epsD from computeAdaptiveEpsD(), not a
    // fixed value (see D12 in the handoff doc). The reference impl in
    // MGE-XE uses 1e30 to disable the depth term — that works for distant
    // land's depth-uniform content but silently degrades occlusion for
    // near-terrain's heterogeneous depths.
    int simplify(Sample* out, int maxSamples,
                 float epsH, float epsD, int tileAlign);

    // Compute an adaptive epsD threshold from the current horizon's depth
    // range. Returns clamp_low(fraction * (maxD - minD), floor) over
    // non-sentinel columns. Returns +infinity if fewer than 2 columns have
    // data (fallback equivalent to "disable depth-driven splits" — safe
    // because the simplifier with zero depth-error contribution falls back
    // to height-only behavior, matching MGE-XE's distant-land setup).
    //
    // Call between the build phase (last update()) and simplify().
    float computeAdaptiveEpsD(float fraction = 0.05f, float floor_ = 100.0f) const;

    // Emit curtain triangles in NDC layout. Per segment [s_i, s_{i+1}]:
    //   y_top    = min(s_i.h, s_{i+1}.h)   (conservative — stays below true silhouette)
    //   z        = max(s_i.d, s_{i+1}.d)   (conservative — at/behind terrain)
    //   y_bottom = ndcYBottom              (typically -1.1f to extend past screen)
    //
    // Output format: w=1.0, depth in z slot. Caller MUST run fixupForMOC()
    // before submitting to MOC's RenderTriangles.
    //
    // Caller must size outVerts >= 6 * (nSamples - 1). Returns triangle count
    // (segments where both endpoint heights are at the sentinel are skipped,
    // so triCount can be < nSamples - 1).
    int emitCurtainNDC(const Sample* samples, int nSamples,
                       float ndcYBottom,
                       CurtainVertex* outVerts, int outVertsCapacity);

    // Convert NDC-layout verts to MOC's pre-transformed clip-space layout
    // in place. Reads (x, y, z=depth, w=1) and writes (x*d, y*d, z=unused, w=d).
    //
    // After this fixup, submit to MOC with VertexLayout(stride=16, offY=4,
    // offW=12) and modelToClip=nullptr.
    static void fixupForMOC(CurtainVertex* verts, int vertCount);

    int resolution() const { return m_resolution; }

    // Telemetry: count of columns with non-sentinel data after the build.
    // Linear scan over h[]; called at most once per frame.
    int columnsTouched() const;

    // Diagnostic accessors — used only by validation and Step 6 troubleshooting.
    // Not for hot-path use.
    float heightAt(int c) const { return m_h[static_cast<size_t>(c)]; }
    float depthAt (int c) const { return m_d[static_cast<size_t>(c)]; }

private:
    int m_resolution = 0;
    int m_maxSamples = 0;
    std::vector<float>   m_h;          // [resolution] — sentinel kYBelow
    std::vector<float>   m_d;          // [resolution] — paired with m_h via D3
    std::vector<uint8_t> m_workspace;  // simplify scratch (keep[] + heap[])
};

} // namespace msoc::horizon
