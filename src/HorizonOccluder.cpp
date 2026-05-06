// Adapted from MGE-XE's terrain_horizon_occluder.c. See HorizonOccluder.h
// for the API and design notes.

#include "HorizonOccluder.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace msoc::horizon {

namespace {

// Linear NDC-x for column c with resolution R: ndc_x = -1 + 2*c / (R-1).
inline float colToNdc(int c, int resolution) {
    return -1.0f + 2.0f * static_cast<float>(c)
                       / static_cast<float>(resolution - 1);
}

// One interval in the simplifier's max-heap. err is the maximum
// normalized error from the linear interpolant; split is the column at
// which it occurred (-1 if too short to split).
struct HeapItem {
    int   a;
    int   b;
    int   split;
    float err;
};

inline void heapSiftUp(HeapItem* items, int i) {
    while (i > 0) {
        const int parent = (i - 1) >> 1;
        if (items[parent].err >= items[i].err) break;
        std::swap(items[parent], items[i]);
        i = parent;
    }
}

inline void heapSiftDown(HeapItem* items, int size, int i) {
    for (;;) {
        const int l = (i << 1) + 1;
        const int r = l + 1;
        int best = i;
        if (l < size && items[l].err > items[best].err) best = l;
        if (r < size && items[r].err > items[best].err) best = r;
        if (best == i) break;
        std::swap(items[best], items[i]);
        i = best;
    }
}

// Find interior column of (a, b) with largest normalized error from the
// linear interpolant. eps_h <= 0 (or eps_d) disables that axis.
HeapItem computeWorst(const std::vector<float>& h, const std::vector<float>& d,
                      int a, int b, float epsH, float epsD) {
    HeapItem item{a, b, -1, 0.0f};
    if (b - a < 2) return item;

    const float hA = h[static_cast<size_t>(a)];
    const float hB = h[static_cast<size_t>(b)];
    const float dA = d[static_cast<size_t>(a)];
    const float dB = d[static_cast<size_t>(b)];

    const float invSpan = 1.0f / static_cast<float>(b - a);
    const float invEpsH = (epsH > 0.0f) ? 1.0f / epsH : 0.0f;
    const float invEpsD = (epsD > 0.0f) ? 1.0f / epsD : 0.0f;

    int   bestIdx = -1;
    float bestErr = 0.0f;

    for (int i = a + 1; i < b; ++i) {
        const float t = static_cast<float>(i - a) * invSpan;
        const float hLerp = hA + t * (hB - hA);
        const float dLerp = dA + t * (dB - dA);
        const float eH = std::fabs(h[static_cast<size_t>(i)] - hLerp) * invEpsH;
        const float eD = std::fabs(d[static_cast<size_t>(i)] - dLerp) * invEpsD;
        const float e  = (eH > eD) ? eH : eD;
        if (e > bestErr) {
            bestErr = e;
            bestIdx = i;
        }
    }

    item.split = bestIdx;
    item.err   = bestErr;
    return item;
}

inline CurtainVertex makeNDCVert(float x, float y, float depth) {
    return CurtainVertex{x, y, depth, 1.0f};
}

} // namespace

HorizonOccluder& HorizonOccluder::getInstance() {
    static HorizonOccluder instance;
    return instance;
}

bool HorizonOccluder::init(int resolution, int maxSamples) {
    if (resolution < 2 || maxSamples < 2) return false;
    if (resolution == m_resolution && maxSamples == m_maxSamples) {
        return true;
    }

    m_resolution = resolution;
    m_maxSamples = maxSamples;

    m_h.assign(static_cast<size_t>(resolution), kYBelow);
    m_d.assign(static_cast<size_t>(resolution), 0.0f);

    // Workspace: keep[resolution] (one byte per column) followed by a
    // heap of HeapItem. 2*maxSamples is the worst-case bound — every
    // split adds at most two new intervals.
    const size_t keepBytes = (static_cast<size_t>(resolution) + 15u) & ~size_t{15u};
    const size_t heapBytes = static_cast<size_t>(2 * maxSamples) * sizeof(HeapItem);
    m_workspace.assign(keepBytes + heapBytes, 0);

    return true;
}

void HorizonOccluder::reset() {
    if (m_h.empty()) return;
    std::fill(m_h.begin(), m_h.end(), kYBelow);
    std::fill(m_d.begin(), m_d.end(), 0.0f);
}

void HorizonOccluder::update(int c, float yUpper, float farDepth) {
    // Strict > so multiple contributors at one column converge to (h, d)
    // where h = max(yUpper) and d = depth-of-the-winner.
    if (yUpper > m_h[static_cast<size_t>(c)]) {
        m_h[static_cast<size_t>(c)] = yUpper;
        m_d[static_cast<size_t>(c)] = farDepth;
    }
}

void HorizonOccluder::updateRange(int c0, int c1, float yUpper, float farDepth) {
    if (c0 < 0) c0 = 0;
    if (c1 >= m_resolution) c1 = m_resolution - 1;
    for (int c = c0; c <= c1; ++c) {
        update(c, yUpper, farDepth);
    }
}

float HorizonOccluder::computeAdaptiveEpsD(float fraction, float floor_) const {
    // m_d is meaningful only where m_h has been set, so the same predicate
    // covers both arrays.
    float minD = std::numeric_limits<float>::infinity();
    float maxD = -std::numeric_limits<float>::infinity();
    int   activeCols = 0;

    const int n = m_resolution;
    for (int c = 0; c < n; ++c) {
        if (m_h[static_cast<size_t>(c)] <= kYBelow + 1.0e-29f) continue;
        const float dc = m_d[static_cast<size_t>(c)];
        if (dc < minD) minD = dc;
        if (dc > maxD) maxD = dc;
        ++activeCols;
    }

    if (activeCols < 2) {
        // No meaningful depth structure — disable the depth term.
        return std::numeric_limits<float>::infinity();
    }

    const float range = maxD - minD;
    const float candidate = fraction * range;
    return (candidate > floor_) ? candidate : floor_;
}

int HorizonOccluder::simplify(Sample* out, int maxSamples,
                              float epsH, float epsD, int tileAlign) {
    if (!out || maxSamples < 2) return 0;
    if (m_resolution < 2) return 0;
    if (maxSamples > m_maxSamples) {
        // Workspace is sized for m_maxSamples. Cap and proceed.
        maxSamples = m_maxSamples;
    }

    const int R = m_resolution;
    const size_t keepBytes = (static_cast<size_t>(R) + 15u) & ~size_t{15u};
    uint8_t* keep = m_workspace.data();
    HeapItem* heapItems = reinterpret_cast<HeapItem*>(m_workspace.data() + keepBytes);
    int heapSize = 0;
    const int heapCap = 2 * m_maxSamples;

    std::memset(keep, 0, static_cast<size_t>(R));
    keep[0] = 1;
    keep[R - 1] = 1;
    int kept = 2;

    HeapItem seed = computeWorst(m_h, m_d, 0, R - 1, epsH, epsD);
    if (seed.split >= 0 && seed.err > 1.0f && heapSize < heapCap) {
        heapItems[heapSize++] = seed;
        heapSiftUp(heapItems, heapSize - 1);
    }

    // DP loop: pop worst interval, split at max-error column, push the
    // halves if they exceed threshold. Stop at budget or when no
    // interval has err > 1.
    while (kept < maxSamples) {
        if (heapSize == 0) break;
        HeapItem top = heapItems[0];
        --heapSize;
        if (heapSize > 0) {
            heapItems[0] = heapItems[heapSize];
            heapSiftDown(heapItems, heapSize, 0);
        }

        if (top.err <= 1.0f) break;
        if (top.split <= top.a || top.split >= top.b) continue;

        keep[top.split] = 1;
        ++kept;

        HeapItem L = computeWorst(m_h, m_d, top.a, top.split, epsH, epsD);
        HeapItem Rg = computeWorst(m_h, m_d, top.split, top.b, epsH, epsD);
        if (L.split >= 0 && L.err > 1.0f && heapSize < heapCap) {
            heapItems[heapSize++] = L;
            heapSiftUp(heapItems, heapSize - 1);
        }
        if (Rg.split >= 0 && Rg.err > 1.0f && heapSize < heapCap) {
            heapItems[heapSize++] = Rg;
            heapSiftUp(heapItems, heapSize - 1);
        }
    }

    // Snap non-endpoints to the nearest tileAlign multiple. Up to
    // half-a-tile movement; absorbed by per-segment conservative values.
    if (tileAlign > 1) {
        for (int c = 1; c < R - 1; ++c) {
            if (!keep[c]) continue;
            int snapped = ((c + tileAlign / 2) / tileAlign) * tileAlign;
            if (snapped <= 0) snapped = 1;
            if (snapped >= R - 1) snapped = R - 2;
            if (snapped != c) {
                keep[c] = 0;
                if (!keep[snapped]) keep[snapped] = 1;
            }
        }
    }

    int n = 0;
    for (int c = 0; c < R && n < maxSamples; ++c) {
        if (!keep[c]) continue;
        out[n].col   = c;
        out[n].ndcX  = colToNdc(c, R);
        out[n].h     = m_h[static_cast<size_t>(c)];
        out[n].d     = m_d[static_cast<size_t>(c)];
        ++n;
    }

    // Force endpoints if simplify somehow returned < 2.
    if (n < 2) {
        out[0].col  = 0;
        out[0].ndcX = -1.0f;
        out[0].h    = m_h.front();
        out[0].d    = m_d.front();
        out[1].col  = R - 1;
        out[1].ndcX = 1.0f;
        out[1].h    = m_h.back();
        out[1].d    = m_d.back();
        n = 2;
    }
    return n;
}

int HorizonOccluder::emitCurtainNDC(const Sample* samples, int nSamples,
                                    float ndcYBottom,
                                    CurtainVertex* outVerts, int outVertsCapacity) {
    if (!samples || !outVerts || nSamples < 2) return 0;
    const int segs = nSamples - 1;
    if (outVertsCapacity < 6 * segs) return 0;

    int w = 0;
    for (int i = 0; i < segs; ++i) {
        const Sample& s0 = samples[i];
        const Sample& s1 = samples[i + 1];

        // Conservative: y_top below silhouette (min), z at/behind
        // terrain (max — farther depth in clip-w convention).
        const float yTop = (s0.h < s1.h) ? s0.h : s1.h;
        const float z    = (s0.d > s1.d) ? s0.d : s1.d;

        // Skip segments where neither endpoint has been touched —
        // otherwise the kYBelow sentinel projects to garbage quads.
        if (yTop <= kYBelow * 0.5f) continue;

        const CurtainVertex TL = makeNDCVert(s0.ndcX, yTop,        z);
        const CurtainVertex TR = makeNDCVert(s1.ndcX, yTop,        z);
        const CurtainVertex BL = makeNDCVert(s0.ndcX, ndcYBottom,  z);
        const CurtainVertex BR = makeNDCVert(s1.ndcX, ndcYBottom,  z);

        // CCW winding for y-up; MOC accepts BACKFACE_NONE so it's cosmetic.
        outVerts[w++] = TL;
        outVerts[w++] = BL;
        outVerts[w++] = BR;
        outVerts[w++] = TL;
        outVerts[w++] = BR;
        outVerts[w++] = TR;
    }
    return w / 3;
}

void HorizonOccluder::fixupForMOC(CurtainVertex* verts, int vertCount) {
    // (x, y, z=depth, w=1) → (x*d, y*d, _, w=d). MOC's RenderTriangles
    // expects this layout when modelToClip is nullptr.
    for (int i = 0; i < vertCount; ++i) {
        const float d = verts[i].z;
        verts[i].x *= d;
        verts[i].y *= d;
        verts[i].w  = d;
    }
}

int HorizonOccluder::columnsTouched() const {
    int n = 0;
    for (float v : m_h) {
        if (v > kYBelow + 1.0e-29f) ++n;
    }
    return n;
}

} // namespace msoc::horizon
