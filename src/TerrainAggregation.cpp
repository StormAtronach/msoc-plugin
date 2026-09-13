// Terrain occluder aggregation: merges each near Land into one MOC
// submission. Split from OcclusionPass.cpp; shared state via
// OcclusionInternal.h.

#include "OcclusionApi.h"
#include "OcclusionInternal.h"

#include "NICamera.h"
#include "NINode.h"
#include "NIAVObject.h"
#include "NIPoint3.h"
#include "NIPoint4.h"
#include "NITriShape.h"
#include "NITriShapeData.h"
#include "NIGeometryData.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>  // numeric_limits; was reaching this through the PCH
#include <vector>

namespace msoc::occlusion::terrain {

// ============================================================
// Terrain aggregation
// ============================================================

// Frustum check for the aggregate-terrain walk. No mask bookkeeping -
// runs before cullShowBody so usedCullingPlanesBitfield stays clean.
static bool frustumCulledSphere(NI::AVObject* obj, NI::Camera* camera) {
    const int n = cameraCountCullingPlanes(camera);
    const float r = obj->worldBoundRadius;
    for (int i = 0; i < n; ++i) {
        const auto* plane = cameraCullingPlane(camera, i);
        const float d = plane->x * obj->worldBoundOrigin.x + plane->y * obj->worldBoundOrigin.y + plane->z * obj->worldBoundOrigin.z - plane->w;
        if (d <= -r) return true;
    }
    return false;
}

// True iff the Land's cell is within the 3x3 active grid around the eye's
// cell (Chebyshev distance <= 1). Both terrain paths skip anything beyond it:
// the engine renders nothing there that terrain could hide, distant land is
// MGE-XE's and has its own visibility handling, and the outer lands are the
// most triangles for the least occlusion. Land bounds are centred on the
// cell, so floor(origin / 8192) is the cell coordinate.
static bool landWithinActiveGrid(NI::AVObject* land, NI::Camera* camera) {
    constexpr float kCellSize = 8192.0f;
    const auto& eye = camera->worldTransform.translation;
    const int eyeCx = static_cast<int>(std::floor(eye.x / kCellSize));
    const int eyeCy = static_cast<int>(std::floor(eye.y / kCellSize));
    const int landCx = static_cast<int>(std::floor(land->worldBoundOrigin.x / kCellSize));
    const int landCy = static_cast<int>(std::floor(land->worldBoundOrigin.y / kCellSize));
    const int dx = landCx - eyeCx;
    const int dy = landCy - eyeCy;
    return dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1;
}

// Append one terrain TriShape (25v/32t) to the aggregate buffers,
// transforming to world space and offsetting indices. Mirrors
// rasterizeTriShape's vertex math without the gates/skinning/thin-
// axis paths.
static void appendTerrainShape(std::vector<float>& aggVerts,
                               std::vector<unsigned int>& aggIdx, NI::TriShape* shape,
                               unsigned int step) {
    auto data = shape->getModelData();
    if (!data) return;
    const unsigned short vcount = data->getActiveVertexCount();
    if (vcount == 0 || data->vertex == nullptr) return;
    const unsigned short tcount = data->getActiveTriangleCount();
    const NI::Triangle* tris = data->getTriList();
    if (tcount == 0 || tris == nullptr) return;

    const auto& xf = shape->worldTransform;
    const auto& R = xf.rotation;
    const auto& T = xf.translation;
    const float s = xf.scale;

    // Downsample fast path. Terrain subcells are 5x5 row-major grids
    // (25v/32t). step=2 -> 3x3 (8 tris); step=4 -> 2x2 (2 tris). Sample
    // every step-th vertex; min-z over the dropped neighbours so the
    // silhouette can only shrink (conservative under-occlude). Other
    // vcount/step combos fall through to full-resolution. Only 2/4
    // divide the 4-quad edge cleanly without dropping the seam.
    if (vcount == 25 && (step == 2 || step == 4)) {
        // Coarse-grid axis count: step=2 -> 3 verts/edge, step=4 -> 2.
        const unsigned int n = (4u / step) + 1u;
        const unsigned int baseVert = static_cast<unsigned int>(aggVerts.size() / 3);
        aggVerts.resize(aggVerts.size() + static_cast<size_t>(n * n) * 3);
        float* out = aggVerts.data() + baseVert * 3;

        // For each kept (cr, cc), take min-world-z over the source-
        // grid neighbourhood. Right/bottom seam verts are only
        // covered by the last-row/column kept vertex - the seam
        // already matches the next subcell exactly.
        for (unsigned int cr = 0; cr < n; ++cr) {
            for (unsigned int cc = 0; cc < n; ++cc) {
                const unsigned int sr0 = cr * step;
                const unsigned int sc0 = cc * step;
                const unsigned int sr1 = (cr + 1 == n) ? sr0 : sr0 + (step - 1);
                const unsigned int sc1 = (cc + 1 == n) ? sc0 : sc0 + (step - 1);

                float minWz = std::numeric_limits<float>::infinity();
                float keepX = 0, keepY = 0;
                // XY anchored at (sr0, sc0); only Z is min-folded.
                for (unsigned int sr = sr0; sr <= sr1; ++sr) {
                    for (unsigned int sc = sc0; sc <= sc1; ++sc) {
                        const auto& v = data->vertex[sr * 5 + sc];
                        const float rx = R.m0.x * v.x + R.m0.y * v.y + R.m0.z * v.z;
                        const float ry = R.m1.x * v.x + R.m1.y * v.y + R.m1.z * v.z;
                        const float rz = R.m2.x * v.x + R.m2.y * v.y + R.m2.z * v.z;
                        const float wx = rx * s + T.x;
                        const float wy = ry * s + T.y;
                        const float wz = rz * s + T.z;
                        if (sr == sr0 && sc == sc0) {
                            keepX = wx;
                            keepY = wy;
                        }
                        if (wz < minWz) minWz = wz;
                    }
                }
                float* dst = out + (cr * n + cc) * 3;
                dst[0] = keepX;
                dst[1] = keepY;
                dst[2] = minWz;
            }
        }

        // (n-1)*(n-1)*2 triangles. Winding must match the full-resolution
        // path, which copies the source patches' own CCW order: the same
        // OcclusionOccluderCCWOnly gate (BACKFACE_CW) runs at submit time, so
        // a coarse quad emitted CW is silently culled. It was, and with the
        // default Half/CCW-only config that dropped most of the downsampled
        // terrain from the mask and left black fractures across the hills.
        // (v00, v01, v10) + (v10, v01, v11) is CCW under this row-major grid;
        // verified against every dumped patch.
        const unsigned int qN = n - 1;
        aggIdx.reserve(aggIdx.size() + static_cast<size_t>(qN * qN) * 6);
        for (unsigned int qr = 0; qr < qN; ++qr) {
            for (unsigned int qc = 0; qc < qN; ++qc) {
                const unsigned int v00 = baseVert + (qr * n + qc);
                const unsigned int v01 = baseVert + (qr * n + (qc + 1));
                const unsigned int v10 = baseVert + ((qr + 1) * n + qc);
                const unsigned int v11 = baseVert + ((qr + 1) * n + (qc + 1));
                aggIdx.push_back(v00);
                aggIdx.push_back(v01);
                aggIdx.push_back(v10);
                aggIdx.push_back(v10);
                aggIdx.push_back(v01);
                aggIdx.push_back(v11);
            }
        }
        return;
    }

    // Full-resolution path: copy every source vert + every source tri.
    const unsigned int baseVert = static_cast<unsigned int>(aggVerts.size() / 3);
    aggVerts.resize(aggVerts.size() + static_cast<size_t>(vcount) * 3);
    float* out = aggVerts.data() + baseVert * 3;
    for (unsigned short i = 0; i < vcount; ++i) {
        const auto& v = data->vertex[i];
        const float rx = R.m0.x * v.x + R.m0.y * v.y + R.m0.z * v.z;
        const float ry = R.m1.x * v.x + R.m1.y * v.y + R.m1.z * v.z;
        const float rz = R.m2.x * v.x + R.m2.y * v.y + R.m2.z * v.z;
        out[i * 3 + 0] = rx * s + T.x;
        out[i * 3 + 1] = ry * s + T.y;
        out[i * 3 + 2] = rz * s + T.z;
    }

    aggIdx.reserve(aggIdx.size() + static_cast<size_t>(tcount) * 3);
    for (unsigned short i = 0; i < tcount; ++i) {
        const unsigned short a = tris[i].vertices[0];
        const unsigned short b = tris[i].vertices[1];
        const unsigned short c = tris[i].vertices[2];
        if (a >= vcount || b >= vcount || c >= vcount) continue;
        aggIdx.push_back(baseVert + a);
        aggIdx.push_back(baseVert + b);
        aggIdx.push_back(baseVert + c);
    }
}

// 0=Full (5x5, 32 tris), 1=Half (3x3, 8 tris), 2=Corners (2x2, 2 tris).
// Out-of-range clamps to Full so malformed JSON doesn't disable terrain.
static unsigned int currentTerrainStep() {
    switch (g_frame.terrainResolution) {
        case 1:
            return 2;
        case 2:
            return 4;
        default:
            return 1;
    }
}

// Build VB+IB for one per-Land NiNode (cache-miss path). Walks every
// subcell unconditionally - result must be valid for any camera angle.
// MSOC clips internally; extra off-frustum triangles cost negligibly
// vs the per-frame walk + transform they replace.
static void buildLandCacheEntry(LandCacheEntry& entry, NI::Node* landNode) {
    entry.verts.clear();
    entry.indices.clear();
    entry.subcellRanges.clear();
    const unsigned int step = currentTerrainStep();
    const auto& subcells = landNode->children;
    for (size_t j = 0; j < subcells.endIndex; ++j) {
        auto* sub = subcells.storage[j].get();
        if (!sub) continue;
        if (!sub->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;
        auto* subNode = static_cast<NI::Node*>(sub);

        // Bracket this subcell's index range so the submit path can
        // frustum-cull at subcell granularity.
        const unsigned int firstIdxBefore = static_cast<unsigned int>(entry.indices.size());

        const auto& shapes = subNode->children;
        for (size_t k = 0; k < shapes.endIndex; ++k) {
            auto* shape = shapes.storage[k].get();
            if (!shape) continue;
            if (!shape->isInstanceOfType(NI::RTTIStaticPtr::NiTriShape)) continue;
            const auto p = classify::occluderProperties(shape);
            if (p.alpha || p.stencil) continue;
            appendTerrainShape(entry.verts, entry.indices, static_cast<NI::TriShape*>(shape), step);
        }

        const unsigned int firstIdxAfter = static_cast<unsigned int>(entry.indices.size());
        if (firstIdxAfter > firstIdxBefore) {
            LandCacheEntry::SubcellRange r;
            r.node = subNode;
            r.firstIdx = firstIdxBefore;
            r.triCount = (firstIdxAfter - firstIdxBefore) / 3;
            entry.subcellRanges.push_back(r);
        }
    }
    entry.triCount = static_cast<unsigned int>(entry.indices.size() / 3);
    entry.builtForResolution = static_cast<uint8_t>(g_frame.terrainResolution);
}

// Mark-and-sweep refresh of the per-Land world-space occluder cache.
// Walks WorldLandscape, get-or-builds an entry per Land (rebuilding when
// the terrain-resolution dropdown changed), and evicts entries whose
// NiNode wasn't seen this frame.
//
// Lands outside the 3x3 active grid are neither built nor kept (they fall
// to the sweep). Precondition: g_worldLandscapeRoot is non-null (the caller
// guards).
static void refreshLandCache(NI::Camera* camera) {
    for (auto& kv : g_caches.land) kv.second.seen = false;

    const auto& landChildren = g_worldLandscapeRoot->children;
    for (size_t i = 0; i < landChildren.endIndex; ++i) {
        auto* land = landChildren.storage[i].get();
        if (!land) continue;
        if (!land->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;
        if (!landWithinActiveGrid(land, camera)) continue;

        auto* landNode = static_cast<NI::Node*>(land);

        // Get-or-build the cached buffer. Keyed by NiNode pointer;
        // a cell-change realloc produces a new key and rebuilds.
        auto [it, inserted] = g_caches.land.try_emplace(landNode);
        LandCacheEntry& entry = it->second;
        // Resolution-mismatch on hit forces a rebuild - MCM dropdown
        // changes propagate lazily. nodePtr stays valid (NI::Pointer
        // pins the NiNode); only verts/indices redo.
        const uint8_t curRes = static_cast<uint8_t>(g_frame.terrainResolution);
        if (inserted) {
            entry.nodePtr = landNode;
            buildLandCacheEntry(entry, landNode);
            ++g_caches.landMisses;
        } else if (entry.builtForResolution != curRes) {
            buildLandCacheEntry(entry, landNode);
            ++g_caches.landMisses;
        } else {
            ++g_caches.landHits;
        }
        entry.seen = true;
    }

    // Sweep stale entries. Safe to free now because ClearBuffer() at
    // top-level entry already flushed previous-frame async work that
    // referenced these pointers.
    for (auto it = g_caches.land.begin(); it != g_caches.land.end();) {
        if (!it->second.seen) {
            it = g_caches.land.erase(it);
            ++g_caches.landEvictions;
        } else {
            ++it;
        }
    }
}

// Aggregate terrain rasteriser. Walks WorldLandscape (root -> Land ->
// 16 subcells -> N NiTriShapes) and submits one combined occluder per
// visible Land. Individual 25v/32t patches fail the thin-axis gate;
// merging gives the hill silhouette that actually occludes
// distant architecture.
//
// Must run inside isTopLevel - after ClearBuffer + uploadCameraTransform,
// before cullShowBody - so the drain sees terrain in the depth buffer.
//
// Uses g_caches.land to amortise the per-Land walk + vertex transform.
// Mark-and-sweep evicts entries whose NiNode wasn't seen this frame.
void rasterizeAggregate(NI::Camera* camera) {
    if (!g_worldLandscapeRoot) return;
    if (g_worldLandscapeRoot->getAppCulled()) return;

    ScopedUsAccumulator timer(g_stats.aggregateTerrainUs);

    // Build/refresh the shared per-Land cache (mark-and-sweep). After
    // this every live Land within the active grid has an up-to-date entry
    // in g_caches.land.
    refreshLandCache(camera);

    const auto& landChildren = g_worldLandscapeRoot->children;
    for (size_t i = 0; i < landChildren.endIndex; ++i) {
        auto* land = landChildren.storage[i].get();
        if (!land) continue;
        if (!land->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;

        auto* landNode = static_cast<NI::Node*>(land);

        // refreshLandCache guarantees presence for every live Land.
        auto it = g_caches.land.find(landNode);
        if (it == g_caches.land.end()) continue;
        LandCacheEntry& entry = it->second;

        // Per-frame submit gates. Shape-level filters are intentionally
        // omitted from cache build - they'd bake view-specific state
        // into a reusable buffer.
        if (land->getAppCulled()) continue;
        if (!landWithinActiveGrid(land, camera)) continue;
        if (frustumCulledSphere(land, camera)) continue;
        if (entry.triCount == 0) continue;

        // Per-subcell frustum cull. ~5-8 of a Land's 16 subcells are
        // typically in-frustum; skipping the rest trades one big
        // submission for N small ones with fewer total triangles -
        // net win on per-tile cost and async queue pressure.
        //
        // Empty subcellRanges fallback (stale cache from before the
        // field landed) submits the whole entry as one range.
        unsigned int submittedTris = 0;
        if (!entry.subcellRanges.empty()) {
            for (const auto& range : entry.subcellRanges) {
                if (range.triCount == 0) continue;
                if (range.node && frustumCulledSphere(range.node, camera)) continue;

                if (g_asyncThisFrame) {
                    ScopedUsAccumulator t(g_stats.rasterizeTimeUs);
                    g_threadpool->RenderTriangles(
                        entry.verts.data(),
                        entry.indices.data() + range.firstIdx,
                        static_cast<int>(range.triCount),
                        g_frame.occluderWinding,
                        ::MaskedOcclusionCulling::CLIP_PLANE_ALL);
                    ++g_stats.asyncJobsQueued;
                } else {
                    ScopedUsAccumulator t(g_stats.rasterizeTimeUs);
                    g_msoc->RenderTriangles(
                        entry.verts.data(),
                        entry.indices.data() + range.firstIdx,
                        static_cast<int>(range.triCount),
                        g_worldToClip,
                        g_frame.occluderWinding,
                        ::MaskedOcclusionCulling::CLIP_PLANE_ALL,
                        ::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
                }
                submittedTris += range.triCount;
            }
        } else {
            // Fallback path - single submission for the whole Land.
            if (g_asyncThisFrame) {
                ScopedUsAccumulator t(g_stats.rasterizeTimeUs);
                g_threadpool->RenderTriangles(entry.verts.data(), entry.indices.data(),
                                              static_cast<int>(entry.triCount),
                                              g_frame.occluderWinding,
                                              ::MaskedOcclusionCulling::CLIP_PLANE_ALL);
                ++g_stats.asyncJobsQueued;
            } else {
                ScopedUsAccumulator t(g_stats.rasterizeTimeUs);
                g_msoc->RenderTriangles(entry.verts.data(), entry.indices.data(),
                                        static_cast<int>(entry.triCount), g_worldToClip,
                                        g_frame.occluderWinding,
                                        ::MaskedOcclusionCulling::CLIP_PLANE_ALL,
                                        ::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
            }
            submittedTris = entry.triCount;
        }

        if (submittedTris > 0) {
            ++g_stats.aggregateTerrainLands;
            g_stats.aggregateTerrainTris += submittedTris;
            g_stats.maskHasOccluders = true;
        }
    }
}

}  // namespace msoc::occlusion::terrain
