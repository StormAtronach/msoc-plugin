// Terrain occluder aggregation: merges each near Land into one MOC
// submission (Raster mode) or a 1D horizon curtain (Horizon mode). Split
// from OcclusionPass.cpp; shared state via OcclusionInternal.h.

#include "OcclusionApi.h"
#include "OcclusionInternal.h"
#include "HorizonOccluder.h"

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
#include <vector>

namespace msoc::patch::occlusion {

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

        // (n-1)*(n-1)*2 triangles. BACKFACE_NONE so winding is moot.
        const unsigned int qN = n - 1;
        aggIdx.reserve(aggIdx.size() + static_cast<size_t>(qN * qN) * 6);
        for (unsigned int qr = 0; qr < qN; ++qr) {
            for (unsigned int qc = 0; qc < qN; ++qc) {
                const unsigned int v00 = baseVert + (qr * n + qc);
                const unsigned int v01 = baseVert + (qr * n + (qc + 1));
                const unsigned int v10 = baseVert + ((qr + 1) * n + qc);
                const unsigned int v11 = baseVert + ((qr + 1) * n + (qc + 1));
                aggIdx.push_back(v00);
                aggIdx.push_back(v10);
                aggIdx.push_back(v01);
                aggIdx.push_back(v10);
                aggIdx.push_back(v11);
                aggIdx.push_back(v01);
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

        // Bracket this subcell's index- AND vertex-buffer ranges so the
        // submit path (indices) and the horizon-project path (verts) can
        // both frustum-cull at subcell granularity.
        const unsigned int firstIdxBefore = static_cast<unsigned int>(entry.indices.size());
        const unsigned int firstVertBefore = static_cast<unsigned int>(entry.verts.size() / 3);

        const auto& shapes = subNode->children;
        for (size_t k = 0; k < shapes.endIndex; ++k) {
            auto* shape = shapes.storage[k].get();
            if (!shape) continue;
            if (!shape->isInstanceOfType(NI::RTTIStaticPtr::NiTriShape)) continue;
            const auto p = classifyOccluderProperties(shape);
            if (p.alpha || p.stencil) continue;
            appendTerrainShape(entry.verts, entry.indices, static_cast<NI::TriShape*>(shape), step);
        }

        const unsigned int firstIdxAfter = static_cast<unsigned int>(entry.indices.size());
        if (firstIdxAfter > firstIdxBefore) {
            const unsigned int firstVertAfter = static_cast<unsigned int>(entry.verts.size() / 3);
            LandCacheEntry::SubcellRange r;
            r.node = subNode;
            r.firstIdx = firstIdxBefore;
            r.triCount = (firstIdxAfter - firstIdxBefore) / 3;
            r.firstVert = firstVertBefore;
            r.vertCount = firstVertAfter - firstVertBefore;
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
// Shared by both terrain modes so neither re-walks the scene graph for
// geometry it has already transformed: Raster submits the cached verts;
// Horizon projects them into the 1D silhouette. Precondition:
// g_worldLandscapeRoot is non-null (both callers guard).
static void refreshLandCache() {
    for (auto& kv : g_caches.land) kv.second.seen = false;

    const auto& landChildren = g_worldLandscapeRoot->children;
    for (size_t i = 0; i < landChildren.endIndex; ++i) {
        auto* land = landChildren.storage[i].get();
        if (!land) continue;
        if (!land->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;

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

// Horizon-mode terrain occluder. Builds the 1D max-y/max-w horizon by
// projecting the shared per-Land world-space cache (the same buffers the
// Raster path submits), then simplifies to ~60 adaptive samples, emits
// ~120 curtain triangles, and submits those to MOC via sync
// RenderTriangles. epsD is adaptive (do NOT pass 1e30 like MGE-XE's
// distant path - see HorizonOccluder.h).
//
// Reusing the cache means the per-vertex world transform and the per-
// shape getModelData/RTTI/alpha-stencil classification are paid once per
// cell (on cache miss) instead of every frame; the per-frame cost here is
// just projectWorld + divide + bin. The cache is already downsampled per
// the terrain-resolution dropdown (Full/Half/Corners), so coarser
// settings cut the projection count too - the min-z fold in the cache
// build keeps the silhouette conservative (it can only sink, never rise).
void rasterizeAggregateTerrainHorizon(NI::Camera* camera) {
    if (!g_worldLandscapeRoot) return;
    if (g_worldLandscapeRoot->getAppCulled()) return;

    // Outer bucket: build = cache refresh + project + simplify + emit.
    // The inner RenderTriangles also accumulates into g_stats.horizonRasterUs
    // and g_stats.rasterizeTimeUs.
    ScopedUsAccumulator timer(g_stats.horizonBuildUs);

    // Build/refresh the shared per-Land cache (mark-and-sweep). The
    // alpha/stencil gate is applied at cache-build time, so the cached
    // verts already exclude transparent terrain patches.
    refreshLandCache();

    auto& horizon = msoc::horizon::HorizonOccluder::getInstance();
    // 512 cols / 60 samples matches MGE-XE's distant-land path.
    horizon.init(512, 60);
    horizon.reset();

    const int resolution = horizon.resolution();
    const float halfSpan = 0.5f * static_cast<float>(resolution - 1);

    uint64_t landsContributed = 0;
    uint64_t vertsProjected = 0;

    const auto& landChildren = g_worldLandscapeRoot->children;
    for (size_t i = 0; i < landChildren.endIndex; ++i) {
        auto* land = landChildren.storage[i].get();
        if (!land) continue;
        if (!land->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;
        if (land->getAppCulled()) continue;
        if (frustumCulledSphere(land, camera)) continue;

        auto* landNode = static_cast<NI::Node*>(land);
        auto it = g_caches.land.find(landNode);
        if (it == g_caches.land.end()) continue;  // refresh guarantees presence
        const LandCacheEntry& entry = it->second;
        if (entry.verts.empty()) continue;

        const float* verts = entry.verts.data();
        bool landFedAny = false;

        // Project one cached world-space vertex range into the horizon.
        // The cache verts are already world-space, so this is just the
        // clip projection + divide + column bin.
        const auto projectRange = [&](unsigned int firstVert, unsigned int vertCount) {
            const unsigned int end = firstVert + vertCount;
            for (unsigned int vi = firstVert; vi < end; ++vi) {
                const float wx = verts[vi * 3 + 0];
                const float wy = verts[vi * 3 + 1];
                const float wz = verts[vi * 3 + 2];

                const ClipXYW c = projectWorld(wx, wy, wz);
                // Near-plane straddling - projection unstable.
                if (c.w <= kNearClipW) continue;

                const float invW = 1.0f / c.w;
                const float ndcX = c.x * invW;
                const float ndcY = c.y * invW;

                // Off-screen X clamps would contaminate column-
                // boundary sentinels; drop. Below-screen doesn't
                // occlude anything; drop. Above-screen clamps up
                // (terrain past the top still defines silhouette).
                if (ndcX < -1.0f || ndcX > 1.0f) continue;
                if (ndcY < -1.0f) continue;
                const float clampedY = (ndcY > 1.0f) ? 1.0f : ndcY;

                int col = static_cast<int>((ndcX + 1.0f) * halfSpan);
                if (col < 0) col = 0;
                if (col >= resolution) col = resolution - 1;

                horizon.update(col, clampedY, c.w);
                ++vertsProjected;
                landFedAny = true;
            }
        };

        // Per-subcell frustum cull. A Land covers 8192x8192; typically
        // ~5-8 of its 16 subcells are in-frustum. Skipping invisible
        // ones avoids the projection. Quality is unchanged (the c.w /
        // ndcX guards above would've rejected these post-projection).
        if (!entry.subcellRanges.empty()) {
            for (const auto& range : entry.subcellRanges) {
                if (range.vertCount == 0) continue;
                if (range.node && frustumCulledSphere(range.node, camera)) continue;
                projectRange(range.firstVert, range.vertCount);
            }
        } else {
            // Stale entry built before vertex ranges existed: project
            // the whole Land at land-frustum granularity.
            projectRange(0, static_cast<unsigned int>(entry.verts.size() / 3));
        }

        if (landFedAny) ++landsContributed;
    }

    if (vertsProjected == 0) return;

    // Simplify -> margin -> emit -> fixup -> submit.
    constexpr int kMaxSamples = 60;
    constexpr float kEpsH = 0.01f;
    constexpr int kTileAlign = 16;
    constexpr float kNdcYBottom = -1.1f;
    constexpr float kYSafetyMargin = 0.04f;
    constexpr float kEpsDFraction = 0.05f;
    constexpr float kEpsDFloor = 100.0f;

    // D12: adaptive epsD computed from this frame's depth range.
    // computeAdaptiveEpsD returns +infinity if fewer than 2 cols are
    // active - equivalent to disabling the depth term, safe fallback.
    const float adaptiveEpsD = horizon.computeAdaptiveEpsD(kEpsDFraction, kEpsDFloor);

    static msoc::horizon::Sample samples[kMaxSamples];
    const int nSamples = horizon.simplify(samples, kMaxSamples, kEpsH, adaptiveEpsD, kTileAlign);
    if (nSamples < 2) return;

    // Pull the curtain top down by the safety margin. Skip sentinel
    // rows (those columns will be skipped by emit anyway).
    for (int i = 0; i < nSamples; ++i) {
        if (samples[i].h > -1.0e29f) samples[i].h -= kYSafetyMargin;
    }

    // Per-frame scratch - function-static so allocations amortise.
    // Sized for the worst case (every segment emits 6 verts).
    static std::vector<msoc::horizon::CurtainVertex> curtainVerts;
    curtainVerts.resize(static_cast<size_t>(6 * (nSamples - 1)));
    const int triCount = horizon.emitCurtainNDC(samples, nSamples, kNdcYBottom,
                                                curtainVerts.data(), static_cast<int>(curtainVerts.size()));
    if (triCount <= 0) return;

    // Convert NDC layout (x, y, z=depth, w=1) -> MOC pre-transformed
    // layout (x*d, y*d, _, d) in place. Submit with VertexLayout
    // matching that layout: stride=16, offY=4, offW=12.
    msoc::horizon::HorizonOccluder::fixupForMOC(curtainVerts.data(), triCount * 3);

    static std::vector<unsigned int> curtainIdx;
    const int vtxCount = triCount * 3;
    curtainIdx.resize(static_cast<size_t>(vtxCount));
    for (int i = 0; i < vtxCount; ++i) {
        curtainIdx[i] = static_cast<unsigned int>(i);
    }

    // Sync submit; bypasses the threadpool - ~120 tris is below the
    // dispatch-overhead break-even. Times into both buckets so total
    // rasterize cost stays comparable across modes AND the curtain's
    // own raster cost stays measurable.
    {
        ScopedUsAccumulator t1(g_stats.rasterizeTimeUs);
        ScopedUsAccumulator t2(g_stats.horizonRasterUs);
        g_msoc->RenderTriangles(
            reinterpret_cast<const float*>(curtainVerts.data()),
            curtainIdx.data(), triCount,
            /*modelToClip=*/nullptr,
            ::MaskedOcclusionCulling::BACKFACE_NONE,
            ::MaskedOcclusionCulling::CLIP_PLANE_ALL,
            ::MaskedOcclusionCulling::VertexLayout(16, 4, 12));
    }

    // columnsTouched is a linear scan but runs once per frame.
    g_stats.horizonCurtainTris = static_cast<uint64_t>(triCount);
    g_stats.horizonLandsFed = landsContributed;
    g_stats.horizonVertsFed = vertsProjected;
    g_stats.horizonColumnsTouched = static_cast<uint64_t>(horizon.columnsTouched());
    g_stats.horizonAdaptiveEpsD = adaptiveEpsD;
}

// Aggregate terrain rasteriser. Walks WorldLandscape (root -> Land ->
// 16 subcells -> N NiTriShapes) and submits one combined occluder per
// visible Land. Individual 25v/32t patches fail the thin-axis gate;
// merging gives the hill/horizon silhouette that actually occludes
// distant architecture.
//
// Must run inside isTopLevel - after ClearBuffer + uploadCameraTransform,
// before cullShowBody - so the drain sees terrain in the depth buffer.
//
// Uses g_caches.land to amortise the per-Land walk + vertex transform.
// Mark-and-sweep evicts entries whose NiNode wasn't seen this frame.
void rasterizeAggregateTerrain(NI::Camera* camera) {
    if (!g_worldLandscapeRoot) return;
    if (g_worldLandscapeRoot->getAppCulled()) return;

    ScopedUsAccumulator timer(g_stats.aggregateTerrainUs);

    // Build/refresh the shared per-Land cache (mark-and-sweep). After
    // this every live Land has an up-to-date entry in g_caches.land.
    refreshLandCache();

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
                        ::MaskedOcclusionCulling::BACKFACE_NONE,
                        ::MaskedOcclusionCulling::CLIP_PLANE_ALL);
                } else {
                    ScopedUsAccumulator t(g_stats.rasterizeTimeUs);
                    g_msoc->RenderTriangles(
                        entry.verts.data(),
                        entry.indices.data() + range.firstIdx,
                        static_cast<int>(range.triCount),
                        g_worldToClip,
                        ::MaskedOcclusionCulling::BACKFACE_NONE,
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
                                              ::MaskedOcclusionCulling::BACKFACE_NONE,
                                              ::MaskedOcclusionCulling::CLIP_PLANE_ALL);
            } else {
                ScopedUsAccumulator t(g_stats.rasterizeTimeUs);
                g_msoc->RenderTriangles(entry.verts.data(), entry.indices.data(),
                                        static_cast<int>(entry.triCount), g_worldToClip,
                                        ::MaskedOcclusionCulling::BACKFACE_NONE,
                                        ::MaskedOcclusionCulling::CLIP_PLANE_ALL,
                                        ::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
            }
            submittedTris = entry.triCount;
        }

        if (submittedTris > 0) {
            ++g_stats.aggregateTerrainLands;
            g_stats.aggregateTerrainTris += submittedTris;
        }
    }
}

}  // namespace msoc::patch::occlusion
