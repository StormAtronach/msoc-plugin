// Terrain occluder aggregation: merges each near Land into one MOC
// submission. Split from OcclusionPass.cpp; shared state via
// OcclusionInternal.h.

#include "OcclusionApi.h"
#include "OcclusionInternal.h"
#include "Log.h"

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
#include <cstdint>
#include <cstring>
#include <limits>  // numeric_limits; was reaching this through the PCH
#include <utility>
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

// Append one terrain TriShape (25v/32t) at full resolution: every source
// vertex and every source triangle, world-transformed, indices offset. Mirrors
// rasterizeTriShape's vertex math without the gates/skinning/thin-axis paths.
static void appendTerrainShapeFull(std::vector<float>& aggVerts,
                                   std::vector<unsigned int>& aggIdx, NI::TriShape* shape) {
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

// Coarse terrain (Half / Corners) is built per Land from the cell's 65x65
// vertex grid, not per 5x5 patch. A cell is 64x64 quads of 128 units; its
// 256 patches share edge vertices as separate copies. Downsampling keeps
// every step-th grid vertex and folds the dropped ones in by taking the
// MINIMUM world-Z over the fine vertices its coarse triangles cover (a
// window of radius step-1 less the far halves of the two quads its
// diagonal cuts it off from):
//
//   - The window is symmetric, so the two patches on either side of a seam
//     compute the same value for the vertex they share. The old per-patch
//     fold used the one-sided window [r, r+step-1], so the patch that owned
//     a seam vertex as its LAST row kept the exact height while its
//     neighbour folded rows 0..step-1 under it, and the surfaces disagreed
//     by up to hundreds of units along every patch boundary. That was the
//     thin dashed crack in the Half/Corners mask.
//   - Radius step-1 is what makes the coarse surface conservative. Every
//     dropped vertex lies within the window of BOTH kept vertices it sits
//     between, so the straight coarse edge between them stays at or below
//     it. The one-sided window only bounded one end: a dip right after a
//     kept row sat under a lerp that never saw it, so the coarse surface
//     could rise above the real terrain there and over-occlude.
//
// Coarse quads are 16/step per subcell edge, so they never straddle a
// subcell; each is emitted into the range of the subcell its fine quads came
// from, keeping the per-subcell frustum cull. A coarse quad is emitted only
// when every fine quad under it came from an accepted patch (no alpha or
// stencil) and all four corners exist, so a missing patch leaves a hole
// rather than a lid. Cell-to-cell seams still clamp the window at the cell
// edge on each side, so a crack can remain along a cell boundary (one line
// per cell edge, versus one per patch edge before); closing that needs the
// neighbour cell's rows and is not done here.
namespace {

constexpr int kFineN = 65;              // vertices per cell edge
constexpr int kFineQuads = 64;          // quads per cell edge
constexpr float kFineSpacing = 128.0f;  // world units per fine quad
constexpr float kLandSize = 8192.0f;

struct CoarsePatch {
    int subcell;           // index into CoarseWorkspace::subNodes
    float x[25], y[25], z[25];
};

struct CoarseWorkspace {
    std::vector<NI::Node*> subNodes;   // every NiNode child of the Land, in child order
    std::vector<CoarsePatch> patches;  // accepted 25-vertex patches, world space
    std::vector<float> z;              // [65*65] min world-Z seen at that grid vertex
    std::vector<uint8_t> have;         // [65*65] 1 once any accepted patch supplied it
    std::vector<int16_t> quadSub;      // [64*64] subcell of the accepted patch owning the fine quad, -1 = none
    std::vector<int> coarseIdx;        // [n*n] index into the entry's verts, -1 = absent
};

}  // namespace

static void buildCoarseLand(LandCacheEntry& entry, NI::Node* landNode, unsigned int step) {
    static CoarseWorkspace ws;
    ws.subNodes.clear();
    ws.patches.clear();

    // Pass 1: collect every accepted patch in world space. A patch is a
    // 25-vertex NiTriShape under a subcell NiNode; alpha/stencil ones are
    // not occluders. The Land can hold any number of subcell nodes (the
    // engine keeps several shapes per patch position), so nothing is capped.
    int shapesSeen = 0, rejected = 0, oddCount = 0;
    const auto& subcells = landNode->children;
    for (size_t j = 0; j < subcells.endIndex; ++j) {
        auto* sub = subcells.storage[j].get();
        if (!sub || !sub->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;
        auto* subNode = static_cast<NI::Node*>(sub);
        const int si = static_cast<int>(ws.subNodes.size());
        ws.subNodes.push_back(subNode);
        const auto& shapes = subNode->children;
        for (size_t k = 0; k < shapes.endIndex; ++k) {
            auto* shape = shapes.storage[k].get();
            if (!shape || !shape->isInstanceOfType(NI::RTTIStaticPtr::NiTriShape)) continue;
            ++shapesSeen;
            const auto p = classify::occluderProperties(shape);
            if (p.alpha || p.stencil) {
                ++rejected;
                continue;
            }
            auto* tri = static_cast<NI::TriShape*>(shape);
            auto data = tri->getModelData();
            if (!data || data->vertex == nullptr || data->getActiveVertexCount() != 25) {
                ++oddCount;
                continue;
            }
            const auto& xf = tri->worldTransform;
            const auto& R = xf.rotation;
            const auto& T = xf.translation;
            const float s = xf.scale;
            ws.patches.emplace_back();
            CoarsePatch& cp = ws.patches.back();
            cp.subcell = si;
            for (int i = 0; i < 25; ++i) {
                const auto& v = data->vertex[i];
                cp.x[i] = (R.m0.x * v.x + R.m0.y * v.y + R.m0.z * v.z) * s + T.x;
                cp.y[i] = (R.m1.x * v.x + R.m1.y * v.y + R.m1.z * v.z) * s + T.y;
                cp.z[i] = (R.m2.x * v.x + R.m2.y * v.y + R.m2.z * v.z) * s + T.z;
            }
        }
    }
    if (ws.patches.empty()) return;

    // Cell origin: the Land's minimum vertex x/y is the cell's south-west
    // corner, a multiple of the cell size; the rounding absorbs float error.
    float minX = std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity();
    for (const CoarsePatch& cp : ws.patches) {
        for (int i = 0; i < 25; ++i) {
            if (cp.x[i] < minX) minX = cp.x[i];
            if (cp.y[i] < minY) minY = cp.y[i];
        }
    }
    const float x0 = std::floor(minX / kLandSize + 0.5f) * kLandSize;
    const float y0 = std::floor(minY / kLandSize + 0.5f) * kLandSize;

    // Pass 2: fill the 65x65 grid and mark each patch's 4x4 fine quads with
    // the subcell that owns them.
    ws.z.assign(kFineN * kFineN, std::numeric_limits<float>::infinity());
    ws.have.assign(kFineN * kFineN, 0);
    ws.quadSub.assign(kFineQuads * kFineQuads, -1);
    int outside = 0, extentFail = 0;
    for (const CoarsePatch& cp : ws.patches) {
        int gMinI = kFineN, gMinJ = kFineN, gMaxI = -1, gMaxJ = -1;
        for (int i = 0; i < 25; ++i) {
            const int gi = static_cast<int>(std::lround((cp.x[i] - x0) / kFineSpacing));
            const int gj = static_cast<int>(std::lround((cp.y[i] - y0) / kFineSpacing));
            if (gi < 0 || gi >= kFineN || gj < 0 || gj >= kFineN) {
                ++outside;
                continue;
            }
            const size_t idx = static_cast<size_t>(gj) * kFineN + gi;
            if (cp.z[i] < ws.z[idx]) ws.z[idx] = cp.z[i];
            ws.have[idx] = 1;
            if (gi < gMinI) gMinI = gi;
            if (gj < gMinJ) gMinJ = gj;
            if (gi > gMaxI) gMaxI = gi;
            if (gj > gMaxJ) gMaxJ = gj;
        }
        if (gMaxI - gMinI == 4 && gMaxJ - gMinJ == 4) {
            for (int qj = gMinJ; qj < gMinJ + 4; ++qj) {
                for (int qi = gMinI; qi < gMinI + 4; ++qi) {
                    ws.quadSub[static_cast<size_t>(qj) * kFineQuads + qi] = static_cast<int16_t>(cp.subcell);
                }
            }
        } else {
            ++extentFail;
        }
    }

    // Coarse vertices: kept grid vertex (r, c) = (cr*step, cc*step), z = the
    // minimum over the fine vertices its coarse triangles pass over. The
    // coarse surface is linear inside each triangle (v00, v01, v10) +
    // (v10, v01, v11), diagonal r + c = step, so a corner only has to bound
    // the fine vertices that get a nonzero barycentric weight from it in
    // one of the (up to six) triangles touching it: the square window of
    // radius step-1 minus the far half of the two quads whose diagonal cuts
    // the corner off. That is the tightest window that keeps the coarse
    // surface under the real terrain everywhere (the difference is
    // piecewise linear on the fine grid, so checking the fine vertices is
    // enough), and it keeps ridges 2-4 points of coverage higher than the
    // full square did. Offsets are enumerated once per build.
    const int istep = static_cast<int>(step);
    const int n = kFineQuads / istep + 1;
    static std::vector<std::pair<int, int>> offsets;
    offsets.clear();
    for (int qr = -istep; qr <= 0; qr += istep) {
        for (int qc = -istep; qc <= 0; qc += istep) {
            // The two triangles of the quad whose origin is (qr, qc); the
            // corner under consideration is (0, 0) in these coordinates.
            const int tri[2][3][2] = {{{qr, qc}, {qr, qc + istep}, {qr + istep, qc}},
                                      {{qr + istep, qc}, {qr, qc + istep}, {qr + istep, qc + istep}}};
            for (const auto& t : tri) {
                int self = -1;
                for (int k = 0; k < 3; ++k) {
                    if (t[k][0] == 0 && t[k][1] == 0) self = k;
                }
                if (self < 0) continue;
                const int r1 = t[(self + 1) % 3][0], c1 = t[(self + 1) % 3][1];
                const int r2 = t[(self + 2) % 3][0], c2 = t[(self + 2) % 3][1];
                const float det = static_cast<float>(r1 * c2 - r2 * c1);
                for (int r = qr; r <= qr + istep; ++r) {
                    for (int c = qc; c <= qc + istep; ++c) {
                        const float w1 = static_cast<float>(r * c2 - r2 * c) / det;
                        const float w2 = static_cast<float>(r1 * c - r * c1) / det;
                        const float w0 = 1.0f - w1 - w2;
                        if (w0 > 1.0e-6f && w1 > -1.0e-6f && w2 > -1.0e-6f) {
                            const std::pair<int, int> o{r, c};
                            if (std::find(offsets.begin(), offsets.end(), o) == offsets.end()) offsets.push_back(o);
                        }
                    }
                }
            }
        }
    }
    ws.coarseIdx.assign(static_cast<size_t>(n) * n, -1);
    for (int cr = 0; cr < n; ++cr) {
        for (int cc = 0; cc < n; ++cc) {
            const int r = cr * istep;
            const int c = cc * istep;
            float zMin = std::numeric_limits<float>::infinity();
            bool any = false;
            for (const auto& o : offsets) {
                const int rr = r + o.first, c2 = c + o.second;
                if (rr < 0 || rr >= kFineN || c2 < 0 || c2 >= kFineN) continue;
                const size_t idx = static_cast<size_t>(rr) * kFineN + c2;
                if (!ws.have[idx]) continue;
                any = true;
                if (ws.z[idx] < zMin) zMin = ws.z[idx];
            }
            if (!any) continue;
            ws.coarseIdx[static_cast<size_t>(cr) * n + cc] = static_cast<int>(entry.verts.size() / 3);
            entry.verts.push_back(x0 + static_cast<float>(c) * kFineSpacing);
            entry.verts.push_back(y0 + static_cast<float>(r) * kFineSpacing);
            entry.verts.push_back(zMin);
        }
    }

    // Coarse quads, grouped by the subcell that owns their fine quads so the
    // per-subcell frustum cull keeps working. (v00, v01, v10) + (v10, v01,
    // v11) with v01 = +x and v10 = +y is CCW seen from above, the winding
    // the source patches use and the CCW-only occluder gate expects.
    int quadsEmitted = 0, quadsMixed = 0, quadsAbsent = 0;
    for (int si = 0; si < static_cast<int>(ws.subNodes.size()); ++si) {
        const unsigned int firstIdx = static_cast<unsigned int>(entry.indices.size());
        for (int cr = 0; cr + 1 < n; ++cr) {
            for (int cc = 0; cc + 1 < n; ++cc) {
                const int r = cr * istep;
                const int c = cc * istep;
                // Only quads whose every fine quad belongs to subcell si.
                bool mine = true;
                for (int qj = r; mine && qj < r + istep; ++qj) {
                    for (int qi = c; qi < c + istep; ++qi) {
                        if (ws.quadSub[static_cast<size_t>(qj) * kFineQuads + qi] != si) {
                            mine = false;
                            break;
                        }
                    }
                }
                if (!mine) continue;
                const int v00 = ws.coarseIdx[static_cast<size_t>(cr) * n + cc];
                const int v01 = ws.coarseIdx[static_cast<size_t>(cr) * n + cc + 1];
                const int v10 = ws.coarseIdx[static_cast<size_t>(cr + 1) * n + cc];
                const int v11 = ws.coarseIdx[static_cast<size_t>(cr + 1) * n + cc + 1];
                if (v00 < 0 || v01 < 0 || v10 < 0 || v11 < 0) {
                    ++quadsAbsent;
                    continue;
                }
                ++quadsEmitted;
                entry.indices.push_back(static_cast<unsigned int>(v00));
                entry.indices.push_back(static_cast<unsigned int>(v01));
                entry.indices.push_back(static_cast<unsigned int>(v10));
                entry.indices.push_back(static_cast<unsigned int>(v10));
                entry.indices.push_back(static_cast<unsigned int>(v01));
                entry.indices.push_back(static_cast<unsigned int>(v11));
            }
        }
        const unsigned int lastIdx = static_cast<unsigned int>(entry.indices.size());
        if (lastIdx > firstIdx) {
            LandCacheEntry::SubcellRange range;
            range.node = ws.subNodes[si];
            range.firstIdx = firstIdx;
            range.triCount = (lastIdx - firstIdx) / 3;
            entry.subcellRanges.push_back(range);
        }
    }
    // Quads no subcell claimed whole (a fine quad under them has no accepted
    // patch, or they straddle two subcells' patches).
    for (int cr = 0; cr + 1 < n; ++cr) {
        for (int cc = 0; cc + 1 < n; ++cc) {
            const int r = cr * istep, c = cc * istep;
            const int first = ws.quadSub[static_cast<size_t>(r) * kFineQuads + c];
            bool uniform = first >= 0;
            for (int qj = r; uniform && qj < r + istep; ++qj) {
                for (int qi = c; qi < c + istep; ++qi) {
                    if (ws.quadSub[static_cast<size_t>(qj) * kFineQuads + qi] != first) {
                        uniform = false;
                        break;
                    }
                }
            }
            if (!uniform) ++quadsMixed;
        }
    }

    // One line per cache build (cell change or resolution change), so a
    // missing region can be traced to its cause.
    log::getLog() << "MSOC: coarse land build step=" << step << " window=" << offsets.size() << " subcellNodes=" << ws.subNodes.size()
                  << " shapes=" << shapesSeen << " patches=" << ws.patches.size() << " rejected=" << rejected
                  << " odd=" << oddCount << " verticesOutsideGrid=" << outside << " extentFail=" << extentFail
                  << " quads emitted=" << quadsEmitted << " absentCorner=" << quadsAbsent << " unclaimed=" << quadsMixed
                  << " origin=(" << x0 << "," << y0 << ") ranges=" << entry.subcellRanges.size() << std::endl;
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

    if (step == 1) {
        // Full: copy the patches as they are, one index range per subcell.
        const auto& subcells = landNode->children;
        for (size_t j = 0; j < subcells.endIndex; ++j) {
            auto* sub = subcells.storage[j].get();
            if (!sub) continue;
            if (!sub->isInstanceOfType(NI::RTTIStaticPtr::NiNode)) continue;
            auto* subNode = static_cast<NI::Node*>(sub);

            const unsigned int firstIdxBefore = static_cast<unsigned int>(entry.indices.size());
            const auto& shapes = subNode->children;
            for (size_t k = 0; k < shapes.endIndex; ++k) {
                auto* shape = shapes.storage[k].get();
                if (!shape) continue;
                if (!shape->isInstanceOfType(NI::RTTIStaticPtr::NiTriShape)) continue;
                const auto p = classify::occluderProperties(shape);
                if (p.alpha || p.stencil) continue;
                appendTerrainShapeFull(entry.verts, entry.indices, static_cast<NI::TriShape*>(shape));
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
    } else {
        buildCoarseLand(entry, landNode, step);
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

// Aggregate terrain. Walks WorldLandscape (root -> Land -> 16 subcells ->
// N NiTriShapes) and queues one occluder per visible subcell of each
// visible Land; the pass submits the queue near-to-far with the meshes. Individual 25v/32t patches fail the thin-axis gate;
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
        // net win on per-tile cost and async queue pressure. Each visible
        // subcell is queued with the squared eye distance of its bound
        // centre; the pass submits the queue near-to-far with the meshes.
        //
        // Empty subcellRanges fallback (stale cache from before the
        // field landed) queues the whole entry as one range.
        const auto& eye = camera->worldTransform.translation;
        const auto dist2To = [&](const NI::AVObject* obj) {
            const auto& o = obj->worldBoundOrigin;
            const float dx = o.x - eye.x, dy = o.y - eye.y, dz = o.z - eye.z;
            return dx * dx + dy * dy + dz * dz;
        };
        unsigned int submittedTris = 0;
        if (!entry.subcellRanges.empty()) {
            for (const auto& range : entry.subcellRanges) {
                if (range.triCount == 0) continue;
                if (range.node && frustumCulledSphere(range.node, camera)) continue;
                enqueueOccluder(entry.verts.data(), entry.indices.data() + range.firstIdx, range.triCount,
                                dist2To(range.node ? static_cast<NI::AVObject*>(range.node) : land), true);
                submittedTris += range.triCount;
            }
        } else {
            // Fallback path - single submission for the whole Land.
            enqueueOccluder(entry.verts.data(), entry.indices.data(), entry.triCount, dist2To(land), true);
            submittedTris = entry.triCount;
        }

        // Triangles and maskHasOccluders are counted at submit time, in the
        // queue flush, so the stats match what actually rasterised.
        if (submittedTris > 0) {
            ++g_stats.aggregateTerrainLands;
        }
    }
}

}  // namespace msoc::occlusion::terrain
