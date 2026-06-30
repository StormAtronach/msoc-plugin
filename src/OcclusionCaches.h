#pragma once

// Per-cell cache types + the single owner object, shared between the core TU
// and the subsystem TUs that read/write the caches. The g_caches instance is
// defined in OcclusionPass.cpp.

#include "MaskedOcclusionCulling.h"

#include "NIPointer.h"
#include "NINode.h"
#include "NIAVObject.h"
#include "NILight.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace msoc::patch::occlusion {

// Per-Land merged occluder cache. Key = per-Land NiNode under
// WorldLandscape; value = world-space vertex+index buffers prebuilt
// on first sight. Mark-and-sweep evicted per frame.
//
// nodePtr (NI::Pointer) refcounts the NiNode so the engine can't
// free a key we still hold. Sweep eviction releases the ref.
struct LandCacheEntry {
    NI::Pointer<NI::Node> nodePtr;
    std::vector<float> verts;
    std::vector<unsigned int> indices;
    unsigned int triCount = 0;
    bool seen = false;
    // Mismatch with current OcclusionTerrainResolution forces a
    // rebuild on hit, so MCM changes take effect lazily without
    // a global cache flush. 0xff = uninitialised.
    uint8_t builtForResolution = 0xff;

    // Per-subcell ranges for per-subcell frustum-cull. `node` is the
    // subcell NiNode used per frame as the frustumCulledSphere argument.
    // (firstIdx, triCount) index into `indices` for the Raster submit
    // path; (firstVert, vertCount) index into `verts` for the Horizon
    // path, which projects the unique world-space verts directly rather
    // than walking the index list.
    struct SubcellRange {
        NI::Node* node;
        unsigned int firstIdx;
        unsigned int triCount;
        unsigned int firstVert;
        unsigned int vertCount;
    };
    std::vector<SubcellRange> subcellRanges;
};

// Temporal coherence cache for drain-phase TestRect verdicts. Keyed
// by NI::AVObject*. OcclusionTemporalCoherenceFrames N: 0 disables;
// N>0 reuses a fresh entry for up to N frames.
//
// Caches only OCCLUDED verdicts - VISIBLE and VIEW_CULLED fall
// through to display() anyway, so caching them adds stale-pointer
// surface for no hit-path win.
//
// Move detection compares worldBound origin + radius against the
// snapshot taken at query time.
//
// shapePtr (NI::Pointer) refcounts the AVObject key. Without it,
// raising N widens the window where a destroyed-but-not-yet-pruned
// entry could collide with a recycled pointer. Age-prune at 2*N
// frames bounds memory.
struct DrainCacheEntry {
    NI::Pointer<NI::AVObject> shapePtr;
    ::MaskedOcclusionCulling::CullingResult result;
    uint32_t lastQueryFrame;
    float boundOriginX, boundOriginY, boundOriginZ;
    float boundRadius;
};

// Terrain-descendant membership cache. Replaces a 7-deep parent-
// chain walk (called twice per deferred leaf) with a flat lookup.
// On a peak Vivec frame: 2354 calls x 7.3 avg = 17k pointer chases
// per frame, dominated by L2 misses on cold scene-graph nodes.
//
// Wiped on cell change. Within a cell, scene-graph parentage and
// g_worldLandscapeRoot are stable. Mid-cell reparenting is rare;
// the failure mode is benign (wrong "is terrain" -> harmless under-
// occlusion for one item).
struct TerrainMembershipEntry {
    NI::Pointer<NI::AVObject> objPtr;
    bool isDescendant;
};

// Per-instance occluder eligibility cache. Combines (a) ancestor-walked
// alpha/stencil classification and (b) world-space vertex/index buffers
// + AABB, keyed on NI::AVObject*. For static cell meshes both are
// invariant between frames. Profile data on user's machine: vertex-loop
// peak ~1.8 ms/frame, classify peak ~260 us/frame - both well above the
// 50 us/frame caching threshold from MSOC_CACHE_AUDIT.md.
//
// Invalidation:
//   - Cell change wipes the whole map (same hook as g_caches.drain).
//   - Geometry slot refreshes whenever the worldTransform memcmp differs
//     (catches moved pickables, scripted moves; ~no-op for cell statics).
//
// Profile probes kept in place so the cache's own hit rate and any
// residual miss-path cost stay visible in MSOC.log.
struct OccluderCacheEntry {
    NI::Pointer<NI::AVObject> objPtr;

    bool propsResolved = false;
    bool alpha = false;
    bool stencil = false;

    bool geomResolved = false;
    std::vector<float> worldVerts;
    std::vector<unsigned int> indices;
    unsigned int outTriCount = 0;
    unsigned short vertexCount = 0;
    float minX = 0, minY = 0, minZ = 0;
    float maxX = 0, maxY = 0, maxZ = 0;

    // 13 floats = sizeof(TES3::Transform). Storing the bytes verbatim
    // (rotation + translation + scale) lets a single memcmp catch any
    // kind of motion without writing a per-field comparator.
    float xfData[13] = {};
};

// Per-light occlusion cache for updateLights (gated by
// OcclusionCullLights). Tracks last query frame, last verdict,
// consecutive-occluded count for hysteresis, and worldBound snapshot
// for move detection.
//
// lightPtr (NI::Pointer) refcounts the NiLight so a freed-then-
// reallocated address can't collide with a stale entry mid-cell.
struct LightCullEntry {
    NI::Pointer<NI::Light> lightPtr;
    uint32_t lastQueryFrame;
    uint8_t consecOccluded;
    bool cullActive;
    float boundOriginX, boundOriginY, boundOriginZ;
    float boundRadius;
};

// Object-space AABB of a geometry's verts, for the optional occludee box
// test. Keyed by the NI::GeometryData pointer (shared across instances via
// NIF dedup), so the vert min/max is computed once per unique mesh and reused
// for every instance. The pointer is held as an opaque key; the cache is
// wiped on cell change like the others, so a freed-then-recycled pointer
// can't survive into the next cell.
struct OccludeeBoxEntry {
    float minX, minY, minZ;
    float maxX, maxY, maxZ;
};

// ------------------------------------------------------------
// Owner for the per-cell caches + their hit/miss counters.
// Single g_caches instance replaces the loose g_*Cache* statics.
// ------------------------------------------------------------
struct OcclusionCaches {
    std::unordered_map<NI::Node*, LandCacheEntry> land;
    uint64_t landHits = 0, landMisses = 0, landEvictions = 0;

    std::unordered_map<NI::AVObject*, DrainCacheEntry> drain;
    uint64_t drainHits = 0, drainMisses = 0;

    std::unordered_map<NI::AVObject*, TerrainMembershipEntry> terrainMembership;
    uint64_t terrainMembershipHits = 0, terrainMembershipMisses = 0;

    std::unordered_map<NI::AVObject*, OccluderCacheEntry> occluder;
    uint64_t occluderHits = 0, occluderMisses = 0;

    std::unordered_map<NI::Light*, LightCullEntry> lightCull;
    uint64_t lightsTested = 0, lightsOccluded = 0, lightCullHits = 0, lightCullMisses = 0;

    // Object-space vertex AABB per geometry (optional occludee box test).
    std::unordered_map<const void*, OccludeeBoxEntry> occludeeBox;
    uint64_t occludeeBoxHits = 0, occludeeBoxMisses = 0;

    // Get-or-create the occluder entry for obj (refcounts the key).
    OccluderCacheEntry& occluderEntry(NI::AVObject* obj) {
        auto it = occluder.find(obj);
        if (it != occluder.end()) {
            return it->second;
        }
        auto& e = occluder[obj];
        e.objPtr = NI::Pointer<NI::AVObject>(obj);
        return e;
    }

    // Cell-change wipe: release every cached NI::Pointer pin. Clear
    // order preserved verbatim from the original inline wipe.
    void wipeForCellChange() {
        land.clear();
        drain.clear();
        lightCull.clear();
        terrainMembership.clear();
        occluder.clear();
        occludeeBox.clear();
    }
};

extern OcclusionCaches g_caches;

}  // namespace msoc::patch::occlusion
