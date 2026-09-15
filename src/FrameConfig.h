#pragma once

// Per-top-level-frame snapshot of the Configuration:: knobs the hot path
// reads, resolved once at the start of an active frame so the inner loops
// stay branch-free on scene type and immune to a mid-frame configure().
// Replaces the loose g_*Effective file-statics that OcclusionPass.cpp
// used to hand-copy in the detour.
//
// Not the phase budgets: those live with the profiling/FrameStats machinery
// (they are read in a separate per-frame block and feed the skip decisions).

#include "Config.h"

#include "MaskedOcclusionCulling.h"  // BackfaceWinding

namespace msoc::occlusion {

struct FrameConfig {
    // Exterior defaults, taken from Configuration:: rather than written out
    // again here. Configuration's statics are constant-initialised
    // fundamentals, so they hold their values before any dynamic initialiser
    // runs and this is safe at static-init time. One construction per process;
    // the hot path is untouched.
    FrameConfig() { snapshot(false); }

    // Occluder eligibility, resolved per scene type by snapshot().
    float occluderRadiusMin = 0.0f;
    float occluderRadiusMax = 0.0f;
    float occluderMinDimension = 0.0f;
    float insideOccluderMargin = 0.0f;
    bool insideOccluderGuard = false;

    // Direct copies (scene-type independent).
    float depthSlackWorldUnits = 128.0f;
    unsigned int occluderMaxTriangles = 4096;
    float occludeeMinRadius = 1.0f;
    bool skipTerrainOccludees = true;
    bool skipNoZTestOccludees = true;
    bool occludeeBoxTest = false;
    int aggregateTerrain = 1;
    unsigned int terrainResolution = 1;
    // Backface mode for every occluder submission, resolved from the
    // OcclusionOccluderCCWOnly bool. BACKFACE_NONE rasterizes both faces.
    ::MaskedOcclusionCulling::BackfaceWinding occluderWinding =
        ::MaskedOcclusionCulling::BACKFACE_NONE;
    // Sort occluders near-to-far and submit after traversal (see Config.h).
    // The value here is only the pre-snapshot state; config.lua ships it on.
    bool occluderFrontToBack = false;
    unsigned int temporalCoherenceFrames = 4;
    bool tintOccluder = false;
    bool tintOccluded = false;
    bool tintTested = false;
    bool maskOverlay = false;
    bool logEnabled = false;

    // Resolve every field from Configuration:: for this frame. isInterior
    // selects the interior/exterior occluder thresholds.
    void snapshot(bool isInterior) {
        using C = Configuration;
        if (isInterior) {
            occluderRadiusMin = C::OcclusionOccluderRadiusMinInterior;
            occluderRadiusMax = C::OcclusionOccluderRadiusMaxInterior;
            occluderMinDimension = C::OcclusionOccluderMinDimensionInterior;
            insideOccluderMargin = C::OcclusionInsideOccluderMarginInterior;
        } else {
            occluderRadiusMin = C::OcclusionOccluderRadiusMinExterior;
            occluderRadiusMax = C::OcclusionOccluderRadiusMaxExterior;
            occluderMinDimension = C::OcclusionOccluderMinDimensionExterior;
            insideOccluderMargin = C::OcclusionInsideOccluderMarginExterior;
        }
        depthSlackWorldUnits = C::OcclusionDepthSlackWorldUnits;
        occluderMaxTriangles = C::OcclusionOccluderMaxTriangles;
        occludeeMinRadius = static_cast<float>(C::OcclusionOccludeeMinRadius);
        skipTerrainOccludees = C::OcclusionSkipTerrainOccludees;
        skipNoZTestOccludees = C::OcclusionSkipNoZTestOccludees;
        occludeeBoxTest = C::OcclusionOccludeeBoxTest;
        aggregateTerrain = C::OcclusionAggregateTerrain;
        terrainResolution = C::OcclusionTerrainResolution;
        // CCW-only intent maps to BACKFACE_CW: MOC's bfWinding names the
        // winding to CULL, so culling CW keeps the CCW front faces.
        occluderWinding = C::OcclusionOccluderCCWOnly
                              ? ::MaskedOcclusionCulling::BACKFACE_CW
                              : ::MaskedOcclusionCulling::BACKFACE_NONE;
        occluderFrontToBack = C::OcclusionOccluderFrontToBack;
        temporalCoherenceFrames = C::OcclusionTemporalCoherenceFrames;
        tintOccluder = C::DebugOcclusionTintOccluder;
        tintOccluded = C::DebugOcclusionTintOccluded;
        tintTested = C::DebugOcclusionTintTested;
        maskOverlay = C::DebugMaskOverlay;
        insideOccluderGuard = C::OcclusionInsideOccluderGuard;
        logEnabled = C::OcclusionLogPerFrame || C::OcclusionLogAggregate || C::OcclusionLogCellCross;
    }
};

}  // namespace msoc::occlusion
