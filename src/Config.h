#pragma once

// Plugin config statics. Populated from Lua via msoc.configure(table) -
// see Config.cpp. Only the slice the occlusion patch reads is exposed.

#include "HardwareTier.h"  // HardwareTier enum + classifyHardwareTier / hardwareTierName

struct lua_State;

namespace msoc {
class Configuration {
public:
    static bool EnableMSOC;

    static bool DebugOcclusionTintOccluded;
    static bool DebugOcclusionTintTested;
    static bool DebugOcclusionTintOccluder;
    // Mirror the finished mask into an engine texture each frame so the Lua
    // side can show it as a HUD image element. Off costs nothing: the
    // readback only runs once Lua has asked for the texture. See MaskOverlay.h.
    static bool DebugMaskOverlay;

    // Interior favours smaller occluders (pillars, crates); exterior
    // raises the bar to skip clutter. Resolved into g_*Effective per
    // frame once scene type is known.
    static float OcclusionOccluderRadiusMinInterior;
    static float OcclusionOccluderRadiusMinExterior;
    static float OcclusionOccluderRadiusMaxInterior;
    static float OcclusionOccluderRadiusMaxExterior;
    static float OcclusionOccluderMinDimensionInterior;
    static float OcclusionOccluderMinDimensionExterior;
    static float OcclusionInsideOccluderMarginInterior;
    static float OcclusionInsideOccluderMarginExterior;
    // Gate for the inside-AABB occluder rejection. When false, the
    // rejection is skipped - meshes whose tight AABB+margin contains
    // the eye are still rasterised. Empirically the guard was over-
    // eager (rejected close-up walls; didn't protect against the
    // concave-shell failure mode it was written for). Default off.
    static bool OcclusionInsideOccluderGuard;

    static float OcclusionDepthSlackWorldUnits;
    static unsigned int OcclusionOccluderMaxTriangles;
    static unsigned int OcclusionOccludeeMinRadius;
    // Optional tighter occludee test: after a VISIBLE sphere result, project
    // the occludee's object-space vertex AABB (8 corners) and re-test. Catches
    // non-spherical leaves the loose bounding sphere keeps visible. Default off.
    static bool OcclusionOccludeeBoxTest;

    static bool OcclusionEnableInterior;
    static bool OcclusionEnableExterior;
    static bool OcclusionSkipTerrainOccludees;
    // 0=Off, 1=Raster. Lua side accepts bool too (true->1, false->0), and
    // the retired value 2 (the Horizon curtain, removed in 1.6.0) reads as
    // Raster; see Config.cpp's parser.
    static int OcclusionAggregateTerrain;
    static unsigned int OcclusionTerrainResolution;

    // When true (default), occluders are submitted with backface culling that
    // keeps only counter-clockwise (front) faces; clockwise faces are culled
    // and not rasterized. When false, both faces are rasterized
    // (BACKFACE_NONE) - correct regardless of winding.
    //
    // No vertex data is touched - winding is decided by MOC from screen-
    // space area; we never reorder vertices. The assumption is that ~99%
    // of NIFs are CCW-wound, so this halves occluder raster work at the
    // cost of dropping the rare CW-wound mesh from the depth buffer. That
    // loss is a safe under-occlude (the object behind it just isn't
    // culled), never a wrong-cull. Global: applies to per-instance
    // occluders and aggregate terrain. See FrameConfig::occluderWinding for the
    // (deliberately counter-intuitive) MOC enum mapping.
    static bool OcclusionOccluderCCWOnly;

    // Submit per-instance occluders sorted near-to-far instead of in scene-graph
    // order. Lets MOC early-reject occluded-occluder triangles against the
    // accumulating HiZ, cutting rasterization on depth-overlapping scenes. Cost:
    // occluders can only be submitted after the full traversal (they must all be
    // collected to sort), forfeiting the async traverse/rasterize overlap - so
    // it's a clear win in sync mode and a measure-it in async (default on;
    // A/B via rasterizeUs + asyncFlushUs in a dense scene).
    static bool OcclusionOccluderFrontToBack;

    static bool OcclusionAsyncOccluders;
    static unsigned int OcclusionThreadpoolThreadCount;
    static unsigned int OcclusionThreadpoolBinsW;
    static unsigned int OcclusionThreadpoolBinsH;
    static unsigned int OcclusionTemporalCoherenceFrames;

    // Applied at launch: latched in installPatches(), which main.lua calls
    // after pushing msoc.json, so a saved value takes effect. Changing it
    // afterwards needs a restart. MOC requires width % 8 == 0 and
    // height % 4 == 0; installPatches rounds and clamps.
    static unsigned int OcclusionMaskWidth;
    static unsigned int OcclusionMaskHeight;

    // Per-phase microsecond budgets. 0 = unlimited. Predictive skip
    // bails the whole phase when EMA(prev) > 2x budget; spike clip
    // bails inside the phase when running elapsed exceeds it.
    // Untested testees fall back to Visible (over-render, never wrong-
    // cull); unsubmitted occluders just miss the mask.
    static unsigned int OcclusionRasterizeBudgetUs;
    static unsigned int OcclusionClassifyBudgetUs;

    static bool OcclusionLogPerFrame;
    static bool OcclusionLogAggregate;
    // Cell-cross profiling: emit the full per-frame stats line on the
    // cell-change frame and the next several re-population frames, so the
    // cross spike's phase breakdown is visible (the 300-frame sample
    // almost never lands on a cross). Lines carry cellCross=<age>.
    static bool OcclusionLogCellCross;

    // Freeze diagnostic: spawns a detached thread that dumps
    // MSOC.forensics.txt every 250ms. Read once in installPatches(), which
    // runs after Lua's first configure(), so a saved value is honoured;
    // changing it afterwards needs a restart. Default off.
    static bool OcclusionForensicsWatchdog;
};

// msoc.configure(table) - reads each field at stack index 1 and
// writes through to the matching static. Unknown keys ignored;
// missing keys leave the static untouched.
int configure(lua_State* L);

}  // namespace msoc
