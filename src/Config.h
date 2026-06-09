#pragma once

// Plugin config statics. Populated from Lua via msoc.configure(table) —
// see Config.cpp. Only the slice the occlusion patch reads is exposed.

struct lua_State;

namespace msoc {
    class Configuration {
    public:
        static bool EnableMSOC;

        static bool DebugOcclusionTintOccluded;
        static bool DebugOcclusionTintTested;
        static bool DebugOcclusionTintOccluder;

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
        // rejection is skipped — meshes whose tight AABB+margin contains
        // the eye are still rasterised. Empirically the guard was over-
        // eager (rejected close-up walls; didn't protect against the
        // concave-shell failure mode it was written for). Default off.
        static bool OcclusionInsideOccluderGuard;

        static float OcclusionDepthSlackWorldUnits;
        static unsigned int OcclusionOccluderMaxTriangles;
        static unsigned int OcclusionOccludeeMinRadius;

        static bool OcclusionEnableInterior;
        static bool OcclusionEnableExterior;
        static bool OcclusionSkipTerrainOccludees;
        // Tri-state: 0=Off, 1=Raster, 2=Horizon. Lua side accepts bool too
        // (true→1, false→0); see Config.cpp's parser.
        static int  OcclusionAggregateTerrain;
        static unsigned int OcclusionTerrainResolution;

        static bool OcclusionCullLights;
        static unsigned int OcclusionLightCullHysteresisFrames;

        static bool OcclusionAsyncOccluders;
        static unsigned int OcclusionThreadpoolThreadCount;
        static unsigned int OcclusionThreadpoolBinsW;
        static unsigned int OcclusionThreadpoolBinsH;
        static unsigned int OcclusionTemporalCoherenceFrames;

        // Restart-only: latched in installPatches(). MOC requires
        // width % 8 == 0 and height % 4 == 0; installPatches rounds and
        // clamps.
        static unsigned int OcclusionMaskWidth;
        static unsigned int OcclusionMaskHeight;

        // Per-phase microsecond budgets. 0 = unlimited. Predictive skip
        // bails the whole phase when EMA(prev) > 2× budget; spike clip
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

        // Restart-only freeze diagnostic: spawns a detached thread that
        // dumps MSOC.forensics.txt every 250ms. Read once in
        // installPatches() (before Lua's first configure() call), so MCM
        // edits only take effect on the next launch. Default off.
        static bool OcclusionForensicsWatchdog;
    };

    // msoc.configure(table) — reads each field at stack index 1 and
    // writes through to the matching static. Unknown keys ignored;
    // missing keys leave the static untouched.
    int configure(lua_State* L);

    // Hardware tier classifier. On weaker CPUs (no AVX2, ≤4 threads)
    // the async occluder path can be net-negative, so the tier maps to
    // a different default set of threadpool/mask knobs. simdImpl values
    // match MaskedOcclusionCulling::Implementation (SSE2=0, SSE41=1,
    // AVX2=2, AVX512=3); negative = probe failed, treat as Low.
    enum class HardwareTier { Low = 0, Mid = 1, High = 2 };
    HardwareTier classifyHardwareTier(int simdImpl, unsigned hwConcurrency);
    const char*  hardwareTierName(HardwareTier tier);
    void         applyHardwareTierDefaults(HardwareTier tier);
}
