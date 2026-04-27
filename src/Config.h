#pragma once

// Plugin config statics. The patch source (PatchOcclusionCulling.cpp)
// references these as `Configuration::Foo` from inside namespace
// msoc::patch::occlusion, and unqualified-name lookup walks up to
// msoc::Configuration here.
//
// Population path: Lua side calls msoc.configure(tbl). The C function
// in Config.cpp walks the table with lua_getfield + lua_to{boolean,
// number,integer} and writes through to these statics. No sol2, no
// usertype — config is a plain Lua table on the Lua side, pushed
// across the FFI by value when MCM changes commit.
//
// Fields the patch never reads are intentionally absent — this isn't
// a clone of MWSE's Configuration, just the slice the occlusion patch
// uses.

struct lua_State;

namespace msoc {
    class Configuration {
    public:
        static bool EnableMSOC;

        static bool DebugOcclusionTintOccluded;
        static bool DebugOcclusionTintTested;
        static bool DebugOcclusionTintOccluder;

        // _Claude_ Scene-type split: interior favours smaller occluders
        // (pillars, crates, furniture); exterior raises the bar to skip
        // clutter. Resolved into g_*Effective per-frame at top-level
        // CullShow_detour entry once scene type is known.
        static float OcclusionOccluderRadiusMinInterior;
        static float OcclusionOccluderRadiusMinExterior;
        static float OcclusionOccluderRadiusMaxInterior;
        static float OcclusionOccluderRadiusMaxExterior;
        static float OcclusionOccluderMinDimensionInterior;
        static float OcclusionOccluderMinDimensionExterior;
        static float OcclusionInsideOccluderMarginInterior;
        static float OcclusionInsideOccluderMarginExterior;

        static float OcclusionDepthSlackWorldUnits;
        static unsigned int OcclusionOccluderMaxTriangles;
        static unsigned int OcclusionOccludeeMinRadius;

        static bool OcclusionEnableInterior;
        static bool OcclusionEnableExterior;
        static bool OcclusionSkipTerrainOccludees;
        // LAYER-A-HORIZON: tri-state — 0=Off, 1=Raster, 2=Horizon. Was
        // bool prior to the LAYER-A handoff. Lua side accepts both bool
        // (true→1, false→0) and int forms; see Config.cpp's parser.
        // Default Raster initially; flipped to Horizon after Step 6
        // validation per the handoff doc.
        static int  OcclusionAggregateTerrain;
        static unsigned int OcclusionTerrainResolution;

        static bool OcclusionCullLights;
        static unsigned int OcclusionLightCullHysteresisFrames;

        static bool OcclusionAsyncOccluders;
        static unsigned int OcclusionThreadpoolThreadCount;
        static unsigned int OcclusionThreadpoolBinsW;
        static unsigned int OcclusionThreadpoolBinsH;
        static unsigned int OcclusionTemporalCoherenceFrames;

        // _Claude_ Mask resolution. Read once during installPatches() —
        // subsequent msoc.configure() updates are ignored for this
        // session. MOC requires width % 8 == 0 and height % 4 == 0
        // (asserted in MaskedOcclusionCullingCommon.inl::SetResolution);
        // installPatches rounds down + clamps to safe bounds.
        static unsigned int OcclusionMaskWidth;
        static unsigned int OcclusionMaskHeight;

        // _Claude_ Per-phase microsecond budgets. 0 = unlimited.
        // Hybrid budgeting (see PatchOcclusionCulling.cpp):
        //   - Predictive skip: at top-of-frame, if EMA(prevFrames) > 2× budget,
        //     skip the whole phase this frame (mark all testees Visible /
        //     don't submit any occluders). Self-regulating, zero in-loop
        //     overhead.
        //   - Spike clip: inside the phase, sampled time check (every 32
        //     iterations of the inner loop) bails when this frame's running
        //     elapsed exceeds the budget. Bounds worst-case latency.
        // Soft fallback in both: untested testees become Visible (over-render,
        // never wrongly cull); unsubmitted occluders just miss the mask.
        // MOC's TestRect invariant (false negatives only) keeps it safe.
        //
        // Both budgets target MSOC-only work — not vanilla rendering:
        //   - RasterizeBudgetUs bounds the cumulative ScopedUsAccumulator
        //     time inside MOC::RenderTriangles, not wall-clock-since-frame-
        //     start (which would also include cullShowBody traversal).
        //   - ClassifyBudgetUs bounds the TestRect loop in classifyDrainRange,
        //     not the whole drain phase (whose phase 2 display() calls are
        //     vanilla D3D8 submissions that run regardless of MSOC).
        static unsigned int OcclusionRasterizeBudgetUs;
        static unsigned int OcclusionClassifyBudgetUs;

        static bool OcclusionLogPerFrame;
        static bool OcclusionLogAggregate;
    };

    // Lua entry: msoc.configure(table). Reads each field of the table
    // at stack index 1 and writes it into the matching static above.
    // Unknown keys ignored; missing keys leave the static untouched.
    int configure(lua_State* L);

    // _Claude_ Hardware-tier classifier. The plugin's per-frame work is
    // split between the main render thread (occluder selection, depth
    // testing) and the MOC threadpool (rasterization). On weaker CPUs
    // — Sandy Bridge / Ivy Bridge with no AVX2, or any 4-thread part —
    // the cost/benefit shifts: SSE4.1 raster is 2× slower per tile, the
    // threadpool's WakeThreads/Flush/SuspendThreads tax stays the same,
    // and only 2 spare hardware threads are available for workers. The
    // result is that the async occluder path can be net-negative.
    //
    // Tiers map to coarse defaults applied at plugin load. Lua-side
    // first-run defaults (test-mod/.../config.lua) read msoc.hardwareTier
    // and apply matching overrides; the C++-side defaults below run as
    // defence-in-depth in case Lua never calls configure().
    //
    // Integer parameter to keep this header MOC-free; values match
    // MaskedOcclusionCulling::Implementation (SSE2=0, SSE41=1, AVX2=2,
    // AVX512=3). Negative means probe failed — treat as Low.
    enum class HardwareTier { Low = 0, Mid = 1, High = 2 };
    HardwareTier classifyHardwareTier(int simdImpl, unsigned hwConcurrency);
    const char*  hardwareTierName(HardwareTier tier);
    void         applyHardwareTierDefaults(HardwareTier tier);
}
