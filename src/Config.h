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
        static bool OcclusionAggregateTerrain;
        static unsigned int OcclusionTerrainResolution;

        static bool OcclusionCullLights;
        static unsigned int OcclusionLightCullHysteresisFrames;

        static bool OcclusionAsyncOccluders;
        static unsigned int OcclusionThreadpoolThreadCount;
        static unsigned int OcclusionThreadpoolBinsW;
        static unsigned int OcclusionThreadpoolBinsH;
        static unsigned int OcclusionTemporalCoherenceFrames;

        static bool OcclusionLogPerFrame;
        static bool OcclusionLogAggregate;
    };

    // Lua entry: msoc.configure(table). Reads each field of the table
    // at stack index 1 and writes it into the matching static above.
    // Unknown keys ignored; missing keys leave the static untouched.
    int configure(lua_State* L);
}
