#include "Config.h"

extern "C" {
#include "lua.h"
#include "lauxlib.h"
}

namespace msoc {

    bool Configuration::EnableMSOC = true;

    bool Configuration::DebugOcclusionTintOccluded = false;
    bool Configuration::DebugOcclusionTintTested   = false;
    bool Configuration::DebugOcclusionTintOccluder = false;

    // Interior defaults lean smaller: rooms, furniture, crates are
    // worth rasterising. Exterior keeps the old defaults (exclude
    // clutter; terrain is handled separately by aggregate).
    float Configuration::OcclusionOccluderRadiusMinInterior    = 128.0f;
    float Configuration::OcclusionOccluderRadiusMinExterior    = 256.0f;
    float Configuration::OcclusionOccluderRadiusMaxInterior    = 2048.0f;
    float Configuration::OcclusionOccluderRadiusMaxExterior    = 4096.0f;
    float Configuration::OcclusionOccluderMinDimensionInterior =  64.0f;
    float Configuration::OcclusionOccluderMinDimensionExterior = 128.0f;
    float Configuration::OcclusionInsideOccluderMarginInterior =  64.0f;
    float Configuration::OcclusionInsideOccluderMarginExterior =  64.0f;

    float Configuration::OcclusionDepthSlackWorldUnits = 128.0f;
    unsigned int Configuration::OcclusionOccluderMaxTriangles = 4096;
    unsigned int Configuration::OcclusionOccludeeMinRadius    = 1;

    bool Configuration::OcclusionEnableInterior        = true;
    bool Configuration::OcclusionEnableExterior        = true;
    bool Configuration::OcclusionSkipTerrainOccludees  = true;
    bool Configuration::OcclusionAggregateTerrain      = true;
    // _Claude_ 0=Full(5x5), 1=Half(3x3), 2=Corners(2x2). Index→step
    // mapping in PatchOcclusionCulling.cpp::currentTerrainStep().
    unsigned int Configuration::OcclusionTerrainResolution = 1;

    bool Configuration::OcclusionCullLights                  = true;
    unsigned int Configuration::OcclusionLightCullHysteresisFrames = 3;

    bool Configuration::OcclusionAsyncOccluders             = true;
    unsigned int Configuration::OcclusionThreadpoolThreadCount   = 0;
    unsigned int Configuration::OcclusionThreadpoolBinsW         = 4;
    unsigned int Configuration::OcclusionThreadpoolBinsH         = 2;
    unsigned int Configuration::OcclusionTemporalCoherenceFrames = 4;
    bool Configuration::OcclusionParallelDrain              = false;

    bool Configuration::OcclusionLogPerFrame  = false;
    bool Configuration::OcclusionLogAggregate = false;

}

namespace {

// Read tbl[key] off the stack as a boolean and store in `out`. nil →
// leave `out` untouched (so missing keys keep the C++ default). Pops
// the value either way.
void readBool(lua_State* L, int tbl, const char* key, bool& out) {
    lua_getfield(L, tbl, key);
    if (!lua_isnil(L, -1)) {
        out = lua_toboolean(L, -1) != 0;
    }
    lua_pop(L, 1);
}

void readFloat(lua_State* L, int tbl, const char* key, float& out) {
    lua_getfield(L, tbl, key);
    if (lua_isnumber(L, -1)) {
        out = static_cast<float>(lua_tonumber(L, -1));
    }
    lua_pop(L, 1);
}

void readUInt(lua_State* L, int tbl, const char* key, unsigned int& out) {
    lua_getfield(L, tbl, key);
    if (lua_isnumber(L, -1)) {
        const lua_Number n = lua_tonumber(L, -1);
        // Negative values would wrap around to huge unsigneds; clamp to 0.
        out = (n < 0.0) ? 0u : static_cast<unsigned int>(n);
    }
    lua_pop(L, 1);
}

} // namespace

namespace msoc {

int configure(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);

    readBool (L, 1, "EnableMSOC",                          Configuration::EnableMSOC);

    readBool (L, 1, "DebugOcclusionTintOccluded",          Configuration::DebugOcclusionTintOccluded);
    readBool (L, 1, "DebugOcclusionTintTested",            Configuration::DebugOcclusionTintTested);
    readBool (L, 1, "DebugOcclusionTintOccluder",          Configuration::DebugOcclusionTintOccluder);

    readFloat(L, 1, "OcclusionOccluderRadiusMinInterior",    Configuration::OcclusionOccluderRadiusMinInterior);
    readFloat(L, 1, "OcclusionOccluderRadiusMinExterior",    Configuration::OcclusionOccluderRadiusMinExterior);
    readFloat(L, 1, "OcclusionOccluderRadiusMaxInterior",    Configuration::OcclusionOccluderRadiusMaxInterior);
    readFloat(L, 1, "OcclusionOccluderRadiusMaxExterior",    Configuration::OcclusionOccluderRadiusMaxExterior);
    readFloat(L, 1, "OcclusionOccluderMinDimensionInterior", Configuration::OcclusionOccluderMinDimensionInterior);
    readFloat(L, 1, "OcclusionOccluderMinDimensionExterior", Configuration::OcclusionOccluderMinDimensionExterior);
    readFloat(L, 1, "OcclusionInsideOccluderMarginInterior", Configuration::OcclusionInsideOccluderMarginInterior);
    readFloat(L, 1, "OcclusionInsideOccluderMarginExterior", Configuration::OcclusionInsideOccluderMarginExterior);
    readFloat(L, 1, "OcclusionDepthSlackWorldUnits",       Configuration::OcclusionDepthSlackWorldUnits);
    readUInt (L, 1, "OcclusionOccluderMaxTriangles",       Configuration::OcclusionOccluderMaxTriangles);
    readUInt (L, 1, "OcclusionOccludeeMinRadius",          Configuration::OcclusionOccludeeMinRadius);

    readBool (L, 1, "OcclusionEnableInterior",             Configuration::OcclusionEnableInterior);
    readBool (L, 1, "OcclusionEnableExterior",             Configuration::OcclusionEnableExterior);
    readBool (L, 1, "OcclusionSkipTerrainOccludees",       Configuration::OcclusionSkipTerrainOccludees);
    readBool (L, 1, "OcclusionAggregateTerrain",           Configuration::OcclusionAggregateTerrain);
    readUInt (L, 1, "OcclusionTerrainResolution",          Configuration::OcclusionTerrainResolution);

    readBool (L, 1, "OcclusionCullLights",                 Configuration::OcclusionCullLights);
    readUInt (L, 1, "OcclusionLightCullHysteresisFrames",  Configuration::OcclusionLightCullHysteresisFrames);

    readBool (L, 1, "OcclusionAsyncOccluders",             Configuration::OcclusionAsyncOccluders);
    readUInt (L, 1, "OcclusionThreadpoolThreadCount",      Configuration::OcclusionThreadpoolThreadCount);
    readUInt (L, 1, "OcclusionThreadpoolBinsW",            Configuration::OcclusionThreadpoolBinsW);
    readUInt (L, 1, "OcclusionThreadpoolBinsH",            Configuration::OcclusionThreadpoolBinsH);
    readUInt (L, 1, "OcclusionTemporalCoherenceFrames",    Configuration::OcclusionTemporalCoherenceFrames);
    readBool (L, 1, "OcclusionParallelDrain",              Configuration::OcclusionParallelDrain);

    readBool (L, 1, "OcclusionLogPerFrame",                Configuration::OcclusionLogPerFrame);
    readBool (L, 1, "OcclusionLogAggregate",               Configuration::OcclusionLogAggregate);

    return 0;
}

} // namespace msoc
