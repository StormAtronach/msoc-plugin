#include "Config.h"

extern "C" {
#include "lua.h"
#include "lauxlib.h"
}

namespace msoc {

bool Configuration::EnableMSOC = true;

bool Configuration::DebugOcclusionTintOccluded = false;
bool Configuration::DebugOcclusionTintTested = false;
bool Configuration::DebugOcclusionTintOccluder = false;
bool Configuration::DebugMaskOverlay = false;

float Configuration::OcclusionOccluderRadiusMinInterior = 128.0f;
float Configuration::OcclusionOccluderRadiusMinExterior = 256.0f;
float Configuration::OcclusionOccluderRadiusMaxInterior = 2048.0f;
float Configuration::OcclusionOccluderRadiusMaxExterior = 4096.0f;
float Configuration::OcclusionOccluderMinDimensionInterior = 64.0f;
float Configuration::OcclusionOccluderMinDimensionExterior = 128.0f;
float Configuration::OcclusionInsideOccluderMarginInterior = 64.0f;
float Configuration::OcclusionInsideOccluderMarginExterior = 64.0f;
bool Configuration::OcclusionInsideOccluderGuard = false;

float Configuration::OcclusionDepthSlackWorldUnits = 128.0f;
unsigned int Configuration::OcclusionOccluderMaxTriangles = 4096;
unsigned int Configuration::OcclusionOccludeeMinRadius = 1;
bool Configuration::OcclusionOccludeeBoxTest = false;

bool Configuration::OcclusionEnableInterior = true;
bool Configuration::OcclusionEnableExterior = true;
bool Configuration::OcclusionSkipTerrainOccludees = true;
// 1 = Raster. config.lua's tier table sends 2 (Horizon) on low tier, where a
// bounded main-thread cost beats per-subcell raster work.
int Configuration::OcclusionAggregateTerrain = 1;
// 0=Full(5x5), 1=Half(3x3), 2=Corners(2x2). See currentTerrainStep().
unsigned int Configuration::OcclusionTerrainResolution = 1;
// Keep only CCW (front) occluder faces.
bool Configuration::OcclusionOccluderCCWOnly = true;
bool Configuration::OcclusionOccluderFrontToBack = true;

bool Configuration::OcclusionAsyncOccluders = true;
unsigned int Configuration::OcclusionThreadpoolThreadCount = 0;
unsigned int Configuration::OcclusionThreadpoolBinsW = 4;
unsigned int Configuration::OcclusionThreadpoolBinsH = 2;
unsigned int Configuration::OcclusionTemporalCoherenceFrames = 4;

// installPatches() latches these into kMsocWidth/Height, and main.lua calls
// it after configure(), so what lands here is whatever config.lua pushed.
unsigned int Configuration::OcclusionMaskWidth = 512;
unsigned int Configuration::OcclusionMaskHeight = 256;

unsigned int Configuration::OcclusionRasterizeBudgetUs = 0;
unsigned int Configuration::OcclusionClassifyBudgetUs = 0;

bool Configuration::OcclusionLogPerFrame = false;
bool Configuration::OcclusionLogAggregate = false;
bool Configuration::OcclusionLogCellCross = false;

bool Configuration::OcclusionForensicsWatchdog = false;

}  // namespace msoc

namespace {

// nil -> leave `out` untouched. Pops the value either way.
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
        // Negatives would wrap; clamp to 0.
        out = (n < 0.0) ? 0u : static_cast<unsigned int>(n);
    }
    lua_pop(L, 1);
}

// Tri-state with legacy-bool acceptance (the key was a bool before
// LAYER-A): false->0, true->1, int N->clamp(N, 0, 2).
void readTerrainOcclusionMode(lua_State* L, int tbl, const char* key, int& out) {
    lua_getfield(L, tbl, key);
    if (lua_isnumber(L, -1)) {
        const lua_Number n = lua_tonumber(L, -1);
        int v = static_cast<int>(n);
        if (v < 0) v = 0;
        if (v > 2) v = 2;
        out = v;
    } else if (lua_isboolean(L, -1)) {
        out = lua_toboolean(L, -1) ? 1 : 0;
    }
    lua_pop(L, 1);
}

}  // namespace

namespace msoc {

// classifyHardwareTier / hardwareTierName live in HardwareTier.cpp (pure,
// unit-tested). There is deliberately no applyHardwareTierDefaults here any
// more: config.lua owns the tier table and pushes the result through
// configure() before main.lua calls msoc.install(). Keeping a second copy in
// C++ meant the restart-only knobs latched the C++ table while the MCM showed
// the Lua one.

int configure(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);

    readBool(L, 1, "EnableMSOC", Configuration::EnableMSOC);

    readBool(L, 1, "DebugOcclusionTintOccluded", Configuration::DebugOcclusionTintOccluded);
    readBool(L, 1, "DebugOcclusionTintTested", Configuration::DebugOcclusionTintTested);
    readBool(L, 1, "DebugOcclusionTintOccluder", Configuration::DebugOcclusionTintOccluder);
    readBool(L, 1, "DebugMaskOverlay", Configuration::DebugMaskOverlay);

    readFloat(L, 1, "OcclusionOccluderRadiusMinInterior", Configuration::OcclusionOccluderRadiusMinInterior);
    readFloat(L, 1, "OcclusionOccluderRadiusMinExterior", Configuration::OcclusionOccluderRadiusMinExterior);
    readFloat(L, 1, "OcclusionOccluderRadiusMaxInterior", Configuration::OcclusionOccluderRadiusMaxInterior);
    readFloat(L, 1, "OcclusionOccluderRadiusMaxExterior", Configuration::OcclusionOccluderRadiusMaxExterior);
    readFloat(L, 1, "OcclusionOccluderMinDimensionInterior", Configuration::OcclusionOccluderMinDimensionInterior);
    readFloat(L, 1, "OcclusionOccluderMinDimensionExterior", Configuration::OcclusionOccluderMinDimensionExterior);
    readFloat(L, 1, "OcclusionInsideOccluderMarginInterior", Configuration::OcclusionInsideOccluderMarginInterior);
    readFloat(L, 1, "OcclusionInsideOccluderMarginExterior", Configuration::OcclusionInsideOccluderMarginExterior);
    readBool(L, 1, "OcclusionInsideOccluderGuard", Configuration::OcclusionInsideOccluderGuard);
    readFloat(L, 1, "OcclusionDepthSlackWorldUnits", Configuration::OcclusionDepthSlackWorldUnits);
    readUInt(L, 1, "OcclusionOccluderMaxTriangles", Configuration::OcclusionOccluderMaxTriangles);
    readUInt(L, 1, "OcclusionOccludeeMinRadius", Configuration::OcclusionOccludeeMinRadius);

    readBool(L, 1, "OcclusionEnableInterior", Configuration::OcclusionEnableInterior);
    readBool(L, 1, "OcclusionEnableExterior", Configuration::OcclusionEnableExterior);
    readBool(L, 1, "OcclusionSkipTerrainOccludees", Configuration::OcclusionSkipTerrainOccludees);
    readBool(L, 1, "OcclusionOccludeeBoxTest", Configuration::OcclusionOccludeeBoxTest);
    readTerrainOcclusionMode(L, 1, "OcclusionAggregateTerrain", Configuration::OcclusionAggregateTerrain);
    readUInt(L, 1, "OcclusionTerrainResolution", Configuration::OcclusionTerrainResolution);
    readBool(L, 1, "OcclusionOccluderCCWOnly", Configuration::OcclusionOccluderCCWOnly);
    readBool(L, 1, "OcclusionOccluderFrontToBack", Configuration::OcclusionOccluderFrontToBack);

    readBool(L, 1, "OcclusionAsyncOccluders", Configuration::OcclusionAsyncOccluders);
    readUInt(L, 1, "OcclusionThreadpoolThreadCount", Configuration::OcclusionThreadpoolThreadCount);
    readUInt(L, 1, "OcclusionThreadpoolBinsW", Configuration::OcclusionThreadpoolBinsW);
    readUInt(L, 1, "OcclusionThreadpoolBinsH", Configuration::OcclusionThreadpoolBinsH);
    readUInt(L, 1, "OcclusionTemporalCoherenceFrames", Configuration::OcclusionTemporalCoherenceFrames);
    // Restart-only - see header. Accepted here so msoc.json round-trips.
    readUInt(L, 1, "OcclusionMaskWidth", Configuration::OcclusionMaskWidth);
    readUInt(L, 1, "OcclusionMaskHeight", Configuration::OcclusionMaskHeight);
    readUInt(L, 1, "OcclusionRasterizeBudgetUs", Configuration::OcclusionRasterizeBudgetUs);
    readUInt(L, 1, "OcclusionClassifyBudgetUs", Configuration::OcclusionClassifyBudgetUs);

    readBool(L, 1, "OcclusionLogPerFrame", Configuration::OcclusionLogPerFrame);
    readBool(L, 1, "OcclusionLogAggregate", Configuration::OcclusionLogAggregate);
    readBool(L, 1, "OcclusionLogCellCross", Configuration::OcclusionLogCellCross);

    // Restart-only - see header. Accepted here so msoc.json round-trips.
    readBool(L, 1, "OcclusionForensicsWatchdog", Configuration::OcclusionForensicsWatchdog);

    return 0;
}

}  // namespace msoc
