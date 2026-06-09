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
// 1 = Raster (default for mid/high tier where async parallelizes the
// rasterization). Low tier overrides to 2 (Horizon) where main-thread
// bounded cost is preferable to per-subcell raster work.
int Configuration::OcclusionAggregateTerrain = 1;
// 0=Full(5x5), 1=Half(3x3), 2=Corners(2x2). See currentTerrainStep().
unsigned int Configuration::OcclusionTerrainResolution = 1;

// Default off in 1.1.0: A/B in a Vivec canton at night showed the
// feature is net-negative (~12% FPS regression). Bracketed savings
// were real (~480 us/frame less drain/display) but engine-side
// relighting churn cost more elsewhere. The C++ knob stays so a
// user can flip it via msoc.json if they want to retest on their
// hardware; the MCM toggle and hysteresis slider are hidden.
bool Configuration::OcclusionCullLights = false;
unsigned int Configuration::OcclusionLightCullHysteresisFrames = 3;

bool Configuration::OcclusionAsyncOccluders = true;
unsigned int Configuration::OcclusionThreadpoolThreadCount = 0;
unsigned int Configuration::OcclusionThreadpoolBinsW = 4;
unsigned int Configuration::OcclusionThreadpoolBinsH = 2;
unsigned int Configuration::OcclusionTemporalCoherenceFrames = 4;

// Tier-aware overrides (applyHardwareTierDefaults) are applied before
// installPatches latches these into kMsocWidth/Height.
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

// classifyHardwareTier / hardwareTierName moved to HardwareTier.cpp (pure,
// unit-tested). applyHardwareTierDefaults stays here - it writes Configuration.
void applyHardwareTierDefaults(HardwareTier tier) {
    // Threadpool / async / mask knobs are tier-sensitive. So is
    // OcclusionSkipTerrainOccludees: A/B in a dense Vivec exterior showed
    // ~1.9 ms/frame `displayUs` saved by letting terrain leaves flow
    // through TestRect on mid/high tiers (denser mask means a meaningful
    // fraction of terrain reads OCCLUDED and skips display()). On low
    // tier the extra TestRect work eats the classifyBudget, so the
    // bypass stays on. Occluder *selection* thresholds (radius/dim/etc.)
    // remain scene-shape-sensitive only.
    switch (tier) {
        case HardwareTier::Low:
            // SSE4.1 or <=4 threads: fixed per-frame threadpool tax
            // (Wake/Flush/Suspend) outweighs the parallelism win on
            // 4-wide SIMD. Synchronous skips the tax entirely.
            // Mask 256x128 = 1/4 rasterization work per triangle. Tight
            // budgets bound spike cost (one log showed 32ms drainUs
            // spikes on i5-2400; 1500us caps that at ~5% of a 30fps
            // frame).
            Configuration::OcclusionAsyncOccluders = false;
            Configuration::OcclusionThreadpoolBinsW = 2;
            Configuration::OcclusionThreadpoolBinsH = 1;
            Configuration::OcclusionMaskWidth = 256;
            Configuration::OcclusionMaskHeight = 128;
            Configuration::OcclusionRasterizeBudgetUs = 1500;
            Configuration::OcclusionClassifyBudgetUs = 1500;
            // Keep the bypass on - classifyUs headroom is tight here
            // and the displayUs win is smaller (the mask is also smaller).
            Configuration::OcclusionSkipTerrainOccludees = true;
            break;

        case HardwareTier::Mid:
            // 6-8 threads with AVX2. 4x2=8 bins is more atomic
            // ping-pong than parallelism with ~4-6 workers; 2x2 keeps
            // work-stealing alive at half the per-bin coordination cost.
            // Mask 384x192 ~ 56% of full work.
            Configuration::OcclusionAsyncOccluders = true;
            Configuration::OcclusionThreadpoolBinsW = 2;
            Configuration::OcclusionThreadpoolBinsH = 2;
            Configuration::OcclusionMaskWidth = 384;
            Configuration::OcclusionMaskHeight = 192;
            Configuration::OcclusionRasterizeBudgetUs = 3000;
            Configuration::OcclusionClassifyBudgetUs = 3000;
            Configuration::OcclusionSkipTerrainOccludees = false;
            break;

        case HardwareTier::High:
            // Explicit assignment so toggling tiers via Lua resets cleanly.
            Configuration::OcclusionAsyncOccluders = true;
            Configuration::OcclusionThreadpoolBinsW = 4;
            Configuration::OcclusionThreadpoolBinsH = 2;
            Configuration::OcclusionMaskWidth = 512;
            Configuration::OcclusionMaskHeight = 256;
            Configuration::OcclusionRasterizeBudgetUs = 0;
            Configuration::OcclusionClassifyBudgetUs = 0;
            Configuration::OcclusionSkipTerrainOccludees = false;
            break;
    }
}

int configure(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);

    readBool(L, 1, "EnableMSOC", Configuration::EnableMSOC);

    readBool(L, 1, "DebugOcclusionTintOccluded", Configuration::DebugOcclusionTintOccluded);
    readBool(L, 1, "DebugOcclusionTintTested", Configuration::DebugOcclusionTintTested);
    readBool(L, 1, "DebugOcclusionTintOccluder", Configuration::DebugOcclusionTintOccluder);

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

    readBool(L, 1, "OcclusionCullLights", Configuration::OcclusionCullLights);
    readUInt(L, 1, "OcclusionLightCullHysteresisFrames", Configuration::OcclusionLightCullHysteresisFrames);

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
