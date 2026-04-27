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

    // _Claude_ Defaults preserve historical behaviour (512×256 mask).
    // Tier-aware overrides are applied in applyHardwareTierDefaults
    // before installPatches latches them into the file-scope kMsoc*
    // variables in PatchOcclusionCulling.cpp.
    unsigned int Configuration::OcclusionMaskWidth  = 512;
    unsigned int Configuration::OcclusionMaskHeight = 256;

    // _Claude_ Phase budgets default to 0 (unlimited) so the patch's
    // historical behaviour is preserved on Mid/High tier and when the
    // user explicitly disables them. Low tier sets non-zero values
    // that bound the per-frame cost — see applyHardwareTierDefaults.
    unsigned int Configuration::OcclusionRasterizeBudgetUs = 0;
    unsigned int Configuration::OcclusionClassifyBudgetUs  = 0;

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

HardwareTier classifyHardwareTier(int simdImpl, unsigned hwConcurrency) {
    // simdImpl values match MaskedOcclusionCulling::Implementation:
    //   SSE2=0, SSE41=1, AVX2=2, AVX512=3. Negative = probe failed.
    // The AVX2 cutoff matters because Sandy/Ivy Bridge (≤2012) lack the
    // FMA + BMI1/2 + AVX2-int feature combo MOC requires, and fall back
    // to SSE4.1's 4-wide lanes — half the per-tile rasterization
    // throughput of Haswell+. Combined with the 4-thread ceiling on
    // those parts, async occluders pay more in synchronization overhead
    // than they save in parallel rasterization.
    const bool noAvx2 = (simdImpl < 2);
    if (noAvx2 || hwConcurrency <= 4) return HardwareTier::Low;
    if (hwConcurrency <= 8)           return HardwareTier::Mid;
    return HardwareTier::High;
}

const char* hardwareTierName(HardwareTier tier) {
    switch (tier) {
        case HardwareTier::Low:  return "low";
        case HardwareTier::Mid:  return "mid";
        case HardwareTier::High: return "high";
    }
    return "unknown";
}

void applyHardwareTierDefaults(HardwareTier tier) {
    // Only the threadpool / async knobs are tier-sensitive. Occluder
    // selection thresholds (radius, dimension, margin) are scene-
    // shape sensitive, not CPU sensitive — leave those at their
    // declared defaults regardless of tier. Same for logging and
    // debug tints.
    switch (tier) {
        case HardwareTier::Low:
            // SSE4.1 or ≤4 threads. The threadpool dance (WakeThreads
            // cv broadcast + per-frame Flush yield-spin + SuspendThreads)
            // costs roughly the same wall-clock everywhere, but 2-wide
            // worker parallelism on 4-wide SIMD means each worker
            // produces half the depth-buffer coverage per microsecond
            // it was a Haswell+/AVX2 part. Synchronous (g_msoc directly)
            // skips the fixed per-frame tax entirely and tends to win.
            //
            // Bins shrunk to 2x1: with async off the threadpool is
            // allocated but unused, so the only effect of fewer bins is
            // a smaller wasted allocation. (TODO: skip threadpool ctor
            // when async is off.)
            //
            // Mask 256×128: a quarter of the rasterization work per
            // triangle. Loses precision at the silhouette of small
            // distant occluders, but that's exactly the part we're
            // happiest to drop on a CPU-bound 4-thread system.
            Configuration::OcclusionAsyncOccluders = false;
            Configuration::OcclusionThreadpoolBinsW = 2;
            Configuration::OcclusionThreadpoolBinsH = 1;
            Configuration::OcclusionMaskWidth      = 256;
            Configuration::OcclusionMaskHeight     = 128;
            // _Claude_ Tight budgets bound the spike cost. Sized for
            // a 4-thread SSE4.1 part where the user's log showed
            // drainUs spiking to 32ms; 1500us cuts that off at ~5%
            // of a 30fps frame budget. Steady-state (~1.5-3ms in
            // that log) sits at or under the cap, so most frames
            // are unaffected. Predictive skip kicks in only when
            // EMA sustainably overruns 2× budget — i.e. the system
            // is genuinely behind, not on one-shot spikes.
            Configuration::OcclusionRasterizeBudgetUs = 1500;
            Configuration::OcclusionClassifyBudgetUs  = 1500;
            break;

        case HardwareTier::Mid:
            // 6–8 threads with AVX2. Async pays off, but 4×2=8 bins
            // gives more atomic ping-pong than parallelism: workers
            // (capped at hw-2 ≈ 4–6) end up sharing 8 bin queues, and
            // each bin's mutex + render-pointer + queue head/tail
            // touches its own cache line. 2×2 keeps the work-stealing
            // path enabled (binCount > threadCount) at half the per-
            // bin coordination cost.
            //
            // Mask 384×192: ~56% of the rasterization work of 512×256
            // — keeps the silhouette accuracy decent while clawing
            // back per-frame ms on mid-tier parts.
            Configuration::OcclusionAsyncOccluders = true;
            Configuration::OcclusionThreadpoolBinsW = 2;
            Configuration::OcclusionThreadpoolBinsH = 2;
            Configuration::OcclusionMaskWidth      = 384;
            Configuration::OcclusionMaskHeight     = 192;
            // Looser budgets — Mid CPUs handle steady-state fine,
            // budgets only intervene on cell-load spikes.
            Configuration::OcclusionRasterizeBudgetUs = 3000;
            Configuration::OcclusionClassifyBudgetUs  = 3000;
            break;

        case HardwareTier::High:
            // ≥10 threads with AVX2/AVX-512. Current defaults — explicit
            // assignment so a user toggling between tiers via Lua sees
            // a consistent reset, not residual values from another tier.
            Configuration::OcclusionAsyncOccluders = true;
            Configuration::OcclusionThreadpoolBinsW = 4;
            Configuration::OcclusionThreadpoolBinsH = 2;
            Configuration::OcclusionMaskWidth      = 512;
            Configuration::OcclusionMaskHeight     = 256;
            // Effectively unlimited — High-tier hardware doesn't
            // need the bound, and the budget machinery's overhead
            // (one branch per phase entry, sampled timer in the
            // drain inner loop) is negligible but not zero.
            Configuration::OcclusionRasterizeBudgetUs = 0;
            Configuration::OcclusionClassifyBudgetUs  = 0;
            break;
    }
}

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
    // _Claude_ Mask resolution is restart-only — these statics are read
    // once during installPatches() and latched into kMsocWidth/Height.
    // We still accept the keys here so msoc.json round-trips cleanly
    // (the value gets stored, just doesn't take effect until next launch).
    readUInt (L, 1, "OcclusionMaskWidth",                  Configuration::OcclusionMaskWidth);
    readUInt (L, 1, "OcclusionMaskHeight",                 Configuration::OcclusionMaskHeight);
    readUInt (L, 1, "OcclusionRasterizeBudgetUs",          Configuration::OcclusionRasterizeBudgetUs);
    readUInt (L, 1, "OcclusionClassifyBudgetUs",           Configuration::OcclusionClassifyBudgetUs);

    readBool (L, 1, "OcclusionLogPerFrame",                Configuration::OcclusionLogPerFrame);
    readBool (L, 1, "OcclusionLogAggregate",               Configuration::OcclusionLogAggregate);

    return 0;
}

} // namespace msoc
