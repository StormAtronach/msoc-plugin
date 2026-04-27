-- msoc Lua-side config.
--
-- Standard MWSE pattern: defaults + mwse.loadConfig + JSON persistence.
-- Field names match the C++ msoc::Configuration field names exactly so a
-- generic `for k, v in pairs(config)` loop on the C side suffices, no
-- translation table needed.
--
-- syncToNative(plugin) hands the loaded values to the native plugin via
-- the bound `msoc.configure(table)` C function. UI Expansion uses the
-- same shape: pure-Lua config, native gets values by-value when called.

local default_config = {
    -- Master gate.
    EnableMSOC                          = true,

    -- Scene gates.
    OcclusionEnableInterior             = true,
    OcclusionEnableExterior             = true,
    OcclusionSkipTerrainOccludees       = true,
    OcclusionAggregateTerrain           = true,
    OcclusionTerrainResolution          = 1,

    -- Light culling.
    OcclusionCullLights                 = true,
    OcclusionLightCullHysteresisFrames  = 3,

    -- Occluder selection (split per scene: interiors favour smaller
    -- occluders; exteriors raise the bar to skip clutter).
    OcclusionOccluderRadiusMinInterior    = 128.0,
    OcclusionOccluderRadiusMinExterior    = 256.0,
    OcclusionOccluderRadiusMaxInterior    = 2048.0,
    OcclusionOccluderRadiusMaxExterior    = 4096.0,
    OcclusionOccluderMinDimensionInterior = 64.0,
    OcclusionOccluderMinDimensionExterior = 128.0,
    OcclusionInsideOccluderMarginInterior = 64.0,
    OcclusionInsideOccluderMarginExterior = 64.0,

    -- Shared (cost / geometry, not scene-dependent).
    OcclusionDepthSlackWorldUnits       = 128.0,
    OcclusionOccluderMaxTriangles       = 4096,
    OcclusionOccludeeMinRadius          = 1,

    -- Threadpool. The three fields below get overridden by hardware
    -- tier (see applyTierDefaults) before mwse.loadConfig — first-run
    -- users on weaker CPUs get conservative defaults, saved msoc.json
    -- still wins over both.
    OcclusionAsyncOccluders             = true,
    OcclusionThreadpoolThreadCount      = 0,  -- 0 = auto-pick
    OcclusionThreadpoolBinsW            = 4,
    OcclusionThreadpoolBinsH            = 2,
    OcclusionTemporalCoherenceFrames    = 4,

    -- Mask resolution. Restart to apply — installPatches latches
    -- these into kMsocWidth/Height once and ignores subsequent
    -- changes for the session. Tier-overridden below (see
    -- applyTierDefaults). MOC requires width%8==0 and height%4==0
    -- (asserted in C++); plugin clamps to [64..2048] × [32..1024].
    OcclusionMaskWidth                  = 512,
    OcclusionMaskHeight                 = 256,

    -- Per-phase budgets in microseconds. 0 = unlimited (gate off).
    -- Hybrid budgeting in C++ — both budgets target MSOC-only work:
    --   - Rasterize: cumulative MOC::RenderTriangles SIMD time only,
    --     not wall-clock-since-frame-start (which would include the
    --     vanilla cullShowBody traversal between rasterize calls).
    --   - Classify: the TestRect loop in classifyDrainRange only,
    --     not the whole drain phase (whose phase 2 display() calls
    --     are vanilla D3D8 submissions that happen regardless).
    -- Tier-overridden below. Low-tier sets non-zero values that
    -- bound spike cost; High-tier disables the gates entirely.
    -- (Was OcclusionDrainBudgetUs in 0.0.9; renamed in 0.0.10 for
    --  semantic accuracy. Migration cleans up the old key.)
    OcclusionRasterizeBudgetUs          = 0,
    OcclusionClassifyBudgetUs           = 0,

    -- Logging.
    OcclusionLogPerFrame                = false,
    OcclusionLogAggregate               = false,

    -- Debug tints.
    DebugOcclusionTintOccluded          = false,
    DebugOcclusionTintTested            = false,
    DebugOcclusionTintOccluder          = false,
}

-- Tier defaults must match Config.cpp::applyHardwareTierDefaults so a
-- first-run user (no msoc.json yet) and a configure-only user (no Lua
-- side) both end up at the same Configuration::* values. Diverging the
-- two would be a quiet bug — load order picks one set silently.
--
-- Why this needs to live on the Lua side too: the C++ side runs
-- applyHardwareTierDefaults during luaopen_msoc, but cfg.syncToNative
-- pushes the Lua default_config across the FFI right after, clobbering
-- those C++ picks. mwse.loadConfig fills in any field not present in
-- the user's saved JSON from default_config — which is why we mutate
-- default_config here, BEFORE that load happens.
local function applyTierDefaults(plugin, target)
    local tier = plugin and plugin.hardwareTier
    if tier == "low" then
        -- SSE4.1 or ≤4 threads: WakeThreads + Flush + SuspendThreads
        -- per-frame fixed cost outweighs the parallel rasterization
        -- win when SIMD is 4-wide and only 1-2 spare workers are
        -- available. Synchronous path skips the entire threadpool
        -- dance. Mask 256×128 = ¼ rasterization work. Tight per-
        -- phase budgets clip cell-load spikes (we saw 32 ms drain
        -- spikes on i5-2400 in the wild).
        target.OcclusionAsyncOccluders     = false
        target.OcclusionThreadpoolBinsW    = 2
        target.OcclusionThreadpoolBinsH    = 1
        target.OcclusionMaskWidth          = 256
        target.OcclusionMaskHeight         = 128
        target.OcclusionRasterizeBudgetUs  = 1500
        target.OcclusionClassifyBudgetUs   = 1500
    elseif tier == "mid" then
        -- 6-8 threads with AVX2: async pays off, but 4×2=8 bins is
        -- atomic-ping-pong overkill for ~4-6 workers. 2×2 keeps
        -- work-stealing alive at lower per-bin coordination cost.
        -- Mask 384×192 ≈ 56% of full work. Looser budgets, only
        -- intervene on spikes.
        target.OcclusionAsyncOccluders     = true
        target.OcclusionThreadpoolBinsW    = 2
        target.OcclusionThreadpoolBinsH    = 2
        target.OcclusionMaskWidth          = 384
        target.OcclusionMaskHeight         = 192
        target.OcclusionRasterizeBudgetUs  = 3000
        target.OcclusionClassifyBudgetUs   = 3000
    elseif tier == "high" then
        target.OcclusionAsyncOccluders     = true
        target.OcclusionThreadpoolBinsW    = 4
        target.OcclusionThreadpoolBinsH    = 2
        target.OcclusionMaskWidth          = 512
        target.OcclusionMaskHeight         = 256
        -- Budgets disabled — High-tier hardware doesn't need them.
        target.OcclusionRasterizeBudgetUs  = 0
        target.OcclusionClassifyBudgetUs   = 0
    end
    -- Unknown / nil tier: leave default_config untouched (matches
    -- legacy behaviour for safety).
end

-- Mutate default_config in place so mwse.loadConfig sees the tier-
-- adjusted values when filling unsaved fields.
local mscPlugin = include("msoc")
applyTierDefaults(mscPlugin, default_config)
if mscPlugin then
    mwse.log("[msoc] hardware tier=%s simdLevel=%s threads=%s — defaults: async=%s bins=%dx%d mask=%dx%d",
        tostring(mscPlugin.hardwareTier),
        tostring(mscPlugin.simdLevel),
        tostring(mscPlugin.cpuThreads),
        tostring(default_config.OcclusionAsyncOccluders),
        default_config.OcclusionThreadpoolBinsW,
        default_config.OcclusionThreadpoolBinsH,
        default_config.OcclusionMaskWidth,
        default_config.OcclusionMaskHeight)
end

local config = mwse.loadConfig("msoc", default_config) ---@cast config table
config.confPath = "msoc"
config.default  = default_config

-- Push the live config table into the native plugin. `plugin` is the
-- table returned from include("msoc"); pass it explicitly rather than
-- re-include()ing inside this module.
local function syncToNative(plugin)
    if not (plugin and plugin.configure) then
        mwse.log("[msoc] native configure() not present; syncToNative is a no-op.")
        return
    end
    plugin.configure(config)
end

return {
    config       = config,
    default      = default_config,
    syncToNative = syncToNative,
}
