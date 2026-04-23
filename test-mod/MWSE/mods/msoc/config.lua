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

    -- Threadpool.
    OcclusionAsyncOccluders             = true,
    OcclusionThreadpoolThreadCount      = 0,  -- 0 = auto-pick
    OcclusionThreadpoolBinsW            = 4,
    OcclusionThreadpoolBinsH            = 2,
    OcclusionTemporalCoherenceFrames    = 4,
    OcclusionParallelDrain              = false,

    -- Logging.
    OcclusionLogPerFrame                = false,
    OcclusionLogAggregate               = false,

    -- Debug tints.
    DebugOcclusionTintOccluded          = false,
    DebugOcclusionTintTested            = false,
    DebugOcclusionTintOccluder          = false,
}

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
