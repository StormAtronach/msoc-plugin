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
    -- Optional tighter occludee test (object-space box after the sphere).
    OcclusionOccludeeBoxTest            = true,
    -- 0=Off, 1=Raster (default on every tier). The cost knob is the
    -- resolution below: 0=Full, 1=Half, 2=Corners. The low tier sets Corners,
    -- which at a fifth of Half's synchronous cost occludes about the same.
    -- (A value of 2 here used to select a 1D "Horizon" silhouette curtain;
    -- it was removed in 1.6.1 after measuring 4-5x the cost of the raster at
    -- equal resolution with less occlusion, and now reads as Raster.)
    OcclusionAggregateTerrain           = 1,
    OcclusionTerrainResolution          = 1,
    -- Cull non-CCW occluder faces (assumes ~99% of NIFs are CCW-wound).
    -- Roughly halves occluder rasterisation work; the rare CW-wound mesh
    -- is dropped from the mask (safe under-occlude, never a wrong-cull).
    OcclusionOccluderCCWOnly            = true,

    -- Occluder selection (split per scene: interiors favour smaller
    -- occluders; exteriors raise the bar to skip clutter).
    OcclusionOccluderRadiusMinInterior    = 128.0,
    OcclusionOccluderRadiusMinExterior    = 256.0,
    OcclusionOccluderRadiusMaxInterior    = 2048.0,
    OcclusionOccluderRadiusMaxExterior    = 7040.0,
    OcclusionOccluderMinDimensionInterior = 64.0,
    OcclusionOccluderMinDimensionExterior = 128.0,
    OcclusionInsideOccluderMarginInterior = 64.0,
    OcclusionInsideOccluderMarginExterior = 64.0,

    -- Shared (cost / geometry, not scene-dependent).
    OcclusionDepthSlackWorldUnits       = 64.0,
    OcclusionOccluderMaxTriangles       = 4096,
    OcclusionOccludeeMinRadius          = 1,
    -- Submit occluders sorted near-to-far (MOC early-rejects occluded
    -- occluders). Clear win in sync mode; in async it trades the
    -- traverse/rasterize overlap for the ordering.
    OcclusionOccluderFrontToBack        = true,

    -- Threadpool. The three fields below get overridden by hardware
    -- tier (see applyTierDefaults) before mwse.loadConfig — first-run
    -- users on weaker CPUs get conservative defaults, saved msoc.json
    -- still wins over both.
    OcclusionAsyncOccluders             = true,
    OcclusionThreadpoolThreadCount      = 0,  -- 0 = auto-pick
    OcclusionThreadpoolBinsW            = 4,
    OcclusionThreadpoolBinsH            = 2,
    OcclusionTemporalCoherenceFrames    = 4,

    -- Mask resolution. Applied at launch, changed on restart: main.lua
    -- pushes this table across before calling msoc.install(), which latches
    -- it into kMsocWidth/Height for the session. (Before 1.6.1 install ran
    -- first and a saved value here never took effect at all.)
    -- Tier-overridden below (see applyTierDefaults). MOC requires
    -- width%8==0 and height%4==0 (asserted in C++); plugin clamps to
    -- [64..2048] × [32..1024].
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
    OcclusionLogCellCross               = false,

    -- Freeze-forensics watchdog. Applied at launch, changed on restart:
    -- the native side reads it at install, which since 1.6.1 runs after this
    -- table is pushed across. An MCM edit persists to msoc.json and takes
    -- effect on the next launch.
    OcclusionForensicsWatchdog          = false,

    -- Debug tints.
    DebugOcclusionTintOccluded          = false,
    DebugOcclusionTintTested            = false,
    DebugOcclusionTintOccluder          = false,

    -- Show the occlusion mask itself as a HUD overlay (see overlay.lua).
    DebugMaskOverlay                    = false,
}

-- This is the tier table. There is no other one.
--
-- Up to 1.4.0 C++ carried a second copy in Config.cpp, on the theory that a
-- "configure-only" consumer might use the DLL without these Lua files. No such
-- consumer existed — the plugin cannot install itself without main.lua — and
-- the duplicate was a standing hazard: whichever ran last won, silently, and
-- the two drifted (OcclusionSkipTerrainOccludees was tier-sensitive in C++ and
-- absent here). 1.6.1 deleted the C++ copy.
--
-- Order matters. mwse.loadConfig fills any field missing from the user's saved
-- JSON out of default_config, so the mutation below has to happen BEFORE that
-- load, and syncToNative pushes the result across afterwards — which is now
-- the only way these values ever reach C++.
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
        -- With async off the rasterization cost surfaces on the main
        -- thread, so take the coarsest terrain: Corners rasterizes 928
        -- triangles for a 9-cell view against Half's 3712, at about a fifth
        -- of the time, and occluded as much in the 2026-09-13 measurement.
        target.OcclusionAggregateTerrain   = 1
        target.OcclusionTerrainResolution  = 2
        -- Keep terrain leaves out of the occludee queue here. Letting them
        -- through saved ~1.9 ms of displayUs per frame in a dense Vivec
        -- exterior on mid/high, because a dense mask reads a useful fraction
        -- of them OCCLUDED — but on low tier the mask is a quarter the size
        -- and the extra TestRect work eats the classify budget instead.
        target.OcclusionSkipTerrainOccludees = true
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
        target.OcclusionSkipTerrainOccludees = false
    elseif tier == "high" then
        target.OcclusionAsyncOccluders     = true
        target.OcclusionThreadpoolBinsW    = 4
        target.OcclusionThreadpoolBinsH    = 2
        target.OcclusionMaskWidth          = 512
        target.OcclusionMaskHeight         = 256
        -- Budgets disabled — High-tier hardware doesn't need them.
        target.OcclusionRasterizeBudgetUs  = 0
        target.OcclusionClassifyBudgetUs   = 0
        target.OcclusionSkipTerrainOccludees = false
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

-- 1.6.1 removed the Horizon terrain mode (value 2). A saved 2 would show as
-- no selection in the dropdown; the native side already reads it as Raster.
if config.OcclusionAggregateTerrain == 2 then
    config.OcclusionAggregateTerrain = 1
end

-- _Claude_ Tier-default migration. mwse.loadConfig favours saved-JSON
-- values over default_config — the right semantics for keys the user
-- has explicitly tuned via MCM, but a footgun for keys whose ideal
-- value is a function of the plugin's heuristics, not the user's
-- choice. Without this migration, an upgrading user keeps stale tier-
-- insensitive defaults forever (e.g. OcclusionAsyncOccluders=true
-- saved from an old build silently overrides the new Low-tier `false`,
-- preserving the very perf footgun the tier system was meant to fix).
--
-- Strategy: stamp the plugin version into config.lastSeenVersion. When
-- it doesn't match the loaded plugin version, force-re-apply the tier-
-- sensitive subset from default_config (already tier-adjusted above)
-- and persist immediately. Subsequent MCM edits to these keys stick
-- until the next plugin version bump — same predictable semantics for
-- both first-run and upgrading users.
--
-- Trade-off: a user who manually overrode (e.g.) OcclusionAsyncOccluders
-- to gauge cost loses that override exactly once per plugin update.
-- They can re-set it in MCM and it persists until the next bump.
-- Acceptable: we'd rather have one forced re-evaluation than silently
-- pin every upgrading user to obsolete heuristics.
local kTierMigratedKeys = {
    "OcclusionAsyncOccluders",
    "OcclusionThreadpoolBinsW",
    "OcclusionThreadpoolBinsH",
    "OcclusionMaskWidth",
    "OcclusionMaskHeight",
    "OcclusionRasterizeBudgetUs",
    "OcclusionClassifyBudgetUs",
    -- 1.6.1: was tier-sensitive in C++ only, so a saved JSON written before
    -- this release holds the flat Lua default rather than the tier's value.
    "OcclusionSkipTerrainOccludees",
    -- 1.6.1: mask size now actually reaches the latch (install runs after
    -- configure), so a stale saved value would take effect for the first time.
    "OcclusionAggregateTerrain",
    -- 1.6.1: the low tier now picks its terrain resolution (Corners).
    "OcclusionTerrainResolution",
}

-- _Claude_ Renamed / removed keys. Each version bump that drops a
-- field appends it here so the migration can clean it out of the
-- user's saved JSON. Without this the JSON accumulates dead keys
-- forever — harmless, but clutters the file.
local kRetiredKeys = {
    "OcclusionDrainBudgetUs", -- 0.0.10: renamed to OcclusionClassifyBudgetUs
    -- 1.6.1: CPU light culling removed. The feature tested net-negative in
    -- 1.1.0 (~12% FPS regression in a Vivec canton at night) and was left in
    -- as a json-only knob; 1.6.1 drops the hook, the cache and both keys.
    "OcclusionCullLights",
    "OcclusionLightCullHysteresisFrames",
}

-- _Claude_ Default-value retunes. Keys whose shipped default changed in a
-- release and should reach upgrading users, whose saved JSON would otherwise
-- pin the old default forever (same footgun as the tier keys, different cause:
-- a tuning refinement rather than a hardware heuristic). Force-re-applied from
-- default_config on a version change. Same trade-off as kTierMigratedKeys: a
-- user who manually tuned one of these loses that override once per release,
-- then it sticks until the next bump.
--   1.3.0: enable the occludee box test, widen the exterior occluder max
--          radius (4096 -> 7040), lower depth slack (128 -> 64).
local kRetunedKeys = {
    "OcclusionOccludeeBoxTest",
    "OcclusionOccluderRadiusMaxExterior",
    "OcclusionDepthSlackWorldUnits",
}

local pluginVersion = mscPlugin and mscPlugin.version or "unknown"
if config.lastSeenVersion ~= pluginVersion then
    local oldVer = config.lastSeenVersion
    for _, k in ipairs(kTierMigratedKeys) do
        config[k] = default_config[k]
    end
    for _, k in ipairs(kRetunedKeys) do
        config[k] = default_config[k]
    end
    -- Remove keys that have been renamed or dropped in this or a
    -- prior version. Setting to nil drops the entry from the table,
    -- so the next mwse.saveConfig won't write it back to JSON.
    local removed = {}
    for _, k in ipairs(kRetiredKeys) do
        if config[k] ~= nil then
            config[k] = nil
            table.insert(removed, k)
        end
    end
    config.lastSeenVersion = pluginVersion
    mwse.saveConfig("msoc", config)
    mwse.log("[msoc] defaults migrated: lastSeen=%s -> %s; async=%s bins=%dx%d mask=%dx%d; boxTest=%s radiusMaxExt=%s depthSlack=%s; retired=[%s]",
        tostring(oldVer), tostring(pluginVersion),
        tostring(config.OcclusionAsyncOccluders),
        config.OcclusionThreadpoolBinsW, config.OcclusionThreadpoolBinsH,
        config.OcclusionMaskWidth, config.OcclusionMaskHeight,
        tostring(config.OcclusionOccludeeBoxTest),
        tostring(config.OcclusionOccluderRadiusMaxExterior),
        tostring(config.OcclusionDepthSlackWorldUnits),
        table.concat(removed, ","))
end

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
