local cfg  = require("msoc.config")
-- include() is cached; same handle main.lua received.
local msoc = include("msoc")

-- Translations live in i18n/<locale>.lua. eng.lua is the source of truth and
-- mandatory; other locales are optional drop-ins. mwse.loadTranslations falls
-- back to the eng.lua key whenever a translated key is missing.
local i18n = mwse.loadTranslations("msoc")

local NEXUS_URL  = "https://www.nexusmods.com/morrowind/mods/58823"
local GITHUB_URL = "https://github.com/StormAtronach/msoc-plugin"

-- Runtime sync: whenever a control commits a change, push the whole
-- Lua config table across the FFI boundary so the native statics
-- (msoc::Configuration::Foo) match the edited Lua table the same
-- frame. Cheap — one lua_getfield per field per change.
local function applyChange()
    cfg.syncToNative(msoc)
end

--- Threadpool guard. Intel's CullingThreadpool crashes if ThreadCount
--- exceeds BinsW * BinsH. The MCM clamps to that product on every
--- relevant edit so the user can't escape the bound accidentally. If
--- they bypass the MCM and edit msoc.json directly, the native side
--- will still catch it at threadpool creation.
--- @type mwseMCMSlider|nil
local threadCountSlider = nil

local function captureThreadCountSlider(self)
    threadCountSlider = self
end

local function clampThreadCount()
    if cfg.config.OcclusionThreadpoolThreadCount == 0 then return end
    local maxThreads = cfg.config.OcclusionThreadpoolBinsW
                     * cfg.config.OcclusionThreadpoolBinsH
    if cfg.config.OcclusionThreadpoolThreadCount > maxThreads then
        if threadCountSlider then
            threadCountSlider:setVariableValue(maxThreads)
        else
            cfg.config.OcclusionThreadpoolThreadCount = maxThreads
        end
    end
end

--- The overlay toggle needs the Lua-side element created or destroyed on the
--- spot, not just the native flag pushed.
local function applyMaskOverlay()
    cfg.syncToNative(msoc)
    require("msoc.overlay").refresh()
end

local function applyChangeClamped()
    clampThreadCount()
    cfg.syncToNative(msoc)
end

----------------------------------------------------------------
-- Sidebar
----------------------------------------------------------------

--- Centre an info/hyperlink's text.
--- @param self mwseMCMInfo|mwseMCMHyperlink
local function centred(self)
    self.elements.info.justifyText = "center"
end

--- Centre and paint in the menu header colour: the sidebar title.
--- @param self mwseMCMInfo
local function heading(self)
    self.elements.info.justifyText = "center"
    self.elements.info.color = tes3ui.getPalette(tes3.palette.headerColor)
end

--- "Hardware tier: High (AVX2, 20 threads)", from the plugin's own probe.
local function hardwareLine()
    if not msoc or not msoc.hardwareTier then
        return i18n("sidebar.hardware.unknown")
    end
    local tier = tostring(msoc.hardwareTier)
    tier = tier:sub(1, 1):upper() .. tier:sub(2)
    return i18n("sidebar.hardware", {
        tier = tier,
        simd = tostring(msoc.simdLevel or "?"),
        threads = tostring(msoc.cpuThreads or "?"),
    })
end

--- Shared sidebar: version, detected hardware, a two-line summary, links.
--- Hovering any setting swaps this for that setting's description.
--- @param page mwseMCMSideBarPage
local function createSidebar(page)
    local sidebar = page.sidebar
    sidebar:createInfo({
        text = i18n("sidebar.title", { version = tostring(msoc and msoc.version or "?") }),
        postCreate = heading,
    })
    sidebar:createInfo({
        text = hardwareLine(),
        postCreate = centred,
    })
    sidebar:createInfo({ text = i18n("sidebar.body") })
    sidebar:createHyperlink({
        text = i18n("sidebar.link.nexus"),
        url = NEXUS_URL,
        postCreate = centred,
    })
    sidebar:createHyperlink({
        text = i18n("sidebar.link.github"),
        url = GITHUB_URL,
        postCreate = centred,
    })
end

--- A side-bar page with the shared sidebar already attached.
--- @param template mwseMCMTemplate
--- @param labelKey string
--- @return mwseMCMSideBarPage
local function createPage(template, labelKey)
    local page = template:createSideBarPage({
        label     = i18n(labelKey),
        showReset = true,
    }) --[[@as mwseMCMSideBarPage]]
    createSidebar(page)
    return page
end

----------------------------------------------------------------
-- Pages
----------------------------------------------------------------

local function registerModConfig()
    -- "MSOC" is the brand name and intentionally not localised. `name` is
    -- the entry in the mod list; `label` is the header above the pages.
    local template = mwse.mcm.createTemplate({
        name               = "MSOC",
        label              = i18n("template.label"),
        config             = cfg.config,
        defaultConfig      = cfg.default,
        showDefaultSetting = true,
    })
    template:register()
    template:saveOnClose(cfg.config.confPath, cfg.config)

    ----------------------------------------------------------------
    -- General: the switches, terrain, and the verdict cache.
    ----------------------------------------------------------------
    local general = createPage(template, "page.general")

    local culling = general:createCategory({ label = i18n("category.culling") })
    culling:createOnOffButton({
        label       = i18n("EnableMSOC.label"),
        description = i18n("EnableMSOC.description"),
        configKey   = "EnableMSOC",
        callback    = applyChange,
    })
    culling:createOnOffButton({
        label       = i18n("OcclusionEnableInterior.label"),
        description = i18n("OcclusionEnableInterior.description"),
        configKey   = "OcclusionEnableInterior",
        callback    = applyChange,
    })
    culling:createOnOffButton({
        label       = i18n("OcclusionEnableExterior.label"),
        description = i18n("OcclusionEnableExterior.description"),
        configKey   = "OcclusionEnableExterior",
        callback    = applyChange,
    })

    -- Off skips terrain occlusion entirely; Raster submits the merged
    -- subcell triangle mesh to MOC. The cost knob is the resolution
    -- dropdown below it. (A "Horizon" silhouette-curtain mode was removed
    -- in 1.6.0: it cost more than the raster and occluded less.)
    local terrain = general:createCategory({ label = i18n("category.terrain") })
    terrain:createDropdown({
        label       = i18n("OcclusionAggregateTerrain.label"),
        description = i18n("OcclusionAggregateTerrain.description"),
        options     = {
            { label = i18n("OcclusionAggregateTerrain.option.0"), value = 0 },
            { label = i18n("OcclusionAggregateTerrain.option.1"), value = 1 },
        },
        configKey   = "OcclusionAggregateTerrain",
        callback    = applyChange,
    })
    terrain:createDropdown({
        label       = i18n("OcclusionTerrainResolution.label"),
        description = i18n("OcclusionTerrainResolution.description"),
        options     = {
            { label = i18n("OcclusionTerrainResolution.option.0"), value = 0 },
            { label = i18n("OcclusionTerrainResolution.option.1"), value = 1 },
            { label = i18n("OcclusionTerrainResolution.option.2"), value = 2 },
        },
        configKey   = "OcclusionTerrainResolution",
        callback    = applyChange,
    })
    terrain:createOnOffButton({
        label       = i18n("OcclusionSkipTerrainOccludees.label"),
        description = i18n("OcclusionSkipTerrainOccludees.description"),
        configKey   = "OcclusionSkipTerrainOccludees",
        callback    = applyChange,
    })

    -- The "Cull occluded lights" toggle and its hysteresis slider were
    -- exposed in 1.0.0, removed from the MCM in 1.1.0 after the feature
    -- tested net-negative (~12% FPS regression), and removed outright in
    -- 1.6.0 along with the 0x6bb7d4 hook and the per-light cache.

    local cache = general:createCategory({ label = i18n("category.cache") })
    cache:createSlider({
        label       = i18n("OcclusionTemporalCoherenceFrames.label"),
        description = i18n("OcclusionTemporalCoherenceFrames.description"),
        min = 0, max = 10, step = 1, jump = 2,
        configKey   = "OcclusionTemporalCoherenceFrames",
        callback    = applyChange,
    })

    ----------------------------------------------------------------
    -- Occluders: what gets drawn into the mask. Split per scene type
    -- (interiors favour smaller occluders, exteriors skip clutter).
    ----------------------------------------------------------------
    local occluder = createPage(template, "page.occluder")

    local interiors = occluder:createCategory({ label = i18n("category.interior") })
    interiors:createSlider({
        label       = i18n("OcclusionOccluderRadiusMinInterior.label"),
        description = i18n("OcclusionOccluderRadiusMinInterior.description"),
        min = 0, max = 2048, step = 16, jump = 128,
        configKey   = "OcclusionOccluderRadiusMinInterior",
        callback    = applyChange,
    })
    interiors:createSlider({
        label       = i18n("OcclusionOccluderRadiusMaxInterior.label"),
        description = i18n("OcclusionOccluderRadiusMaxInterior.description"),
        min = 256, max = 16384, step = 128, jump = 1024,
        configKey   = "OcclusionOccluderRadiusMaxInterior",
        callback    = applyChange,
    })
    interiors:createSlider({
        label       = i18n("OcclusionOccluderMinDimensionInterior.label"),
        description = i18n("OcclusionOccluderMinDimensionInterior.description"),
        min = 0, max = 1024, step = 8, jump = 64,
        configKey   = "OcclusionOccluderMinDimensionInterior",
        callback    = applyChange,
    })
    interiors:createSlider({
        label       = i18n("OcclusionInsideOccluderMarginInterior.label"),
        description = i18n("OcclusionInsideOccluderMarginInterior.description"),
        min = 0, max = 512, step = 8, jump = 32,
        configKey   = "OcclusionInsideOccluderMarginInterior",
        callback    = applyChange,
    })

    local exteriors = occluder:createCategory({ label = i18n("category.exterior") })
    exteriors:createSlider({
        label       = i18n("OcclusionOccluderRadiusMinExterior.label"),
        description = i18n("OcclusionOccluderRadiusMinExterior.description"),
        min = 0, max = 2048, step = 16, jump = 128,
        configKey   = "OcclusionOccluderRadiusMinExterior",
        callback    = applyChange,
    })
    exteriors:createSlider({
        label       = i18n("OcclusionOccluderRadiusMaxExterior.label"),
        description = i18n("OcclusionOccluderRadiusMaxExterior.description"),
        min = 256, max = 16384, step = 128, jump = 1024,
        configKey   = "OcclusionOccluderRadiusMaxExterior",
        callback    = applyChange,
    })
    exteriors:createSlider({
        label       = i18n("OcclusionOccluderMinDimensionExterior.label"),
        description = i18n("OcclusionOccluderMinDimensionExterior.description"),
        min = 0, max = 1024, step = 8, jump = 64,
        configKey   = "OcclusionOccluderMinDimensionExterior",
        callback    = applyChange,
    })
    exteriors:createSlider({
        label       = i18n("OcclusionInsideOccluderMarginExterior.label"),
        description = i18n("OcclusionInsideOccluderMarginExterior.description"),
        min = 0, max = 512, step = 8, jump = 32,
        configKey   = "OcclusionInsideOccluderMarginExterior",
        callback    = applyChange,
    })

    -- Shared (cost, not scene-dependent).
    local shared = occluder:createCategory({ label = i18n("category.shared") })
    shared:createSlider({
        label       = i18n("OcclusionOccluderMaxTriangles.label"),
        description = i18n("OcclusionOccluderMaxTriangles.description"),
        min = 64, max = 16384, step = 64, jump = 512,
        configKey   = "OcclusionOccluderMaxTriangles",
        callback    = applyChange,
    })
    shared:createOnOffButton({
        label       = i18n("OcclusionOccluderFrontToBack.label"),
        description = i18n("OcclusionOccluderFrontToBack.description"),
        configKey   = "OcclusionOccluderFrontToBack",
        callback    = applyChange,
    })
    shared:createOnOffButton({
        label       = i18n("OcclusionOccluderCCWOnly.label"),
        description = i18n("OcclusionOccluderCCWOnly.description"),
        configKey   = "OcclusionOccluderCCWOnly",
        callback    = applyChange,
    })

    ----------------------------------------------------------------
    -- Occludees: how the mask is queried.
    ----------------------------------------------------------------
    local occludee = createPage(template, "page.occludee")

    local query = occludee:createCategory({ label = i18n("category.query") })
    query:createSlider({
        label       = i18n("OcclusionDepthSlackWorldUnits.label"),
        description = i18n("OcclusionDepthSlackWorldUnits.description"),
        min = 0, max = 1024, step = 8, jump = 32,
        configKey   = "OcclusionDepthSlackWorldUnits",
        callback    = applyChange,
    })
    query:createSlider({
        label       = i18n("OcclusionOccludeeMinRadius.label"),
        description = i18n("OcclusionOccludeeMinRadius.description"),
        min = 0, max = 256, step = 1, jump = 16,
        configKey   = "OcclusionOccludeeMinRadius",
        callback    = applyChange,
    })
    query:createOnOffButton({
        label       = i18n("OcclusionOccludeeBoxTest.label"),
        description = i18n("OcclusionOccludeeBoxTest.description"),
        configKey   = "OcclusionOccludeeBoxTest",
        callback    = applyChange,
    })

    ----------------------------------------------------------------
    -- Performance: where the rasterisation runs. The tier note at the top
    -- tells the user what was auto-picked, including the two knobs that
    -- live only in msoc.json (mask size, phase budgets).
    ----------------------------------------------------------------
    local performance = createPage(template, "page.performance")

    performance:createInfo({
        text = i18n("performance.info", {
            tier = tostring(msoc and msoc.hardwareTier or "?"),
            width = tostring(cfg.config.OcclusionMaskWidth),
            height = tostring(cfg.config.OcclusionMaskHeight),
            rasterBudget = tostring(cfg.config.OcclusionRasterizeBudgetUs),
            classifyBudget = tostring(cfg.config.OcclusionClassifyBudgetUs),
        }),
    })

    local async = performance:createCategory({ label = i18n("category.async") })
    async:createOnOffButton({
        label       = i18n("OcclusionAsyncOccluders.label"),
        description = i18n("OcclusionAsyncOccluders.description"),
        configKey   = "OcclusionAsyncOccluders",
        callback    = applyChange,
    })
    async:createSlider({
        label       = i18n("OcclusionThreadpoolThreadCount.label"),
        description = i18n("OcclusionThreadpoolThreadCount.description"),
        min = 0, max = 16, step = 1, jump = 2,
        configKey   = "OcclusionThreadpoolThreadCount",
        postCreate  = captureThreadCountSlider,
        callback    = applyChangeClamped,
    })
    async:createSlider({
        label       = i18n("OcclusionThreadpoolBinsW.label"),
        description = i18n("OcclusionThreadpoolBinsW.description"),
        min = 1, max = 8, step = 1, jump = 2,
        configKey   = "OcclusionThreadpoolBinsW",
        callback    = applyChangeClamped,
    })
    async:createSlider({
        label       = i18n("OcclusionThreadpoolBinsH.label"),
        description = i18n("OcclusionThreadpoolBinsH.description"),
        min = 1, max = 8, step = 1, jump = 2,
        configKey   = "OcclusionThreadpoolBinsH",
        callback    = applyChangeClamped,
    })

    ----------------------------------------------------------------
    -- Debug: overlay, tints, logging, the freeze watchdog.
    ----------------------------------------------------------------
    local debugPage = createPage(template, "page.debug")

    debugPage:createInfo({ text = i18n("debug.info") })

    -- The mask overlay is not a tint: it draws the depth mask itself in the
    -- corner of the HUD rather than recolouring scene geometry, so it gets
    -- its own category above the tints.
    local overlay = debugPage:createCategory({ label = i18n("category.overlay") })
    overlay:createOnOffButton({
        label       = i18n("DebugMaskOverlay.label"),
        description = i18n("DebugMaskOverlay.description"),
        configKey   = "DebugMaskOverlay",
        callback    = applyMaskOverlay,
    })

    local tinting = debugPage:createCategory({ label = i18n("category.tinting") })
    tinting:createOnOffButton({
        label       = i18n("DebugOcclusionTintOccluder.label"),
        description = i18n("DebugOcclusionTintOccluder.description"),
        configKey   = "DebugOcclusionTintOccluder",
        callback    = applyChange,
    })
    tinting:createOnOffButton({
        label       = i18n("DebugOcclusionTintOccluded.label"),
        description = i18n("DebugOcclusionTintOccluded.description"),
        configKey   = "DebugOcclusionTintOccluded",
        callback    = applyChange,
    })
    tinting:createOnOffButton({
        label       = i18n("DebugOcclusionTintTested.label"),
        description = i18n("DebugOcclusionTintTested.description"),
        configKey   = "DebugOcclusionTintTested",
        callback    = applyChange,
    })

    local logging = debugPage:createCategory({ label = i18n("category.logging") })
    logging:createOnOffButton({
        label       = i18n("OcclusionLogPerFrame.label"),
        description = i18n("OcclusionLogPerFrame.description"),
        configKey   = "OcclusionLogPerFrame",
        callback    = applyChange,
    })
    logging:createOnOffButton({
        label       = i18n("OcclusionLogAggregate.label"),
        description = i18n("OcclusionLogAggregate.description"),
        configKey   = "OcclusionLogAggregate",
        callback    = applyChange,
    })
    logging:createOnOffButton({
        label       = i18n("OcclusionLogCellCross.label"),
        description = i18n("OcclusionLogCellCross.description"),
        configKey   = "OcclusionLogCellCross",
        callback    = applyChange,
    })

    -- Read once at install, which happens after main.lua pushes msoc.json,
    -- so the saved value is what starts up. A change made here takes effect
    -- on the next launch; restartRequired makes the MCM say so on change.
    local watchdog = debugPage:createCategory({ label = i18n("category.watchdog") })
    watchdog:createOnOffButton({
        label                  = i18n("OcclusionForensicsWatchdog.label"),
        description            = i18n("OcclusionForensicsWatchdog.description"),
        configKey              = "OcclusionForensicsWatchdog",
        restartRequired        = true,
        restartRequiredMessage = i18n("OcclusionForensicsWatchdog.restart"),
        callback               = applyChange,
    })
end

event.register("modConfigReady", registerModConfig)
