-- msoc: native plugin loader + MCM bootstrap.
--
-- Load order is load-bearing. The native side latches a few knobs once and
-- never re-reads them (mask resolution, forensics watchdog), so msoc.json
-- has to reach C++ BEFORE the patches install:
--
--   1. include("msoc")     loads msoc.dll and runs luaopen_msoc, which
--                          probes the MOC link and classifies the CPU tier.
--   2. cfg.syncToNative()  pushes msoc.json into the native statics.
--   3. msoc.install()      installs the engine hooks and latches the
--                          restart-only knobs from those statics.
--   4. require("msoc.mcm") registers the MCM page.
--
-- Before 1.6.1 the DLL called installPatches() from luaopen_msoc itself, at
-- step 1, so the latch only ever saw the C++ compile-time defaults and a
-- user-set OcclusionMaskWidth/Height in msoc.json was silently ignored.
-- Step 3 is what fixes that.

local msoc = include("msoc")

if not msoc then
    mwse.log("[msoc] msoc.dll not loaded. If you're using a mod manager, "
        .. "check that .dll files weren't filtered out of the install.")
    return
end

mwse.log("[msoc] plugin loaded, version=%s, mocLink=%s",
    tostring(msoc.version), tostring(msoc.mocLink))

local cfg = require("msoc.config")
cfg.syncToNative(msoc)

mwse.log("[msoc] config synced from msoc.json: EnableMSOC=%s, ExteriorCull=%s",
    tostring(cfg.config.EnableMSOC),
    tostring(cfg.config.OcclusionEnableExterior))

-- Install the engine hooks now that the native statics hold the user's
-- config. A failure here is not fatal to the rest of this file: the MCM
-- still registers so the user can inspect and toggle settings, and MSOC.log
-- carries the detail.
if type(msoc.install) == "function" then
    local ok, err = pcall(msoc.install)
    if ok then
        mwse.log("[msoc] engine hooks installed; requested mask %sx%s "
            .. "(MSOC.log reports the size actually latched).",
            tostring(cfg.config.OcclusionMaskWidth),
            tostring(cfg.config.OcclusionMaskHeight))
    else
        mwse.log("[msoc] msoc.install() failed: %s -- occlusion culling is "
            .. "inactive this session. See MSOC.log.", tostring(err))
    end
else
    -- Pre-1.6.1 msoc.dll: it installed its hooks during include() above,
    -- before this file could push msoc.json across. Culling still works,
    -- but the restart-only knobs (mask resolution, forensics watchdog) hold
    -- the DLL's own tier defaults rather than the saved ones.
    mwse.log("[msoc] msoc.dll predates msoc.install(); hooks self-installed "
        .. "at load, so a saved mask resolution is not applied. "
        .. "Update msoc.dll to match this mod's Lua files.")
end

-- mcm.lua registers its own modConfigReady handler; require at top
-- level so that handler is installed before the event fires.
require("msoc.mcm")
