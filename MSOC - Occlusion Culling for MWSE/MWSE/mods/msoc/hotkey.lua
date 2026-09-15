-- msoc: optional hotkey that flips the master switch (EnableMSOC) during
-- play, for before-and-after comparisons without opening the MCM. Unbound
-- by default; the key binder lives on the MCM General page. A toggle is
-- pushed to the native side the same frame and saved to msoc.json like an
-- MCM edit, so the MCM shows the current state the next time it opens.

local cfg = require("msoc.config")
-- include() is cached; same handle main.lua received.
local msoc = include("msoc")
local i18n = mwse.loadTranslations("msoc")

local function toggle()
    cfg.config.EnableMSOC = not cfg.config.EnableMSOC
    cfg.syncToNative(msoc)
    mwse.saveConfig(cfg.config.confPath, cfg.config)
    tes3.messageBox(i18n(cfg.config.EnableMSOC and "hotkey.enabled" or "hotkey.disabled"))
    mwse.log("[msoc] hotkey: EnableMSOC=%s", tostring(cfg.config.EnableMSOC))
end

--- @param e keyDownEventData
local function onKeyDown(e)
    local combo = cfg.config.ToggleHotkey
    if type(combo) ~= "table" or not combo.keyCode then return end
    -- Not while a menu, the console or a text field owns the keyboard, and
    -- not before a game is loaded.
    if tes3.menuMode() or not tes3.player then return end
    if not tes3.isKeyEqual({ actual = e, expected = combo }) then return end
    toggle()
end

event.register(tes3.event.keyDown, onKeyDown)

return { toggle = toggle }
