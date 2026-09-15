-- msoc: occlusion-mask HUD overlay.
--
-- The native side mirrors the finished mask into an engine NiSourceTexture
-- once per frame (see src/MaskOverlay.cpp). This module hangs that texture on
-- an image element in the help layer, which is what makes the mask visible in
-- game. MGE-XE shows its shadow layers the same way, but it owns its D3D
-- device and can draw a quad directly; going through the engine's UI keeps the
-- plugin out of the renderer's state cache.
--
-- The help layer rather than MenuMulti: it draws above every other menu, it is
-- not torn down and rebuilt when the HUD is, and it does not have to fight
-- MenuMulti's layout for a corner. A debug view wants to stay visible and stay
-- put.
--
-- Nothing exists until the toggle is on. Asking the plugin for the texture is
-- also what arms the per-frame readback, so an off overlay costs nothing on
-- either side of the FFI boundary.

local cfg = require("msoc.config")
local msoc = include("msoc")

local MENU_ID = "msoc_maskOverlayMenu"
local IMAGE_ID = "msoc_maskOverlayImage"

-- createImage insists on a resolvable path, so the element is born with a
-- stock texture and we overwrite it immediately. menu_divider ships with the
-- game and is what MWSE's own createDivider uses.
local SEED_TEXTURE = "Textures\\menu_divider.tga"

-- Cap the drawn width so a 512-wide mask does not dominate a small screen.
local MAX_WIDTH = 384

local function findOverlayMenu()
    return tes3ui.findHelpLayerMenu(MENU_ID)
end

--- tes3ui.createHelpLayerMenu builds a fixed frame with bReplaceThisElement
--- set, so the element it hands back is not reliably the one
--- findHelpLayerMenu resolves to afterwards. Re-find so the create and reuse
--- paths always operate on the same element.
local function getOrCreateMenu()
    local menu = findOverlayMenu()
    if menu then return menu end
    tes3ui.createHelpLayerMenu({ id = MENU_ID })
    return findOverlayMenu()
end

local function destroyOverlay()
    local menu = findOverlayMenu()
    if menu then
        menu:destroy()
    end
end

--- Push the mask texture into the element's drawn material.
---
--- This has to wait for the element to own a scene node. MWSE's
--- Element::setTexture writes the NiTexturingProperty base map only
--- `if (sceneNode && !sceneNode->children.empty())` and silently does nothing
--- otherwise: it still sets the element field, so a read-back of
--- image.texture looks correct while the screen keeps showing the seed
--- texture. updateLayout() alone does not reliably build the scene node for a
--- help-layer menu that has not been drawn yet, so retry across frames until
--- it appears. MWSE exposes no way to reach the texturing property directly,
--- so going through setTexture at the right moment is the only route.
local function bindTexture(texture, attempts)
    local menu = findOverlayMenu()
    local image = menu and menu:findChild(IMAGE_ID)
    if not image then return end

    if image.sceneNode then
        image.texture = texture
        menu:updateLayout()
        if attempts > 0 then
            mwse.log("[msoc] overlay: texture bound after %d frame(s).", attempts)
        end
        return
    end

    -- Frames stop while a loading screen is up, so after 30 frame retries
    -- fall back to a real-time poll for up to another half minute.
    if attempts >= 30 then
        if attempts >= 90 then
            mwse.log("[msoc] overlay: element never gained a scene node; "
                .. "texture not bound. The frame will render empty.")
            return
        end
        timer.start({ type = timer.real, duration = 0.5,
                      callback = function() bindTexture(texture, attempts + 1) end })
        return
    end

    timer.frame.delayOneFrame(function() bindTexture(texture, attempts + 1) end)
end
--- Returns true once the overlay exists (or already existed), false when the
--- mask texture is not available yet so the caller can retry.
local function createOverlay()
    if not (msoc and msoc.maskOverlayTexture) then return false end

    -- 0 means the mask resources are not live: EnableMSOC is off, the
    -- allocation failed, or - right after a load - the first culled frame has
    -- not run yet. The caller retries for a while rather than reporting an
    -- error the user cannot act on.
    local address = msoc.maskOverlayTexture()
    if not address or address == 0 then return false end

    local texture = mwse.memory.convertTo.niObject(address)
    if not texture then
        mwse.log("[msoc] overlay: could not wrap the mask texture address.")
        return true
    end

    local menu = getOrCreateMenu()
    if not menu then
        mwse.log("[msoc] overlay: could not create the help-layer menu.")
        return true
    end
    local existing = menu:findChild(IMAGE_ID)
    if existing then
        -- Already there: re-assert the binding, since a load can rebuild the
        -- UI under the element and leave it showing the seed texture.
        bindTexture(texture, 0)
        return true
    end

    local w, h = 512, 256
    if msoc.maskResolution then
        w, h = msoc.maskResolution()
    end
    local scale = math.min(1.0, MAX_WIDTH / w)

    local image = menu:createImage({ id = IMAGE_ID, path = SEED_TEXTURE })
    image.scaleMode = true
    image.width = math.floor(w * scale)
    image.height = math.floor(h * scale)
    image.consumeMouseEvents = false

    menu.autoWidth = true
    menu.autoHeight = true
    menu.absolutePosAlignX = 1.0
    menu.absolutePosAlignY = 0.0
    menu.consumeMouseEvents = false
    menu:updateLayout()

    bindTexture(texture, 0)
    mwse.log("[msoc] overlay: created (%dx%d mask drawn at %dx%d).", w, h, image.width, image.height)
    return true
end

--- The mask texture only exists once the plugin has built a mask, which on a
--- fresh load is a few frames after the HUD comes up. If the toggle is on and
--- the texture is not there yet, poll for it for a while; the timer dies
--- with the toggle or as soon as the overlay exists. Without this a saved
--- DebugMaskOverlay=true did nothing at load and had to be toggled off and
--- on in the MCM.
local retryTimer = nil
local RETRY_SECONDS = 0.5
local RETRY_LIMIT = 60

local function stopRetry()
    if retryTimer then
        retryTimer:cancel()
        retryTimer = nil
    end
end

local function scheduleRetry(attempt)
    stopRetry()
    if attempt > RETRY_LIMIT then
        mwse.log("[msoc] overlay: mask texture never appeared; giving up until the next load or toggle.")
        return
    end
    retryTimer = timer.start({
        type = timer.real,
        duration = RETRY_SECONDS,
        callback = function()
            retryTimer = nil
            if not cfg.config.DebugMaskOverlay then return end
            if createOverlay() then
                mwse.log("[msoc] overlay: created after %d retr%s (mask not ready at HUD activation).", attempt, attempt == 1 and "y" or "ies")
            else
                scheduleRetry(attempt + 1)
            end
        end,
    })
end

--- Create or destroy the overlay to match the current config. Called from the
--- MCM toggle and re-asserted on HUD activation and on game load.
local function refresh()
    if cfg.config.DebugMaskOverlay then
        if not createOverlay() then
            scheduleRetry(1)
        end
    else
        stopRetry()
        destroyOverlay()
    end
end

-- The help layer outlives ordinary menu churn, but a game load rebuilds the UI
-- wholesale. MenuMulti activation is the cheap signal that the world is back;
-- refresh is idempotent, so re-asserting costs a lookup. `loaded` covers the
-- case where MenuMulti was already up before the mask existed.
event.register("uiActivated", refresh, { filter = "MenuMulti" })
event.register("loaded", refresh)

return {
    refresh = refresh,
}
