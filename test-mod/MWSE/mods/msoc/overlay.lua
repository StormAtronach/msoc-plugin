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

    if attempts >= 30 then
        mwse.log("[msoc] overlay: element never gained a scene node; "
            .. "texture not bound. The frame will render empty.")
        return
    end

    timer.frame.delayOneFrame(function() bindTexture(texture, attempts + 1) end)
end
local function createOverlay()
    if not (msoc and msoc.maskOverlayTexture) then return end

    -- 0 means the mask resources are not live: EnableMSOC is off, or the
    -- allocation failed. Try again on the next activation rather than
    -- reporting an error the user cannot act on.
    local address = msoc.maskOverlayTexture()
    if not address or address == 0 then return end

    local texture = mwse.memory.convertTo.niObject(address)
    if not texture then
        mwse.log("[msoc] overlay: could not wrap the mask texture address.")
        return
    end

    local menu = getOrCreateMenu()
    if not menu then
        mwse.log("[msoc] overlay: could not create the help-layer menu.")
        return
    end
    if menu:findChild(IMAGE_ID) then return end

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
end

--- Create or destroy the overlay to match the current config. Called from the
--- MCM toggle and re-asserted on HUD activation.
local function refresh()
    if cfg.config.DebugMaskOverlay then
        createOverlay()
    else
        destroyOverlay()
    end
end

-- The help layer outlives ordinary menu churn, but a game load rebuilds the UI
-- wholesale. MenuMulti activation is the cheap signal that the world is back;
-- refresh is idempotent, so re-asserting costs a lookup.
event.register("uiActivated", refresh, { filter = "MenuMulti" })

return {
    refresh = refresh,
}
