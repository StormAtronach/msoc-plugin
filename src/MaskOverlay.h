#pragma once

// Debug occlusion-mask overlay, plus the PFM file dump that shares its buffer.
//
// The overlay mirrors the completed mask into an engine NiSourceTexture once
// per frame; the Lua side hangs that texture on a HUD image element, which is
// what makes the mask visible in-game. MGE-XE surfaces its shadow layers the
// same way (renderShadowDebug paints them into a screen corner), but it owns
// its D3D9 device and can simply draw a quad. The plugin does not, so it goes
// through the engine's own texture + UI path and never mutates render state
// behind the NiDX8 renderer's state cache.
//
// Both entry points read the LIVE buffer (g_msoc), so they must be called
// after the drain, when the frame's mask is complete.
//
// Replaces the mwse_dumpOcclusionMask export removed in 1.6.0; the dump is now
// reachable from Lua as msoc.dumpMask(path).

namespace msoc::patch::occlusion {

// Get-or-create the overlay texture, returned as an opaque NI::SourceTexture*.
// Null when the mask resources are not live yet or the engine refused the
// allocation. Cheap after the first call. Creating the texture is also what
// arms updateMaskOverlay.
void* maskOverlayTexture();

// Refresh the overlay texture from the completed mask. No-op until Lua has
// asked for the texture, so the readback costs nothing while the overlay is
// switched off.
void updateMaskOverlay();

// Tone-mapped PFM snapshot of the completed mask, for offline inspection.
bool dumpMaskToPfm(const char* path);

// Mask resolution as actually latched at install (rounded and clamped from
// the configured value), so the Lua side can size the overlay element to the
// texture instead of guessing from msoc.json.
void maskResolution(int* outWidth, int* outHeight);

}  // namespace msoc::patch::occlusion
