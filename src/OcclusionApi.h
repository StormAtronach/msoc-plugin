#pragma once

// Plugin entry point.
//
// Until 1.6.0 this header also carried the MGE-XE consumer ABI: the mask
// query API, external occluder injection, the light-observed and
// visible-geom callbacks, and the mwse_* C exports that fronted them. No
// released MGE-XE ever resolved those exports, so the whole surface was
// removed along with the snapshot double buffer that backed it. msoc.dll
// now exports luaopen_msoc only.

namespace msoc::patch::occlusion {

// Install hooks for DX8 Masked Software Occlusion Culling. Hooks
// always install - EnableMSOC is a runtime gate re-checked every
// frame inside the detour. When the gate is off the detour falls
// through to vanilla cullShowBody at zero overhead. Resources
// (g_msoc + ~57MB ring-buffer threadpool) are allocated eagerly at
// startup if EnableMSOC begins true, lazily on first MCM toggle-on
// otherwise, and torn down on toggle-off.
void installPatches();

}  // namespace msoc::patch::occlusion
