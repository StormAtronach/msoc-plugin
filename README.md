# MSOC plugin

CPU-side occlusion culling for Morrowind. Each frame the plugin software-
rasterizes a coarse depth mask from opaque near-scene geometry, then tests
every small NiTriShape leaf against it before the engine draws it. Leaves
that fall fully behind the mask are skipped — fewer GPU draw calls, fewer
vertex-shader invocations on geometry the player would never have seen.

This is the **near-scene** half of the work and runs standalone — no extra
mods required beyond MWSE.

A **distant-statics** half is also implemented: the plugin exposes its mask
to MGE-XE, which can then skip distant statics that fall fully behind it.
The MGE-XE side of this integration **is not yet released upstream** —
the work lives on a development branch and will land in a future MGE-XE
release. Until then the plugin's distant-statics path is dormant, and the
near-scene culling is what you get.

## Hardware notes

Most of the runtime cost lives in two places: software rasterization of
near-scene occluders into the mask, and per-occludee `TestRect` queries on
the drain. Both parallelize well on multi-core CPUs via Intel's MaskedSoftware
Occlusion Culling threadpool.

- **Modern multi-core CPUs (4+ cores, AVX2 or better):** the threadpool
  amortizes mask-build cost across workers, and the main-thread query path
  stays cheap. This is where the plugin earns its keep.
- **Older / dual-core CPUs (Sandy / Ivy Bridge era, SSE4.1 only):** the
  threadpool's wake / flush / suspend overhead per frame outweighs the
  parallelism win. The plugin auto-detects this on first launch and selects
  the **low-tier** preset, which disables the async path entirely and runs
  rasterization on the main thread. Results on this hardware vary — the
  plugin may be net-neutral or slightly negative depending on scene density.

You don't need to think about any of this — hardware tier is detected and
applied automatically. The MCM lets you override if needed.

## Requirements

- **Morrowind** (vanilla `Morrowind.exe`). OpenMW is **not** supported — the
  plugin links against MWSE which only targets the original engine.
- **MWSE** (recent build). The plugin loads via the MWSE Lua loader.
- **CPU** with at least SSE4.1. AVX2 / AVX-512 paths are auto-selected when
  available. Fallback path exists below SSE4.1 but isn't recommended.

Optional, for the additional distant-statics culling layer:

- **MGE-XE** with distant statics rendering enabled.
- **MGE-XE build with msoc-plugin integration.** This is not yet in any
  released MGE-XE; the integration lives on a development branch and will
  be upstreamed in a future MGE-XE release. With an unintegrated MGE-XE,
  distant statics are still rendered normally — the plugin simply can't
  cull them, and only the near-scene path is active.
- **Static instancing** (`MGE.ini → [Misc] → Use Static Instancing=True`)
  is strongly recommended once integration ships. The plugin culls *which*
  statics are submitted; instancing reduces *how many draw calls* the
  survivors produce.

## Performance characteristics

Per-frame plugin cost measured on a high-tier (AVX2, 8+ thread) reference
machine, comparing the four meaningful mode combinations in a representative
exterior scene. All values are total per-frame CPU time spent inside the
plugin's hot path — *cost*, not *gain*. The mask-build is the same regardless
of whether MGE-XE distant-statics integration is active, so these numbers
apply both to the standalone near-scene path and to the future integrated
build.

| Mode          | aggTerrainUs           | horizonBuildUs | rasterizeUs       | asyncFlushUs | Total visible |
|---------------|------------------------|----------------|-------------------|--------------|---------------|
| Sync Horizon  | 0                      | 157            | 430 (full sync)   | 0            | ~581 µs       |
| Sync Raster   | 177 (walk + sync)      | 0              | 651 (full sync)   | 0            | ~828 µs       |
| Async Horizon | 0                      | 179            | 6 (curtain only)  | 127          | ~306 µs       |
| Async Raster  | 13 (walk only)         | 0              | 0–2               | 117          | ~130 µs       |

The async modes shift mask-rasterization off the main thread onto worker
cores, so the visible main-thread cost collapses. **Async Raster** is the
cheapest combination and is the default on the mid- and high-tier hardware
presets. **Sync Horizon** wins among synchronous modes — its bounded-cost
silhouette curtain is cheaper to construct on the main thread than running
the full per-shape terrain rasterization synchronously — and is the default
on the low-tier (no-async) preset.

These numbers are the plugin's own CPU cost, not the time it saves
downstream. The user-visible win is fewer GPU draw calls / vertex-shader
invocations on culled near-scene leaves today, plus distant-statics culling
once MGE-XE integration ships. How that translates into FPS depends entirely
on whether your scene was CPU-draw-bound or GPU-bound to begin with.

## Installation

Drop the contents of the release archive into your Morrowind `Data Files`
directory:

```text
Data Files/MWSE/lib/msoc.dll
Data Files/MWSE/mods/msoc/main.lua
Data Files/MWSE/mods/msoc/config.lua
Data Files/MWSE/mods/msoc/mcm.lua
```

A mod manager works too — both halves (`MWSE/lib/msoc.dll` and
`MWSE/mods/msoc/`) need to land under `Data Files/`.

After install, launch Morrowind. The plugin probes hardware on first run and
writes a sensible config to `MWSE/config/msoc.json`. You can tweak via the MCM
afterwards.

## Configuration

Open the MCM (Mod Configuration Menu) → **MSOC**. The interesting knobs:

- **Master enable / interior / exterior toggles** — start with all on. Turn
  off interior culling if you see issues in cells with weird visibility
  rules.
- **Terrain occluder mode** — `Off / Raster / Horizon`. Default tracks your
  hardware tier (Raster on multi-core, Horizon on low). Read the in-MCM
  description if you want to tune; otherwise leave it.
- **Occlusion source** — `MSOC mask` (default) or `MGE depth pass`. Stick
  with `MSOC mask` unless you know what you're doing — the depth-pass
  alternative requires specific MGE-XE builds and is documented as
  experimental.
- **Async occluders** — controls whether mask-rasterization runs on the
  threadpool or the main thread. Hardware-tier default is correct for most
  users; flip only if you're benchmarking or debugging.
- **Light culling** — opt-in. Tests every NiLight against the mask and
  disables fully-occluded ones for the frame. Helps in dense interiors with
  many fixtures behind walls; adds a per-light test on the hot path.

The hardware-tier defaults table:

| Tier | CPU profile        | Async | Mask resolution | Defaults conservative? |
|------|--------------------|-------|-----------------|------------------------|
| low  | SSE4.1, ≤4 threads | off   | 256×128         | yes — phase budgets on |
| mid  | AVX2, 6-8 threads  | on    | 384×192         | yes — phase budgets on |
| high | AVX2+, 8+ threads  | on    | 512×256         | no — budgets disabled  |

If you don't know which tier you landed in, check the first few lines of
`MWSE.log` after launch — the plugin reports `hardwareTier=...` there.

## How it works

Each frame, while Morrowind's renderer traverses the scene graph:

1. **Rasterize occluders.** Large opaque NiTriShape leaves are submitted to
   Intel's MaskedSoftware Occlusion Culling rasterizer as occluders into a
   coarse depth mask (default 512×256 on high-tier hardware).
2. **Defer leaves.** Small NiTriShape leaves are queued for later instead of
   being tested against a partially-built mask (avoids same-frame ordering
   false-positives).
3. **Optional: aggregate terrain.** Either the merged near-scene terrain
   surface (`Raster`) or a 1D screen-space horizon curtain at the terrain
   silhouette's far depth (`Horizon`) gets rasterized into the same mask.
4. **Drain.** Every queued leaf gets `TestRect` against the now-complete
   mask. Verdicts: VISIBLE, OCCLUDED, VIEW_CULLED. OCCLUDED leaves skip
   `display()` entirely.
5. **External consumers query the mask.** The plugin exposes
   `mwse_testOcclusionSphere(Batch)` and a few snapshot accessors so other
   native code can use the same mask without rebuilding it. MGE-XE will use
   this once the integration is upstreamed (see Requirements above) to skip
   occluded distant statics in its instanced-draw setup. Until then the
   exports are present but no consumer is calling them.

Phase budgets and temporal coherence absorb worst-case spikes (cell loads,
sudden camera reveals) so the per-frame cost stays bounded.

## Compatibility

- **MGE-XE versions:** the plugin's distant-statics-culling path is dormant
  on every released MGE-XE build to date; the consumer-side code is on a
  development branch awaiting upstream merge. The plugin still loads cleanly
  and runs near-scene culling regardless of MGE-XE's version (or absence).
  When the integration ships, MGE-XE will dynamic-resolve the plugin's
  exports at load time and fall back cleanly on older plugin versions, so
  forward/backward compatibility will be a non-issue.
- **ENB / shader replacers:** unaffected. The plugin operates before GPU
  submission; ENB sees fewer draw calls but otherwise unchanged geometry.
- **Other MWSE mods:** the plugin patches the same engine path MWSE proper
  used to (`OcclusionCulling`). It assumes nothing else is hooking the same
  CullShow detour. If you have another mod doing CPU occlusion, disable one.
- **Save games:** no save-game state. Toggling the plugin on/off is reversible
  per-launch.

## Building from source

Win32 only — Morrowind is 32-bit and an x64 plugin won't load.

```bash
cd msoc-plugin
cmake --preset win32-release
cmake --build --preset win32-release
```

After build, `msoc.dll` lands in `test-mod/MWSE/lib/msoc.dll`.

If CMake can't find LuaJIT, set `MWSE_ROOT` to your MWSE checkout's inner
source dir (the one containing `deps/LuaJIT/`):

```bash
cmake -B build -A Win32 -S . -DMWSE_ROOT="C:/path/to/MWSE/MWSE"
```

You may need to build MWSE at least once first so `lua51.lib` exists.

## License

This project is licensed under the MIT License. See [`LICENSE`](LICENSE).

It bundles a modified copy of Intel's [Masked Software Occlusion Culling][msoc]
under Apache License 2.0; see [`deps/msoc/LICENSE-Apache-2.0.txt`](deps/msoc/LICENSE-Apache-2.0.txt)
for the full text and [`deps/msoc/NOTICE`](deps/msoc/NOTICE) for the list of
modifications.

## Credits

- **Intel** for [Masked Software Occlusion Culling][msoc] — the core
  rasterizer and threadpool this project builds on.
- **The MWSE team** for the engine-patching and Lua infrastructure.
- **The MGE-XE team** for distant-statics rendering and the integration
  surface this plugin plugs into.

[msoc]: https://github.com/GameTechDev/MaskedOcclusionCulling
