# MSOC plugin

CPU-side occlusion culling for Morrowind. Each frame the plugin software-
rasterizes a coarse depth mask from opaque near-scene geometry, then tests
every small `NiTriBasedGeom` leaf (both `NiTriShape` and `NiTriStrips`)
against it before the engine draws it. Leaves that fall fully behind the
mask are skipped. The GPU gets fewer draw calls and fewer vertex-shader
invocations on geometry the player would never have seen.

The plugin runs standalone. It needs nothing beyond MWSE, and nothing else
has to know it is there.

## What's new in 1.6.0

- **The MGE-XE integration is gone.** The plugin used to publish a
  double-buffered copy of its mask plus a set of `mwse_*` C exports so
  MGE-XE could cull distant statics against it. No released MGE-XE ever
  called them, and keeping the contract cost a second mask buffer, an
  external-occluder queue, observer callbacks on the drain, and a worker
  round trip on every async frame. All of it is removed; `msoc.dll` no
  longer exports an occlusion API.
- **Light culling removed.** It was opt-in, never showed a clear win, and
  needed a second engine detour of its own. One hook fewer on the hot path.
- **Live occlusion-mask overlay.** A debug toggle draws the mask the
  rasterizer is building in the top-right corner of the screen, so
  you can watch what gets occluded as you move. It replaces the old file
  dump as the way to inspect the mask; the dump is still reachable from Lua
  for offline comparison.

See [`CHANGELOG.md`](CHANGELOG.md) for the longer write-up.

## Hardware notes

Most of the runtime cost lives in two places: software rasterization of
near-scene occluders into the mask, and per-occludee `TestRect` queries on
the drain. Both parallelize well on multi-core CPUs via Intel's Masked Software
Occlusion Culling threadpool.

- **Modern multi-core CPUs (4+ cores, AVX2 or better).** The threadpool
  amortizes the mask-build cost across workers and the main-thread query path
  stays cheap. This is where the plugin earns its keep.
- **Older or dual-core CPUs (Sandy / Ivy Bridge era, SSE4.1 only).** The
  threadpool's wake / flush / suspend overhead per frame outweighs the
  parallelism win. The plugin detects this on first launch and selects the
  low-tier preset, which runs rasterization on the main thread at a lower
  mask and terrain resolution. Results on this hardware vary with scene
  density; the site table below has the numbers.

You don't need to think about any of this. The plugin detects the tier and
applies it, and the MCM lets you override it.

## Requirements

- **Morrowind** (vanilla `Morrowind.exe`). OpenMW is not supported. The
  plugin links against MWSE, which only targets the original engine.
- **MWSE** (recent build). The plugin loads via the MWSE Lua loader.
- **Windows 8 or newer** (worker-thread parking uses `WaitOnAddress`).
- **CPU** with at least SSE4.1. The rasterizer uses AVX2 where available and
  SSE4.1 otherwise. Intel's AVX-512 path is not built; it never selected at
  runtime, and 1.6.0 stopped compiling the dead translation unit. A fallback
  below SSE4.1 exists but isn't recommended.
- No Visual C++ redistributable is required. The runtime is linked in.

## Performance characteristics

Terrain has two modes, `Off` and `Raster`, and the raster's resolution
(`Full` / `Half` / `Corners`) is the cost knob. Main-thread microseconds per
frame for the terrain pass alone, measured 2026-09-13 on a 9-cell exterior
(Azura's Coast) on a high-tier machine:

| Terrain resolution | triangles | sync (main thread) | async (dispatch + flush) |
|--------------------|-----------|--------------------|--------------------------|
| Full               | 14848     | 282 us             | ~36 us                   |
| Half               | 3712      | 114 us             | ~31 us                   |
| Corners            | 928       | 54 us              | ~27 us                   |

All three occluded the same number of objects at that site. The async modes
shift the rasterization onto worker cores, so the visible main-thread cost
collapses; **Async Raster at Half** is the default on the mid- and high-tier
presets. The low-tier (no-async) preset runs **Raster at Corners**. A 1D
"Horizon" silhouette-curtain mode existed until 1.6.0. Measured properly it
cost four to five times the synchronous raster at equal resolution and
occluded less, so 1.6.0 removed it.

These numbers are the plugin's own CPU cost, not the time it saves
downstream. The user-visible win is fewer GPU draw calls and vertex-shader
invocations on culled leaves.

### Where it helps, and where it does not

The cost above is close to constant. What the plugin returns is not: it
depends entirely on how much of the view is hidden behind something else. So
the same build is a large win in one place and a small loss in another.

Measured for 1.6.0 across four exterior sites, culler off against culler on:

| site | cull rate | result |
|------|-----------|--------|
| Old Ebonheart | 90% | **−22%** (108 to 139 fps) |
| Narsis | 77% | **−14%** (74 to 87 fps) |
| Balmora | 51% | −4% (210 to 218 fps) |
| Vivec, Foreign Quarter | 11% | +1% (232 to 230 fps) |

Read the pattern before you judge a number. The plugin wins in dense
architecture and roughly breaks even on open ground, and the places where it
breaks even are the places already running at 200 fps. It has never been a
large loss on the hardware it most helps. On a simulated four-core machine
the same two dense cities gain 2.3 to 2.6 ms a frame, more than on a fast
CPU, because the drawing it avoids costs more there.

The one case that is still a real regression is a sparse exterior on a weak
CPU. Vivec on four cores measures +1.07 ms, a 16% loss, on a frame that was
already running at 151 fps. If you spend your time in open terrain on older
hardware and you can see the difference, turn the master switch off. The
plugin has no way to work that out for itself. Three attempts to make it
decide automatically are written up in the moreFPS notes; none of them was
reliable enough to ship.

### Cell-cross cost

Enabling **Log cell-cross spikes** (`OcclusionLogCellCross`) emits the full
stats line on each cell change and the next several frames, tagged
`cellCross=<age>` (0 = the cross frame). Profiled in Narsis (Tamriel
Rebuilt, a dense exterior city) over 17 crossings, the cross "hitch" is two
frames with different owners:

| Frame             | frame time | vs settled | owner  |
| ----------------- | ---------- | ---------- | ------ |
| age 0 (the cross) | ~30 ms     | +1.7 ms    | MSOC   |
| age 1 (next)      | ~53 ms     | +24 ms     | engine |
| age 2+ (settled)  | ~28 ms     | settled    | n/a    |

Baseline was ~28 ms (~35 FPS). The +24 ms spike on age 1 is the engine
committing and rendering the freshly loaded cell. MSOC does no work on that
frame, zero classify steps and zero occluder rebuilds. The hitch is an engine
cell-load cost, not an MSOC one.

MSOC's own cross cost lands entirely on age 0 (~1.7 ms over baseline) and
breaks down as:

- **Cache wipe, ~0.9 ms.** Releasing the outgoing cell's `NI::Pointer` pins.
  Because the caches are the last holders of the old cell's shapes by the time
  the wipe runs, each release that drops a refcount to zero fires that shape's
  engine destructor. This is the cell-unload teardown deferred onto the cross
  frame, not `malloc`/`free`.
- **Terrain re-aggregation, ~1.1 ms.** Rebuilding the merged per-Land
  occluder for the new cell.
- **Occluder world-vertex re-transform, ~0.26 ms for 68k verts.** About
  4 ns per vertex; fast, not a bottleneck.

The three overlap rather than sum, since the age-0 total is only +1.7 ms.
There is no worthwhile MSOC-side cell-cross optimization. Its contribution is
small, on a separate frame from the hitch, and dominated by necessary NI
teardown.

## Installation

The release archive is the repository's `MSOC - Occlusion Culling for MWSE`
folder, zipped. Drop its contents into your Morrowind `Data Files` directory:

```text
Data Files/MWSE/lib/msoc.dll
Data Files/MWSE/mods/msoc/main.lua
Data Files/MWSE/mods/msoc/config.lua
Data Files/MWSE/mods/msoc/mcm.lua
Data Files/MWSE/mods/msoc/overlay.lua
Data Files/MWSE/mods/msoc/i18n/eng.lua
```

The `LICENSE`, `LICENSE-Apache-2.0.txt` and `NOTICE` files do nothing in
game. A mod manager works too. Both halves, `MWSE/lib/msoc.dll` and
`MWSE/mods/msoc/`, need to land under `Data Files/`.

After install, launch Morrowind. The plugin probes hardware on first run and
writes a sensible config to `MWSE/config/msoc.json`. You can tweak via the MCM
afterwards.

## Configuration

Open the MCM (Mod Configuration Menu) and pick MSOC. The interesting knobs:

- **Master enable and the interior / exterior toggles.** Start with all on.
  Turn interior culling off if you see issues in cells with weird visibility
  rules.
- **Toggle hotkey.** Optional key that flips the master switch during play,
  for before-and-after comparisons. Unbound by default. Modifier
  combinations work, and the key is ignored while a menu or the console is
  open.
- **Terrain occluder mode.** `Off / Raster`, plus a resolution dropdown,
  `Full / Half / Corners`. The default follows your hardware tier, Half on
  multi-core and Corners on low. Read the in-MCM description if you want to
  tune; otherwise leave it.
- **Async occluders.** Whether mask rasterization runs on the threadpool or
  the main thread. The hardware-tier default is right for most users; flip it
  only to benchmark or debug.
- **Show occlusion mask** (Debug page). Draws the live mask in the top-right
  corner. Bright regions are occluder depth, nearest brightest. Anything the
  rasterizer never wrote stays black. Pair it with the yellow occluder tint
  to see which meshes feed the mask.

The hardware-tier defaults table:

| Tier | CPU profile             | Async | Mask resolution | Phase budgets          |
|------|-------------------------|-------|-----------------|------------------------|
| low  | no AVX2, or ≤4 threads | off   | 256×128         | on                     |
| mid  | AVX2, 5-8 threads       | on    | 384×192         | on                     |
| high | AVX2, >8 threads        | on    | 512×256         | off                    |

If you don't know which tier you landed in, check the first few lines of
`MWSE.log` after launch. The plugin reports `hardwareTier=...` there.

## How it works

Each frame, while Morrowind's renderer traverses the scene graph:

1. **Rasterize occluders.** The plugin submits large opaque mesh leaves to
   Intel's Masked Software Occlusion Culling rasterizer as occluders into a
   coarse depth mask, 512×256 by default on high-tier hardware. Alpha,
   stencil and depth-test-off shapes never occlude.
2. **Defer leaves.** It queues small leaves for later instead of testing
   them against a half-built mask, which avoids same-frame ordering false
   positives.
3. **Terrain.** The near-scene terrain surface, the nine active cells at the
   chosen resolution, joins the queue with the meshes, and the plugin submits
   everything nearest first.
4. **Drain.** The plugin tests every queued leaf against the now-complete
   mask. Verdicts: VISIBLE, OCCLUDED, VIEW_CULLED. OCCLUDED leaves skip
   `display()` entirely. Leaves the renderer draws with the depth test off,
   such as detection-spell markers, are never tested.

The mask lives and dies inside one frame: cleared at the top of the pass,
built during traversal, read on the drain, and handed to nobody.

Per-phase time budgets absorb worst-case spikes such as cell loads and sudden
camera reveals, so the per-frame cost stays bounded.

## Compatibility

- **MGE-XE.** Any version, including none. Up to 1.4.0 the plugin published
  its mask so MGE-XE could cull distant statics against it; 1.6.0 removed
  that contract because no released MGE-XE ever used it. The two now share
  nothing but the frame.
- **Other MWSE mods.** The plugin detours Morrowind's `cullShow`, the
  per-frame scene-graph traversal that does frustum culling and dispatches
  `display()` calls, and assumes nothing else hooks it. If another mod does
  CPU occlusion or intercepts `cullShow`, disable one of them.
- **Save games.** No save-game state. Toggling the plugin is reversible per
  launch.

## Building from source

Win32 only. Morrowind is 32-bit and an x64 plugin won't load.

```bash
cd msoc-plugin
cmake --preset win32-release
cmake --build --preset win32-release
```

After build, `msoc.dll` lands in `MSOC - Occlusion Culling for MWSE/MWSE/lib/`,
which is the folder that ships. The PDB goes to `build/<preset>/pdb/`.

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

- **Intel** for [Masked Software Occlusion Culling][msoc], the core
  rasterizer and threadpool this project builds on.
- **The MWSE team** for the engine-patching and Lua infrastructure.
- **The MGE-XE team**, whose shadow-map debug overlay is the model for the
  occlusion-mask overlay here.

[msoc]: https://github.com/GameTechDev/MaskedOcclusionCulling
