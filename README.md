# MSOC plugin

CPU-side occlusion culling for Morrowind. Each frame the plugin software-
rasterizes a coarse depth mask from opaque near-scene geometry, then tests
every small `NiTriBasedGeom` leaf (both `NiTriShape` and `NiTriStrips`)
against it before the engine draws it. Leaves that fall fully behind the
mask are skipped — fewer GPU draw calls, fewer vertex-shader invocations
on geometry the player would never have seen.

The plugin runs standalone — no extra mods required beyond MWSE, and
nothing else has to know it is there.

## What's new in 1.6.0

- **The MGE-XE integration surface is gone.** The plugin used to publish a
  double-buffered copy of its mask plus a set of `mwse_*` C exports so
  MGE-XE could cull distant statics against it. No released MGE-XE ever
  called them, and keeping the contract cost a second mask buffer, an
  external-occluder queue, observer callbacks on the drain, and a worker
  round trip on every async frame. All of it is removed; `msoc.dll` no
  longer exports an occlusion API.
- **Light culling removed.** It was opt-in, never showed a clear win, and
  needed a second engine detour of its own. One hook fewer on the hot path.
- **Live occlusion-mask overlay.** A debug toggle draws the mask the
  rasterizer is actually building in the top-right corner of the screen, so
  you can watch what gets occluded as you move. It replaces the old file
  dump as the way to inspect the mask; the dump is still reachable from Lua
  for offline comparison.

See [`CHANGELOG.md`](CHANGELOG.md) for the longer write-up.

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
- **Windows 8 or newer** (worker-thread parking uses `WaitOnAddress`).
- **CPU** with at least SSE4.1. The rasterizer uses AVX2 where available and
  SSE4.1 otherwise. (Intel's AVX-512 path is not built: it never selected at
  runtime, and 1.6.0 stopped compiling the dead translation unit.) A fallback
  below SSE4.1 exists but isn't recommended.
- No Visual C++ redistributable required — the runtime is linked in.

## Performance characteristics

Per-frame plugin cost measured on a high-tier (AVX2, 8+ thread) reference
machine, comparing the four meaningful mode combinations in a representative
exterior scene. All values are total per-frame CPU time spent inside the
plugin's hot path — *cost*, not *gain*.

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
downstream. The user-visible win is fewer GPU draw calls and vertex-shader
invocations on culled leaves. How that translates into FPS depends on
whether your scene was CPU-draw-bound or GPU-bound to begin with, and on how
much of the view is actually hidden — in a dense city the culler pays for
itself several times over, on an open plain it is a small net cost.

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

Baseline was ~28 ms (~35 FPS). The big +24 ms spike on age 1 is the engine
committing/rendering the freshly-loaded cell; MSOC does essentially no work on
that frame (zero classify steps, zero occluder rebuilds). **The hitch is an
engine cell-load cost, not an MSOC one.**

MSOC's own cross cost lands entirely on age 0 (~1.7 ms over baseline) and
breaks down as:

- **cache wipe ~0.9 ms:** releasing the outgoing cell's `NI::Pointer` pins.
  Because the caches are the last holders of the old cell's shapes by the time
  the wipe runs, each release that drops a refcount to zero fires that shape's
  engine destructor. This is the cell-unload teardown deferred onto the cross
  frame, not `malloc`/`free`.
- **terrain re-aggregation ~1.1 ms:** rebuilding the merged per-Land occluder
  for the new cell.
- **occluder world-vertex re-transform ~0.26 ms for 68k verts** (~4 ns/vert):
  fast, not a bottleneck.

(The three overlap rather than sum, since the age-0 total is only +1.7 ms.)
Takeaway: there is no worthwhile MSOC-side cell-cross optimization. Its
contribution is small, on a separate frame from the hitch, and dominated by
necessary NI teardown.

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
- **Async occluders** — controls whether mask-rasterization runs on the
  threadpool or the main thread. Hardware-tier default is correct for most
  users; flip only if you're benchmarking or debugging.
- **Show occlusion mask** (Debug page) — draws the live mask in the
  top-right corner. Bright regions are occluded depth; anything the
  rasterizer never wrote stays black. Pair it with **Tint occluders yellow**
  to see which meshes are feeding the mask.

The hardware-tier defaults table:

| Tier | CPU profile             | Async | Mask resolution | Defaults conservative? |
|------|-------------------------|-------|-----------------|------------------------|
| low  | no AVX2, or ≤4 threads | off   | 256×128         | yes — phase budgets on |
| mid  | AVX2, 5-8 threads       | on    | 384×192         | yes — phase budgets on |
| high | AVX2, >8 threads        | on    | 512×256         | no — budgets disabled  |

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

The mask lives and dies inside one frame: cleared at the top of the pass,
built during traversal, read on the drain, and handed to nobody.

Phase budgets and temporal coherence absorb worst-case spikes (cell loads,
sudden camera reveals) so the per-frame cost stays bounded.

## Compatibility

- **MGE-XE versions:** any of them, including none. Up to 1.4.0 the plugin
  published its mask so MGE-XE could cull distant statics against it; that
  contract was removed in 1.6.0 because no released MGE-XE ever used it.
  The two now share nothing but the frame, so the plugin neither requires a
  particular MGE-XE build nor cares whether one is installed.
- **Other MWSE mods:** the plugin detours Morrowind's `cullShow` — the
  per-frame scene-graph traversal that does frustum culling and dispatches
  `display()` calls. The plugin assumes nothing else is hooking the same
  detour. If you have another mod doing CPU occlusion or otherwise
  intercepting `cullShow`, disable one.
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
- **The MGE-XE team**, whose shadow-map debug overlay is the model for the
  occlusion-mask overlay here.

[msoc]: https://github.com/GameTechDev/MaskedOcclusionCulling
