# MSOC plugin architecture

CPU-side occlusion culling for Morrowind. Each frame the plugin software-
rasterizes a coarse depth mask from near-scene opaque geometry (via Intel's
Masked Software Occlusion Culling), tests every small `NiTriBasedGeom` leaf
against it, and skips `display()` on leaves that fall fully behind the mask.
The mask is private to the frame that builds it. See [`README.md`](README.md)
for the user-facing story; this document is the code map.

## Layering

The source is organized as a dependency DAG: every module depends only on
modules below it. Nothing below the core depends back up into it.

```
  Entry point       plugin.cpp (luaopen_msoc, the Lua table, install)
                    OcclusionApi.h (installPatches declaration)
  ----------------------------------------------------------------------------
  Core orchestrator OcclusionPass.cpp
                      - CullShow detour + frame lifecycle
                      - scene-graph traversal (cullShowBody)
                      - occluder rasterization (rasterizeTriShape)   [hot]
                      - deferred-display drain pipeline              [hot]
                      - install, forensics read-accessor
  ----------------------------------------------------------------------------
  Subsystems        TerrainAggregation near-Land merge (Raster + Horizon)
  (engine-coupled)  MaskResources     MOC buffer + threadpool lifecycle
                    DiagnosticsLog    per-frame stats line (cold path)
                    MaskOverlay       live mask readback -> NI texture (cold)
                    DebugTint         debug recolor overlay
  ----------------------------------------------------------------------------
  Leaf helpers      LiveQuery         live sphere-vs-mask query
  (engine-coupled)  OccluderClassify  ancestor alpha/stencil classification
  ----------------------------------------------------------------------------
  State owners      FrameConfig   per-frame Configuration snapshot (g_frame)
  (data structs,    FrameStats    per-frame work counters/timers  (g_stats)
   one extern       BudgetState   two-layer phase budget          (g_budget)
   instance each)   FrameDiag     per-frame/session diagnostics   (g_diag)
                    OcclusionCaches  the five per-cell caches      (g_caches)
  ----------------------------------------------------------------------------
  Pure leaves       ClipMath      projection / matrix math
  (engine-free,     Profiling     EMA + budget skip/clip decisions
   unit-tested)     HardwareTier  CPU tier classifier
                    HorizonOccluder 1D silhouette curtain builder
```

### Why the leaf layer exists
`OccluderClassify` is not a subsystem in its own right - it is a helper shared
by *both* the core and a subsystem. `classifyOccluderProperties` is called by
the core rasterizer and by `TerrainAggregation`; keeping it in the core would
make `TerrainAggregation` depend back into the core (a cycle), so it lives in a
leaf TU below both. `LiveQuery` is there for the same reason historically: its
second caller was `LightCulling`, removed in 1.6.0. It is now a leaf with one
caller, the core drain, and folding it back into the core is a reasonable
future tidy.

## The shared-state seam: `OcclusionInternal.h`

The plugin's private internal contract (distinct from the public ABI in
`OcclusionApi.h`). It is included by every patch TU and provides two
things:

1. **Shared state** - `extern` declarations of the single global instances
   defined in `OcclusionPass.cpp`: the state owners (`g_frame`, `g_stats`,
   `g_budget`, `g_diag`, `g_caches`), the MOC resources (`g_msoc`,
   `g_threadpool`), the live projection (`g_worldToClip`, `g_ndcRadius*`,
   `g_wGradMag`), and assorted frame flags.
2. **The cross-TU function contract** - declarations of the functions a TU
   exposes to the others (e.g. `rasterizeAggregateTerrain`, `createMSOCResources`,
   `emitPerFrameStatsLine`, `testSphereVisible`, `updateMaskOverlay`). Every
   one of these is *defined in a subsystem/leaf TU* - the core only calls them.

Hot-path helpers (`projectWorld`, `ScopedUsAccumulator`, the camera-plane
accessors) are header-inline so they still inline across TU boundaries.

### State owners
The decomposition replaced ~130 loose file-static globals with owner structs,
each a single `extern` instance reached through the seam. Grouping rationale:
- **FrameConfig** - the hot-path slice of `Configuration`, snapshotted once per
  top-level frame (`snapshot(isInterior)`) so inner loops stay branch-free.
- **FrameStats** - per-frame work counters + phase timers, reset each frame.
- **BudgetState** - the phase-budget controller; pairs with `Profiling.h`'s pure
  math (`emaUpdate` / `predictiveSkip` / `spikeClipTripped`).
- **FrameDiag** - the remaining per-frame + session diagnostic bookkeeping.
- **OcclusionCaches** - the five per-cell caches (`land` / `drain` /
  `terrainMembership` / `occluder` / `occludeeBox`) + their hit/miss counters +
  `wipeForCellChange()`. A sixth, `lightCull`, went with light culling in 1.6.0.

## Per-frame data flow

The engine calls `renderMainScene`, which is wrapped, and `cullShow`, which is
detoured. One top-level pass per scene:

1. `renderMainScene_wrapper` resets per-frame state; `resetFrameTints` (DebugTint).
2. `CullShow_detour` (top-level entry):
   - `ensureMSOCResourcesMatchConfig` (MaskResources) reconciles the gate.
   - `g_frame.snapshot(isInterior)` (FrameConfig) caches the hot-path knobs.
   - `ClearBuffer`; `uploadCameraTransform` sets the live projection.
   - scene traversal (`cullShowBody`): large opaque leaves -> `rasterizeTriShape`
     (occluders); small leaves -> deferred queue; `classifyOccluderProperties`
     (OccluderClassify) gates alpha/stencil out of the occluder pass.
   - `rasterizeAggregateTerrain[Horizon]` (TerrainAggregation) adds terrain.
   - drain: `classifyDrainRange` -> `TestRect` verdicts -> `drainPendingDisplays`
     skips `display()` on OCCLUDED leaves. Phase 1 writes nothing phase 2 reads,
     with one deliberate exception: the occludee box cache fills lazily there.
   - `updateMaskOverlay` (MaskOverlay) refreshes the debug texture, but only
     once Lua has asked for it; otherwise the readback never runs.
   - `emitPerFrameStatsLine` (DiagnosticsLog) on enabled log channels.

The mask buffer is not published, copied or swapped: `SetBuffer` is called once
at pool creation and the same buffer is cleared and rebuilt each frame.

## The Lua surface

`plugin.cpp` is the only outward-facing boundary. `luaopen_msoc` returns a table
with `install`, `configure`, the forensics read-accessors, and the debug entry
points (`maskOverlayTexture`, `maskResolution`, `dumpMask`, `logMark`,
`flushLog`).

**Load order is part of the contract.** `luaopen_msoc` probes the MOC link and
classifies the CPU tier but installs nothing. `main.lua` then pushes `msoc.json`
across with `configure()` and only afterwards calls `install()`, which is what
latches the restart-only knobs (mask resolution, the forensics watchdog). Before
1.6.0 the DLL installed itself during `include()`, so those knobs latched
compile-time defaults and a saved mask size was discarded in silence. The tier
table that feeds this lives in `config.lua` and nowhere else; C++ carries no
copy. The DLL exports no occlusion C API; through 1.4.0 it exported
`mwse_*` thunks for MGE-XE, which never shipped a consumer, and 1.6.0 removed
them along with the published snapshot they served.

## Testing

`MSOC_BUILD_TESTS` (default ON) builds `msoc_tests` from the pure-leaf modules +
doctest, gated behind `MSOC_BUILD_DLL` so it builds with no MWSE/LuaJIT/Win32
(CI-friendly). Covered: ClipMath, Profiling, HorizonOccluder, HardwareTier
(26 cases / 48203 assertions). The engine-coupled TUs are verified by build/link
and the in-game `OcclusionLogAggregate` stats line (parse with
`scripts/parse_msoc_log.py`).

## Conventions
- Formatting is enforced by `.clang-format` (4-space, no tabs, namespace bodies
  not indented, `ColumnLimit 0`). `deps/` is excluded (`DisableFormat`).
- ASCII only in source.
- SIMD is capped at AVX2 (`Create(AVX2)` in plugin.cpp + MaskResources.cpp);
  AVX512 is disabled pending a revisit.

## Known structure notes
- The core's two remaining hot subsystems - occluder rasterization
  (`rasterizeTriShape`) and the drain pipeline (`classifyDrainRange` /
  `drainPendingDisplays`) - have not been split out; they are the highest-risk
  extractions and would need in-game verification. After them the core would be
  a thin orchestrator.
- `OcclusionInternal.h` carries both shared state and the function contract; if
  it keeps growing, splitting the data declarations into an `OcclusionState.h`
  would clarify it.
