# MSOC plugin architecture

CPU-side occlusion culling for Morrowind. Each frame the plugin software-
rasterizes a coarse depth mask from near-scene opaque geometry (via Intel's
Masked Software Occlusion Culling), tests every small `NiTriBasedGeom` leaf
against it, and skips `display()` on leaves that fall fully behind the mask.
A double-buffered snapshot of the mask is also published for out-of-tree
consumers (MGE-XE). See [`README.md`](README.md) for the user-facing story;
this document is the code map.

## Layering

The source is organized as a dependency DAG: every module depends only on
modules below it. Nothing below the core depends back up into it.

```
  ABI boundary      OcclusionApi.h (public C exports' declarations)
                    Exports.cpp (mwse_* __cdecl thunks)   plugin.cpp (luaopen)
  ----------------------------------------------------------------------------
  Core orchestrator OcclusionPass.cpp
                      - CullShow detour + frame lifecycle
                      - scene-graph traversal (cullShowBody)
                      - occluder rasterization (rasterizeTriShape)   [hot]
                      - deferred-display drain pipeline              [hot]
                      - install, forensics read-accessor
  ----------------------------------------------------------------------------
  Subsystems        QueryApi          out-of-tree mask query API (snapshot)
  (engine-coupled)  LightCulling      updateLights enabled-read hook
                    TerrainAggregation near-Land merge (Raster + Horizon)
                    MaskResources     MOC buffer + threadpool lifecycle
                    DiagnosticsLog    per-frame stats line (cold path)
                    ExternalOccluders MGE-XE consumer occluder injection
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
                    MaskSnapshot     published snapshot metadata   (g_snapshot)
  ----------------------------------------------------------------------------
  Pure leaves       ClipMath      projection / matrix math
  (engine-free,     Profiling     EMA + budget skip/clip decisions
   unit-tested)     HardwareTier  CPU tier classifier
                    HorizonOccluder 1D silhouette curtain builder
```

### Why the leaf layer exists
`LiveQuery` and `OccluderClassify` are not subsystems in their own right - they
are helpers shared by *both* the core and a subsystem. `testSphereVisible` is
called by the core drain and by `LightCulling`; `classifyOccluderProperties` by
the core rasterizer and by `TerrainAggregation`. Keeping them in the core would
make those subsystems depend back into the core (a cycle). Pulling them down
into leaf TUs keeps the graph acyclic. `ExternalOccluders` is similar: the core
detour drains it, `MaskResources` clears it, so it lives below both.

## The shared-state seam: `OcclusionInternal.h`

The plugin's private internal contract (distinct from the public ABI in
`OcclusionApi.h`). It is included by every patch TU and provides two
things:

1. **Shared state** - `extern` declarations of the single global instances
   defined in `OcclusionPass.cpp`: the state owners (`g_frame`,
   `g_stats`, `g_budget`, `g_diag`, `g_caches`, `g_snapshot`), the MOC
   resources (`g_msoc`, `g_msoc_prev`, `g_threadpool`), the live projection
   (`g_worldToClip`, `g_ndcRadius*`, `g_wGradMag`), and assorted frame flags.
2. **The cross-TU function contract** - declarations of the functions a TU
   exposes to the others (e.g. `rasterizeAggregateTerrain`, `createMSOCResources`,
   `emitPerFrameStatsLine`, `testSphereVisible`, `drainPendingOccluders`). Every
   one of these is *defined in a subsystem/leaf TU* - the core only calls them.

Hot-path helpers (`projectWorld`, `ScopedUsAccumulator`, the camera-plane
accessors) are header-inline so they still inline across TU boundaries.

### State owners
The decomposition replaced ~130 loose file-static globals with six owner
structs, each a single `extern` instance reached through the seam. Grouping
rationale:
- **FrameConfig** - the hot-path slice of `Configuration`, snapshotted once per
  top-level frame (`snapshot(isInterior)`) so inner loops stay branch-free.
- **FrameStats** - per-frame work counters + phase timers, reset each frame.
- **BudgetState** - the phase-budget controller; pairs with `Profiling.h`'s pure
  math (`emaUpdate` / `predictiveSkip` / `spikeClipTripped`).
- **FrameDiag** - the remaining per-frame + session diagnostic bookkeeping.
- **OcclusionCaches** - the five per-cell caches (`land` / `drain` /
  `terrainMembership` / `occluder` / `lightCull`) + their hit/miss counters +
  `wipeForCellChange()`.
- **MaskSnapshot** - the matrix + NDC constants published at the buffer swap.

## Per-frame data flow

The engine calls `renderMainScene`, which is wrapped, and `cullShow`, which is
detoured. One top-level pass per scene:

1. `renderMainScene_wrapper` resets per-frame state; `resetFrameTints` (DebugTint).
2. `CullShow_detour` (top-level entry):
   - `ensureMSOCResourcesMatchConfig` (MaskResources) reconciles the gate.
   - `g_frame.snapshot(isInterior)` (FrameConfig) caches the hot-path knobs.
   - `ClearBuffer`; `uploadCameraTransform` sets the live projection.
   - `drainPendingOccluders` (ExternalOccluders) rasterizes consumer submissions.
   - scene traversal (`cullShowBody`): large opaque leaves -> `rasterizeTriShape`
     (occluders); small leaves -> deferred queue; `classifyOccluderProperties`
     (OccluderClassify) gates alpha/stencil out of the occluder pass.
   - `rasterizeAggregateTerrain[Horizon]` (TerrainAggregation) adds terrain.
   - drain: `classifyDrainRange` -> `TestRect` verdicts -> `drainPendingDisplays`
     skips `display()` on OCCLUDED leaves.
   - visible-geom callback fires the surviving set (for MGE depth/shadows).
   - `emitPerFrameStatsLine` (DiagnosticsLog) on enabled log channels.
   - swap `g_msoc` <-> `g_msoc_prev`; publish `g_snapshot` for consumers.
3. `updateLights_enabledRead_hook` (LightCulling) fires during light updates,
   testing each NiPointLight via `testSphereVisible` (LiveQuery).
4. Out-of-tree consumers query the published snapshot through QueryApi /
   the `mwse_*` exports (Exports.cpp).

## ABI

- `OcclusionApi.h` is the frozen public contract: the `mwse_*` C
  exports and the `msoc::patch::occlusion` query/callback API. Result codes are
  fixed (`0=Visible 1=Occluded 2=ViewCulled 3=NotReady`).
- `Exports.cpp` is a thin thunk layer; the API functions it forwards to live in
  QueryApi / LightCulling / ExternalOccluders.

## Testing

`MSOC_BUILD_TESTS` (default ON) builds `msoc_tests` from the pure-leaf modules +
doctest, gated behind `MSOC_BUILD_DLL` so it builds with no MWSE/LuaJIT/Win32
(CI-friendly). Covered: ClipMath, Profiling, HorizonOccluder, HardwareTier
(25 cases / 202 assertions). The engine-coupled TUs are verified by build/link
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
