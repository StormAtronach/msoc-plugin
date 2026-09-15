# Changelog

## 1.6.0 - 2026-09-15

Three removals, one new debug tool, and a round of terrain and bookkeeping
fixes. The MGE-XE integration ABI, CPU light culling and the Horizon terrain
mode are gone; the live mask overlay is new; downsampled terrain is wound the
right way again and stops at the nine active cells; the low tier now runs
the raster at Corners resolution. Nexus lists the previous release as 1.5.

- **Removed the MGE-XE integration ABI.** Through 1.4.0 the plugin published a
  double-buffered snapshot of its mask and a set of `mwse_*` `__cdecl` exports
  (`mwse_testOcclusionSphere`, the batch and AABB/OBB variants, the snapshot
  accessors, the visible-geometry and light observer registrations, and the
  external-occluder submission queue) so MGE-XE could cull distant statics
  against the same mask. No released MGE-XE ever called any of it, including
  G7; the only consumer lives on an unreleased development branch. Keeping the
  contract cost a second `MaskedOcclusionCulling` instance, a per-frame
  `SetBuffer` re-point on the threadpool (and therefore an extra worker
  suspend/wake round trip on every async frame), an external-occluder drain at
  the top of the pass, and observer callbacks on the drain. `Exports.cpp`,
  `QueryApi.cpp` and `ExternalOccluders.cpp` are deleted and `OcclusionApi.h`
  now declares nothing but `installPatches()`. `msoc.dll` exports no occlusion
  API. If a future MGE-XE wants this, it should be designed against a shipped
  consumer rather than kept warm speculatively.
- **Removed light culling** (`OcclusionCullLights`,
  `OcclusionLightCullHysteresisFrames`). The feature tested every `NiLight`
  against the mask and disabled fully-occluded ones for the frame. It was
  opt-in, never demonstrated a clear win, and required the plugin's second
  engine detour (the `updateLights` enabled-read hook at 0x6BB7D4), a per-cell
  light cache with its own age-prune pass, and four fields on the stats line.
  Both config keys are retired and removed from saved JSON on first launch.
- **Added a live occlusion-mask overlay** (`DebugMaskOverlay`, "Show occlusion
  mask" on the MCM Debug page). Draws the mask the rasterizer is building into
  a help-layer panel in the top-right corner: unwritten tiles black, occluder
  depth ramped on a log scale of inverse-w, nearest brightest. The ramp is
  logarithmic rather than linear because a linear stretch is owned by the
  single nearest pixel, and in Raster terrain mode that pixel is pinned at the
  ceiling on every outdoor frame: the ground under the player straddles MOC's
  near plane and each clipped edge lands at w = 1, so everything past the
  foreground collapsed into one flat grey while the near hillside saturated
  white. Modelled on MGE-XE's shadow-map debug overlay, but
  delivered through the engine UI so no render state is touched behind the
  NiDX8 state cache. The readback is armed by Lua asking for the texture, so
  the overlay costs nothing while it is off. It replaces the PFM dump as the
  way to inspect the mask; `msoc.dumpMask(path)` still writes the same buffer
  to a file, with the same tone map, for offline comparison.
- **Fixed: downsampled terrain (Half / Corners) was wound backwards and mostly
  culled.** The coarse terrain builder emits its own quad triangles rather than
  copying the source patch's, and it wound them clockwise while the full-
  resolution path and the source patches are counter-clockwise. The same
  `OcclusionOccluderCCWOnly` gate (default on) that halves occluder work by
  culling CW faces then dropped almost every coarse triangle: at the Half
  default about a third of the terrain silhouette went missing, Corners lost
  the lot. It showed as black fractures running across the hills in the mask
  overlay and, worse, as terrain that occluded far less than it should. Found
  by dumping the mask against the live camera and re-rasterising the same
  patches offline; the coarse quads now wind CCW to match the full path.
- **Removed the Horizon terrain mode.** The 1D silhouette curtain that the
  low tier defaulted to never worked: it was fed one vertex per column, and
  near terrain has a vertex only every 40 to 200 mask columns, so the columns
  between took their height from the hidden far terrain behind the hill and
  the curtain settled at eye level. (That scheme came from MGE-XE's
  distant-land curtain, where ROAM keeps vertices dense along silhouettes;
  Morrowind's near land is a fixed grid, and MGE-XE G7 has no ROAM terrain.)
  Rebuilt properly, with each triangle's upper edge written into every column
  it spans, the depth coupled to the silhouette as the design required, and
  eight nested depth bands so the curtains could carry near ground under a
  far ridge, it was a correct, conservative occluder, and that made its cost
  measurable: 511 us of main-thread time per frame at Half against 114 us for
  the synchronous Half raster and 54 us for Corners, which also occluded more.
  A scalar 1D fill spends more column operations on a near triangle than
  MOC spends SIMD tile operations, so it cannot undercut the rasterizer. The
  mode and its counters are gone; `OcclusionAggregateTerrain` keeps its
  integer type with 0 = Off and 1 = Raster, a saved 2 reads as Raster, and
  the low tier now runs Raster at Corners resolution. The investigation is
  written up in the moreFPS engineering notes.
- **Both terrain modes now skip lands outside the 3x3 active grid** around
  the player's cell. Nothing the engine renders stands behind such terrain,
  MGE-XE's distant land has its own visibility handling, and the outer lands
  are the most triangles for the least occlusion.
- **Fixed: the drain's empty-mask fast path inferred emptiness from two
  submit counters** instead of asking whether anything had been submitted,
  so a path that touched neither counter (the since-removed horizon curtain)
  had its occludees skipped entirely. Both that gate and the async flush gate
  now read facts recorded where the triangles are actually submitted.
- **Fixed: a wasted threadpool barrier on budget-limited frames.** With
  front-to-back submission the occluder queue is drained after traversal and a
  spike-clip can bail the tail. The flush gate counted occluders recorded rather
  than jobs queued, so a frame that bailed before queuing anything still paid
  for a full `Flush` barrier.
- **Fixed: `rasterized=` in the stats line over-counted.** It was incremented
  when an occluder was accepted, not when it was submitted, so budget-dropped
  occluders were counted as if they had reached the mask.
- **Fixed: per-frame log spam on single-worker machines.** The reconciler runs
  every frame and only short-circuits once a threadpool exists, so a machine
  where the pool is deliberately declined re-made and re-logged that decision
  forever. It is latched now, and cleared when resources are torn down so an MCM
  toggle re-evaluates.
- **The tier defaults live in one place, and your saved mask resolution now
  works.** `config.lua` and `Config.cpp` each carried a tier table; whichever ran
  last won, silently, and they had drifted. The C++ copy is gone. Related, and
  the reason it matters: the DLL used to install its engine hooks during
  `include()`, before `main.lua` could push `msoc.json` across, so the
  restart-only knobs latched compile-time defaults and a hand-edited
  `OcclusionMaskWidth`/`Height` was discarded without a word. Installation is now
  a separate `msoc.install()` that `main.lua` calls after the config sync.
  **Two behaviour changes follow.** A saved mask resolution takes effect at
  launch where before it never did. And `OcclusionSkipTerrainOccludees`, which
  was tier-sensitive in C++ only, is now tier-sensitive everywhere: off on mid
  and high tier, where letting terrain leaves through the occludee test saves
  more `displayUs` than the tests cost. Both keys are re-applied once on upgrade.
- **Fixed: a stale occludee box could survive its mesh.** The cache is keyed on
  the address of a mesh's geometry data but held no reference to it, so the
  engine could free that mesh mid-cell and hand the same address back for a
  different one, leaving the entry describing the wrong bounding box and
  producing a wrong cull. It pins its key now, at one refcount per unique mesh
  per cell, which is what the other four per-cell caches already did.
- **Fixed: the phase budget could not see async rasterization.** On an async
  frame the main thread pays for rasterization twice, once to enqueue and again
  waiting at the flush barrier, and the budget's moving average sampled only the
  enqueue. It therefore read a few hundred microseconds on frames that cost a
  millisecond, and predictive skip never engaged under async however dense the
  scene got. The average now includes the flush stall.
- **AVX-512 is no longer built.** Intel's `USE_AVX512` has always defaulted to
  0, which compiled that translation unit into a stub returning null, so the
  runtime never selected it. 1.6.0 stops compiling the file and preprocesses out
  the dispatch branch. No behaviour change; one fewer untested path, and a
  slightly smaller DLL. Recorded in `deps/msoc/NOTICE`.
- Removed dead state and API that nothing read: the `inlineTested` counter,
  which the stats line has been reporting as a constant 0 since the inline-test
  path went away, two unread cache fields, the commented-out parallel drain and
  the includes that served it, and `OpenLog` / `CloseLog` / `getDebug` /
  `prettyDump` from the logger.
- Corrected three MCM descriptions that said "Default off" for settings that
  have shipped on by default since 1.3.0 and 1.4.0, and one that predicted
  front-to-back submission would be "a clear win on single-threaded setups".
  Measurement for this release put it within noise of zero, with or without
  async, so the description now says so.
- Internal: there is one mask buffer instead of two, and `SetBuffer` is called
  once at pool creation rather than every frame. The drain reads a bool for
  terrain membership instead of doing a second hash lookup per occludee per
  frame. `FrameConfig`'s defaults come from `Configuration` rather than a
  hand-maintained copy, and the camera's culling-plane count is read rather
  than assumed to be six. `src/` drops from 6278 to 5206 lines.

- Documented where the culler is a net loss. It wins in dense architecture and
  breaks even on open ground, and the places where it breaks even are already
  running at 200 fps. The one real regression left is a sparse exterior on a
  weak CPU: Vivec on four cores costs 16%, on a frame that was at 151 fps. An
  adaptive switch that turned the pass off in those scenes was prototyped three
  times and none of the three was reliable, so 1.6.0 ships the measurements and
  the master toggle instead of a mechanism that guesses.

**Upgrading:** `main.lua` and `msoc.dll` must be updated together. A new DLL with
an old `main.lua` loads but never installs its hooks, and says so in `MSOC.log`;
an old DLL with a new `main.lua` works as it did before, and says so too.

## Superseded parallel line (never released)

Developed in parallel with 1.6.0 from the same commit and not carried forward.
It hardened the MGE-XE external-occluder path that 1.6.0 removes outright, so
the two could not both be kept.

Its finding is worth recording even though its code is not. It root-caused a
long-standing exterior-transition crash to an ODR violation: `OcclusionPass.cpp`
and `ExternalOccluders.cpp` each declared a different `struct PendingOccluder`
in the same namespace, 8 bytes against 116. Neither sat in an unnamed
namespace, so the *types* could not have internal linkage however `static` the
globals were, and the linker folded `std::vector<PendingOccluder>` to one
instantiation. Every external-occluder submission then constructed a 116-byte
object into an 8-byte allocation, overflowing the heap by 108 bytes on every
frame outdoors with MGE-XE's horizon curtain on. An ODR violation is
ill-formed-no-diagnostic-required, so no warning was ever coming.

1.6.0 cannot hit it: `ExternalOccluders.cpp` is gone and one `PendingOccluder`
is left. The bug is fixed by deletion rather than by repair. The wider
convention that line adopted, every translation-unit-local type in an unnamed
namespace, is not in 1.6.0 and is worth adopting on its own merits.

## 1.4.0 - 2026-07-02

A query correctness fix, two occluder-throughput features, and threadpool /
build improvements.

- **Fixed: false occlusion at steep view angles.** The sphere occludee query
  divided its clip-space extents by the center w only; a sphere's near half
  has smaller w, so off-axis close objects got undersized screen rects and
  could be wrongly culled near screen edges - visible as buildings vanishing
  when looking up close to large architecture (Vivec cantons), flipping with
  small camera movements. The rect is now an exact-conservative interval
  bound at any angle (`clipmath::conservativeSphereNdcRect`), applied to both
  the drain query and the external-consumer snapshot query, with a
  sphere-surface property test and a regression case pinning the old failure.
- **CCW-only occluder winding** (`OcclusionOccluderCCWOnly`, default on).
  Occluders rasterize front (counter-clockwise) faces only, roughly halving
  occluder raster work across per-instance occluders, aggregate terrain, the
  horizon curtain, and external-consumer occluders. The rare CW-wound mesh
  drops out of the mask as a safe under-occlude, never a wrong cull.
- **Front-to-back occluder submission** (`OcclusionOccluderFrontToBack`,
  default on). Per-instance occluders collected during traversal are sorted
  near-to-far and submitted before the flush, letting the rasterizer
  early-reject occluder triangles already behind the accumulating mask.
- **Threadpool workers park instead of yield-spinning.** Idle workers used to
  spin through the entire per-frame wake window (~20% of a core each, mostly
  syscall churn); they now sleep on a work signal (`WaitOnAddress`) and wake
  on job submission. Verified ~4x less worker CPU with unchanged mask timing.
  This makes **Windows 8 the minimum supported OS**.
- **Static CRT.** The DLL no longer requires the Visual C++ redistributable.
- Release builds now emit full debugging symbols (PDB, kept out of the
  shipped archive) so profilers can resolve plugin frames.

## 1.3.0 - 2026-06-30

Source decomposition, an occludee box test, a Horizon terrain perf rework, and
tuned exterior occluder defaults.

- **Decomposed the occlusion monolith into subsystem translation units.**
  `PatchOcclusionCulling.cpp` (~3800 lines) is replaced by `OcclusionPass.cpp`
  (core: CullShow detour, occluder rasterization, drain) plus per-subsystem TUs
  (QueryApi, LightCulling, TerrainAggregation, MaskResources, DiagnosticsLog,
  ExternalOccluders, DebugTint), six state-owner structs reached through the
  `OcclusionInternal.h` seam, and pure-leaf modules (ClipMath, Profiling,
  HardwareTier, HorizonOccluder) with doctest unit tests. No behaviour change.
  Module map in `ARCHITECTURE.md`. AVX512 capped to AVX2 pending a revisit.

- **Occludee bounding-box test (`OcclusionOccludeeBoxTest`, now default on).**
  After a sphere-VISIBLE verdict, re-tests the occludee's object-space vertex
  AABB (8 corners) against the mask. Tighter than the loose bounding sphere for
  long/flat meshes; only ever upgrades Visible -> Occluded. Cached per geometry.

- **Horizon terrain projects the shared per-Land cache.** The Horizon-mode
  curtain now projects the same cached world-space per-Land verts the Raster
  path submits, instead of re-walking WorldLandscape and re-transforming every
  vertex each frame. The per-vertex transform + RTTI/alpha-stencil classify is
  paid once per cell on cache miss; the per-frame cost is projection + bin.

- **Tuned exterior occluder defaults.** Exterior occluder max radius
  4096 -> 7040 (admits larger architecture as occluders); depth slack
  128 -> 64 world units.

- **Fix: per-frame reset of the box-test counters.** `boxOccluded` /
  `occludeeBoxHits` / `occludeeBoxMisses` were never zeroed in the per-frame
  reset, so the stats line and `parse_msoc_log.py` read them as lifetime
  cumulative totals. Reset like every other counter; `parse_msoc_log.py` also
  now surfaces Horizon-mode terrain timing (`horizonBuildUs`/`horizonRasterUs`).

## 1.2.0 - 2026-06-09

Build against the unified-NI MWSE/SharedSE, a cell-cross profiler, and a
crash-path fix. No gameplay behaviour change.

- **Compile against the unified SharedSE NI headers.** Bumped the pinned
  MWSE submodule to the post-unification engine: NI types moved to
  `SharedSE/`, vectors are now `NI::Point3` / `NI::Point4` (was
  `TES3::Vector3` / `Vector4`), memory helpers are `se::memory::`, and LuaJIT
  moved to the `luajit2` submodule. The plugin declares its SharedSE consumer
  config in `stdafx.h` (a prelude mirroring MGE-XE's: `SE_TARGETS_MW` plus the
  Morrowind allocator addresses) and compiles the SharedSE sources directly
  via a file-glob, the same model MGE-XE uses.

- **Replaced the hand-rolled NI forwarders with the canonical SharedSE
  sources.** `MWSEImports.cpp` previously bound ~12 NI methods with raw inline
  engine addresses; those are now the versioned, `SE_*_FNADDR`-gated impls
  compiled from SharedSE. `MWSEImports.cpp` now holds only the three TES3
  globals SharedSE has no equivalent for.

- **Cell-cross profiling (`OcclusionLogCellCross`; MCM: Debug -> Logging).**
  Emits the full stats line on each cell change plus the next few
  re-population frames, tagged `cellCross=<age>` (0 = the cross frame), with a
  dedicated `cellWipeUs` (cache wipe / NI teardown), `occXformUs` (occluder
  vertex re-transform), and per-frame `frameDeltaUs`. The 300-frame aggregate
  sample almost never lands on a crossing; this makes the cross spike's phase
  breakdown directly visible. See README -> Performance characteristics ->
  Cell-cross cost for the Narsis numbers (the hitch is engine cell-load on the
  frame after the cross, not MSOC).

- **Fix: leaked `g_msoc_prev` on threadpool-creation failure.** The two catch
  blocks in `createMSOCResources` freed `g_msoc` + `g_threadpool` but not the
  snapshot buffer; both now route through the complete `destroyMSOCResources`
  teardown.

## 1.1.0 — 2026-05-12

Occluder population fixes + a per-instance eligibility cache. Lifetime
cull rate in a mixed exterior + canton test session moved from ~36% to
~43% with no visible regressions. Each change shipped with measurement
justification (probe counters in `MSOC.log`).

- **Accept `NiTriBasedGeom` (NiTriStrips + NiTriShape) as occluder
  candidates.** The previous gate filtered on `NiTriShape` only, silently
  dropping every `NiTriStrips` mesh. Vivec canton architecture and a
  large slice of vanilla statics ship as NiTriStrips — they were invisible
  to the mask. `getTriList()` / `getActiveTriangleCount()` are virtual on
  the base data class, so the rasterizer's existing call sites work
  polymorphically against either shape type.

- **Inside-AABB occluder guard is now opt-in
  (`OcclusionInsideOccluderGuard`, default `false`).** The guard rejected
  any mesh whose tight world-space AABB + margin contained the eye, on
  the theory that a concave shell with the camera inside would write
  near-face depths and falsely occlude things behind the far face. With
  `BACKFACE_NONE` rasterizing both shell sides, MOC's per-tile zMin/zMax
  mask handles the concave case fine — and the actual effect of the gate
  was eating close-up walls. Disabling it added 7 percentage points to
  the lifetime cull rate with no visual regression. Set the new INI key
  to `true` in `msoc.json` if you want the old rejection back.

- **Per-instance occluder eligibility cache.** `classifyOccluderProperties`
  (~145 µs/frame median, peak 260 µs) and `rasterizeTriShape`'s per-vert
  world transform (~1.2 ms median, peak 1.8 ms) were both being redone
  every frame for static cell meshes. Now both results live in
  `g_occluderCache` keyed on `NI::AVObject*`, invalidated on cell change
  and on `worldTransform` memcmp drift. Sustained 99.9% hit rate after
  cell warmup; ~14 µs peak miss-path work on cell transitions. Net effect
  ≈ 1.5 ms/frame eliminated from the cullShow path on dense scenes.

Diagnostic surface for the new cache (in periodic log when
`OcclusionLogAggregate` is on): `occCacheHit`, `occCacheMiss`,
`occCacheSize`, plus probe counters `classOccCalls/Steps` and
`occVertCalls/Verts` that bracket residual miss-path workload.

- **`OcclusionSkipTerrainOccludees` now defaults per hardware tier.** A/B in
  a dense Vivec exterior showed letting terrain leaves flow through
  `TestRect` saves ~1.9 ms/frame in `displayUs` on mid/high-tier hardware
  (a meaningful fraction of terrain reads `OCCLUDED` against the now-denser
  mask and skips `display()`). The bypass stays on for low-tier hardware
  where the extra `TestRect` work would eat the `classifyBudget` and the
  smaller mask makes the `displayUs` win narrower. Override in `msoc.json`
  if you want a specific value.

- **Light culling hidden from the MCM, default off.** The `Cull occluded
  lights` toggle and its hysteresis slider tested net-negative in a Vivec
  canton at night (~12% FPS regression). The bracketed savings inside the
  MSOC drain (~480 µs/frame) were real, but engine-side relighting churn
  outside the instrumented region cost more than that. The native
  `OcclusionCullLights` key is still read from `msoc.json` so a power user
  can flip it on for re-testing on different hardware/scenes; the UI
  surface is removed so casual users don't trip on a feature that doesn't
  help. Fixes a 1.0 bug along the way — the cull was previously reading
  the uncomputed `worldBoundRadius` field on every light and silently
  bailing, so 1.0's "Cull occluded lights" toggle never actually did
  anything.

## 1.0.0 — 2026-04-26

Initial public release. Near-scene CPU occlusion culling for Morrowind via
a software-rasterized depth mask, with conservative-direction defaults
(hysteresis, sphere inflate, depth slack) tuned for under-cull stability.
Distant-statics integration with MGE-XE present as a C-ABI export surface
but the consumer side is not in any released MGE-XE.
