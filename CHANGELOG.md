# Changelog

## 1.6.0 - 2026-07-21

A crash-immunity release for the external-occluder path, plus config
lifecycle fixes. Driven by a multi-day forensic hunt into intermittent
exterior-transition crashes reported on 1.4/1.5: the process hosts a
still-unidentified heap corrupter (writes recycled mesh/instance data
through stale pointers; every threaded suspect in this plugin and MGE-XE
was individually eliminated), and this plugin's external-occluder queue -
CRT-heap `std::vector`s, hot-allocated every frame - was its most
frequent victim, faulting inside MOC's SIMD vertex gather during the
drain.

- **External-occluder queue moved to a page-protected arena.** Consumer
  submissions (MGE-XE's horizon curtain) now live in a dedicated
  `VirtualAlloc` arena kept `PAGE_READONLY` except during this module's
  own bracketed writes - outside the contested CRT heap entirely, so a
  stray heap-recycling writer can't land on it, and a writer that
  targets it anyway faults at *their* instruction, naming themselves in
  the crash log. Entries are canary-bracketed PODs revalidated at drain
  time and rasterized under SEH; a corrupt entry becomes one dropped
  submission plus a diagnostic log line instead of a process crash.
  Verified in-game: sessions that previously logged 100k+ corrupted
  entries and crashed now run clean with zero corruption detections.
- **Boundary validation hardened.** `mwse_addOccluder` /
  `mwse_addPreTransformedOccluder` now reject out-of-range triangle
  indices (MOC's gather has no bounds check) and non-float-multiple
  strides (previously a heap overrun in the copy sizing), per-submission
  vertex payloads are capped, and the shared triangle budget is enforced
  before arena append.
- **Cell-transition safety.** Camera transforms are validated for
  finiteness before mask population (a mid-load frame no longer poisons
  projections or publishes a garbage snapshot; consumers see NotReady
  and render conservatively). Terrain-cache subcell nodes are
  `NI::Pointer`-pinned, and all live landscape-graph walks clamp
  `endIndex` against `storageCount` so a mid-populate array can't be
  walked out of bounds.
- **Config lifecycle fixes.** Version migration now runs on genuine
  upgrades only (a downgraded or version-flapping install no longer
  re-forces tier defaults every launch) and preserves user-overridden
  keys via a per-key baseline, logging exactly what was applied vs
  preserved. The MCM only rewrites `msoc.json` when a setting actually
  changed, so manual file edits survive a browsing session. Threadpool
  thread-count/bin changes now apply live through a safe resource
  rebuild instead of silently requiring a restart (mask dimensions
  remain restart-only by design).
- **Build.** MWSE submodule pin bumped to current upstream master
  (`de84eea`, 2026-07-16); SharedSE's new crash-logger TUs (which assume
  MWSE's own PCH environment) are excluded from the compile-all glob.

## 1.5.0 - 2026-07-05

Single fix release (published on Nexus; this changelog entry was added
retroactively in 1.6.0).

- **Fixed: distant statics flickering just above ridgelines**
  (Horizon-mode terrain). The curtain top was min(endpoint heights) over
  sparse simplified samples of a per-column MAX envelope, so it sat above
  the true silhouette wherever a skyline notch fell between samples, and
  the error shifted per frame with bin-winner and tile-snap phase
  changes. Replaced the blunt global y-margin with a conservative
  erosion pass: each sample's height drops to the min touched horizon
  height over the span to its neighbours (depth rises to the max),
  making the emitted curtain conservative against the full 512-column
  horizon at every column.

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
