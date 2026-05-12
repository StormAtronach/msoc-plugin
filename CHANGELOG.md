# Changelog

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
