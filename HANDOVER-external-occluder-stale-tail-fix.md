# SUPERSEDED — diagnosis was wrong

**Do not apply the patch this document originally described.** The stale-tail /
SIMD-gather theory it proposed is **incorrect** and has been refuted:

- This MOC version's gather variants are all tail-guarded (AVX2 `SIMD_LANE_MASK`
  offset clamping, SSE `numLanes`-bounded scalar loop, `VtxFetch4`'s per-lane
  `if (numLanes > lane)`). There is no read past the index buffer on a partial
  final batch, so there was nothing to "pad."
- Empirical: the MGE-side curtain padding (d3d8.dll built 11:24) was already live
  in the 11:28 repro and it crashed **identically**. Triangle count / tail
  alignment is not the cause.

## Actual signature (from 3 crash dumps)

Identical faulting instruction in `drainPendingOccluders` → `RenderTriangles`,
but the values read out of the queued `PendingOccluder` are **different garbage
each time** — `stride` ~`0x6c050ea2` (could never pass intake validation),
`triCount` in the billions, sometimes a wild `verts` pointer, sometimes a
plausible one. Every value the drain supplies itself (clip mask `0x1F`, null
matrix, winding) is correct. → The queue entry's memory is being **overwritten
between submission (consumer thread) and drain (render thread)** by something
not yet identified.

## Current instrumentation (deployed on the msoc side)

Each queue entry is bracketed by canaries and fully revalidated at drain time
(`entryStillSane`); a corrupted entry is **dropped instead of rasterized**, and
one log line captures the full field snapshot — canary state (head/tail/full
overwrite), all scalars, both vector sizes + data pointers. Next repro should
not crash; instead `MSOC.log` gets `MSOC drain: CORRUPTED queue entry dropped …`
lines whose pattern distinguishes heap-reuse vs. linear overflow vs. targeted
write. If it still crashes with canaries in place, the corruption is mid-drain
(same-frame code).

## MGE-side status

The curtain padding in `renderexterior.cpp` (round `triCount` up to 16, degenerate
tail) is functionally harmless but its **comment documents the non-existent
stale-tail mechanism** — to be reverted once root cause is confirmed. Left in
place for now so the deployed `d3d8.dll` is frozen across repro runs (no new
variable). MGE copies its buffers into plugin-owned storage synchronously in
`addOccluder`/`addPreTransformedOccluder`, so the corruption is of msoc's own
heap allocations, not a dangling pointer into MGE's buffers.
