#!/usr/bin/env python3
"""Analyze MSOC cell-cross profiling lines from MWSE.log / MSOC.log.

The plugin (with OcclusionLogCellCross enabled) emits, on every cell change
and the next several re-population frames, one line of the form:

  MSOC: frame N cellCross=A cellWipeUs=B frameDeltaUs=C rasterizeUs=... ...

where cellCross=0 is the cross frame and 1..7 are the recovery frames as the
caches refill on the miss path. This groups those lines into crossings and
attributes the spike: for each phase timer / miss counter it reports the
value on the spike frame(s) vs the recovered frame(s), so you can see which
phase actually grows on a crossing.

Usage:
  python analyze_cellcross.py "<path to MWSE.log>"
  python analyze_cellcross.py "<log>" --crossings   # per-crossing detail too
"""
import argparse
import re
import statistics
import sys

# key=int pairs. cumul=12/34 yields cumul=12 (the /34 is ignored), which is fine.
_KV = re.compile(r"([A-Za-z_]\w*)=(-?\d+)")

# Phase wall-time timers (microseconds), ordered roughly by the pipeline.
TIMERS = [
    "frameDeltaUs",   # whole-frame wall time (the spike magnitude itself)
    "cellWipeUs",     # cache wipe on the cross frame
    "occXformUs",     # occluder world-vert transform (the cache-miss rebuild)
    "rasterizeUs",    # occluder rasterisation (MOC RenderTriangles)
    "aggTerrainUs",   # terrain aggregation (incl. buildLandCacheEntry on miss)
    "horizonBuildUs",
    "classifyUs",     # drain phase-1 verdicts
    "drainUs",        # whole drain
    "displayUs",
    "wakeUs",
    "asyncFlushUs",
]

# Miss-path workload counters that drive the re-population cost.
MISSES = [
    "occCacheMiss",
    "classOccCalls",
    "classOccSteps",
    "landMembershipMiss",
    "landCacheMiss",
    "landCacheEvict",
    "occVertCalls",
    "occVertVerts",
]


def parse(path):
    rows = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            if "MSOC: frame" not in line or "cellCross=" not in line:
                continue
            d = {k: int(v) for k, v in _KV.findall(line)}
            if "cellCross" in d:
                rows.append(d)
    return rows


def group_crossings(rows):
    """Split into runs that start at cellCross==0 and ascend."""
    crossings, cur = [], []
    for r in rows:
        age = r.get("cellCross", -1)
        if age == 0:
            if cur:
                crossings.append(cur)
            cur = [r]
        elif age > 0 and cur:
            cur.append(r)
        else:
            if cur:
                crossings.append(cur)
            cur = []
    if cur:
        crossings.append(cur)
    return crossings


def med(values):
    vals = [v for v in values if v is not None]
    return statistics.median(vals) if vals else 0


def fmt_us(v):
    return f"{v/1000:8.2f}ms" if v >= 1000 else f"{v:8.0f}us"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--crossings", action="store_true",
                    help="print a per-crossing breakdown in addition to the aggregate")
    ap.add_argument("--spike-ages", default="0,1",
                    help="cellCross ages treated as 'spike' (default 0,1)")
    ap.add_argument("--settled-ages", default="6,7",
                    help="cellCross ages treated as 'settled' baseline (default 6,7)")
    args = ap.parse_args()

    spike_ages = {int(x) for x in args.spike_ages.split(",")}
    settled_ages = {int(x) for x in args.settled_ages.split(",")}

    rows = parse(args.log)
    if not rows:
        print(f"No cell-cross lines in {args.log!r}.")
        print("  - Is EnableMSOC = true? (the culler must run to log anything)")
        print("  - Is OcclusionLogCellCross = true? (the MCM 'Log cell-cross spikes' toggle)")
        print("  - Is the profiling build deployed (msoc.dll) to this install?")
        return 1

    crossings = group_crossings(rows)
    print(f"Parsed {len(rows)} cell-cross lines across {len(crossings)} crossing(s).")

    # Per-AGE medians across crossings. The cross cost does NOT smear over the
    # whole window: the cache wipe + MSOC re-population land on age 0, and any
    # engine load/commit shows on a separate frame. Averaging a "spike window"
    # blends owners — so report each age independently and let the misses say
    # which frame is MSOC vs engine.
    by_age = {}  # age -> field -> [values across crossings]
    for c in crossings:
        for r in c:
            a = r.get("cellCross", -1)
            by_age.setdefault(a, {k: [] for k in TIMERS + MISSES})
            for k in TIMERS + MISSES:
                by_age[a][k].append(r.get(k, 0))
    ages = sorted(by_age)
    settled = med([r.get("frameDeltaUs", 0) for c in crossings for r in c
                   if r.get("cellCross", -1) in settled_ages]) or 0
    print(f"Settled baseline frameDelta (ages {sorted(settled_ages)}): {fmt_us(settled)}\n")

    def m(age, k):
        return med(by_age[age][k])

    print("=== PER-AGE MEDIAN (the cross frame is age 0; owner = who does the work) ===")
    print(f"  {'age':>3} {'frameUs':>10} {'vsSettled':>11} {'wipeUs':>8} {'xformUs':>8} "
          f"{'aggTerrUs':>9} {'classSteps':>10} {'landMiss':>8} {'occMiss':>7} {'occVerts':>8}")
    for a in ages:
        fd = m(a, "frameDeltaUs")
        print(f"  {a:>3} {fmt_us(fd):>10} {fmt_us(fd-settled):>11} "
              f"{m(a,'cellWipeUs'):>8.0f} {m(a,'occXformUs'):>8.0f} {m(a,'aggTerrainUs'):>9.0f} "
              f"{m(a,'classOccSteps'):>10.0f} {m(a,'landMembershipMiss'):>8.0f} "
              f"{m(a,'occCacheMiss'):>7.0f} {m(a,'occVertVerts'):>8.0f}")
    print()

    # Verdict: which age carries the MSOC re-population (most misses) vs which
    # age carries the worst frame-time spike. If they differ, the time spike
    # is not MSOC.
    miss_key = lambda a: m(a, "classOccSteps") + m(a, "occVertVerts")
    spike_age = max(ages, key=lambda a: m(a, "frameDeltaUs"))
    repop_age = max(ages, key=miss_key)
    print("=== VERDICT ===")
    print(f"  MSOC re-population frame : age {repop_age} "
          f"(+{fmt_us(m(repop_age,'frameDeltaUs')-settled).strip()} over settled, "
          f"{m(repop_age,'classOccSteps'):.0f} classify steps, "
          f"{m(repop_age,'occVertVerts'):.0f} occluder verts, "
          f"wipe {m(repop_age,'cellWipeUs'):.0f}us)")
    print(f"  Worst frame-time frame  : age {spike_age} "
          f"(+{fmt_us(m(spike_age,'frameDeltaUs')-settled).strip()} over settled, "
          f"{m(spike_age,'classOccSteps'):.0f} classify steps, "
          f"{m(spike_age,'occCacheMiss'):.0f} occ misses)")
    if spike_age != repop_age and miss_key(spike_age) < miss_key(repop_age) * 0.1:
        print(f"  => The worst spike (age {spike_age}) has ~no MSOC work; it is ENGINE,")
        print(f"     not MSOC. MSOC's own cross cost is the age-{repop_age} delta above.")
    print()

    if args.crossings:
        print("=== PER-CROSSING DETAIL ===")
        for i, c in enumerate(crossings):
            seq = " ".join(f"a{r.get('cellCross')}={fmt_us(r.get('frameDeltaUs',0)).strip()}"
                           for r in c)
            print(f"  crossing {i}: {seq}")
        print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
