"""Slope of frame time against hidden static draws, from a render-skip sweep.

Reads the MWSE logs a `run_perf.py --sweep renderskip[-small|-large]` run
copied into its results directory, pairs each pass's `RESULT run=...` line
with the `[RenderSkip] rebuild ...` line the probe printed when that pass's
percentage was applied (and the last `[RenderSkip] window ...` line before
the result, which carries the shapes actually skipped per frame), takes the
median over repeats, and fits

    meanMs = a + b * x

per site by least squares, for x = hidden population shapes, x = hidden
triangles, and x = shapes skipped at Display per frame. The last is the
per-draw cost that matters: a shape is only offered to Display when its
parent node passed the frustum test, so skipped-per-frame is the in-frustum
hidden draw count. Within one sweep the three regressors are collinear, so
read them as three expressions of one slope.

Given two results directories from the -small and -large sweeps, the two
per-site slopes are solved together for (ms per draw, ms per triangle),
since the two populations have very different triangles per draw. The
populations differ in other ways too, so that split is indicative.

Usage:
    python analyze_renderskip.py [results-dir ...]
With no argument the newest results directory is used.
"""

import glob
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS_ROOT = os.path.join(HERE, "results")

RESULT_RE = re.compile(
    r"RESULT run=(?P<run>\S+) frames=(?P<frames>\d+) meanMs=(?P<mean>[\d.]+) "
    r"p50Ms=(?P<p50>[\d.]+) p95Ms=(?P<p95>[\d.]+) p99Ms=(?P<p99>[\d.]+) meanFps=(?P<fps>[\d.]+)")
REBUILD_RE = re.compile(
    r"\[RenderSkip\] rebuild percent=(?P<percent>\d+) tris=\[(?P<minTris>\d+),(?P<maxTris>\d+)\) "
    r"eligibleRefs=(?P<eligibleRefs>\d+) partialRefs=(?P<partialRefs>\d+) "
    r"eligibleShapes=(?P<eligibleShapes>\d+) rejectedShapes=(?P<rejectedShapes>\d+) "
    r"eligibleTris=(?P<eligibleTris>\d+) hiddenShapes=(?P<hiddenShapes>\d+) hiddenTris=(?P<hiddenTris>\d+)")
WINDOW_RE = re.compile(
    r"\[RenderSkip\] window frames=(?P<frames>\d+) percent=(?P<percent>\d+) .*"
    r"skipped/frame=(?P<skipped>\d+) visited/frame=(?P<visited>\d+)")


def newest_results():
    dirs = [d for d in glob.glob(os.path.join(RESULTS_ROOT, "*")) if os.path.isdir(d) and not d.endswith("latest")]
    if not dirs:
        raise SystemExit("no results directories under %s" % RESULTS_ROOT)
    return max(dirs, key=os.path.getmtime)


def parse_dir(results_dir):
    """-> {(site, variant): [sample, ...]} one sample per repeat."""
    out = {}
    for log in sorted(glob.glob(os.path.join(results_dir, "*.mwse.log"))):
        rebuild = None
        window = None
        with open(log, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                m = REBUILD_RE.search(line)
                if m:
                    rebuild = {k: int(v) for k, v in m.groupdict().items()}
                    window = None
                    continue
                m = WINDOW_RE.search(line)
                if m:
                    window = {k: int(v) for k, v in m.groupdict().items()}
                    continue
                m = RESULT_RE.search(line)
                if not m:
                    continue
                run = m.group("run")
                head = run.split("#", 1)[0]
                site, _, variant = head.rpartition("/")
                sample = {
                    "mean": float(m.group("mean")),
                    "p50": float(m.group("p50")),
                    "p95": float(m.group("p95")),
                    "fps": float(m.group("fps")),
                    "frames": int(m.group("frames")),
                }
                if rebuild:
                    sample.update(rebuild)
                else:
                    # Baseline pass with the probe already applied at 0, or a log
                    # from a DLL without the probe: no hidden draws either way.
                    sample.update({"percent": 0, "hiddenShapes": 0, "hiddenTris": 0,
                                   "eligibleShapes": 0, "eligibleTris": 0})
                if window:
                    sample["skippedPerFrame"] = window["skipped"]
                    sample["visitedPerFrame"] = window["visited"]
                out.setdefault((site, variant), []).append(sample)
    return out


def median_samples(samples):
    agg = {}
    keys = set()
    for s in samples:
        keys.update(s.keys())
    for k in keys:
        vals = [s[k] for s in samples if k in s]
        if vals:
            agg[k] = statistics.median(vals)
    agg["reps"] = len(samples)
    agg["spread"] = (max(s["mean"] for s in samples) - min(s["mean"] for s in samples)) if len(samples) > 1 else 0.0
    return agg


def fit(xs, ys):
    """Least-squares slope and intercept; None when x has no variance."""
    n = len(xs)
    if n < 2:
        return None, None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None, None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    b = sxy / sxx
    return b, my - b * mx


def per_site(parsed):
    """-> {site: {"rows": [...], "slopeDraw": b1, "slopeTri": b2, "slopeSkip": b3, ...}}"""
    sites = {}
    for (site, variant), samples in parsed.items():
        sites.setdefault(site, []).append((variant, median_samples(samples)))
    out = {}
    for site, rows in sites.items():
        rows.sort(key=lambda r: r[1].get("hiddenShapes", 0))
        xs = [r[1].get("hiddenShapes", 0) for r in rows]
        ts = [r[1].get("hiddenTris", 0) for r in rows]
        ks = [r[1].get("skippedPerFrame", 0) for r in rows]
        ys = [r[1]["mean"] for r in rows]
        bd, _ = fit(xs, ys)
        bt, _ = fit(ts, ys)
        bk, _ = fit(ks, ys)
        out[site] = {"rows": rows, "slopeDraw": bd, "slopeTri": bt, "slopeSkip": bk,
                     "hiddenMax": max(xs) if xs else 0, "trisMax": max(ts) if ts else 0,
                     "skipMax": max(ks) if ks else 0,
                     "trisPerDraw": (max(ts) / max(xs)) if xs and max(xs) else 0,
                     "baseMs": ys[0] if ys else 0.0,
                     "fullMs": ys[-1] if ys else 0.0}
    return out


def print_dir(results_dir, sites):
    print("\n== %s" % results_dir)
    print("%-16s %-8s %5s %10s %10s %9s %8s %8s %8s %6s" % (
        "site", "variant", "pct", "hidden", "hiddenTris", "skip/frm", "mean ms", "p95 ms", "fps", "reps"))
    for site, info in sorted(sites.items()):
        for variant, agg in info["rows"]:
            print("%-16s %-8s %5d %10d %10d %9d %8.2f %8.2f %8.1f %6d  spread %.2f" % (
                site, variant, int(agg.get("percent", 0)), int(agg.get("hiddenShapes", 0)),
                int(agg.get("hiddenTris", 0)), int(agg.get("skippedPerFrame", 0)),
                agg["mean"], agg.get("p95", 0.0), agg.get("fps", 0.0),
                int(agg["reps"]), agg["spread"]))
    print("\n%-16s %13s %12s %14s %10s %9s %10s %10s" % (
        "site", "us/shape(pop)", "us/tri(pop)", "us/skipped-frm", "tris/shape", "max skip", "base ms", "all-hidden"))
    for site, info in sorted(sites.items()):
        bd, bt, bk = info["slopeDraw"], info["slopeTri"], info["slopeSkip"]
        print("%-16s %13s %12s %14s %10.1f %9d %10.2f %10.2f" % (
            site,
            ("%.3f" % (bd * 1000.0)) if bd is not None else "n/a",
            ("%.4f" % (bt * 1000.0)) if bt is not None else "n/a",
            ("%.3f" % (bk * 1000.0)) if bk is not None else "n/a",
            info["trisPerDraw"], info["skipMax"], info["baseMs"], info["fullMs"]))
    print("(negative = frame got faster as draws were hidden. us/skipped-frm is the slope against shapes")
    print(" actually skipped at Display per frame, i.e. in-frustum hidden draws: the per-draw cost.)")


def solve_split(a, b):
    """Two sweeps with different tris/draw: solve slope = d + t * trisPerDraw per site."""
    print("\n== per-draw / per-triangle split (two sweeps solved together, population slopes)")
    print("%-16s %14s %14s %s" % ("site", "us per draw", "us per tri", "condition"))
    for site in sorted(set(a) & set(b)):
        s1, r1 = a[site]["slopeDraw"], a[site]["trisPerDraw"]
        s2, r2 = b[site]["slopeDraw"], b[site]["trisPerDraw"]
        if s1 is None or s2 is None or r1 == r2:
            print("%-16s %14s %14s %s" % (site, "n/a", "n/a", "missing slope or identical tris/draw"))
            continue
        t = (s1 - s2) / (r1 - r2)
        d = s1 - t * r1
        cond = abs(r1 - r2) / max(r1, r2, 1e-9)
        print("%-16s %14.3f %14.4f r1=%.1f r2=%.1f sep=%.2f" % (site, d * 1000.0, t * 1000.0, r1, r2, cond))
    print("(sep near 0 means the two populations had similar tris/draw and the split is ill-conditioned)")


def main():
    dirs = sys.argv[1:] or [newest_results()]
    analysed = []
    for d in dirs:
        parsed = parse_dir(d)
        if not parsed:
            print("no RESULT lines with RenderSkip context in %s" % d)
            continue
        sites = per_site(parsed)
        print_dir(d, sites)
        analysed.append(sites)
    if len(analysed) == 2:
        solve_split(analysed[0], analysed[1])
    return 0


if __name__ == "__main__":
    sys.exit(main())
