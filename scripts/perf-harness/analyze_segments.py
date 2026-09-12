"""Per-bearing breakdown of a rotating-view perf run.

`run_perf.py` reports one number per run. That number hides the thing a view-
rotating run exists to expose: an occlusion culler's payoff depends entirely on
what the camera is pointed at, so a site's mean is an average over eight very
different scenes. This reads the `SEGMENTS` lines the in-game harness leaves in
each saved `MWSE.log` and shows the eight separately.

Usage:  python scripts/perf-harness/analyze_segments.py [results-dir]
"""

import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS_ROOT = os.path.join(HERE, "results")


def newest_results():
    """The directory run_perf.py wrote last, or the root for older layouts."""
    pointer = os.path.join(RESULTS_ROOT, "latest")
    if os.path.isfile(pointer):
        name = open(pointer, encoding="utf-8").read().strip()
        path = os.path.join(RESULTS_ROOT, name)
        if os.path.isdir(path):
            return path
    subdirs = [os.path.join(RESULTS_ROOT, d) for d in os.listdir(RESULTS_ROOT)
               if os.path.isdir(os.path.join(RESULTS_ROOT, d))] \
        if os.path.isdir(RESULTS_ROOT) else []
    return max(subdirs, key=os.path.getmtime) if subdirs else RESULTS_ROOT

# SEGMENTS run=balmora/on#2 0:5.31 1:4.88 2:5.02 ...
SEG_RE = re.compile(r"SEGMENTS run=(?P<run>\S+)\s+(?P<pairs>[\d:.\s]+)")
BEARINGS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


def parse(results_dir):
    """-> {(site, variant): {segment_index: [ms, ms, ...one per repeat]}}"""
    runs = {}
    for name in sorted(os.listdir(results_dir)):
        if not name.endswith(".mwse.log"):
            continue
        path = os.path.join(results_dir, name)
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                m = SEG_RE.search(line)
                if not m:
                    continue
                run = m.group("run")
                # "site/variant#rep" -> ("site", "variant"); repeats merge.
                head = run.split("#", 1)[0]
                if "/" not in head:
                    continue
                site, variant = head.rsplit("/", 1)
                tagged = "#" in run
                key = (site, variant)
                # The results directory accumulates. A run labelled with a
                # repeat number came from a --repeat batch; an untagged one is
                # a single run from an earlier session, quite possibly at a
                # different vantage or before view rotation existed. Mixing
                # them silently inflates bearing 0. Tagged wins outright.
                seen_tagged = runs.get(key, (False, {}))[0]
                if seen_tagged and not tagged:
                    continue
                if tagged and not seen_tagged:
                    runs[key] = (True, {})
                runs.setdefault(key, (tagged, {}))
                bucket = runs[key][1]
                for pair in m.group("pairs").split():
                    idx, _, ms = pair.partition(":")
                    if ms:
                        bucket.setdefault(int(idx), []).append(float(ms))
    return {k: v[1] for k, v in runs.items()}


def label(idx, count):
    """Compass name when the run used the eight standard bearings."""
    if count == len(BEARINGS):
        return "%d %-2s" % (idx, BEARINGS[idx])
    return "%d   " % idx


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else newest_results()
    print("reading %s" % results_dir)
    if not os.path.isdir(results_dir):
        print("no results directory: %s" % results_dir)
        return 1

    runs = parse(results_dir)
    if not runs:
        print("no SEGMENTS lines found in %s" % results_dir)
        return 1

    sites = sorted({site for site, _ in runs})
    for site in sites:
        off = runs.get((site, "off"), {})
        on = runs.get((site, "on"), {})
        if not (off and on):
            print("\n%s: incomplete pair, skipping" % site)
            continue

        shared = sorted(set(off) & set(on))
        print("\n%s - per-bearing mean frame time" % site)
        print("  %-6s %9s %9s %9s %8s  %s"
              % ("view", "off ms", "on ms", "delta ms", "delta %", "repeats"))
        print("  " + "-" * 60)

        deltas = []
        for idx in shared:
            o = statistics.median(off[idx])
            n = statistics.median(on[idx])
            d = n - o
            deltas.append(d)
            print("  %-6s %9.2f %9.2f %+9.2f %+7.1f%%  %d/%d"
                  % (label(idx, len(shared)), o, n, d, 100.0 * d / o,
                     len(off[idx]), len(on[idx])))

        best, worst = min(deltas), max(deltas)
        print("  best view %+.2f ms, worst view %+.2f ms, spread %.2f ms"
              % (best, worst, worst - best))
        wins = sum(1 for d in deltas if d < 0)
        print("  culler is a win on %d of %d bearings" % (wins, len(deltas)))

    return 0


if __name__ == "__main__":
    sys.exit(main())
