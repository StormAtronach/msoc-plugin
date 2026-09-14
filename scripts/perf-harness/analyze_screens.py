"""Pixel diff of harness screenshots: batching on against off, same session.

Pairs `msocperf_<site>_<variant>_<rep>_b<bearing>.png` files from a results
directory: `batch` against `off` (culler off both) and `both` against `on`
(culler on both), per bearing. For each pair it prints the share of pixels
whose max channel difference exceeds a threshold and the mean absolute
difference, and writes a composite `<pair>_b<bearing>.png` with the two frames
side by side over an amplified difference image, downscaled for viewing.

Usage:
    python analyze_screens.py [results-dir] [--threshold 24] [--scale 0.5]
"""

import argparse
import glob
import os
import re
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS_ROOT = os.path.join(HERE, "results")
NAME_RE = re.compile(r"msocperf_(?P<site>.+?)_(?P<variant>on|batch|off|both)(?:_(?P<rep>\d+))?_b(?P<bearing>\d+)(?P<set>h?)\.png$")
# Toggle mode: msocperf_<site>_<pass>_b<bearing>_(on|off).png, same spot a second apart.
TOGGLE_RE = re.compile(r"msocperf_(?P<site>.+?)_b(?P<bearing>\d+)_(?P<state>on|off)\.png$")
PAIRS = (("batch", "off"), ("both", "on"))


def newest_results():
    dirs = [d for d in glob.glob(os.path.join(RESULTS_ROOT, "*")) if os.path.isdir(d) and not d.endswith("latest")]
    if not dirs:
        raise SystemExit("no results directories under %s" % RESULTS_ROOT)
    return max(dirs, key=os.path.getmtime)


def load(path):
    return np.asarray(Image.open(path).convert("RGB"), dtype=np.int16)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results", nargs="?", default=None)
    ap.add_argument("--threshold", type=int, default=24, help="per-pixel max channel delta that counts as different (default 24)")
    ap.add_argument("--scale", type=float, default=0.5, help="composite downscale factor (default 0.5)")
    args = ap.parse_args()
    results = args.results or newest_results()

    shots = {}
    for path in glob.glob(os.path.join(results, "msocperf_*_b*.png")):
        name = os.path.basename(path)
        m = TOGGLE_RE.search(name)
        if m:
            # Map toggle pairs onto the batch/off pair with rep 0.
            shots[(m.group("site"), "batch" if m.group("state") == "on" else "off", 0, int(m.group("bearing")))] = path
            continue
        m = NAME_RE.search(name)
        if m:
            shots[(m.group("site"), m.group("variant"), int(m.group("rep") or 0), int(m.group("bearing")) + (100 if m.group("set") == "h" else 0))] = path
    if not shots:
        print("no harness screenshots in %s" % results)
        return 1

    print("%-16s %-10s %3s %3s %10s %10s %s" % ("site", "pair", "rep", "brg", "pct>thr", "meanAbs", "composite"))
    for (site, variant, rep, bearing), path in sorted(shots.items()):
        for a_name, b_name in PAIRS:
            if variant != a_name:
                continue
            other = shots.get((site, b_name, rep, bearing))
            if not other:
                continue
            a = load(path)
            b = load(other)
            if a.shape != b.shape:
                print("%-16s %-10s %3d %3d  size mismatch %s vs %s" % (site, a_name + "/" + b_name, rep, bearing, a.shape, b.shape))
                continue
            diff = np.abs(a - b)
            maxc = diff.max(axis=2)
            pct = 100.0 * float((maxc > args.threshold).mean())
            mean_abs = float(diff.mean())
            # Composite: A | B on top, amplified diff below.
            amp = np.clip(diff * 4, 0, 255).astype(np.uint8)
            top = np.concatenate([a.astype(np.uint8), b.astype(np.uint8)], axis=1)
            bottom = np.concatenate([amp, np.repeat(np.clip(maxc * 4, 0, 255).astype(np.uint8)[:, :, None], 3, axis=2)], axis=1)
            comp = Image.fromarray(np.concatenate([top, bottom], axis=0))
            if args.scale != 1.0:
                comp = comp.resize((int(comp.width * args.scale), int(comp.height * args.scale)), Image.LANCZOS)
            out = os.path.join(results, "diff_%s_%s-vs-%s_%d_b%d.png" % (site, a_name, b_name, rep, bearing))
            comp.save(out)
            print("%-16s %-10s %3d %3d %9.2f%% %10.2f %s" % (site, a_name + "/" + b_name, rep, bearing, pct, mean_abs, os.path.basename(out)))
    print("(pct>thr: share of pixels differing by more than the threshold in any channel; composites: A | B above, 4x amplified diff below)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
