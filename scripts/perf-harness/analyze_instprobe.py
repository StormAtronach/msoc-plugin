#!/usr/bin/env python3
"""Summarise MGE's instancing-feasibility probe from a results directory.

Reads the `*.mge.log` files a `--instprobe` run leaves behind and reports what
the draw stream would collapse to under three grouping keys:

    geom    buffer identity and primitive range - the ceiling that ignores
            whether merging would be correct
    state   the above plus every resolved render-state and texture-stage value
    light   the above plus the exact active light set

`light` is the number that decides whether instancing is worth building without
a lighting rewrite, because Morrowind pushes lights per object and instances
under different lights cannot share a draw. The gap between `geom` and `light`
is what index-based pooled lighting would have to recover.

One session holds every variant it ran, and the probe writes into MGE's own log
with no per-pass marker, so a multi-variant session cannot be attributed. Run
one variant per session (`--only on`) when the split matters.

Usage:
    python analyze_instprobe.py                 # results/latest
    python analyze_instprobe.py <results-dir>
"""

import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RESULTS_ROOT = os.path.join(REPO, "scripts", "perf-harness", "results")

HEAD_RE = re.compile(
    r"INSTPROBE (?P<tag>\w+) frames=(?P<frames>\d+) draws=(?P<draws>[\d.]+) "
    r"opq=(?P<opq>[\d.]+) geom=(?P<geom>[\d.]+) state=(?P<state>[\d.]+) "
    r"light=(?P<light>[\d.]+)")
COLLAPSE_RE = re.compile(
    r"INSTPROBE (?P<tag>\w+) collapse geom=(?P<geom>[\d.]+)x "
    r"state=(?P<state>[\d.]+)x light=(?P<light>[\d.]+)x")
OPAQUE_RE = re.compile(
    r"INSTPROBE (?P<tag>\w+) opaque geom=(?P<geom>[\d.]+) state=(?P<state>[\d.]+) "
    r"light=(?P<light>[\d.]+) cGeom=(?P<cgeom>[\d.]+)x cState=(?P<cstate>[\d.]+)x "
    r"cLight=(?P<clight>[\d.]+)x")
GROUPS_RE = re.compile(
    r"INSTPROBE (?P<tag>\w+) groups g1=(?P<g1>[\d.]+) g2=(?P<g2>[\d.]+) "
    r"g34=(?P<g34>[\d.]+) g58=(?P<g58>[\d.]+) g916=(?P<g916>[\d.]+) "
    r"g17=(?P<g17>[\d.]+) maxgrp=(?P<maxgrp>\d+) inGroups=(?P<ingroups>[\d.]+)%")
LIGHTS_RE = re.compile(
    r"INSTPROBE (?P<tag>\w+) lights avg=(?P<avg>[\d.]+) max=(?P<max>\d+) "
    r"zeroLit=(?P<zero>[\d.]+)%")


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def summarise(path, busy_only=False, busy_frac=0.5):
    """Fold the windows in one log into a frame-weighted mean.

    A probe session is not homogeneous. It opens on the main menu, passes
    through load screens, then rotates the camera through eight bearings in the
    world, and each phase has a completely different draw population - the menu
    alone reported a 5.1x geometry ratio against ~1.9x in the world, because a
    menu backdrop is a handful of meshes repeated.

    Averaging across all of that describes no scene that exists. With
    `busy_only`, only windows carrying at least `busy_frac` of the session's
    peak draw count are folded in, which isolates the full-frustum world views
    that dominate frame cost and are the only ones an optimisation would target.
    """
    text = open(path, encoding="utf-8", errors="replace").read()

    heads = [m.groupdict() for m in HEAD_RE.finditer(text) if m.group("tag") == "win"]
    if not heads:
        return None

    keep = list(range(len(heads)))
    if busy_only:
        peak = max(float(h["draws"]) for h in heads)
        keep = [i for i, h in enumerate(heads) if float(h["draws"]) >= busy_frac * peak]
        if not keep:
            return None

    # Captured before `heads` is reassigned: pick() matches a row family against
    # the *unfiltered* window count, and comparing against the filtered length
    # would silently pass every detail family through unfiltered.
    window_count = len(heads)

    def pick(rows):
        return [rows[i] for i in keep] if rows and len(rows) == window_count else rows

    heads = pick(heads)
    frames = [float(h["frames"]) for h in heads]
    total = sum(frames)

    def wmean(key, rows):
        # Weight each window by its frame count; windows are equal-sized except
        # possibly the last, so an unweighted mean would overstate a short tail.
        if not rows or len(rows) != len(frames):
            return mean([float(r[key]) for r in rows]) if rows else 0.0
        return sum(float(r[key]) * f for r, f in zip(rows, frames)) / total

    collapses = pick([m.groupdict() for m in COLLAPSE_RE.finditer(text) if m.group("tag") == "win"])
    opaques = pick([m.groupdict() for m in OPAQUE_RE.finditer(text) if m.group("tag") == "win"])
    groups = pick([m.groupdict() for m in GROUPS_RE.finditer(text) if m.group("tag") == "win"])
    lights = pick([m.groupdict() for m in LIGHTS_RE.finditer(text) if m.group("tag") == "win"])

    out = {
        "windows": len(heads),
        "frames": int(total),
        "draws": wmean("draws", heads),
        "opq": wmean("opq", heads),
        "cGeom": wmean("geom", collapses),
        "cState": wmean("state", collapses),
        "cLight": wmean("light", collapses),
        "oGeom": wmean("cgeom", opaques),
        "oState": wmean("cstate", opaques),
        "oLight": wmean("clight", opaques),
        "inGroups": wmean("ingroups", groups),
        "maxgrp": max((int(g["maxgrp"]) for g in groups), default=0),
        "lightAvg": wmean("avg", lights),
        "lightMax": max((int(l["max"]) for l in lights), default=0),
        "zeroLit": wmean("zero", lights),
    }

    # The line that matters: how many draws survive if only same-light groups
    # merge. inGroups is the share of opaque draws sitting in a group above one.
    out["opqAfter"] = out["opq"] / out["oLight"] if out["oLight"] > 0 else 0.0
    out["removed"] = out["opq"] - out["opqAfter"]
    return out


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else None
    if not target:
        pointer = os.path.join(RESULTS_ROOT, "latest")
        if not os.path.isfile(pointer):
            print("no results/latest; pass a results directory")
            return 1
        target = os.path.join(RESULTS_ROOT, open(pointer, encoding="utf-8").read().strip())

    logs = sorted(f for f in os.listdir(target) if f.endswith(".mge.log"))
    if not logs:
        print("no .mge.log in %s - was the run made with --instprobe?" % target)
        return 1

    rows = []
    for name in logs:
        base = name[:-len(".mge.log")]
        for label, busy in ((base, False), (base + " (world)", True)):
            r = summarise(os.path.join(target, name), busy_only=busy)
            if r:
                rows.append((label, r))

    if not rows:
        print("no INSTPROBE windows found in %s" % target)
        print("the probe is off unless MGE_INSTPROBE=1 reached the game, and it")
        print("needs a d3d8.dll built with instprobe.cpp")
        return 1

    print("%-16s %6s %8s %8s %8s %8s %8s" %
          ("site", "frames", "draws", "opaque", "cGeom", "cState", "cLight"))
    for name, s in rows:
        print("%-16s %6d %8.1f %8.1f %7.2fx %7.2fx %7.2fx" %
              (name, s["frames"], s["draws"], s["opq"],
               s["cGeom"], s["cState"], s["cLight"]))

    print("")
    print("%-16s %8s %8s %8s %9s %7s %8s %8s" %
          ("site (opaque)", "oGeom", "oState", "oLight", "inGroups",
           "maxgrp", "after", "removed"))
    for name, s in rows:
        print("%-16s %7.2fx %7.2fx %7.2fx %8.1f%% %7d %8.1f %8.1f" %
              (name, s["oGeom"], s["oState"], s["oLight"], s["inGroups"],
               s["maxgrp"], s["opqAfter"], s["removed"]))

    print("")
    print("%-16s %10s %10s %10s" % ("site", "lightsAvg", "lightsMax", "zeroLit"))
    for name, s in rows:
        print("%-16s %10.2f %10d %9.1f%%" %
              (name, s["lightAvg"], s["lightMax"], s["zeroLit"]))

    print("")
    print("Rows marked (world) fold only windows at >=50%% of the session's peak")
    print("draw count - the full-frustum views. The unmarked row includes menu")
    print("and load-screen frames and describes no scene that actually renders.")
    print("")
    print("cLight / oLight is the correctness-preserving collapse: only draws")
    print("with an identical active light set are merged. 'after' is the opaque")
    print("draw count that would remain, 'removed' what instancing would delete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
