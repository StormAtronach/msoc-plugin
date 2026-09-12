#!/usr/bin/env python3
"""Automated performance measurement for the msoc plugin.

Launches Morrowind through Mod Organizer, auto-loads a save, travels to a
settlement, and measures frame time under each config variant in turn. Prints a
comparison at the end.

Five things this gets right that a naive timing run does not:

* **Vsync is disabled.** MGE XE G7 keeps `vsync = "one"` in mgeXE.toml, and a
  vsync-capped session reports the refresh interval no matter what the culler
  does - the first run of this harness produced 6.98 ms with the culler off and
  6.99 ms with it on, which is a 144 Hz monitor rather than a measurement. The
  driver rewrites that one key for the duration and restores the file
  afterwards, and also passes DXVK_CONFIG so the layer below cannot re-impose
  it. If the numbers still look quantised it says so rather than letting you
  read a display refresh as a result.

* **Frame timing is measured in-game**, by the Lua half, not read out of
  MSOC.log. The plugin's stats line only exists on frames where the culler
  runs, so an EnableMSOC=false run emits none - and that run is the baseline
  everything else is measured against.

* **The camera is pinned and the sites are fixed**, so two runs are comparable.
  Sites come from `sites.json`, written by `--scan`, which picks the densest
  cell per town by reference count: `coc` by name lands wherever the lookup
  resolves, which in the first run of this harness meant a 22-reference corner
  of Balmora against the real cell's 483.

* **The view rotates** through eight bearings during the window. The sign of the
  result can flip between them - at Old Ebonheart the culler is worth -1.87 ms
  facing north and +0.78 ms facing south-east - so a single-view number is a
  measurement of one view, not of the site. `analyze_segments.py` prints the
  per-bearing breakdown from the saved logs.

* **One game session covers every variant and repeat at a site.** A launch costs
  about 100 seconds to buy a 24 second sample, so measuring one config per
  process spent an hour to measure ten minutes. `EnableMSOC` and the rest are
  live, so the session travels once and walks the pass list, re-warming after
  each change and interleaving variants so session drift lands on both sides of
  a comparison. Checked against the old way at Old Ebonheart: the two agree
  within 0.12 ms, and the shared session is the tighter of the two. `--relaunch`
  restores a process per measurement.

Each invocation writes to `results/<timestamp>/` and updates `results/latest`.

Usage:
    python run_perf.py                          # all sites, culler on vs off
    python run_perf.py --sweep                  # one knob at a time
    python run_perf.py --sweep sync             # same, rasterizer on the main
                                                # thread, as low tier runs it
    python run_perf.py --sites balmora,vivec    # a subset
    python run_perf.py --scan                   # rebuild sites.json
    python run_perf.py --here                   # wherever the save already is
    python run_perf.py --sample 45 --vsync      # longer window, keep vsync
    python scripts/perf-harness/analyze_segments.py   # per-bearing breakdown

Mask resolution latches at install, so sweeping it needs `--relaunch`.
"""

import argparse
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import time

# --- paths ------------------------------------------------------------------

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HARNESS_SRC = os.path.join(REPO, "scripts", "perf-harness", "msocperf")

MO2_ROOT = r"D:\Modlists\Morrowind75testing"
MO2_EXE = os.path.join(MO2_ROOT, "ModOrganizer.exe")
MO2_SHORTCUT = "moshortcut://:Morrowind"
DEPLOY_MOD = os.path.join(MO2_ROOT, "mods", "test-mod")
DEPLOY_HARNESS = os.path.join(DEPLOY_MOD, "MWSE", "mods", "msocperf")

# MGE XE G7 keeps its configuration in TOML, served to the game root by MO2.
MGE_TOML = os.path.join(MO2_ROOT, "mods", "Configuration", "Root", "mgeXE.toml")
MGE_TOML_BACKUP = MGE_TOML + ".perfbak"

GAME_ROOT = r"D:\GOG\Morrowind\Morrowind"
MWSE_LOG = os.path.join(GAME_ROOT, "MWSE.log")
MSOC_LOG = os.path.join(GAME_ROOT, "MSOC.log")

# The spec is delivered as an MWSE config file inside the deployed mod, so the
# game reads it through MO2's virtual file system at
# Data Files/MWSE/config/msocperf_spec.json.
SPEC_DIR = os.path.join(DEPLOY_MOD, "MWSE", "config")
SPEC_FILE = os.path.join(SPEC_DIR, "msocperf_spec.json")

RESULTS_ROOT = os.path.join(REPO, "scripts", "perf-harness", "results")
# Set by main() to results/<timestamp>. One directory per invocation, because
# a run's logs are only comparable with the logs from the same run: naming
# schemes differ between the multipass and --relaunch paths, and the analyser
# would otherwise average this run against every previous one.
RESULTS = RESULTS_ROOT
LATEST_POINTER = os.path.join(RESULTS_ROOT, "latest")


def start_results_dir():
    """Create this invocation's results directory and point `latest` at it."""
    global RESULTS
    RESULTS = os.path.join(RESULTS_ROOT,
                           time.strftime("%Y%m%d-%H%M%S", time.localtime()))
    os.makedirs(RESULTS, exist_ok=True)
    with open(LATEST_POINTER, "w", encoding="utf-8") as fh:
        fh.write(os.path.basename(RESULTS) + "\n")
    return RESULTS

SYNC = {"EnableMSOC": True, "OcclusionAsyncOccluders": False}


def _sync(**kw):
    """A sync-baseline variant with extra keys layered on top."""
    d = dict(SYNC)
    d.update(kw)
    return d


# --sweep sync: the same knobs with the rasterizer on the main thread, which is
# how the low-tier preset runs. The baseline keeps the name "on" so the summary
# compares against it rather than against the async default.
SWEEP_SYNC_VARIANTS = [
    ("off", {"EnableMSOC": False}),
    ("on", dict(SYNC)),
    # Raster-side knobs. Under async these vanished into the noise because
    # submission is enqueue-and-forget; here the work is on this thread.
    ("no-ccw", _sync(OcclusionOccluderCCWOnly=False)),
    ("no-f2b", _sync(OcclusionOccluderFrontToBack=False)),
    # Terrain, including the mode low tier actually defaults to.
    ("horizon", _sync(OcclusionAggregateTerrain=2)),
    ("terrain-off", _sync(OcclusionAggregateTerrain=0)),
    # Occludee-side knobs.
    ("no-box", _sync(OcclusionOccludeeBoxTest=False)),
    ("skip-terrain", _sync(OcclusionSkipTerrainOccludees=True)),
    # The low-tier phase budgets. Predictive skip and spike clip only engage
    # when a phase is expensive, which under async it never was.
    ("budgets", _sync(OcclusionRasterizeBudgetUs=1500,
                      OcclusionClassifyBudgetUs=1500)),
    # The whole low-tier profile bar the mask size, which cannot be set here.
    ("low-tier", _sync(OcclusionAggregateTerrain=2,
                       OcclusionSkipTerrainOccludees=True,
                       OcclusionRasterizeBudgetUs=1500,
                       OcclusionClassifyBudgetUs=1500)),
]

# --- what to measure --------------------------------------------------------

DEFAULT_VARIANTS = [
    ("off", {"EnableMSOC": False}),
    ("on", {"EnableMSOC": True}),
]

# --sweep: one knob at a time against the default-on config, to find where the
# plugin's cost actually sits. Every entry sets EnableMSOC explicitly because a
# pass restores the swept keys to their startup values and then applies its own,
# so what is not named here is whatever msoc.json holds.
#
# Mask resolution is deliberately absent: it latches at install, so sweeping it
# needs a process per value. Run that separately with --relaunch.
SWEEP_VARIANTS = [
    ("off", {"EnableMSOC": False}),
    ("on", {"EnableMSOC": True}),
    # Where does the mask get built, and is it worth building?
    ("sync", {"EnableMSOC": True, "OcclusionAsyncOccluders": False}),
    ("terrain-off", {"EnableMSOC": True, "OcclusionAggregateTerrain": 0}),
    ("horizon", {"EnableMSOC": True, "OcclusionAggregateTerrain": 2}),
    # Occluder submission throughput.
    ("no-f2b", {"EnableMSOC": True, "OcclusionOccluderFrontToBack": False}),
    ("no-ccw", {"EnableMSOC": True, "OcclusionOccluderCCWOnly": False}),
    # Occludee query cost: the drain is the biggest single phase.
    ("no-box", {"EnableMSOC": True, "OcclusionOccludeeBoxTest": False}),
    ("skip-terrain", {"EnableMSOC": True, "OcclusionSkipTerrainOccludees": True}),
    ("no-coherence", {"EnableMSOC": True, "OcclusionTemporalCoherenceFrames": 0}),
    # Front-to-back looks like a net loss under async, and the flush column says
    # why: deferring every occluder to after traversal collapses the overlap
    # between traversal and worker rasterization, so asyncFlushUs goes from
    # ~14us to ~100us. That mechanism cannot apply when rasterization is
    # synchronous, where the sort should be a pure win. This pair tests that.
    ("sync-no-f2b", {"EnableMSOC": True, "OcclusionAsyncOccluders": False,
                     "OcclusionOccluderFrontToBack": False}),
    # The two candidate default changes together, to check they compose rather
    # than each recovering the same time.
    ("lean", {"EnableMSOC": True, "OcclusionOccluderFrontToBack": False,
              "OcclusionTemporalCoherenceFrames": 0}),
]

# The places that actually hurt: dense exterior architecture, which is where a
# CPU occlusion culler either earns its keep or does not. Several candidate cell
# names per site because Tamriel Rebuilt and vanilla name things differently; a
# site whose cells do not resolve is reported as skipped rather than silently
# measured somewhere else.
DEFAULT_SITES = [
    {"name": "balmora", "cells": ["Balmora"]},
    {"name": "vivec", "cells": ["Vivec, Foreign Quarter", "Vivec"]},
    {"name": "narsis", "cells": ["Narsis"]},
    {"name": "old-ebonheart", "cells": ["Old Ebonheart"]},
]

SITES_FILE = os.path.join(REPO, "scripts", "perf-harness", "sites.json")

SCAN_RE = re.compile(r"SCAN town=(?P<town>.+?) refs=(?P<refs>\d+) x=(?P<x>-?\d+) y=(?P<y>-?\d+) name=(?P<name>.*)$")

RESULT_RE = re.compile(
    r"RESULT run=(?P<run>\S+) frames=(?P<frames>\d+) meanMs=(?P<mean>[\d.]+) "
    r"p50Ms=(?P<p50>[\d.]+) p95Ms=(?P<p95>[\d.]+) p99Ms=(?P<p99>[\d.]+) "
    r"meanFps=(?P<fps>[\d.]+)"
)
PASS_RE = re.compile(r"PASS (?P<i>\d+)/(?P<n>\d+) run=(?P<run>\S+)")
VSYNC_RE = re.compile(r'^(\s*vsync\s*=\s*)"[^"]*"', re.MULTILINE)


# --- vsync ------------------------------------------------------------------

def disable_vsync():
    """Point mgeXE.toml at an immediate present interval, keeping a backup.

    Returns True if the file was changed, so the caller knows to restore it.
    """
    if not os.path.isfile(MGE_TOML):
        print("  note: %s not found; cannot disable MGE vsync" % MGE_TOML)
        return False
    text = open(MGE_TOML, encoding="utf-8").read()
    m = VSYNC_RE.search(text)
    if not m:
        print("  note: no vsync key in mgeXE.toml; leaving it alone")
        return False
    if '"immediate"' in m.group(0):
        return False  # already off, nothing to restore
    shutil.copy(MGE_TOML, MGE_TOML_BACKUP)
    open(MGE_TOML, "w", encoding="utf-8").write(
        VSYNC_RE.sub(r'\1"immediate"', text, count=1))
    print("  mgeXE.toml vsync -> immediate (original backed up)")
    return True


def restore_vsync():
    if os.path.isfile(MGE_TOML_BACKUP):
        shutil.move(MGE_TOML_BACKUP, MGE_TOML)
        print("mgeXE.toml restored")


def child_env(vsync):
    """Environment for the game process.

    DXVK reads DXVK_CONFIG for inline overrides, so the layer below MGE cannot
    re-impose a present interval, and nothing is written into the install.
    """
    env = dict(os.environ)
    if not vsync:
        env["DXVK_CONFIG"] = "d3d9.presentInterval = 0"
    return env


# --- session plumbing -------------------------------------------------------

def _running(exe):
    out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq " + exe],
                         capture_output=True, text=True).stdout
    return exe.lower() in out.lower()


def kill_game():
    """Kill the game and MO2, and wait until they are really gone.

    Launching while a previous MO2 is still shutting down silently does
    nothing, which showed up as a variant that produced no log at all.
    """
    for exe in ("Morrowind.exe", "ModOrganizer.exe"):
        subprocess.run(["taskkill", "/F", "/IM", exe],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(20):
        time.sleep(1)
        if not (_running("Morrowind.exe") or _running("ModOrganizer.exe")):
            break
    time.sleep(2)


def launch(args):
    """Start the game, retrying once if the process never appears."""
    for attempt in (1, 2):
        subprocess.Popen([MO2_EXE, MO2_SHORTCUT], env=child_env(args.vsync),
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(30):
            time.sleep(1)
            if _running("Morrowind.exe"):
                return True
        print("    launch attempt %d produced no game process; retrying" % attempt)
        kill_game()
    return False


def deploy_harness():
    if os.path.isdir(DEPLOY_HARNESS):
        shutil.rmtree(DEPLOY_HARNESS)
    shutil.copytree(HARNESS_SRC, DEPLOY_HARNESS)


def remove_harness():
    if os.path.isdir(DEPLOY_HARNESS):
        shutil.rmtree(DEPLOY_HARNESS)
    if os.path.isfile(SPEC_FILE):
        os.remove(SPEC_FILE)


def run_site(label, variants, args, site=None, mode="measure", repeats=1):
    """One game session covering every variant and repeat at one site.

    Returns a list of parsed result dicts, one per pass, in the order the
    harness ran them. A pass that failed simply has no RESULT line and is
    absent from the list.
    """
    kill_game()
    for f in (MWSE_LOG, MSOC_LOG):
        if os.path.isfile(f):
            os.remove(f)

    spec = {
        "run": label,
        "save": args.save,
        "warmupSeconds": args.warmup,
        "sampleSeconds": args.sample,
        "settleSeconds": args.settle,
        "gameHour": args.hour,
        "variants": [{"name": n, "config": c} for n, c in variants],
        "repeats": repeats,
        "site": site,
        "mode": mode,
        "minRefs": args.min_refs,
        "rotateEveryFrames": args.rotate_every,
        "viewSegments": args.views,
    }
    os.makedirs(SPEC_DIR, exist_ok=True)
    with open(SPEC_FILE, "w", encoding="utf-8") as fh:
        json.dump(spec, fh, indent=2)

    print("  launching...", flush=True)
    if not launch(args):
        print("  game never started")
        return None

    # Fixed overhead before the first sample: MO2 hand-off, engine and mod
    # init, the 8s pre-load delay, the save load, travel, and a settle.
    # Measured ~60-80s on the reference install, so budget generously - a
    # session killed one second before its last pass reports throws away every
    # pass in it, which is now the whole site.
    passes = max(1, len(variants) * repeats)
    per_pass = args.settle + args.warmup + args.sample + 10
    budget = 180 + passes * per_pass
    deadline = time.time() + budget
    started = time.time()
    result, progress = None, None
    while time.time() < deadline:
        time.sleep(3)
        if not os.path.isfile(MWSE_LOG):
            continue
        try:
            text = open(MWSE_LOG, encoding="utf-8", errors="replace").read()
        except OSError:
            continue

        # Surface the harness's own progress so a stall is visible while it
        # happens rather than only as a timeout at the end. Report the furthest
        # stage reached: scanning for "any marker that differs from the last
        # one printed" flip-flops, because every earlier stage is still in the
        # log too.
        # With several passes per session the interesting progress marker is
        # which pass is running, not which stage - the stages now repeat.
        stage = None
        for m in PASS_RE.finditer(text):
            stage = "pass %s/%s %s" % (m.group("i"), m.group("n"), m.group("run"))
        if stage is None:
            for marker in ("site ", "initialized"):
                if "[msocperf] " + marker in text:
                    stage = marker
                    break
        done = len(RESULT_RE.findall(text))
        if stage:
            stage = "%s  (%d/%d reported)" % (stage, done, passes)
        if stage and stage != progress:
            progress = stage
            print("    %3ds  %s" % (time.time() - started, stage.strip()), flush=True)

        if "SKIPPED site unavailable" in text:
            print("    site unavailable - skipped")
            break

        if mode == "scan" and "[msocperf] SCAN complete" in text:
            result = {"scan": True}
            break

        if "[msocperf] DONE" in text:
            result = [m.groupdict() for m in RESULT_RE.finditer(text)]
            break

    kill_game()

    os.makedirs(RESULTS, exist_ok=True)
    safe = label.replace("/", "_")
    for src, tag in ((MWSE_LOG, "mwse"), (MSOC_LOG, "msoc")):
        if os.path.isfile(src):
            shutil.copy(src, os.path.join(RESULTS, "%s.%s.log" % (safe, tag)))

    if not result:
        print("  no result (timeout or skip)")
        return [] if mode != "scan" else None

    if mode == "scan":
        return result

    msoc_log = os.path.join(RESULTS, "%s.msoc.log" % safe)
    for r in result:
        r["plugin"] = plugin_costs(msoc_log, r["run"])
        print("  %-24s mean %.2f ms  p95 %.2f ms  (%.1f fps, %s frames)"
              % (r["run"], float(r["mean"]), float(r["p95"]),
                 float(r["fps"]), r["frames"]))
    if len(result) < passes:
        print("  warning: %d of %d passes reported" % (len(result), passes))
    return result


def plugin_costs(msoc_log, run=None):
    """Median of the plugin's own per-phase timers across its stats lines.

    Empty for an EnableMSOC=false pass, which emits no stats lines at all.
    """
    if not os.path.isfile(msoc_log):
        return {}
    # Only the stats lines between this pass's markers belong to it. The plugin
    # emits a line every 300 frames from startup, and one session now holds
    # every pass, so an unlabelled split would attribute the whole log to the
    # first one.
    text = open(msoc_log, encoding="utf-8", errors="replace").read()
    start = "MSOC MARK SAMPLE-START %s" % run if run else "MSOC MARK SAMPLE-START"
    end = "MSOC MARK SAMPLE-END %s" % run if run else "MSOC MARK SAMPLE-END"
    if start in text:
        text = text.split(start, 1)[1]
        text = text.split(end, 1)[0]
    keys = ("rasterizeUs", "drainUs", "classifyUs", "displayUs", "asyncFlushUs",
            "aggTerrainUs", "horizonBuildUs")
    acc = {k: [] for k in keys}
    cull, rast = [], []
    for line in text.splitlines():
        if not line.startswith("MSOC: frame "):
            continue
        fields = dict(tok.split("=", 1) for tok in line.split() if "=" in tok)
        for k in keys:
            try:
                acc[k].append(float(fields[k]))
            except (KeyError, ValueError):
                pass
        try:
            rast.append(float(fields["rasterized"]))
        except (KeyError, ValueError):
            pass
        occ = fields.get("queryOccluded", "")
        if "/" in occ:
            a, b = occ.split("/", 1)
            try:
                a, b = int(a), int(b)
                if b:
                    cull.append(100.0 * a / b)
            except ValueError:
                pass
    out = {k: statistics.median(v) for k, v in acc.items() if v}
    if cull:
        out["cullPct"] = statistics.median(cull)
    if rast:
        out["occluders"] = statistics.median(rast)
    out["windows"] = len(cull)
    return out


def noise_floor(results, site):
    """Smallest frame-time difference this run can actually resolve, in ms.

    Estimated from the repeats, which is the only non-circular source: for a
    variant measured twice, |a - b| / sqrt(2) estimates the per-run standard
    deviation, and pooling that over every variant at the site gives a decent
    sigma without assuming which variants "should" be equal.

    Returns None when nothing was repeated - with one run per variant there is
    no way to know, and saying so is better than implying precision.
    """
    diffs = [r["repMeans"] for r in results
             if r.get("site") == site and len(r.get("repMeans") or []) >= 2]
    if not diffs:
        return None
    sq = []
    for means in diffs:
        # Pairwise across however many repeats there were.
        for i in range(len(means)):
            for j in range(i + 1, len(means)):
                sq.append((means[i] - means[j]) ** 2 / 2.0)
    if not sq:
        return None
    sigma = (sum(sq) / len(sq)) ** 0.5
    reps = min(len(m) for m in diffs)
    # ~95% two-sided for a difference of two medians at this repeat count.
    return 2.8 * sigma / (reps ** 0.5)


def warn_if_capped(results):
    """A capped session reports the refresh interval, not the workload.

    The tell is every variant sitting on a common refresh interval with almost
    no spread between them.
    """
    caps = {60: 16.667, 75: 13.333, 120: 8.333, 144: 6.944, 165: 6.061, 240: 4.167}
    p50s = [float(r["p50"]) for r in results]
    if len(p50s) < 2:
        return
    spread = (max(p50s) - min(p50s)) / max(p50s)
    for hz, ms in sorted(caps.items()):
        if all(abs(p - ms) / ms < 0.04 for p in p50s) and spread < 0.03:
            print("\nWARNING: every variant sits within 4%% of %.2f ms (%d Hz) with under "
                  "3%% spread.\n         That is a frame cap, not a measurement - read the "
                  "plugin-side costs\n         below instead, and check that vsync was "
                  "actually disabled." % (ms, hz))
            return


def load_sites():
    """Prefer sites discovered by a scan; fall back to the name-based list."""
    if os.path.isfile(SITES_FILE):
        try:
            found = json.load(open(SITES_FILE, encoding="utf-8"))
            if found:
                print("using %d scanned site(s) from sites.json" % len(found))
                return found
        except (OSError, ValueError):
            pass
    return DEFAULT_SITES


def do_scan(args):
    """One session that enumerates exterior cells, then writes sites.json.

    Reference count is a cheap proxy for "worth measuring": an occlusion culler
    needs geometry to occlude. Picking cells by name lands wherever the lookup
    happens to resolve, which in the first run of this harness meant a
    22-reference corner of Balmora and a Vivec vantage that culled nothing at
    all.
    """
    patched = False if args.vsync else disable_vsync()
    start_results_dir()
    deploy_harness()
    try:
        print("")
        print("[scan] enumerating exterior cells (>= %d refs)" % args.min_refs)
        run_site("scan", [("scan", {})], args, None, mode="scan")
    finally:
        if not args.keep:
            remove_harness()
        if patched:
            restore_vsync()
        kill_game()

    log = os.path.join(RESULTS, "scan.mwse.log")
    if not os.path.isfile(log):
        print("no scan log produced")
        return 1

    found = []
    for line in open(log, encoding="utf-8", errors="replace"):
        m = SCAN_RE.search(line)
        if m:
            name = m.group("name").strip()
            found.append({
                "town": m.group("town").strip(),
                "name": name or "wilderness %s,%s" % (m.group("x"), m.group("y")),
                "x": int(m.group("x")),
                "y": int(m.group("y")),
                "refs": int(m.group("refs")),
            })
    if not found:
        print("scan reported no cells above the threshold; lower --min-refs")
        return 1

    found.sort(key=lambda c: -c["refs"])
    print("")
    print("densest exterior cells:")
    for c in found[:20]:
        print("  %6d refs  grid %4d,%-4d  %s" % (c["refs"], c["x"], c["y"], c["name"]))

    # One site per town, the densest cell of its neighbourhood. Taking a global
    # top-N instead drops whole towns: Old Ebonheart and Narsis own eight of the
    # ten densest cells between them, which would have silently excluded Vivec -
    # and a sparse town is exactly the case worth keeping in the comparison,
    # because that is where a culler stops paying for itself.
    # Within a town's neighbourhood, prefer the densest cell that is actually
    # named after that town. The plain densest can be a different place
    # entirely: searching "Vivec" loads a 3x3 grid whose busiest cell is Ald
    # Sotha, and labelling that "vivec" would quietly measure the wrong site.
    best = {}
    for c in found:
        town_key = c["town"].split(",")[0].strip().lower()
        matches = c["name"].strip().lower().startswith(town_key)
        cur = best.get(c["town"])
        better = (cur is None
                  or (matches and not cur["_match"])
                  or (matches == cur["_match"] and c["refs"] > cur["refs"]))
        if better:
            c = dict(c)
            c["_match"] = matches
            best[c["town"]] = c

    picked = []
    for town, c in sorted(best.items(), key=lambda kv: -kv[1]["refs"]):
        label = town.split(",")[0].strip().lower().replace(" ", "-") or "cell"
        picked.append({
            "name": label,
            "x": c["x"], "y": c["y"], "refs": c["refs"], "cell": c["name"],
        })
        if not c.get("_match"):
            print("  note: %s has no cell named after it nearby; using %s"
                  % (town, c["name"]))

    json.dump(picked, open(SITES_FILE, "w", encoding="utf-8"), indent=2)
    print("")
    print("wrote %d site(s) to %s" % (len(picked), SITES_FILE))
    for s in picked:
        print("  %-16s grid %4d,%-4d  %5d refs  (%s)"
              % (s["name"], s["x"], s["y"], s["refs"], s.get("cell", "")))
    return 0



def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sample", type=int, default=30, help="sample window seconds (default 30)")
    ap.add_argument("--warmup", type=int, default=10, help="settle seconds before sampling (default 10)")
    ap.add_argument("--save", default="quiksave", help="save to auto-load (default quiksave)")
    ap.add_argument("--hour", type=float, default=12.0, help="pin GameHour for stable lighting (default 12)")
    ap.add_argument("--keep", action="store_true", help="leave the harness deployed afterwards")
    ap.add_argument("--only", help="comma-separated variant names to run")
    ap.add_argument("--sites", help="comma-separated site names to run")
    ap.add_argument("--here", action="store_true",
                    help="skip site travel; measure wherever the save already is")
    ap.add_argument("--scan", action="store_true",
                    help="enumerate exterior cells by reference count and write sites.json")
    ap.add_argument("--min-refs", type=int, default=250,
                    help="scan: only report cells with at least this many references")
    ap.add_argument("--top", type=int, default=4, help="scan: how many sites to keep")
    ap.add_argument("--repeat", dest="repeats", type=int, default=1,
                    help="measure each pair this many times and aggregate (default 1)")
    # Coprime with the plugin's 300-frame stats interval on purpose. At 300 the
    # two alias: every stats line lands at the same bearing, so the reported cull
    # percentage describes one view rather than the window, and the same config
    # reported 77% in one run and 49% in the next.
    ap.add_argument("--rotate-every", type=int, default=271,
                    help="rotate the view every N frames during sampling (default 300)")
    ap.add_argument("--views", type=int, default=8,
                    help="how many bearings to rotate through (default 8)")
    ap.add_argument("--sweep", nargs="?", const="async", default=None,
                    choices=["async", "sync"],
                    help="sweep one knob at a time instead of just on/off. "
                         "'sync' pins the rasterizer to the main thread for "
                         "every variant, as the low-tier preset runs it")
    ap.add_argument("--settle", type=int, default=3,
                    help="seconds after applying a variant before warmup (default 3)")
    ap.add_argument("--relaunch", action="store_true",
                    help="one game process per measurement instead of one per site "
                         "(much slower; use to check the in-session toggle)")
    ap.add_argument("--vsync", action="store_true",
                    help="leave vsync as configured (default disables it)")
    args = ap.parse_args()

    variants = DEFAULT_VARIANTS
    if args.sweep == "async":
        variants = SWEEP_VARIANTS
    elif args.sweep == "sync":
        variants = SWEEP_SYNC_VARIANTS
    if args.only:
        wanted = {s.strip() for s in args.only.split(",")}
        variants = [v for v in variants if v[0] in wanted]
        if not variants:
            print("no variants matched --only")
            return 2

    if args.scan:
        return do_scan(args)

    sites = load_sites()
    if args.sites:
        wanted = {s.strip() for s in args.sites.split(",")}
        sites = [s for s in sites if s["name"] in wanted]
    if args.here:
        sites = [None]

    total = len(sites) * len(variants) * args.repeats
    print("msoc performance check: %d site(s) x %d variant(s) x %d repeat(s) = %d run(s), "
          "%ds sample each" % (len(sites), len(variants), args.repeats, total, args.sample))
    # One launch per site amortises the ~100s of startup over every pass in it;
    # --relaunch pays it per measurement.
    per_pass = args.settle + args.warmup + args.sample + 10
    launches = total if args.relaunch else len(sites)
    print("  %d launch(es), ~%d min estimated"
          % (launches, round((launches * 100 + total * per_pass) / 60.0)))

    patched = False if args.vsync else disable_vsync()
    print("  results -> %s" % start_results_dir())
    deploy_harness()
    results = []
    try:
        for site in sites:
            site_name = site["name"] if site else "here"
            by_variant = {}
            if args.relaunch:
                # One process per measurement. Slower by roughly the launch
                # overhead times the pass count; kept so the fast path can be
                # checked against it.
                for name, config in variants:
                    for rep in range(args.repeats):
                        label = "%s/%s" % (site_name, name)
                        if args.repeats > 1:
                            label += "#%d" % (rep + 1)
                        print("")
                        print("[%s] %s" % (label, json.dumps(config)))
                        for r in run_site(label, [(name, config)], args, site, repeats=1):
                            by_variant.setdefault(name, []).append(r)
            else:
                print("")
                print("[%s] %d variant(s) x %d repeat(s) in one session"
                      % (site_name, len(variants), args.repeats))
                for r in run_site(site_name, variants, args, site,
                                  repeats=args.repeats):
                    # "site/variant#rep" -> variant
                    head = r["run"].split("#", 1)[0]
                    by_variant.setdefault(head.rsplit("/", 1)[-1], []).append(r)

            for name, _config in variants:
                reps = by_variant.get(name) or []
                if not reps:
                    continue
                # Median across repeats: a single run can catch a background
                # task or a shader compile, and a mean would carry that in.
                agg = dict(reps[len(reps) // 2])
                for key in ("mean", "p50", "p95", "p99", "fps"):
                    agg[key] = statistics.median(float(r[key]) for r in reps)
                agg["site"] = site_name
                agg["variant"] = name
                agg["repMeans"] = [float(r["mean"]) for r in reps]
                agg["reps"] = len(reps)
                agg["run"] = "%s/%s" % (agg["site"], name)
                if len(reps) > 1:
                    spread = (max(float(r["mean"]) for r in reps)
                              - min(float(r["mean"]) for r in reps))
                    agg["spread"] = spread
                    print("  %d reps, mean spread %.2f ms" % (len(reps), spread))
                results.append(agg)
    finally:
        if not args.keep:
            remove_harness()
        if patched:
            restore_vsync()
        kill_game()

    if not results:
        print("\nno results")
        return 1

    print("\n%-16s %-8s %9s %9s %9s %8s"
          % ("site", "variant", "mean ms", "p50 ms", "p95 ms", "fps"))
    print("-" * 64)
    for r in results:
        print("%-16s %-8s %9.2f %9.2f %9.2f %8.1f"
              % (r["site"], r["variant"], float(r["mean"]), float(r["p50"]),
                 float(r["p95"]), float(r["fps"])))

    warn_if_capped(results)

    # Per site, culler on versus off. The plugin's own timers say what it costs;
    # only this says what it returns.
    print("\nculler on vs off, per site (negative = faster with the culler):")
    by_site = {}
    for r in results:
        by_site.setdefault(r["site"], {})[r["variant"]] = r
    for site, runs in sorted(by_site.items()):
        off, on = runs.get("off"), runs.get("on")
        if not (off and on):
            print("  %-16s incomplete pair" % site)
            continue
        d = float(on["mean"]) - float(off["mean"])
        p = on.get("plugin") or {}
        print("  %-16s %+7.2f ms  %+6.1f%%   (%.1f -> %.1f fps, cull %.0f%%, %.0f occluders)"
              % (site, d, 100.0 * d / float(off["mean"]),
                 float(off["fps"]), float(on["fps"]),
                 p.get("cullPct", 0.0), p.get("occluders", 0.0)))

    # With more than the on/off pair, the interesting comparison is each knob
    # against the default-on config: a variant that beats "on" is a knob whose
    # default is costing more than it returns at this site.
    if len(variants) > 2:
        print("")
        print("each variant vs default-on (negative = that change is faster):")
        for site, runs in sorted(by_site.items()):
            base = runs.get("on")
            if not base:
                continue
            b = float(base["mean"])
            floor = noise_floor(results, site)
            if floor is None:
                print("  %s  (default-on %.2f ms; no repeats, so no error bar - "
                      "treat every delta below as unverified)" % (site, b))
            else:
                print("  %s  (default-on %.2f ms; resolves differences down to "
                      "%.2f ms)" % (site, b, floor))
            rows = []
            for name, _cfg in variants:
                r = runs.get(name)
                if not r or name == "on":
                    continue
                p = r.get("plugin") or {}
                rows.append((float(r["mean"]) - b, name, p))
            for d, name, p in sorted(rows):
                # An unresolvable delta is not a small effect, it is no
                # measurement. Say so on the line rather than letting the sort
                # order imply a ranking that the data does not support.
                mark = "  --" if (floor is not None and abs(d) < floor) else "    "
                print("   %s %-14s %+7.2f ms %+6.1f%%   cull %3.0f%%  rast %4.0fus  "
                      "drain %5.0fus  classify %4.0fus"
                      % (mark, name, d, 100.0 * d / b, p.get("cullPct", 0.0),
                         p.get("rasterizeUs", 0.0), p.get("drainUs", 0.0),
                         p.get("classifyUs", 0.0)))
            if floor is not None:
                print("    (-- marks a difference below the noise floor: "
                      "not a small effect, no effect measured)")

    print("\nplugin-side per-frame cost (median over stats windows, microseconds):")
    hdr = ("rasterizeUs", "aggTerrainUs", "classifyUs", "drainUs", "displayUs", "asyncFlushUs")
    print("%-24s %s" % ("run", " ".join("%12s" % h for h in hdr)))
    for r in results:
        p = r.get("plugin") or {}
        if not p.get("windows"):
            print("%-24s   (culler off - no stats lines)" % r["run"])
            continue
        print("%-24s %s" % (r["run"], " ".join("%12.0f" % p.get(h, 0) for h in hdr)))

    print("\nlogs: %s" % RESULTS)
    return 0


if __name__ == "__main__":
    sys.exit(main())
