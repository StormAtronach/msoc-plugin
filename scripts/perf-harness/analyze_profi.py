#!/usr/bin/env python3
"""Summarise a ProFi Lua profile by mod, from a results directory.

ProFi reports per function, which is the right granularity for fixing one hot
loop but the wrong one for deciding *which mod* to look at: a modlist spreads its
cost over hundreds of small functions, and no single line looks alarming. This
rolls the per-function rows up by owning mod so the question VTune could not
answer - lua51.dll is a fifth of the main thread, but whose Lua? - gets an answer.

Read the RELATIVE column only. ProFi hooks the Lua VM, so absolute times in a
profiled session bear no relation to real frame times; the shares between entries
are what carry meaning.

Usage:
    python analyze_profi.py                  # results/latest
    python analyze_profi.py <results-dir>
    python analyze_profi.py <file.profi.txt>
"""

import collections
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RESULTS_ROOT = os.path.join(REPO, "scripts", "perf-harness", "results")

TOTAL_RE = re.compile(r"TOTAL TIME\s*=\s*([\d.]+)")


def parse(path):
    """Yield (file, function, line, seconds, calls) for each profiled function.

    The report is fixed-width and pipe-delimited, but paths contain colons on
    Windows and function names can contain almost anything, so fields are taken
    from the right where the numeric columns are unambiguous.
    """
    rows = []
    total = None
    for raw in open(path, encoding="utf-8", errors="replace"):
        line = raw.rstrip("\n")
        m = TOTAL_RE.search(line)
        if m:
            total = float(m.group(1))
            continue
        if not line.startswith("|") or line.count(":") < 3:
            continue
        body = line.strip().strip("|")
        parts = [p.strip() for p in body.split(":")]
        if len(parts) < 5 or parts[0].upper() == "FILE":
            continue
        # From the right: CALLED, RELATIVE, TIME, LINE; everything else is the
        # file/function pair, which may itself have contained a colon.
        try:
            calls = int(parts[-1])
            seconds = float(parts[-3])
            lineno = parts[-4]
        except (ValueError, IndexError):
            continue
        head = parts[:-4]
        if len(head) < 2:
            continue
        src = ":".join(head[:-1]).strip()
        func = head[-1].strip()
        rows.append((src, func, lineno, seconds, calls))
    return rows, total


def owner(src):
    """Attribute a source path to the mod that owns it.

    MWSE mods live under `mods/<name>/`, core framework code under `core/`, and
    anything else (C functions, stdin chunks) has no file to blame.
    """
    s = src.replace("\\", "/").lower()
    m = re.search(r"/mods/([^/]+)/", s) or re.search(r"^mods/([^/]+)/", s)
    if m:
        return m.group(1)
    if "/core/" in s or s.startswith("core/"):
        return "(mwse core)"
    if not s or s in ("[c]", "=[c]", "?"):
        return "(C function)"
    return os.path.dirname(s) or "(other)"


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else None
    if not target:
        pointer = os.path.join(RESULTS_ROOT, "latest")
        if not os.path.isfile(pointer):
            print("no results/latest; pass a results dir or a .profi.txt")
            return 1
        target = os.path.join(RESULTS_ROOT, open(pointer, encoding="utf-8").read().strip())

    if os.path.isdir(target):
        files = sorted(os.path.join(target, f) for f in os.listdir(target)
                       if f.endswith(".profi.txt"))
    else:
        files = [target]
    if not files:
        print("no .profi.txt in %s - was the run made with --luaprofile?" % target)
        return 1

    for path in files:
        rows, total = parse(path)
        if not rows:
            print("%s: no profiled functions parsed" % os.path.basename(path))
            continue
        grand = sum(r[3] for r in rows)
        print("=" * 78)
        print("%s   (%d functions, %.2fs of hooked Lua%s)" % (
            os.path.basename(path), len(rows), grand,
            ", %.2fs wall" % total if total else ""))
        print("=" * 78)

        by_mod = collections.Counter()
        calls_by_mod = collections.Counter()
        for src, func, lineno, secs, calls in rows:
            by_mod[owner(src)] += secs
            calls_by_mod[owner(src)] += calls
        print("\n%-34s %10s %8s %14s" % ("mod", "sec", "share", "calls"))
        for mod, secs in by_mod.most_common(18):
            print("%-34s %10.3f %7.1f%% %14d" % (mod, secs, 100 * secs / grand,
                                                 calls_by_mod[mod]))

        print("\n%-28s %-30s %8s %7s %12s" % ("function", "file", "sec", "share", "calls"))
        rows.sort(key=lambda r: -r[3])
        for src, func, lineno, secs, calls in rows[:20]:
            short = src.replace("\\", "/").split("/")
            short = "/".join(short[-2:]) if len(short) > 1 else src
            print("%-28s %-30s %8.3f %6.1f%% %12d" % (
                func[:28], short[:30], secs, 100 * secs / grand, calls))
        print("")

    print("RELATIVE shares only. The VM hook distorts absolute time, so these")
    print("numbers rank Lua work against other Lua work - not against the frame.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
