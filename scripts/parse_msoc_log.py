#!/usr/bin/env python3
"""Parse MSOC.log into a health + performance summary.

Each 'MSOC: frame N' line is an aggregate snapshot emitted every ~300 frames,
carrying that window's latest instantaneous timings plus rolling counters.

Rather than sampling only the tail (the old "last 50 frames" behaviour, which
landed wherever the session happened to end and made cross-run comparison
meaningless), this summarises the WHOLE run three ways:

  1. Outlier-robust distribution over the full run (mean / p50 / p95), after
     dropping warmup, cell-cross, and hitch/menu windows.
  2. A per-window time series, so you can see how load varied across the
     session and pick comparable segments between two runs.
  3. A load-bucketed view keyed on terrain lands in view, so two runs with
     different overall scene mixes can still be compared like-for-like
     (e.g. "horizon-build at 4 terrain lands" in run A vs run B).

Usage:  parse_msoc_log.py [logpath] [nwindows]
"""
import sys, statistics

def parse_frame(line):
    d = {}
    for tok in line.split():
        if '=' in tok:
            k, v = tok.split('=', 1)
            d[k] = v
    return d

def num(d, k):
    v = d.get(k, '0')
    if '/' in v: v = v.split('/')[0]
    try: return float(v)
    except: return 0.0

def occ_test(f):
    v = f.get('queryOccluded', '0/0')
    if '/' in v:
        a, b = v.split('/', 1)
        try: return int(a), int(b)
        except: return 0, 0
    return 0, 0

def pct(vals, p):
    """Linear-interpolated percentile (p in [0,1]); robust to spikes."""
    if not vals: return 0.0
    s = sorted(vals)
    if len(s) == 1: return s[0]
    idx = (len(s) - 1) * p
    lo = int(idx); hi = min(lo + 1, len(s) - 1)
    frac = idx - lo
    return s[lo] * (1 - frac) + s[hi] * frac

def cullrate(frames):
    o = sum(occ_test(f)[0] for f in frames)
    t = sum(occ_test(f)[1] for f in frames)
    return (100.0 * o / t if t else 0.0), o, t

def mean(frames, k):
    return statistics.mean(num(f, k) for f in frames) if frames else 0.0

# (key, label) for the per-frame timing breakdown.
TIMINGS = [('frameDeltaUs','frame-total'),('rasterizeUs','rasterize'),('occXformUs','occ-xform'),
           ('drainUs','drain'),('classifyUs','classify'),('displayUs','display'),
           ('aggTerrainUs','agg-terrain'),('horizonBuildUs','horizon-build'),
           ('horizonRasterUs','horizon-raster'),('asyncFlushUs','async-flush'),('wakeUs','wake-threads')]

def terrain_lands(f):
    return int(num(f, 'aggTerrainLands') + num(f, 'horizonLandsFed'))

def fnum(f):
    try: return int(f.get('_frame', '-1'))
    except Exception: return -1

def main(path, nwin=6, minframe=0):
    install, frames = [], []
    for ln in open(path, encoding='utf-8', errors='replace'):
        ln = ln.rstrip('\n')
        if ln.startswith('MSOC: frame '):
            fr = parse_frame(ln)
            try: fr['_frame'] = ln.split()[2]
            except Exception: fr['_frame'] = '?'
            frames.append(fr)
        elif ln.startswith('MSOC:'):
            install.append(ln)

    print("=== install / setup ===")
    for l in install[:8]: print(" ", l)
    print(f"\n=== {len(frames)} stats windows logged (each ~300 frames) ===")
    if not frames: return

    # Optional frame slice. When set, the slice start is the user's chosen
    # boundary, so we do NOT drop a "warmup" window off the front of it.
    if minframe:
        frames = [f for f in frames if fnum(f) >= minframe]
        print(f"(sliced to frame >= {minframe}: {len(frames)} windows)")
        if not frames: return

    # Representative set: drop warmup (first window = initial cell load) and
    # spike windows. Cell-cross windows carry cellWipeUs>0; load hitches and
    # menu-mode windows show as a frameDeltaUs far above the run median.
    # p50/p95 below are robust to spikes anyway, but excluding these keeps the
    # mean honest and the per-window series readable.
    body = frames[1:] if (not minframe and len(frames) > 1) else frames
    med_dt = statistics.median([num(f, 'frameDeltaUs') for f in body]) or 1.0
    clean, excl_cross, excl_hitch = [], 0, 0
    for f in body:
        if num(f, 'cellWipeUs') > 0: excl_cross += 1; continue
        if num(f, 'frameDeltaUs') > 3 * med_dt: excl_hitch += 1; continue
        clean.append(f)
    warm = len(frames) - len(body)
    if not clean: clean = body
    print(f"(representative set = {len(clean)} windows; excluded "
          f"{warm} warmup, {excl_cross} cell-cross, {excl_hitch} hitch/menu)\n")

    # ---- effectiveness (whole representative run) ----
    cr, o, t = cullrate(clean)
    print("-- occlusion effectiveness (representative run) --")
    print(f"  occludees OCCLUDED:  {o}/{t}  ({cr:.1f}% cull rate)")
    box = sum(num(f, 'boxOccluded') for f in clean)
    if o:
        print(f"  of which from box test: {int(box)}  ({100*box/o:.1f}% of occlusions)")
    bh = sum(num(f, 'boxCacheHit') for f in clean); bm = sum(num(f, 'boxCacheMiss') for f in clean)
    if bh + bm:
        print(f"  box-AABB cache: {100*bh/(bh+bm):.1f}% hit  (hits={int(bh)} miss={int(bm)})")
    print(f"  occluders rasterized/frame: {mean(clean,'rasterized'):.0f}"
          f"   viewCulled/frame: {mean(clean,'viewCulled'):.0f}")
    print(f"  terrain lands/frame: {mean(clean,'aggTerrainLands'):.1f} raster"
          f" + {mean(clean,'horizonLandsFed'):.1f} horizon")

    # ---- cache hit rates ----
    print("\n-- cache hit rates (representative run) --")
    for label, h, m in [('occluder','occCacheHit','occCacheMiss'),
                        ('landMembership','landMembershipHit','landMembershipMiss'),
                        ('land','landCacheHit','landCacheMiss'),
                        ('drain (temporal)','tcHit','tcMiss')]:
        H = sum(num(f, h) for f in clean); M = sum(num(f, m) for f in clean)
        r = 100*H/(H+M) if (H+M) else 0
        print(f"  {label:18s} {r:5.1f}% hit   (hits={int(H)} miss={int(M)})")

    # ---- timing distribution (robust to scene/spike variance) ----
    print("\n-- per-frame timing (us; mean / p50 / p95 over representative run) --")
    print(f"  {'metric':14s} {'mean':>8s} {'p50':>8s} {'p95':>8s}")
    for k, lbl in TIMINGS:
        vals = [num(f, k) for f in clean]
        print(f"  {lbl:14s} {statistics.mean(vals):8.0f} {pct(vals,0.50):8.0f} {pct(vals,0.95):8.0f}")

    # ---- per-window time series ----
    print(f"\n-- per-window breakdown ({nwin} windows across the representative run) --")
    print(f"  {'frames':>15s} {'cull%':>6s} {'dDelta':>7s} {'drain':>6s} "
          f"{'disp':>6s} {'rast':>5s} {'hBuild':>6s} {'tLand':>5s} {'tested':>7s}")
    W = max(1, len(clean) // nwin)
    i = 0
    while i < len(clean):
        seg = clean[i:i+W]; i += W
        rng = f"{seg[0]['_frame']}-{seg[-1]['_frame']}"
        cr_s = cullrate(seg)[0]
        print(f"  {rng:>15s} {cr_s:6.1f} {pct([num(f,'frameDeltaUs') for f in seg],0.5):7.0f}"
              f" {mean(seg,'drainUs'):6.0f} {mean(seg,'displayUs'):6.0f} {mean(seg,'rasterizeUs'):5.0f}"
              f" {mean(seg,'horizonBuildUs'):6.0f} {statistics.mean(terrain_lands(f) for f in seg):5.1f}"
              f" {statistics.mean(occ_test(f)[1] for f in seg):7.0f}")

    # ---- load-bucketed (like-for-like across runs) ----
    print("\n-- horizon/raster cost by terrain load (compare same 'lands' row across runs) --")
    buckets = {}
    for f in clean:
        buckets.setdefault(terrain_lands(f), []).append(f)
    print(f"  {'lands':>5s} {'frames':>6s} {'hBuild':>7s} {'hRast':>6s} {'rast':>6s} "
          f"{'vertsFed':>9s} {'curtTri':>8s}")
    for lands in sorted(buckets):
        seg = buckets[lands]
        print(f"  {lands:5d} {len(seg):6d} {mean(seg,'horizonBuildUs'):7.0f} {mean(seg,'horizonRasterUs'):6.0f}"
              f" {mean(seg,'rasterizeUs'):6.0f} {mean(seg,'horizonVertsFed'):9.0f}"
              f" {mean(seg,'horizonCurtainTris'):8.0f}")

    # ---- CCW-only back-face cull A/B ----
    # The option's effect lands on occluderTris + rasterizeUs (back faces
    # skipped), not terrain. cull% should stay ~flat; a drop means CW-wound
    # meshes were dropped as occluders.
    print("\n-- CCW-only back-face cull --")
    if not any('ccwOnly' in f for f in clean):
        print("  not logged (pre-instrumentation build); rebuild + redeploy to capture")
    else:
        off = [f for f in clean if num(f, 'ccwOnly') < 0.5]
        on  = [f for f in clean if num(f, 'ccwOnly') >= 0.5]
        if off and on:
            print(f"  MIXED run ({len(off)} off / {len(on)} on) -> A/B split")
            print("  (fair only if both halves are the same scene/vantage)")
            print(f"    {'state':>5s} {'frames':>6s} {'cull%':>6s} {'occ/f':>6s} "
                  f"{'occTris/f':>9s} {'rast mean':>9s} {'rast p50':>8s}")
            for label, grp in [('off', off), ('on', on)]:
                rv = [num(f, 'rasterizeUs') for f in grp]
                print(f"    {label:>5s} {len(grp):6d} {cullrate(grp)[0]:6.1f} {mean(grp,'rasterized'):6.0f}"
                      f" {mean(grp,'occluderTris'):9.0f} {statistics.mean(rv):9.0f} {pct(rv,0.5):8.0f}")
        else:
            state = 'ON' if on else 'OFF'
            print(f"  single-state run: CCW-only {state} for the whole representative set")
            print(f"  (occTris/f={mean(clean,'occluderTris'):.0f}  rast mean={mean(clean,'rasterizeUs'):.0f}"
                  f"  cull%={cullrate(clean)[0]:.1f}  - compare against the other run's line)")

    # ---- sanity ----
    last = clean[-1]
    print("\n-- sanity --")
    print(f"  budget trips (lifetime): rast={int(num(last,'rastTripsSess'))} class={int(num(last,'classTripsSess'))}")
    print(f"  maxDepthSess={int(num(last,'maxDepthSess'))}  cellChanges={int(num(last,'cellChanges'))}")
    print(f"  cumulative OCCLUDED/tested: {last.get('cumul','?')}")

if __name__ == '__main__':
    # Positional: [logpath] [nwindows]. Flag: --from N / --min N slices to
    # frames >= N (useful to drop the early-session / movement portion).
    args, minframe, rest = sys.argv[1:], 0, []
    i = 0
    while i < len(args):
        if args[i] in ('--from', '--min') and i + 1 < len(args):
            minframe = int(args[i+1]); i += 2; continue
        rest.append(args[i]); i += 1
    path = rest[0] if rest else r"D:\Modlists\Morrowind Refreshed 260602\Morrowind Refreshed\root\MSOC.log"
    nwin = int(rest[1]) if len(rest) > 1 else 6
    main(path, nwin, minframe)
