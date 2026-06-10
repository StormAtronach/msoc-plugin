#!/usr/bin/env python3
"""Parse MSOC.log (one wide key=value line per frame) into a health summary."""
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

def main(path):
    install, frames = [], []
    for ln in open(path, encoding='utf-8', errors='replace'):
        ln = ln.rstrip('\n')
        if ln.startswith('MSOC: frame '): frames.append(parse_frame(ln))
        elif ln.startswith('MSOC:'): install.append(ln)

    print("=== install / setup ===")
    for l in install[:8]: print(" ", l)
    print(f"\n=== {len(frames)} stats frames logged ===")
    if not frames: return
    ss = frames[-50:]
    avg = lambda k: statistics.mean(num(f, k) for f in ss)

    tot_occ = sum(int(f['queryOccluded'].split('/')[0]) for f in ss if 'queryOccluded' in f)
    tot_test = sum(int(f['queryOccluded'].split('/')[1]) for f in ss if '/' in f.get('queryOccluded', ''))
    print(f"(steady state = last {len(ss)} frames)\n")
    print("-- occlusion effectiveness --")
    print(f"  occludees OCCLUDED:  {tot_occ}/{tot_test}  ({100*tot_occ/tot_test:.1f}% cull rate)")
    box = sum(num(f, 'boxOccluded') for f in ss)
    if box or any('boxOccluded' in f for f in ss):
        share = 100 * box / tot_occ if tot_occ else 0
        print(f"  of which from box test: {int(box)}  ({share:.1f}% of occlusions)")
        bh = sum(num(f, 'boxCacheHit') for f in ss)
        bm = sum(num(f, 'boxCacheMiss') for f in ss)
        if bh + bm:
            print(f"  box-AABB cache: {100*bh/(bh+bm):.1f}% hit  (hits={int(bh)} miss={int(bm)})")
    print(f"  occluders rasterized/frame: {avg('rasterized'):.0f}   viewCulled/frame: {avg('viewCulled'):.0f}")
    print(f"  terrain lands aggregated/frame: {avg('aggTerrainLands'):.0f}")

    print("\n-- cache hit rates (steady state) --")
    for label, h, m in [('occluder', 'occCacheHit', 'occCacheMiss'),
                        ('landMembership', 'landMembershipHit', 'landMembershipMiss'),
                        ('land', 'landCacheHit', 'landCacheMiss'),
                        ('drain (temporal)', 'tcHit', 'tcMiss')]:
        H = sum(num(f, h) for f in ss); M = sum(num(f, m) for f in ss)
        r = 100*H/(H+M) if (H+M) else 0
        print(f"  {label:18s} {r:5.1f}% hit   (hits={int(H)} miss={int(M)})")

    print("\n-- per-frame timing (us, steady-state avg) --")
    for k, lbl in [('rasterizeUs','rasterize'),('occXformUs','occ-xform'),('drainUs','drain'),
                   ('classifyUs','classify'),('displayUs','display'),('aggTerrainUs','agg-terrain'),
                   ('asyncFlushUs','async-flush'),('wakeUs','wake-threads')]:
        print(f"  {lbl:14s} {avg(k):8.0f}")

    last = ss[-1]
    print("\n-- sanity --")
    print(f"  budget trips (lifetime): rast={int(num(last,'rastTripsSess'))} class={int(num(last,'classTripsSess'))}")
    print(f"  maxDepthSess={int(num(last,'maxDepthSess'))}  cellChanges={int(num(last,'cellChanges'))}")
    print(f"  cumulative OCCLUDED/tested: {last.get('cumul','?')}")

if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1
         else r"D:\Modlists\Morrowind Refreshed 260602\Morrowind Refreshed\root\MSOC.log")
