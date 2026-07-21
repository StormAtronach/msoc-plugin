# Handover: config lifecycle silently reverts user settings (async toggle & threadpool knobs)

**Component:** `msoc.dll` + `MWSE/mods/msoc/*.lua` (this repo) · **Severity:** settings loss / invalid A-B tests — no crash by itself
**Files to change:** `test-mod/MWSE/mods/msoc/config.lua`, `test-mod/MWSE/mods/msoc/mcm.lua`, `src/MaskResources.cpp`, i18n strings
**Repo head at diagnosis:** `4daf58a` + uncommitted hardening work · **Date:** 2026-07-21

---

## Context

Surfaced during the exterior heap-corruption investigation (see
`HANDOVER-external-occluder-stale-tail-fix.md` for that thread — note its
stale-tail diagnosis was later disproven; the corruption hunt is ongoing).
A diagnostic run required the threadpool disabled
(`OcclusionThreadpoolThreadCount=1`). The user disabled async / edited the
config, yet the crash-session logs showed
`MSOC: threadpool created; threads=4 ... cfgAsync=1` and the JSON back at
`ThreadCount:0` — the experiment silently never ran. Any future A-B test
of threadpool behaviour is invalid until this is fixed.

## Symptoms

1. User toggles **Async occluders** off (MCM) → a later session runs with
   `cfgAsync=1` anyway.
2. User edits `msoc.json` by hand → values revert, file key-order changes.
3. The MCM **thread count** slider appears live but has no effect until
   the next launch (or an EnableMSOC off/on cycle).

## Root causes (three distinct mechanisms)

### 1. Version-change migration force-reapplies tier-sensitive keys — and version flapping makes it fire every launch

`config.lua:249-266`: when `config.lastSeenVersion ~= pluginVersion`, every
key in `kTierMigratedKeys` (`config.lua:214-222` — includes
`OcclusionAsyncOccluders`) and `kRetunedKeys` is overwritten from
`default_config` and saved. On a High-tier machine that resets
`OcclusionAsyncOccluders` to `true` (`config.lua:125-160`, mirrored in
`Config.cpp::applyHardwareTierDefaults:126-186`).

The design intent ("user loses the override exactly once per plugin
update", `config.lua:209-213`) assumed monotonic upgrades. Observed
reality: **two different msoc.dll builds alternating in one install**
(a stale copy in the Root Builder `root\` overlay vs. the MO2 mod copy)
made `lastSeenVersion` flap `1.4.0 → 1.2.0 → 1.4.0` across launches —
the migration fired **every launch**, re-forcing `async=true` each time.
Log evidence:

```
[msoc] tier defaults migrated: lastSeen=1.4.0 -> 1.2.0; async=true bins=4x2 mask=512x256; retired=[]
[msoc] defaults migrated: lastSeen=1.2.0 -> 1.4.0; async=true bins=4x2 mask=512x256; boxTest=true ...
```

**Fix (both parts):**
- Only migrate on *upgrade*: parse the two versions and skip (with a
  warning log) when the loaded plugin is *older* than `lastSeenVersion`.
  Kills the flap loop.
- Preserve user overrides: persist a `tierBaseline` sub-table recording
  the default each migrated key had when last written. On migration,
  overwrite `config[k]` **only if `config[k] == tierBaseline[k]`** (user
  never diverged), then refresh `tierBaseline[k]`. Log a per-key line for
  every value actually changed, and every user override preserved.

### 2. MCM save-on-close clobbers manual file edits

`mcm.lua:72`: `template:saveOnClose(cfg.config.confPath, cfg.config)`
rewrites the whole JSON from the in-memory table when the menu closes /
session ends. Any hand-edit made to `msoc.json` while a session is live
(or before the session's exit-save lands) is silently lost — this is what
reverted `ThreadCount 1 → 0` (note `OcclusionThreadpoolThreadCount` is
*not* in `kTierMigratedKeys`; this mechanism, not migration, reverted it).

**Fix:** track a dirty flag — set in the shared `applyChange` /
`applyChangeClamped` callbacks (`mcm.lua:43-46`) — and only save on close
when the MCM actually changed something. Optionally log
`"[msoc] config saved by MCM"` so a clobber is at least visible.

### 3. Threadpool/mask knobs are restart-only but presented as live

`MaskResources.cpp:185-193`: `ensureMSOCResourcesMatchConfig()` only
reconciles `EnableMSOC` on/off; `createMSOCResources` early-outs when
resources exist, so `OcclusionThreadpoolThreadCount` / `BinsW/H` /
`MaskWidth/Height` changes never take effect mid-session. Only
`OcclusionForensicsWatchdog` documents restart-only semantics
(`config.lua:102-105`); the threadpool sliders imply live effect.

**Fix (either level):**
- Minimal: mark the four widgets restart-only in their i18n descriptions.
- Proper: snapshot `{threadCount, binsW, binsH, maskW, maskH}` at
  creation; in `ensureMSOCResourcesMatchConfig()` compare against current
  `Configuration::*` and `destroyMSOCResources` + recreate on mismatch —
  the function already runs at the safe top-of-frame point and is
  documented as an idempotent reconciler, so this fits its contract.

## Also worth doing

- The tier-default tables exist twice (`Config.cpp::applyHardwareTierDefaults`
  and `config.lua::applyTierDefaults`) with a comment demanding they stay in
  sync (`config.lua:114-124`). Consider generating one from the other or
  asserting equality at startup via the FFI.
- Root cause of the version flap — duplicate `msoc.dll` copies — is an
  install-hygiene hazard: the plugin could log its own module path at load
  and warn if a second msoc.dll is present in `Data Files/MWSE/lib` outside
  the VFS view.

## Verify

1. Set `OcclusionAsyncOccluders=false` via MCM, restart twice → MSOC.log
   shows `cfgAsync=0` (or no threadpool) both times; value survives in JSON.
2. With the game closed, hand-edit `OcclusionThreadpoolThreadCount:1`,
   launch, open+close the MCM without touching anything, exit → JSON still
   has `1`; MSOC.log startup shows
   `skipping threadpool allocation ... using direct serial submission`.
3. Simulate a downgrade (`lastSeenVersion:"9.9.9"`) → migration skips with
   a warning instead of re-forcing tier defaults.
4. Change thread count in MCM mid-session (if the "proper" fix landed) →
   MSOC.log shows destroy + recreate with the new count on the next frame.
