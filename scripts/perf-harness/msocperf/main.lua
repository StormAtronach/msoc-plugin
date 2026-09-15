-- msoc performance harness, in-game half.
--
-- Deployed as a temporary MWSE mod by scripts/perf-harness/run_perf.py. Inert
-- unless a run spec is present, so it does nothing on an ordinary launch.
--
-- Two modes:
--
--   scan     Enumerate exterior cells and report how many references each
--            holds, so the driver can pick dense places to measure instead of
--            guessing by name. Named towns span several cells and the busy one
--            is not always the one a name lookup returns.
--
--   measure  Load a save, travel to a cell, apply a config variant, hold the
--            camera on a fixed spot, and sample frame times - rotating the
--            view every N frames so the window covers a spread of directions
--            rather than whatever one bearing happened to be cheap.
--
-- Frame timing is measured here rather than read out of MSOC.log on purpose.
-- The plugin's stats line is only emitted on frames where the culler runs, so
-- an EnableMSOC=false run produces none at all - and that run is the baseline
-- everything else is measured against.

-- The spec travels as an MWSE config file: mwse.loadConfig(name) reads
-- Data Files/MWSE/config/<name>.json, which the driver writes into the
-- deployed mod folder for MO2's virtual file system to serve. json.loadfile
-- appends the extension itself, so the name carries none.
local SPEC_NAME = "msocperf_spec"

local msocPlugin = include("msoc")

local spec = nil
do
    local ok, result = pcall(mwse.loadConfig, SPEC_NAME)
    if ok then spec = result end
end

-- mwse.log is `print(tostring(str):format(...))`, so it formats its first
-- argument a second time even with no varargs. A message that legitimately
-- contains a percent sign - "(%.0f%% off)" renders one - then dies there with
-- "bad argument #1 to 'format'". Passing the text as an argument to "%s" keeps
-- it out of that second format pass, which fixes the whole class rather than
-- requiring every call site to avoid percent signs.
-- Every line also goes to spec.progressFile, an absolute path outside MO2's
-- virtual file system: MO2 keeps MWSE.log virtual until the game exits, so
-- the driver cannot watch it for progress. The file is the only live signal.
local progressFile = spec and spec.progressFile or nil
local function say(fmt, ...)
    local line = "[msocperf] " .. string.format(fmt, ...)
    mwse.log("%s", line)
    if progressFile then
        local f = io.open(progressFile, "ab")
        if f then
            f:write(line, "\n")
            f:close()
        end
    end
end

mwse.log("[msocperf] harness loaded; spec=%s", spec and "present" or "absent")
if not spec then
    return  -- ordinary launch, stay out of the way
end

local MODE = spec.mode or "measure"
say("mode=%s run=%s", MODE, tostring(spec.run))

-- ---------------------------------------------------------------- scan mode

--- Report reference counts for the cells around each candidate town.
---
--- Reference counts are the best cheap proxy for "is this worth measuring": an
--- occlusion culler needs geometry to occlude, and the first pass of this
--- harness measured a Balmora spot holding 22 references, where the culler was
--- unsurprisingly pure overhead.
---
--- Counting cannot be done by walking every cell in the master list, because
--- iterateReferences yields nothing for a cell the engine has not loaded - a
--- sweep of all 4006 exterior cells reported zero for every one of them. What
--- does work: teleport to a town, let the engine load its 3x3 neighbourhood,
--- and count the nine cells that are now real. That also surveys the
--- surrounding cells for free, which is where the dense edge of a town often
--- turns out to be.
local function runScan()
    local towns = spec.scanTowns or { "Balmora", "Vivec", "Narsis", "Old Ebonheart" }
    local index, reported = 0, 0

    local function step()
        index = index + 1
        if index > #towns then
            say("SCAN complete towns=%d reported=%d", #towns, reported)
            say("DONE")
            return
        end

        local town = towns[index]
        local cell = tes3.getCell({ id = town })
        if not cell then
            say("SCAN town %s: no cell of that name (mod not installed?)", town)
            timer.start({ type = timer.real, duration = 0.1, callback = step })
            return
        end

        -- Drop in above the cell centre; we only need the engine to load the
        -- neighbourhood, not a stable stance.
        tes3.positionCell({
            reference = tes3.player,
            position = { (cell.gridX + 0.5) * 8192, (cell.gridY + 0.5) * 8192, 3000 },
            orientation = { 0, 0, 0 },
        })

        timer.start({
            type = timer.real,
            duration = 2.5,
            callback = function()
                local seen = {}
                for _, ext in pairs(tes3.dataHandler.exteriorCells) do
                    local c = ext and ext.cell
                    if c then
                        local key = c.gridX .. "," .. c.gridY
                        if not seen[key] then
                            seen[key] = true
                            local refs = 0
                            pcall(function()
                                for _ in c:iterateReferences() do refs = refs + 1 end
                            end)
                            say("SCAN town=%s refs=%d x=%d y=%d name=%s",
                                town, refs, c.gridX, c.gridY, tostring(c.name or ""))
                            reported = reported + 1
                        end
                    end
                end
                say("SCAN town %s done", town)
                step()
            end,
        })
    end

    step()
end

-- ------------------------------------------------------------- measure mode

local timestamps = {}
local segmentOf = {}       -- parallel to timestamps: which view segment each frame belongs to
local sampling = false
local frameInSegment, segment = 0, 0
local rotateEvery = spec.rotateEveryFrames or 300
local segments = spec.viewSegments or 8

-- One session measures every variant, repeated. Interleaving the variants
-- within each repeat (off, on, off, on) spreads any drift over the session
-- across both sides of the comparison instead of loading it onto the second.
local passes = {}
do
    local variants = spec.variants
    if not variants or #variants == 0 then
        variants = { { name = spec.run or "run", config = spec.config or {} } }
    end
    for rep = 1, (spec.repeats or 1) do
        -- Rotate the starting variant each repeat. Running the same order every
        -- time makes position a perfect proxy for variant, so any drift over the
        -- session is charged to whichever knob happens to sit at that slot -
        -- which is how two knobs came to look like consistent 0.2 ms regressions
        -- across both repeats of a sweep, and then failed to reproduce.
        local offset = (rep - 1) % #variants
        for i = 1, #variants do
            local v = variants[((i - 1 + offset) % #variants) + 1]
            local label = tostring(spec.site and spec.site.name or "here") .. "/" .. v.name
            if (spec.repeats or 1) > 1 then label = label .. "#" .. rep end
            passes[#passes + 1] = { label = label, config = v.config or {} }
        end
    end
end
local passIndex = 0
local currentRun = tostring(spec.run)

-- Every key any pass touches, with the value it had before the first pass ran.
-- Passes write into the shared cfg.config table, so without this a sweep is
-- cumulative: once one pass sets OcclusionAsyncOccluders = false it stays false
-- for every pass after it, and the results describe a config nobody asked for.
local baseline = {}
do
    local cfg = require("msoc.config")
    for _, pass in ipairs(passes) do
        for k in pairs(pass.config) do
            if baseline[k] == nil then
                -- false is a legitimate value, so record presence separately.
                baseline[k] = { value = cfg.config[k] }
            end
        end
    end
end

--- "EnableMSOC=false" - the pass config, flattened for the log line.
local function describe(config)
    local parts = {}
    for k, v in pairs(config) do
        parts[#parts + 1] = string.format("%s=%s", k, tostring(v))
    end
    table.sort(parts)
    return table.concat(parts, " ")
end

--- Face a new bearing. Rotating through the compass during the sample window
--- means the result reflects the site rather than one lucky or unlucky view.
local function faceSegment(i)
    local yaw = (2 * math.pi) * (i % segments) / segments
    pcall(function()
        tes3.player.orientation = tes3vector3.new(0, 0, yaw)
    end)
end

-- Real wall time at microsecond resolution: QueryPerformanceCounter behind
-- MWSE's profiling clock. Deliberately not e.timestamp, which is the simulation
-- clock and resolves to about a millisecond - fine for a mean over thousands of
-- frames, useless for a percentile, which just snaps to the grid.
local hiresClock = os.getHighPrecisionClock
if not hiresClock then
    -- Fail loudly and early. A fallback to the simulation clock would produce
    -- numbers that look fine and are not comparable with anything else in the
    -- results directory, which is worse than not measuring.
    say("FATAL os.getHighPrecisionClock is missing; update MWSE. No measurement "
        .. "taken - the simulation clock is not a substitute.")
    say("DONE")
    return
end

event.register("enterFrame", function()
    if not sampling then return end
    timestamps[#timestamps + 1] = hiresClock()
    segmentOf[#timestamps] = segment

    frameInSegment = frameInSegment + 1
    if frameInSegment >= rotateEvery then
        frameInSegment = 0
        segment = segment + 1
        faceSegment(segment)
    end
end)

--- Percentile over a sorted array, linear interpolation.
local function pct(sorted, p)
    if #sorted == 0 then return 0 end
    if #sorted == 1 then return sorted[1] end
    local idx = (#sorted - 1) * p + 1
    local lo = math.floor(idx)
    local hi = math.min(lo + 1, #sorted)
    return sorted[lo] * (1 - (idx - lo)) + sorted[hi] * (idx - lo)
end

local function mean(t)
    if #t == 0 then return 0 end
    local s = 0
    for _, v in ipairs(t) do s = s + v end
    return s / #t
end

-- Forward declaration: report() ends a pass and hands off to the next one.
local runNextPass

local function report()
    sampling = false
    if msocPlugin and msocPlugin.logMark then
        msocPlugin.logMark("SAMPLE-END " .. currentRun)
    end

    local frames = #timestamps
    if frames < 12 then
        say("RESULT run=%s FAILED samples=%d", currentRun, frames)
        if msocPlugin and msocPlugin.flushLog then msocPlugin.flushLog() end
        runNextPass()
        return
    end

    -- Seconds of real time, so the conversion is a constant rather than a
    -- calibration. The window length is now a cross-check instead of an input:
    -- if the series does not span roughly the window we asked for, something
    -- ate frames and the numbers should be treated with suspicion.
    local windowSeconds = spec.sampleSeconds or 30
    local span = timestamps[frames] - timestamps[1]
    if span <= 0 then
        say("RESULT run=%s FAILED degenerate clock span", currentRun)
        runNextPass()
        return
    end
    local drift = math.abs(span - windowSeconds) / windowSeconds
    if drift > 0.15 then
        say("WARN run=%s sampled %.1fs of a %.0fs window (%.0f%% off)",
            currentRun, span, windowSeconds, drift * 100)
    end
    local toMs = 1000.0

    local deltas, perSegment = {}, {}
    for i = 2, frames do
        local d = (timestamps[i] - timestamps[i - 1]) * toMs
        -- Drop absurd gaps: a hitch from disk or the OS is not a frame cost.
        if d > 0 and d < 1000 then
            deltas[#deltas + 1] = d
            local s = segmentOf[i] or 0
            perSegment[s] = perSegment[s] or {}
            table.insert(perSegment[s], d)
        end
    end

    local sorted = {}
    for i, v in ipairs(deltas) do sorted[i] = v end
    table.sort(sorted)
    local m = mean(deltas)

    -- Reported as frame time, not FPS: frame time is what the plugin's own
    -- phase timers are in, so the two are directly comparable, and averaging
    -- FPS across frames is the wrong operation anyway.
    say("RESULT run=%s frames=%d meanMs=%.3f p50Ms=%.3f p95Ms=%.3f p99Ms=%.3f meanFps=%.1f",
        currentRun, #deltas, m,
        pct(sorted, 0.50), pct(sorted, 0.95), pct(sorted, 0.99), 1000.0 / m)

    -- Per-view means show how much the bearing mattered. A site where these
    -- vary wildly is one where a single-view measurement would have been
    -- meaningless.
    local segMeans = {}
    for s = 0, segments do
        if perSegment[s] and #perSegment[s] > 20 then
            segMeans[#segMeans + 1] = string.format("%d:%.2f", s % segments, mean(perSegment[s]))
        end
    end
    if #segMeans > 0 then
        say("SEGMENTS run=%s %s", currentRun, table.concat(segMeans, " "))
    end

    if msocPlugin and msocPlugin.flushLog then msocPlugin.flushLog() end
    runNextPass()
end

--- Place the player at a cell, deterministically.
---
--- Prefers explicit grid coordinates from the driver (which get them from a
--- scan, so they point at a cell that actually holds geometry). Falls back to
--- a name lookup. Within the cell, stand on whichever reference is nearest the
--- centroid of them all: that lands in the dense middle rather than an edge.
local function gotoSite(site)
    local cell = nil
    if site.x and site.y then
        cell = tes3.getCell({ x = site.x, y = site.y })
        if cell then say("site %s -> grid %d,%d", tostring(site.name), site.x, site.y) end
    end
    if not cell then
        for _, name in ipairs(site.cells or { site.name }) do
            cell = tes3.getCell({ id = name })
            if cell then
                say("site %s -> cell '%s'", tostring(site.name), tostring(name))
                break
            end
        end
    end
    if not cell then
        say("site %s: no candidate cell resolved (mod not installed?)", tostring(site.name))
        return false
    end

    local positions = {}
    for ref in cell:iterateReferences() do
        positions[#positions + 1] = ref.position:copy()
    end
    if #positions == 0 then
        say("site %s: cell has no references", tostring(site.name))
        return false
    end

    local cx, cy = 0, 0
    for _, v in ipairs(positions) do cx, cy = cx + v.x, cy + v.y end
    cx, cy = cx / #positions, cy / #positions

    local best, bestDist = nil, math.huge
    for _, v in ipairs(positions) do
        local dx, dy = v.x - cx, v.y - cy
        if dx * dx + dy * dy < bestDist then
            bestDist, best = dx * dx + dy * dy, v
        end
    end

    tes3.positionCell({
        reference = tes3.player,
        position = { best.x, best.y, best.z + (site.zOffset or 160) },
        orientation = { 0, 0, 0 },
    })
    say("site %s: placed at %.0f, %.0f, %.0f (%d refs)",
        tostring(site.name), best.x, best.y, best.z, #positions)
    return true
end

--- Apply one pass's config, let it settle, warm up, then sample.
---
--- Defined as an assignment because report() forward-declares it: the two call
--- each other and Lua has no hoisting.
runNextPass = function()
    passIndex = passIndex + 1
    local pass = passes[passIndex]
    if not pass then
        say("ALL PASSES COMPLETE")
        if msocPlugin and msocPlugin.flushLog then msocPlugin.flushLog() end
        say("DONE")
        return
    end

    currentRun = pass.label
    local cfg = require("msoc.config")
    -- Phase timings for the plugin-side breakdown. Harmless when the culler is
    -- off; the stats line simply never fires.
    cfg.config.OcclusionLogAggregate = true
    -- Reset every swept key first, so each pass is measured against the same
    -- starting config rather than against whatever the previous pass left.
    for k, slot in pairs(baseline) do
        cfg.config[k] = slot.value
    end
    for k, v in pairs(pass.config) do
        cfg.config[k] = v
    end
    cfg.syncToNative(msocPlugin)
    say("PASS %d/%d run=%s %s", passIndex, #passes, currentRun, describe(pass.config))

    -- Toggling EnableMSOC tears down or rebuilds the mask buffer, the
    -- threadpool and the per-cell occluder cache at the next top-of-frame.
    -- Settle before the warmup so that work is not charged to this pass.
    local settle = spec.settleSeconds or 3
    local warmup = spec.warmupSeconds or 8
    local sample = spec.sampleSeconds or 30
    timer.start({
        type = timer.real,
        duration = settle,
        callback = function()
            faceSegment(0)
            say("warmup %ss", tostring(warmup))
            timer.start({
                type = timer.real,
                duration = warmup,
                callback = function()
                    timestamps, segmentOf = {}, {}
                    frameInSegment, segment = 0, 0
                    faceSegment(0)
                    sampling = true
                    -- Bracket the window in MSOC.log too, labelled: one session
                    -- now holds every pass, so an unlabelled marker cannot say
                    -- which stats lines belong to which variant.
                    if msocPlugin and msocPlugin.logMark then
                        msocPlugin.logMark("SAMPLE-START " .. currentRun)
                    end
                    say("sampling %ss, rotating every %d frames over %d views",
                        tostring(sample), rotateEvery, segments)
                    timer.start({ type = timer.real, duration = sample, callback = report })
                end,
            })
        end,
    })
end

local function startMeasure()
    -- Hold everything still except the bearing we rotate deliberately: a
    -- drifting camera or a passing NPC makes two runs incomparable.
    tes3.setPlayerControlState({ enabled = false })
    if spec.gameHour then
        tes3.setGlobal("GameHour", spec.gameHour)
    end

    -- Travel once. Every pass measures the same spot, which is the point.
    if spec.site and not gotoSite(spec.site) then
        say("RESULT run=%s SKIPPED site unavailable", tostring(spec.run))
        say("DONE")
        return
    end

    say("%d pass(es) queued for this session", #passes)
    runNextPass()
end

-- ------------------------------------------------------------------ startup

event.register("initialized", function()
    say("initialized; loading save '%s' in 8s", tostring(spec.save or "quiksave"))
    timer.start({
        type = timer.real,
        duration = 8,
        callback = function()
            local ok, err = pcall(tes3.loadGame, spec.save or "quiksave")
            if not ok then say("loadGame failed: %s", tostring(err)) end
        end,
    })
end)

local started = false
event.register("loaded", function()
    if started then return end
    started = true
    -- Let the cell finish committing before touching config or the camera.
    timer.start({
        type = timer.real,
        duration = 3,
        callback = (MODE == "scan") and runScan or startMeasure,
    })
end)
