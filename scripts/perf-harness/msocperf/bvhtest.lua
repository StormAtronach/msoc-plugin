-- BVH correctness + speed A/B, driven by the harness.
--
-- Ported from the standalone livecoding script
-- ("Livecoding/MWSE/mods/livecoding/livecoding - bvh.lua", Alt+B/Alt+V) into a
-- module so a scripted session can run it. The standalone version shipped as its
-- own MO2 mod prevented the game from starting at all, so it lives here in the
-- harness's own deploy folder instead, where nothing extra reaches RootBuilder.
--
-- Casts a fibonacci sphere of rays from the player's eye and runs the identical
-- set three times:
--
--   pass 1  UsePhysicsOptimizations = false  - vanilla exhaustive triangle
--           loops; the reference results
--   pass 2  UsePhysicsOptimizations = true   - first run, pays any BVH builds
--   pass 3  same, warm - pure query cost
--
-- Every hit of every ray is compared against pass 1 exactly: same hit count,
-- same object, bit-identical distance and intersection. Any difference is a
-- correctness failure of the candidate enumeration.
--
-- All passes use accurateSkinned = true deliberately. Without it, rayTest forces
-- the accurate-skinned flag off and the reference pass falls through to the
-- vanilla x87 loop, whose results differ from the reimplementation at the last
-- float bit - a pre-existing skew, not a property of the optimisation. The
-- invariant under test is exhaustive-reimpl vs accelerated-reimpl.
--
-- accurateSkinned = true is also what exercises the skinned path specifically,
-- which is the one a model-space triangle BVH cannot describe and which the
-- per-bone bound rejection targets.

---@diagnostic disable: undefined-global, undefined-field

local M = {}

local MAX_MISMATCH_LOG = 10

local clock = os.getHighPrecisionClock

local function log(fmt, ...)
    mwse.log("[bvhtest] " .. string.format(fmt, ...))
end

--- Evenly distributed unit directions via the golden-angle spiral.
local function fibonacciSphere(n)
    local dirs = {}
    local golden = math.pi * (3.0 - math.sqrt(5.0))
    for i = 0, n - 1 do
        local z = 1.0 - (2.0 * i + 1.0) / n
        local r = math.sqrt(math.max(0.0, 1.0 - z * z))
        local theta = golden * i
        dirs[#dirs + 1] = tes3vector3.new(math.cos(theta) * r, math.sin(theta) * r, z)
    end
    return dirs
end

--- Casts every direction once and snapshots the hits into plain Lua values,
--- because rayTest results are invalidated by the next rayTest call.
local function castPass(origin, dirs, findAll, maxDistance)
    local results = {}
    local totalHits = 0
    local t0 = clock()
    for i, dir in ipairs(dirs) do
        local hits = tes3.rayTest{
            position = origin,
            direction = dir,
            findAll = findAll,
            maxDistance = maxDistance,
            ignore = { tes3.player },
            accurateSkinned = true,
        }
        local snapshot = {}
        if hits then
            if not findAll then hits = { hits } end
            for h, hit in ipairs(hits) do
                local object = hit.reference or hit.object
                snapshot[h] = {
                    id = object and (object.id or object.name) or "?",
                    dist = hit.distance,
                    x = hit.intersection.x,
                    y = hit.intersection.y,
                    z = hit.intersection.z,
                }
            end
            totalHits = totalHits + #snapshot
        end
        results[i] = snapshot
    end
    return results, clock() - t0, totalHits
end

--- Exact comparison of two pass snapshots; logs the first few differences.
local function comparePasses(reference, candidate, dirs, label)
    local mismatches = 0
    for i = 1, #dirs do
        local a, b = reference[i], candidate[i]
        local bad = nil
        if #a ~= #b then
            bad = string.format("hit count %d vs %d", #a, #b)
        else
            for h = 1, #a do
                local ha, hb = a[h], b[h]
                if ha.id ~= hb.id then
                    bad = string.format("hit %d object '%s' vs '%s'", h, ha.id, hb.id)
                    break
                elseif ha.dist ~= hb.dist or ha.x ~= hb.x or ha.y ~= hb.y or ha.z ~= hb.z then
                    bad = string.format(
                        "hit %d (%s) dist %.9g vs %.9g, point (%.9g %.9g %.9g) vs (%.9g %.9g %.9g)",
                        h, ha.id, ha.dist, hb.dist, ha.x, ha.y, ha.z, hb.x, hb.y, hb.z)
                    break
                end
            end
        end
        if bad then
            mismatches = mismatches + 1
            if mismatches <= MAX_MISMATCH_LOG then
                local d = dirs[i]
                log("MISMATCH %s ray %d dir (%.6f %.6f %.6f): %s", label, i, d.x, d.y, d.z, bad)
            end
        end
    end
    return mismatches
end

--- Run the A/B. Returns true when every ray matched the reference.
function M.run(opts)
    opts = opts or {}
    local rayCount = opts.rays or 400
    local findAll = opts.findAll ~= false
    local maxDistance = opts.maxDistance or 0   -- 0 = unbounded, the worst case

    if mwseConfig == nil or mwseConfig.UsePhysicsOptimizations == nil then
        log("SKIP: this MWSE build has no UsePhysicsOptimizations")
        return false
    end

    local cell = tes3.player.cell
    local origin = tes3.getPlayerEyePosition()
    local dirs = fibonacciSphere(rayCount)

    log("=== A/B: %d rays from (%.1f %.1f %.1f) in %s%s ===",
        rayCount, origin.x, origin.y, origin.z,
        cell and (cell.editorName or cell.id) or "?",
        findAll and ", findAll" or ", first hit")

    local saved = mwseConfig.UsePhysicsOptimizations

    mwseConfig.UsePhysicsOptimizations = false
    local refResults, offTime, offHits = castPass(origin, dirs, findAll, maxDistance)

    mwseConfig.UsePhysicsOptimizations = true
    local onResults, onTime1, onHits1 = castPass(origin, dirs, findAll, maxDistance)
    local bad1 = comparePasses(refResults, onResults, dirs, "on-1st")

    local onResults2, onTime2, onHits2 = castPass(origin, dirs, findAll, maxDistance)
    local bad2 = comparePasses(refResults, onResults2, dirs, "on-2nd")
    if onHits2 ~= onHits1 then
        log("NOTE: on-2nd total hits %d vs on-1st %d", onHits2, onHits1)
    end

    mwseConfig.UsePhysicsOptimizations = saved

    log("pass off:    %8.1f ms  (%.4f ms/ray, %d hits)",
        offTime * 1000.0, offTime * 1000.0 / rayCount, offHits)
    log("pass on-1st: %8.1f ms  (%.4f ms/ray, %d hits, includes BVH builds)",
        onTime1 * 1000.0, onTime1 * 1000.0 / rayCount, onHits1)
    log("pass on-2nd: %8.1f ms  (%.4f ms/ray, warm)",
        onTime2 * 1000.0, onTime2 * 1000.0 / rayCount)

    local totalBad = bad1 + bad2
    if totalBad == 0 then
        local speedup = (onTime2 > 0) and (offTime / onTime2) or 0
        log("=== PASS: %d rays identical on vs off, warm speedup %.1fx ===", rayCount, speedup)
        return true
    end

    log("=== FAIL: %d of %d comparisons differ (on-1st %d, on-2nd %d) ===",
        totalBad, rayCount * 2, bad1, bad2)
    return false
end

return M
