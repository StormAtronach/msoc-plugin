-- Active-grid batching ceiling census.
--
-- Answers, for the loaded cells, how far the static draw population could
-- collapse if shapes that share one resolved render state were merged into
-- pooled NiTriShapeDynamicData batches (G7's active-grid batching), under the
-- constraints the engine imposes (see
-- docs/plans/active-grid-batching-plan.md):
--
--   * one draw = one (property signature, spatial bin) group, split at the
--     16-bit vertex limit of NiGeometryData;
--   * a batch inherits the union of its members' point lights, and
--     NiNode::PushLocalEffects keeps at most 7 point/spot lights per node, so
--     a group whose union exceeds 7 loses lights somewhere;
--   * alpha-blended shapes stay out (NiAlphaAccumulator sorts them per
--     object); alpha-tested shapes are fine;
--   * anything animated, skinned, switched, billboarded or BSP-split stays
--     out. Node-level blockers reject the subtree; shape-level blockers reject
--     only that shape, so a building with one blended window pane still
--     contributes its opaque walls.
--
-- Runs inside the harness's anatomy pass (msocperf/main.lua, runAnatomy) and
-- prints BATCH lines to MWSE.log next to the ANATOMY lines. Nothing here is
-- timed; the walk reads what the engine holds.
--
-- Two views of the same population are reported:
--   population  every eligible shape in the active cells, regardless of view;
--   per-view    the subset whose bound intersects a horizontal wedge from the
--               player's eye for each of 8 headings (the harness rotation),
--               and the exact frustum of the current view from the camera's
--               culling planes. Draws per frame scale with the per-view rows,
--               not the population rows. Neither view sees occlusion, so both
--               are upper bounds on a population the culler has already
--               thinned; the `total` collapse rows include the residual draws
--               batching cannot touch and are the numbers the gate reads.

local M = {}

local function say(fmt, ...)
    mwse.log("%s", "[msocperf] " .. string.format(fmt, ...))
end

local VERTEX_LIMIT = 65535
local LIGHT_LIMIT = 7
local BIN_SIZES = { 2048, 4096, 8192, 16384 }
local HEADINGS = 8
-- Horizontal half-angle of the wedge. MGE XE installs commonly run a 75-90
-- degree horizontal FOV; 50 degrees of half-angle over-approximates so the
-- per-view rows err on the side of counting a draw rather than missing one.
local HALF_FOV = math.rad(50)
local PLANE_COUNT = 6

local function safe(fn, default)
    local ok, r = pcall(fn)
    if ok then return r end
    return default
end

local function typeOf(name)
    return safe(function() return ni.type[name] end, nil)
end

-- Node-level blockers: the whole subtree is out. NiBSPNode and NiSwitchNode
-- have their own Display that calls CullShow on children directly, bypassing
-- any hide bit set on the child; the others animate or re-orient.
local rejectTypes = {}
for _, n in ipairs({ "NiBillboardNode", "NiSwitchNode", "NiLODNode", "NiBSPNode",
                     "NiBSAnimationNode", "NiBSParticleNode", "NiParticles",
                     "NiAutoNormalParticles", "NiRotatingParticles" }) do
    local t = typeOf(n)
    if t then rejectTypes[#rejectTypes + 1] = { name = n, type = t } end
end
local triGeomType = typeOf("NiTriBasedGeom")
local triStripsType = typeOf("NiTriStrips")
local pointLightType = typeOf("NiPointLight")
local spotLightType = typeOf("NiSpotLight")
local lightType = typeOf("NiLight")

local function isType(obj, t)
    if not t then return false end
    local ok, r = pcall(function() return obj:isInstanceOfType(t) end)
    return ok and r == true
end

-- tes3.objectType keys are lowercase words ("static", "activator"), not the
-- FourCC mnemonics. Reverse the table once so a reference's type prints and
-- compares as the word.
local objTypeName = {}
pcall(function()
    for name, value in pairs(tes3.objectType) do objTypeName[value] = name end
end)

local function countKey(t, k)
    if k == nil then return end
    t[k] = (t[k] or 0) + 1
end

local function tally(t)
    local n = 0
    for _ in pairs(t) do n = n + 1 end
    return n
end

local function topN(counts, n)
    local list = {}
    for k, v in pairs(counts) do list[#list + 1] = { k = k, v = v } end
    table.sort(list, function(a, b)
        if a.v ~= b.v then return a.v > b.v end
        return tostring(a.k) < tostring(b.k)
    end)
    local out = {}
    for i = 1, math.min(n, #list) do out[i] = list[i] end
    return out
end

local function ratio(a, b)
    if not b or b == 0 then return 0 end
    return a / b
end

-- ----------------------------------------------------------- render state

-- NetImmerse properties inherit down the tree and the nearest one wins, so
-- the resolved state of a shape is the first property of each type found
-- walking from the shape to the root. This is what NiGeometry::spPropertyState
-- ends up holding. Stops as soon as every slot is filled.
local PROPS = { "texturingProperty", "materialProperty", "alphaProperty",
                "stencilProperty", "vertexColorProperty", "zBufferProperty",
                "fogProperty" }

local function resolveProperties(shape)
    local out, missing = {}, #PROPS
    local node, depth = shape, 0
    while node and depth < 32 and missing > 0 do
        for _, key in ipairs(PROPS) do
            if out[key] == nil then
                local p = safe(function() return node[key] end, nil)
                if p then
                    out[key] = p
                    missing = missing - 1
                end
            end
        end
        node = safe(function() return node.parent end, nil)
        depth = depth + 1
    end
    return out
end

local function mapSig(map)
    if not map then return "-" end
    local name = safe(function() return map.texture.fileName end, nil) or "?"
    return string.format("%s/%s/%s/%s", string.lower(name),
        tostring(safe(function() return map.clampMode end, "")),
        tostring(safe(function() return map.filterMode end, "")),
        tostring(safe(function() return map.texCoordSet end, "")))
end

local function colorSig(c)
    if not c then return "-" end
    return string.format("%.2f,%.2f,%.2f", c.r, c.g, c.b)
end

--- Everything NiDX8Renderer reads to set up one draw, folded into a string.
--- Two shapes with equal signatures can share a batch; the FVF part (normals,
--- colours, texture sets) is included because a batch has one vertex layout.
--- Not visible from Lua and therefore missing: the renderer-specific property,
--- wireframe and specular. Treat the signature count as a lower bound.
local function signature(props, data)
    local parts = {}
    local tp = props.texturingProperty
    if tp then
        parts[#parts + 1] = "T:" .. tostring(safe(function() return tp.applyMode end, ""))
            .. "|" .. mapSig(safe(function() return tp.baseMap end, nil))
            .. "|" .. mapSig(safe(function() return tp.darkMap end, nil))
            .. "|" .. mapSig(safe(function() return tp.detailMap end, nil))
            .. "|" .. mapSig(safe(function() return tp.glossMap end, nil))
            .. "|" .. mapSig(safe(function() return tp.glowMap end, nil))
            .. "|" .. mapSig(safe(function() return tp.bumpMap end, nil))
            .. "|d" .. tostring(safe(function() return tp.decalCount end, 0))
    else
        parts[#parts + 1] = "T:-"
    end
    local mp = props.materialProperty
    if mp then
        parts[#parts + 1] = "M:" .. colorSig(safe(function() return mp.ambient end, nil))
            .. "|" .. colorSig(safe(function() return mp.diffuse end, nil))
            .. "|" .. colorSig(safe(function() return mp.specular end, nil))
            .. "|" .. colorSig(safe(function() return mp.emissive end, nil))
            .. "|a" .. string.format("%.2f", safe(function() return mp.alpha end, 1))
            .. "|s" .. string.format("%.1f", safe(function() return mp.shininess end, 0))
    else
        parts[#parts + 1] = "M:-"
    end
    local ap = props.alphaProperty
    if ap then
        parts[#parts + 1] = "A:" .. tostring(safe(function() return ap.propertyFlags end, ""))
            .. "|" .. tostring(safe(function() return ap.alphaTestRef end, ""))
    else
        parts[#parts + 1] = "A:-"
    end
    local sp = props.stencilProperty
    if sp then
        parts[#parts + 1] = "S:" .. tostring(safe(function() return sp.enabled end, ""))
            .. "|" .. tostring(safe(function() return sp.drawMode end, ""))
    else
        parts[#parts + 1] = "S:-"
    end
    local vp = props.vertexColorProperty
    if vp then
        parts[#parts + 1] = "V:" .. tostring(safe(function() return vp.source end, ""))
            .. "|" .. tostring(safe(function() return vp.lighting end, ""))
    else
        parts[#parts + 1] = "V:-"
    end
    local zp = props.zBufferProperty
    if zp then
        parts[#parts + 1] = "Z:" .. tostring(safe(function() return zp.propertyFlags end, ""))
            .. "|" .. tostring(safe(function() return zp.testFunction end, ""))
    else
        parts[#parts + 1] = "Z:-"
    end
    parts[#parts + 1] = "F:" .. (props.fogProperty and "1" or "0")
    if data then
        parts[#parts + 1] = "L:" .. (safe(function() return data.normals end, nil) and "n" or "-")
            .. (safe(function() return data.colors end, nil) and "c" or "-")
            .. tostring(safe(function() return data.textureSets end, 1))
    end
    return table.concat(parts, ";")
end

-- ----------------------------------------------------------- effects

--- Split a node's effect list into light keys and a flag for anything that
--- is not a light (NiTextureEffect environment maps ride the same list and
--- would be inherited by a batch just as lights are).
local function nodeEffects(node)
    local lights, nonLight = {}, false
    local e = safe(function() return node.effectList end, nil)
    local guard = 0
    while e and guard < 64 do
        local d = safe(function() return e.data end, nil)
        if d then
            if isType(d, pointLightType) or isType(d, spotLightType) then
                local t = safe(function() return d.worldTransform.translation end, nil)
                local key = tostring(safe(function() return d.name end, "?"))
                if t then key = key .. string.format("@%.0f,%.0f,%.0f", t.x, t.y, t.z) end
                lights[#lights + 1] = key
            elseif not isType(d, lightType) then
                nonLight = true
            end
        end
        e = safe(function() return e.next end, nil)
        guard = guard + 1
    end
    return lights, nonLight
end

-- ----------------------------------------------------------- reference walk

--- Walk one reference's subtree with per-shape eligibility. Returns the
--- eligible shapes with signatures and bounds, plus the rejected shapes with
--- their reason, so the residual is counted exactly once.
local function walkReference(ref)
    local info = { shapes = {}, rejected = {}, strips = 0, highFlags = {} }
    local node = ref.sceneNode

    local function visit(n, depth, blocked)
        if depth > 32 then return end
        -- Hidden subtrees (RootCollisionNode and friends carry the NIF hidden
        -- flag) never render, so they are neither draws nor obstacles.
        if safe(function() return n.appCulled end, false) then return end
        if not blocked then
            for _, rt in ipairs(rejectTypes) do
                if isType(n, rt.type) then blocked = rt.name; break end
            end
            if not blocked and safe(function() return n.controller end, nil) then
                blocked = "controller"
            end
            if not blocked and not isType(n, triGeomType) then
                local _, nonLight = nodeEffects(n)
                if nonLight then blocked = "textureEffect" end
            end
        end
        if isType(n, triGeomType) then
            local data = safe(function() return n.data end, nil)
            local verts = data and safe(function() return data.vertexCount end, 0) or 0
            local tris = data and safe(function() return data.triangleCount end, 0) or 0
            if verts == 0 then return end
            local o = safe(function() return n.worldBoundOrigin end, nil)
            local shape = {
                verts = verts, tris = tris,
                x = o and o.x or 0, y = o and o.y or 0, z = o and o.z or 0,
                radius = safe(function() return n.worldBoundRadius end, 0),
            }
            local flags = safe(function() return n.flags end, 0) or 0
            local high = bit.band(flags, 0xFF00)
            if high ~= 0 then countKey(info.highFlags, string.format("0x%04x", high)) end
            local reason = blocked
            if not reason and safe(function() return n.skinInstance end, nil) then reason = "skinned" end
            if not reason and isType(n, triStripsType) then
                -- niTriStrips does not expose skinInstance to Lua, so a skinned
                -- strip would pass. Strips are rare in Morrowind content and
                -- counted separately.
                info.strips = info.strips + 1
            end
            local props = reason == nil and resolveProperties(n) or nil
            if not reason then
                for _, key in ipairs({ "texturingProperty", "materialProperty", "alphaProperty" }) do
                    local p = props[key]
                    if p and safe(function() return p.controller end, nil) then
                        reason = "propertyController"
                        break
                    end
                end
            end
            if not reason then
                local ap = props.alphaProperty
                local aflags = ap and safe(function() return ap.propertyFlags end, 0) or 0
                if bit.band(aflags, 1) ~= 0 then reason = "alphaBlend" end
            end
            if not reason and verts > VERTEX_LIMIT then reason = "oversize" end
            if reason then
                shape.reason = reason
                info.rejected[#info.rejected + 1] = shape
            else
                shape.sig = signature(props, data)
                info.shapes[#info.shapes + 1] = shape
            end
            return
        end
        local children = safe(function() return n.children end, nil)
        if children then
            local count = safe(function() return #children end, 0)
            for i = 1, count do
                local c = safe(function() return children[i] end, nil)
                if c then visit(c, depth + 1, blocked) end
            end
        end
    end

    visit(node, 0, nil)
    return info
end

-- ----------------------------------------------------------- view tests

local function eyePosition()
    local p = safe(function() return tes3.getPlayerEyePosition() end, nil)
    if p then return p.x, p.y, p.z end
    local pp = safe(function() return tes3.player.position end, nil)
    if pp then return pp.x, pp.y, pp.z + 100 end
    return 0, 0, 0
end

-- niCamera.viewDistance came back as garbage on this install (1.6e29 in one
-- run, tiny in another), so the wedge uses a fixed far plane a little over the
-- 3x3 active grid. The frustum rows use the real culling planes and are the
-- ones to quote.
local function viewFar()
    return 8192 * 3
end

--- Sphere against a horizontal wedge: heading in radians, half-angle, far.
local function inWedge(ex, ey, heading, far, x, y, radius)
    local dx, dy = x - ex, y - ey
    local dist = math.sqrt(dx * dx + dy * dy)
    if dist - radius > far then return false end
    if dist <= radius then return true end
    local ang = math.atan2(dx, dy) - heading
    while ang > math.pi do ang = ang - 2 * math.pi end
    while ang < -math.pi do ang = ang + 2 * math.pi end
    local pad = math.asin(math.min(1, radius / dist))
    return math.abs(ang) <= HALF_FOV + pad
end

--- Sphere against the camera's real culling planes, same sign convention as
--- NiAVObject::CullShow: d = n.p - w, culled when d <= -radius on any plane.
--- Read with a numeric loop; the planes come through as a fixed C array and
--- ipairs is not guaranteed to iterate it under LuaJIT.
local function currentPlanes()
    local planes = {}
    local ok, err = pcall(function()
        local cam = tes3.worldController.worldCamera.cameraData.camera
        local arr = cam.cullingPlanes
        for i = 1, PLANE_COUNT do
            local p = arr[i]
            if p then planes[#planes + 1] = { x = p.x, y = p.y, z = p.z, w = p.w } end
        end
    end)
    if not ok then
        say("BATCH cullingPlanes unavailable: %s", tostring(err))
        return nil
    end
    if #planes == 0 then
        say("BATCH cullingPlanes read returned no planes")
        return nil
    end
    return planes
end

local function inFrustum(planes, x, y, z, radius)
    for _, p in ipairs(planes) do
        local d = p.x * x + p.y * y + p.z * z - p.w
        if d <= -radius then return false end
    end
    return true
end

-- ----------------------------------------------------------- grouping

--- Group eligible shapes by (signature, bin) for one bin size. Returns the
--- list of batches, each split at the vertex limit, with an enclosing bound
--- and the light union of its own members.
local function buildBatches(eligible, binSize)
    local groups = {}
    for _, s in ipairs(eligible) do
        local bx = math.floor(s.x / binSize)
        local by = math.floor(s.y / binSize)
        local key = s.sig .. "#" .. bx .. "," .. by
        local g = groups[key]
        if not g then
            g = { sig = s.sig, members = {} }
            groups[key] = g
        end
        g.members[#g.members + 1] = s
    end

    local batches = {}
    for _, g in pairs(groups) do
        -- Split greedily at the vertex limit, in member order. Each split is a
        -- draw of its own, with its own enclosing bound and light union.
        local cur = nil
        local function flush()
            if not cur or #cur.members == 0 then return end
            local cx, cy, cz = 0, 0, 0
            for _, m in ipairs(cur.members) do
                cx, cy, cz = cx + m.x, cy + m.y, cz + m.z
            end
            local n = #cur.members
            cx, cy, cz = cx / n, cy / n, cz / n
            local r = 0
            local lights, nLights = {}, 0
            for _, m in ipairs(cur.members) do
                local dx, dy, dz = m.x - cx, m.y - cy, m.z - cz
                local d = math.sqrt(dx * dx + dy * dy + dz * dz) + m.radius
                if d > r then r = d end
                for _, l in ipairs(m.lights) do
                    if not lights[l] then
                        lights[l] = true
                        nLights = nLights + 1
                    end
                end
            end
            cur.x, cur.y, cur.z, cur.radius = cx, cy, cz, r
            cur.nLights = nLights
            batches[#batches + 1] = cur
            cur = nil
        end
        for _, m in ipairs(g.members) do
            if cur and cur.verts + m.verts > VERTEX_LIMIT then flush() end
            if not cur then cur = { sig = g.sig, members = {}, verts = 0, tris = 0 } end
            cur.members[#cur.members + 1] = m
            cur.verts = cur.verts + m.verts
            cur.tris = cur.tris + m.tris
        end
        flush()
    end
    return batches, tally(groups)
end

-- ----------------------------------------------------------- entry

function M.collect(siteName, extraTypes, label)
    local t0 = os.clock()
    local stats = {
        cells = 0, refs = 0, statRefs = 0, eligibleRefs = 0, partialRefs = 0, rejectedRefs = 0,
        noNode = 0, rejectReasons = {}, residualByType = {}, highFlags = {},
        eligible = {}, residual = {},   -- residual: visible shapes we cannot batch
        eligibleVerts = 0, eligibleTris = 0, strips = 0,
        maxRefLights = 0, refsWithLights = 0,
    }

    -- Object types considered for batching, as tes3.objectType key words.
    -- "static" only by default; pass e.g. { "activator", "container" } as the
    -- second argument to widen it and see what the rejection rules make of them.
    local wanted = { static = true }
    for _, name in ipairs(extraTypes or {}) do wanted[string.lower(name)] = true end

    for _, cell in ipairs(tes3.getActiveCells()) do
        stats.cells = stats.cells + 1
        for ref in cell:iterateReferences() do
            stats.refs = stats.refs + 1
            local node = ref.sceneNode
            if not node then
                stats.noNode = stats.noNode + 1
                goto continue
            end
            local objType = "unknown"
            pcall(function()
                objType = objTypeName[ref.object.objectType] or tostring(ref.object.objectType)
            end)
            local info = walkReference(ref)
            for k, v in pairs(info.highFlags) do
                stats.highFlags[k] = (stats.highFlags[k] or 0) + v
            end
            if #info.shapes == 0 and #info.rejected == 0 then
                goto continue
            end
            if wanted[objType] then stats.statRefs = stats.statRefs + 1 end

            local lights, _ = nodeEffects(node)
            if wanted[objType] and #info.shapes > 0 then
                stats.eligibleRefs = stats.eligibleRefs + 1
                if #info.rejected > 0 then stats.partialRefs = stats.partialRefs + 1 end
                if #lights > 0 then stats.refsWithLights = stats.refsWithLights + 1 end
                if #lights > stats.maxRefLights then stats.maxRefLights = #lights end
                for _, s in ipairs(info.shapes) do
                    s.lights = lights
                    stats.eligible[#stats.eligible + 1] = s
                    stats.eligibleVerts = stats.eligibleVerts + s.verts
                    stats.eligibleTris = stats.eligibleTris + s.tris
                end
                stats.strips = stats.strips + info.strips
            elseif wanted[objType] then
                stats.rejectedRefs = stats.rejectedRefs + 1
            end

            -- Everything that still draws after batching: rejected shapes of
            -- wanted references, and every shape of every other reference.
            if wanted[objType] then
                for _, s in ipairs(info.rejected) do
                    stats.residual[#stats.residual + 1] = s
                    countKey(stats.rejectReasons, s.reason)
                    local label = objType .. "/" .. tostring(s.reason)
                    stats.residualByType[label] = (stats.residualByType[label] or 0) + 1
                end
            else
                local n = #info.shapes + #info.rejected
                for _, s in ipairs(info.shapes) do stats.residual[#stats.residual + 1] = s end
                for _, s in ipairs(info.rejected) do stats.residual[#stats.residual + 1] = s end
                stats.residualByType[objType] = (stats.residualByType[objType] or 0) + n
            end
            ::continue::
        end
    end

    local walk = os.clock() - t0
    local cellName = "?"
    pcall(function()
        local c = tes3.player.cell
        if c then cellName = tostring(c.name or c.id or "?") end
    end)

    local nEligible = #stats.eligible
    local nResidual = #stats.residual
    local sigs = {}
    for _, s in ipairs(stats.eligible) do countKey(sigs, s.sig) end

    say("BATCH site=%s cell=%s types=%s walk=%.2fs", tostring(siteName), cellName,
        tostring(label or "static"), walk)
    say("BATCH refs=%d noNode=%d static=%d eligibleRefs=%d partialRefs=%d rejectedRefs=%d",
        stats.refs, stats.noNode, stats.statRefs, stats.eligibleRefs, stats.partialRefs, stats.rejectedRefs)
    if stats.statRefs == 0 then
        say("BATCH WARNING no references matched the wanted object types; every row below is empty")
    end
    local reasons = {}
    for _, e in ipairs(topN(stats.rejectReasons, 12)) do
        reasons[#reasons + 1] = string.format("%s=%d", tostring(e.k), e.v)
    end
    say("BATCH rejectReasons(shapes) %s", table.concat(reasons, " "))
    say("BATCH population eligibleShapes=%d verts=%d tris=%d strips=%d residualShapes=%d signatures=%d",
        nEligible, stats.eligibleVerts, stats.eligibleTris, stats.strips, nResidual, tally(sigs))
    say("BATCH lights refsWithLights=%d maxLightsOnRef=%d", stats.refsWithLights, stats.maxRefLights)
    local resid = {}
    for _, e in ipairs(topN(stats.residualByType, 14)) do
        resid[#resid + 1] = string.format("%s=%d", tostring(e.k), e.v)
    end
    say("BATCH residualByType %s", table.concat(resid, " "))
    -- Any NIF-loaded shape already carrying a high flag bit would collide with
    -- a private render-hide bit; report what the loaded scene holds.
    local hf = {}
    for _, e in ipairs(topN(stats.highFlags, 8)) do
        hf[#hf + 1] = string.format("%s=%d", tostring(e.k), e.v)
    end
    say("BATCH flagsHighByte %s", #hf > 0 and table.concat(hf, " ") or "none")

    -- Per-view setup: eye position, 8 headings, the real frustum for the
    -- current view.
    local ex, ey, ez = eyePosition()
    local far = viewFar()
    local planes = currentPlanes()

    local function viewCounts(items)
        local perHeading = {}
        for h = 0, HEADINGS - 1 do
            local heading = h * (2 * math.pi / HEADINGS)
            local n = 0
            for _, it in ipairs(items) do
                if inWedge(ex, ey, heading, far, it.x, it.y, it.radius) then n = n + 1 end
            end
            perHeading[#perHeading + 1] = n
        end
        local sum, mx = 0, 0
        for _, n in ipairs(perHeading) do
            sum = sum + n
            if n > mx then mx = n end
        end
        local frustum = -1
        if planes then
            frustum = 0
            for _, it in ipairs(items) do
                if inFrustum(planes, it.x, it.y, it.z, it.radius) then frustum = frustum + 1 end
            end
        end
        return sum / HEADINGS, mx, frustum
    end

    local function viewTris(items)
        local sum = 0
        for h = 0, HEADINGS - 1 do
            local heading = h * (2 * math.pi / HEADINGS)
            for _, it in ipairs(items) do
                if inWedge(ex, ey, heading, far, it.x, it.y, it.radius) then sum = sum + it.tris end
            end
        end
        return sum / HEADINGS
    end

    local eligAvg, eligMax, eligFrustum = viewCounts(stats.eligible)
    local residAvg, residMax, residFrustum = viewCounts(stats.residual)
    local shapeTrisAvg = viewTris(stats.eligible)
    say("BATCH view far=%.0f eligibleShapes avg=%.0f max=%d frustum=%d residualShapes avg=%.0f max=%d frustum=%d drawsBefore avg=%.0f frustum=%d",
        far, eligAvg, eligMax, eligFrustum, residAvg, residMax, residFrustum,
        eligAvg + residAvg, (eligFrustum >= 0 and residFrustum >= 0) and (eligFrustum + residFrustum) or -1)

    for _, binSize in ipairs(BIN_SIZES) do
        local batches, groups = buildBatches(stats.eligible, binSize)
        local over, maxLights, withLights, maxTris, maxRadius, sumTris = 0, 0, 0, 0, 0, 0
        for _, b in ipairs(batches) do
            if b.nLights > LIGHT_LIMIT then over = over + 1 end
            if b.nLights > 0 then withLights = withLights + 1 end
            if b.nLights > maxLights then maxLights = b.nLights end
            if b.tris > maxTris then maxTris = b.tris end
            if b.radius > maxRadius then maxRadius = b.radius end
            sumTris = sumTris + b.tris
        end
        local bAvg, bMax, bFrustum = viewCounts(batches)
        -- Triangles the GPU would rasterise per view: every batch in view
        -- draws all of its triangles, culled or not.
        local trisAvg = viewTris(batches)
        local totalBefore = eligAvg + residAvg
        local totalAfter = bAvg + residAvg
        local totalBeforeF = (eligFrustum >= 0 and residFrustum >= 0) and (eligFrustum + residFrustum) or -1
        local totalAfterF = (bFrustum >= 0 and residFrustum >= 0) and (bFrustum + residFrustum) or -1

        say("BATCH bin=%d groups=%d batches=%d populationCollapse=%.2fx avgTris=%.0f maxTris=%d maxRadius=%.0f",
            binSize, groups, #batches, ratio(nEligible, #batches), ratio(sumTris, #batches), maxTris, maxRadius)
        say("BATCH bin=%d lights batchesWithLights=%d over7=%d (%.1f%%) maxLights=%d",
            binSize, withLights, over, 100 * ratio(over, #batches), maxLights)
        say("BATCH bin=%d view batches avg=%.0f max=%d frustum=%d eligibleCollapse=%.2fx",
            binSize, bAvg, bMax, bFrustum, ratio(eligAvg, bAvg))
        say("BATCH bin=%d total drawsBefore=%.0f drawsAfter=%.0f totalCollapse=%.2fx frustumBefore=%d frustumAfter=%d frustumCollapse=%.2fx trisInView shapes=%.0f batches=%.0f (%.2fx)",
            binSize, totalBefore, totalAfter, ratio(totalBefore, totalAfter),
            totalBeforeF, totalAfterF, (totalBeforeF > 0 and totalAfterF > 0) and (totalBeforeF / totalAfterF) or 0,
            shapeTrisAvg, trisAvg, ratio(trisAvg, shapeTrisAvg))
    end

    for _, e in ipairs(topN(sigs, 12)) do
        say("BATCH topsig %d %s", e.v, tostring(e.k))
    end

    return stats
end

return M
