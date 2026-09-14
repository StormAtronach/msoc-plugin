-- Active-cell render anatomy probe.
--
-- Characterises what the engine is actually asked to draw in a loaded cell, to
-- size the ways of collapsing draw calls that keep the scenegraph authoritative
-- and mutable. Baking the cell into merged buffers is off the table, so the
-- only candidates are instancing (many live nodes, one shared vertex/index
-- buffer) and state merging (many draws, one material bind).
--
-- Prior measurement settled the state-grouping half: the engine's own
-- BeginBatch/EndBatch API caps at a ~1.4-1.55x content-hash ceiling and is
-- already achieving 1.35x of it, because 70-80% of per-shape cost is per-entry
-- work (SetStreamSource, SetIndices, SetModelTransform, DrawIndexedPrimitive)
-- that no state key can amortise. Collapsing that requires fewer, bigger draws.
--
-- So the question this probe answers is whether the geometry is even shareable.
-- `niGeometryData.uniqueID` is assigned at model creation. If two references of
-- the same model share an ID, the engine already shares one buffer pair and
-- instancing is reachable from the graph as it stands. If every shape carries
-- its own ID, the NIF loader clones per instance -- the same structural fact
-- that makes NiPropertyState pointer-unique -- and any collapse has to dedupe
-- against the source mesh path and build its own buffers first.
--
-- Counting distinct IDs against distinct mesh paths separates those two worlds
-- directly, and the gap between them is the cost of the second one.

local M = {}

local function say(fmt, ...)
    mwse.log("[msocperf] " .. string.format(fmt, ...))
end

-- Morrowind meshes are mostly NiTriShape, but NiTriStrips exists and the
-- renderer has a separate RenderTristrips path for it, so both are draws.
-- Built defensively: an absent enum name would otherwise abort the traverse.
local function geometryTypes()
    local types = {}
    for _, name in ipairs({ "NiTriShape", "NiTriStrips" }) do
        local t = ni.type[name]
        if t then types[#types + 1] = t end
    end
    return types
end

-- NetImmerse properties inherit down the tree: a texturing property attached to
-- an ancestor NiNode applies to every shape under it unless one overrides it.
-- Reading only the shape's own slot reported no texture for 35% of shapes on
-- the first run, which is the inherited case, not an untextured one.
local function baseTextureOf(shape)
    local node, depth = shape, 0
    while node and depth < 16 do
        local ok, name = pcall(function()
            local tp = node.texturingProperty
            if not tp then return nil end
            local map = tp.baseMap
            if not map then return nil end
            local tex = map.texture
            if not tex then return nil end
            return tex.fileName
        end)
        if ok and name then return name end
        local parent
        if not pcall(function() parent = node.parent end) then return nil end
        node, depth = parent, depth + 1
    end
    return nil
end

-- tes3.objectType values are FourCCs ('STAT', 'NPC_'), which print as large
-- integers. Reverse the enum once so the breakdown is readable.
local objTypeName = {}
pcall(function()
    for name, value in pairs(tes3.objectType) do
        objTypeName[value] = name
    end
end)

-- Triangle-count buckets. Draw-call cost is near flat in triangle count at
-- these sizes, so a population skewed to tiny shapes is the signature of a
-- draw-bound frame -- which is the case worth attacking.
local function bucketOf(n)
    if n < 16 then return "0-15" end
    if n < 64 then return "16-63" end
    if n < 256 then return "64-255" end
    if n < 1024 then return "256-1023" end
    return "1024+"
end

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

--- Walk every reference in every active cell and attribute its shapes back to
--- the source model, which is the axis instancing would key on.
local function walkReferences(stats)
    local types = geometryTypes()

    for _, cell in ipairs(tes3.getActiveCells()) do
        stats.cells = stats.cells + 1
        for ref in cell:iterateReferences() do
            stats.refs = stats.refs + 1

            local node = ref.sceneNode
            if not node then
                stats.refsNoNode = stats.refsNoNode + 1
                goto continue
            end

            local objType = "unknown"
            local mesh = nil
            pcall(function()
                if ref.object then
                    local t = ref.object.objectType
                    objType = objTypeName[t] or tostring(t)
                    mesh = ref.object.mesh
                end
            end)
            if mesh then mesh = string.lower(mesh) end

            countKey(stats.byMesh, mesh or "<none>")

            local shapesHere = 0
            local ok = pcall(function()
                for shape in node:traverse({ type = types }) do
                    shapesHere = shapesHere + 1
                    stats.shapes = stats.shapes + 1

                    local culled = false
                    pcall(function() culled = shape:isAppCulled() end)
                    if culled then stats.shapesCulled = stats.shapesCulled + 1 end

                    local skinned = false
                    pcall(function() skinned = shape.skinInstance ~= nil end)
                    if skinned then stats.shapesSkinned = stats.shapesSkinned + 1 end

                    local hasAlpha = false
                    pcall(function() hasAlpha = shape.alphaProperty ~= nil end)
                    if hasAlpha then stats.shapesAlpha = stats.shapesAlpha + 1 end

                    local geomID, tris, verts
                    pcall(function()
                        local data = shape.data
                        if data then
                            geomID = data.uniqueID
                            tris = data.triangleCount
                            verts = data.vertexCount
                        end
                    end)

                    tris = tris or 0
                    verts = verts or 0
                    stats.tris = stats.tris + tris
                    stats.verts = stats.verts + verts
                    countKey(stats.triHist, bucketOf(tris))

                    local tex = baseTextureOf(shape)
                    if tex then tex = string.lower(tex) end

                    countKey(stats.byGeomID, geomID)
                    countKey(stats.byTexture, tex or "<none>")
                    if not culled then
                        stats.shapesVis = stats.shapesVis + 1
                        stats.trisVis = stats.trisVis + tris
                        countKey(stats.byGeomIDVis, geomID)
                        countKey(stats.byTextureVis, tex or "<none>")
                    end
                    if geomID then
                        countKey(stats.byGeomTex, tostring(geomID) .. "|" .. (tex or "<none>"))
                    end
                    if mesh then
                        countKey(stats.byMeshTex, mesh .. "|" .. (tex or "<none>"))
                    end

                    -- Attribute draws to the kind of object that owns them, so
                    -- the static share (the part instancing could reach) is
                    -- separable from actors and other animated content.
                    stats.byObjType[objType] = (stats.byObjType[objType] or 0) + 1
                end
            end)
            if not ok then stats.walkErrors = stats.walkErrors + 1 end

            if shapesHere == 0 then stats.refsNoShapes = stats.refsNoShapes + 1 end

            ::continue::
        end
    end
end

--- Total shape population under the world camera, for the delta against what
--- the reference walk attributed. The remainder is sky, water and anything
--- else parented outside a cell reference.
local function walkCameraRoot(stats)
    -- `cameraRoot` is the camera's own node and holds no scene geometry; the
    -- world subtree hangs off `root`. The first run walked the former and
    -- reported zero shapes, so try both and record which one answered.
    local types = geometryTypes()
    for _, field in ipairs({ "root", "cameraRoot" }) do
        local count, culled = 0, 0
        local ok = pcall(function()
            local cam = tes3.worldController.worldCamera
            local root = cam and cam[field]
            if not root then return end
            for shape in root:traverse({ type = types }) do
                count = count + 1
                local c = false
                pcall(function() c = shape:isAppCulled() end)
                if c then culled = culled + 1 end
            end
        end)
        if ok and count > 0 then
            stats.rootShapes = count
            stats.rootCulled = culled
            stats.rootField = field
            return
        end
    end
end

function M.collect(siteName)
    local stats = {
        cells = 0, refs = 0, refsNoNode = 0, refsNoShapes = 0, walkErrors = 0,
        shapes = 0, shapesCulled = 0, shapesSkinned = 0, shapesAlpha = 0,
        tris = 0, verts = 0,
        rootShapes = 0, rootCulled = 0, rootField = "none",
        shapesVis = 0, trisVis = 0, byGeomIDVis = {}, byTextureVis = {},
        byMesh = {}, byTexture = {}, byGeomID = {}, byGeomTex = {},
        byMeshTex = {}, byObjType = {}, triHist = {},
    }

    local t0 = os.clock()
    walkReferences(stats)
    walkCameraRoot(stats)
    local elapsed = os.clock() - t0

    local cellName = "?"
    pcall(function()
        local c = tes3.player.cell
        if c then cellName = tostring(c.name or c.id or "?") end
    end)

    local nGeom = tally(stats.byGeomID)
    local nMesh = tally(stats.byMesh)
    local nTex = tally(stats.byTexture)
    local nGeomTex = tally(stats.byGeomTex)
    local nMeshTex = tally(stats.byMeshTex)
    local nGeomVis = tally(stats.byGeomIDVis)
    local nTexVis = tally(stats.byTextureVis)

    say("ANATOMY site=%s cell=%s walk=%.2fs", tostring(siteName), cellName, elapsed)
    say("ANATOMY cells=%d refs=%d noNode=%d noShapes=%d errors=%d",
        stats.cells, stats.refs, stats.refsNoNode, stats.refsNoShapes, stats.walkErrors)
    say("ANATOMY shapes=%d culled=%d skinned=%d alpha=%d tris=%d verts=%d",
        stats.shapes, stats.shapesCulled, stats.shapesSkinned, stats.shapesAlpha,
        stats.tris, stats.verts)
    say("ANATOMY rootShapes=%d rootCulled=%d rootField=%s outsideRefs=%d",
        stats.rootShapes, stats.rootCulled, tostring(stats.rootField),
        stats.rootShapes - stats.shapes)
    say("ANATOMY distinct geomID=%d mesh=%d tex=%d geomTex=%d meshTex=%d",
        nGeom, nMesh, nTex, nGeomTex, nMeshTex)

    -- The two numbers the decision turns on.
    --
    -- geomShare > 1 means the engine already hands the same buffer pair to
    -- several shapes, so instancing can key on it directly. geomShare ~= 1 with
    -- meshShare > 1 means the loader clones per instance and the collapse has
    -- to be rebuilt against the mesh path.
    say("ANATOMY ceiling geomShare=%.2fx meshShare=%.2fx texMerge=%.2fx geomTexInstance=%.2fx",
        ratio(stats.shapes, nGeom), ratio(stats.shapes, nMesh),
        ratio(stats.shapes, nTex), ratio(stats.shapes, nGeomTex))

    -- The population that actually reaches the renderer. appCulled shapes are
    -- skipped before submission, so this is the row to quote.
    say("ANATOMY visible shapes=%d tris=%d geomID=%d tex=%d geomShare=%.2fx texMerge=%.2fx",
        stats.shapesVis, stats.trisVis, nGeomVis, nTexVis,
        ratio(stats.shapesVis, nGeomVis), ratio(stats.shapesVis, nTexVis))

    local hist = {}
    for _, b in ipairs({ "0-15", "16-63", "64-255", "256-1023", "1024+" }) do
        hist[#hist + 1] = string.format("%s=%d", b, stats.triHist[b] or 0)
    end
    say("ANATOMY trihist %s", table.concat(hist, " "))

    local objs = {}
    for _, e in ipairs(topN(stats.byObjType, 12)) do
        objs[#objs + 1] = string.format("%s=%d", tostring(e.k), e.v)
    end
    say("ANATOMY objtypes %s", table.concat(objs, " "))

    for _, e in ipairs(topN(stats.byMesh, 20)) do
        say("ANATOMY topmesh %d %s", e.v, tostring(e.k))
    end
    for _, e in ipairs(topN(stats.byTexture, 20)) do
        say("ANATOMY toptex %d %s", e.v, tostring(e.k))
    end
    for _, e in ipairs(topN(stats.byGeomID, 10)) do
        say("ANATOMY topgeom %d id=%s", e.v, tostring(e.k))
    end

    return stats
end

return M
