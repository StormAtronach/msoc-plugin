return {
    -- ----------------------------------------------------------------
    -- Template + shared sidebar
    -- ----------------------------------------------------------------
    ["template.label"] = "MSOC - Occlusion Culling for MWSE",

    ["sidebar.title"] = "MSOC %{version}",
    ["sidebar.hardware"] = "Hardware tier: %{tier} (%{simd}, %{threads} threads)",
    ["sidebar.hardware.unknown"] = "Hardware tier: unknown (msoc.dll not loaded)",
    ["sidebar.body"] = "Software occlusion culling. Every frame, large opaque meshes are "
        .. "drawn into a small depth mask and everything else is tested against it; "
        .. "what is fully hidden is never sent to the GPU.\n\n"
        .. "Defaults are chosen for your hardware tier. Hover a setting for what it "
        .. "does. Distances and sizes are in world units (1 unit is about 1.4 cm). "
        .. "Changes apply on the next frame unless a setting says otherwise.",
    ["sidebar.link.nexus"]  = "Nexus Mods page",
    ["sidebar.link.github"] = "Source code and issues (GitHub)",

    -- ----------------------------------------------------------------
    -- Page labels
    -- ----------------------------------------------------------------
    ["page.general"]     = "General",
    ["page.occluder"]    = "Occluders",
    ["page.occludee"]    = "Occludees",
    ["page.performance"] = "Performance",
    ["page.debug"]       = "Debug",

    -- ----------------------------------------------------------------
    -- Category labels
    -- ----------------------------------------------------------------
    ["category.culling"]  = "Occlusion culling",
    ["category.terrain"]  = "Terrain",
    ["category.cache"]    = "Verdict cache",
    ["category.interior"] = "Interior cells",
    ["category.exterior"] = "Exterior cells",
    ["category.shared"]   = "All cells",
    ["category.query"]    = "Visibility test",
    ["category.async"]    = "Background rasterisation",
    ["category.overlay"]  = "Mask overlay",
    ["category.tinting"]  = "Tinting",
    ["category.logging"]  = "Logging",
    ["category.watchdog"] = "Freeze forensics",

    -- ----------------------------------------------------------------
    -- Page notes
    -- ----------------------------------------------------------------
    ["performance.info"] = "Your hardware tier is \"%{tier}\". The values below were picked for it "
        .. "and re-applied on each plugin update. Two more tier-set knobs live only in "
        .. "MWSE/config/msoc.json: the mask size (%{width}x%{height}) and the per-phase budgets "
        .. "(rasterize %{rasterBudget} us, classify %{classifyBudget} us; 0 = unlimited).",
    ["debug.info"] = "Everything on this page costs frame time or disk. Leave it off unless "
        .. "you are checking what the culler does or reporting a problem.",

    -- ----------------------------------------------------------------
    -- General page
    -- ----------------------------------------------------------------
    ["EnableMSOC.label"] = "Enable occlusion culling",
    ["EnableMSOC.description"] = "Master switch for the MSOC pass. When off, the scene-graph "
        .. "traversal falls back to the engine's original frustum-only path and "
        .. "rasterisation + queries stop the next frame. The CullShow detour "
        .. "remains installed either way.",

    ["OcclusionEnableInterior.label"] = "Enable in interiors",
    ["OcclusionEnableInterior.description"] = "Interior scenes with lots of walls and doors "
        .. "(Vivec cantons, tombs, Dwemer ruins) are the highest-benefit case for "
        .. "occlusion culling.",

    ["OcclusionEnableExterior.label"] = "Enable in exteriors",
    ["OcclusionEnableExterior.description"] = "Open-world scenes with dense architecture "
        .. "(Balmora, Vivec exterior) benefit most. Sparse wilderness sees little gain "
        .. "and can be disabled here without affecting interiors.",

    ["OcclusionAggregateTerrain.label"] = "Terrain occluder mode",
    ["OcclusionAggregateTerrain.description"] = "Off: no terrain in the occlusion mask "
        .. "(lowest CPU, lowest cull rate). "
        .. "Raster (default): rasterizes the loaded terrain into the mask so hills hide "
        .. "what stands behind them. With Background rasterisation enabled the threadpool "
        .. "does the work off the main thread; with it off, the Terrain occluder resolution "
        .. "below is the cost knob (Corners is about a fifth of Half's time and occludes "
        .. "about as much). The low hardware tier defaults to Raster at Corners.",
    ["OcclusionAggregateTerrain.option.0"] = "Off",
    ["OcclusionAggregateTerrain.option.1"] = "Raster",

    ["OcclusionTerrainResolution.label"] = "Terrain occluder resolution",
    ["OcclusionTerrainResolution.description"] = "How many triangles each terrain subcell "
        .. "contributes to the aggregate occluder. Downsampled vertices take the lowest "
        .. "world-Z across the neighbourhood they replace, so the silhouette can only "
        .. "shrink (safe under-occlude).",
    ["OcclusionTerrainResolution.option.0"] = "Full (5x5, 32 tris/subcell)",
    ["OcclusionTerrainResolution.option.1"] = "Half (3x3, 8 tris/subcell)",
    ["OcclusionTerrainResolution.option.2"] = "Corners (2x2, 2 tris/subcell)",

    ["OcclusionSkipTerrainOccludees.label"] = "Skip terrain occludee queries",
    ["OcclusionSkipTerrainOccludees.description"] = "Landscape patches (25 verts / 32 tris, "
        .. "4x4 per cell) sit under the camera and are visible from nearly every "
        .. "viewpoint. Enabled: they bypass the visibility test and render unconditionally, "
        .. "saving one TestRect call per patch per frame. Disable only to A/B test.",

    ["OcclusionTemporalCoherenceFrames.label"] = "Reuse occluded verdicts (frames)",
    ["OcclusionTemporalCoherenceFrames.description"] = "Frames to keep treating a mesh as "
        .. "hidden after the mask said so, without re-testing it. 0 (default) re-tests "
        .. "every frame, so a mesh reappears the frame it becomes visible. Higher values "
        .. "skip that many tests per mesh but can leave a mesh missing for up to that many "
        .. "frames after the camera moves past its occluder. Entries invalidate when the "
        .. "mesh itself moves, so only static geometry benefits.",

    -- ----------------------------------------------------------------
    -- Occluder page
    -- ----------------------------------------------------------------
    ["OcclusionOccluderRadiusMinInterior.label"] = "Min radius",
    ["OcclusionOccluderRadiusMinInterior.description"] = "Minimum world-bound sphere radius "
        .. "for a mesh to qualify as an occluder in interior cells. Interiors usually want "
        .. "this lower so pillars, crates, and larger furniture contribute to occlusion.",

    ["OcclusionOccluderRadiusMaxInterior.label"] = "Max radius",
    ["OcclusionOccluderRadiusMaxInterior.description"] = "Maximum world-bound sphere radius "
        .. "for a mesh to qualify in interior cells. Rooms are bounded; large cell-hull "
        .. "meshes above this usually cover most of the view and hurt more than they help.",

    ["OcclusionOccluderMinDimensionInterior.label"] = "Min thin-axis dimension",
    ["OcclusionOccluderMinDimensionInterior.description"] = "Reject pencil-shaped meshes in "
        .. "interiors: a mesh is rejected if two or more world-AABB axes are shorter than "
        .. "this. Walls / floors (thin on one axis) still qualify.",

    ["OcclusionInsideOccluderMarginInterior.label"] = "Inside-occluder margin",
    ["OcclusionInsideOccluderMarginInterior.description"] = "Slack added to an occluder's "
        .. "world AABB when testing whether the camera sits inside. If within this margin "
        .. "of the tight AABB, the mesh is skipped for the frame. Interiors may want this "
        .. "tighter because the camera clips architecture more often.",

    ["OcclusionOccluderRadiusMinExterior.label"] = "Min radius",
    ["OcclusionOccluderRadiusMinExterior.description"] = "Minimum world-bound sphere radius "
        .. "for a mesh to qualify as an occluder in exterior cells. Exteriors usually want "
        .. "this higher to skip clutter; only building-scale meshes contribute meaningfully.",

    ["OcclusionOccluderRadiusMaxExterior.label"] = "Max radius",
    ["OcclusionOccluderRadiusMaxExterior.description"] = "Maximum world-bound sphere radius "
        .. "for a mesh to qualify in exterior cells. Meshes above this (terrain patches, "
        .. "skydomes, whole-cell hulls) are skipped.",

    ["OcclusionOccluderMinDimensionExterior.label"] = "Min thin-axis dimension",
    ["OcclusionOccluderMinDimensionExterior.description"] = "Reject pencil-shaped meshes in "
        .. "exteriors: flagpoles, railings, antennae. Walls / floors (thin on one axis) "
        .. "still qualify.",

    ["OcclusionInsideOccluderMarginExterior.label"] = "Inside-occluder margin",
    ["OcclusionInsideOccluderMarginExterior.description"] = "Slack added to an occluder's "
        .. "world AABB when testing whether the camera sits inside, evaluated in exterior "
        .. "cells.",

    ["OcclusionOccluderMaxTriangles.label"] = "Max triangles per occluder",
    ["OcclusionOccluderMaxTriangles.description"] = "Upper bound on triangle count for any "
        .. "occluder, regardless of scene type. Rasterisation cost scales linearly with "
        .. "triangles, so very dense meshes cost more than they pay back in occlusion.",

    ["OcclusionOccluderFrontToBack.label"] = "Submit occluders front to back",
    ["OcclusionOccluderFrontToBack.description"] = "Submit occluders sorted nearest-first so the "
        .. "rasteriser can reject the parts of far occluders already hidden by near ones, cutting "
        .. "raster work in scenes where occluders overlap in depth. The catch: occluders can only "
        .. "be submitted after the whole scene is walked, which on multi-core setups gives up the "
        .. "overlap between walking the scene and rasterising in the background. Measured in two "
        .. "dense cities for 1.6.0 it changed frame time by less than the noise either way, with "
        .. "or without async, so there is no reason to move it unless your own numbers say so.",

    ["OcclusionOccluderCCWOnly.label"] = "Cull back faces (CCW only)",
    ["OcclusionOccluderCCWOnly.description"] = "Rasterise only counter-clockwise (front) "
        .. "occluder faces and cull clockwise ones, roughly halving occluder rasterisation "
        .. "cost. Assumes meshes are CCW-wound, which holds for the vast majority of NIFs; "
        .. "a rare clockwise-wound mesh is simply dropped from the occlusion mask. That can "
        .. "only under-occlude (the object behind it stays visible) and never hides something "
        .. "that should be drawn. Applies to all occluders - cell meshes and aggregated "
        .. "terrain. Worth about 0.4 ms per frame in a dense "
        .. "city when occluders rasterise on the main thread, which is how the low-tier preset "
        .. "runs; with async on, the saving lands on a worker and does not show in frame time. "
        .. "Turn it off only if you suspect a mis-wound mesh is failing to occlude.",

    -- ----------------------------------------------------------------
    -- Occludee page
    -- ----------------------------------------------------------------
    ["OcclusionDepthSlackWorldUnits.label"] = "Depth slack",
    ["OcclusionDepthSlackWorldUnits.description"] = "Extra world-space distance added to a "
        .. "shape's near-surface estimate before TestRect. Biases toward visible; prevents "
        .. "flicker when a mesh sits nearly flush with an occluder. Raise if you see shapes "
        .. "popping behind their own walls.",

    ["OcclusionOccludeeMinRadius.label"] = "Min radius to test",
    ["OcclusionOccludeeMinRadius.description"] = "Shapes below this world-bound sphere radius "
        .. "skip the visibility test entirely. Footprints too small for the hierarchical "
        .. "depth buffer to decide reliably, and the test cost exceeds any cull benefit.",

    ["OcclusionOccludeeBoxTest.label"] = "Tighter box test after the sphere",
    ["OcclusionOccludeeBoxTest.description"] = "After the bounding-sphere test "
        .. "reports an occludee visible, re-test its tighter object-space bounding "
        .. "box. The sphere is conservative for long or flat meshes, so the box can "
        .. "catch occlusions the sphere misses, raising the cull rate. The box is "
        .. "computed once per mesh and cached. Costs an extra projection only on "
        .. "occludees the sphere left visible.",

    -- ----------------------------------------------------------------
    -- Performance page
    -- ----------------------------------------------------------------
    ["OcclusionAsyncOccluders.label"] = "Rasterise occluders on worker threads",
    ["OcclusionAsyncOccluders.description"] = "Submits occluders to Intel's CullingThreadpool "
        .. "for parallel rasterisation on worker threads; main thread continues scene-graph "
        .. "traversal while occluders are drawn. A Flush barrier before the drain guarantees "
        .. "the depth buffer is complete. Enable when rasterizeUs > drainUs in MSOC.log; "
        .. "disable on low core counts.",

    ["OcclusionThreadpoolThreadCount.label"] = "Worker threads (0 = auto)",
    ["OcclusionThreadpoolThreadCount.description"] = "Worker threads used to rasterise "
        .. "occluders (when background rasterisation is on). 0 = auto (min(hardware_concurrency - 2, "
        .. "BinsW*BinsH / 2), floor 1). Manual values must not exceed BinsW*BinsH; the MCM "
        .. "clamps on every change.",

    ["OcclusionThreadpoolBinsW.label"] = "Screen bins across",
    ["OcclusionThreadpoolBinsW.description"] = "The screen is divided into BinsW x BinsH "
        .. "rectangular bins for load balancing across worker threads. Total bins should be "
        .. "at least equal to the worker count.",

    ["OcclusionThreadpoolBinsH.label"] = "Screen bins down",
    ["OcclusionThreadpoolBinsH.description"] = "The screen is divided into BinsW x BinsH "
        .. "rectangular bins for load balancing across worker threads.",

    -- ----------------------------------------------------------------
    -- Debug page
    -- ----------------------------------------------------------------
    ["DebugMaskOverlay.label"] = "Show the occlusion mask",
    ["DebugMaskOverlay.description"] = "Draws the occlusion depth mask itself in the top-right corner of the HUD, the way MGE-XE can show its shadow layers. Brighter means nearer: black is empty mask, and every lit region is geometry that was rasterised as an occluder. Use it to see what the culler is actually working from - combined with the occluder tint below, bright patches in the corner should line up with the tinted meshes in the world. Costs one mask readback per frame while it is on and nothing at all while it is off.",

    ["DebugOcclusionTintOccluder.label"] = "Tint occluders yellow",
    ["DebugOcclusionTintOccluder.description"] = "Overlays a yellow emissive tint on every "
        .. "mesh rasterised as an occluder. Use to check which meshes qualify under the "
        .. "current Occluder settings.",

    ["DebugOcclusionTintOccluded.label"] = "Tint occluded shapes red",
    ["DebugOcclusionTintOccluded.description"] = "Keeps shapes that failed the visibility "
        .. "test visible and tints them red. Verify the culler is rejecting the right meshes.",

    ["DebugOcclusionTintTested.label"] = "Tint visible occludees green",
    ["DebugOcclusionTintTested.description"] = "Tints meshes that passed the visibility test "
        .. "green. Combined with red and yellow, gives a full visual classification of the "
        .. "frame's occlusion decisions.",

    ["OcclusionLogPerFrame.label"] = "Log per-frame culling events",
    ["OcclusionLogPerFrame.description"] = "Writes one MSOC.log line per frame that produced "
        .. "at least one OCCLUDED verdict. Verbose; leave off unless investigating.",

    ["OcclusionLogAggregate.label"] = "Log periodic aggregate stats",
    ["OcclusionLogAggregate.description"] = "Writes one MSOC.log line every 300 frames with "
        .. "cumulative culler counters (rasterised, occluded/tested, view-culled, drain "
        .. "timings). For steady-state profiling.",

    ["OcclusionLogCellCross.label"] = "Log cell-cross spikes",
    ["OcclusionLogCellCross.description"] = "Writes the full stats line on each cell change "
        .. "and the next several frames (tagged cellCross=0..7), so the cross spike's phase "
        .. "breakdown is visible. Watch frameDeltaUs (spike size), cellWipeUs (cache wipe), "
        .. "and the rasterize/classify/aggTerrain timings + cache-miss counts. The 300-frame "
        .. "aggregate sample almost never lands on a cross, so use this to profile crossings.",

    ["OcclusionForensicsWatchdog.label"] = "Freeze-forensics watchdog (restart to apply)",
    ["OcclusionForensicsWatchdog.description"] = "Spawns a background thread that polls the "
        .. "MSOC pipeline's checkpoints every 250 ms and overwrites MSOC.forensics.txt next to "
        .. "MWSE.log. If the game hard-freezes and Windows kills it, the file shows which "
        .. "stage the main thread was stuck in, the recursion depth, and the time since the "
        .. "last clean frame. Diagnostic-only; leave off unless you are reproducing a freeze. "
        .. "The plugin reads this once while starting up, so a change here is saved to "
        .. "msoc.json and takes effect the next time you launch.",
    ["OcclusionForensicsWatchdog.restart"] = "MSOC reads the freeze-forensics setting at startup. "
        .. "The change is saved and takes effect the next time you launch Morrowind.",
}
