// Per-frame stats logging: emits the MSOC diagnostic line on the
// Aggregate / PerFrame / CellCross channels. Reads only the shared owners
// (g_stats / g_caches / g_frame / g_budget / g_diag) via OcclusionInternal.h;
// split from the CullShow detour. Cold path - runs only when a log channel
// is enabled.

#include "OcclusionInternal.h"
#include "Config.h"
#include "Log.h"

#include <atomic>
#include <ostream>

namespace msoc::patch::occlusion {

void emitPerFrameStatsLine() {
    // Cumulative counters survive across frames so rare OCCLUDED
    // events - scenes where one building sits squarely behind
    // another - show up even when the 300-frame sampling misses
    // them. Logged alongside per-frame counts so we can spot when
    // the ratio g_diag.totalOccluded/g_diag.totalTested is nonzero.
    g_diag.totalTested += g_stats.queryTested;
    g_diag.totalOccluded += g_stats.queryOccluded;
    g_diag.totalViewCulled += g_stats.queryViewCulled;

    // Two independently-toggled log channels:
    //   - OcclusionLogAggregate: periodic 300-frame sample. Steady
    //     baseline of cumulative counters; useful for "is the
    //     culler doing anything" at a glance.
    //   - OcclusionLogPerFrame: any frame that produced an
    //     OCCLUDED verdict. Reconciliation channel for "I see
    //     culling but the counters say 0" - every culling event
    //     gets a line.
    // Both default off. Identical line format on both channels.
    const bool baselineTick = Configuration::OcclusionLogAggregate && (g_frameCounter % 300) == 0;
    const bool hadOccluded = Configuration::OcclusionLogPerFrame && g_stats.queryOccluded > 0;
    // Cell-cross channel: fire on the cross frame + the re-population
    // frames. cellCross=<age> (0 = cross frame). Same line format.
    const bool cellCrossTick = Configuration::OcclusionLogCellCross && g_diag.cellCrossLogFrames > 0;
    if (baselineTick || hadOccluded || cellCrossTick) {
        log::getLog() << "MSOC: frame " << g_frameCounter
                      << " cellCross=" << (cellCrossTick ? (8 - g_diag.cellCrossLogFrames) : -1)
                      << " cellWipeUs=" << g_diag.cellWipeUs
                      << " frameDeltaUs=" << g_diag.lastFrameDeltaUs
                      << " rasterized=" << g_stats.rasterizedAsOccluder
                      << " occluderTris=" << g_stats.occluderTriangles
                      << " queryOccluded=" << g_stats.queryOccluded
                      << "/" << g_stats.queryTested
                      << " boxOccluded=" << g_stats.boxOccluded
                      << " boxCacheHit=" << g_caches.occludeeBoxHits
                      << " boxCacheMiss=" << g_caches.occludeeBoxMisses
                      << " viewCulled=" << g_stats.queryViewCulled
                      << " nearClip=" << g_stats.queryNearClip.load(std::memory_order_relaxed)
                      << " deferred=" << g_stats.deferred
                      << " inlineTested=" << g_stats.inlineTested
                      << " recursive=" << g_stats.recursiveCalls
                      << " appCulled=" << g_stats.recursiveAppCulled
                      << " frustumCulled=" << g_stats.recursiveFrustumCulled
                      << " insideSkipped=" << g_stats.skippedInside
                      << " thinSkipped=" << g_stats.skippedThin
                      << " alphaSkipped=" << g_stats.skippedAlpha
                      << " stencilSkipped=" << g_stats.skippedStencil
                      << " triSkipped=" << g_stats.skippedTriCount
                      << " tinySkipped=" << g_stats.skippedTesteeTiny
                      << " terrainSkipped=" << g_stats.skippedTerrain
                      << " aggTerrainLands=" << g_stats.aggregateTerrainLands
                      << " aggTerrainTris=" << g_stats.aggregateTerrainTris
                      << " aggTerrainUs=" << g_stats.aggregateTerrainUs
                      // Horizon-mode counters; zero unless g_frame.aggregateTerrain == 2.
                      << " horizonBuildUs=" << g_stats.horizonBuildUs
                      << " horizonRasterUs=" << g_stats.horizonRasterUs
                      << " horizonLandsFed=" << g_stats.horizonLandsFed
                      << " horizonVertsFed=" << g_stats.horizonVertsFed
                      << " horizonColumnsTouched=" << g_stats.horizonColumnsTouched
                      << " horizonCurtainTris=" << g_stats.horizonCurtainTris
                      << " horizonAdaptiveEpsD=" << g_stats.horizonAdaptiveEpsD
                      << " landMembershipHit=" << g_caches.terrainMembershipHits
                      << " landMembershipMiss=" << g_caches.terrainMembershipMisses
                      << " landMembershipSize=" << g_caches.terrainMembership.size()
                      << " classOccCalls=" << g_stats.classifyOccluderCalls
                      << " classOccSteps=" << g_stats.classifyOccluderSteps
                      << " occVertCalls=" << g_stats.occluderVertexCalls
                      << " occVertVerts=" << g_stats.occluderVertexVerts
                      << " occCacheHit=" << g_caches.occluderHits
                      << " occCacheMiss=" << g_caches.occluderMisses
                      << " occCacheSize=" << g_caches.occluder.size()
                      << " landCacheHit=" << g_caches.landHits
                      << " landCacheMiss=" << g_caches.landMisses
                      << " landCacheEvict=" << g_caches.landEvictions
                      // Light-cull A/B counters. lightCullMiss == lightsTested
                      // (every miss runs the MSOC test); kept separate for
                      // symmetry with other *Hit/Miss pairs.
                      << " lightCullHit=" << g_caches.lightCullHits
                      << " lightCullMiss=" << g_caches.lightCullMisses
                      << " lightOccluded=" << g_caches.lightsOccluded
                      << " lightCacheSize=" << g_caches.lightCull.size()
                      // Average per-frame time across this sample window.
                      // 1e6 / avgFrameUs = average FPS. Reset right after.
                      << " avgFrameUs=" << (g_diag.windowFrameCount ? (g_diag.windowFrameTimeUs / g_diag.windowFrameCount) : 0)
                      << " framesInWin=" << g_diag.windowFrameCount
                      << " tcHit=" << g_caches.drainHits
                      << " tcMiss=" << g_caches.drainMisses
                      << " tcSize=" << g_caches.drain.size()
                      << " cellChanges=" << g_diag.cellChanges
                      << " sceneGateSkipped=" << g_stats.skippedSceneGate
                      << " menuModeSkipped=" << g_stats.skippedMenuMode  // cumulative
                      << " rasterizeUs=" << g_stats.rasterizeTimeUs
                      << " occXformUs=" << g_stats.occluderTransformUs
                      << " drainUs=" << g_stats.drainPhaseTimeUs
                      << " classifyUs=" << g_stats.classifyUs     // phase-1 wall (serial = work; parallel = barrier)
                      << " displayUs=" << g_stats.drainDisplayUs  // audit
                      << " asyncFlushUs=" << g_diag.asyncFlushTimeUs
                      // Hybrid budget diagnostics. *Trip is per-frame (0/1),
                      // *TripsSess is lifetime sum, *Ema is the predictive-skip metric
                      // (compared against 2x *BudgetUs). *BudgetUs=0 means the gate
                      // is disabled for that phase.
                      //
                      // Names changed in 0.0.10: was drain*BudgetUs/Ema/Trip; renamed
                      // to class*BudgetUs/Ema/Trip because the phase being budgeted
                      // is classifyDrainRange (TestRect work), not the whole drain
                      // (whose phase 2 display() calls are vanilla render work).
                      << " rastBudgetUs=" << g_budget.rasterizeBudgetUsEffective
                      << " rastEmaUs=" << g_budget.rasterizeEmaUs
                      << " rastTrip=" << g_budget.rasterizeBudgetTrips
                      << " rastTripsSess=" << g_budget.rasterizeBudgetTripsSession
                      << " classBudgetUs=" << g_budget.classifyBudgetUsEffective
                      << " classEmaUs=" << g_budget.classifyEmaUs
                      << " classTrip=" << g_budget.classifyBudgetTrips
                      << " classTripsSess=" << g_budget.classifyBudgetTripsSession
                      << " wakeUs=" << g_diag.wakeThreadsTimeUs                            // this frame's WakeThreads spin
                      << " maxWakeUsSess=" << g_diag.maxWakeThreadsUsSession               // lifetime peak
                      << " maxDepthFrame=" << g_diag.maxCallDepthThisFrame                 // this frame's recursion peak
                      << " maxDepthSess=" << g_diag.maxCallDepthSession                    // lifetime peak
                      << " tp=" << (g_threadpool ? 1 : 0)                                  // threadpool liveness
                      << " cfgAsync=" << (Configuration::OcclusionAsyncOccluders ? 1 : 0)  // async fires iff tp && cfgAsync
                      << " topLvlThisScene=" << g_diag.isTopLevelFiresThisScene
                      << " maxTopLvlSess=" << g_diag.maxIsTopLevelFiresSession
                      << " mainAttemptsThisScene=" << g_diag.mainCamCullShowAttemptsThisScene
                      << " maxMainAttemptsSess=" << g_diag.maxMainCamCullShowAttemptsSession
                      << " cumul=" << g_diag.totalOccluded << "/" << g_diag.totalTested
                      << "(vc=" << g_diag.totalViewCulled << ")" << std::endl;
        // Reset the per-window frame-time accumulator so the next
        // log line reflects its own window only.
        g_diag.windowFrameTimeUs = 0;
        g_diag.windowFrameCount = 0;
    }

    // Count down the cell-cross profiling budget (set to 8 on the
    // cross); runs every frame so cellCross ages 0..7 then stops.
    if (g_diag.cellCrossLogFrames > 0) {
        --g_diag.cellCrossLogFrames;
    }
}

}  // namespace msoc::patch::occlusion
