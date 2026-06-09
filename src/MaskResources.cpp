// MSOC mask resource lifecycle: allocate / free the live + snapshot MOC
// buffers and the culling threadpool, and reconcile them against the
// EnableMSOC gate. Not hot-path (install / MCM toggle / create-failure).
// Split from PatchOcclusionCulling.cpp; shared state via OcclusionInternal.h.

#include "OcclusionInternal.h"
#include "Config.h"
#include "Log.h"

#include <algorithm>
#include <exception>
#include <ostream>
#include <thread>

namespace msoc::patch::occlusion {

// Allocate g_msoc + g_threadpool. Idempotent. Returns false on
// allocation failure (callers treat as permanent disable). Called
// from installPatches at startup and from the detour's ensure-helper
// on MCM toggle-on.
bool createMSOCResources(std::ostream& log) {
    if (g_msoc && g_threadpool) return true;

    if (!g_msoc) {
        // AVX2 cap: AVX512 is intentionally disabled (known issues). See the
        // probe in plugin.cpp. Revisit to re-enable (Create() defaults to AVX512).
        g_msoc = ::MaskedOcclusionCulling::Create(::MaskedOcclusionCulling::AVX2);
        if (!g_msoc) {
            log << "MSOC: MaskedOcclusionCulling::Create() returned null; occlusion disabled." << std::endl;
            return false;
        }
        g_msoc->SetResolution(kMsocWidth, kMsocHeight);
        g_msoc->SetNearClipPlane(kNearClipW);
    }

    if (!g_msoc_prev) {
        // Snapshot buffer - external consumers read this. Same config
        // as g_msoc so the drain-complete swap preserves rasterizer
        // state. Pre-cleared so the first query (before any frame
        // has completed) sees a defined all-far mask. AVX2 cap (AVX512 off).
        g_msoc_prev = ::MaskedOcclusionCulling::Create(::MaskedOcclusionCulling::AVX2);
        if (!g_msoc_prev) {
            log << "MSOC: snapshot buffer Create() returned null; occlusion disabled." << std::endl;
            ::MaskedOcclusionCulling::Destroy(g_msoc);
            g_msoc = nullptr;
            return false;
        }
        g_msoc_prev->SetResolution(kMsocWidth, kMsocHeight);
        g_msoc_prev->SetNearClipPlane(kNearClipW);
        g_msoc_prev->ClearBuffer();
        g_snapshot.tickMs = 0;
    }

    if (!g_threadpool) {
        const unsigned int binCount = Configuration::OcclusionThreadpoolBinsW * Configuration::OcclusionThreadpoolBinsH;
        // Intel's CullingThreadpool uses hand-rolled spin-semaphores
        // in worker dispatch and livelocks at ~98% CPU when too many
        // workers spin against the same bin queues. A 28-thread CPU
        // freeze was traced to 26 spinning workers saturating the
        // scheduler. 12 is well above hot-path parallelism needs and
        // well under the saturation threshold on many-core CPUs.
        constexpr unsigned int kAutoMax = 12;
        unsigned int threadCount = Configuration::OcclusionThreadpoolThreadCount;
        if (threadCount == 0) {
            // Reserve 2 cores when hw > 4 (main render + slack); 1
            // when hw <= 4. The old plain hw-2 stranded half the CPU
            // on 4-thread parts (i5-2400: hw=4 -> hwBudget=2 wasted
            // binCap=4 and gave only 2 workers).
            // binCap = binCount/2 keeps Intel's work-stealing path
            // enabled (binCount > threadCount) and avoids the 1:1
            // case where the busiest bin sets frame latency.
            const unsigned hw = std::thread::hardware_concurrency();
            const unsigned int hwBudget = (hw <= 4)
                                              ? (hw > 1 ? hw - 1 : 1)
                                              : hw - 2;
            const unsigned int binCap = binCount / 2 > 0 ? binCount / 2 : 1;
            threadCount = std::min({hwBudget, binCap, kAutoMax});
        }
        // Manual overrides above kAutoMax - auto path already caps
        // itself, so this only fires for explicit user values.
        if (threadCount > kAutoMax) {
            log << "MSOC: clamping threadCount from " << threadCount
                << " to kAutoMax=" << kAutoMax
                << " (Intel CullingThreadpool livelocks at high"
                << " worker counts on saturated CPUs)." << std::endl;
            threadCount = kAutoMax;
        }
        // Threads > bins crashes inside RenderTrilist (surplus
        // workers contend for the same bin mutex).
        if (threadCount > binCount) {
            log << "MSOC: clamping threadCount from " << threadCount
                << " to binCount=" << binCount
                << " (threads > bins causes worker crash)." << std::endl;
            threadCount = binCount;
        }
        // One worker is strictly worse than the synchronous path -
        // pays the full WakeThreads/Flush/dispatch tax for zero
        // parallelism gain. g_threadpool stays nullptr; the
        // per-frame gate (g_threadpool && cfgAsync) silently
        // downgrades async to sync.
        if (threadCount <= 1) {
            log << "MSOC: threadCount=" << threadCount
                << " - skipping threadpool allocation. Async path"
                   " would pay dispatch overhead with no parallelism"
                   " gain; using direct serial submission instead."
                   " Set OcclusionThreadpoolThreadCount=2+ to override."
                << std::endl;
            g_threadpool = nullptr;
            return true;
        }

        constexpr unsigned int kMaxJobs = 64;
        // Realistic throw site: threadpool ctor's ~57MB ring-buffer
        // new[]. On any exception we have a half-built world - tear
        // it all down so the reconciler sees disabled-state next frame.
        try {
            g_threadpool = new ::CullingThreadpool(threadCount,
                                                   Configuration::OcclusionThreadpoolBinsW,
                                                   Configuration::OcclusionThreadpoolBinsH,
                                                   kMaxJobs);
            g_threadpool->SetBuffer(g_msoc);
            g_threadpool->SetResolution(kMsocWidth, kMsocHeight);
            g_threadpool->SetNearClipPlane(kNearClipW);
            g_threadpool->SetVertexLayout(::MaskedOcclusionCulling::VertexLayout(12, 4, 8));
            g_threadpool->SuspendThreads();
        } catch (const std::exception& e) {
            log << "MSOC: threadpool creation threw: " << e.what()
                << "; freeing partial state, occlusion disabled this session." << std::endl;
            destroyMSOCResources(log);
            return false;
        } catch (...) {
            log << "MSOC: threadpool creation threw non-std exception; "
                   "freeing partial state, occlusion disabled this session."
                << std::endl;
            destroyMSOCResources(log);
            return false;
        }

        log << "MSOC: threadpool created; threads=" << threadCount
            << " binsW=" << Configuration::OcclusionThreadpoolBinsW
            << " binsH=" << Configuration::OcclusionThreadpoolBinsH
            << " maxJobs=" << kMaxJobs
            << " tp=" << (g_threadpool ? 1 : 0)
            << " cfgAsync=" << (Configuration::OcclusionAsyncOccluders ? 1 : 0)
            << std::endl;
    }
    return true;
}

// Free g_msoc + g_threadpool and release cached state. Caller must
// ensure no async work is in flight (between frames, g_msocActive
// false). Threadpool dtor joins all workers - bounded but blocking,
// up to a few ms.
void destroyMSOCResources(std::ostream& log) {
    if (!g_msoc && !g_threadpool) return;

    if (g_threadpool) {
        delete g_threadpool;
        g_threadpool = nullptr;
    }
    if (g_msoc) {
        ::MaskedOcclusionCulling::Destroy(g_msoc);
        g_msoc = nullptr;
    }
    if (g_msoc_prev) {
        ::MaskedOcclusionCulling::Destroy(g_msoc_prev);
        g_msoc_prev = nullptr;
    }
    g_snapshot.tickMs = 0;
    // Release NI::Pointer refcounts on every cached static, drop the
    // world-vert/index buffers; re-enable rebuilds from scratch.
    g_caches.occluder.clear();
    // External occluder queue: drop pending submissions so re-enable
    // doesn't replay stale ones against the fresh mask.
    clearExternalOccluderQueue();
    g_asyncThisFrame = false;
    g_maskReady = false;

    log << "MSOC: resources freed (threadpool joined, mask buffer destroyed)." << std::endl;
}

// Idempotent reconciler called at the safe top-of-frame point.
// Returns true when resources are live and MSOC can run; false
// when disabled or creation failed.
bool ensureMSOCResourcesMatchConfig() {
    auto& log = log::getLog();
    if (Configuration::EnableMSOC) {
        return createMSOCResources(log);
    } else {
        destroyMSOCResources(log);
        return false;
    }
}

}  // namespace msoc::patch::occlusion
