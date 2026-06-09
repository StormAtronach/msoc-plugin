#include "PatchForensicsWatchdog.h"

#include "Config.h"

#include <atomic>
#include <chrono>
#include <exception>
#include <fstream>
#include <ostream>
#include <thread>

namespace msoc::patch::occlusion::forensics {

const char* stageName(uint32_t s) {
    switch (s) {
        case 0:
            return "idle";
        case 1:
            return "entered";
        case 2:
            return "wakeThreads";
        case 3:
            return "clearBuffer";
        case 4:
            return "cellWipe";
        case 5:
            return "agePrune";
        case 6:
            return "uploadCamera";
        case 7:
            return "setMatrix";
        case 8:
            return "aggTerrain";
        case 9:
            return "cullShowBody";
        case 10:
            return "flush";
        case 11:
            return "drain";
        case 12:
            return "suspendThreads";
        case 13:
            return "exitedClean";
        case 14:
            return "drainClassify";
        case 15:
            return "drainAction";
        default:
            return "unknown";
    }
}

namespace {

// Reserved for an orderly-shutdown path that doesn't currently
// exist (the watchdog thread is detached). Wired so a future
// shutdown hook can flip it and the loop will exit.
std::atomic<bool> g_stop{false};

// 250ms cadence: fast enough to catch a freeze before the user
// alt-tabs out, slow enough not to thrash the disk. File lives
// in CWD (= modlist root under MO2) next to MWSE.log.
void watchdogTick() {
    using namespace std::chrono;
    while (!g_stop.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(milliseconds(250));

        const Snapshot snap = captureSnapshot();

        const uint64_t nowMs = static_cast<uint64_t>(
            duration_cast<milliseconds>(
                steady_clock::now().time_since_epoch())
                .count());
        const uint64_t sinceFrameEndMs = snap.lastFrameEndMs == 0
                                             ? 0
                                             : (nowMs - snap.lastFrameEndMs);

        // Trunc mode: each tick overwrites. On hard freeze,
        // whatever the watchdog last wrote is what's on disk.
        std::ofstream f("MSOC.forensics.txt",
                        std::ios::out | std::ios::trunc);
        if (!f.is_open()) continue;
        f << "frame=" << snap.frame
          << " stage=" << snap.stage << "(" << stageName(snap.stage) << ")"
          << " depth=" << snap.callDepth
          << " maxDepthSess=" << snap.maxCallDepthSession
          << " sinceFrameEndMs=" << sinceFrameEndMs
          << " asyncThisFrame=" << (snap.asyncThisFrame ? 1 : 0)
          << " threadpoolAlive=" << (snap.threadpoolAlive ? 1 : 0)
          << " nowMs=" << nowMs
          << " lastEndMs=" << snap.lastFrameEndMs
          << std::endl;
    }
}

}  // namespace

void spawnIfEnabled(std::ostream& log) {
    // Detached: no orderly shutdown for the occlusion patches, and
    // the thread is read-only on racy globals so a torn read at
    // process exit is harmless.
    if (msoc::Configuration::OcclusionForensicsWatchdog) {
        try {
            std::thread(watchdogTick).detach();
            log << "MSOC: forensics watchdog thread spawned (250ms tick; "
                   "writes MSOC.forensics.txt)."
                << std::endl;
        } catch (const std::exception& e) {
            log << "MSOC: failed to spawn forensics watchdog: "
                << e.what() << std::endl;
        }
    } else {
        log << "MSOC: forensics watchdog disabled "
               "(Configuration::OcclusionForensicsWatchdog=false)."
            << std::endl;
    }
}

}  // namespace msoc::patch::occlusion::forensics
