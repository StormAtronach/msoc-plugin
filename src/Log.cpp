// Plugin-local logger. Writes to <MorrowindRoot>/MSOC.log, truncated
// each launch. Uses a 64KB buffer with a no-op sync() so std::endl
// writes '\n' but does not fsync - flush is explicit (or atexit on
// clean exit). A hard process kill loses up to kBufferSize trailing
// bytes; call flush() before known-risky ops.

#include "Log.h"

#include <cstdlib>
#include <fstream>
#include <mutex>
#include <ostream>
#include <sstream>  // g_fallback, the sink used when the file will not open

#include <Windows.h>

namespace {

constexpr std::size_t kBufferSize = 64 * 1024;

class DeferredFilebuf : public std::filebuf {
public:
    DeferredFilebuf() {
        setbuf(buffer_, sizeof(buffer_));
    }

    int forceSync() {
        return std::filebuf::sync();
    }

protected:
    // Swallow the implicit flush from std::endl / std::flush; overflow()
    // drains the buffer when it fills.
    int sync() override {
        return 0;
    }

private:
    char buffer_[kBufferSize];
};

std::mutex g_initMutex;
bool g_initAttempted = false;
DeferredFilebuf g_filebuf;
std::ostream g_logStream(&g_filebuf);
// Used when the file open fails so getLog never returns a dangling ref.
std::ostringstream g_fallback;

void registerAtExitFlushOnce() {
    static bool registered = false;
    if (registered) return;
    registered = true;
    std::atexit([]() {
        std::lock_guard<std::mutex> lk(g_initMutex);
        if (g_filebuf.is_open()) g_filebuf.forceSync();
    });
}

std::ostream& openOnce() {
    std::lock_guard<std::mutex> lk(g_initMutex);
    if (!g_initAttempted) {
        g_initAttempted = true;
        if (g_filebuf.open("MSOC.log",
                           std::ios::out | std::ios::trunc | std::ios::binary)) {
            registerAtExitFlushOnce();
        } else {
            g_logStream.rdbuf(g_fallback.rdbuf());
        }
    }
    return g_logStream;
}

}  // namespace

namespace msoc::log {

std::ostream& getLog() {
    return openOnce();
}

void flush() {
    std::lock_guard<std::mutex> lk(g_initMutex);
    if (g_filebuf.is_open()) g_filebuf.forceSync();
}

}  // namespace msoc::log

// MWSE's upstream MemoryUtil.cpp (compiled into the plugin) calls
// mwse::log::getLog(). Forward to our msoc::log sink so everything ends
// up in MSOC.log.
namespace mwse::log {
std::ostream& getLog() {
    return ::msoc::log::getLog();
}
}  // namespace mwse::log
