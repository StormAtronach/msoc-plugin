// Plugin-local logger. Mirrors the API surface of MWSE's Log.h (only the
// pieces PatchOcclusionCulling.cpp uses) but writes to its own file so
// there's no contention with MWSE.log. The patch's existing
// `mwse::log::getLog() << ...` call sites carry over without edits.
//
// File: <MorrowindRoot>/MSOC.log, truncated each launch (matches
// MSOC.forensics.txt and MSOC.flushdump.txt convention).
//
// Buffering:
//   The patch logs heavily, including per-frame stat lines. With a
//   default ofstream, every `<< std::endl` would synchronously flush
//   to disk - one fsync per frame. We use a custom std::filebuf that
//   (a) maintains a large internal buffer (kBufferSize), and
//   (b) overrides sync() to be a no-op, so std::endl writes the '\n'
//       but does not flush.
//   The buffer flushes naturally when full (filebuf::overflow), and
//   explicitly via mwse::log::flush() / atexit / CloseLog.
//
//   Tradeoff: a hard process kill (not a clean exit) can lose up to
//   kBufferSize of trailing log data. Diagnostic dumps that need to
//   be on disk before a known-risky op should call flush() first.

#include "Log.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <ostream>
#include <sstream>

#include <Windows.h>

namespace {

constexpr std::size_t kBufferSize = 64 * 1024;

// std::filebuf with a large internal buffer and a no-op sync(). std::endl
// triggers sync() in the standard pipeline; making it a no-op is what
// turns std::endl from "flush every line" into "just write \n".
class DeferredFilebuf : public std::filebuf {
public:
    DeferredFilebuf() {
        // Hand the filebuf a writable buffer it'll use for batching.
        // Must outlive any open() call - hence storing as a member.
        setbuf(buffer_, sizeof(buffer_));
    }

    // Force a real flush. Bypasses the sync() override.
    int forceSync() {
        return std::filebuf::sync();
    }

protected:
    int sync() override {
        // Swallow the implicit flush from std::endl / std::flush.
        // Buffer fills naturally; overflow() drains it when full.
        return 0;
    }

private:
    char buffer_[kBufferSize];
};

// Single sink used by getLog/getDebug. Lazy init under g_initMutex so
// the first call from any thread sets it up exactly once.
std::mutex g_initMutex;
bool g_initAttempted = false;
DeferredFilebuf g_filebuf;
std::ostream g_logStream(&g_filebuf);
// Fallback used when the file open failed (read-only Morrowind dir,
// etc.) so getLog never returns a dangling reference.
std::ostringstream g_fallback;

void registerAtExitFlushOnce() {
    static bool registered = false;
    if (registered) return;
    registered = true;
    // Best-effort flush at normal process exit. Won't fire on crash;
    // for those, callers should call mwse::log::flush() at known-risky
    // points (forensics watchdog already does this for its own files).
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

} // namespace

namespace msoc::log {

void OpenLog(const char* /*path*/) {
    // Path ignored; we always write to MSOC.log next to Morrowind.exe.
    (void)openOnce();
}

void CloseLog() {
    std::lock_guard<std::mutex> lk(g_initMutex);
    if (g_filebuf.is_open()) {
        g_filebuf.forceSync();
        g_filebuf.close();
    }
}

std::ostream& getLog() {
    return openOnce();
}

std::ostream& getDebug() {
    // OutputDebugString routing is not worth the per-line cost;
    // route to the same sink as getLog so calls don't disappear.
    return openOnce();
}

void flush() {
    std::lock_guard<std::mutex> lk(g_initMutex);
    if (g_filebuf.is_open()) g_filebuf.forceSync();
}

void prettyDump(const void* data, const size_t length) {
    prettyDump(data, length, openOnce());
}

void prettyDump(const void* data, const size_t length, std::ostream& output) {
    // Compact hex dump: 16 bytes per line, offset prefix, ASCII gutter.
    // Doesn't try to mirror MWSE's prettyDump byte-for-byte; the patch
    // only uses this for forensic dumps where exact format parity isn't
    // needed.
    const auto* bytes = static_cast<const unsigned char*>(data);
    const auto oldFlags = output.flags();
    const auto oldFill = output.fill();
    output << std::hex << std::setfill('0');
    for (size_t i = 0; i < length; i += 16) {
        output << std::setw(8) << i << "  ";
        size_t row = std::min<size_t>(16, length - i);
        for (size_t j = 0; j < row; ++j) {
            output << std::setw(2) << static_cast<unsigned int>(bytes[i + j]) << ' ';
        }
        for (size_t j = row; j < 16; ++j) output << "   ";
        output << " ";
        for (size_t j = 0; j < row; ++j) {
            unsigned char b = bytes[i + j];
            output << static_cast<char>((b >= 32 && b < 127) ? b : '.');
        }
        output << '\n';
    }
    output.flags(oldFlags);
    output.fill(oldFill);
}

} // namespace msoc::log

// MWSE's upstream MemoryUtil.cpp (compiled directly into the plugin)
// calls `log::getLog()` from inside namespace mwse, which resolves to
// mwse::log::getLog(). MWSE's Log.h declares that symbol; MWSE's Log.cpp
// defines it — but that file depends on more MWSE internals than we
// want to pull in. Provide the subset MemoryUtil.cpp actually references
// (only getLog) as forwarders to our msoc::log sink, so everything
// MWSE proper would have written to MWSE.log ends up in MSOC.log here.
namespace mwse::log {
    std::ostream& getLog() { return ::msoc::log::getLog(); }
}
