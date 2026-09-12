#pragma once

// Plugin-local logger. Output goes to MSOC.log next to Morrowind.exe
// (separate file from MWSE.log to avoid contention). Call sites read
// `log::getLog() << ...`, matching MWSE's own Log.h.
//
// getLog() opens the file lazily on first use and an atexit handler flushes
// it, so there is nothing to open or close by hand. std::endl does not force
// a flush - the underlying filebuf has a 64KB buffer and a no-op sync() - so
// call flush() at a safe sync point when the process may not exit cleanly.
// A harness that kills the game needs it; msoc.flushLog() exposes it to Lua.
//
// 1.6.0 removed OpenLog, CloseLog, getDebug and prettyDump, none of which had
// callers, and the first two of which did not describe the lifecycle above.

#include <iosfwd>

namespace msoc::log {
std::ostream& getLog();

void flush();
}  // namespace msoc::log
