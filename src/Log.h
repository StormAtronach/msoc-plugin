#pragma once

// Plugin-local logger. API surface mirrors MWSE's Log.h so the patch's
// existing `log::getLog() << ...` call sites carry over with at most a
// namespace-alias line. Output goes to MSOC.log next to Morrowind.exe
// (separate file from MWSE.log to avoid contention).
//
// Implementation lives in src/Log.cpp. The std::ostream& return uses a
// custom std::filebuf with a 64KB internal buffer and a no-op sync(),
// so std::endl does not force a flush; callers can call msoc::log::flush()
// at safe sync points (or rely on the atexit handler at clean exit).

#include <iosfwd>

namespace msoc::log {
	void OpenLog(const char* path);
	void CloseLog();

	std::ostream& getLog();
	std::ostream& getDebug(); // alias for getLog; OutputDebugString routing not implemented

	// Force the deferred buffer to disk. Useful before known-risky
	// operations or for live tail-following. atexit covers clean exit;
	// this is for crash-resilience of specific log lines.
	void flush();

	void prettyDump(const void* data, const size_t length);
	void prettyDump(const void* data, const size_t length, std::ostream& output);

	template<class T> void prettyDump(const T* data) {
		prettyDump(data, sizeof(T));
	};

	template<class T> void prettyDump(const T* data, std::ostream& output) {
		prettyDump(data, sizeof(T), output);
	};
}
