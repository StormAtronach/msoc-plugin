#pragma once

// Plugin-local logger. Output goes to MSOC.log next to Morrowind.exe
// (separate file from MWSE.log to avoid contention). API mirrors MWSE's
// Log.h so existing `log::getLog() << ...` call sites carry over.
//
// std::endl does not force a flush — the underlying filebuf has a 64KB
// buffer and a no-op sync(). Call msoc::log::flush() at safe sync points
// or rely on atexit for clean exit.

#include <iosfwd>

namespace msoc::log {
	void OpenLog(const char* path);
	void CloseLog();

	std::ostream& getLog();
	std::ostream& getDebug(); // alias for getLog

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
