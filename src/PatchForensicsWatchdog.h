#pragma once

// Freeze-forensics watchdog. A detached background thread polls the MSOC
// pipeline's checkpoints every ~250ms and overwrites MSOC.forensics.txt
// with the freshest snapshot. When the game hard-freezes and Windows
// kills it, the file shows which stage the main thread was stuck in,
// recursion depth at freeze, and time since the last clean frame end.
//
// Decouples watchdog logic from PatchOcclusionCulling.cpp. The hot path
// keeps its globals internal-linkage; the only crossing point is
// captureSnapshot(), defined in PatchOcclusionCulling.cpp.

#include <cstdint>
#include <iosfwd>

namespace msoc::patch::occlusion::forensics {

	// Reads tear under hard freeze, but torn 32/64-bit values still
	// bracket the truth well enough for post-mortem.
	struct Snapshot {
		uint32_t stage;                // g_lastStage (0..15, see stageName)
		uint32_t callDepth;            // current cullShowBody recursion depth
		uint32_t maxCallDepthSession;
		uint64_t frame;
		uint64_t lastFrameEndMs;       // steady_clock ms at last clean frame exit
		bool     asyncThisFrame;
		bool     threadpoolAlive;      // g_threadpool != nullptr
	};

	// Implemented in PatchOcclusionCulling.cpp where the read targets live.
	Snapshot captureSnapshot();

	// 0..15 → human-readable label. Kept in sync with the stage-numbering
	// comment above g_lastStage in PatchOcclusionCulling.cpp.
	const char* stageName(uint32_t stage);

	// Spawn the detached watchdog thread when
	// Configuration::OcclusionForensicsWatchdog is true. No-op otherwise.
	// Restart-only — installPatches runs from luaopen_msoc before Lua's
	// first msoc.configure() call, so the gate reflects C++/tier defaults
	// at startup, not the user's most recent MCM edit.
	void spawnIfEnabled(std::ostream& log);

}
