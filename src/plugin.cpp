// msoc plugin - luaopen_msoc entry point. Loaded by MWSE's
// `include("msoc")`; whatever this returns becomes the Lua-side `msoc`
// table.

#include <Windows.h>

#include <thread>

#include "Config.h"
#include "HardwareTier.h"
#include "Log.h"
#include "MaskOverlay.h"
#include "MaskedOcclusionCulling.h"
#include "OcclusionApi.h"
#include "SimdCap.h"

extern "C" {
#include "lua.h"
#include "lauxlib.h"
}

namespace {

void setStringField(lua_State* L, const char* key, const char* value) {
    lua_pushstring(L, key);
    lua_pushstring(L, value);
    lua_settable(L, -3);
}

void setNumberField(lua_State* L, const char* key, lua_Number value) {
    lua_pushstring(L, key);
    lua_pushnumber(L, value);
    lua_settable(L, -3);
}

void setCFunctionField(lua_State* L, const char* key, lua_CFunction fn) {
    lua_pushstring(L, key);
    lua_pushcfunction(L, fn);
    lua_settable(L, -3);
}

struct ProbeResult {
    const char* linkText;
    int impl;  // MaskedOcclusionCulling::Implementation, or -1
};

// Create + exercise + destroy. Catches AVX2 link failures at load instead of
// at first patch use.
//
// Asking for AVX2 documents intent; it does not gate anything. As of 1.6.0 the
// AVX-512 translation unit is not compiled at all and the dispatch branch is
// preprocessed out (deps/msoc/NOTICE), so Create() has no AVX-512 path to
// return. Re-enabling it means restoring the TU to MOC_SOURCES and setting
// Intel's USE_AVX512, not removing this argument.
ProbeResult probeMocLink() {
    auto* moc = MaskedOcclusionCulling::Create(msoc::simdCap());
    if (!moc) return {"Create() returned null", -1};

    moc->SetResolution(64, 32);
    unsigned int w = 0, h = 0;
    moc->GetResolution(w, h);

    auto impl = moc->GetImplementation();
    MaskedOcclusionCulling::Destroy(moc);

    const int implInt = static_cast<int>(impl);
    if (w != 64 || h != 32) {
        return {"GetResolution mismatch after SetResolution", implInt};
    }

    switch (impl) {
        case MaskedOcclusionCulling::SSE2:
            return {"ok (SSE2)", implInt};
        case MaskedOcclusionCulling::SSE41:
            return {"ok (SSE4.1)", implInt};
        case MaskedOcclusionCulling::AVX2:
            return {"ok (AVX2)", implInt};
        case MaskedOcclusionCulling::AVX512:
            return {"ok (AVX-512)", implInt};
        default:
            return {"ok (unknown ISA)", implInt};
    }
}

// msoc.maskOverlayTexture() -> integer address of the NI::SourceTexture
// mirroring the occlusion mask, or 0 if it is not available yet. Lua turns
// the address back into a usertype with mwse.memory.convertTo.niObject and
// assigns it to a UI image element's texture. Creating the texture on the
// first call is also what arms the per-frame refresh.
int maskOverlayTexture_lua(lua_State* L) {
    void* tex = msoc::occlusion::maskOverlayTexture();
    lua_pushnumber(L, static_cast<lua_Number>(reinterpret_cast<uintptr_t>(tex)));
    return 1;
}

// msoc.dumpMask(path) -> bool. Tone-mapped PFM of the finished mask.
// Replaces the mwse_dumpOcclusionMask export removed in 1.6.0.
int dumpMask_lua(lua_State* L) {
    const char* path = luaL_checkstring(L, 1);
    const bool ok = msoc::occlusion::dumpMaskToPfm(path);
    lua_pushboolean(L, ok ? 1 : 0);
    return 1;
}

// msoc.maskResolution() -> width, height. The size actually latched at
// install, which may differ from msoc.json after rounding and clamping.
int maskResolution_lua(lua_State* L) {
    int w = 0, h = 0;
    msoc::occlusion::maskResolution(&w, &h);
    lua_pushnumber(L, static_cast<lua_Number>(w));
    lua_pushnumber(L, static_cast<lua_Number>(h));
    return 2;
}

// msoc.flushLog() - force MSOC.log to disk. The log uses a 64KB buffer with a
// no-op sync (see Log.cpp), so a run that ends in a kill rather than a clean
// exit loses its tail. Any harness that reads the log after killing the game
// needs this first.
int flushLog_lua(lua_State*) {
    msoc::log::flush();
    return 0;
}
// msoc.logMark(text) - write a marker line into MSOC.log and flush. Lets a
// harness bracket the region of the log that belongs to its sample window,
// so stats lines emitted before the run's config was applied are not counted.
int logMark_lua(lua_State* L) {
    const char* text = luaL_checkstring(L, 1);
    msoc::log::getLog() << "MSOC MARK " << text << std::endl;
    msoc::log::flush();
    return 0;
}
// msoc.install() - install the engine hooks. Separate from luaopen_msoc so
// main.lua can push msoc.json across first: installPatches() latches the
// restart-only knobs (mask resolution, forensics watchdog) as it runs, and
// before 1.6.0 it ran during include(), when Configuration:: still held
// compile-time defaults. Idempotent; MWSE's include() can load the same DLL
// twice across Lua states.
//
// IMPORTANT: assumes MWSE's own MSOC patch is NOT compiled into MWSE.dll. If
// it is, both patchers collide on the same Morrowind.exe addresses
// (0x6EB480, 0x41C08E, 0x42E655, 0x4B50FF).
int install_lua(lua_State*) {
    static bool s_installed = false;
    if (!s_installed) {
        s_installed = true;
        msoc::occlusion::installPatches();
    }
    return 0;
}

const char* simdLevelName(int impl) {
    switch (impl) {
        case MaskedOcclusionCulling::SSE2:
            return "SSE2";
        case MaskedOcclusionCulling::SSE41:
            return "SSE4.1";
        case MaskedOcclusionCulling::AVX2:
            return "AVX2";
        case MaskedOcclusionCulling::AVX512:
            return "AVX-512";
        default:
            return "unknown";
    }
}

}  // namespace

extern "C" __declspec(dllexport) int luaopen_msoc(lua_State* L) {
    lua_newtable(L);

    // Probe and classify only. The tier is reported to Lua, which owns the
    // table of knobs it implies and pushes them back through configure()
    // before calling msoc.install().
    const auto probe = probeMocLink();
    const unsigned hwConcurrency = std::thread::hardware_concurrency();
    const auto tier = msoc::classifyHardwareTier(probe.impl, hwConcurrency);

    setStringField(L, "version", "1.6.0");
    setStringField(L, "mocLink", probe.linkText);
    setStringField(L, "simdLevel", simdLevelName(probe.impl));
    setStringField(L, "hardwareTier", msoc::hardwareTierName(tier));
    setNumberField(L, "cpuThreads", static_cast<lua_Number>(hwConcurrency));
    setCFunctionField(L, "configure", &msoc::configure);
    setCFunctionField(L, "install", &install_lua);
    setCFunctionField(L, "maskOverlayTexture", &maskOverlayTexture_lua);
    setCFunctionField(L, "dumpMask", &dumpMask_lua);
    setCFunctionField(L, "maskResolution", &maskResolution_lua);
    setCFunctionField(L, "flushLog", &flushLog_lua);
    setCFunctionField(L, "logMark", &logMark_lua);

    // No installPatches() here. main.lua calls msoc.install() once it has
    // pushed msoc.json; until then the plugin is loaded but inert. A log
    // reading "plugin loaded" with no later "installing occlusion patches"
    // means main.lua is older than the DLL.
#ifdef NDEBUG
    constexpr const char* kBuildConfig = "release";
#else
    constexpr const char* kBuildConfig = "DEBUG";
#endif
    // The configuration goes in the log because both builds are called
    // msoc.dll and a deployed debug binary is otherwise indistinguishable from
    // a slow release one. Reading 10.87 ms instead of 7.19 and calling it a
    // regression is a real way to lose an hour.
    msoc::log::getLog() << "MSOC: loaded, awaiting msoc.install() from main.lua."
                        << " build=" << kBuildConfig
                        << " simd=" << simdLevelName(probe.impl)
                        << " (" << msoc::simdCapSource() << ")"
                        << " tier=" << msoc::hardwareTierName(tier) << std::endl;

    return 1;
}

BOOL WINAPI DllMain(HINSTANCE, DWORD reason, LPVOID) {
    return TRUE;
}
