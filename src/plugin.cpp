// msoc plugin — luaopen_msoc entry point.
//
// Loaded by MWSE's `include("msoc")`, which calls LoadLibrary on
// MWSE/lib/msoc.dll then GetProcAddress("luaopen_msoc"). Whatever this
// function returns becomes the value `include()` returns to Lua.
//
// Returned table:
//   .version    string  — bookkeeping
//   .mocLink    string  — MOC link probe result, "ok (AVX2)" / etc.
//   .configure  cfn     — configure(tbl): push Lua config into native statics
//
// No usertypes, no sol2, no LuaManager shim. Config flows in by value
// when the Lua side calls msoc.configure(cfg.config) — same shape UI
// Expansion uses for its native plugin.

#include <Windows.h>

#include <thread>

#include "Config.h"
#include "MaskedOcclusionCulling.h"
#include "PatchOcclusionCulling.h"

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

// _Claude_ Probe result. linkText is the human-readable link verdict
// shown in MWSE.log; impl is the raw MaskedOcclusionCulling::Implementation
// integer value (or -1 on probe failure) for HardwareTier classification.
struct ProbeResult {
    const char* linkText;
    int         impl;
};

// Probe the MOC link by creating an instance, exercising one method,
// destroying it. Catches AVX2/AVX512 specialisation link failures at
// load rather than waiting for the patch to actually use MOC.
ProbeResult probeMocLink() {
    auto* moc = MaskedOcclusionCulling::Create();
    if (!moc) return { "Create() returned null", -1 };

    moc->SetResolution(64, 32);
    unsigned int w = 0, h = 0;
    moc->GetResolution(w, h);

    auto impl = moc->GetImplementation();
    MaskedOcclusionCulling::Destroy(moc);

    const int implInt = static_cast<int>(impl);
    if (w != 64 || h != 32) {
        return { "GetResolution mismatch after SetResolution", implInt };
    }

    switch (impl) {
        case MaskedOcclusionCulling::SSE2:    return { "ok (SSE2)",    implInt };
        case MaskedOcclusionCulling::SSE41:   return { "ok (SSE4.1)",  implInt };
        case MaskedOcclusionCulling::AVX2:    return { "ok (AVX2)",    implInt };
        case MaskedOcclusionCulling::AVX512:  return { "ok (AVX-512)", implInt };
        default:                              return { "ok (unknown ISA)", implInt };
    }
}

// _Claude_ Map the MOC implementation enum to a stable short string for
// the Lua side. Kept separate from probeMocLink's "ok (XXX)" so the Lua
// MCM can switch on simdLevel cleanly without parsing the link verdict.
const char* simdLevelName(int impl) {
    switch (impl) {
        case MaskedOcclusionCulling::SSE2:    return "SSE2";
        case MaskedOcclusionCulling::SSE41:   return "SSE4.1";
        case MaskedOcclusionCulling::AVX2:    return "AVX2";
        case MaskedOcclusionCulling::AVX512:  return "AVX-512";
        default:                              return "unknown";
    }
}

} // namespace

extern "C" __declspec(dllexport)
int luaopen_msoc(lua_State* L) {
    lua_newtable(L);

    // _Claude_ Probe MOC + classify hardware BEFORE installPatches() so
    // the Configuration::* statics that the threadpool reads at first
    // createMSOCResources() (called from inside installPatches when
    // EnableMSOC starts true) reflect the tier picks rather than the
    // module-init defaults from Config.cpp.
    //
    // Lua's later cfg.syncToNative(msoc) → plugin.configure(table) call
    // can still overwrite these, so the Lua-side default_config in
    // test-mod/.../config.lua reads msoc.hardwareTier and applies the
    // matching overrides for first-run users (no msoc.json yet). Saved
    // user values in msoc.json take precedence over both, which is
    // intentional — explicit user choices are sticky.
    const auto probe = probeMocLink();
    const unsigned hwConcurrency = std::thread::hardware_concurrency();
    const auto tier = msoc::classifyHardwareTier(probe.impl, hwConcurrency);
    msoc::applyHardwareTierDefaults(tier);

    setStringField(L, "version", "0.0.12-single-worker-bypass");
    setStringField(L, "mocLink", probe.linkText);
    setStringField(L, "simdLevel", simdLevelName(probe.impl));
    setStringField(L, "hardwareTier", msoc::hardwareTierName(tier));
    setNumberField(L, "cpuThreads", static_cast<lua_Number>(hwConcurrency));
    setCFunctionField(L, "configure", &msoc::configure);

    // Install the occlusion patches exactly once. MWSE's include() can
    // in theory load the same DLL twice (different Lua states during
    // hot-reload); guard with a function-local static so hooks are only
    // written into the Morrowind.exe code segment on first call.
    //
    // IMPORTANT: this assumes MWSE's own in-tree MSOC patch is NOT
    // compiled into MWSE.dll. If it is, both patchers fight for the
    // same addresses (0x6EB480 CullShow detour, 0x41C08E/0x42E655/
    // 0x4B50FF renderMainScene call sites, 0x6bb7d4 light cull). Use
    // the plugin with a vanilla MWSE.dll during bring-up.
    static bool s_installed = false;
    if (!s_installed) {
        s_installed = true;
        msoc::patch::occlusion::installPatches();
    }

    return 1;
}

BOOL WINAPI DllMain(HINSTANCE, DWORD reason, LPVOID) {
    return TRUE;
}
