// msoc plugin — luaopen_msoc entry point. Loaded by MWSE's
// `include("msoc")`; whatever this returns becomes the Lua-side `msoc`
// table.

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

struct ProbeResult {
    const char* linkText;
    int         impl;  // MaskedOcclusionCulling::Implementation, or -1
};

// Create + exercise + destroy. Catches AVX2/AVX512 link failures at load
// instead of at first patch use.
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

    // Probe + classify before installPatches() so the threadpool's first
    // createMSOCResources() reads tier-adjusted Configuration:: values
    // rather than the module-init defaults. The Lua side's
    // cfg.syncToNative(msoc) call later overwrites these from msoc.json
    // if present (saved user values are sticky).
    const auto probe = probeMocLink();
    const unsigned hwConcurrency = std::thread::hardware_concurrency();
    const auto tier = msoc::classifyHardwareTier(probe.impl, hwConcurrency);
    msoc::applyHardwareTierDefaults(tier);

    setStringField(L, "version", "1.1.0");
    setStringField(L, "mocLink", probe.linkText);
    setStringField(L, "simdLevel", simdLevelName(probe.impl));
    setStringField(L, "hardwareTier", msoc::hardwareTierName(tier));
    setNumberField(L, "cpuThreads", static_cast<lua_Number>(hwConcurrency));
    setCFunctionField(L, "configure", &msoc::configure);

    // Install the occlusion patches exactly once. MWSE's include() can
    // load the same DLL twice across Lua states; guard with a static.
    //
    // IMPORTANT: assumes MWSE's own MSOC patch is NOT compiled into
    // MWSE.dll. If it is, both patchers collide on the same Morrowind.exe
    // addresses (0x6EB480, 0x41C08E, 0x42E655, 0x4B50FF, 0x6BB7D4).
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
