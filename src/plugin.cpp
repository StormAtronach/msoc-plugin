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

void setCFunctionField(lua_State* L, const char* key, lua_CFunction fn) {
    lua_pushstring(L, key);
    lua_pushcfunction(L, fn);
    lua_settable(L, -3);
}

// Probe the MOC link by creating an instance, exercising one method,
// destroying it. Catches AVX2/AVX512 specialisation link failures at
// load rather than waiting for the patch to actually use MOC.
const char* probeMocLink() {
    auto* moc = MaskedOcclusionCulling::Create();
    if (!moc) return "Create() returned null";

    moc->SetResolution(64, 32);
    unsigned int w = 0, h = 0;
    moc->GetResolution(w, h);

    auto impl = moc->GetImplementation();
    MaskedOcclusionCulling::Destroy(moc);

    if (w != 64 || h != 32) return "GetResolution mismatch after SetResolution";

    switch (impl) {
        case MaskedOcclusionCulling::SSE2:    return "ok (SSE2)";
        case MaskedOcclusionCulling::SSE41:   return "ok (SSE4.1)";
        case MaskedOcclusionCulling::AVX2:    return "ok (AVX2)";
        case MaskedOcclusionCulling::AVX512:  return "ok (AVX-512)";
        default:                              return "ok (unknown ISA)";
    }
}

} // namespace

extern "C" __declspec(dllexport)
int luaopen_msoc(lua_State* L) {
    lua_newtable(L);

    setStringField(L, "version", "0.0.6-installPatches");
    setStringField(L, "mocLink", probeMocLink());
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
