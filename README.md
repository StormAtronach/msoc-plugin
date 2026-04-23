# msoc-plugin

MWSE native plugin extracting the MSOC (Masked Software Occlusion Culling)
patch from MWSE proper into a standalone DLL loaded via `include("msoc")`.

This is the **stub** — verify the load path works before moving any patch
code over.

## Build

### Visual Studio (Open Folder mode)

`CMakePresets.json` ships two presets: `win32-debug` and `win32-release`.
After opening the folder in VS 2022, pick one from the configuration
dropdown at the top of the IDE and build.

If you opened the folder before the presets existed, VS may have cached
an x64 configuration (which trips the Win32 guard in `CMakeLists.txt`).
Delete the `out/` or `build/` cache via *Project → CMake → Delete Cache
and Reconfigure*, then pick a preset.

### Command line

```bash
cd msoc-plugin
cmake --preset win32-release
cmake --build --preset win32-release
```

Or without presets:

```bash
cmake -B build -A Win32 -S .
cmake --build build --config Release
```

The `-A Win32` is mandatory — Morrowind is 32-bit and an x64 plugin
won't load. The preset enforces this for you.

After a successful build, `msoc.dll` lands in `test-mod/MWSE/lib/msoc.dll`.

If CMake can't find LuaJIT, set `MWSE_ROOT` to your MWSE checkout's inner
source dir (the one containing `deps/LuaJIT/`):

```bash
cmake -B build -A Win32 -S . -DMWSE_ROOT="C:/path/to/MWSE/MWSE"
```

You may need to build MWSE at least once first so `lua51.lib` exists.

## Test load

1. Copy or symlink `test-mod/` into your mod manager's mods directory
   (or directly into `<MorrowindRoot>/Data Files/`).
2. Boot Morrowind.
3. Check `MWSE.log` for a line like:

   ```text
   [msoc] plugin loaded, version=0.0.2-stub-with-moc, mocLink=ok (AVX2)
   ```

   If you see `[msoc] msoc.dll not loaded` instead, the mod manager
   probably stripped the .dll on install — most allow toggling that.

## Project layout

```text
msoc-plugin/
├── CMakeLists.txt              # Win32 build, links MWSE's bundled LuaJIT
├── CMakePresets.json           # win32-debug / win32-release for VS
├── README.md
├── deps/
│   ├── msoc/                   # Intel MaskedOcclusionCulling + threadpool
│   │                           # (with atomify + suspended-Reset fixes)
│   ├── sol/                    # sol2 single-header (LuaJIT bridge)
│   └── mwse/                   # NI/TES3 engine struct headers vendored
│                               # from MWSE checkout, verbatim
├── src/
│   └── plugin.cpp              # luaopen_msoc — extension point
└── test-mod/
    └── MWSE/
        ├── lib/                # msoc.dll lands here after build
        └── mods/
            └── msoc/
                └── main.lua    # include("msoc") + log probe
```

## Roadmap (after the stub loads cleanly)

1. Pull `deps/msoc/` (Intel MOC + the patched threadpool with
   atomify + suspended-Reset fixes) into the project.
2. Pull `MWSE/PatchOcclusionCulling.cpp` over, gate `installPatches()`
   behind a static once-flag, call from `luaopen_msoc`.
3. Replace `mwse::Configuration::Occlusion*` reads with plugin-local
   globals set by a Lua-callable `msoc.configure({...})`.
4. Replace `log::getLog() <<` with own `MSOC.log` writer (avoid races
   with MWSE.log).
5. Add MCM Lua page calling `msoc.configure()` on slider changes.
6. Coordinate with MGE-XE: either expose the same C ABI from msoc.dll
   that MGE-XE used to query in MWSE.dll, or keep a thin shim in MWSE
   that forwards to msoc.dll.
