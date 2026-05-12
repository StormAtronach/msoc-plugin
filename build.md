# Building msoc-plugin from source

## Prerequisites

- Visual Studio 2022 (Community or higher) with the **Desktop C++** workload.
- CMake 3.21+ (the bundled VS one works; Microsoft ships it under
  `Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe`).
- 32-bit Windows build tools — Morrowind is 32-bit and the plugin must
  match.

## Configure + build

The repo ships CMake presets for both configurations:

```pwsh
cmake --preset win32-release
"C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" `
  build\win32-release\msoc-plugin.sln -t:Build -p:Configuration=Release -p:Platform=Win32
```

(`win32-debug` is the debug equivalent.) Output lands at
`test-mod\MWSE\lib\msoc.dll` via the CMake `RUNTIME_OUTPUT_DIRECTORY`
rule — copy it into your Morrowind install at
`Data Files\MWSE\lib\msoc.dll` to test.

## MWSE source pairing

The plugin compiles against MWSE engine headers. **`CMakeLists.txt`
defaults `MWSE_ROOT` to the vendored snapshot at `deps/mwse-upstream/`** —
a pinned, known-good commit that's immune to upstream drift. This is the
safe default and should build cleanly out of the box.

If you point `MWSE_ROOT` at a sibling MWSE checkout (e.g. you're
developing both repos in parallel), it must be on a branch carrying the
SharedSE refactor — the build relies on `SE_*_FNADDR_*` macros and
`NIConfig.Morrowind.h`. MWSE master at the time of writing does **not**
carry these. Common branches that do: `sharedse-ni-unification` and
anything based on it.

If you see errors like:

- `'_new' / '_delete' is not a member of 'se::memory'`
- `SE_MEMORY_FNADDR_*' undeclared identifier`
- `Cannot open include file: 'NIConfig.Morrowind.h'`

… your `MWSE_ROOT` is pointing at a MWSE checkout without the SharedSE
work. Reconfigure with:

```pwsh
cmake --preset win32-release -DMWSE_ROOT="${PWD}\deps\mwse-upstream"
```

The CMake cache remembers `MWSE_ROOT`, so once it's set correctly
subsequent builds don't need the flag.

## At runtime

The runtime msoc.dll is independent of which MWSE.dll the user has
installed — it uses MWSE's stable plugin-loader API only. Build against
the vendored pin, ship the resulting DLL, it will load fine against any
reasonably-current MWSE release.
