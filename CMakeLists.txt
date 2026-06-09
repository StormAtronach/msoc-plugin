cmake_minimum_required(VERSION 3.20)
project(msoc-plugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The plugin DLL needs the full MWSE + LuaJIT + Win32 toolchain. The unit
# tests cover only engine-free helpers and need none of that, so they can be
# built and run on any host (e.g. CI without a Morrowind/MWSE checkout).
# Gate the two independently.
option(MSOC_BUILD_DLL   "Build the msoc plugin DLL (needs MWSE + LuaJIT + Win32)" ON)
option(MSOC_BUILD_TESTS "Build the unit-test target (engine-free, no MWSE needed)" ON)

# ---------------------------------------------------------------------------
# Plugin DLL
# ---------------------------------------------------------------------------
if(MSOC_BUILD_DLL)
    # Morrowind is x86 — the plugin must match. Configure with:
    #   cmake -B build -A Win32 -S .
    #   cmake --build build --config Release
    if(NOT CMAKE_GENERATOR_PLATFORM STREQUAL "Win32")
        message(FATAL_ERROR
            "Configure with -A Win32. Morrowind is 32-bit; an x64 DLL won't load. "
            "(To build only the unit tests on another arch, pass -DMSOC_BUILD_DLL=OFF.)")
    endif()

    # Path to upstream MWSE source. Default points at the vendored submodule
    # at deps/mwse-upstream/ — pinned to a known-good MWSE commit, immune to
    # whatever in-progress edits live in a developer's working MWSE checkout.
    # Override via -DMWSE_ROOT=... to test against a different version.
    #
    # The path points at the OUTER MWSE repo root; LuaJIT lives at
    # ${MWSE_ROOT}/deps/luajit2/src and the engine struct headers live at
    # ${MWSE_ROOT}/MWSE/.
    set(MWSE_ROOT "${CMAKE_SOURCE_DIR}/deps/mwse-upstream"
        CACHE PATH "Path to MWSE source root (outer dir, containing deps/ and MWSE/)")

    if(NOT EXISTS "${MWSE_ROOT}/deps/luajit2/src/lua51.lib")
        message(FATAL_ERROR
            "Couldn't find lua51.lib at ${MWSE_ROOT}/deps/luajit2/src/. "
            "luajit2 is a nested submodule that ships source only — build it "
            "the way MWSE does: initialise submodules "
            "(git submodule update --init --recursive), then from a Visual "
            "Studio x86 developer prompt run, in that directory, "
            "`msvcbuild.bat lua52compat` to produce lua51.lib.")
    endif()

    if(NOT EXISTS "${MWSE_ROOT}/MWSE/NIAVObject.h"
        AND NOT EXISTS "${MWSE_ROOT}/SharedSE/NIAVObject.h")
        message(FATAL_ERROR
            "Couldn't find MWSE engine headers at ${MWSE_ROOT}/MWSE/ or "
            "${MWSE_ROOT}/SharedSE/. If using the submodule, run: "
            "git submodule update --init --recursive")
    endif()

    # Intel MOC sources — vendored locally in deps/msoc/. The threadpool
    # carries the atomify + suspended-Reset fixes (volatile→std::atomic,
    # Flush() suspends workers before Reset). Plugin is self-contained:
    # MWSE no longer needs its own copy.
    set(MOC_SOURCES
        "${CMAKE_SOURCE_DIR}/deps/msoc/MaskedOcclusionCulling.cpp"
        "${CMAKE_SOURCE_DIR}/deps/msoc/MaskedOcclusionCullingAVX2.cpp"
        "${CMAKE_SOURCE_DIR}/deps/msoc/MaskedOcclusionCullingAVX512.cpp"
        "${CMAKE_SOURCE_DIR}/deps/msoc/CullingThreadpool.cpp"
    )

    # SharedSE engine sources, compiled in full — the same model MGE-XE uses
    # (it imports SharedSE.vcxitems). SharedSE's NI .cpp cross-reference each
    # other (NIObject -> NIStream/NIRTTI, NIAVObject -> NINode/NITransform/
    # NIMatrix33, NITriBasedGeometryData -> NISkinInstance/NIGeometry/NIBound,
    # ...), so the plugin compiles the whole set rather than hand-rolling
    # forwarders for the handful of methods it calls. They build with the PCH,
    # so stdafx.h's SharedSE prelude (SE_TARGETS_MW + the SE_*_FNADDR address
    # sets; SE_USE_LUA / SE_IS_MWSE left off) drives the engine-address thunks
    # and gates out the Lua / MWSE-private surface. MemoryUtil.cpp is in here too.
    file(GLOB SHAREDSE_SOURCES CONFIGURE_DEPENDS "${MWSE_ROOT}/SharedSE/*.cpp")

    add_library(msoc SHARED
        src/plugin.cpp
        src/Log.cpp
        src/Config.cpp
        # 1D-horizon → curtain occluder used by the Horizon mode of
        # rasterizeAggregateTerrain. Adapted from MGE-XE's distant-land
        # equivalent; see src/HorizonOccluder.h for the rationale.
        src/HorizonOccluder.cpp
        # Debug-only occlusion tint overlay (split out of the patch TU).
        src/DebugTint.cpp
        # Detached watchdog thread that dumps MSOC.forensics.txt every
        # ~250 ms while the gate is on. Reads hot-path checkpoints via
        # forensics::captureSnapshot() defined in PatchOcclusionCulling.cpp.
        src/PatchForensicsWatchdog.cpp
        src/PatchOcclusionCulling.cpp
        # Out-of-tree mask query API (testOcclusion*, snapshot accessors).
        # Reads the shared snapshot state via OcclusionInternal.h.
        src/QueryApi.cpp
        # CPU light culling (updateLights enabled-read hook + observers).
        src/LightCulling.cpp
        # Terrain occluder aggregation (Raster + Horizon modes).
        src/TerrainAggregation.cpp
        # MSOC mask buffer + threadpool lifecycle (create/destroy/reconcile).
        src/MaskResources.cpp
        # C-ABI exports (mwse_* thunks) for out-of-tree consumers; split out
        # of the patch TU. Forward to the msoc::patch::occlusion public API.
        src/Exports.cpp
        # Local forwarders for the NI/TES3 engine methods the patch calls.
        # UIE-style: each body is a one-liner reinterpret_cast to a fixed
        # Morrowind.exe address. Pure engine thunks, no MWSE runtime dep.
        src/MWSEImports.cpp
        ${SHAREDSE_SOURCES}
        ${MOC_SOURCES}
    )

    # Precompiled header. Brings windows.h + STL + the sol forward-decl shim
    # into every TU that needs MWSE engine headers to parse. Matches the PCH
    # model MWSE proper and UI Expansion's plugin use.
    target_precompile_headers(msoc PRIVATE src/stdafx.h)

    # Intel MOC is self-contained third-party code — skip the PCH on its
    # sources so windows.h and friends don't bleed into them.
    set_source_files_properties(${MOC_SOURCES}
        PROPERTIES SKIP_PRECOMPILE_HEADERS TRUE)

    # Per-file ISA flags for the SIMD specialisations. Intel ships separate
    # .cpp files per ISA so only that file is compiled with the elevated
    # instruction set; the dispatcher in MaskedOcclusionCulling.cpp picks
    # whichever the host CPU supports at runtime.
    set_source_files_properties(
        "${CMAKE_SOURCE_DIR}/deps/msoc/MaskedOcclusionCullingAVX2.cpp"
        PROPERTIES COMPILE_OPTIONS "/arch:AVX2"
    )
    set_source_files_properties(
        "${CMAKE_SOURCE_DIR}/deps/msoc/MaskedOcclusionCullingAVX512.cpp"
        PROPERTIES COMPILE_OPTIONS "/arch:AVX512"
    )

    # Include paths:
    #   1. mwse-upstream/MWSE/    - engine struct mirrors (NI*.h, TES3*.h),
    #                              served verbatim from upstream MWSE.
    #   2. msoc                   - vendored Intel MOC.
    #   3. mwse-upstream/deps/sol - real sol2 single-header. The plugin never
    #                              calls sol, but MWSE headers take sol::table
    #                              and sol::object by value in signatures, so
    #                              forward-decls aren't enough.
    #   4. LuaJIT src             - C API headers.
    # No deps/mwse/ overrides anymore: Config.h replaces MWSEConfig.h, no
    # LuaManager shim because the plugin doesn't bind a sol usertype.
    target_include_directories(msoc PRIVATE
        "${MWSE_ROOT}/MWSE"
        # SharedSE/ holds the unified NI*.h engine headers + MemoryUtil after
        # the SharedSE split. TES3*.h still live under MWSE/.
        "${MWSE_ROOT}/SharedSE"
        "${CMAKE_SOURCE_DIR}/deps/msoc"
        "${MWSE_ROOT}/deps/sol/include"
        "${MWSE_ROOT}/deps/DirectX8/include"
        "${MWSE_ROOT}/deps/luajit2/src"
    )

    target_link_libraries(msoc PRIVATE
        "${MWSE_ROOT}/deps/luajit2/src/lua51.lib"
    )

    target_compile_definitions(msoc PRIVATE
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
        WIN32_LEAN_AND_MEAN
        # Mirrors MWSE's MSVC iterator deprecation suppression — sol2 + the
        # vendored MWSE headers trigger this otherwise.
        _SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING
    )

    # Match MWSE's runtime so we can share heap allocations later (CRT mismatch
    # would crash on cross-module new/delete once we move the patch over).
    set_property(TARGET msoc PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

    # Link-time optimization for Release. Maps to MSVC /GL (compile) + /LTCG
    # (link). Lets the back-end optimizer see across translation units —
    # inlines our drain loop into Intel MOC's TestRect, DCEs the unused
    # AVX2/AVX512 template instantiations, etc. Adds ~30-60s to the link
    # step; expected ~5-10% drop on drainUs and a smaller .dll.
    set_property(TARGET msoc PROPERTY
        INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)

    # Drop the built DLL straight into the test mod's lib dir so include("msoc")
    # can find it without manual copying.
    set_target_properties(msoc PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY         "${CMAKE_SOURCE_DIR}/test-mod/MWSE/lib"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_SOURCE_DIR}/test-mod/MWSE/lib"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/test-mod/MWSE/lib"
        OUTPUT_NAME "msoc"
    )
endif()

# ---------------------------------------------------------------------------
# Unit tests (doctest)
# ---------------------------------------------------------------------------
# These compile a small set of engine-free source files directly into a test
# executable — no PCH, no MWSE engine headers, no LuaJIT. As the monolith is
# decomposed (see moreFPS docs plan) and more pure helpers are isolated, add
# their .cpp here and a matching tests/*_tests.cpp.
if(MSOC_BUILD_TESTS)
    enable_testing()

    add_executable(msoc_tests
        tests/test_main.cpp
        tests/horizon_occluder_tests.cpp
        tests/clip_math_tests.cpp  # ClipMath.h is header-only — no .cpp to compile
        tests/profiling_tests.cpp  # Profiling.h is header-only too
        # Unit under test — engine-free, already PCH-free.
        src/HorizonOccluder.cpp
    )

    target_include_directories(msoc_tests PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/doctest"
    )

    target_compile_definitions(msoc_tests PRIVATE
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
    )

    # Keep the CRT consistent with the DLL so a future shared build matches.
    set_property(TARGET msoc_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

    add_test(NAME msoc_tests COMMAND msoc_tests)
endif()
