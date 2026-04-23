cmake_minimum_required(VERSION 3.20)
project(msoc-plugin LANGUAGES CXX)

# Morrowind is x86 — the plugin must match. Configure with:
#   cmake -B build -A Win32 -S .
#   cmake --build build --config Release
if(NOT CMAKE_GENERATOR_PLATFORM STREQUAL "Win32")
    message(FATAL_ERROR
        "Configure with -A Win32. Morrowind is 32-bit; an x64 DLL won't load.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Path to upstream MWSE source. Default points at the vendored submodule
# at deps/mwse-upstream/ — pinned to a known-good MWSE commit, immune to
# whatever in-progress edits live in a developer's working MWSE checkout.
# Override via -DMWSE_ROOT=... to test against a different version.
#
# The path points at the OUTER MWSE repo root; LuaJIT lives at
# ${MWSE_ROOT}/deps/LuaJIT/src and the engine struct headers live at
# ${MWSE_ROOT}/MWSE/.
set(MWSE_ROOT "${CMAKE_SOURCE_DIR}/deps/mwse-upstream"
    CACHE PATH "Path to MWSE source root (outer dir, containing deps/ and MWSE/)")

if(NOT EXISTS "${MWSE_ROOT}/deps/LuaJIT/src/lua51.lib")
    message(FATAL_ERROR
        "Couldn't find lua51.lib at ${MWSE_ROOT}/deps/LuaJIT/src/. "
        "Run msvcbuild.bat from a VS x86 dev prompt in that directory, "
        "or copy the prebuilt lua51.lib from a working MWSE checkout.")
endif()

if(NOT EXISTS "${MWSE_ROOT}/MWSE/NIAVObject.h")
    message(FATAL_ERROR
        "Couldn't find MWSE engine headers at ${MWSE_ROOT}/MWSE/. "
        "If using the submodule, run: git submodule update --init --recursive")
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

add_library(msoc SHARED
    src/plugin.cpp
    src/Log.cpp
    src/Config.cpp
    src/PatchOcclusionCulling.cpp
    # Local forwarders for the NI/TES3 engine methods the patch calls.
    # UIE-style: each body is a one-liner reinterpret_cast to a fixed
    # Morrowind.exe address. Pure engine thunks, no MWSE runtime dep.
    src/MWSEImports.cpp
    # Compile MWSE's own MemoryUtil.cpp directly rather than rewriting
    # the gen*Unprotected helpers. Only adaptation: a tiny mwse::log
    # forwarder in src/Log.cpp points MWSE's log::getLog() calls at
    # our msoc::log sink.
    "${MWSE_ROOT}/MWSE/MemoryUtil.cpp"
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
    "${CMAKE_SOURCE_DIR}/deps/msoc"
    "${MWSE_ROOT}/deps/sol/include"
    "${MWSE_ROOT}/deps/span-lite"
    "${MWSE_ROOT}/deps/DirectX8/include"
    "${MWSE_ROOT}/deps/LuaJIT/src"
)

target_link_libraries(msoc PRIVATE
    "${MWSE_ROOT}/deps/LuaJIT/src/lua51.lib"
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

# Drop the built DLL straight into the test mod's lib dir so include("msoc")
# can find it without manual copying.
set_target_properties(msoc PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY         "${CMAKE_SOURCE_DIR}/test-mod/MWSE/lib"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_SOURCE_DIR}/test-mod/MWSE/lib"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/test-mod/MWSE/lib"
    OUTPUT_NAME "msoc"
)
