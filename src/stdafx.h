#pragma once

// Precompiled header for the plugin, wired via target_precompile_headers
// in CMakeLists.txt. Mirrors the role of stdafx.h in MWSE proper and in
// UI Expansion's plugin: brings in the headers MWSE's engine mirrors
// (MemoryUtil.h, TES3IteratedList.h, NI*.h, TES3*.h) rely on being
// available to every MWSE-header-consuming translation unit.
//
// Kept deliberately narrow — not a clone of MWSE's fat PCH. Only what
// the occlusion patch's transitive MWSE includes actually need to
// parse. Add here when a new MWSE header surfaces an extra dependency,
// not speculatively.
//
// Excluded from the MOC sources via SKIP_PRECOMPILE_HEADERS — Intel's
// code is self-contained and shouldn't pick up windows.h transitively.

// WIN32_LEAN_AND_MEAN / NOMINMAX are set via target_compile_definitions
// in CMakeLists.txt; don't redefine here or we get C4005 warnings.
#include <windows.h>
#include <d3d8.h>       // D3DFORMAT — NIPixelFormat.h

// Undo windows.h macro pollution that clashes with NI:: field names.
#undef near
#undef far
#undef PlaySound

// STL pieces the MWSE engine headers assume are already visible.
#include <cstddef>      // offsetof — TES3IteratedList.h static_asserts
#include <iterator>     // std::reverse_iterator, std::bidirectional_iterator_tag
#include <string>       // std::string toString() / toJson() signatures in
                        //   NIColor.h, TES3Vectors.h, etc.
#include <map>          // std::map — TES3DataHandler.h
#include <unordered_map>// std::unordered_map — TES3 / NI headers
#include <vector>       // std::vector — TES3 / NI headers

// Real sol2 (single-header, served from the MWSE submodule's
// deps/sol/include). Plugin never CALLS into sol, but MWSE engine
// headers take sol::table/sol::object by value in ctor/op= signatures,
// so forward-decls aren't enough — we need the full type.
#include <sol/sol.hpp>

// span-lite. nonstd::span<...> appears in MWSE's NISkinInstance.h,
// NITriShape.h, NITriShapeData.h return types.
#include <nonstd/span.hpp>
