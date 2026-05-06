#pragma once

// Precompiled header (target_precompile_headers in CMakeLists.txt). Brings
// in what MWSE's engine mirrors (MemoryUtil.h, NI*.h, TES3*.h) need to
// parse. Skipped on the MOC sources — Intel's code is self-contained.

// WIN32_LEAN_AND_MEAN / NOMINMAX are set in CMakeLists.txt.
#include <windows.h>
#include <d3d8.h>       // D3DFORMAT — NIPixelFormat.h

// Undo windows.h macro pollution that clashes with NI:: field names.
#undef near
#undef far
#undef PlaySound

#include <cstddef>      // offsetof — TES3IteratedList.h static_asserts
#include <iterator>
#include <string>
#include <map>
#include <unordered_map>
#include <vector>

// MWSE engine headers take sol::table/sol::object by value, so forward
// decls aren't enough.
#include <sol/sol.hpp>

// nonstd::span appears in MWSE's NISkinInstance.h / NITriShape*.h.
#include <nonstd/span.hpp>
