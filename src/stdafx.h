#pragma once

// Precompiled header (target_precompile_headers in CMakeLists.txt). Brings
// in what MWSE's engine mirrors (MemoryUtil.h, NI*.h, TES3*.h) need to
// parse. Skipped on the MOC sources - Intel's code is self-contained.

// WIN32_LEAN_AND_MEAN / NOMINMAX are set in CMakeLists.txt.
#include <windows.h>
#include <d3d8.h>  // D3DFORMAT - NIPixelFormat.h

// Undo windows.h macro pollution that clashes with NI:: field names.
#undef near
#undef far
#undef PlaySound

#include <cstddef>  // offsetof - TES3IteratedList.h static_asserts
#include <iterator>
#include <string>
#include <map>
#include <unordered_map>
#include <vector>
// STL the directly-compiled SharedSE sources expect from the consumer PCH.
// We compile all of SharedSE, so this mirrors MGE-XE's mge_se_prelude.h set:
// <sstream>/<iomanip> for NIObject/NIPoint3 toString; <filesystem> for
// se::windows::getModulePath; the rest for assorted SharedSE utilities.
#include <cassert>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <sstream>
#include <type_traits>
#include <unordered_set>

// MWSE engine headers take sol::table/sol::object by value, so forward
// decls aren't enough.
#include <sol/sol.hpp>

// std::span appears in MWSE's NISkinInstance.h / NITriShape*.h.
#include <span>

// --- SharedSE consumer prelude -----------------------------------------
// SharedSE's unified NI headers require every consumer to declare its
// target application and the engine-allocator addresses up front: the
// per-project NIConfig headers were removed in the unification pass, so
// SharedSE/NIDefines.h now static_asserts unless a target scope is set.
// This is the equivalent of MWSE's stdafx.h and MGE-XE's mge_se_prelude.h;
// because the plugin has a PCH, the config lives here rather than in a
// force-included shim. Values match those projects exactly.

// Target gate: the plugin patches morrowind.exe, so select the MW arm
// (vs SE_TARGETS_CS for the Construction Set).
#define SE_TARGETS_MW 1

// Morrowind.exe engine allocator entry points. se::memory::_new/_delete
// are gated on these; NI::Object::operator new/delete (provided in
// MWSEImports.cpp) routes through them so engine-owned objects the plugin
// allocates - e.g. the tint MaterialProperty - live on the engine heap.
#define SE_MEMORY_FNADDR_NEW 0x727692
#define SE_MEMORY_FNADDR_DELETE 0x727530
#define SE_MEMORY_FNADDR_MALLOC 0x727738
#define SE_MEMORY_FNADDR_FREE 0x727732
#define SE_MEMORY_FNADDR_REALLOC 0x746288

// Neutralise the engine-allocator references inside the NI containers
// (NITArray / NIHashMap / NIIteratedList / StlList). Like MGE-XE, the
// plugin never constructs or destroys engine-owned NI containers - it
// reads the live scene graph and keeps its own std:: structures - so the
// container allocator is irrelevant. (Does not affect NI::Object operator
// new above, which is a separate, ungated declaration.)
#define MWSE_NO_CUSTOM_ALLOC 1
// -----------------------------------------------------------------------
