#pragma once

// Testing lever: cap the rasterizer's instruction set from the environment.
//
// The hardware tier the plugin picks turns on whether the CPU has AVX2, and
// the low tier exists for machines that do not: SSE4.1 is 4-wide against AVX2's
// 8, so roughly half the per-tile rasterizer throughput. That is the single
// most important difference between the tiers, and it cannot be reproduced on a
// modern CPU by any amount of core pinning or clock throttling, because those
// slow everything down uniformly and leave the SIMD width alone.
//
// MSOC_SIMD_CAP=sse41 makes this machine look like that one to the parts of the
// plugin that care: probeMocLink reports SSE4.1, classifyHardwareTier therefore
// returns Low, and Lua applies the low-tier defaults on its own.
//
// It reads an environment variable rather than msoc.json because the probe runs
// inside luaopen_msoc, before main.lua can push any config across. It is also
// the honest home for it. This is a lever for measuring what other people's
// hardware does, not a setting anyone should ship with.

#include "MaskedOcclusionCulling.h"

#include <cstdlib>
#include <cstring>

namespace msoc {

// Highest implementation Create() may return. AVX2 unless overridden, which is
// also the plugin's normal cap: the AVX-512 translation unit is not built.
inline ::MaskedOcclusionCulling::Implementation simdCap() {
    const char* env = std::getenv("MSOC_SIMD_CAP");
    if (env) {
        if (std::strcmp(env, "sse2") == 0) return ::MaskedOcclusionCulling::SSE2;
        if (std::strcmp(env, "sse41") == 0) return ::MaskedOcclusionCulling::SSE41;
    }
    return ::MaskedOcclusionCulling::AVX2;
}

// For the load-time log line, so a capped session is obvious in MSOC.log rather
// than being mistaken for a machine that genuinely lacks AVX2.
inline const char* simdCapSource() {
    const char* env = std::getenv("MSOC_SIMD_CAP");
    return (env && (std::strcmp(env, "sse2") == 0 || std::strcmp(env, "sse41") == 0))
               ? "MSOC_SIMD_CAP"
               : "default";
}

}  // namespace msoc
