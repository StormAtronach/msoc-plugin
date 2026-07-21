// External-occluder injection (MGE-XE consumer API): consumers submit
// triangles via addOccluder / addPreTransformedOccluder; the detour drains
// them into the mask each frame before native occluders. A self-contained
// feature module - depends only on shared state + leaf math, nothing in the
// core - so it closes the last core<->subsystem back-edge.
//
// STORAGE MODEL (2026-07-21 rewrite): the queue lives in a dedicated
// VirtualAlloc arena kept PAGE_READONLY except during this module's own
// bracketed writes - NOT in CRT-heap std::vectors. Forensics across many
// crash sessions showed process-wide heap corruption (a party unknown
// writes recycled mesh/instance data through stale pointers; every
// threaded suspect in MGE-XE and this plugin was individually eliminated,
// and the corruption killed innocent bystanders like DSOUND/quartz once
// this module stopped crashing first). The arena serves two purposes:
//   1. Immunity: page-protected, page-aligned, outside the contested
//      CRT heap - a stray heap-recycling writer can't land here.
//   2. Tripwire: if the corrupter targets these addresses anyway, the
//      write faults at THEIR instruction and the MWSE crash log names
//      them - converting a needle-in-haystack hunt into one log read.
// The canary + drain-time revalidation + SEH layers from the earlier
// containment work are retained as defense in depth.

#include "OcclusionApi.h"
#include "OcclusionInternal.h"
#include "Config.h"
#include "Log.h"

#include <cstdint>
#include <cstring>
#include <mutex>

#include <Windows.h>

namespace msoc::patch::occlusion {

namespace {

constexpr std::uint32_t kOccluderCanary = 0x0CC10DA5u;
constexpr std::uint32_t kArenaMagic = 0x0CC1A7EAu;

// 1 MB total. The shared triangle budget (OcclusionOccluderMaxTriangles,
// default 4096) bounds index bytes at ~48 KB; vertex payloads are capped
// per submission below. Generous headroom, still one VirtualAlloc.
constexpr std::uint32_t kArenaBytes = 1u << 20;
// Per-submission vertex payload cap - rejects absurd vtxCount*stride
// before it can exhaust the arena. MGE's curtain is ~6 KB.
constexpr std::uint32_t kMaxVertexBytesPerSubmission = 512u * 1024u;
// Hard count caps, enforced BEFORE any size arithmetic: vtxCount*stride
// and triCount*3*4 are computed in uint32, and a huge count could wrap
// the product past both the payload cap here and the identical
// recomputation in entryStillSane. 1M verts / 512k tris is far above
// any legitimate submission (budget cap is 4096 tris) and keeps every
// product comfortably inside 32 bits (1M * max stride 4096 via the
// bytes cap; tris 512k*12).
constexpr int kMaxVertexCount = 1 << 20;
constexpr int kMaxTriangleCount = 1 << 19;

inline std::uint32_t alignUp16(std::uint32_t v) { return (v + 15u) & ~15u; }

// POD entry header, packed sequentially in the arena, each followed by
// its 16-aligned vertex and index payloads.
struct ArenaEntry {
    std::uint32_t canaryHead;
    int stride;
    int offY;
    int offW;
    int vtxCount;
    int triCount;
    std::int32_t preTransformed;   // 0/1
    std::int32_t hasMatrix;        // 0/1
    float matrix[16];
    std::uint32_t vertsOffset;     // absolute arena offset
    std::uint32_t vertsBytes;
    std::uint32_t trisOffset;      // absolute arena offset
    std::uint32_t trisBytes;
    std::uint32_t nextOffset;      // absolute arena offset of next entry
    std::uint32_t canaryTail;
};
static_assert(sizeof(ArenaEntry) % 4 == 0, "ArenaEntry must pack to dwords");

struct ArenaHeader {
    std::uint32_t magic;
    std::uint32_t entryCount;
    std::uint32_t bytesUsed;       // allocation cursor (absolute offset)
    std::uint32_t firstEntryOffset;
};

std::uint8_t* g_arena = nullptr;
std::mutex g_arenaMutex;
int g_externalOccluderTrisQueued = 0;

// Bracket the plugin's own arena writes. Everything outside these
// brackets sees the pages read-only; a foreign write faults at the
// writer's instruction (the tripwire). Returns false if the OS refused
// the transition - callers must NOT write on false, or the plugin
// faults on its own memcpy against still-readonly pages.
bool arenaProtect(bool writable) {
    DWORD oldProtect = 0;
    if (!VirtualProtect(g_arena, kArenaBytes, writable ? PAGE_READWRITE : PAGE_READONLY, &oldProtect)) {
        static bool warnOnce = true;
        if (warnOnce) {
            log::getLog() << "MSOC external occluders: VirtualProtect("
                          << (writable ? "RW" : "RO") << ") failed ("
                          << GetLastError() << ")" << std::endl;
            warnOnce = false;
        }
        return false;
    }
    return true;
}

ArenaHeader* arenaHeader() { return reinterpret_cast<ArenaHeader*>(g_arena); }

// Lazy one-time arena setup. Returns false if the OS refused the pages.
bool ensureArena() {
    if (g_arena) {
        return true;
    }
    g_arena = static_cast<std::uint8_t*>(
        VirtualAlloc(nullptr, kArenaBytes, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
    if (!g_arena) {
        log::getLog() << "MSOC external occluders: arena VirtualAlloc failed ("
                      << GetLastError() << ") - external submissions disabled" << std::endl;
        return false;
    }
    ArenaHeader* h = arenaHeader();
    h->magic = kArenaMagic;
    h->entryCount = 0;
    h->bytesUsed = alignUp16(sizeof(ArenaHeader));
    h->firstEntryOffset = h->bytesUsed;
    arenaProtect(false);
    log::getLog() << "MSOC external occluders: page-protected arena live at "
                  << static_cast<const void*>(g_arena) << " (" << kArenaBytes << " bytes)" << std::endl;
    return true;
}

// Reset the arena to empty. Caller holds the mutex.
void arenaReset() {
    if (!arenaProtect(true)) {
        return;  // pages still RO; stale entries remain but stay valid
    }
    ArenaHeader* h = arenaHeader();
    h->magic = kArenaMagic;
    h->entryCount = 0;
    h->bytesUsed = alignUp16(sizeof(ArenaHeader));
    h->firstEntryOffset = h->bytesUsed;
    arenaProtect(false);
}

// Boundary validation shared by both submission paths. Rejects what
// would corrupt memory or crash MOC later rather than at the DLL
// boundary:
//   - stride not a whole number of floats (payload sizing assumes it).
//   - out-of-range index: MOC's vertex gather has no bounds check, so a
//     bad index would read out of bounds on the render thread during
//     the drain's RenderTriangles.
bool validateSubmission(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount, const char* who) {
    if (!verts || !tris || vtxCount <= 0 || triCount <= 0) {
        return false;
    }
    if (vtxCount > kMaxVertexCount || triCount > kMaxTriangleCount) {
        return false;
    }
    if (stride <= 0 || stride > 4096 || stride % static_cast<int>(sizeof(float)) != 0 || offY < 0 || offW < 0 || offY + static_cast<int>(sizeof(float)) > stride || offW + static_cast<int>(sizeof(float)) > stride) {
        return false;
    }
    const size_t idxCount = static_cast<size_t>(triCount) * 3;
    for (size_t i = 0; i < idxCount; ++i) {
        if (tris[i] >= static_cast<unsigned int>(vtxCount)) {
            static bool warnOnce = true;
            if (warnOnce) {
                log::getLog() << "MSOC " << who << ": index " << tris[i]
                              << " out of range (vtxCount " << vtxCount
                              << ") - rejecting external submission" << std::endl;
                warnOnce = false;
            }
            return false;
        }
    }
    return true;
}

// Copy a validated submission into the arena. Caller holds the mutex.
// Returns false when the arena or budget can't take it.
bool arenaAppend(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount,
    const float* modelMatrix16, bool preTransformed) {
    const std::uint32_t vertsBytes = static_cast<std::uint32_t>(vtxCount) * static_cast<std::uint32_t>(stride);
    const std::uint32_t trisBytes = static_cast<std::uint32_t>(triCount) * 3u * sizeof(std::uint32_t);
    if (vertsBytes > kMaxVertexBytesPerSubmission) {
        static bool warnOnce = true;
        if (warnOnce) {
            log::getLog() << "MSOC external occluders: submission vertex payload "
                          << vertsBytes << " bytes exceeds cap " << kMaxVertexBytesPerSubmission
                          << " - rejecting" << std::endl;
            warnOnce = false;
        }
        return false;
    }

    ArenaHeader* h = arenaHeader();
    const std::uint32_t entryOff = alignUp16(h->bytesUsed);
    const std::uint32_t vertsOff = alignUp16(entryOff + sizeof(ArenaEntry));
    const std::uint32_t trisOff = alignUp16(vertsOff + vertsBytes);
    const std::uint32_t endOff = alignUp16(trisOff + trisBytes);
    if (endOff > kArenaBytes) {
        static bool warnOnce = true;
        if (warnOnce) {
            log::getLog() << "MSOC external occluders: arena full ("
                          << h->bytesUsed << " used, submission needs "
                          << (endOff - entryOff) << ") - rejecting" << std::endl;
            warnOnce = false;
        }
        return false;
    }

    if (!arenaProtect(true)) {
        return false;  // cannot write; reject the submission
    }
    ArenaEntry* e = reinterpret_cast<ArenaEntry*>(g_arena + entryOff);
    e->canaryHead = kOccluderCanary;
    e->stride = stride;
    e->offY = offY;
    e->offW = offW;
    e->vtxCount = vtxCount;
    e->triCount = triCount;
    e->preTransformed = preTransformed ? 1 : 0;
    e->hasMatrix = modelMatrix16 ? 1 : 0;
    if (modelMatrix16) {
        std::memcpy(e->matrix, modelMatrix16, sizeof(e->matrix));
    } else {
        std::memset(e->matrix, 0, sizeof(e->matrix));
    }
    e->vertsOffset = vertsOff;
    e->vertsBytes = vertsBytes;
    e->trisOffset = trisOff;
    e->trisBytes = trisBytes;
    e->nextOffset = endOff;
    e->canaryTail = kOccluderCanary;
    std::memcpy(g_arena + vertsOff, verts, vertsBytes);
    std::memcpy(g_arena + trisOff, tris, trisBytes);
    h->bytesUsed = endOff;
    h->entryCount += 1;
    arenaProtect(false);
    return true;
}

// Drain-time recheck of everything RenderTriangles is about to trust.
// The arena is page-protected, so a mismatch here means either a bug in
// this module or a corrupter armed with VirtualProtect - both worth the
// loudest possible log line.
bool entryStillSane(const ArenaEntry& e) {
    if (e.canaryHead != kOccluderCanary || e.canaryTail != kOccluderCanary) return false;
    if (e.stride <= 0 || e.stride % static_cast<int>(sizeof(float)) != 0) return false;
    if (e.offY < 0 || e.offW < 0) return false;
    if (e.offY + static_cast<int>(sizeof(float)) > e.stride) return false;
    if (e.offW + static_cast<int>(sizeof(float)) > e.stride) return false;
    if (e.vtxCount <= 0 || e.triCount <= 0) return false;
    if (e.vtxCount > kMaxVertexCount || e.triCount > kMaxTriangleCount || e.stride > 4096) return false;
    if (e.vertsBytes != static_cast<std::uint32_t>(e.vtxCount) * static_cast<std::uint32_t>(e.stride)) return false;
    if (e.trisBytes != static_cast<std::uint32_t>(e.triCount) * 3u * sizeof(std::uint32_t)) return false;
    if (e.vertsOffset < sizeof(ArenaHeader) || e.vertsOffset + e.vertsBytes > kArenaBytes) return false;
    if (e.trisOffset < sizeof(ArenaHeader) || e.trisOffset + e.trisBytes > kArenaBytes) return false;
    const std::uint32_t* tris = reinterpret_cast<const std::uint32_t*>(g_arena + e.trisOffset);
    const std::uint32_t idxCount = static_cast<std::uint32_t>(e.triCount) * 3u;
    for (std::uint32_t i = 0; i < idxCount; ++i) {
        if (tris[i] >= static_cast<std::uint32_t>(e.vtxCount)) return false;
    }
    return true;
}

enum class EntryDrainResult { Ok, Corrupt, Faulted };

// Recheck + rasterize one arena entry under SEH. Separate function
// because __try cannot share a scope with unwindable C++ objects
// (C2712); everything in here is POD.
EntryDrainResult rasterizeEntryGuarded(const ArenaEntry& e) {
    __try {
        if (!entryStillSane(e)) {
            return EntryDrainResult::Corrupt;
        }
        // Matrix selection:
        //   preTransformed  -> nullptr (MOC consumes clip-space as-is)
        //   otherwise        -> g_worldToClip [* entry matrix if set]
        float combinedMatrix[16];
        const float* modelToClip = nullptr;
        if (!e.preTransformed) {
            modelToClip = g_worldToClip;
            if (e.hasMatrix) {
                clipmath::mat4MulColumnMajor(g_worldToClip, e.matrix, combinedMatrix);
                modelToClip = combinedMatrix;
            }
        }
        const ::MaskedOcclusionCulling::VertexLayout layout(e.stride, e.offY, e.offW);
        // Winding from g_frame.occluderWinding - see the discipline
        // notes in OcclusionPass.cpp; CW consumer hulls drop out as a
        // safe under-occlude when OcclusionOccluderCCWOnly is set.
        g_msoc->RenderTriangles(
            reinterpret_cast<const float*>(g_arena + e.vertsOffset),
            reinterpret_cast<const unsigned int*>(g_arena + e.trisOffset),
            e.triCount,
            modelToClip,
            g_frame.occluderWinding,
            ::MaskedOcclusionCulling::CLIP_PLANE_ALL,
            layout);
        return EntryDrainResult::Ok;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return EntryDrainResult::Faulted;
    }
}

}  // namespace

// Drop queued external-occluder submissions on teardown (OcclusionInternal.h).
void clearExternalOccluderQueue() {
    std::lock_guard<std::mutex> lock(g_arenaMutex);
    if (g_arena) {
        arenaReset();
    }
    g_externalOccluderTrisQueued = 0;
}

void drainPendingOccluders() {
    std::lock_guard<std::mutex> lock(g_arenaMutex);
    if (!g_arena) {
        return;
    }
    const ArenaHeader* h = arenaHeader();
    if (h->magic != kArenaMagic) {
        log::getLog() << "MSOC drain: ARENA HEADER CORRUPTED (magic="
                      << std::hex << h->magic << std::dec
                      << ") - resetting queue. A writer got through page"
                         " protection; check the next crash log for the"
                         " faulting module." << std::endl;
        arenaReset();
        g_externalOccluderTrisQueued = 0;
        return;
    }
    if (h->entryCount == 0) {
        return;
    }

    std::uint32_t off = h->firstEntryOffset;
    for (std::uint32_t i = 0; i < h->entryCount; ++i) {
        if (off + sizeof(ArenaEntry) > kArenaBytes) {
            log::getLog() << "MSOC drain: entry walk left the arena (off="
                          << off << ") - abandoning drain" << std::endl;
            break;
        }
        const ArenaEntry& e = *reinterpret_cast<const ArenaEntry*>(g_arena + off);
        const EntryDrainResult r = rasterizeEntryGuarded(e);
        if (r != EntryDrainResult::Ok) {
            log::getLog() << "MSOC drain: CORRUPTED queue entry dropped ("
                          << (r == EntryDrainResult::Faulted ? "faulted" : "recheck") << ") -"
                          << " canaries=" << std::hex << e.canaryHead << "/" << e.canaryTail
                          << " stride=" << e.stride << " offY=" << e.offY << " offW=" << e.offW
                          << " vtxCount=" << e.vtxCount << " triCount=" << e.triCount
                          << " vertsOff=" << e.vertsOffset << " trisOff=" << e.trisOffset
                          << std::dec << std::endl;
            break;  // entry chain is untrustworthy past a corrupt link
        }
        g_stats.occluderTriangles += e.triCount;
        ++g_stats.rasterizedAsOccluder;
        off = e.nextOffset;
    }

    arenaReset();
    g_externalOccluderTrisQueued = 0;
}

bool addOccluder(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount,
    const float* modelMatrix16) {
    if (!validateSubmission(verts, vtxCount, stride, offY, offW, tris, triCount, "addOccluder")) {
        return false;
    }
    // Mask not live (EnableMSOC off, init failed, teardown). Drop
    // silently - soft-feature semantics.
    if (!g_msoc) {
        return false;
    }
    const int cap = static_cast<int>(Configuration::OcclusionOccluderMaxTriangles);
    std::lock_guard<std::mutex> lock(g_arenaMutex);
    if (g_externalOccluderTrisQueued + triCount > cap) {
        static bool warnOnce = true;
        if (warnOnce) {
            log::getLog() << "MSOC addOccluder: triangle budget exceeded ("
                          << g_externalOccluderTrisQueued << " queued + " << triCount
                          << " requested > cap " << cap
                          << ") - rejecting external submission" << std::endl;
            warnOnce = false;
        }
        return false;
    }
    if (!ensureArena()) {
        return false;
    }
    if (!arenaAppend(verts, vtxCount, stride, offY, offW, tris, triCount,
                     modelMatrix16, /*preTransformed*/ false)) {
        return false;
    }
    g_externalOccluderTrisQueued += triCount;
    return true;
}

bool addPreTransformedOccluder(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount) {
    if (!validateSubmission(verts, vtxCount, stride, offY, offW, tris, triCount, "addPreTransformedOccluder")) {
        return false;
    }
    if (!g_msoc) {
        return false;
    }
    const int cap = static_cast<int>(Configuration::OcclusionOccluderMaxTriangles);
    std::lock_guard<std::mutex> lock(g_arenaMutex);
    if (g_externalOccluderTrisQueued + triCount > cap) {
        static bool warnOnce = true;
        if (warnOnce) {
            log::getLog() << "MSOC addPreTransformedOccluder: triangle budget exceeded ("
                          << g_externalOccluderTrisQueued << " queued + " << triCount
                          << " requested > cap " << cap
                          << ") - rejecting external submission" << std::endl;
            warnOnce = false;
        }
        return false;
    }
    if (!ensureArena()) {
        return false;
    }
    if (!arenaAppend(verts, vtxCount, stride, offY, offW, tris, triCount,
                     /*modelMatrix16*/ nullptr, /*preTransformed*/ true)) {
        return false;
    }
    g_externalOccluderTrisQueued += triCount;
    return true;
}

}  // namespace msoc::patch::occlusion
