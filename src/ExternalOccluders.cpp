// External-occluder injection (MGE-XE consumer API): consumers submit
// triangles via addOccluder / addPreTransformedOccluder; the detour drains
// them into the mask each frame before native occluders. A self-contained
// feature module - depends only on shared state + leaf math, nothing in the
// core - so it closes the last core<->subsystem back-edge.

#include "OcclusionApi.h"
#include "OcclusionInternal.h"
#include "Config.h"
#include "Log.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <optional>
#include <vector>

namespace msoc::occlusion {

// External occluder injection. Each PendingExternalOccluder is a self-
// contained copy of a consumer's submission; plugin owns the memory.
// Populated on the consumer's thread, drained on the render thread
// before native near-scene occluders rasterize. Mutex contention is
// trivial in practice (consumers submit a handful of batches per
// frame; plugin drains once per frame).
//
// g_externalOccluderTrisQueued enforces external-first rejection
// against OcclusionOccluderMaxTriangles, preserving the native budget.
// Unnamed namespace: TU-local type. `static` cannot be applied to a
// type, so this is the only way to keep it out of reach of an
// identically-named type in another TU - which is exactly how this
// struct and OcclusionPass.cpp's DeferredOccluderRef (then also called
// PendingOccluder) silently merged. See CHANGELOG.
namespace {
struct PendingExternalOccluder {
    std::vector<float> verts;
    std::vector<std::uint32_t> tris;
    int stride;
    int offY;
    int offW;
    int vtxCount;
    int triCount;
    std::optional<std::array<float, 16>> modelMatrix;
    // True: verts are clip-space (x, y, w); drain passes nullptr to
    // RenderTriangles. Used by screen-space submitters (horizon curtains).
    bool preTransformed = false;
};

static std::vector<PendingExternalOccluder> g_pendingExternalOccluders;
static std::mutex g_pendingExternalOccludersMutex;
static int g_externalOccluderTrisQueued = 0;
}  // namespace

// Drop queued external-occluder submissions on teardown (OcclusionInternal.h).
void clearExternalOccluderQueue() {
    std::lock_guard<std::mutex> lock(g_pendingExternalOccludersMutex);
    g_pendingExternalOccluders.clear();
    g_externalOccluderTrisQueued = 0;
}

void drainPendingOccluders() {
    // Swap-and-release so a slow rasterise doesn't block consumer
    // threads in mwse_addOccluder.
    std::vector<PendingExternalOccluder> localQueue;
    int drainedTris = 0;
    {
        std::lock_guard<std::mutex> lock(g_pendingExternalOccludersMutex);
        if (g_pendingExternalOccluders.empty()) {
            return;
        }
        localQueue = std::move(g_pendingExternalOccluders);
        g_pendingExternalOccluders.clear();
        drainedTris = g_externalOccluderTrisQueued;
        g_externalOccluderTrisQueued = 0;
    }

    for (const auto& p : localQueue) {
        // Matrix selection:
        //   preTransformed=true  -> nullptr (MOC skips the transform,
        //                           consumes clip-space vertices as-is)
        //   otherwise             -> g_worldToClip [* p.modelMatrix if set]
        float combinedMatrix[16];
        const float* modelToClip = nullptr;
        if (!p.preTransformed) {
            modelToClip = g_worldToClip;
            if (p.modelMatrix.has_value()) {
                clipmath::mat4MulColumnMajor(g_worldToClip, p.modelMatrix->data(), combinedMatrix);
                modelToClip = combinedMatrix;
            }
        }

        const ::MaskedOcclusionCulling::VertexLayout layout(p.stride, p.offY, p.offW);
        // Winding from g_frame.occluderWinding: BACKFACE_NONE (default)
        // rasterizes both faces - consumers typically submit closed convex
        // hulls, and since Hi-Z tiles store minimum depth a back face can't
        // tighten the mask, so culling it is a pure throughput choice. When
        // OcclusionOccluderCCWOnly is set, a CW-wound consumer hull is
        // dropped (safe under-occlude).
        g_msoc->RenderTriangles(
            p.verts.data(), p.tris.data(), p.triCount,
            modelToClip,
            g_frame.occluderWinding,
            ::MaskedOcclusionCulling::CLIP_PLANE_ALL,
            layout);
        // Folds into g_stats.occluderTriangles so the per-frame log line
        // reflects the external contribution.
        g_stats.occluderTriangles += p.triCount;
        ++g_stats.rasterizedAsOccluder;
    }
}

bool addOccluder(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount,
    const float* modelMatrix16) {
    if (!verts || !tris || vtxCount <= 0 || triCount <= 0) {
        return false;
    }
    // stride/offY/offW: stride positive, offsets fit one vertex.
    if (stride <= 0 || offY < 0 || offW < 0 || offY + static_cast<int>(sizeof(float)) > stride || offW + static_cast<int>(sizeof(float)) > stride) {
        return false;
    }

    // Mask not live (EnableMSOC off, init failed, teardown). Drop
    // silently - soft-feature semantics.
    if (!g_msoc) {
        return false;
    }

    // External submissions share OcclusionOccluderMaxTriangles with
    // native occluders. External-first rejection preserves the
    // native mask under contention.
    const int cap = static_cast<int>(Configuration::OcclusionOccluderMaxTriangles);
    std::lock_guard<std::mutex> lock(g_pendingExternalOccludersMutex);
    if (g_externalOccluderTrisQueued + triCount > cap) {
        // Rate-limit the log so a chatty consumer can't flood it.
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

    // Copy into plugin-owned storage; consumer can free after return.
    PendingExternalOccluder p;
    p.stride = stride;
    p.offY = offY;
    p.offW = offW;
    p.vtxCount = vtxCount;
    p.triCount = triCount;

    // stride is bytes per vertex - copy as raw bytes to preserve
    // whatever layout the caller picked.
    const size_t vtxBytes = static_cast<size_t>(vtxCount) * static_cast<size_t>(stride);
    p.verts.resize(vtxBytes / sizeof(float));
    std::memcpy(p.verts.data(), verts, vtxBytes);

    p.tris.assign(tris, tris + static_cast<size_t>(triCount) * 3);

    if (modelMatrix16) {
        std::array<float, 16> m;
        std::memcpy(m.data(), modelMatrix16, sizeof(m));
        p.modelMatrix = m;
    }

    g_externalOccluderTrisQueued += triCount;
    g_pendingExternalOccluders.emplace_back(std::move(p));
    return true;
}

bool addPreTransformedOccluder(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount) {
    // Same validation + budget as addOccluder, plus preTransformed.
    // Validation is duplicated rather than factored - keeps both
    // public paths readable in isolation.
    if (!verts || !tris || vtxCount <= 0 || triCount <= 0) {
        return false;
    }
    if (stride <= 0 || offY < 0 || offW < 0 || offY + static_cast<int>(sizeof(float)) > stride || offW + static_cast<int>(sizeof(float)) > stride) {
        return false;
    }
    if (!g_msoc) {
        return false;
    }

    const int cap = static_cast<int>(Configuration::OcclusionOccluderMaxTriangles);
    std::lock_guard<std::mutex> lock(g_pendingExternalOccludersMutex);
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

    PendingExternalOccluder p;
    p.stride = stride;
    p.offY = offY;
    p.offW = offW;
    p.vtxCount = vtxCount;
    p.triCount = triCount;
    p.preTransformed = true;

    const size_t vtxBytes = static_cast<size_t>(vtxCount) * static_cast<size_t>(stride);
    p.verts.resize(vtxBytes / sizeof(float));
    std::memcpy(p.verts.data(), verts, vtxBytes);

    p.tris.assign(tris, tris + static_cast<size_t>(triCount) * 3);

    g_externalOccluderTrisQueued += triCount;
    g_pendingExternalOccluders.emplace_back(std::move(p));
    return true;
}

}  // namespace msoc::occlusion
