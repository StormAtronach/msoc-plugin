#pragma once

namespace NI {
struct Light;
}

namespace msoc::patch::occlusion {

// Install hooks for DX8 Masked Software Occlusion Culling. Hooks
// always install - EnableMSOC is a runtime gate re-checked every
// frame inside the detour. When the gate is off the detour falls
// through to vanilla cullShowBody at zero overhead. Resources
// (g_msoc + ~57MB ring-buffer threadpool) are allocated eagerly at
// startup if EnableMSOC begins true, lazily on first MCM toggle-on
// otherwise, and torn down on toggle-off.
void installPatches();

// Observer callback fired once per iteration of
// NiDX8LightManager::updateLights, before any cull decision. Sees
// every iterated light (disabled/enabled, occluded/not). For
// external systems (e.g. MGE-XE) snapshotting the renderer-iterated
// light list without re-patching 0x6BB7D4.
//
// Constraints:
//   - Runs in the render-thread hot loop. Keep work minimal; no
//     logging, no allocation on the steady path.
//   - Treat the pointer as read-only; do not mutate the scene graph.
//   - Fires every frame once installed, regardless of EnableMSOC.
//     Set OcclusionCullLights off if you want observation without
//     the cull side-effect.
//   - Register before rendering starts; the vector is iterated
//     lock-free from the render thread. Double-register is a no-op.
using LightObservedCallback = void(__cdecl*)(NI::Light* light);
void registerLightObservedCallback(LightObservedCallback cb);
void unregisterLightObservedCallback(LightObservedCallback cb);

// Fires after all drain-slot verdicts are assigned but BEFORE any display() calls.
// Receives every node whose verdict is NOT Occluded or CachedOccluded - i.e., exactly
// the set that Morrowind will draw this frame. Callback is synchronous on the render
// thread; MGE renders depth/shadows inside it, then returns so display() can proceed.
// shapes[i] and boundsXYZR[i*4..i*4+3] are parallel arrays. bounds = (x, y, z, r).
// Do not mutate the scene graph or retain pointers past the callback's return.
using VisibleGeomCallback = void(__cdecl*)(void* const* shapes,
                                           const float* boundsXYZR, int count);
void registerVisibleGeomCallback(VisibleGeomCallback cb);
void unregisterVisibleGeomCallback(VisibleGeomCallback cb);

// Mask query API. Result codes are frozen ABI - values match the
// mwse_testOcclusion* C exports below.
enum MaskQueryResult {
    kMaskQueryVisible = 0,     // not occluded; draw the thing
    kMaskQueryOccluded = 1,    // fully behind the mask; safe to skip
    kMaskQueryViewCulled = 2,  // rect collapsed; outside the frustum
    kMaskQueryNotReady = 3,    // mask not populated this frame
};

// True once the depth buffer reflects the complete vanilla
// main-scene occluder set. Flips false at the next ClearBuffer.
// Gate expensive queries on this, or accept that a !ready query
// returns kMaskQueryNotReady (treat as visible).
bool isOcclusionMaskReady();

// World-space sphere test. Uses the engine's main-scene CullShow
// projection - verdicts aren't meaningful for draw passes that use
// a different camera.
MaskQueryResult testOcclusionSphere(
    float worldX, float worldY, float worldZ, float radius);

// World-space AABB test. Converts to bounding sphere internally -
// same looseness as NiAVObject::worldBoundRadius.
MaskQueryResult testOcclusionAABB(
    float minX, float minY, float minZ,
    float maxX, float maxY, float maxZ);

// World-space OBB test. Expands center + (+/-vx +/-vy +/-vz) into 8
// corners, projects their screen-space rectangle, and tests that
// footprint against the mask. Tighter than testOcclusionSphere for
// non-spherical shapes while remaining conservative for tall slabs.
MaskQueryResult testOcclusionOBB(
    float cx, float cy, float cz,
    float vxX, float vxY, float vxZ,
    float vyX, float vyY, float vyZ,
    float vzX, float vzY, float vzZ);

// Batch sphere test: `count` consecutive (x,y,z,r) float4 tuples
// (centersAndRadii length = count*4); resultsOut is caller-provided
// int[count] filled with kMaskQuery* codes. Equivalent to N
// testOcclusionSphere calls but amortises the DLL boundary cost.
// On !ready, every slot is kMaskQueryNotReady and returns quickly.
void testOcclusionSphereBatch(
    const float* centersAndRadii, int count, int* resultsOut);

// Copy the snapshot world-to-clip matrix (16 floats, Intel/column-
// major) into outMatrix. Use this to project points through the
// exact view the mask was built for.
void getSnapshotViewProj(float outMatrix[16]);

// Wall-clock age of the current snapshot in ms. 0 if no snapshot
// captured yet.
unsigned long long getSnapshotAgeMs();

void getMaskResolution(int* outWidth, int* outHeight);

// Dump the current occlusion mask as PFM (raw float32 depths,
// bottom-up). Opens in GIMP/Photoshop/ImageJ without conversion.
bool dumpOcclusionMask(const char* path);

// Queue triangles to be rasterized into the NEXT mask build.
// One-frame latency by design - data lands in frame N+1's mask,
// consumers query in frame N+2, matching the double-buffer contract.
// Plugin copies the input arrays; caller may free after return.
// modelMatrix may be null (world-space identity). stride/offY/offW
// match MOC's VertexLayout. Returns false on rejection (budget
// exhausted, resources not live, invalid input).
bool addOccluder(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount,
    const float* modelMatrix16);

// PRE-TRANSFORMED variant: vertices already in clip space (x, y, w).
// Plugin skips the world-to-clip multiply (modelToClip=nullptr to
// MOC::RenderTriangles). MOC reads depth from w (offset offW).
// Expected layout: stride=16, offY=4, offW=12 (x, y, z_ignored, w).
// For consumers building screen-space occluder geometry directly
// (e.g. horizon-curtain terrain silhouettes).
bool addPreTransformedOccluder(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount);

}  // namespace msoc::patch::occlusion

// Stable C-ABI exports for out-of-tree consumers (MGE-XE etc.). Resolve
// with GetProcAddress(GetModuleHandleA("MWSE.dll"), "mwse_..."). NI::Light*
// is passed as void* across the ABI boundary.
extern "C" __declspec(dllexport) void __cdecl mwse_registerLightObservedCallback(void(__cdecl* cb)(void* niLight));

extern "C" __declspec(dllexport) void __cdecl mwse_unregisterLightObservedCallback(void(__cdecl* cb)(void* niLight));

extern "C" __declspec(dllexport) void __cdecl mwse_registerVisibleGeomCallback(void(__cdecl* cb)(void* const*, const float*, int));

extern "C" __declspec(dllexport) void __cdecl mwse_unregisterVisibleGeomCallback(void(__cdecl* cb)(void* const*, const float*, int));

// MSOC mask query exports. Return values match
// msoc::patch::occlusion::MaskQueryResult:
//   0 = Visible     (draw)
//   1 = Occluded    (skip)
//   2 = ViewCulled  (outside frustum)
//   3 = NotReady    (treat as Visible)
extern "C" __declspec(dllexport) int __cdecl mwse_isOcclusionMaskReady();

extern "C" __declspec(dllexport) int __cdecl mwse_testOcclusionSphere(float worldX, float worldY, float worldZ, float radius);

extern "C" __declspec(dllexport) int __cdecl mwse_testOcclusionAABB(
    float minX, float minY, float minZ,
    float maxX, float maxY, float maxZ);

// 12 floats describe the box as center + 3 axis half-extent vectors.
// Escalation path for giants whose bounding sphere is too loose to
// resolve as OCCLUDED. Codes match the MWSE_OCC_* set above.
extern "C" __declspec(dllexport) int __cdecl mwse_testOcclusionOBB(
    float cx, float cy, float cz,
    float vxX, float vxY, float vxZ,
    float vyX, float vyY, float vyZ,
    float vzX, float vzY, float vzZ);

// Same semantics as mwse_testOcclusionSphere but amortises DLL-boundary
// cost over N tests.
//   centersAndRadii: count*4 floats: (x0,y0,z0,r0, x1,y1,z1,r1, ...)
//   resultsOut:      caller-allocated int[count]; each slot gets a MWSE_OCC_* code.
extern "C" __declspec(dllexport) void __cdecl mwse_testOcclusionSphereBatch(
    const float* centersAndRadii, int count, int* resultsOut);

// Snapshot metadata accessors. Read-only; safe to call any time
// (return harmless zeros before first capture).
extern "C" __declspec(dllexport) void __cdecl mwse_getSnapshotViewProj(float outMatrix[16]);

extern "C" __declspec(dllexport) unsigned long long __cdecl mwse_getSnapshotAgeMs();

extern "C" __declspec(dllexport) void __cdecl mwse_getMaskResolution(int* outWidth, int* outHeight);

// Dump current mask to `path` as PFM. Returns 1 on success, 0 otherwise.
// Consumer picks the path so it can sit next to its own logs.
extern "C" __declspec(dllexport) int __cdecl mwse_dumpOcclusionMask(const char* path);

// External occluder injection. Triangles rasterize into the NEXT mask
// build alongside native near-scene occluders. Returns 1 if queued, 0
// if rejected. One-frame latency matches the double-buffered snapshot.
extern "C" __declspec(dllexport) int __cdecl mwse_addOccluder(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount,
    const float* modelMatrix16);

// Pre-transformed variant: clip-space verts (x, y, w). Plugin
// rasterizes without world-to-clip multiply. For screen-space occluder
// geometry like horizon curtains.
extern "C" __declspec(dllexport) int __cdecl mwse_addPreTransformedOccluder(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount);
