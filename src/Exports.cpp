// C-ABI exports for out-of-tree consumers (MGE-XE etc.). Thin __cdecl
// thunks forwarding to the msoc::patch::occlusion public API; the
// matching dllexport declarations live in OcclusionApi.h.
// Split out of OcclusionPass.cpp.

#include "OcclusionApi.h"

// ============================================================
// C-ABI exports (out-of-tree consumers: MGE-XE etc.)
// ============================================================

// C-ABI shims. void*/NI::Light* share representation on x86 and both
// callback pointers are __cdecl, so the reinterpret_cast is an ABI
// formality. Consumers look up by name (not ordinal) - names are
// frozen.
extern "C" __declspec(dllexport) void __cdecl mwse_registerLightObservedCallback(void(__cdecl* cb)(void* niLight)) {
    using Typed = msoc::patch::occlusion::LightObservedCallback;
    msoc::patch::occlusion::registerLightObservedCallback(
        reinterpret_cast<Typed>(cb));
}

extern "C" __declspec(dllexport) void __cdecl mwse_unregisterLightObservedCallback(void(__cdecl* cb)(void* niLight)) {
    using Typed = msoc::patch::occlusion::LightObservedCallback;
    msoc::patch::occlusion::unregisterLightObservedCallback(
        reinterpret_cast<Typed>(cb));
}

extern "C" __declspec(dllexport) void __cdecl mwse_registerVisibleGeomCallback(
    void(__cdecl* cb)(void* const*, const float*, int)) {
    using Typed = msoc::patch::occlusion::VisibleGeomCallback;
    msoc::patch::occlusion::registerVisibleGeomCallback(reinterpret_cast<Typed>(cb));
}

extern "C" __declspec(dllexport) void __cdecl mwse_unregisterVisibleGeomCallback(
    void(__cdecl* cb)(void* const*, const float*, int)) {
    using Typed = msoc::patch::occlusion::VisibleGeomCallback;
    msoc::patch::occlusion::unregisterVisibleGeomCallback(reinterpret_cast<Typed>(cb));
}

// Mask query exports. Returns MWSE_OCC_* codes (0=Visible, 1=Occluded,
// 2=ViewCulled, 3=NotReady) - frozen ABI independent of Intel's enum.
// Gate with mwse_isOcclusionMaskReady() if unsure about timing.
extern "C" __declspec(dllexport) int __cdecl mwse_isOcclusionMaskReady() {
    return msoc::patch::occlusion::isOcclusionMaskReady() ? 1 : 0;
}

extern "C" __declspec(dllexport) int __cdecl mwse_testOcclusionSphere(float worldX, float worldY, float worldZ, float radius) {
    return static_cast<int>(
        msoc::patch::occlusion::testOcclusionSphere(worldX, worldY, worldZ, radius));
}

extern "C" __declspec(dllexport) int __cdecl mwse_testOcclusionAABB(
    float minX, float minY, float minZ,
    float maxX, float maxY, float maxZ) {
    return static_cast<int>(
        msoc::patch::occlusion::testOcclusionAABB(minX, minY, minZ, maxX, maxY, maxZ));
}

extern "C" __declspec(dllexport) int __cdecl mwse_testOcclusionOBB(
    float cx, float cy, float cz,
    float vxX, float vxY, float vxZ,
    float vyX, float vyY, float vyZ,
    float vzX, float vzY, float vzZ) {
    return static_cast<int>(
        msoc::patch::occlusion::testOcclusionOBB(
            cx, cy, cz,
            vxX, vxY, vxZ,
            vyX, vyY, vyZ,
            vzX, vzY, vzZ));
}

extern "C" __declspec(dllexport) void __cdecl mwse_testOcclusionSphereBatch(
    const float* centersAndRadii, int count, int* resultsOut) {
    msoc::patch::occlusion::testOcclusionSphereBatch(
        centersAndRadii, count, resultsOut);
}

extern "C" __declspec(dllexport) void __cdecl mwse_getSnapshotViewProj(float outMatrix[16]) {
    msoc::patch::occlusion::getSnapshotViewProj(outMatrix);
}

extern "C" __declspec(dllexport) unsigned long long __cdecl mwse_getSnapshotAgeMs() {
    return msoc::patch::occlusion::getSnapshotAgeMs();
}

extern "C" __declspec(dllexport) void __cdecl mwse_getMaskResolution(int* outWidth, int* outHeight) {
    msoc::patch::occlusion::getMaskResolution(outWidth, outHeight);
}

extern "C" __declspec(dllexport) int __cdecl mwse_dumpOcclusionMask(const char* path) {
    return msoc::patch::occlusion::dumpOcclusionMask(path) ? 1 : 0;
}

extern "C" __declspec(dllexport) int __cdecl mwse_addOccluder(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount,
    const float* modelMatrix16) {
    return msoc::patch::occlusion::addOccluder(
               verts, vtxCount, stride, offY, offW,
               tris, triCount,
               modelMatrix16)
               ? 1
               : 0;
}

extern "C" __declspec(dllexport) int __cdecl mwse_addPreTransformedOccluder(
    const float* verts, int vtxCount, int stride, int offY, int offW,
    const unsigned int* tris, int triCount) {
    return msoc::patch::occlusion::addPreTransformedOccluder(
               verts, vtxCount, stride, offY, offW,
               tris, triCount)
               ? 1
               : 0;
}
