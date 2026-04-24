#pragma once

namespace NI {
	struct Light;
}

namespace msoc::patch::occlusion {

	// Install hooks for DX8 Masked Software Occlusion Culling.
	// _Claude_ Hooks always install — EnableMSOC is a pure runtime gate,
	// re-checked every frame inside the detour. MCM can toggle culling
	// on/off at any time. When the gate is off, the detour falls through
	// to vanilla cullShowBody at zero overhead. Resources (g_msoc + the
	// ~57MB ring-buffer threadpool) are allocated lazily on the first
	// MCM toggle-on (or eagerly at startup if EnableMSOC begins true) and
	// torn down on toggle-off (workers joined, buffers freed). Toggling
	// off→on costs one threadpool spin-up (~1ms); toggling on→off blocks
	// the next render frame for the worker join (bounded, typically <1ms
	// because workers were already suspended).
	void installPatches();

	// _Claude_ Observer callback fired once per iteration of
	// NiDX8LightManager::updateLights for every NiLight* the renderer walks
	// — before the MSOC cull decision, before the enabled check. Observers
	// see every iterated light (disabled or enabled, occluded or not) and
	// must treat the pointer as read-only.
	//
	// Use: external systems (e.g. MGE-XE) that want to snapshot the live
	// renderer-iterated light list without re-patching 0x6BB7D4. Without
	// this registry, only the first patcher wins the site.
	//
	// Constraints:
	//   - Runs inside the render-thread hot loop. Keep work minimal: dedup
	//     + cheap publish. No logging, no allocation on the steady path.
	//   - Do not mutate the light or the scene graph.
	//   - Observers fire whenever the detour runs, which is every frame
	//     once the patches are installed. EnableMSOC OFF still walks the
	//     light list (the detour just short-circuits the cull decision),
	//     so observers see the same data either way — set OcclusionCullLights
	//     off if you want observation without the cull side-effect.
	//
	// Thread safety: register before rendering starts. The vector is
	// iterated lock-free from the render thread; post-frame mutation is
	// undefined. Double-register of the same pointer is a no-op.
	using LightObservedCallback = void(__cdecl*)(NI::Light* light);
	void registerLightObservedCallback(LightObservedCallback cb);
	void unregisterLightObservedCallback(LightObservedCallback cb);

	// _Claude_ Mask query API for consumers that want to reuse MWSE's
	// CPU occlusion mask (e.g. MGE-XE culling its own draw passes
	// before submission to D3D). Result codes are frozen in ABI —
	// values match the mwse_testOcclusion* C exports below.
	enum MaskQueryResult {
		kMaskQueryVisible    = 0,  // not occluded; draw the thing
		kMaskQueryOccluded   = 1,  // fully behind the mask; safe to skip
		kMaskQueryViewCulled = 2,  // rect collapsed; outside the frustum
		kMaskQueryNotReady   = 3,  // mask not populated this frame
	};

	// True once the depth buffer reflects the complete vanilla main-scene
	// occluder set. Flips false at the next frame's ClearBuffer. Callers
	// should either gate expensive queries on this or accept that a
	// !ready query returns kMaskQueryNotReady (treat as visible).
	bool isOcclusionMaskReady();

	// World-space sphere test. Uses the same projection as the engine's
	// main-scene CullShow — if the caller's draw pass uses a different
	// camera (different near/far plane, different projection), the
	// verdict is not meaningful for that pass.
	MaskQueryResult testOcclusionSphere(
		float worldX, float worldY, float worldZ, float radius);

	// World-space AABB test. Converts to bounding sphere internally —
	// same looseness as the engine's own NiAVObject::worldBoundRadius.
	MaskQueryResult testOcclusionAABB(
		float minX, float minY, float minZ,
		float maxX, float maxY, float maxZ);

	// World-space oriented bounding box (OBB) test. Expands `center +
	// (±vx ±vy ±vz)` into 8 corners, runs MOC's TestTriangles against
	// the 12 box faces. Tighter than testOcclusionSphere for non-
	// spherical shapes (architectural giants, towers) — catches meshes
	// whose sphere was too loose to fit behind a single occluder in
	// screen space. Expect ~1-5 μs per call vs ~100 ns for TestRect;
	// callers should gate on radius threshold before using this path.
	MaskQueryResult testOcclusionOBB(
		float cx, float cy, float cz,
		float vxX, float vxY, float vxZ,
		float vyX, float vyY, float vyZ,
		float vzX, float vzY, float vzZ);

	// Batch sphere test. Consumer passes `count` spheres as consecutive
	// (x, y, z, r) float4 tuples (so `centersAndRadii` is count*4 floats
	// long), and `resultsOut` is a caller-provided int array of length
	// `count` that the plugin fills with kMaskQuery* codes. Equivalent
	// to calling testOcclusionSphere `count` times but saves the per-
	// call DLL boundary overhead — MGE-XE does 1500-2800 tests per cull
	// pass and benefits measurably.
	//
	// If the mask isn't ready (staleness, teardown), every slot is
	// filled with kMaskQueryNotReady and the function returns quickly.
	void testOcclusionSphereBatch(
		const float* centersAndRadii, int count, int* resultsOut);

	// Copy the snapshot world-to-clip matrix (16 floats, Intel/column-
	// major layout) into caller-provided buffer. Matches the depth data
	// in g_msoc_prev — use this to project points through the exact view
	// the mask was built for.
	void getSnapshotViewProj(float outMatrix[16]);

	// Wall-clock age of the current snapshot, in milliseconds. Returns
	// 0 if no snapshot has ever been captured (before first swap).
	unsigned long long getSnapshotAgeMs();

	// Fixed mask resolution (currently 512x256 for this build).
	void getMaskResolution(int* outWidth, int* outHeight);

	// Write the current occlusion mask to disk as a PFM (Portable
	// FloatMap) image. Grayscale, raw float32 depths bottom-up — opens
	// in GIMP / Photoshop / ImageJ without conversion. Use for
	// diagnosing occluder coverage ("is the canton actually in the
	// mask?" without wiring visual tints). Returns true on success.
	bool dumpOcclusionMask(const char* path);

	// Queue a batch of triangles to be rasterized into the NEXT mask
	// build. One-frame latency by design — data lands in frame N+1's
	// mask, which consumers query in frame N+2, matching the existing
	// double-buffer snapshot contract.
	//
	// Plugin takes ownership by copying the input arrays on the
	// caller's thread; caller can free or reuse the buffers after the
	// call returns. `modelMatrix` may be null (world-space / identity).
	// Stride/offY/offW match MOC's VertexLayout semantics.
	//
	// Returns true if queued, false if rejected (budget exhausted, mask
	// resources not live, or invalid input).
	bool addOccluder(
		const float* verts, int vtxCount, int stride, int offY, int offW,
		const unsigned int* tris, int triCount,
		const float* modelMatrix16);

}

// Stable C-ABI exports for out-of-tree consumers (MGE-XE etc.). Resolve
// with GetProcAddress(GetModuleHandleA("MWSE.dll"), "mwse_...").
// NI::Light* is passed as void* across the ABI boundary; consumers are
// expected to know the engine offsets they need.
extern "C" __declspec(dllexport)
void __cdecl mwse_registerLightObservedCallback(void(__cdecl* cb)(void* niLight));

extern "C" __declspec(dllexport)
void __cdecl mwse_unregisterLightObservedCallback(void(__cdecl* cb)(void* niLight));

// _Claude_ MSOC mask query exports. Return values are the stable
// MWSE_OCC_* codes (match msoc::patch::occlusion::MaskQueryResult):
//   0 = Visible     (draw it)
//   1 = Occluded    (safe to skip)
//   2 = ViewCulled  (outside the frustum)
//   3 = NotReady    (mask empty or unbuilt this frame — treat as Visible)
extern "C" __declspec(dllexport)
int __cdecl mwse_isOcclusionMaskReady();

extern "C" __declspec(dllexport)
int __cdecl mwse_testOcclusionSphere(float worldX, float worldY, float worldZ, float radius);

extern "C" __declspec(dllexport)
int __cdecl mwse_testOcclusionAABB(
	float minX, float minY, float minZ,
	float maxX, float maxY, float maxZ);

// _Claude_ OBB (oriented bounding box) test — 12 floats describe the
// box as center + 3 axis half-extent vectors (vx/vy/vz). Escalation
// path for giants whose bounding sphere is too loose to ever resolve
// as OCCLUDED even when most of the mesh sits behind an occluder.
// Return codes match the MWSE_OCC_* set used by the sphere/AABB exports.
extern "C" __declspec(dllexport)
int __cdecl mwse_testOcclusionOBB(
	float cx, float cy, float cz,
	float vxX, float vxY, float vxZ,
	float vyX, float vyY, float vyZ,
	float vzX, float vzY, float vzZ);

// _Claude_ Batch sphere query — same semantics as mwse_testOcclusionSphere
// but amortises DLL-boundary cost over N tests in one call.
//   centersAndRadii: count*4 floats: (x0,y0,z0,r0, x1,y1,z1,r1, ...)
//   count:           number of spheres
//   resultsOut:      caller-allocated int array of length `count`; each
//                    slot receives a MWSE_OCC_* code.
extern "C" __declspec(dllexport)
void __cdecl mwse_testOcclusionSphereBatch(
	const float* centersAndRadii, int count, int* resultsOut);

// _Claude_ Snapshot metadata accessors. Help consumers reason about the
// view the current mask was built for, and about its freshness. All
// read-only; safe to call at any time (return harmless zeros if the
// plugin hasn't captured a snapshot yet).
extern "C" __declspec(dllexport)
void __cdecl mwse_getSnapshotViewProj(float outMatrix[16]);

extern "C" __declspec(dllexport)
unsigned long long __cdecl mwse_getSnapshotAgeMs();

extern "C" __declspec(dllexport)
void __cdecl mwse_getMaskResolution(int* outWidth, int* outHeight);

// _Claude_ Dump the current occlusion mask to `path` as a PFM image
// (Portable FloatMap — 3-line ASCII header + raw float32 depths,
// bottom-up). Returns 1 on success, 0 otherwise. Intended for
// keypress-driven diagnostics from consumer processes; consumer
// picks the file path so it can sit next to its own logs.
extern "C" __declspec(dllexport)
int __cdecl mwse_dumpOcclusionMask(const char* path);

// _Claude_ External occluder injection. Consumer (MGE-XE, other mods)
// submits triangles that will be rasterized into the NEXT mask build,
// alongside the plugin's native near-scene occluders. Plugin copies
// the input arrays — caller may free after return. Returns 1 if queued,
// 0 if rejected (over budget, invalid args, or mask resources inactive).
// One-frame latency matches the double-buffered snapshot contract.
extern "C" __declspec(dllexport)
int __cdecl mwse_addOccluder(
	const float* verts, int vtxCount, int stride, int offY, int offW,
	const unsigned int* tris, int triCount,
	const float* modelMatrix16);
