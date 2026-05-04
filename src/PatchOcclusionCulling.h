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

	// Queue a batch of PRE-TRANSFORMED (clip-space) triangles for the
	// NEXT mask build. Identical semantics to addOccluder except the
	// plugin skips the world-to-clip multiply when rasterizing — input
	// vertices are already in homogeneous clip space (x, y, w), so they
	// feed MOC's RenderTriangles with modelToClipMatrix=nullptr.
	//
	// MOC reads depth from the w component (offset offW in the vertex).
	// Expected layout: stride=16, offY=4, offW=12 matches MOC's default
	// layout for `{x, y, z_ignored, w}` tuples — the canonical clip-space
	// vertex. The `z` slot at offset 8 is not read by MOC.
	//
	// Designed for consumers that build screen-space occluder geometry
	// directly in clip space (e.g. horizon-curtain terrain silhouettes).
	bool addPreTransformedOccluder(
		const float* verts, int vtxCount, int stride, int offY, int offW,
		const unsigned int* tris, int triCount);

}

// Stable C-ABI exports for out-of-tree consumers (MGE-XE etc.). Resolve
// with GetProcAddress(GetModuleHandleA("MWSE.dll"), "mwse_...").
// NI::Light* is passed as void* across the ABI boundary; consumers are
// expected to know the engine offsets they need.
extern "C" __declspec(dllexport)
void __cdecl mwse_registerLightObservedCallback(void(__cdecl* cb)(void* niLight));

extern "C" __declspec(dllexport)
void __cdecl mwse_unregisterLightObservedCallback(void(__cdecl* cb)(void* niLight));

// _Claude_ Per-frame snapshot of NiLight nodes (Point/Spot/Directional/
// Ambient) that survived frustum culling during this frame's main-camera
// CullShow walk. Independent of the engine's per-object
// NiDynamicEffectState selection — gives consumers (MGE-XE and other
// shader pipelines) the unfiltered scene-graph light set so they can
// drive their own per-pixel light loop past the engine's
// 7-lights-per-local-effect-list cap.
//
// Wire format (frozen ABI):
//
//   struct MwseLight {
//       uint32_t type;             // see kMwseLight* constants below
//       float    pos[3];           // world-space, from worldBoundOrigin
//       float    direction[3];     // world-space; spot/directional only
//       float    diffuse[3];       // light.diffuse * dimmer
//       float    ambient[3];       // light.ambient * dimmer
//       float    falloffConstant;  // point/spot: 1 / (k0 + k1*d + k2*d²)
//       float    falloffLinear;
//       float    falloffQuadratic;
//       float    spotAngle;        // spot only, in degrees (engine convention)
//       float    spotExponent;     // spot only
//       float    radius;           // engine "useful range"; point/spot only
//   };
//   // 1 uint32 + 18 floats = 76 bytes
//
// Per-type field validity:
//
//   type=1 (Point):       pos, diffuse, ambient, falloff{Constant,Linear,Quadratic}, radius
//   type=2 (Spot):        same as Point + direction, spotAngle, spotExponent
//   type=3 (Directional): direction, diffuse, ambient
//   type=4 (Ambient):     ambient (pos optionally meaningful)
//
// `radius` is the engine's "useful range" for the light, read from
// NI::Light::specular.r — Bethesda hijacked the unused specular slot to
// stash the modder-set radius (see MWSE NIPointLight.cpp, comment "for
// some reason"). Consumers can use it as a sphere radius for spatial
// selection without re-deriving from C/L/Q (which loses the linear
// term and has a hard-coded 1% threshold). 0 for non-point/spot.
//
// Fields irrelevant to a given type are zero-filled. type=0 indicates
// an unrecognised NiLight subclass and the entire record should be
// ignored by the consumer.
//
// The producer (msoc) extracts these from the NI::*Light wrapper
// classes' known offsets; consumers read the struct and never touch
// engine internals. This decouples MGE-XE from Morrowind binary version.
//
// Population gate: requires Configuration::OcclusionLightExport AND the
// existing isTopLevel preconditions (EnableMSOC + matching scene-type
// gate). When the gate is off, getCurrentFrameLights returns count=0
// and consumers should fall back to the engine's per-object set.
//
// Lifetime: snapshot is rebuilt every frame's main-camera CullShow.
// Pointers in the producer's internal storage are NiPointer-held so
// the engine can't free a light mid-frame; the copy returned to the
// consumer is by-value and has no lifetime concerns.
enum {
    kMwseLightUnknown     = 0,
    kMwseLightPoint       = 1,
    kMwseLightSpot        = 2,
    kMwseLightDirectional = 3,
    kMwseLightAmbient     = 4,
};

struct MwseLight {
    unsigned int type;
    float    pos[3];
    float    direction[3];
    float    diffuse[3];
    float    ambient[3];
    float    falloffConstant;
    float    falloffLinear;
    float    falloffQuadratic;
    float    spotAngle;
    float    spotExponent;
    float    radius;
};
static_assert(sizeof(MwseLight) == 76, "MwseLight ABI size mismatch");

// Copy up to `maxCount` MwseLight entries into `outArray`. `outCount`
// receives the number actually written (≤ maxCount, ≤ snapshot size).
// If the export gate is off or no lights are visible this frame,
// `*outCount` is set to 0 and `outArray` is untouched.
//
// Safe to call from the render thread (single-threaded contract; the
// producer populates during CullShow, the consumer queries during the
// subsequent draw calls; both run on the same thread).
extern "C" __declspec(dllexport)
void __cdecl mwse_getCurrentFrameLights(
    MwseLight* outArray, unsigned int* outCount, unsigned int maxCount);

// _Claude_ Revision counter for the current-frame lights snapshot.
// Increments every time the snapshot is rebuilt (top-of-frame in
// CullShow_detour, gated by Configuration::OcclusionLightExport).
//
// Consumers (e.g. MGE-XE's texture-light path) cache the value
// returned here and only re-upload their GPU representation when it
// changes — once-per-frame instead of once-per-draw. Returns 0 if the
// gate is off or msoc isn't producing a snapshot.
//
// Wraps at UINT_MAX. Consumers should compare for inequality, not
// strict ordering, so wrap-around is harmless.
extern "C" __declspec(dllexport)
unsigned int __cdecl mwse_getCurrentFrameLightsRevision();

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

// _Claude_ Pre-transformed variant: vertices already in clip space
// (x, y, w) per MOC's default layout. Plugin rasterizes without a
// world-to-clip multiply (nullptr to MOC::RenderTriangles). Intended
// for screen-space occluder geometry like horizon curtains.
extern "C" __declspec(dllexport)
int __cdecl mwse_addPreTransformedOccluder(
	const float* verts, int vtxCount, int stride, int offY, int offW,
	const unsigned int* tris, int triCount);
