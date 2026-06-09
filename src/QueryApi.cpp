// Out-of-tree mask query API (testOcclusion*, snapshot accessors), split
// from PatchOcclusionCulling.cpp. Reads the published snapshot
// (g_snapshot / g_msoc_prev) via OcclusionInternal.h; the public C-ABI
// declarations live in PatchOcclusionCulling.h.

#include "PatchOcclusionCulling.h"
#include "OcclusionInternal.h"

#include "NIPoint3.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>

namespace msoc::patch::occlusion {

	// External-consumer mask query API. Reads the SNAPSHOT (g_msoc_prev)
	// captured at drain-complete - race-free regardless of when the
	// consumer calls. The freshness gate (kSnapshotMaxAgeMs) rejects
	// snapshots older than ~200ms so paused/load/alt-tab states don't
	// serve a frozen mask.
	bool isOcclusionMaskReady() {
		if (!g_msoc_prev || g_snapshot.tickMs == 0) {
			return false;
		}
		const unsigned long long now = GetTickCount64();
		return (now - g_snapshot.tickMs) <= kSnapshotMaxAgeMs;
	}

	// Sphere test against the SNAPSHOT buffer using the _prev matrix +
	// NDC constants captured at swap time. Math mirrors testSphereVisible;
	// kept separate so the in-progress-mask path stays untouched.
	static ::MaskedOcclusionCulling::CullingResult testSphereVisiblePrev(
		const NI::Point3& center, float radius)
	{
		// Project center through last frame's world-to-clip.
		const clipmath::ClipXYW c =
			clipmath::projectWorld(g_snapshot.worldToClip, center.x, center.y, center.z);

		const float wMin = c.w - (radius + g_frame.depthSlackWorldUnits) * g_snapshot.wGradMag;
		if (wMin <= kNearClipW) {
			return ::MaskedOcclusionCulling::VISIBLE;
		}

		const float invW = 1.0f / c.w;
		const float cxNdc = c.x * invW;
		const float cyNdc = c.y * invW;
		const float rxNdc = radius * g_snapshot.ndcRadiusX * invW;
		const float ryNdc = radius * g_snapshot.ndcRadiusY * invW;

		float ndcMinX = std::max(cxNdc - rxNdc, -1.0f);
		float ndcMinY = std::max(cyNdc - ryNdc, -1.0f);
		float ndcMaxX = std::min(cxNdc + rxNdc, 1.0f);
		float ndcMaxY = std::min(cyNdc + ryNdc, 1.0f);
		if (ndcMinX >= ndcMaxX || ndcMinY >= ndcMaxY) {
			return ::MaskedOcclusionCulling::VIEW_CULLED;
		}

		return g_msoc_prev->TestRect(ndcMinX, ndcMinY, ndcMaxX, ndcMaxY, wMin);
	}

	MaskQueryResult testOcclusionSphere(float worldX, float worldY, float worldZ, float radius) {
		if (!isOcclusionMaskReady()) {
			return kMaskQueryNotReady;
		}
		const NI::Point3 center(worldX, worldY, worldZ);
		const auto result = testSphereVisiblePrev(center, radius);
		switch (result) {
		case ::MaskedOcclusionCulling::VISIBLE:     return kMaskQueryVisible;
		case ::MaskedOcclusionCulling::OCCLUDED:    return kMaskQueryOccluded;
		case ::MaskedOcclusionCulling::VIEW_CULLED: return kMaskQueryViewCulled;
		}
		return kMaskQueryVisible;
	}

	MaskQueryResult testOcclusionAABB(
		float minX, float minY, float minZ,
		float maxX, float maxY, float maxZ)
	{
		const float cx = (minX + maxX) * 0.5f;
		const float cy = (minY + maxY) * 0.5f;
		const float cz = (minZ + maxZ) * 0.5f;
		const float dx = maxX - cx;
		const float dy = maxY - cy;
		const float dz = maxZ - cz;
		const float r = std::sqrt(dx * dx + dy * dy + dz * dz);
		return testOcclusionSphere(cx, cy, cz, r);
	}

	void getSnapshotViewProj(float outMatrix[16]) {
		if (!outMatrix) return;
		std::memcpy(outMatrix, g_snapshot.worldToClip, sizeof(g_snapshot.worldToClip));
	}

	unsigned long long getSnapshotAgeMs() {
		if (g_snapshot.tickMs == 0) return 0;
		return GetTickCount64() - g_snapshot.tickMs;
	}

	void getMaskResolution(int* outWidth, int* outHeight) {
		if (outWidth)  *outWidth  = static_cast<int>(kMsocWidth);
		if (outHeight) *outHeight = static_cast<int>(kMsocHeight);
	}

	void testOcclusionSphereBatch(
		const float* centersAndRadii, int count, int* resultsOut)
	{
		if (!resultsOut || count <= 0) {
			return;
		}
		if (!centersAndRadii) {
			for (int i = 0; i < count; ++i) {
				resultsOut[i] = kMaskQueryNotReady;
			}
			return;
		}

		// Single readiness check - Morrowind's render-adjacent work is
		// single-threaded so the mask can't flip mid-batch.
		if (!isOcclusionMaskReady()) {
			for (int i = 0; i < count; ++i) {
				resultsOut[i] = kMaskQueryNotReady;
			}
			return;
		}

		for (int i = 0; i < count; ++i) {
			const float* s = centersAndRadii + i * 4;
			const NI::Point3 center(s[0], s[1], s[2]);
			const auto result = testSphereVisiblePrev(center, s[3]);
			switch (result) {
			case ::MaskedOcclusionCulling::VISIBLE:     resultsOut[i] = kMaskQueryVisible;    break;
			case ::MaskedOcclusionCulling::OCCLUDED:    resultsOut[i] = kMaskQueryOccluded;   break;
			case ::MaskedOcclusionCulling::VIEW_CULLED: resultsOut[i] = kMaskQueryViewCulled; break;
			default:                                    resultsOut[i] = kMaskQueryVisible;    break;
			}
		}
	}

	MaskQueryResult testOcclusionOBB(
		float cx, float cy, float cz,
		float vxX, float vxY, float vxZ,
		float vyX, float vyY, float vyZ,
		float vzX, float vzY, float vzZ)
	{
		if (!isOcclusionMaskReady()) {
			return kMaskQueryNotReady;
		}

		// Project the 8 corners through the snapshot matrix and test the
		// covered screen-space rectangle against the mask. This keeps the
		// query tied to the bbox footprint instead of a loose sphere, while
		// avoiding TestTriangles false positives on tall slabs where only the
		// bottom face is behind occluders.
		const float sx[8] = { -1, +1, +1, -1, -1, +1, +1, -1 };
		const float sy[8] = { -1, -1, +1, +1, -1, -1, +1, +1 };
		const float sz[8] = { -1, -1, -1, -1, +1, +1, +1, +1 };
		const float* m = g_snapshot.worldToClip;
		float ndcMinX = +FLT_MAX;
		float ndcMinY = +FLT_MAX;
		float ndcMaxX = -FLT_MAX;
		float ndcMaxY = -FLT_MAX;
		float wMin = +FLT_MAX;

		for (int i = 0; i < 8; ++i) {
			const float wx = cx + sx[i] * vxX + sy[i] * vyX + sz[i] * vzX;
			const float wy = cy + sx[i] * vxY + sy[i] * vyY + sz[i] * vzY;
			const float wz = cz + sx[i] * vxZ + sy[i] * vyZ + sz[i] * vzZ;
			const clipmath::ClipXYW clip = clipmath::projectWorld(m, wx, wy, wz);

			if (clip.w <= kNearClipW) {
				return kMaskQueryVisible;
			}

			const float invW = 1.0f / clip.w;
			const float ndcX = clip.x * invW;
			const float ndcY = clip.y * invW;
			ndcMinX = std::min(ndcMinX, ndcX);
			ndcMinY = std::min(ndcMinY, ndcY);
			ndcMaxX = std::max(ndcMaxX, ndcX);
			ndcMaxY = std::max(ndcMaxY, ndcY);
			wMin = std::min(wMin, clip.w);
		}

		wMin -= g_frame.depthSlackWorldUnits * g_snapshot.wGradMag;
		if (wMin <= kNearClipW) {
			return kMaskQueryVisible;
		}

		ndcMinX = std::max(ndcMinX, -1.0f);
		ndcMinY = std::max(ndcMinY, -1.0f);
		ndcMaxX = std::min(ndcMaxX, 1.0f);
		ndcMaxY = std::min(ndcMaxY, 1.0f);
		if (ndcMinX >= ndcMaxX || ndcMinY >= ndcMaxY) {
			return kMaskQueryViewCulled;
		}

		const auto result = g_msoc_prev->TestRect(ndcMinX, ndcMinY, ndcMaxX, ndcMaxY, wMin);

		switch (result) {
		case ::MaskedOcclusionCulling::VISIBLE:     return kMaskQueryVisible;
		case ::MaskedOcclusionCulling::OCCLUDED:    return kMaskQueryOccluded;
		case ::MaskedOcclusionCulling::VIEW_CULLED: return kMaskQueryViewCulled;
		}
		return kMaskQueryVisible;
	}

} // namespace msoc::patch::occlusion
