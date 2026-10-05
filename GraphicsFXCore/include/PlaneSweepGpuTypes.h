///////////////////////////////////////////////////////////////////////
//  Programmer  -  Piyush (to review)
//  File Name   -  PlaneSweepGpuTypes.h
//  Description -  GPU-friendly C++ types and conversion helpers for
//                 PlaneSweep compute shader resources (std430/SSBO layout)
//
//  NOTE: This file contains only plain POD types and trivial converters.
//        No algorithmic logic is changed. These types are intended to
//        mirror the fields used by the CPU plane sweep implementation
//        and provide a deterministic memory layout for GPU upload.
///////////////////////////////////////////////////////////////////////

#pragma once

#include "MultiViewUtility.h"

#include <cstddef>
#include <cstdint>
#include <vector>
#include <type_traits>

namespace graphicsFX
{
	// Status codes for per-pixel final result (production). Backed by uint32_t
	// and intentionally does NOT expose intermediate per-plane reasons such as
	// "InsufficientSourceViews" which are plane-level diagnostics handled
	// inside ComputePlaneCost().
	enum class PlaneSweepGpuStatus : uint32_t
	{
		Invalid = 0,
		TransparentReference,
		TextureRejected,
		NoValidPlane,
		CostRejected,
		ConfidenceRejected,
		Accepted
	};

	// GPU-friendly camera intrinsics layout.
	// Matches std430-friendly packing: 4-byte scalars, tightly packed.
	struct GpuCameraIntrinsics
	{
		int32_t m_width;    // image width
		int32_t m_height;   // image height
		float   m_fx;
		float   m_fy;
		float   m_cx;
		float   m_cy;
		float   m_k1;
		float   m_k2;
		float   m_p1;
		float   m_p2;
		// Total bytes expected: 40
	};

	// GPU-friendly camera pose layout. Use explicit 16-byte alignment and
	// vec4 groups so the C++ memory layout matches GLSL std430 vec4 fields.
	struct alignas(16) GpuCameraPose
	{
		float m_position[4]; // xyz position, w = 0.0f
		float m_forward[4];  // forward vector, w = 0.0f
		float m_up[4];       // up vector, w = 0.0f
		float m_right[4];    // right vector, w = 0.0f
		// Total bytes expected: 64
	};

	// Small push-constant / parameter struct convenience for CPU side only.
	// This is not the shader push-constant definition (kept minimal here).
	struct PlaneSweepGpuParams
	{
		float m_inverseNear;                  // 0
		float m_inverseFar;                   // 4
		int32_t m_planeCount;                 // 8
		int32_t m_patchRadius;                // 12
		int32_t m_minValidSourceViews;        // 16
		float m_alphaThreshold;               // 20
		float m_minimumOpaquePatchRatio;      // 24
		float m_costThreshold;                // 28
		float m_confidenceMinimumDifference;  // 32
		float m_textureMinimumStdDev;         // 36
		uint32_t m_sourceCount;               // 40
		uint32_t m_textureRejectionEnabled;   // 44  (0/1)
		uint32_t m_confidenceEnabled;         // 48  (0/1)
		uint32_t m_pointStep;                 // 52
		uint32_t m_scaledWidth;               // 56
		uint32_t m_scaledHeight;              // 60
		// sizeof == 64
	};

	// Maximum number of source images supported by GPU version 1.
	// This is a capacity only; CPU must detect and reject configs > capacity.
	static constexpr uint32_t MAX_PLANE_SWEEP_SOURCE_IMAGES = 8u;

	// PRODUCTION: depth buffer is a contiguous float array sized scaledWidth*scaledHeight
	// and maps directly to PlaneSweepDepthMap::m_depthValues on the CPU.
	// The earlier PlaneSweepGpuResultRecord is intentionally removed from the
	// production path. Keep debug record below for diagnostics.

	// Debug result record (optional) containing additional diagnostics.
	struct PlaneSweepGpuDebugRecord
	{
		uint32_t m_scaledX;         // 0
		uint32_t m_scaledY;         // 4
		uint32_t m_depthMapIndex;   // 8
		uint32_t m_status;          // 12
		float    m_depth;           // 16
		float    m_bestCost;        // 20
		float    m_secondBestCost;  // 24
		int32_t  m_validSourceCount;// 28
		// Total: 32 bytes
	};

	// Conversion helpers: copy from CPU Colmap structs into GPU-friendly PODs.
	inline GpuCameraIntrinsics MakeGpuCameraIntrinsics(const ColmapCameraIntrinsicData &src)
	{
		GpuCameraIntrinsics dst{};
		dst.m_width = static_cast<int32_t>(src.m_width);
		dst.m_height = static_cast<int32_t>(src.m_height);
		dst.m_fx = static_cast<float>(src.m_fx);
		dst.m_fy = static_cast<float>(src.m_fy);
		dst.m_cx = static_cast<float>(src.m_cx);
		dst.m_cy = static_cast<float>(src.m_cy);
		dst.m_k1 = static_cast<float>(src.m_k1);
		dst.m_k2 = static_cast<float>(src.m_k2);
		dst.m_p1 = static_cast<float>(src.m_p1);
		dst.m_p2 = static_cast<float>(src.m_p2);
		return dst;
	}

	inline GpuCameraPose MakeGpuCameraPose(const ColmapCameraData &src)
	{
		GpuCameraPose dst{};
		dst.m_position[0] = src.m_position.x; dst.m_position[1] = src.m_position.y; dst.m_position[2] = src.m_position.z; dst.m_position[3] = 0.0f;
		dst.m_forward[0] = src.m_forward.x;   dst.m_forward[1] = src.m_forward.y;   dst.m_forward[2] = src.m_forward.z;   dst.m_forward[3] = 0.0f;
		dst.m_up[0] = src.m_up.x;             dst.m_up[1] = src.m_up.y;             dst.m_up[2] = src.m_up.z;             dst.m_up[3] = 0.0f;
		dst.m_right[0] = src.m_right.x;       dst.m_right[1] = src.m_right.y;       dst.m_right[2] = src.m_right.z;       dst.m_right[3] = 0.0f;
		return dst;
	}

	// Bulk helpers: convert vectors
	inline std::vector<GpuCameraIntrinsics> MakeGpuCameraIntrinsicsArray(const std::vector<ColmapCameraIntrinsicData> &src)
	{
		std::vector<GpuCameraIntrinsics> out;
		out.reserve(src.size());
		for (const auto &s : src) out.push_back(MakeGpuCameraIntrinsics(s));
		return out;
	}

	inline std::vector<GpuCameraPose> MakeGpuCameraPoseArray(const std::vector<ColmapCameraData> &src)
	{
		std::vector<GpuCameraPose> out;
		out.reserve(src.size());
		for (const auto &s : src) out.push_back(MakeGpuCameraPose(s));
		return out;
	}

	// Strong compile-time layout assertions for GPU-facing PODs.
	static_assert(std::is_standard_layout_v<GpuCameraIntrinsics>, "GpuCameraIntrinsics must be standard-layout");
	static_assert(std::is_trivially_copyable_v<GpuCameraIntrinsics>, "GpuCameraIntrinsics must be trivially copyable");
	static_assert(sizeof(GpuCameraIntrinsics) == 40, "GpuCameraIntrinsics size must be 40 bytes");
	static_assert(offsetof(GpuCameraIntrinsics, m_width) == 0, "m_width offset");
	static_assert(offsetof(GpuCameraIntrinsics, m_height) == 4, "m_height offset");
	static_assert(offsetof(GpuCameraIntrinsics, m_fx) == 8, "m_fx offset");
	static_assert(offsetof(GpuCameraIntrinsics, m_cx) == 16, "m_cx offset");
	static_assert(offsetof(GpuCameraIntrinsics, m_k1) == 24, "m_k1 offset");
	static_assert(offsetof(GpuCameraIntrinsics, m_p1) == 32, "m_p1 offset");

	static_assert(std::is_standard_layout_v<GpuCameraPose>, "GpuCameraPose must be standard-layout");
	static_assert(std::is_trivially_copyable_v<GpuCameraPose>, "GpuCameraPose must be trivially copyable");
	static_assert(sizeof(GpuCameraPose) == 64, "GpuCameraPose size must be 64 bytes");
	static_assert(alignof(GpuCameraPose) == 16, "GpuCameraPose alignment must be 16");
	static_assert(offsetof(GpuCameraPose, m_position) == 0, "m_position offset");
	static_assert(offsetof(GpuCameraPose, m_forward) == 16, "m_forward offset");
	static_assert(offsetof(GpuCameraPose, m_up) == 32, "m_up offset");
	static_assert(offsetof(GpuCameraPose, m_right) == 48, "m_right offset");

	static_assert(std::is_standard_layout_v<PlaneSweepGpuParams>, "PlaneSweepGpuParams must be standard-layout");
	static_assert(std::is_trivially_copyable_v<PlaneSweepGpuParams>, "PlaneSweepGpuParams must be trivially copyable");
	static_assert(sizeof(PlaneSweepGpuParams) == 64, "PlaneSweepGpuParams size must be 64 bytes");
	static_assert(offsetof(PlaneSweepGpuParams, m_inverseNear) == 0, "m_inverseNear offset");
	static_assert(offsetof(PlaneSweepGpuParams, m_planeCount) == 8, "m_planeCount offset");
	static_assert(offsetof(PlaneSweepGpuParams, m_alphaThreshold) == 20, "m_alphaThreshold offset");
	static_assert(offsetof(PlaneSweepGpuParams, m_textureMinimumStdDev) == 36, "m_textureMinimumStdDev offset");
	static_assert(offsetof(PlaneSweepGpuParams, m_pointStep) == 52, "m_pointStep offset");
	static_assert(offsetof(PlaneSweepGpuParams, m_scaledWidth) == 56, "m_scaledWidth offset");
	static_assert(offsetof(PlaneSweepGpuParams, m_scaledHeight) == 60, "m_scaledHeight offset");

	static_assert(std::is_standard_layout_v<PlaneSweepGpuDebugRecord>, "PlaneSweepGpuDebugRecord must be standard-layout");
	static_assert(std::is_trivially_copyable_v<PlaneSweepGpuDebugRecord>, "PlaneSweepGpuDebugRecord must be trivially copyable");
	static_assert(sizeof(PlaneSweepGpuDebugRecord) == 32, "PlaneSweepGpuDebugRecord must be 32 bytes");
}
