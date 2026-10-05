# 🎯 Phase 2O Full Reference Depth-Map Dispatch

## Understanding
Extend the runtime-verified Phase 2N one-pixel depth search to a 2D compute dispatch over the CPU-scaled depth map. Match CPU scaled dimensions, pointStep traversal, integer-scaled coordinate mapping, row-major float depth storage and unsampled-zero behavior using controlled 8x6 textures; use production `BuildReferencePoints` as CPU oracle. Keep texture rejection disabled and do not generate point candidates on GPU.
## Assumptions
- Phase 2N is runtime verified from the user-provided output, including inverse-depth search and selection behavior.
- Existing reconstruction-lifetime `PlaneSweepGpu` and its descriptor/synchronization conventions remain the only GPU ownership path.
- `PlaneSweepGpuParams` already exists and must be reused as required by the request; inspect its exact layout and existing binding conventions before touching it.
- Shader startup compilation uses the installed Vulkan SDK `glslc`; runtime verification and Vulkan validation will require application execution.
## Approach
Inspect `PlaneSweepGpuParams`, Phase 2N shader/API and actual production `BuildReferencePoints`. Add a distinct full-depth-map shader with an 8x8 local size, fixed literal source sampler switch, integer coordinate scaling and Phase 2N plane search. Add an API that uploads the synthetic 8x6 images, validated camera records, `PlaneSweepGpuParams`, source metadata, and a zero-initialized full scaled depth buffer; dispatch ceil(sampleGrid/8), then copy depth and nine compact invocation diagnostics back. Replace the Phase 2N automatic hook with a controlled Phase 2O CPU setup, run production BuildReferencePoints, gate GPU dispatch on exact CPU map/mapping/geometry expectations, and compare full output.
## Key Files
- `GraphicsFXCore/src/MultiViewUtility.cpp` — synthetic images/cameras, CPU geometry and `BuildReferencePoints` oracle, automatic phase invocation.
- `GraphicsFXCore/src/PlaneSweepGpu.cpp` — GPU parameter/image buffers, descriptor setup, 2D dispatch, zero fill, output copies and comparisons.
- `GraphicsFXCore/include/PlaneSweepGpu.h` — Phase 2O entry point without exposing `PlaneSweepImage`.
- `GraphicsFXCore/include/PlaneSweepGpuTypes.h` — existing `PlaneSweepGpuParams` definition and camera conversions.
- `Shaders/planeSweepDepthMapSmokeTest.comp` — new 8x8 depth-map compute shader.
## Risks & Open Questions
- Exact `PlaneSweepGpuParams` ABI/padding and whether storage-buffer usage is already demonstrated need verification; do not redesign the struct.
- Source reprojections at image boundaries must use production SamplePlaneSweepPixel bounds semantics rather than image clipping in ProjectPoint.
- `vkCmdFillBuffer` requires transfer-destination usage plus transfer-write to compute-write synchronization before dispatch.
- Existing runtime success is available only after user launches the app; no claim of Vulkan validation success before supplied logs.

**Progress**: 14% [█░░░░░░░░░]

**Last Updated**: 2026-10-05 21:45:56

## 📝 Plan Steps
-  **Inspect the production `PlaneSweepGpuParams` ABI, Phase 2N resources and CPU mapping.**
-  **Define the Phase 2O GPU API and flat diagnostic layout.**
-  **Create the 8x8 depth-map shader with literal source descriptors and exact integer mapping.**
-  **Implement image/config/metadata staging, depth zero-fill, dispatch, and diagnostic readback.**
-  **Replace the automatic Phase 2N test hook with controlled Phase 2O CPU/GPU validation.**
-  **Build, compile GLSL, and check automatic invocation/static-indexing and synchronization.**
- ✅ **Validate Phase 2O under Vulkan at runtime and compare the full CPU/GPU depth maps.**

