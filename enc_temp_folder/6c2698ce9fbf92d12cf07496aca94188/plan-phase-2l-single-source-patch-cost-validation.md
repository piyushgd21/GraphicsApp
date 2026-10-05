# 🎯 Phase 2L Single-Source Patch-Cost Validation

## Understanding
Implement a GPU equivalent of only the one-source inner patch-cost calculation and validate eight synthetic cases against the actual production `ComputePlaneCost`. Do not add multi-source aggregation or depth-plane logic. Run only Phase 2L once through the existing reconstruction-lifetime GPU handoff.
## Assumptions
- Phase 2K is runtime verified per user; the prior K rerun is superseded.
- Existing validated bilinear sampling, masked sampling, UnprojectPixel, ProjectPoint shader semantics and camera POD layouts should be reused where possible.
- `PlaneSweepImage` is private to MultiViewUtility.cpp and must remain so.
- Production CPU helper visibility permits invoking `ComputePlaneCost` and the sampling/projection helpers from the same translation unit.
- Runtime results require the app to run the updated shader and cannot be claimed from a C++ build alone.
## Approach
Inspect the production ComputePlaneCost body and existing shader helper implementations/test API. Create two exact 7x7 CPU PlaneSweepImage textures and the specified cameras/cases. Verify projection mapping with production UnprojectPixel and ProjectPoint before creating/running the GPU cost test; abort GPU execution on failed mapping or conceptual CPU validity mismatch. Generate CPU expected validity/cost using production ComputePlaneCost. Add a focused shader with reference/source sampling, alpha checks in exact CPU order, unprojection, projection, one source slot, patch count qualification, and cost output. Extend PlaneSweepGpu with synthetic texture uploads, all valid descriptor-array slots populated, five raw case buffers, GPU dispatch/readback, validity-first comparison, cost tolerance, and diagnostic counts. Hook only Phase 2L; retain earlier smoke code but stop invoking it automatically.
## Key Files
- `GraphicsFXCore/src/MultiViewUtility.cpp` - private synthetic PlaneSweepImages, production CPU oracle, projection sanity check and handoff.
- `GraphicsFXCore/src/PlaneSweepGpu.cpp` - synthetic image upload, descriptor writes, compute/readback and comparison.
- `GraphicsFXCore/include/PlaneSweepGpu.h` - narrow test API that does not expose PlaneSweepImage.
- `Shaders/planeSweepSingleSourceCostSmokeTest.comp` - one-source patch-cost implementation.
- `Shaders/planeSweepPixelSamplingSmokeTest.comp` - reference for already validated sample helper semantics.
## Risks & Open Questions
- ComputePlaneCost’s exact signature and helper visibility must be inspected before wiring; do not duplicate the CPU oracle.
- The mandated projection shift must be empirically verified from the production helpers; stop before GPU execution if it fails.
- Runtime Vulkan validation and exact cost/count output remain pending until the updated app and startup shader compiler run.

**Progress**: 100% [██████████]

**Last Updated**: 2026-10-05 20:47:48

## 📝 Plan Steps
- ✅ **Inspect production ComputePlaneCost, PlaneSweepImage construction/access, the validated sampling/projection GLSL helpers, and current automatic test hook.**
- ✅ **Create the one-source cost GLSL shader, retaining CPU operation order and diagnostic outputs.**
- ✅ **Declare the narrow PlaneSweepGpu test interface for texture bytes, cameras, cases and expected CPU results.**
- ✅ **Implement synthetic image/mask uploads, full descriptor-array initialization, staged case-buffer uploads, dispatch, readback, validation and cleanup.**
- ✅ **Construct exact CPU synthetic images/cameras/cases, perform production projection sanity checks and CPU-oracle comparisons, then invoke the GPU test once.**
- ✅ **Ensure only Phase 2L executes automatically, build and inspect changed code, and report runtime status without inventing results.**

