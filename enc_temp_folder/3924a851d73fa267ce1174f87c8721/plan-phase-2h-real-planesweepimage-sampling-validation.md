# 🎯 Phase 2H Real PlaneSweepImage Sampling Validation

## Understanding
At the first successfully loaded suitable internal PlaneSweepImage, run one real-data upload/sample validation through the existing non-null reconstruction-lifetime PlaneSweepGpu pointer, compare with the actual production CPU SamplePlaneSweepPixel helper, and then continue CPU reconstruction unchanged. Keep PlaneSweepImage private and do not create Vulkan resources beyond test-local buffers/images on the existing device/queue/command pool.
## Assumptions
- Phase 2H-A ownership bridge and diagnostic calls are present in live files.
- The existing Phase 2G shader implements the proven bilinear and mask semantics and can be reused with its current six-binding interface.
- MultiViewUtility.cpp can include PlaneSweepGpu.h and call a narrow public-data-only method.
- The one-shot flag is per process and only commits after a suitable loaded image test attempt.
## Approach
Inspect current bridge and `TextureData` shape, then add a public-data-only PlaneSweepGpu method to validate/upload supplied RGBA bytes, upload 16 coordinates and uint modes, dispatch the existing Phase 2G shader, and return GPU outputs. In MultiViewUtility.cpp validate texture sizes/overflow and mask classification, generate coordinates, call production `SamplePlaneSweepPixel` for CPU expectations, invoke the supplied GPU object once on the first suitable loaded image, and compare validity then valid RGBA with diagnostics. Preserve the existing CPU reference/source loading and reconstruction path. Build Debug x64 and obtain debugger/Vulkan validation runtime output.
## Key Files
- `GraphicsFXCore/include/PlaneSweepGpu.h` - narrow public-data GPU method declaration.
- `GraphicsFXCore/src/PlaneSweepGpu.cpp` - generalize/reuse the proven Phase 2G image path for real textures without changing shader semantics.
- `GraphicsFXCore/src/MultiViewUtility.cpp` - real loaded-image validation, production CPU oracle, one-shot integration and comparisons.
- `GraphicsFXCore/src/MultiViewUtility.h` - existing optional PlaneSweepGpu pointer API, preserve PlaneSweepImage encapsulation.
- `GraphicsApp00/src/Scene.cpp` - existing reconstruction-lifetime owner/pointer handoff; avoid ownership changes.
## Risks & Open Questions
- Real image extents may approach device limits or allocation limits; reject overflow/malformed inputs with clear diagnostics.
- Debugger execution may be controlled by Visual Studio; runtime output must be provided if it cannot be captured here.

**Progress**: 16% [█░░░░░░░░░]

**Last Updated**: 2026-10-05 19:37:01

## 📝 Plan Steps
- 🔄 **Verify current Phase 2H-A bridge and inspect TextureData/API constraints.**
-  **Add the narrow real-texture GPU sampling API and result types.**
-  **Implement real-image upload/readback using the proven shader and existing Vulkan context.**
-  **Add validation, production CPU oracle, coordinate generation, one-shot hook and comparisons in MultiViewUtility.**
-  **Build Debug x64 and fix relevant issues.**
- ✅ **Run under Vulkan validation and report captured real-image results.**

