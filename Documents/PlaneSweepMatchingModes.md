# Plane-sweep matching modes

Choose the reconstruction algorithm in the `[COLMAP]` section of `GraphicsApp00/setup/setup.ini`:

```ini
PlaneSweepMatchingMode = PlaneSweep       # Existing algorithm; default when omitted.
# PlaneSweepMatchingMode = PatchMatchSlanted
PlaneSweepExecutionMode = GPU             # CPU_ONLY, GPU or CPU_GPU_COMPARE.
```

The algorithm choice is independent of CPU/GPU execution. `PlaneSweep` retains the existing exhaustive or coarse-to-fine search, selected with `PlaneSweepCoarseToFineEnabled`. `PatchMatchSlanted` always includes pyramid initialization, followed by refinement at the finest matching resolution. It does not automatically switch back to the original matching algorithm. Unsupported costs, invalid settings or unavailable requested GPU execution produce an error.

## PatchMatchSlanted

```ini
PlaneSweepMatchingMode = PatchMatchSlanted
PlaneSweepExecutionMode = GPU
PlaneSweepPatchCostMethod = ZeroMeanNormalizedCrossCorrelation
PlaneSweepPatchRadius = 3
PlaneSweepPatchMatchIterations = 3
PlaneSweepPatchMatchTiltEnabled = true
PlaneSweepPatchMatchColorSigma = 0.15
PlaneSweepPatchMatchSpatialSigma = 2.0
```

Supported costs are `MeanAbsoluteDifference`, `NormalizedCrossCorrelation` and `ZeroMeanNormalizedCrossCorrelation`. GPU NCC and ZNCC require the existing `shaderFloat64` device feature. Rank, Census and AD-Census remain available in `PlaneSweep` mode. Patch radius must be 1..16 and refinement iterations 1..12.

For a controlled tilt-only experiment, keep `PlaneSweepMatchingMode=PatchMatchSlanted` in both runs and vary only `PlaneSweepPatchMatchTiltEnabled`. Its default is `true`. Setting it to `false` keeps adaptive weights, pyramid initialization, propagation, depth refinement and acceptance rules, while restricting every candidate to a front-facing plane. The real-data GPU runner is `PlaneSweepTiltBenchmark`; it fails rather than reconstructing on the CPU. `Tests/analyze_tilt_benchmark.py` analyzes its saved GPU depth maps.

Each patch sample receives a reference-image weight:

`exp(-RGB_L1_difference / colorSigma - squared_pixel_distance / (2 * spatialSigma^2))`.

RGB values are normalized to `[0,1]`, so RGB L1 distance is in `[0,3]`. Smaller color sigma suppresses samples whose colors differ from the center more strongly. Spatial sigma is measured in pixels at the finest matching resolution. The outer square window remains bounded by `PlaneSweepPatchRadius`; the effective support changes with the image. No neural network, model checkpoint or training is involved.

Each pixel also stores a 3D plane, represented relative to the reference camera center `C` by `dot(k, X-C)=1`. A camera-Z unit ray `r` intersects it at depth `1/dot(k,r)`. This supports tilted surfaces and the existing calibrated, potentially distorted cameras. It does not assume horizontally rectified stereo. Invalid intersections and grazing planes are rejected.

Initialization preserves provisional pyramid depths until weighted plane costs can evaluate them. It tests a front-facing plane and local plane fits from neighboring depth estimates. Each refinement iteration tests neighboring planes, then deterministically perturbs center inverse depth and orientation with shrinking steps. Updates use separate input/output buffers so CPU and GPU follow the same synchronous schedule. The shared C++/GLSL algorithm is in `Shaders/planeSweepSlanted.inl`.

Costs are normalized by accepted weight. ZNCC uses weighted means, variances and covariance. A source view must retain at least `PlaneSweepMinimumOpaquePatchRatio` of the reference support weight and at least three effective samples. Masked centers stay invalid. Texture rejection uses the weighted reference variance.

Confidence is evaluated using distinct center-depth probes at +/- one eighth of the original inverse-depth plane spacing, keeping the winning orientation. The existing confidence gap threshold is scaled by the squared spacing ratio (1/64). Alternative normals at the same center depth are not treated as competing depths. Existing cost, texture, cross-view consistency and point-fusion settings still apply; their useful thresholds may differ from the unweighted matcher.

The refinement currently operates at the finest pyramid level. Cross-camera plane propagation, variable outer window sizes and hole filling are not included. The existing single-pixel sweep diagnostic is available in `PlaneSweep` mode only.

## Validation

`PlaneSweepSlantedTests` checks mode parsing, invalid settings, analytically rendered slanted geometry, rotated cameras, lens distortion, depth boundaries, masks, transparent sources, weighted texture rejection, local-depth confidence and deterministic CPU refinement. `--gpu` additionally checks GPU geometry and CPU/GPU agreement. Floating-point ties can produce small differences in selected refinement hypotheses; the GPU test checks both depth agreement and error against the analytic surface.

The existing `PlaneSweepPyramidTests` and `PlaneSweepPatchCostTests` cover the original algorithm. CMake registers CPU and GPU tests with CTest. New shaders are compiled by both the CMake shader target and startup shader compilation, including shared-include dependencies.
