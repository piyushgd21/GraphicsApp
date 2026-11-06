# Plane-sweep matching modes

Choose the reconstruction algorithm in the `[COLMAP]` section of `GraphicsApp00/setup/setup.ini`:

```ini
PlaneSweepMatchingMode = PlaneSweep       # Existing algorithm; default when omitted.
# PlaneSweepMatchingMode = PatchMatchSlanted
PlaneSweepExecutionMode = GPU             # CPU_ONLY, GPU or CPU_GPU_COMPARE.
```

The algorithm choice is independent of CPU/GPU execution. `PlaneSweep` retains the existing exhaustive or coarse-to-fine search, selected with `PlaneSweepCoarseToFineEnabled`. `PatchMatchSlanted` always performs pyramid initialization and tilted-plane refinement at every matching resolution. It does not automatically switch back to the original matching algorithm. Unsupported costs, invalid settings or unavailable requested GPU execution produce an error.

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

RGB values are normalized to `[0,1]`, so RGB L1 distance is in `[0,3]`. Smaller color sigma suppresses samples whose colors differ from the center more strongly. Spatial sigma and `PlaneSweepPatchRadius` are measured in pixels at each matching resolution, so coarse levels cover a larger area of the original image. The outer square window remains bounded by `PlaneSweepPatchRadius`; the effective support changes with the image. No neural network, model checkpoint or training is involved.

Each pixel also stores a 3D plane, represented relative to the reference camera center `C` by `dot(k, X-C)=1`. A camera-Z unit ray `r` intersects it at depth `1/dot(k,r)`. This supports tilted surfaces and the existing calibrated, potentially distorted cameras. It does not assume horizontally rectified stereo. Invalid intersections and grazing planes are rejected.

At every level, initialization preserves provisional sweep depths until weighted plane costs can evaluate them. It tests a front-facing plane, local plane fits from neighboring depth estimates, and the preceding level's plane selected by nearest pixel-center mapping. Plane coefficients are relative to the unchanged reference camera center, so they need no image-scale conversion; inherited planes are re-scored using the current resolution. Each refinement iteration tests neighboring planes, then deterministically perturbs center inverse depth and orientation with shrinking steps. `PlaneSweepPatchMatchIterations` applies independently at every level. Updates use separate input/output buffers so CPU and GPU follow the same synchronous schedule. The shared C++/GLSL algorithm is in `Shaders/planeSweepSlanted.inl`.

Costs are normalized by accepted weight. ZNCC uses weighted means, variances and covariance. A source view must retain at least `PlaneSweepMinimumOpaquePatchRatio` of the reference support weight and at least three effective samples. Masked centers stay invalid. Texture rejection uses the weighted reference variance.

Confidence is evaluated using distinct center-depth probes at +/- one eighth of the original inverse-depth plane spacing, keeping the winning orientation. The existing confidence gap threshold is scaled by the squared spacing ratio (1/64). Alternative normals at the same center depth are not treated as competing depths. Existing cost, texture, cross-view consistency and point-fusion settings still apply; their useful thresholds may differ from the unweighted matcher.

Refined depths and planes are carried to the next finer level. Intermediate states retain provisional geometry even if cost or confidence checks reject it; masked pixels and invalid weighted supports remain invalid. Final output applies the existing rejection rules. Saved pyramid depth maps and per-level counters now describe the refined level, with acceptance counters reporting the rejection checks before provisional depths are retained. Cross-camera plane propagation, variable outer window sizes and explicit hole filling are not included. The existing single-pixel sweep diagnostic is available in `PlaneSweep` mode only.

## Validation

Cross-view consistency now requires the same supporting depth sample to pass forward and return relative-depth checks and a round-trip pixel check. `PlaneSweepCrossViewRelativeDepthTolerance` defaults to `0.01` (1%); `PlaneSweepCrossViewReprojectionTolerancePixels` defaults to `1.0`, measured in reference depth-map pixels at the matching resolution. The existing neighborhood search accommodates discretization and sparse legacy sampling, but a nearby depth alone is no longer sufficient. Each comparison camera contributes at most one agreeing view. Missing, invalid or insufficient supporting maps reject a point when consistency is enabled. The filter preserves surviving positions and colors; it does not average depths or fuse points. Startup output reports candidates kept and the active thresholds.

`PlaneSweepConsistencyTests` covers exact translated-camera matches, a false depth-only match, independent pixel/depth thresholds, odd and unequal map dimensions, legacy sampling, rotated/distorted cameras, invalid depths and maps, insufficient views, unchanged surviving points and configuration parsing. It is registered as the CPU-only CTest `PlaneSweepConsistency`.

`PlaneSweepSlantedTests` checks mode parsing, invalid settings, analytically rendered slanted geometry, rotated cameras, lens distortion, depth boundaries, masks, transparent sources, weighted texture rejection, local-depth confidence and deterministic CPU refinement. Regression cases also cover coarse-level refinement work, inheritance without sweep seeds, odd parent dimensions and provisional geometry retention before final rejection. `--gpu` additionally checks GPU geometry, coarse-level refinement work and CPU/GPU agreement. Floating-point ties can produce small differences in selected refinement hypotheses; the GPU test checks both depth agreement and error against the analytic surface.

The existing `PlaneSweepPyramidTests` and `PlaneSweepPatchCostTests` cover the original algorithm. CMake registers CPU and GPU tests with CTest. New shaders are compiled by both the CMake shader target and startup shader compilation, including shared-include dependencies.

For a three-arm scan9 comparison, run `PlaneSweepTiltBenchmark setup.ini NEW_OUTPUT_FOLDER 48 modes`, then `python Tests/analyze_matching_modes.py NEW_OUTPUT_FOLDER`. This compares normal `PlaneSweep`, weighted front-facing `PatchMatchSlanted`, and tilted `PatchMatchSlanted` on the same selected views and sources. It saves raw depth, matching cost and accepted plane tilt maps before cross-view filtering or fusion. The analysis records acceptance transitions, paired sparse-reference errors and paired cross-view checks; a `report.txt`, JSON results and plots are saved beside the depth maps. The front-facing/slanted pair isolates tilt, while the normal/slanted comparison includes weighting, propagation and confidence differences. These are geometric proxies when dense ground truth is unavailable.
