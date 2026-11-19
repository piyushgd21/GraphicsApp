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

### Optional boundary color selection

`PlaneSweepBoundaryColorSelectionEnabled = false` is the default. With the switch omitted or false, point colors and the reconstruction path retain their previous behavior. Set it to `true` in `[COLMAP]` to run a separate CPU color pass after cross-view filtering and before the selected fusion mode. No shader changes are required. Both CMake and the Visual Studio solution include the color pass.

The pass only reconsiders candidates whose original reference observation is less than eight depth-map pixels from a foreground-mask or image boundary. A conservative foreground grid requires the full original-image footprint of each cell to have mask opacity at least 0.99. Clearance uses chessboard distance on this grid; eight pixels correspond to approximately sixteen original-image pixels at image scale 0.5. These are fixed initial experiment constants, not changes to matching or geometry settings. Images without a foreground mask retain their original colors.

The nearby-camera ranking used by geometric validation also ranks alternative color views, but `PlaneSweepBoundaryColorViewCount` independently limits the color search (1..128, default 4 when omitted). Set it to 16 or 32 for a wider color-only search while leaving `PlaneSweepCrossViewCount` and `PlaneSweepCrossViewMinimumViews` unchanged. This is a candidate limit, not a requirement for 16 or 32 agreeing views. A replacement still requires at least two distinct masked views with sufficient interior clearance. Each observation must pass the configured forward/return depth tolerances and round-trip pixel tolerance. Every depth sample in the projected interpolation footprint must be valid and agree in depth; the closest sample supplies the round-trip check. There is no nearby-depth search across holes or occlusions. RGB is sampled at the actual projection of the candidate, not at a neighboring support point. Of the reliable observations, an RGB L1 medoid selects one observed color; ties prefer greater mask clearance. Colors are not averaged, and green is not explicitly rejected.

```ini
PlaneSweepCrossViewCount = 4
PlaneSweepCrossViewMinimumViews = 3
PlaneSweepBoundaryColorSelectionEnabled = true
PlaneSweepBoundaryColorViewCount = 16
```

For a controlled color-search comparison, vary only `PlaneSweepBoundaryColorViewCount` between 4, 16 and 32. The existing enabled switch still bypasses the entire color pass when false. The log reports the configured color camera limit. Regression tests cover independent color/geometric limits, wider search reaching reliable cameras beyond occluded nearest views, and parsing the limits separately.

Only candidate RGB changes. Alpha, positions, count, ordering, raw saved depth maps and geometric rejection rules are preserved. Interior reference colors, insufficient alternatives, missing masks/images, invalid depths and occlusions leave the original color intact. With geometric consistency disabled, enabling color selection retains the depth maps needed for its own visibility checks but does not turn on geometric rejection. Optional fusion/outlier filtering still runs afterward with its existing settings. Images are decoded one comparison camera at a time, avoiding an all-camera full-resolution texture cache. This adds CPU processing and image reads only when enabled; color-pass time is included in the final filtering metric.

Startup output reports `Boundary color selection: recolored ... / ... boundary points`, insufficient interior observations and unavailable images. The pass targets boundary contamination and does not remove green illumination already recorded on interior surfaces. Nearby cameras can share the same boundary contamination, leaving too few eligible interior observations; this conservative pass can therefore retain most original colors. Compare runs by changing only this switch. `PlaneSweepColorSelectionTests` covers the disabled path, observed-color selection, geometry/alpha preservation, interior/fallback behavior, occlusion, depth holes, masks, valid green colors, odd-sized maps and legacy coordinates; configuration parsing is covered by `PlaneSweepConsistencyTests`.

### Geometric consistency

Cross-view consistency now requires the same supporting depth sample to pass forward and return relative-depth checks and a round-trip pixel check. `PlaneSweepCrossViewRelativeDepthTolerance` defaults to `0.01` (1%); `PlaneSweepCrossViewReprojectionTolerancePixels` defaults to `1.0`, measured in reference depth-map pixels at the matching resolution. The existing neighborhood search accommodates discretization and sparse legacy sampling, but a nearby depth alone is no longer sufficient. Each comparison camera contributes at most one agreeing view. Missing, invalid or insufficient supporting maps reject a point when consistency is enabled. The filter preserves surviving positions and colors; it does not average depths or fuse points. Startup output reports candidates kept and the active thresholds.

`PlaneSweepConsistencyTests` covers exact translated-camera matches, a false depth-only match, independent pixel/depth thresholds, odd and unequal map dimensions, legacy sampling, rotated/distorted cameras, invalid depths and maps, insufficient views, unchanged surviving points and configuration parsing. It is registered as the CPU-only CTest `PlaneSweepConsistency`.

### Geometric fusion

`PlaneSweepFusionMode` selects `None`, `Voxel`, or `Geometric`. An explicit selector overrides `PlaneSweepPointFusionEnabled`. When omitted, the old boolean retains its previous behavior and default. The checked-in setup selects `Geometric`; its existing depth generation and cross-view thresholds are unchanged.

`Geometric` first rejects unsupported candidates using the existing cross-view checks, then groups corresponding observations and emits a robust representative for each accepted group. It always validates geometry, including when `PlaneSweepCrossViewConsistencyEnabled=false`. It reuses `PlaneSweepCrossViewCount`, `PlaneSweepCrossViewMinimumViews`, `PlaneSweepCrossViewRelativeDepthTolerance`, and `PlaneSweepCrossViewReprojectionTolerancePixels`. Minimum views counts other cameras, excluding the reference; pixel tolerance is measured at the reference depth-map resolution. Voxel size and voxel minimum-observation settings do not apply.

Validation and grouping share camera ranking and the same discrete forward/return camera-Z and round-trip pixel checks. Grouping chooses the best passing pixel in each comparison camera. For depth estimation, the standalone correction's guarded inverse-depth interpolation samples the continuous projection: every contributing footprint depth must be valid, pass the depth limit, and span no more than half the depth tolerance. This preserves pinhole planes and prevents estimating through holes or across depth discontinuities. Source camera-Z observations are transformed into the reference camera before estimation. A shared median/MAD estimator rejects depth outliers and weights remaining depths by their robust residual and geometric score, with an original-reference anchor. This is the same estimator used by standalone correction. Movement stays on the seed camera ray and is bounded to half the configured relative depth tolerance. Discrete contributor pixels must pass the original geometric limits again at the final position; a failed move can instead consolidate at the original seed. An insufficient safe interpolation footprint prevents consolidation, without rejecting an already validated seed.

Each group stays anchored to its seed; there is no transitive graph traversal that could join a chain of nearby surfaces. A split between two depth populations (at least two observations on each side, separated by more than a quarter of the depth tolerance) prevents consolidation at that seed, matching the standalone correction's ambiguity guard. Native `(depth map, pixel)` identity is retained on CPU and GPU candidates. A bit per map pixel tracks ownership, committed only after acceptance. A claimed correspondence cannot be selected again or emit another copy. Ownership is separate from validation: if a validated seed lacks enough unused observations for a new group, it remains at its original position rather than becoming a hole. This conservative fallback means some overlapping samples may remain. Input depth maps are immutable, and saved `.bin` files remain the original accepted predictions. The seed RGBA is preserved, including any preceding optional boundary color selection. Optional spatial outlier filtering runs afterward.

In `Geometric` mode the standalone `PlaneSweepCrossViewCorrectionEnabled` pass is bypassed: fusion performs its own single consensus estimate and revalidation. The standalone surface-regularization path remains available with `None` or `Voxel`. No additional normal or learned-confidence maps are required. `Geometric` applies to both matching modes and both CPU/GPU depth estimators; fusion itself runs on the CPU.

Startup output reports emitted representatives, consolidated output observations, moved representatives, ambiguous groups, and seeds retained without a new group. Existing before/after-fusion metrics capture the reduction after cross-view rejection. `PlaneSweepGeometricFusionTests` covers consolidation and robust positioning, separate neighboring samples and depth layers, unsupported observations, view-count semantics, independent geometric limits, odd/unequal and legacy sampling, input immutability, RGBA, and selector parsing/backward compatibility. These are analytic correctness checks, not evidence of improved accuracy on a real scene. Compare the modes on one fixed set of depth maps before tuning thresholds.

### Optional cross-view depth correction

`PlaneSweepCrossViewCorrectionEnabled = false` is the default. With `None` or `Voxel` fusion (including legacy configurations), setting it to `true` adds a separate CPU geometry pass after cross-view filtering, shared by CPU and GPU reconstruction. It requires `PlaneSweepCrossViewConsistencyEnabled=true`. Use `PlaneSweepFusionMode=None` in both arms of a standalone correction comparison. `Geometric` fusion instead uses its own single consensus estimate as described above. Depth estimation, matching thresholds, renderer settings and original RGBA are unchanged.

Filtering still decides which original candidates survive. The enabled correction now combines a local surface model with cross-view evidence. A reference-sized lookup grid contains only surviving candidates. Each reference uses an immutable snapshot of its initial depths and rays; later candidates do not read previously corrected neighbors. Only one reference grid/snapshot is resident at a time.

Local regularization starts from a connected 5x5 neighborhood at the matching resolution. It requires at least eight surrounding accepted samples, support on both sides of both image axes, nondegenerate two-dimensional geometry, and at least six effective weighted samples after robust fitting. It cannot traverse missing cells. Connectivity stops at color/depth changes; weights combine pixel distance and reference RGB using the existing `PlaneSweepPatchMatchColorSigma` (0.15 by default). Plane fitting uses inverse depth versus undistorted camera-ray coordinates, so it accommodates tilted surfaces and lens distortion. Three Huber-weighted least-squares fits suppress local outliers. Opposite half-window fits check orientation and center-depth agreement; incompatible orientations over 35 degrees, excessive residuals, and one-sided/narrow support skip surface refinement. These are initial implementation constants, not guarantees of detail preservation.

Correction still examines all configured comparison cameras and selects the lowest normalized geometric-error passing discrete sample per camera. A direct cross-view corrective depth additionally requires a valid interpolation footprint at the actual projection, without holes or a footprint depth range exceeding half the relative depth tolerance. A view with a valid nearby match but no safe projected footprint can validate a locally proposed correction but cannot supply an interpolated corrective depth. Thus local regularization does not require four pristine interpolation footprints when the surviving neighborhood independently supports a safe plane.

Separated multi-observation depth clusters remain ambiguous and are left unchanged. Direct depth consensus uses median/MAD, geometric weights and the original estimate as an anchor. With a valid local plane it may use two clean interpolated source measurements; without one it still requires `max(2, PlaneSweepCrossViewMinimumViews)`. The configured minimum count of supporting cameras is always required during geometric validation, independently of interpolation availability.

Candidates include 70% and 35% steps toward the local plane, the prior half-step consensus proposal, and a blended plane/consensus proposal when both exist. Total movement remains bounded to half the configured relative depth tolerance and stays on the original reference-camera ray. All original supporting cameras must remain geometrically valid, with the existing hard view/depth/pixel limits unchanged. Acceptance minimizes a combined objective: mean paired geometric score, a 0.25-weight original-depth anchor, and a locally confidence-weighted surface term up to weight 0.5. Surface residuals use the robust fit noise with a floor of one tenth of the depth tolerance. With a safe surface model only, each accepted move must also reduce its local-plane residual; its geometric score may increase by at most 0.1 normalized units if the combined objective improves. Geometric score alone is no longer required to decrease on every moved point. Otherwise the original point is retained. Rejected points are never rescued and no output points are added across holes.

Input maps remain immutable throughout, so there is no feedback between corrected references. Count, order, reference indices and RGBA of filtered candidates are preserved. Raw and pyramid `.bin` files remain the original, uncorrected predictions; this pass changes final point positions only. Optional boundary RGB selection and voxel/outlier filtering still run afterward if separately enabled.

Startup prints `Cross-view depth correction: corrected ...`, insufficient support, ambiguous support, unchanged points, mean/maximum relative movement, and paired agreement scores for moved points. A second `Surface refinement:` line separates surface-supported/regularized points, consensus-only moves, missing neighborhoods, edge/crease skips and rejected proposals. It also reports combined objectives on all moved points and relative local-surface residuals on surface-regularized points. Geometric score alone can rise slightly because acceptance now balances surface coherence against geometric residuals. These are diagnostics, not ground-truth accuracy measures. The pass adds CPU fitting and validation work without new shader dispatches or all-map copies. Sparse legacy point grids may lack connected 5x5 support and fall back to the direct cross-view path.

The prior `PlaneSweepCrossViewCorrectionTests` cover the original direct-consensus path, analytic planes, invalid support, disabled behavior and ordering. They do not establish coverage or accuracy of the new neighborhood regularization; this requires separate validation of depth accuracy, detail preservation and runtime.

For an offline comparison without re-estimating depth, build `PlaneSweepCrossViewCompare` and run:

```text
PlaneSweepCrossViewCompare setup.ini RAW_DEPTH_FOLDER NEW_OUTPUT_FOLDER 6
```

The runner selects six evenly spaced reference views from the configured reference sequence and loads their nearest comparison maps. It uses the same saved depths, images, masks and supplied camera calibration for both arms. It produces `filter_only.ply`, `corrected.ply`, `point_movements.csv`, `inputs.csv`, `source_setup.ini` and `summary.txt`, including the separate surface/consensus counters and combined objective. Voxel fusion, optional color repair and display differences are excluded. The output folder must be new; original files are never overwritten. Saved maps must match the active image-scale dimensions. Old map files have no original settings/calibration manifest, so the runner records provenance and timestamps but cannot independently establish the historical generating configuration. Use a known single-run input set for an authoritative comparison.

`PlaneSweepSlantedTests` checks mode parsing, invalid settings, analytically rendered slanted geometry, rotated cameras, lens distortion, depth boundaries, masks, transparent sources, weighted texture rejection, local-depth confidence and deterministic CPU refinement. Regression cases also cover coarse-level refinement work, inheritance without sweep seeds, odd parent dimensions and provisional geometry retention before final rejection. `--gpu` additionally checks GPU geometry, coarse-level refinement work and CPU/GPU agreement. Floating-point ties can produce small differences in selected refinement hypotheses; the GPU test checks both depth agreement and error against the analytic surface.

The existing `PlaneSweepPyramidTests` and `PlaneSweepPatchCostTests` cover the original algorithm. CMake registers CPU and GPU tests with CTest. New shaders are compiled by both the CMake shader target and startup shader compilation, including shared-include dependencies.

For a three-arm scan9 comparison, run `PlaneSweepTiltBenchmark setup.ini NEW_OUTPUT_FOLDER 48 modes`, then `python Tests/analyze_matching_modes.py NEW_OUTPUT_FOLDER`. This compares normal `PlaneSweep`, weighted front-facing `PatchMatchSlanted`, and tilted `PatchMatchSlanted` on the same selected views and sources. It saves raw depth, matching cost and accepted plane tilt maps before cross-view filtering or fusion. The analysis records acceptance transitions, paired sparse-reference errors and paired cross-view checks; a `report.txt`, JSON results and plots are saved beside the depth maps. The front-facing/slanted pair isolates tilt, while the normal/slanted comparison includes weighting, propagation and confidence differences. These are geometric proxies when dense ground truth is unavailable.
