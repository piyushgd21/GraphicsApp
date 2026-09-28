# GraphicsApp Context

This file stores compressed project context so future work can resume with fewer tokens. Update it periodically when architecture, build steps, or user preferences change.

## Workspace

- Work only inside `D:\Documents\GraphicsApp`.
- Project targets Windows 11, C++, Visual Studio full, Vulkan, GLSL.
- Main solution: `GraphicsApp.sln`.
- Core library: `GraphicsFXCore`.
- App project: `GraphicsApp00`.
- Central folders: `Asset`, `Shaders`, `Output/Shaders`, `Output`, `GraphicsFXCore`, `GraphicsApp00/setup`.

## User Preferences

- Always plan before implementing.
- Verify code changes every time; no compilation warnings or errors.
- Clean temporary build verify folders.
- Use `.h` files, not `.hpp`.
- Use Sujal header block on source/header files.
- Add short one-line comments for classes, functions, and members.
- Use `m_`, `g_`, `s_`; pointer suffix `Ptr` or `PtrGPU`.
- No-argument functions use `(void)`.
- In classes, indent `public`, `protected`, and `private` one level and leave one blank line before/after.
- Class function names start uppercase.
- Use `Float3 &name` style, with `&` and `*` close to variable name.
- Keep code readable. Avoid needless tiny helper functions.
- Keep lines <= 200 characters. Split only when too long.
- Preferred class layout: data structures/classes first, then data, public functions, private functions.

## Build Notes

- Runtime shader compilation is enabled and writes `.spv` files to `Output/Shaders`.
- Aggregate `.sln` MSBuild from Codex shell can hit a `Path`/`PATH` environment issue.
- Reliable CMake build:

```powershell
& 'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --build .\build\cmake --config Debug --target GraphicsApp00 --parallel -- /p:TreatWarningAsError=true
& 'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --build .\build\cmake --config Release --target GraphicsApp00 --parallel -- /p:TreatWarningAsError=true
```

- Reliable direct Visual Studio project verification:

```powershell
& 'C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe' .\GraphicsFXCore\GraphicsFXCore.vcxproj /p:Configuration=Debug /p:Platform=x64 /p:TreatWarningAsError=true /p:SolutionDir=D:\Documents\GraphicsApp\ /m /v:minimal
& 'C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe' .\GraphicsApp00\GraphicsApp00.vcxproj /p:Configuration=Debug /p:Platform=x64 /p:TreatWarningAsError=true /p:SolutionDir=D:\Documents\GraphicsApp\ /m /v:minimal
```

Repeat with `Release`.

## Current Architecture

- `GraphicsFXCore` contains reusable core systems: graphics, UI, input, window, setup, scene objects, math, texture, shader, OpenXR, and multiview utilities.
- `GraphicsApp00` contains app-specific scene code.
- Namespace `graphicsFX` is used in core. App code avoids wrapping itself in that namespace.
- `SceneObject` owns common GPU resources, vertex/index/instance buffers, textures, render options, and pipelines.
- `SceneObjectRenderOptions` includes shadow, wireframe, clickable, selection, alpha blending, depth test, point size, and max point size behavior.

## Gaussian Splat Implementation

Relevant files:

- `GraphicsFXCore/src/GaussianSplatUtility.h`
- `GraphicsFXCore/src/GaussianSplatUtility.cpp`
- `GraphicsFXCore/src/SceneObjectGaussianSplat.h`
- `Shaders/gaussianSplat.vert`
- `Shaders/gaussianSplat.frag`
- `GraphicsApp00/src/Scene.cpp`
- `GraphicsApp00/setup/setup.ini`

### Loading

- `GaussianSplatUtility::LoadFolder(...)` finds PLY files from `[GaussianSplat]`.
- `GaussianSplatFileUtility` owns folder/multiple-file handling; its nested `PLYFileUtility` owns single-PLY opening and parsing.
- Supports ASCII and binary little-endian PLY.
- Binary little-endian PLY loading uses a header-driven fast path that reads the whole vertex block once, then decodes only the fields stored in `GaussianSplatData`.
- `StoreNormals`, `StoreSphericalHarmonics`, and `ColorMode` are parsed from setup; optional normal and `f_rest_*` data is stored only when requested.
- Reads:
  - position: `x`, `y`, `z`
  - color: `f_dc_0..2` or `red/green/blue`
  - opacity
  - scale: `scale_0..2`
  - rotation: `rot_0..3`
- Standard 3DGS scale values are log scale and are converted with:

```cpp
realScale = exp(scale_i) * SplatScale;
```

- Opacity uses sigmoid and `OpacityScale`.
- `CoordinateMode = ColmapToApp` converts positions as `(x, y, z) -> (x, -y, -z)`.
- Rotation is also converted for `ColmapToApp` by pre-multiplying the loaded quaternion by a 180-degree X quaternion `{ 1, 0, 0, 0 }` in `x, y, z, w` form.

### Rendering

- `Scene::DrawGaussianSplats(...)` creates `SceneObjectGaussianSplat`.
- Each splat becomes one point vertex.
- Vertex data stores position, color/opacity, scale, and rotation.
- Pipeline uses point-list topology, alpha blending, optional depth test, and depth write off by default.
- `SceneObjectGaussianSplat::UpdateSort(...)` sorts splats back-to-front from `m_eye.Position()` when enabled.
- Sorting only updates the index buffer order, not vertex data.

### Shader Behavior

- `gaussianSplat.vert` builds local covariance from scale and rotation:

```glsl
localCovariance = rotation * scaleSquared * transpose(rotation);
```

- It transforms covariance to view space and projects it to screen space.
- It computes point sprite size with:

```glsl
unclampedPointSize = max(pc.objectSettings.y, 8.0 * sqrt(maximumEigenvalue));
maximumPointSize = max(pc.objectSettings.x, pc.objectSettings.y);
pointSize = clamp(unclampedPointSize, 1.0, maximumPointSize);
```

- If point size is clamped, covariance is scaled down so edges fade instead of forming square blocks.
- A near-camera guard suppresses splats too close to or behind the camera.
- `gaussianSplat.frag` uses `gl_PointCoord`, evaluates the 2D Gaussian, discards low-alpha fragments, and outputs alpha-blended color.

## Gaussian Splat Setup

Current important `[GaussianSplat]` entries:

```ini
Enabled = true
Folder = E:\DocumentsExtra\Assets\Cheri_Take3_Full\OutputPly
FilePattern = *.ply
LoadMode = First
CoordinateMode = ColmapToApp
BaseTranslation = 4.0, 2.25, -11.0
BaseRotationDegrees = 173.23, 55.92, -175.03
BaseScale = 1.0
PositionScale = 1.0
DisplayEveryNthSplat = 1
MaxSplats = 0
StreamFps = 30.0
StreamLoop = true
StreamPreloadCount = 2
StreamWorkerThreadCount = 1
StreamWaitForFrame = false
StreamTimingLog = false
StreamTimingLogIntervalSeconds = 1.0
StreamTimingLogEveryFrame = false
PointSize = 3.0
MaxPointSize = 1024.0
SplatScale = 1.0
OpacityScale = 1.0
SortEnabled = true
SortEveryFrame = true
DepthTest = true
DepthWrite = false
Clickable = true
```

### Streaming

- `LoadMode = Stream` treats matching PLY files as ordered animation frames.
- `StreamFps` controls how quickly the stream advances.
- `StreamLoop = true` wraps to the first file after the last file; `false` leaves the last frame displayed and stops advancing.
- `StreamPreloadCount` controls how many upcoming frames the background loader requests.
- `StreamWaitForFrame = false` avoids render-thread waiting and leaves the current frame visible if the next decoded frame is not ready.
- `StreamTimingLog = true` prints focused Gaussian stream timing summaries for load/decode, queue wait, render-thread apply/upload, sort, and missed frames.
- `GaussianSplatStreamLoader` uses C++17 `std::thread`, `std::mutex`, `std::condition_variable`, and `std::atomic_bool` to decode PLY files in the background. `StreamWorkerThreadCount` selects one to four parallel decode threads, while frame application stays ordered on the render thread.
- Streaming keeps one `SceneObjectGaussianSplat` alive and updates its splat mesh data per frame.
- Vulkan vertex/index buffer upload still happens on the render thread through `SceneObjectGaussianSplat::UpdateSplats(...)`.
- Replaced vertex/index buffers are retired for a few update frames inside `SceneObject` before Vulkan destruction, avoiding in-flight buffer lifetime hazards.

## Recent Gaussian Fixes

- Changed `SplatScale` default/setup from `100.0` to `1.0`.
- Added coordinate conversion for Gaussian rotations.
- Added `MaxPointSize` setup value.
- Enabled Vulkan `largePoints` when supported.
- Passed GPU point-size range through `GraphicsUtility -> EngineCore -> SceneObject`.
- Removed fixed `128.0` shader point-size cap.
- Added covariance scaling when point size is clamped.
- Added near-camera guard in the vertex shader.

## If Gaussian Splats Still Need Tuning

- If close-up still looks particle-like, increase `MaxPointSize`, but it is clamped to GPU max point size.
- Try `SplatScale` around `1.2` to `2.0` for global size tuning.
- If point sprite limits remain a problem, move from Vulkan point sprites to instanced camera-facing quads or tile-based splat rendering.
- For transparency artifacts, consider weighted blended OIT or a better sorting strategy.

## COLMAP And Plane Sweep

- `MultiViewUtility` handles COLMAP camera parsing, sparse points, metric depth unprojection, and CPU plane sweep.
- `PointDisplayMode = Auto` prefers `points3D.txt` and falls back to metric depth unprojection.
- `PointDisplayMode = Points3D`, `Depth`, `PlaneSweep`, or `Off` selects explicit behavior.
- `PointDisplayMode = PlaneSweep` calls `MultiViewUtility::BuildColmapPlaneSweepPointCloud(...)`.
- Plane sweep uses COLMAP intrinsics/extrinsics, images from `CameraImageFolder`, and optional sparse `points3D.txt` tracks.
- Plane sweep sampling settings include `PlaneSweepReferenceStep`, `PlaneSweepSourceCount`, `PlaneSweepPlaneCount`, `PlaneSweepImageScale`, and `PlaneSweepPatchRadius`.
- Plane sweep acceptance settings include `PlaneSweepMinValidSourceViews`, `PlaneSweepCostThreshold`, `PlaneSweepAlphaThreshold`, `PlaneSweepMinimumOpaquePatchRatio`, and `PlaneSweepDepthRangeMode`.
- Plane sweep skips transparent reference pixels and source patch samples; fully transparent images are skipped during loading.
- `CameraMaskFolder` optionally replaces image alpha using masks matched by filename stem; RGBA masks use alpha, while opaque grayscale/RGB masks use luminance.
- Plane sweep can independently enable confidence rejection, texture rejection, cross-view consistency, voxel fusion, and spatial outlier filtering through `[COLMAP]` settings.
- Plane sweep output settings include `PlaneSweepPointStep`, `PlaneSweepOutputFolder`, and `PlaneSweepSaveDepth`.
- Source cameras are chosen by shared sparse tracks when available, with nearest/similar-view fallback.
- `PlaneSweepDepthRangeMode = SparsePoints` estimates per-reference near/far by sparse point depth percentiles; `Manual` uses `PlaneSweepNearDepth` and `PlaneSweepFarDepth`.
- Depth previews and float depth files are saved under `Output/PlaneSweep` when `PlaneSweepSaveDepth = true`.

## Last Known Verification

- `gaussianSplat.vert` and `gaussianSplat.frag` compiled successfully to `Output/Shaders`.
- CMake Debug and Release builds passed with warnings as errors.
- `StreamWorkerThreadCount` parallel decode worker support passed CMake Debug and Release builds with warnings as errors.
- CPU plane sweep implementation passed CMake Debug and Release builds with warnings as errors.
- Direct Visual Studio project verification from Codex was blocked by the sandbox `PATH`/`Path` MSBuild issue and the workspace spend cap for escalation.
- Direct Visual Studio Debug and Release builds passed for `GraphicsFXCore` and `GraphicsApp00`.
- No temporary `BuildVerify` or `BinVerify` folders remained.
