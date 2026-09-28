# Graphics Application Plan

## Goal

Maintain a Windows 11 desktop graphics application written in C++ for Visual Studio Full. The application uses Vulkan for rendering, GLSL shaders compiled to SPIR-V, and a simple scene system that displays cubes, spheres, grid lines, axes, textures, camera movement, and in-graphics UI.

The solution is split into a reusable core static library and application projects. `GraphicsFXCore` contains reusable graphics, window, input, UI, texture, setup, camera, and scene-object code. `GraphicsApp00` contains the first application scene and entry point.

## Target Platform

- Operating system: Windows 11
- IDE: Visual Studio Full, not Visual Studio Code
- Main solution: `GraphicsApp.sln`
- Language: C++
- Graphics API: Vulkan
- Shader language: GLSL
- Optional build support: CMake

## Current Project Layout

```text
GraphicsApp/
  GraphicsApp.sln
  CMakeLists.txt
  Asset/
    checker_blue.png
    checker_red.png
    grid_yellow.png
    stripes_green.png
  Shaders/
    default.vert
    default.frag
    ui.vert
    ui.frag
  GraphicsFXCore/
    GraphicsFXCore.vcxproj
    src/
      Camera.h / Camera.cpp
      EngineCore.h / EngineCore.cpp
      GraphicsUtility.h / GraphicsUtility.cpp
      InputUtility.h / InputUtility.cpp
      MathUtility.h
      RenderTypes.h
      SceneInterface.h
      SceneObject.h / SceneObject.cpp
      SceneObjectAxis.h
      SceneObjectBox.h
      SceneObjectGrid.h
      SceneObjectSphere.h
      SetupUtility.h / SetupUtility.cpp
      TextureUtility.h / TextureUtility.cpp
      UIUtility.h / UIUtility.cpp
      Window.h / Window.cpp
  GraphicsApp00/
    GraphicsApp00.vcxproj
    src/
      WinMain.cpp
      Scene.h / Scene.cpp
    setup/
      setup.ini
    Asset/
      optional app-specific assets
  Documents/
    CopilotInstructions.md
    plans.md
```

## Main Architecture

- `GraphicsFXCore` builds `GraphicsFXCore.lib` for shared graphics code.
- `GraphicsApp00` builds the first executable application and links `GraphicsFXCore.lib`.
- `GraphicsFXCore` uses the `graphicsFX` namespace for reusable types and utilities.
- `GraphicsApp00` keeps app-specific scene and entry-point code outside a namespace so it is easier to copy into `GraphicsApp01` or later apps.
- `EngineCore` owns the window loop, swapchain resources, frame synchronization, descriptor sets, and render frame flow.
- `EngineCore` talks to scenes through `SceneInterface`, so future apps can provide different scenes without changing the core library.
- `GraphicsUtility` owns Vulkan instance, device, queues, command pool, image helpers, buffer helpers, and GPU utility behavior.
- `Scene` owns scene objects, the default camera `m_eye`, scene update behavior, and scene-level UI calls.
- `SceneObject` owns shared mesh, shader, texture, GPU buffer, pipeline, descriptor, update, show, and cleanup behavior.
- `SceneObjectBox`, `SceneObjectSphere`, `SceneObjectGrid`, and `SceneObjectAxis` create object mesh data and pass it to `SceneObject`.
- `InputUtility` owns keyboard and mouse state.
- `Camera` handles movement, pitch, yaw, roll, view matrix, and projection matrix.
- `TextureUtility` loads image files through Windows WIC and creates image data for textures.
- `UIUtility` draws text, HUD, popup windows, button states, and persistent UI windows.
- `SetupUtility` loads each app's `setup/setup.ini` values for window, graphics, camera, input, and UI.

## Rendering Plan

The current renderer should keep these features working:

1. Create a Win32 window.
2. Initialize Vulkan instance, validation layers in Debug, surface, physical device, logical device, queues, swapchain, image views, render pass, depth buffer, framebuffers, command buffers, and sync objects.
3. Compile GLSL shaders with `glslc.exe` at application startup when enabled in `setup.ini`.
4. Render colored cubes, colored spheres, textured cubes, grid lines, and axes.
5. Use `default.vert` and `default.frag` for scene objects.
6. Use `ui.vert` and `ui.frag` for UI geometry.
7. Draw FPS and camera vectors in the bottom HUD.
8. Draw persistent popup windows through `UIUtility`, with callbacks for button presses.

## XR Plan

OpenXR support should be PC-launched first. The program should use standard OpenXR before Meta-specific extensions, so Meta Quest 3 can work through the active PC OpenXR runtime when connected through Quest Link or Air Link.

Current XR setup values live in `setup/setup.ini`:

- `Enabled`: tries OpenXR startup when true.
- `Required`: exits on missing OpenXR support when true; otherwise falls back to desktop Vulkan.
- `MirrorToWindow`: keeps the PC window rendering while XR is active.
- `MirrorMode`: selects the PC mirror view. `CenterEye` mirrors the XR center eye.
- `ReferenceSpace`: stores the preferred XR reference space.
- `RenderScale`: stores XR render resolution scale.
- `UseDepthLayer`: stores whether XR passthrough environment depth should be requested.
- `ControllerInput`: stores whether OpenXR controller input should be used.
- `Passthrough`: enables the optional `XR_FB_passthrough` camera background when the runtime supports it.
- `PassthroughDepthTest`: requests real passthrough environment-depth support through `XR_META_environment_depth` when the runtime supports it.
- `PassthroughDepthMode`: selects the passthrough depth provider. `OpenXRMeta` uses `XR_META_environment_depth`.
- `PassthroughDepthDebugView`: selects the passthrough depth debug view. `Off` is the default.
- `PassthroughDepthBiasMeters`: offsets environment depth slightly to reduce edge flicker.
- `PassthroughDepthMaxMeters`: clamps how far passthrough depth occlusion is trusted.
- `PassthroughDepthHandRemoval`: requests Meta hand removal when the runtime supports it.

Default XR controller movement bindings:

- Left thumbstick: move forward, backward, left, and right.
- Right thumbstick: yaw and pitch.
- Right A button: move up.
- Right B button: move down.

Implementation stages:

1. Add `OpenXRUtility` in `GraphicsFXCore` for dynamic loader startup and HMD detection without requiring the OpenXR SDK to be installed.
2. Keep desktop Vulkan rendering as the stable path when XR is disabled or unavailable.
3. Create a first XR session path using the available OpenXR headers and `XR_KHR_vulkan_enable`, then submit a simple stereo clear layer to prove headset display.
4. Query `xrGetVulkanGraphicsRequirementsKHR`, `xrGetVulkanInstanceExtensionsKHR`, and `xrGetVulkanDeviceExtensionsKHR`, then pass those requirements into `GraphicsUtility`.
5. Create the Vulkan instance first, query `xrGetVulkanGraphicsDeviceKHR`, and use that OpenXR-required physical device when creating the logical Vulkan device.
6. Add the full OpenXR Vulkan session path using `XR_KHR_vulkan_enable2`, so OpenXR can choose a compatible Vulkan physical device when newer headers are available.
7. Add XR swapchains, per-eye render targets, `xrWaitFrame`, `xrBeginFrame`, `xrLocateViews`, and `xrEndFrame`.
8. Render the existing scene once per XR eye using the OpenXR pose and field of view; keep the blue/green clear path as a fallback diagnostic.
9. Use the desktop camera as the shared XR rig parent, then apply OpenXR eye poses as local child transforms.
10. When `MirrorMode = CenterEye`, update the desktop mirror camera from the averaged XR eye pose so headset movement is reflected in the PC window.
11. When `MirrorMode = CenterEye`, drive keyboard and mouse movement from the mirrored center-eye forward, right, and up directions.
12. Add OpenXR action input for Quest controllers through standard OpenXR interaction profiles before adding Meta-only extensions.
13. Keep controller input flowing through a general `ControllerMovementState`, so camera movement is not tied directly to OpenXR types.
14. Add optional `XR_FB_passthrough` support as a non-fatal feature; when enabled, clear XR scene alpha to transparent and submit passthrough behind the projection layer.
15. Keep `XR_KHR_composition_layer_depth` disabled for passthrough occlusion, because it does not provide real-world camera depth and can cause flicker when treated as environment depth.
16. Use `XR_META_environment_depth` on Quest runtimes when `PassthroughDepthTest = true`, acquire the per-frame environment depth image, and pass it through the XR eye render information for scene-depth comparison.
17. Bind the acquired environment depth image to the scene descriptor set and discard object fragments when the real-world depth is closer than the virtual fragment depth.

## Scene Objects

Scene objects should stay simple:

- Derived objects build vertex and index data.
- The base `SceneObject` creates and owns GPU resources.
- Use 32-bit indices by default.
- Allow switching between 16-bit and 32-bit indices when useful.
- Use `Float3` for positions and directions.
- Use `Float4` for colors.
- Keep object-specific creation helpers in headers when they are short and readable.

## Gaussian Splat Streaming Plan

Goal: allow a folder of ordered Gaussian splat PLY files to play back as a time-based sequence. For example, a folder with 100 PLY files should display one PLY at a time and advance at the configured Gaussian stream FPS.

Proposed setup values under `[GaussianSplat]`:

- `LoadMode = Stream` with comments updated to `[First, All, Stream]`.
- `StreamFps = 30.0` to control playback rate from `setup.ini`.
- `StreamLoop = true` to wrap from the last PLY back to the first.
- `StreamPreloadCount = 2` to control how many upcoming frames are decoded in advance.
- `StreamWorkerThreadCount = 1` to control how many background threads decode upcoming frames, capped at four.
- `StreamWaitForFrame = false` to keep the render thread moving when the next decoded frame is not ready.
- Optional later values only if needed: `StreamStartFrame` and `StreamCacheFrameCount`.

Implementation stages:

1. Extend `GaussianSplatSetup` and `SetupUtility` with `m_streamFps` and `m_streamLoop`, keeping conservative defaults so existing `First` and `All` modes behave exactly as they do now.
2. Refactor `GaussianSplatUtility` just enough to expose sorted file discovery and single-file loading. Keep `LoadFolder(...)` for current behavior, but let streaming ask for the ordered file list and load one frame by file path.
3. Add Gaussian stream state to `Scene`: sorted file list, active file index, playback accumulator, stream FPS, loop flag, shader files, current `SceneObjectContext`, and active object pointer.
4. In `Scene::Init`, if `LoadMode = Stream`, initialize the stream from the folder and load the first frame instead of calling the current whole-folder load path. If `LoadMode` is not `Stream`, keep the current `DrawGaussianSplats(...)` path.
5. In `Scene::Update`, advance the desired frame from elapsed time using `frameDuration = 1.0f / StreamFps`. If the decoded frame is ready, update the active `SceneObjectGaussianSplat` mesh data. If not ready, keep showing the current frame.
6. Keep one Gaussian scene object alive during streaming so texture descriptor sets and pipelines are not reallocated every frame.
7. Retire replaced vertex and index buffers inside `SceneObject` for a few update frames before destroying them. This avoids freeing buffers still referenced by in-flight command buffers.
8. Decode future PLY frames with `GaussianSplatStreamLoader`, using C++17 `std::thread`, `std::mutex`, `std::condition_variable`, and `std::atomic_bool`. `StreamWorkerThreadCount` starts one to four parallel decode workers while the render thread still applies frames in sequence.
9. Add setup options later if normal or spherical-harmonic storage becomes needed:

```ini
StoreNormals = false
StoreSphericalHarmonics = false
ColorMode = DCOnly # [DCOnly, SphericalHarmonics]
```

10. Keep sorting behavior unchanged: the active streamed object still uses `UpdateSort(m_eye.Position())`, and `SortEveryFrame` continues to control per-frame sorting cost.
11. Update `GraphicsApp00/setup/setup.ini` comments and defaults to show the new stream mode without forcing it on by default.

Implemented path:

- Synchronous first-frame loading, asynchronous CPU decode for upcoming stream frames, one persistent Gaussian scene object, render-thread Vulkan upload, and deferred GPU-buffer retirement.
- Binary little-endian PLY decoding now reads one contiguous vertex block and extracts only stored Gaussian fields.
- Optional `StoreNormals`, `StoreSphericalHarmonics`, and `ColorMode` setup values are implemented for loading normal and `f_rest_*` data.
- `StreamTimingLog` setup values are implemented for focused Gaussian stream timing summaries and optional every-frame timing details.
- If large PLY files still cause hitches, the next step should be frame caching or a dedicated Vulkan transfer upload path.

Verification after implementation:

1. Build Debug with warnings as errors.
2. Build Release with warnings as errors when practical.
3. Test `LoadMode = First` to confirm current behavior remains unchanged.
4. Test `LoadMode = All` to confirm multi-file aggregate loading still works.
5. Test `LoadMode = Stream` with a small folder and a low `StreamFps`, then with the intended dataset.
6. Confirm no temporary verification folders remain.

## Camera And Coordinates

Use a right-handed coordinate system:

- +X points to the side.
- +Y points up.
- +Z points out of the screen.
- The default camera is `m_eye`.
- Default camera position: `0, 0, 5`.
- Default `LookAt` direction: `0, 0, -1`.
- Default `Up` direction: `0, 1, 0`.
- Default `Side` direction: `1, 0, 0`.

`LookAt`, `Up`, and `Side` are directions, not target positions.

## Input

Input is loaded from `setup/setup.ini` and handled through `InputUtility`.

Default mappings:

- Forward: `Up`
- Back: `Down`
- Left: `Left`
- Right: `Right`
- MoveUp: `PageUp`
- MoveDown: `PageDown`
- Fast: `+`
- Slow: `-`
- PitchUp: `W`
- PitchDown: `S`
- YawLeft: `A`
- YawRight: `D`
- Mouse wheel moves forward and back.
- Middle mouse wheel press can move forward and back smoothly.

## Setup File

`setup/setup.ini` stores editable runtime defaults:

- Window title and size
- Clear color
- Grid and axis line widths
- Camera position, directions, FOV, speed, and sensitivity
- Keyboard mapping
- Mouse mapping
- HUD text size
- UI text, shadow, glass, header, accent, border, button, hover, and pressed colors

Keep spaces around assignment in setup values:

```ini
FovDegrees = 45.0
ButtonColor = 0.10, 0.30, 0.52, 0.78
```

## UI Plan

`UIUtility` should own persistent UI windows once they are shown.

Current behavior:

- `Scene::Init` calls `UIUtility::ShowPopup`.
- `UIUtility` stores the popup title, message, buttons, position, drag state, and callback.
- `EngineCore` calls `BeginFrame`, `UpdateInput`, and `DrawWindows` each frame.
- Button clicks call the stored callback.
- The startup popup callback can use `UIUtility::Print` to write button actions to the Visual Studio debug window.

Future UI additions should follow the same pattern: create UI state once, let `UIUtility` redraw and handle interaction, and use callbacks for actions.

## Texture Plan

Shared textures are stored in the repository `Asset/` folder and loaded through `TextureUtility`. Application-specific textures can be added under `GraphicsApp00/Asset/` or the matching app folder for future apps.

Supported image goal:

- BMP
- PNG
- JPG

PNG remains the default sample texture format. Texture loading should continue to use Windows WIC.

## Build Plan

Visual Studio is the primary build path:

- Build `GraphicsApp.sln`.
- `GraphicsFXCore` builds first as a static library.
- `GraphicsApp00` links against `GraphicsFXCore.lib`.
- Use x64.
- Keep Debug and Release configurations working.
- Treat warnings as errors during verification.
- Runtime shader compilation reads shared GLSL from the repository `Shaders/` folder and writes SPIR-V to `Output/Shaders/`.
- Runtime texture loading reads shared assets from the repository `Asset/` folder and app-specific assets when present.

CMake support remains secondary but should stay valid.

## Verification Plan

After code changes:

1. Build Debug with warnings as errors.
2. Build Release with warnings as errors when practical.
3. Confirm `0 Warning(s)` and `0 Error(s)`.
4. Confirm no temporary verification folders remain, such as `build_verify*`, `bin_verify*`, or `manual_verify`.
5. Keep memory leak checking code in the program, but do not run the leak check unless specifically requested.

For documentation-only changes, a full C++ build is optional unless requested.

## Documentation Plan

Planning and repository guidance documents should live under `Documents/`.

Primary documentation files:

- `Documents/plans.md`
- `Documents/CopilotInstructions.md`

Future planning updates should update `Documents/plans.md` unless a more specific document is requested.
