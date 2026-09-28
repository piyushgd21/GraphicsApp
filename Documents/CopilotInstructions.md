# Copilot Instructions

Use these preferences when working in this repository.

## Scope

- Work only inside this repository folder unless explicitly told otherwise.
- Always verify the plan before implementing code or project structure changes.
- Keep the program simple to understand.
- Prefer `std::string` instead of `std::wstring` when possible.

## Project

- Target Windows 11.
- Use C++ in Visual Studio full IDE, not Visual Studio Code.
- Use Vulkan for graphics.
- Use GLSL for shaders.
- Use standard OpenXR first for XR support, and only add Meta-specific extensions when the base OpenXR path cannot provide the needed feature.
- Keep both CMake support and the Visual Studio `.sln` project working.
- Keep reusable engine code in `GraphicsFXCore`.
- Put reusable `GraphicsFXCore` code in the `graphicsFX` namespace.
- Keep application projects such as `GraphicsApp00` outside a namespace so they can be copied into future apps more easily.
- Keep app-specific entry points, scenes, setup files, and optional assets under app folders such as `GraphicsApp00`.
- Keep the Visual Studio working directory set to the solution/repository folder so runtime paths resolve from the base folder.
- Keep reusable assets, shaders, setup references, and future model folders centralized from the solution folder where practical.

## Files

- Use `.h` for headers, not `.hpp`.
- Use descriptive class names such as `SceneObjectBox`, `SceneObjectSphere`, `SceneObjectGrid`, `SceneObjectAxis`, `SceneObjectPyramid`, and `SceneObjectPointCloud`.
- Keep short derived object functions in headers when they are simple.
- Put app setup files under each app's `setup` folder.
- Put shared shaders and shared assets under the repository root `Shaders` and `Asset` folders.
- Use `TextureFolder = Asset`, `ModelFolder = Asset`, `ShaderSourceFolder = Shaders`, `ShaderOutputFolder = Output/Shaders`, and `CaptureOutputFolder = Output` as the preferred central path settings.
- Compile shaders at application startup when enabled in `setup.ini`; keep build-time shader compilation disabled by default.
- Put planning and repository guidance documents under the `Documents` folder.
- Use `Documents/plans.md` as the main project plan file.
- Use spaced assignments in setup files, such as `FovDegrees = 45.0`.
- Use `#` for comments in setup files.
- Align setup comments with tabs when comments are useful.
- Put available string options in comments for non-numeric settings, such as `PointDisplayMode = Auto			# [Auto, Points3D, Depth, PlaneSweep, Off]`.
- Avoid large option comments for keyboard and mouse mappings because they make setup files noisy.

## Naming

- Prefix member variables with `m_`.
- Prefix global variables with `g_`.
- Prefix static variables with `s_`.
- Add `Ptr` as a suffix for CPU pointer variables.
- Add `PtrGPU` as a suffix for GPU handles and GPU pointer-style variables.
- Start class member functions with an uppercase letter.
- Use `void` in the parameter list when a function takes no input, such as `Init(void)`.
- Put `*` and `&` next to the variable name, such as `Float3 &name`.

## Formatting

- Indent class access sections one level inside the class.
- Add one empty line before and after `public:`, `protected:`, and `private:`.
- For full classes, order declarations as nested types first, public functions next, private or protected helper functions next, and member data last.
- For small data-like structs, place member data first and helper functions after the data.
- In `.cpp` files, keep function definitions in a similar public-to-private order when practical.
- Keep assignments, function calls, and declarations on one line when the resulting line is 200 characters or less.
- When a function call or declaration would be longer than 200 characters, keep the call or declaration name on the first line and put each argument on its own indented line.
- Break other code into multiple lines only when the joined line would be longer than 200 characters or joining would make syntax/readability worse.
- Avoid anonymous namespaces in `.cpp` files when they hide important behavior.
- Move reusable helpers into utility files and keep file-specific helpers on a named private helper class or private class/static functions.
- Use this class layout style:

```cpp
class SceneObjectSphere final : public SceneObject
{

    public:

        SceneObjectSphere(void);
};
```

## Comments

- Add the standard file header to every `.h` and `.cpp` file:

```cpp
///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	FileName.h
//	Description	:	
//		Defines the ...
//
//
///////////////////////////////////////////////////////////////////////
```

- Write a one-line comment before each class and struct describing what it does.
- Write a one-line comment before each function describing what it does.
- Add a short one-line comment after each class member variable, such as `int m_count; //Counts number of vertices.`
- For functions declared in `.h` and defined in `.cpp`, use the same comment in both places.

## Math And Types

- Use `Float2`, `Float3`, `Float4`, `Float3x3`, and `Float4x4` names for math types.
- Use `Float3` for position and direction values.
- Use `Float4` for colors.
- Use 32-bit indices by default.
- Allow switching between 16-bit and 32-bit indices when useful.
- Keep index validation helper logic in `SceneObject`.

## Scene Objects

- Use `SceneObject` as the base class for shared mesh, shader, GPU buffer, pipeline, update, draw, and cleanup behavior.
- Derived scene objects should create their mesh data and pass it to the base `SceneObject`.
- Include common functions such as `Init`, `Update`, `Show`, and `Clean`.
- Pass shader file names into scene object `Init` functions instead of hard-coding them inside object constructors.
- Allow scene objects to control whether they cast shadows, receive shadows, and draw as wireframe.
- Use `SceneObjectBox` for boxes and allow separate X, Y, and Z size values.
- Use `SceneObjectPyramid` for camera and light markers, with configurable base size and height.
- Use instancing for repeated objects when possible.
- Keep scene-related behavior in `Scene`.
- Keep GPU/device-related behavior in `GraphicsUtility`.
- Keep XR runtime and session behavior in `OpenXRUtility`.
- Keep input-related behavior in `InputUtility`.
- Keep COLMAP parsing and point cloud building in `GraphicsFXCore`, not in scene-specific code.
- Arrange example scene setup from simple to complex, such as boxes, spheres, textured objects, instanced objects, then data-driven objects.
- Keep world examples intentionally positioned so they teach the scene layout instead of looking random.

## Camera And Coordinates

- Use a right-handed coordinate system.
- +X points to the side.
- +Y points up.
- +Z points out of the screen.
- The default camera position is `0, 0, 5`.
- The default `LookAt`, `Up`, and `Side` values are directions, not positions.
- The default camera looks along `0, 0, -1`.
- Use `m_eye` as the default world camera.
- Use a smaller HUD axis in the lower-right screen area instead of a large world axis when the scene needs camera-friendly orientation help.
- In XR mode, use the desktop camera as the shared rig parent and apply headset eye poses as local child transforms.
- Use `MirrorMode = CenterEye` when the desktop window should mirror the XR center-eye pose.
- When `MirrorMode = CenterEye`, align keyboard and mouse movement with the visible center-eye camera directions.
- Keep XR controller input in `OpenXRUtility`, pass it through `ControllerMovementState`, and let `Camera` consume the general movement state.
- Keep XR passthrough optional and non-fatal; if `XR_FB_passthrough` is unavailable, disable passthrough and continue normal XR rendering.
- Do not use `XR_KHR_composition_layer_depth` as real passthrough environment depth.
- It does not depth-test the camera passthrough image against scene depth; use `XR_META_environment_depth` for Quest passthrough environment depth.
- For Quest passthrough occlusion, bind the acquired `XR_META_environment_depth` image to the scene shader and discard virtual fragments when environment depth is closer.
- Keep passthrough depth support optional because Quest runtime behavior can be unstable; prefer disabling it cleanly over breaking regular XR rendering.

## COLMAP And Depth Data

- Support both COLMAP sparse `points3D.txt` display and metric depth unprojection.
- Use `PointDisplayMode = Auto` to prefer `points3D.txt` when available and fall back to depth unprojection.
- Use `PointDisplayMode = Points3D`, `Depth`, `PlaneSweep`, or `Off` for explicit behavior.
- Use `DisplayEveryNthCamera`, `DisplayEveryNthPoint`, and `DisplayEveryNthPixel` to keep heavy research datasets interactive.
- Support COLMAP `OPENCV`, `SIMPLE_RADIAL`, and `RADIAL` camera models.
- Treat `BaseTranslation` and `BaseRotationDegrees` as visualization transforms, not edits to the original COLMAP camera data.
- Use `CameraImageFolder`, `CameraDepthFolder`, and `CameraPointsFile` for data paths.
- Metric depth may need conversion to COLMAP scale. Use `DepthScaleMode = Manual` with `DepthScaleValueMetricToColmap` for direct control.
- Use `DepthScaleMode = AutoGlobalFromPoints3D` to estimate one global metric-to-COLMAP depth scale from sparse COLMAP points and metric depth images.
- Use `DepthScaleMode = AutoPerImageFromPoints3D` when metric-depth scale varies per image.
- Use `DepthScaleMode = AutoHybridFromPoints3D` when per-image estimates should be used when available with global/manual fallback.
- Use `DepthValueMode = CameraZ` when depth values are along the camera Z axis.
- Use `DepthValueMode = RayLength` when depth values are distance along the pixel ray.

## UI And Debug Output

- Keep UI drawing owned by `UIUtility` once a popup is shown; use callbacks for button presses.
- Use white text by default in UI windows, but keep it configurable in `setup.ini`.
- Use a glassy blue-tinted panel style by default and keep UI colors configurable in `setup.ini`.
- Allow popup windows to move by dragging the title area.
- Make HUD text size configurable in `setup.ini`.
- Display FPS, camera position, LookAt, Up, and Right together on one HUD line near the bottom of the screen.
- Use `UIUtility::Print(std::string data)` for Visual Studio debug output.
- Do not add newlines inside `UIUtility::Print`; callers decide whether to pass text ending in `"\n"`.

## Build And Verification

- Every time code is changed, check that there are no compilation warnings or errors.
- Build Debug and Release when practical.
- Treat warnings as errors.
- Do not leave temporary verification folders such as `build_verify*`, `bin_verify*`, or `manual_verify` in the repository.
- Keep memory leak checking code in the program, but do not run the leak check unless specifically requested because it takes time.
- When adding capture/debug features, prefer key-triggered behavior instead of running capture work during initialization.

## Input Defaults

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
- Press `/` to print camera and axis information and save frame buffers when frame capture is enabled.
- Press `.` to toggle wireframe mode when the device supports wireframe rendering.
- Press `Esc` to close the application.

## Frame Capture

- Use `Output` under the repository base folder for captures.
- Save color, depth, and normal capture images with the displayed resolution.
- Keep capture requests owned by `Scene::UpdateInput`, then let `EngineCore` consume the request and save the GPU buffers.

## Shaders And Rendering

- Use `default.vert` and `default.frag` as the default shader names.
- Let users specify shader source paths such as `shaders/default.vert`; internally add `.spv` for compiled shader output where needed.
- Support startup shader compilation through the configured shader source and output folders.
- Use lit shader variants for examples that should demonstrate lighting, normal mapping, shadows, or textured materials.
- Use a single 2048 shadow map unless a feature request asks for something more advanced.
- Use hard shadows or simple 3 x 3 PCF before adding more complex shadow algorithms.
- Allow grid drawing as either line grid or checkerboard plane; prefer checkerboard plane by default when shadows should be visible.

