cmake_minimum_required(VERSION 3.20)

project(GraphicsApp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Vulkan REQUIRED)

set(OPENXR_LOADER_DLL
    "${CMAKE_CURRENT_SOURCE_DIR}/packages/OpenXR.Loader.1.0.10.2/native/x64/release/bin/openxr_loader.dll"
)

add_library(GraphicsFXCore STATIC
    GraphicsFXCore/src/Camera.cpp
    GraphicsFXCore/src/Camera.h
    GraphicsFXCore/src/EngineCore.cpp
    GraphicsFXCore/src/EngineCore.h
    GraphicsFXCore/src/GaussianSplatStreamLoader.cpp
    GraphicsFXCore/src/GaussianSplatStreamLoader.h
    GraphicsFXCore/src/GaussianSplatUtility.cpp
    GraphicsFXCore/src/GaussianSplatUtility.h
    GraphicsFXCore/src/GraphicsUtility.cpp
    GraphicsFXCore/src/PlaneSweepGpu.cpp
    GraphicsFXCore/src/GraphicsUtility.h
    GraphicsFXCore/src/global.h
    GraphicsFXCore/src/InputUtility.cpp
    GraphicsFXCore/src/InputUtility.h
    GraphicsFXCore/src/MathUtility.h
    GraphicsFXCore/src/MultiViewUtility.cpp
    GraphicsFXCore/src/MultiViewUtility.h
    GraphicsFXCore/src/OpenXRUtility.cpp
    GraphicsFXCore/src/OpenXRUtility.h
    GraphicsFXCore/src/PathUtility.cpp
    GraphicsFXCore/src/PathUtility.h
    GraphicsFXCore/src/RenderTypes.h
    GraphicsFXCore/src/SceneInterface.h
    GraphicsFXCore/src/SceneObject.cpp
    GraphicsFXCore/src/SceneObject.h
    GraphicsFXCore/src/SceneObjectAxis.h
    GraphicsFXCore/src/SceneObjectBox.h
    GraphicsFXCore/src/SceneObjectGrid.h
    GraphicsFXCore/src/SceneObjectGridPlane.h
    GraphicsFXCore/src/SceneObjectGaussianSplat.h
    GraphicsFXCore/src/SceneObjectPointCloud.h
    GraphicsFXCore/src/SceneObjectPyramid.h
    GraphicsFXCore/src/SceneObjectSphere.h
    GraphicsFXCore/src/SetupUtility.cpp
    GraphicsFXCore/src/SetupUtility.h
    GraphicsFXCore/src/ShaderUtility.cpp
    GraphicsFXCore/src/ShaderUtility.h
    GraphicsFXCore/src/StringUtility.cpp
    GraphicsFXCore/src/StringUtility.h
    GraphicsFXCore/src/TextureUtility.cpp
    GraphicsFXCore/src/TextureUtility.h
    GraphicsFXCore/src/UIUtility.cpp
    GraphicsFXCore/src/UIUtility.h
    GraphicsFXCore/src/WindowsUtility.cpp
    GraphicsFXCore/src/WindowsUtility.h
    GraphicsFXCore/src/Window.cpp
    GraphicsFXCore/src/Window.h
)

target_compile_definitions(GraphicsFXCore PUBLIC
    UNICODE
    _UNICODE
)

target_include_directories(GraphicsFXCore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/GraphicsFXCore/src
    ${CMAKE_CURRENT_SOURCE_DIR}/GraphicsFXCore/include
    ${CMAKE_CURRENT_SOURCE_DIR}/packages/OpenXR.Headers.1.0.10.2/include
)

target_link_libraries(GraphicsFXCore PUBLIC
    Vulkan::Vulkan
    user32
    windowscodecs
)

add_executable(GraphicsApp00 WIN32
    GraphicsApp00/src/WinMain.cpp
    GraphicsApp00/src/Scene.cpp
    GraphicsApp00/src/Scene.h
)

target_include_directories(GraphicsApp00 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/GraphicsApp00/src
)

target_link_libraries(GraphicsApp00 PRIVATE
    GraphicsFXCore
)

add_custom_command(TARGET GraphicsApp00 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OPENXR_LOADER_DLL}"
        "$<TARGET_FILE_DIR:GraphicsApp00>/openxr_loader.dll"
    COMMENT "Copying OpenXR loader beside the executable"
    VERBATIM
)

include(CTest)
if(BUILD_TESTING)
    add_executable(PlaneSweepPatchCostTests
        Tests/PlaneSweepPatchCostTests.cpp
    )
    target_link_libraries(PlaneSweepPatchCostTests PRIVATE GraphicsFXCore)
    if(MSVC)
        target_compile_options(PlaneSweepPatchCostTests PRIVATE /W4 /WX)
    endif()
    add_test(
        NAME PlaneSweepPatchCosts
        COMMAND PlaneSweepPatchCostTests
            "${CMAKE_CURRENT_BINARY_DIR}/Testing/PatchCosts"
    )

    if(Vulkan_GLSLC_EXECUTABLE)
        add_custom_target(PlaneSweepPatchCostShader
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:PlaneSweepPatchCostTests>/shaders"
            COMMAND "${Vulkan_GLSLC_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/Shaders/planeSweepDepthMapSmokeTest.comp"
                -o "$<TARGET_FILE_DIR:PlaneSweepPatchCostTests>/shaders/planeSweepDepthMapSmokeTest.comp.spv"
            COMMAND "${Vulkan_GLSLC_EXECUTABLE}"
                -DGRAPHICSAPP_PLANE_SWEEP_FP64=1
                "${CMAKE_CURRENT_SOURCE_DIR}/Shaders/planeSweepDepthMapSmokeTest.comp"
                -o "$<TARGET_FILE_DIR:PlaneSweepPatchCostTests>/shaders/planeSweepDepthMapFp64.comp.spv"
            DEPENDS
                "${CMAKE_CURRENT_SOURCE_DIR}/Shaders/planeSweepDepthMapSmokeTest.comp"
            COMMENT "Compiling plane sweep shader for GPU regression tests"
            VERBATIM
        )
        add_dependencies(PlaneSweepPatchCostTests PlaneSweepPatchCostShader)

        add_test(
            NAME PlaneSweepGpuPatchCosts
            COMMAND PlaneSweepPatchCostTests
                "${CMAKE_CURRENT_BINARY_DIR}/Testing/GpuPatchCosts" --gpu
        )
    endif()
endif()
