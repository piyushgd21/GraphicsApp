///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	ShaderUtility.cpp
//	Description	:
//		Implements runtime GLSL shader compilation helpers.
//
//
///////////////////////////////////////////////////////////////////////

#include "ShaderUtility.h"

#include "PathUtility.h"
#include "UIUtility.h"

#include <Windows.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <stdexcept>

namespace graphicsFX
{
    //Compiles startup shaders when enabled.
    void ShaderUtility::CompileStartupShaders(const ShaderSetup &shaderSetup)
    {
        if (!shaderSetup.m_compileAtStartup)
        {
            UIUtility::Print("Startup shader compilation is disabled.\n");
            return;
        }

        const std::filesystem::path sourceFolder = ResolveSourceFolder(shaderSetup.m_sourceFolder);
        const std::filesystem::path outputFolder = ResolveOutputFolder(shaderSetup.m_outputFolder);
        const std::filesystem::path compilerPath = ResolveCompilerPath(shaderSetup.m_compilerPath);
        const std::vector<std::filesystem::path> shaderFiles = ShaderSourceFiles(sourceFolder);

        if (shaderFiles.empty())
        {
            throw std::runtime_error("No GLSL shader source files were found.");
        }

        std::size_t compiledCount = 0;
        for (const std::filesystem::path &sourceFile : shaderFiles)
        {
            const std::filesystem::path outputFile =
                outputFolder / (sourceFile.filename().string() + ".spv");

#if !defined(GRAPHICSAPP_ENABLE_PHASE_2PB_GPU_TRACE)
            if (ShouldCompileShader(sourceFile, outputFile,
                                    shaderSetup.m_compileOnlyIfChanged))
#endif
            {
                CompileShader(compilerPath, sourceFile, outputFile);
                ++compiledCount;
            }

            if (sourceFile.filename() == "planeSweepDepthMapSmokeTest.comp" || sourceFile.filename() == "planeSweepPyramid.comp" || sourceFile.filename() == "planeSweepPatchMatch.comp")
            {
                const std::filesystem::path fp64OutputFile =
                    outputFolder / (sourceFile.filename() == "planeSweepPatchMatch.comp" ? "planeSweepPatchMatchFp64.comp.spv" :
                        (sourceFile.filename() == "planeSweepPyramid.comp" ? "planeSweepPyramidFp64.comp.spv" : "planeSweepDepthMapFp64.comp.spv"));

#if !defined(GRAPHICSAPP_ENABLE_PHASE_2PB_GPU_TRACE)
                if (ShouldCompileShader(sourceFile, fp64OutputFile,
                                        shaderSetup.m_compileOnlyIfChanged))
#endif
                {
                    CompileShader(compilerPath, sourceFile, fp64OutputFile, true);
                    ++compiledCount;
                }
            }
        }

        UIUtility::Print("Startup shader compilation checked " + std::to_string(shaderFiles.size()) + " shader files and compiled " + std::to_string(compiledCount) + ".\n");
    }

    //Returns the executable folder.
    std::filesystem::path ShaderUtility::ModuleFolder(void)
    {
        wchar_t modulePath[MAX_PATH]{};
        const DWORD modulePathLength = GetModuleFileNameW(nullptr, modulePath, MAX_PATH);
        if (modulePathLength == 0)
        {
            return std::filesystem::current_path();
        }

        return std::filesystem::path(modulePath).parent_path();
    }

    //Returns the first matching shader source folder.
    std::filesystem::path ShaderUtility::ResolveSourceFolder(const std::string &folderName)
    {
        const std::filesystem::path setupFolder = folderName.empty() ? std::filesystem::path("Shaders") : std::filesystem::path(folderName);
        std::error_code errorCode;

        if (setupFolder.is_absolute() && FolderHasShaderSources(setupFolder))
        {
            return setupFolder;
        }

        if (!setupFolder.is_absolute())
        {
            const std::filesystem::path configuredPath = PathUtility::ResolvePath(setupFolder.string());
            if (FolderHasShaderSources(configuredPath))
            {
                return configuredPath;
            }
        }

        const std::array<std::filesystem::path, 2> startFolders = {
            std::filesystem::current_path(errorCode), ModuleFolder()
        };

        for (const std::filesystem::path &startFolder : startFolders)
        {
            const std::filesystem::path directPath = startFolder / setupFolder;
            if (FolderHasShaderSources(directPath))
            {
                return directPath;
            }
        }

        std::filesystem::path searchName = setupFolder.filename();
        if (searchName.empty())
        {
            searchName = setupFolder;
        }

        for (const std::filesystem::path &startFolder : startFolders)
        {
            std::filesystem::path currentFolder = startFolder;
            for (int count = 0; count < 8 && !currentFolder.empty(); ++count)
            {
                const std::filesystem::path candidatePath = currentFolder / searchName;
                if (FolderHasShaderSources(candidatePath))
                {
                    return candidatePath;
                }

                if (!currentFolder.has_parent_path())
                {
                    break;
                }

                const std::filesystem::path parentPath = currentFolder.parent_path();
                if (parentPath == currentFolder)
                {
                    break;
                }

                currentFolder = parentPath;
            }
        }

        throw std::runtime_error("Failed to find the shader source folder.");
    }

    //Returns the compiled shader output folder.
    std::filesystem::path ShaderUtility::ResolveOutputFolder(const std::string &folderName)
    {
        const std::filesystem::path setupFolder = folderName.empty() ? std::filesystem::path("Output/Shaders") : std::filesystem::path(folderName);
        const std::filesystem::path outputFolder = setupFolder.is_absolute() ? setupFolder : PathUtility::ResolvePath(setupFolder.string());
        std::error_code errorCode;

        std::filesystem::create_directories(outputFolder, errorCode);
        if (errorCode)
        {
            throw std::runtime_error("Failed to create the compiled shader output folder.");
        }

        return outputFolder;
    }

    //Returns the glslc compiler path.
    std::filesystem::path ShaderUtility::ResolveCompilerPath(const std::string &compilerPath)
    {
        std::error_code errorCode;
        const std::filesystem::path setupCompilerPath(compilerPath);

        if (!compilerPath.empty() && std::filesystem::exists(setupCompilerPath, errorCode))
        {
            return setupCompilerPath;
        }

        char vulkanSdkPath[MAX_PATH]{};
        const DWORD vulkanSdkPathLength = GetEnvironmentVariableA("VULKAN_SDK", vulkanSdkPath, MAX_PATH);
        if (vulkanSdkPathLength > 0 && vulkanSdkPathLength < MAX_PATH)
        {
            const std::filesystem::path sdkCompilerPath = std::filesystem::path(vulkanSdkPath) / "Bin" / "glslc.exe";
            if (std::filesystem::exists(sdkCompilerPath, errorCode))
            {
                return sdkCompilerPath;
            }
        }

        if (!compilerPath.empty())
        {
            throw std::runtime_error("Failed to find the setup glslc compiler path.");
        }

        throw std::runtime_error("Failed to find glslc.exe. Set VULKAN_SDK or Shaders.CompilerPath.");
    }

    //Returns source shader files in a folder.
    std::vector<std::filesystem::path> ShaderUtility::ShaderSourceFiles(const std::filesystem::path &sourceFolder)
    {
        std::vector<std::filesystem::path> shaderFiles;
        std::error_code errorCode;

        for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(sourceFolder, errorCode))
        {
            if (errorCode)
            {
                throw std::runtime_error("Failed to scan the shader source folder.");
            }

            if (entry.is_regular_file(errorCode) && IsShaderSource(entry.path()))
            {
                shaderFiles.push_back(entry.path());
            }
        }

        std::sort(shaderFiles.begin(), shaderFiles.end());
        return shaderFiles;
    }

    //Returns whether a folder contains GLSL shader source files.
    bool ShaderUtility::FolderHasShaderSources(const std::filesystem::path &folderName)
    {
        std::error_code errorCode;
        if (!std::filesystem::is_directory(folderName, errorCode))
        {
            return false;
        }

        for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(folderName, errorCode))
        {
            if (errorCode)
            {
                return false;
            }

            if (entry.is_regular_file(errorCode) && IsShaderSource(entry.path()))
            {
                return true;
            }
        }

        return false;
    }

    //Returns whether a shader source file extension can be compiled.
    bool ShaderUtility::IsShaderSource(const std::filesystem::path &fileName)
    {
        std::string extension = fileName.extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char character) { return static_cast<char>(std::tolower(character)); });

        const std::array<std::string, 6> shaderExtensions = {
            ".vert", ".frag", ".geom", ".tesc", ".tese", ".comp"
        };

        return std::find(shaderExtensions.begin(), shaderExtensions.end(), extension) != shaderExtensions.end();
    }

    //Returns whether a shader should be compiled.
    bool ShaderUtility::ShouldCompileShader(const std::filesystem::path &sourceFile, const std::filesystem::path &outputFile, bool compileOnlyIfChanged)
    {
        std::error_code errorCode;
        if (!compileOnlyIfChanged || !std::filesystem::exists(outputFile, errorCode))
        {
            return true;
        }

        const std::filesystem::file_time_type sourceTime = std::filesystem::last_write_time(sourceFile, errorCode);
        if (errorCode)
        {
            return true;
        }

        const std::filesystem::file_time_type outputTime = std::filesystem::last_write_time(outputFile, errorCode);
        if (errorCode)
        {
            return true;
        }

        if (sourceFile.filename() == "planeSweepPyramid.comp" || sourceFile.filename() == "planeSweepDepthMapSmokeTest.comp" || sourceFile.filename() == "planeSweepPatchMatch.comp")
        {
            const auto sharedTime = std::filesystem::last_write_time(sourceFile.parent_path() / "planeSweepMatching.glsl", errorCode);
            if (errorCode || outputTime < sharedTime) return true;
            if (sourceFile.filename() != "planeSweepDepthMapSmokeTest.comp")
            {
                for (const char *dependency : { "planeSweepPyramid.glsl", "planeSweepSlanted.glsl", "planeSweepSlanted.inl" })
                {
                    const auto time = std::filesystem::last_write_time(sourceFile.parent_path() / dependency, errorCode);
                    if (errorCode || outputTime < time) return true;
                }
            }
        }
        return outputTime < sourceTime;
    }

    //Compiles one GLSL shader file.
    void ShaderUtility::CompileShader(const std::filesystem::path &compilerPath, const std::filesystem::path &sourceFile, const std::filesystem::path &outputFile, bool useFloat64)
    {
        std::string compilerOutput;
        const int result = RunProcess(
            ShaderCommandLine(compilerPath, sourceFile, outputFile, useFloat64),
            compilerOutput);

        if (!compilerOutput.empty())
        {
            UIUtility::Print(compilerOutput + "\n");
        }

        if (result != 0)
        {
            throw std::runtime_error("Failed to compile shader: " + sourceFile.string());
        }
    }

    //Runs one process and captures its output.
    int ShaderUtility::RunProcess(const std::wstring &commandLine, std::string &output)
    {
        SECURITY_ATTRIBUTES securityAttributes{};
        securityAttributes.nLength = sizeof(securityAttributes);
        securityAttributes.bInheritHandle = TRUE;

        HANDLE readPipePtr = nullptr;
        HANDLE writePipePtr = nullptr;
        if (!CreatePipe(&readPipePtr, &writePipePtr, &securityAttributes, 0))
        {
            throw std::runtime_error("Failed to create shader compiler output pipe.");
        }

        SetHandleInformation(readPipePtr, HANDLE_FLAG_INHERIT, 0);

        STARTUPINFOW startupInfo{};
        startupInfo.cb = sizeof(startupInfo);
        startupInfo.dwFlags = STARTF_USESTDHANDLES;
        startupInfo.hStdOutput = writePipePtr;
        startupInfo.hStdError = writePipePtr;
        startupInfo.hStdInput = GetStdHandle(STD_INPUT_HANDLE);

        PROCESS_INFORMATION processInfo{};
        std::wstring mutableCommandLine = commandLine;

        const BOOL processCreated = CreateProcessW(nullptr, mutableCommandLine.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &startupInfo, &processInfo);

        CloseHandle(writePipePtr);
        writePipePtr = nullptr;

        if (!processCreated)
        {
            CloseHandle(readPipePtr);
            throw std::runtime_error("Failed to start glslc shader compiler.");
        }

        char buffer[4096]{};
        DWORD bytesRead = 0;
        while (ReadFile(readPipePtr, buffer, static_cast<DWORD>(sizeof(buffer)), &bytesRead, nullptr) && bytesRead > 0)
        {
            output.append(buffer, bytesRead);
        }

        WaitForSingleObject(processInfo.hProcess, INFINITE);

        DWORD exitCode = 0;
        if (!GetExitCodeProcess(processInfo.hProcess, &exitCode))
        {
            exitCode = 1;
        }

        CloseHandle(processInfo.hThread);
        CloseHandle(processInfo.hProcess);
        CloseHandle(readPipePtr);

        return static_cast<int>(exitCode);
    }

    //Returns a quoted process command line.
    std::wstring ShaderUtility::ShaderCommandLine(const std::filesystem::path &compilerPath, const std::filesystem::path &sourceFile, const std::filesystem::path &outputFile, bool useFloat64)
    {
        std::wstring commandLine = L"\"" + compilerPath.wstring() + L"\" \"" + sourceFile.wstring() + L"\" -o \"" + outputFile.wstring() + L"\"";
#if defined(GRAPHICSAPP_ENABLE_PHASE_2PB_GPU_TRACE)
        commandLine += L" -DGRAPHICSAPP_ENABLE_PHASE_2PB_GPU_TRACE=1";
#endif
        if (useFloat64)
        {
            commandLine += L" -DGRAPHICSAPP_PLANE_SWEEP_FP64=1";
        }
        return commandLine;
    }

}

