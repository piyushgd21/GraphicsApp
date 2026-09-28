///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	ShaderUtility.h
//	Description	:
//		Defines runtime GLSL shader compilation helpers.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "global.h"

#include "SetupUtility.h"

#include <filesystem>
#include <string>
#include <vector>

namespace graphicsFX
{
    //Defines runtime GLSL shader compilation helpers.
    class ShaderUtility
    {

        public:

            //Creates the shader utility.
            ShaderUtility(void) = default;

            //Destroys the shader utility.
            ~ShaderUtility(void) = default;

            ShaderUtility(const ShaderUtility &) = delete;
            ShaderUtility &operator=(const ShaderUtility &) = delete;

            //Compiles startup shaders when enabled.
            static void CompileStartupShaders(const ShaderSetup &shaderSetup);

        private:

            //Returns the executable folder.
            static std::filesystem::path ModuleFolder(void);

            //Returns the first matching shader source folder.
            static std::filesystem::path ResolveSourceFolder(const std::string &folderName);

            //Returns the compiled shader output folder.
            static std::filesystem::path ResolveOutputFolder(const std::string &folderName);

            //Returns the glslc compiler path.
            static std::filesystem::path ResolveCompilerPath(const std::string &compilerPath);

            //Returns source shader files in a folder.
            static std::vector<std::filesystem::path> ShaderSourceFiles(const std::filesystem::path &sourceFolder);

            //Returns whether a folder contains GLSL shader source files.
            static bool FolderHasShaderSources(const std::filesystem::path &folderName);

            //Returns whether a shader source file extension can be compiled.
            static bool IsShaderSource(const std::filesystem::path &fileName);

            //Returns whether a shader should be compiled.
            static bool ShouldCompileShader(const std::filesystem::path &sourceFile, const std::filesystem::path &outputFile, bool compileOnlyIfChanged);

            //Compiles one GLSL shader file.
            static void CompileShader(const std::filesystem::path &compilerPath, const std::filesystem::path &sourceFile, const std::filesystem::path &outputFile);

            //Runs one process and captures its output.
            static int RunProcess(const std::wstring &commandLine, std::string &output);

            //Returns a quoted process command line.
            static std::wstring ShaderCommandLine(const std::filesystem::path &compilerPath, const std::filesystem::path &sourceFile, const std::filesystem::path &outputFile);

    };
}

