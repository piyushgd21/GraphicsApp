///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	PathUtility.h
//	Description	:
//		Defines centralized runtime path helpers.
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
    //Defines centralized runtime path helpers.
    class PathUtility final
    {

        public:

            //Configures runtime path folders.
            static void SetPaths(const PathSetup &paths);

            //Returns current runtime path folders.
            static const PathSetup &Paths(void);

            //Returns the solution root folder.
            static std::filesystem::path SolutionFolder(void);

            //Returns a path under the texture folder.
            static std::filesystem::path TexturePath(const std::string &fileName);

            //Returns a path under the model folder.
            static std::filesystem::path ModelPath(const std::string &fileName);

            //Returns the shader source folder.
            static std::filesystem::path ShaderSourceFolder(void);

            //Returns the shader output folder.
            static std::filesystem::path ShaderOutputFolder(void);

            //Returns the frame capture output folder.
            static std::filesystem::path CaptureOutputFolder(void);

            //Returns a path under the solution folder.
            static std::filesystem::path ResolvePath(const std::string &pathName);

        private:

            //Returns the executable folder.
            static std::filesystem::path ModuleFolder(void);

            //Searches upward for the solution folder.
            static std::filesystem::path SearchSolutionFolder(std::filesystem::path startPath);

            //Returns file path candidates.
            static std::vector<std::filesystem::path> FileCandidates(const std::string &fileName, const std::string &folderName);

            //Returns the first existing path.
            static std::filesystem::path FirstExistingPath(const std::vector<std::filesystem::path> &candidates);

            static PathSetup s_paths; //Stores configured runtime path folders.

    };
}

