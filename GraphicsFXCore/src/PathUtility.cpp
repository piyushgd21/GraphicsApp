///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	PathUtility.cpp
//	Description	:
//		Implements centralized runtime path helpers.
//
//
///////////////////////////////////////////////////////////////////////

#include "PathUtility.h"

#include <Windows.h>

namespace graphicsFX
{
    PathSetup PathUtility::s_paths; //Stores configured runtime path folders.

    //Configures runtime path folders.
    void PathUtility::SetPaths(const PathSetup &paths)
    {
        s_paths = paths;
    }

    //Returns current runtime path folders.
    const PathSetup &PathUtility::Paths(void)
    {
        return s_paths;
    }

    //Returns the solution root folder.
    std::filesystem::path PathUtility::SolutionFolder(void)
    {
        std::error_code errorCode;
        std::filesystem::path solutionFolder = SearchSolutionFolder(std::filesystem::current_path(errorCode));
        if (!solutionFolder.empty())
        {
            return solutionFolder;
        }

        solutionFolder = SearchSolutionFolder(ModuleFolder());
        if (!solutionFolder.empty())
        {
            return solutionFolder;
        }

        return std::filesystem::current_path(errorCode);
    }

    //Returns a path under the texture folder.
    std::filesystem::path PathUtility::TexturePath(const std::string &fileName)
    {
        const std::filesystem::path existingPath = FirstExistingPath(FileCandidates(fileName, s_paths.m_textureFolder));
        if (!existingPath.empty())
        {
            return existingPath;
        }

        const std::filesystem::path texturePath(fileName);
        if (texturePath.is_absolute() || texturePath.has_parent_path())
        {
            return texturePath;
        }

        return ResolvePath(s_paths.m_textureFolder) / texturePath;
    }

    //Returns a path under the model folder.
    std::filesystem::path PathUtility::ModelPath(const std::string &fileName)
    {
        const std::filesystem::path existingPath = FirstExistingPath(FileCandidates(fileName, s_paths.m_modelFolder));
        if (!existingPath.empty())
        {
            return existingPath;
        }

        const std::filesystem::path modelPath(fileName);
        if (modelPath.is_absolute() || modelPath.has_parent_path())
        {
            return modelPath;
        }

        return ResolvePath(s_paths.m_modelFolder) / modelPath;
    }

    //Returns the shader source folder.
    std::filesystem::path PathUtility::ShaderSourceFolder(void)
    {
        return ResolvePath(s_paths.m_shaderSourceFolder);
    }

    //Returns the shader output folder.
    std::filesystem::path PathUtility::ShaderOutputFolder(void)
    {
        return ResolvePath(s_paths.m_shaderOutputFolder);
    }

    //Returns the frame capture output folder.
    std::filesystem::path PathUtility::CaptureOutputFolder(void)
    {
        return ResolvePath(s_paths.m_captureOutputFolder);
    }

    //Returns a path under the solution folder.
    std::filesystem::path PathUtility::ResolvePath(const std::string &pathName)
    {
        const std::filesystem::path path(pathName);
        if (path.is_absolute())
        {
            return path;
        }

        return SolutionFolder() / path;
    }

    //Returns the executable folder.
    std::filesystem::path PathUtility::ModuleFolder(void)
    {
        wchar_t modulePath[MAX_PATH]{};
        const DWORD modulePathLength = GetModuleFileNameW(nullptr, modulePath, MAX_PATH);
        if (modulePathLength == 0)
        {
            return {};
        }

        return std::filesystem::path(modulePath).parent_path();
    }

    //Searches upward for the solution folder.
    std::filesystem::path PathUtility::SearchSolutionFolder(std::filesystem::path startPath)
    {
        std::error_code errorCode;
        std::filesystem::path currentPath = std::filesystem::absolute(startPath, errorCode);
        if (errorCode)
        {
            currentPath = startPath;
        }

        while (!currentPath.empty())
        {
            if (std::filesystem::exists(currentPath / "GraphicsApp.sln", errorCode))
            {
                return currentPath;
            }

            const std::filesystem::path parentPath = currentPath.parent_path();
            if (parentPath == currentPath)
            {
                break;
            }

            currentPath = parentPath;
        }

        return {};
    }

    //Returns file path candidates.
    std::vector<std::filesystem::path> PathUtility::FileCandidates(const std::string &fileName, const std::string &folderName)
    {
        std::vector<std::filesystem::path> candidates;
        const std::filesystem::path filePath(fileName);
        if (filePath.is_absolute())
        {
            candidates.push_back(filePath);
            return candidates;
        }

        std::error_code errorCode;
        const std::vector<std::filesystem::path> rootFolders = {
            SolutionFolder(), std::filesystem::current_path(errorCode), ModuleFolder()
        };

        for (const std::filesystem::path &rootFolder : rootFolders)
        {
            if (rootFolder.empty())
            {
                continue;
            }

            candidates.push_back(rootFolder / filePath);
            if (!filePath.has_parent_path() && !folderName.empty())
            {
                candidates.push_back(rootFolder / folderName / filePath);
            }
        }

        candidates.push_back(filePath);
        return candidates;
    }

    //Returns the first existing path.
    std::filesystem::path PathUtility::FirstExistingPath(const std::vector<std::filesystem::path> &candidates)
    {
        std::error_code errorCode;
        for (const std::filesystem::path &candidate : candidates)
        {
            if (std::filesystem::exists(candidate, errorCode))
            {
                return candidate;
            }
        }

        return {};
    }
}

