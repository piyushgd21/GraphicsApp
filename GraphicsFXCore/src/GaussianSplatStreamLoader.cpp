///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	GaussianSplatStreamLoader.cpp
//	Description	:
//		Implements an asynchronous Gaussian splat stream loader.
//
//
///////////////////////////////////////////////////////////////////////

#include "GaussianSplatStreamLoader.h"

#include <algorithm>
#include <chrono>
#include <utility>

namespace graphicsFX
{
    //Returns elapsed milliseconds between two time points.
    static double GaussianSplatStreamMilliseconds(std::chrono::steady_clock::time_point startTime, std::chrono::steady_clock::time_point endTime)
    {
        return std::chrono::duration<double, std::milli>(endTime - startTime).count();
    }

    //Stops the Gaussian splat stream loader.
    GaussianSplatStreamLoader::~GaussianSplatStreamLoader(void)
    {
        Stop();
    }

    //Starts the Gaussian splat stream workers.
    void GaussianSplatStreamLoader::Start(std::vector<std::filesystem::path> files, const GaussianSplatSetup &setup, int preloadCount)
    {
        Stop();

        int workerThreadCount = 1;
        bool hasFiles = false;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_files = std::move(files);
            m_setup = setup;
            m_requests.clear();
            m_loadingFrames.clear();
            m_readyFrames.clear();
            m_preloadCount = std::max(1, preloadCount);
            m_stopRequested.store(false);
            workerThreadCount = std::clamp(m_setup.m_streamWorkerThreadCount, 1, 4);
            hasFiles = !m_files.empty();
        }

        if (hasFiles)
        {
            m_workers.reserve(static_cast<std::size_t>(workerThreadCount));

			//create and run the threads for loading Gaussian splat stream frames
            for (int workerIndex = 0; workerIndex < workerThreadCount; ++workerIndex)
            {
                m_workers.emplace_back(&GaussianSplatStreamLoader::WorkerLoop, this);
            }
        }
    }

    //Stops the Gaussian splat stream workers.
    void GaussianSplatStreamLoader::Stop(void)
    {
        m_stopRequested.store(true);
        m_condition.notify_all();

        for (std::thread &worker : m_workers)
        {
            if (worker.joinable())
            {
                worker.join();
            }
        }
        m_workers.clear();

        std::lock_guard<std::mutex> lock(m_mutex);
        m_requests.clear();
        m_loadingFrames.clear();
        m_readyFrames.clear();
        m_files.clear();
    }

    //Requests one Gaussian splat stream frame.
    void GaussianSplatStreamLoader::RequestFrame(std::size_t frameIndex)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_stopRequested.load() || frameIndex >= m_files.size() || FrameQueuedOrReady(frameIndex))
        {
            return;
        }

        m_requests.push_back(GaussianSplatStreamFrameRequest{ frameIndex, std::chrono::steady_clock::now() });
        m_condition.notify_one();
    }

    //Returns a decoded Gaussian splat stream frame when ready.
    bool GaussianSplatStreamLoader::TryConsumeFrame(std::size_t frameIndex, GaussianSplatReadResult &result, GaussianSplatStreamFrameTiming *timingPtr)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        const auto found = std::find_if(
            m_readyFrames.begin(),
            m_readyFrames.end(),
            [frameIndex](const GaussianSplatStreamFrame &frame)
            {
                return frame.m_frameIndex == frameIndex;
            }
        );

        if (found == m_readyFrames.end())
        {
            return false;
        }

        result = std::move(found->m_result);
        if (timingPtr != nullptr)
        {
            *timingPtr = std::move(found->m_timing);
        }

        m_readyFrames.erase(found);
        return true;
    }

    //Runs the Gaussian splat stream worker loop.
    void GaussianSplatStreamLoader::WorkerLoop(void)
    {
        while (!m_stopRequested.load())
        {
            std::size_t frameIndex = 0U;
            std::filesystem::path filePath;
            GaussianSplatSetup setup;
            std::chrono::steady_clock::time_point requestTime;

            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_condition.wait(
                    lock,
                    [this](void)
                    {
                        return m_stopRequested.load() || !m_requests.empty();
                    }
                );

                if (m_stopRequested.load())
                {
                    break;
                }

                const GaussianSplatStreamFrameRequest request = m_requests.front();
                m_requests.pop_front();
                frameIndex = request.m_frameIndex;
                requestTime = request.m_requestTime;

                if (frameIndex >= m_files.size())
                {
                    continue;
                }

                filePath = m_files[frameIndex];
                setup = m_setup;
                m_loadingFrames.push_back(frameIndex);
            }

            const auto loadStartTime = std::chrono::steady_clock::now();
            GaussianSplatReadResult result = GaussianSplatUtility::LoadFile(filePath, setup);
            const auto loadEndTime = std::chrono::steady_clock::now();

            GaussianSplatStreamFrameTiming timing;
            timing.m_frameIndex = frameIndex;
            timing.m_splatCount = result.m_splats.size();
            timing.m_queueWaitMilliseconds = GaussianSplatStreamMilliseconds(requestTime, loadStartTime);
            timing.m_loadMilliseconds = GaussianSplatStreamMilliseconds(loadStartTime, loadEndTime);
            timing.m_fileName = filePath.filename().string();

            {
                std::lock_guard<std::mutex> lock(m_mutex);
                const auto loadingFrame = std::find(m_loadingFrames.begin(), m_loadingFrames.end(), frameIndex);
                if (loadingFrame != m_loadingFrames.end())
                {
                    m_loadingFrames.erase(loadingFrame);
                }

                if (m_stopRequested.load())
                {
                    break;
                }

                m_readyFrames.push_back(GaussianSplatStreamFrame{ frameIndex, std::move(result), std::move(timing) });
            }
        }
    }

    //Returns whether a frame is already queued or ready.
    bool GaussianSplatStreamLoader::FrameQueuedOrReady(std::size_t frameIndex) const
    {
        const auto requestFound = std::find_if(
            m_requests.begin(),
            m_requests.end(),
            [frameIndex](const GaussianSplatStreamFrameRequest &request)
            {
                return request.m_frameIndex == frameIndex;
            }
        );

        if (requestFound != m_requests.end())
        {
            return true;
        }

        if (std::find(m_loadingFrames.begin(), m_loadingFrames.end(), frameIndex) != m_loadingFrames.end())
        {
            return true;
        }

        return std::find_if(
            m_readyFrames.begin(),
            m_readyFrames.end(),
            [frameIndex](const GaussianSplatStreamFrame &frame)
            {
                return frame.m_frameIndex == frameIndex;
            }
        ) != m_readyFrames.end();
    }
}
