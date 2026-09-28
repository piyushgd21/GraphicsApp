///////////////////////////////////////////////////////////////////////
//	Programmer	-	Sujal Bista
//	File Name	-	GaussianSplatStreamLoader.h
//	Description	:
//		Defines an asynchronous Gaussian splat stream loader.
//
//
///////////////////////////////////////////////////////////////////////

#pragma once

#include "GaussianSplatUtility.h"
#include "SetupUtility.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace graphicsFX
{
    //Stores timing information for one Gaussian splat stream frame.
    struct GaussianSplatStreamFrameTiming
    {
        std::size_t m_frameIndex{ 0U }; //Stores the stream frame index.
        std::size_t m_splatCount{ 0U }; //Stores decoded splat count.
        double m_queueWaitMilliseconds{ 0.0 }; //Stores time spent waiting in the decode queue.
        double m_loadMilliseconds{ 0.0 }; //Stores CPU file load and decode time.
        std::string m_fileName; //Stores the decoded source file name.
    };

    //Stores one decoded Gaussian splat stream frame.
    struct GaussianSplatStreamFrame
    {
        std::size_t m_frameIndex{ 0U }; //Stores the stream frame index.
        GaussianSplatReadResult m_result; //Stores the decoded Gaussian splat result.
        GaussianSplatStreamFrameTiming m_timing; //Stores timing information for the decoded frame.
    };

    //Stores one queued Gaussian splat stream request.
    struct GaussianSplatStreamFrameRequest
    {
        std::size_t m_frameIndex{ 0U }; //Stores the requested stream frame index.
        std::chrono::steady_clock::time_point m_requestTime; //Stores when the frame was requested.
    };

    //Defines an asynchronous Gaussian splat stream loader.
    class GaussianSplatStreamLoader final
    {

        public:

            //Creates the Gaussian splat stream loader.
            GaussianSplatStreamLoader(void) = default;

            //Stops the Gaussian splat stream loader.
            ~GaussianSplatStreamLoader(void);

            GaussianSplatStreamLoader(const GaussianSplatStreamLoader &) = delete;
            GaussianSplatStreamLoader &operator=(const GaussianSplatStreamLoader &) = delete;

            //Starts the Gaussian splat stream workers.
            void Start(std::vector<std::filesystem::path> files, const GaussianSplatSetup &setup, int preloadCount);

            //Stops the Gaussian splat stream workers.
            void Stop(void);

            //Requests one Gaussian splat stream frame.
            void RequestFrame(std::size_t frameIndex);

            //Returns a decoded Gaussian splat stream frame when ready.
            bool TryConsumeFrame(std::size_t frameIndex, GaussianSplatReadResult &result, GaussianSplatStreamFrameTiming *timingPtr = nullptr);

        private:

            //Runs one Gaussian splat stream worker loop.
            void WorkerLoop(void);

            //Returns whether a frame is already queued or ready.
            bool FrameQueuedOrReady(std::size_t frameIndex) const;

            std::vector<std::filesystem::path> m_files; //Stores ordered stream files.
            GaussianSplatSetup m_setup; //Stores Gaussian splat loading setup.
            std::deque<GaussianSplatStreamFrameRequest> m_requests; //Stores queued frame decode requests.
            std::vector<std::size_t> m_loadingFrames; //Stores frame indices currently being decoded.
            std::vector<GaussianSplatStreamFrame> m_readyFrames; //Stores decoded stream frames.
            mutable std::mutex m_mutex; //Protects request and ready frame data.
            std::condition_variable m_condition; //Signals worker queue changes.
            std::vector<std::thread> m_workers; //Stores background worker threads.
            std::atomic_bool m_stopRequested{ false }; //Stores whether workers should stop.
            int m_preloadCount{ 2 }; //Stores the preferred preload count.
    };
}
