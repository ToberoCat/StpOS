//
// Created by tobias on 12/26/24.
//

#pragma once
#include <functional>
#include <thread>

namespace libstp::threads
{
    struct ManagedThread
    {
        std::thread thread;

        explicit ManagedThread(const std::function<void()>& function)
        {
            thread = std::thread(function);
        }

        void kill();

        void join();
    };

    ManagedThread createThread(const std::function<void()>& function);

    void killAllThreads();

    void shutDownIn(const std::chrono::milliseconds& duration);
}
