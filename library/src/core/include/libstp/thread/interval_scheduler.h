//
// Created by tobias on 12/26/24.
//

#pragma once
#include <functional>
#include <thread>

namespace libstp::threads {
    typedef std::function<void()> CancelTask;

    class IntervalScheduler {
        std::function<void(CancelTask)> task_;
        std::chrono::milliseconds interval_;
        std::atomic<bool> running_;
        std::atomic<bool> stop_;
        std::thread thread_;

    public:
        IntervalScheduler(std::function<void(const CancelTask &)> task,
                          const std::chrono::milliseconds interval) : task_(std::move(task)),
                                                                interval_(interval) {}

        ~IntervalScheduler() {
            stop();
        }

        bool isRunning() {
            return running_;
        }

        void start() {
            running_ = true;
            stop_ = false;
            CancelTask cancelTask = [this]() {
                running_ = false;
                stop_ = true;
            };

            thread_ = std::thread(&IntervalScheduler::run_, this);
        }

        void cancelTask() {
            running_ = false;
            stop_ = true;
        }

        void run_() {
            const CancelTask cancelTask = [this]() { this->cancelTask(); };
            while (!stop_) {
                std::this_thread::sleep_for(interval_);
                if (!stop_ && running_) {
                    task_(cancelTask);
                }
            }
        }

        void stop() {
            if (!running_) {
                return;
            }

            running_ = false;
            stop_ = true;
            if (thread_.joinable()) {
                thread_.join();
            }
        }
    };
}
