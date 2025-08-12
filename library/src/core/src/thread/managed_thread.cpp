//
// Created by tobias on 12/26/24.
//

#include "libstp/thread/managed_thread.h"
#include "libstp/_config.h"
#include "libstp/motor/motor.h"
#include "libstp/servo/servo.h"

auto createdThreads = std::vector<libstp::threads::ManagedThread *>();

libstp::threads::ManagedThread libstp::threads::createThread(const std::function<void()> &function) {
    ManagedThread managedThread(function);
    createdThreads.push_back(&managedThread);

    managedThread.thread.detach();
    return managedThread;
}

void libstp::threads::killAllThreads() {
    SPDLOG_WARN("Killing all threads");
    for (const auto thread: createdThreads) {
        thread->kill();
    }
}


void libstp::threads::ManagedThread::kill() {
    SPDLOG_DEBUG("Killing thread");
    pthread_cancel(thread.native_handle());
    std::erase(createdThreads, this);
}

void libstp::threads::ManagedThread::join() {
    if (thread.joinable()) {
        thread.join();
    }
}

void libstp::threads::shutDownIn(const std::chrono::milliseconds &duration) {
    SPDLOG_DEBUG("Shutting down in {} ms", duration.count());
    std::thread([duration]() {
        std::this_thread::sleep_for(duration);
        SPDLOG_INFO("Shutting down now");
        killAllThreads();
        motor::Motor::stopAllMotors();
        servo::Servo::disableAllServos();
        exit(0);
    }).detach();
}
