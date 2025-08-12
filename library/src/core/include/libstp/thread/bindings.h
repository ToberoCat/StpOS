//
// Created by tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>

#include "interval_scheduler.h"
#include "managed_thread.h"


namespace py = pybind11;


namespace libstp::threads
{
    inline void createManagedThreadBindings(py::module_& m)
    {
        py::class_<ManagedThread, std::shared_ptr<ManagedThread>>(m, "ManagedThread", R"pbdoc(
            Represents a managed thread with capabilities for controlled execution and termination.
        )pbdoc")
            .def(py::init<const std::function<void()>&>(), py::arg("function"), R"pbdoc(
                Initializes a new ManagedThread instance with the specified function.

                Args:
                    function (Callable): The function to execute in the thread.
            )pbdoc")

            .def("kill", &ManagedThread::kill, R"pbdoc(
                Terminates the thread execution immediately.
            )pbdoc")

            .def("join", &ManagedThread::join, R"pbdoc(
                Blocks the calling thread until this thread finishes execution.
            )pbdoc");

        m.def("create_thread", &createThread, py::arg("function"), R"pbdoc(
            Creates a new managed thread with the specified function.

            Args:
                function (Callable): The function to execute in the thread.

            Returns:
                ManagedThread: The created managed thread.
        )pbdoc");

        m.def("kill_all_threads", &killAllThreads, R"pbdoc(
            Terminates all managed threads immediately.
        )pbdoc");

        m.def("shut_down_in", &shutDownIn, py::arg("duration"), R"pbdoc(
            Schedules a shutdown of all threads after the specified duration.

            Args:
                duration (timedelta): The duration after which to shut down threads.
        )pbdoc");
    }

    inline void createIntervalBindings(const py::module_& m)
    {
        py::class_<IntervalScheduler, std::shared_ptr<IntervalScheduler>>(m, "IntervalScheduler", R"pbdoc(
            Represents a scheduler that executes a task at fixed intervals.
        )pbdoc")
            .def(py::init<std::function<void(const CancelTask&)>, std::chrono::milliseconds>(),
                 py::arg("task"), py::arg("interval"), R"pbdoc(
                Initializes a new IntervalScheduler instance with the specified task and interval.

                Args:
                    task (Callable): The task to execute at intervals.
                    interval (timedelta): The interval at which to execute the task.
            )pbdoc")

            .def("is_running", &IntervalScheduler::isRunning, R"pbdoc(
                Checks whether the scheduler is currently running.

                Returns:
                    bool: True if the scheduler is running, False otherwise.
            )pbdoc")

            .def("start", &IntervalScheduler::start, R"pbdoc(
                Starts the scheduler, executing the task at the specified intervals.
            )pbdoc")

            .def("cancel_task", &IntervalScheduler::cancelTask, R"pbdoc(
                Cancels the current task and stops the scheduler.
            )pbdoc")

            .def("stop", &IntervalScheduler::stop, R"pbdoc(
                Stops the scheduler.
            )pbdoc");
    }
}
