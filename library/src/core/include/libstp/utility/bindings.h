//
// Created by tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>

#include "pid.h"
#include "libstp/_config.h"

namespace py = pybind11;

namespace libstp::utility
{
    inline void createPidBindings(const py::module& m)
    {
        py::class_<PIDController>(m, "PIDController", R"pbdoc(
            A PID controller for managing control loops.

            This class provides methods for tuning and executing a PID control loop
            with configurable proportional, integral, and derivative gains.
        )pbdoc")
            .def(py::init<float, float, float>(),
                 py::arg("Kp"), py::arg("Ki"), py::arg("Kd"),
                 R"pbdoc(
                     Initialize a PIDController.

                     Args:
                         Kp (float): Proportional gain.
                         Ki (float): Integral gain.
                         Kd (float): Derivative gain.
                 )pbdoc")
            .def("calculate",
                 &PIDController::calculate,
                 py::arg("error"),
                 R"pbdoc(
                     Calculate the PID output for a given error.

                     Args:
                         error (float): The current error in the system.

                     Returns:
                         float: The calculated PID output, clamped to the specified range.
                 )pbdoc")
            .def("set_parameters",
                 &PIDController::setParameters,
                 py::arg("parameters"),
                 R"pbdoc(
                     Set the PID parameters.

                     Args:
                         parameters (PidParameters): The new PID parameters to set.
                 )pbdoc")
            .def("reset",
                 &PIDController::reset,
                 R"pbdoc(
                     Reset the PID controller state.
                 )pbdoc");
    }

    inline void createLoggingBindings(py::module_& m)
    {
        m.def("debug", [](const char* message) { spdlog::debug(message); }, R"pbdoc(
        Log a message with severity level debug
    )pbdoc", py::arg("message"));

        m.def("info", [](const char* message) { spdlog::info(message); }, R"pbdoc(
        Log a message with severity level info
    )pbdoc", py::arg("message"));

        m.def("warn", [](const char* message) { spdlog::warn(message); }, R"pbdoc(
        Log a message with severity level warn
    )pbdoc", py::arg("message"));

        m.def("error", [](const char* message) { spdlog::error(message); }, R"pbdoc(
        Log a message with severity level error
    )pbdoc", py::arg("message"));
    }
}
