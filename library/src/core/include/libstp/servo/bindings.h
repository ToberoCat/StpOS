//
// Created by tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>

#include "servo.h"

namespace py = pybind11;

namespace libstp::servo
{
    inline void createServoBindings(const py::module& m)
    {
        py::class_<Servo>(m, "Servo", R"pbdoc(
            Represents a servo motor.

            A Servo object provides methods to control the position and behavior
            of a servo motor connected to a specific port.
        )pbdoc")
            .def(py::init<int>(),
                 py::arg("port"),
                 R"pbdoc(
                     Initialize a Servo with a port number.

                     Args:
                         port (int): The port number to which the servo is connected.
                 )pbdoc")
            .def("set_position",
                 &Servo::setPosition,
                 py::arg("position"),
                 R"pbdoc(
                     Set the servo position.

                     Args:
                         position (int): The target position for the servo.
                 )pbdoc")
            .def("get_position",
                 &Servo::getPosition,
                 R"pbdoc(
                     Get the current servo position.

                     Returns:
                         int: The current position of the servo.
                 )pbdoc")
            .def("slowly_set_position",
                 &Servo::slowlySetPosition,
                 py::arg("target_position"),
                 py::arg("duration"),
                 py::arg("interpolation_function"),
                 R"pbdoc(
                     Set the position slowly using an interpolation function.

                     Args:
                         target_position (int): The desired position for the servo.
                         duration (chrono::milliseconds): The duration over which the servo moves.
                         interpolation_function (InterpolationFunction): The function used to interpolate movement.
                 )pbdoc")
            .def("shake",
                 &Servo::shake,
                 py::arg("center_position"),
                 py::arg("amplitude"),
                 py::arg("speed_hz"),
                 py::arg("conditional"),
                 R"pbdoc(
                     Shake the servo around a center position with a specified amplitude and speed.

                     Args:
                         center_position (int): The central position around which the servo oscillates.
                         amplitude (float): The range of oscillation.
                         speed_hz (float): The speed of oscillation in hertz.
                         conditional (ConditionalFunction): A condition to control the shaking behavior.
                 )pbdoc")
            .def("disable",
                 &Servo::disable,
                 R"pbdoc(
                     Disable the servo.

                     Disables the servo, preventing it from holding its position.
                 )pbdoc")
            .def("enable",
                 &Servo::enable,
                 R"pbdoc(
                     Enable the servo.

                     Enables the servo, allowing it to hold its position and respond to commands.
                 )pbdoc")
            .def_static("disable_all_servos",
                        &Servo::disableAllServos,
                        R"pbdoc(
                            Disable all servos.

                            This method disables all servos connected to the system.
                        )pbdoc");
    }
}
