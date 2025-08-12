//
// Created by tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>
#include "motor.h"
#include "servo_like_motor.h"

namespace py = pybind11;

namespace libstp::motor
{
    inline void createMotorBindings(const py::module_& m)
    {
        py::class_<Motor, std::shared_ptr<Motor>>(m, "Motor", R"pbdoc(
            Represents a generic motor with functionality for movement and velocity control.
        )pbdoc")
            .def(py::init<int, bool>(), py::arg("port"), py::arg("reverse_polarity") = false, R"pbdoc(
                Initializes a new Motor instance.

                For this to function properly, make sure that the set_velocity(500) command drives your motor **forward**

                Args:
                    port (int): The motor port number.
                    reverse_polarity (bool): Whether to reverse the motor polarity.
            )pbdoc")

            .def("get_current_position_estimate", &Motor::getCurrentPositionEstimate, R"pbdoc(
                Gets the current position estimate of the motor. The position will be positive for a forward drive, and negative for a reverse drive.

                Returns:
                    int: The current position estimate in ticks.
            )pbdoc")

            .def("reset_position_estimate", &Motor::resetPositionEstimate, R"pbdoc(
                Resets the position estimate of the motor to zero.
            )pbdoc")

            .def("set_velocity", &Motor::setVelocity, py::arg("velocity"), R"pbdoc(
                Sets the motor velocity.

                Args:
                    velocity (int): The velocity in ticks per second.
            )pbdoc")

            .def("move_while", &Motor::moveWhile, py::arg("condition"), py::arg("velocity"), R"pbdoc(
                Moves the motor while a given condition is true.

                Args:
                    condition (ConditionalFunction): A function that determines whether the motor should keep moving.
                    velocity (int): The velocity in ticks per second.
            )pbdoc")
            .def("move_by_ticks", &Motor::moveByTicks, py::arg("ticks"), py::arg("velocity") = 500, R"pbdoc(
                Moves the motor by a given number of ticks.

                Args:
                    ticks (int): The number of ticks to move.
                    velocity (int): The velocity in ticks per second.)pbdoc")
            .def("move_to_ticks", &Motor::moveToTicks, py::arg("ticks"), py::arg("velocity") = 500, R"pbdoc(
                Moves the motor to a given position in ticks.

                Args:
                    ticks (int): The target position in ticks.
                    velocity (int): The velocity in ticks per second.)pbdoc")

            .def("stop", &Motor::stop, R"pbdoc(
                Stops the motor.
            )pbdoc")

            .def_static("stop_all_motors", &Motor::stopAllMotors, R"pbdoc(
                Stops all motors.
            )pbdoc");
    }

    inline void createServoLikeMotorBindings(const py::module_& m)
    {
        py::class_<ServoLikeMotor, Motor, std::shared_ptr<ServoLikeMotor>>(m, "ServoLikeMotor", R"pbdoc(
            Represents a motor with servo-like functionality, allowing precise position control.
        )pbdoc")
            .def(py::init<int, bool>(), py::arg("port"), py::arg("reverse_polarity") = false, R"pbdoc(
                Initializes a new ServoLikeMotor instance.

                Args:
                    port (int): The motor port number.
                    reverse_polarity (bool): Whether to reverse the motor polarity.
            )pbdoc")

            .def("disable", &ServoLikeMotor::disable, R"pbdoc(
                Disables the servo motor, cutting power to it.
            )pbdoc")

            .def("enable", &ServoLikeMotor::enable, R"pbdoc(
                Enables the servo motor, providing power to it.
            )pbdoc")

            .def("get_position", &ServoLikeMotor::getPosition, R"pbdoc(
                Gets the current position of the servo motor.

                Returns:
                    int: The current position in ticks.
            )pbdoc")

            .def("set_position", &ServoLikeMotor::setPosition, py::arg("position"), R"pbdoc(
                Sets the position of the servo motor.

                Args:
                    position (int): The target position in ticks.
            )pbdoc")

            .def("set_position_velocity", &ServoLikeMotor::setPositionVelocity, py::arg("position"),
                 py::arg("velocity"), R"pbdoc(
                Sets the target position and velocity for the servo motor.

                Args:
                    position (int): The target position in ticks.
                    velocity (int): The velocity in ticks per second.
            )pbdoc");
    }
}
