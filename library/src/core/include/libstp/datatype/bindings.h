//
// Created by tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#include "axis.h"
#include "conditions.h"
#include "functions.h"

namespace py = pybind11;

namespace libstp::datatype
{
    class ConditionalResultTrampoline final : public ConditionalResult
    {
    public:
        using ConditionalResult::ConditionalResult;

        [[nodiscard]] float progress() const override
        {
            PYBIND11_OVERLOAD_PURE(
                float,
                ConditionalResult,
                progress
            );
        }

        [[nodiscard]] bool is_loop_running() const override
        {
            PYBIND11_OVERLOAD_PURE(
                bool,
                ConditionalResult,
                is_loop_running
            );
        }

        void update(motion::DifferentialDriveState& state) override
        {
            PYBIND11_OVERLOAD_PURE(
                void,
                ConditionalResult,
                update
            );
        }
        
        [[nodiscard]] std::string to_string() const override
        {
            PYBIND11_OVERLOAD_PURE(
                std::string,
                ConditionalResult,
                to_string
            );
        }
    };

    inline void createAxisBindings(const py::module& m)
    {
        py::enum_<Axis>(m, "Axis", "Represents the axis of the robot")
            .value("X", X, "Represents the X-axis")
            .value("Y", Y, "Represents the Y-axis")
            .value("Z", Z, "Represents the Z-axis")
            .export_values();

        py::enum_<Direction>(m, "Direction", "Represents the direction of the robot")
            .value("Normal", Direction::Forward, "Represents the normal direction")
            .value("Reversed", Direction::Backward, "Represents the reversed direction")
            .export_values();
    }

    inline void createConditionsBindings(const py::module& m)
    {
        py::class_<ConditionalResult, ConditionalResultTrampoline, std::shared_ptr<ConditionalResult>>(
                m, "ConditionalResult", "Base class for conditional results")
            .def(py::init<>(), "Initialize a ConditionalResult")
            .def("progress", &ConditionalResult::progress, py::return_value_policy::reference,
                 "Get the progress of the condition")
            .def("is_loop_running", &ConditionalResult::is_loop_running, py::return_value_policy::reference,
                 "Check if the loop is running")
            .def("to_string", &ConditionalResult::to_string, py::return_value_policy::copy,
                 "Get a string representation of the condition state")
            .def("__str__", &ConditionalResult::to_string)
            .def("__repr__", [](const ConditionalResult& self) {
                return "<ConditionalResult: " + self.to_string() + ">";
            });

        py::class_<UndefinedConditionalResult, ConditionalResult, std::shared_ptr<UndefinedConditionalResult>>(
                m, "UndefinedConditionalResult",
                "Represents a condition that may or may not be met")
            .def(py::init<const bool>(), py::arg("conditionMet"), "Initialize with whether the condition is met")
            .def("__str__", &UndefinedConditionalResult::to_string)
            .def("__repr__", [](const UndefinedConditionalResult& self) {
                return "<UndefinedConditionalResult(condition_met=" + 
                       std::string(self._conditionMet ? "True" : "False") + ")>";
            });

        py::class_<DefinedConditionalResult, ConditionalResult, std::shared_ptr<DefinedConditionalResult>>(
                m, "DefinedConditionalResult",
                "Represents a condition with a defined target")
            .def(py::init<const float>(), py::arg("target"), "Initialize with a target value")
            .def_readwrite("target", &DefinedConditionalResult::target, "The target value")
            .def_readwrite("current", &DefinedConditionalResult::current, "The current value")
            .def("__str__", &DefinedConditionalResult::to_string)
            .def("__repr__", [](const DefinedConditionalResult& self) {
                return "<DefinedConditionalResult(target=" + std::to_string(self.target) + 
                       ", current=" + std::to_string(self.current) + ")>";
            });

        py::class_<DistanceConditionalResult, DefinedConditionalResult, std::shared_ptr<DistanceConditionalResult>>(
                m, "DistanceConditionalResult",
                "Represents a distance-based condition")
            .def(py::init<const float>(), py::arg("target"), "Initialize with a distance target")
            .def("__str__", &DistanceConditionalResult::to_string)
            .def("__repr__", [](const DistanceConditionalResult& self) {
                return "<DistanceConditionalResult(target=" + std::to_string(self.target) + 
                       ", current=" + std::to_string(self.current) + ")>";
            });

        py::class_<RotationConditionalResult, DefinedConditionalResult, std::shared_ptr<RotationConditionalResult>>(
                m, "RotationConditionalResult",
                "Represents a rotation-based condition")
            .def(py::init<const float>(), py::arg("target"), "Initialize with a rotation target")
            .def("__str__", &RotationConditionalResult::to_string)
            .def("__repr__", [](const RotationConditionalResult& self) {
                return "<RotationConditionalResult(target=" + std::to_string(self.target) + 
                       ", current=" + std::to_string(self.current) + ")>";
            });

        py::class_<MotorTicksConditionalResult, DefinedConditionalResult, std::shared_ptr<MotorTicksConditionalResult>>(
                m, "MotorTicksConditionalResult",
                "Represents a motor ticks-based condition")
            .def(py::init<const float>(), py::arg("target"), "Initialize with a motor ticks target")
            .def("__str__", &MotorTicksConditionalResult::to_string)
            .def("__repr__", [](const MotorTicksConditionalResult& self) {
                return "<MotorTicksConditionalResult(target=" + std::to_string(self.target) + 
                       ", current=" + std::to_string(self.current) + ")>";
            });

        py::class_<TimedConditionalResult, DefinedConditionalResult, std::shared_ptr<TimedConditionalResult>>(
                m, "TimedConditionalResult",
                "Represents a time-based condition")
            .def(py::init<const float, const float>(), py::arg("target"), py::arg("current"),
                 "Initialize with a time target and current time")
            .def("__str__", &TimedConditionalResult::to_string)
            .def("__repr__", [](const TimedConditionalResult& self) {
                return "<TimedConditionalResult(target=" + std::to_string(self.target) + 
                       ", current=" + std::to_string(self.current) + ")>";
            });
    }

    inline void createFunctionsBindings(py::module_& m)
    {
        m.def("for_time", forTime, R"pbdoc(Execute a function for a certain amount of time.
        Args:
            time (float): The duration in time units.
        Returns:
            None
        )pbdoc", py::arg("time"));

        m.def("for_seconds", forSeconds, R"pbdoc(Execute a function for a certain amount of seconds.
        Args:
            seconds (float): The duration in seconds.
        Returns:
            None
        )pbdoc", py::arg("seconds"));

        m.def("for_ticks", &forTicks, R"pbdoc(Execute a function for a certain amount of ticks.
        Args:
            ticks (int): The number of ticks.
        Returns:
            ConditionalFunction: A function that returns a MotorTicksConditionalResult.

        )pbdoc", py::arg("ticks"));

        m.def("for_distance", forDistance, R"pbdoc(Execute a function for a certain distance.
        Args:
            distance (float): The distance to cover.
        Returns:
            None
        )pbdoc", py::arg("distance"));

        m.def("for_cw_rotation", forCWRotation, R"pbdoc(Execute a function for a clockwise rotation.
        Args:
            rotation_degrees (float): The rotation in degrees.
        Returns:
            None
        )pbdoc", py::arg("rotation_degrees"));

        m.def("for_ccw_rotation", forCCWRotation, R"pbdoc(Execute a function for a counter-clockwise rotation.
        Args:
            rotation_degrees (float): The rotation in degrees.
        Returns:
            None
        )pbdoc", py::arg("rotation_degrees"));

        m.def("while_true", whileTrue, R"pbdoc(Execute a function while a condition is true.
        Args:
            condition (function): The condition to evaluate.
        Returns:
            None
        )pbdoc", py::arg("condition"));

        m.def("while_false", whileFalse, R"pbdoc(Execute a function while a condition is false.
        Args:
            condition (function): The condition to evaluate.
        Returns:
            None
        )pbdoc", py::arg("condition"));

        m.def("generator", generator, R"pbdoc(Return a speed generator function.
        Args:
            generator (function): The generator function.
        Returns:
            function: A speed generator function.
        )pbdoc", py::arg("generator"), py::return_value_policy::reference);

        m.def("constant", constant, R"pbdoc(Return a constant speed function.
        Args:
            speed (float): The constant speed.
        Returns:
            function: A constant speed function.
        )pbdoc", py::arg("speed"));

        m.def("lerp", lerp, R"pbdoc(Return a linearly interpolated speed function.
        Args:
            start_speed (float): The initial speed.
            end_speed (float): The final speed.
        Returns:
            function: A linearly interpolated speed function.
        )pbdoc", py::arg("start_speed"), py::arg("end_speed"));
    }

    inline void createSpeedBindings(const py::module& m)
    {
        py::class_<Speed>(m, "Speed", "Represents a speed with forward and angular components")
            .def(py::init<float, float>(),
                 py::arg("forward_speed_m_per_s"), py::arg("angular_speed_deg_per_s"),
                 "Initialize with forward and angular speeds")
            .def(py::init<float, float, float>(),
                 py::arg("forward_speed_m_per_s"), py::arg("strafe_speed_m_per_s"), py::arg("angular_speed_deg_per_s"),
                 "Initialize with forward, strafe, and angular speeds")
            .def_property_readonly("forward_percent", [](const Speed& self) { return self.forwardPercent; },
                                   "Get the forward speed as a percentage")
            .def_property_readonly("angular_percent", [](const Speed& self) { return self.angularPercent; },
                                   "Get the angular speed as a percentage")
            .def_property_readonly("strafe_percent", [](const Speed& self) { return self.strafePercent; },
                                   "Get the strafe speed as a percentage")
            .def("backward", &Speed::backward, "Get a speed representing backward movement")
            .def_static("wheels", &Speed::wheels, "Create a speed based on wheel velocities")
            .def_static("stop", &Speed::stop, "Get a speed representing no movement")
            .def("__str__", [](const Speed& self) {
                return "Speed(forward=" + std::to_string(self.forwardPercent) + 
                       ", strafe=" + std::to_string(self.strafePercent) + 
                       ", angular=" + std::to_string(self.angularPercent) + ")";
            })
            .def("__repr__", [](const Speed& self) {
                return "<Speed(forward_percent=" + std::to_string(self.forwardPercent) + 
                       ", strafe_percent=" + std::to_string(self.strafePercent) + 
                       ", angular_percent=" + std::to_string(self.angularPercent) + ")>";
            })
            .def_readonly_static("Slowest", &Speed::Slowest, "Predefined slowest speed")
            .def_readonly_static("Slow", &Speed::Slow, "Predefined slow speed")
            .def_readonly_static("Medium", &Speed::Medium, "Predefined medium speed")
            .def_readonly_static("Fast", &Speed::Fast, "Predefined fast speed")
            .def_readonly_static("Fastest", &Speed::Fastest, "Predefined fastest speed");
    }
}

