//
// Created by tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>

#include "drive_straight.h"
#include "line_follower.h"
#include "line_up.h"
#include "rotate.h"
#include "libstp/datatype/bindings.h"
#include "libstp/utility/constants.h"


namespace py = pybind11;

namespace libstp::motion
{
    template <typename PyClass>
    void createMotionBindings(PyClass& m)
    {
        m.def("rotate", [](typename PyClass::type& self, const datatype::ConditionalFunction& condition,
                           const datatype::SpeedFunction& speedFunction)
              {
                  return rotate(self, condition, speedFunction);
              }, R"pbdoc(Rotates the device while a condition is met.

            Args:
                device (Device): The device to rotate.
                condition (ConditionalFunction): A function that returns a ConditionalResult to determine if the condition is still met.
                speedFunction (SpeedFunction): A function that determines the current speed based on the ConditionalResult.)pbdoc",
              py::arg("condition"), py::arg("speedFunction"));

        m.def("rotate", [](typename PyClass::type& self, const datatype::ConditionalFunction& condition,
                           const datatype::Speed speed)
              {
                  return rotate(self, condition, constant(speed));
              }, R"pbdoc(Rotates the device while a condition is met.

            Args:
                device (Device): The device to rotate.
                condition (ConditionalFunction): A function that returns a ConditionalResult to determine if the condition is still met.
                speed (Speed): The constant speed the robot should try to rotate with)pbdoc",
              py::arg("condition"), py::arg("speed"));

        m.def("follow_line", [](typename PyClass::type& self,
                                sensor::LightSensor leftSensor,
                                sensor::LightSensor rightSensor,
                                const datatype::ConditionalFunction& condition,
                                datatype::SpeedFunction speedFunction)
              {
                  return follow_line(self, leftSensor, rightSensor, condition, speedFunction);
              }, R"pbdoc(Follows a line using light sensors while a condition is met.

            Args:
                device (Device): The device to control.
                leftSensor (LightSensor): The left light sensor.
                rightSensor (LightSensor): The right light sensor.
                condition (ConditionalFunction): A function that returns a ConditionalResult to determine if the condition is still met.
                speedFunction (SpeedFunction): A function that determines the current speed based on the ConditionalResult.)pbdoc",
              py::arg("leftSensor"), py::arg("rightSensor"), py::arg("condition"),
              py::arg("speedFunction"));

        m.def("follow_line", [](typename PyClass::type& self,
                                sensor::LightSensor leftSensor,
                                sensor::LightSensor rightSensor,
                                const datatype::ConditionalFunction& condition,
                                const datatype::Speed speed)
              {
                  return follow_line(self, leftSensor, rightSensor, condition, constant(speed));
              }, R"pbdoc(Follows a line using light sensors while a condition is met.

            Args:
                device (Device): The device to control.
                leftSensor (LightSensor): The left light sensor.
                rightSensor (LightSensor): The right light sensor.
                condition (ConditionalFunction): A function that returns a ConditionalResult to determine if the condition is still met.
                speed (Speed): The constant speed the robot should try to drive with)pbdoc",
              py::arg("leftSensor"), py::arg("rightSensor"), py::arg("condition"),
              py::arg("speed"));

        m.def("drive_straight", [](typename PyClass::type& self, const datatype::ConditionalFunction& condition,
                                   const datatype::SpeedFunction& speedFunction)
              {
                  return drive_straight(self, condition, speedFunction);
              }, R"pbdoc(Drives the device straight while a condition is met.

            Args:
                device (Device): The device to control.
                condition (ConditionalFunction): A function that returns a ConditionalResult to determine if the condition is still met.
                speedFunction (SpeedFunction): A function that determines the current speed based on the ConditionalResult.)pbdoc",
              py::arg("condition"), py::arg("speedFunction"));

        m.def("drive_straight", [](typename PyClass::type& self, const datatype::ConditionalFunction& condition,
                                   const datatype::Speed& speed)
              {
                  return drive_straight(self, condition, constant(speed));
              }, R"pbdoc(Drives the device straight while a condition is met.

            Args:
                device (Device): The device to control.
                condition (ConditionalFunction): A function that returns a ConditionalResult to determine if the condition is still met.
                speed (Speed): The constant speed the robot should try to drive with)pbdoc",
              py::arg("condition"), py::arg("speed"));

        m.def("forward_line_up", [](typename PyClass::type& self, sensor::LightSensor& left_sensor,
                                    sensor::LightSensor& right_sensor)
              {
                  return forward_line_up(self, left_sensor, right_sensor);
              }, R"pbdoc(Lines up the device forward using light sensors.

            Args:
                device (Device): The device to control.
                left_sensor (LightSensor): The left light sensor.
                right_sensor (LightSensor): The right light sensor.)pbdoc",
              py::arg("left_sensor"), py::arg("right_sensor"));

        m.def("backward_line_up", [](typename PyClass::type& self, sensor::LightSensor& left_sensor,
                                     sensor::LightSensor& right_sensor)
              {
                  return backward_line_up(self, left_sensor, right_sensor);
              }, R"pbdoc(Lines up the device backward using light sensors.

            Args:
                device (Device): The device to control.
                left_sensor (LightSensor): The left light sensor.
                right_sensor (LightSensor): The right light sensor.)pbdoc",
              py::arg("left_sensor"), py::arg("right_sensor"));
    }
}
