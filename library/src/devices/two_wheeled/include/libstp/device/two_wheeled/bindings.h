//
// Created by tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>
#include "two_wheeled_device.h"
#include "two_wheeled_calibration.h"

namespace py = pybind11;

namespace libstp::device::two_wheeled
{
    inline void createTwoWheeledBindings(py::module_& m)
    {
        py::class_<TwoWheeledDevice, Device, std::shared_ptr<TwoWheeledDevice>>(m, "TwoWheeledNativeDevice", R"pbdoc(
            Represents a two-wheeled robot device.
        )pbdoc")
            .def(py::init<datatype::Axis, datatype::Direction, const motor::Motor&, const motor::Motor&>(),
                 py::arg("orientation"), py::arg("direction"), py::arg("left_motor"), py::arg("right_motor"), R"pbdoc(
                Initializes a TwoWheeledDevice instance.

                   Make sure both motors drive with the set_velocity(500) command the robot **forward**.

                Args:
                    orientation (Axis): The axis of orientation.
                    direction (Direction): The direction of the robot.
                    left_motor (Motor): The motor for the left wheel.
                    right_motor (Motor): The motor for the right wheel.
            )pbdoc")

            .def_readwrite("ticks_per_revolution", &TwoWheeledDevice::ticksPerRevolution, R"pbdoc(
                The number of ticks per wheel revolution. This must be calibrated.
            )pbdoc")

            .def_readwrite("wheel_base", &TwoWheeledDevice::wheelBase, R"pbdoc(
                The distance between the two wheels. This must be calibrated.
            )pbdoc")
            .def_readwrite("wheel_radius", &TwoWheeledDevice::wheelRadius, R"pbdoc(
                The radius of the wheels.
            )pbdoc");

        m.def("calibrate_ticks_per_revolution", &calibrateTicksPerRevolution, py::arg("device"),
              py::arg("covered_distance"), py::arg("max_retries") = 5, R"pbdoc(
            Calibrates the ticks per revolution for a two-wheeled device.

            Args:
                device (TwoWheeledDevice): The two-wheeled device to calibrate.
                covered_distance (float): The distance covered in meters.
                max_retries (int): The maximum number of retries.
        )pbdoc");

        m.def("calibrate_wheel_base", &calibrateWheelBase, py::arg("device"), py::arg("max_retries") = 5, R"pbdoc(
            Calibrates the wheelbase of a two-wheeled device.

            Args:
                device (TwoWheeledDevice): The two-wheeled device to calibrate.
                max_retries (int): The maximum number of retries.
        )pbdoc");
    }
}
