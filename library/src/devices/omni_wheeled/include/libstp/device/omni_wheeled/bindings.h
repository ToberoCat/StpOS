//
// Created by tobias on 1/10/25.
//

#pragma once

#include <pybind11/pybind11.h>
#include "omni_wheeled_device.h"
#include "omni_wheeled_calibration.h"

namespace py = pybind11;

namespace libstp::device::omni_wheeled
{
    inline void createOmniWheeledBindings(py::module_& m)
    {
        py::class_<OmniWheeledDevice, Device, std::shared_ptr<OmniWheeledDevice>>(m, "OmniWheeledNativeDevice", R"pbdoc(
            Represents a four-wheeled omni robot device.
        )pbdoc")
            .def(py::init<datatype::Axis, datatype::Direction, const motor::Motor&, const motor::Motor&, const
                          motor::Motor&, const
                          motor::Motor&>(),
                 py::arg("orientation"),
                 py::arg("direction"),
                 py::arg("front_left_motor"),
                 py::arg("front_right_motor"),
                 py::arg("rear_left_motor"),
                 py::arg("rear_right_motor"),
                 R"pbdoc(
                Initializes an OmniWheeledDevice instance.
                Motors should drive forward with a positive velocity. Forward is the direction the wombat's ports point 

                Args:
                    orientation (Axis): The axis of orientation for IMU.
                    direction (Direction): The direction of the robot.
                    front_left_motor (Motor): The motor for the front-left wheel.
                    front_right_motor (Motor): The motor for the front-right wheel.
                    rear_left_motor (Motor): The motor for the rear-left wheel.
                    rear_right_motor (Motor): The motor for the rear-right wheel.
            )pbdoc")
            .def("strafe", static_cast<async::AsyncAlgorithm<int> (OmniWheeledDevice::*)(datatype::ConditionalFunction,
                     const float& strafeAngle,
                     const float& speedPercent)>(&
                     OmniWheeledDevice::strafe), R"pbdoc(
            Strafes the robot at a given speed in a given direction.
    
            Args:
                condition (function): The condition to evaluate.
                strafe_angle (float): The angle to strafe at.
                speed (float): The speed to strafe at.
        )pbdoc", py::arg("condition"), py::arg("strafe_angle"), py::arg("speed"))
            .def("strafe", static_cast<async::AsyncAlgorithm<int> (OmniWheeledDevice::*)(datatype::ConditionalFunction,
                     datatype::SpeedFunction)>(&
                     OmniWheeledDevice::strafe), R"pbdoc(
            Strafes the robot at a given speed.
    
            Args:
                condition (function): The condition to evaluate.
                speedFunction (float): The speed to strafe at.
        )pbdoc", py::arg("condition"), py::arg("speedFunction"))
            .def("strafe", static_cast<async::AsyncAlgorithm<int> (OmniWheeledDevice::*)(datatype::ConditionalFunction,
                     datatype::Speed)>(&
                     OmniWheeledDevice::strafe), R"pbdoc(
            Strafes the robot at a given speed.

            Args:
                condition (function): The condition to evaluate.
                speed (float): The speed to strafe at.
            )pbdoc", py::arg("condition"), py::arg("speed"))
            .def("strafe", static_cast<async::AsyncAlgorithm<int> (OmniWheeledDevice::*)(datatype::ConditionalFunction,
                     std::function<float()> strafeAngle,
                     const float& speedPercent)>(&
                     OmniWheeledDevice::strafe), R"pbdoc(
            Strafes the robot at a given speed.

            Args:
                condition (function): The condition to evaluate.
                speed_angle_function (function): The angle to strafe at as a function.
                speed_percent (float): The speed to strafe at.
            )pbdoc", py::arg("condition"), py::arg("strafe_angle"), py::arg("speed_percent"))
            .def("speed_by_wheel_sides", &OmniWheeledDevice::speedByWheelSides, R"pbdoc(
            Calculates the speed of the robot based on the speed of the left and right wheels.
        )pbdoc", py::arg("left_speed"), py::arg("right_speed"))
            .def("speed_by_wheels", &OmniWheeledDevice::speedByWheels, R"pbdoc(
            Calculates the speed of the robot based on the speed of the individual wheels.
        )pbdoc", py::arg("wheel_speeds"))
            .def_readwrite("ticks_per_revolution", &OmniWheeledDevice::ticksPerRevolution, R"pbdoc(
                The number of ticks per wheel revolution (must be calibrated).
            )pbdoc")
            .def_readwrite("wheel_radius", &OmniWheeledDevice::wheelRadius, R"pbdoc(
                The radius of the omni wheels in meters.
            )pbdoc")
            .def_readwrite("wheel_distance_from_center", &OmniWheeledDevice::wheelDistanceFromCenter, R"pbdoc(
                Distance (in meters) between front and rear wheel axes.
            )pbdoc");

        m.def("calibrate_ticks_per_revolution", &calibrateTicksPerRevolution,
              py::arg("device"), py::arg("covered_distance"), py::arg("max_retries") = 5, R"pbdoc(
            Calibrates the ticks per revolution for an omni-wheeled device.

            Args:
                device (OmniWheeledDevice): The omni-wheeled device to calibrate.
                covered_distance (float): The distance covered in meters.
                max_retries (int): The maximum number of retries.
        )pbdoc");
    }
}
