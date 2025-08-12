//
// Created by Tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>
#include "device.h"
#include "libstp/motion/bindings.h"

namespace py = pybind11;

namespace libstp::device
{
    inline void createDeviceBindings(py::module_& m)
    {
        auto device = py::class_<Device, std::shared_ptr<Device>>(m, "NativeDevice",
                                                                  R"pbdoc(
            Represents a generic device with control over speed and shutdown functionality.
            This class serves as an abstract base for specific device implementations.

            Example:
                >>> device = NativeDevice(Axis.X, Direction.Forward)
                >>> device.set_speed_while(while_true(lambda: True), Speed(0.3, 0.0, 0.0))  # Move at 30% speed while condition holds
            )pbdoc")
                      .def(py::init<datatype::Axis, datatype::Direction>(),
                           py::arg("orientation"), py::arg("direction"),
                           R"pbdoc(
                Initializes a new instance of the Device class.

                Args:
                    orientation (Axis): The orientation of the device.
                    direction (Direction): The movement direction.

                Returns:
                    NativeDevice: A new instance of the device.

                Example:
                    >>> device = NativeDevice(Axis.Z, Direction.Forward)
                )pbdoc")
                      .def("reset_ramps", &Device::resetRamps, R"pbdoc("
                Resets the speed ramps)pbdoc")

                      // Lifecycle methods
                      .def("shutdown", &Device::shutdown,
                           R"pbdoc(
                Gracefully shuts down the device and performs necessary cleanup.

                Returns:
                    None

                Example:
                    >>> device.shutdown()
                )pbdoc")

                      // Speed control functions
                      .def("set_speed_while",
                           py::overload_cast<datatype::ConditionalFunction, datatype::SpeedFunction, bool, bool, bool>(
                               &Device::setSpeedWhile),
                           py::arg("condition"),
                           py::arg("speed_function"),
                           py::arg("do_correction") = true,
                           py::arg("auto_stop_device") = true,
                           py::arg("reset_ramps") = true,
                           R"pbdoc(
                 Adjusts the device speed dynamically while a condition is met.

                 Args:
                     condition (ConditionalFunction): Function returning a ConditionalResult to evaluate.
                     speed_function (SpeedFunction): Function to determine the speed dynamically.
                     do_correction (bool): Whether to apply correction to the speed based on the gyroscope

                 Returns:
                     AsyncAlgorithm[int]: An async object managing speed adjustments.

                 Example:
                     >>> device.set_speed_while(while_true(lambda: True), lambda result: Speed(0.3, 0.0, 0.0))  # Speed follows function while True
                )pbdoc")
                      .def_readonly("imu", &Device::imu, R"pbdoc(
                The IMU sensor attached to the device.)pbdoc")
                      .def("set_speed_while",
                           py::overload_cast<datatype::ConditionalFunction, datatype::Speed>(&Device::setSpeedWhile),
                           py::arg("condition"), py::arg("constant_speed"),
                           R"pbdoc(
                 Sets the device speed to a constant value while a condition is met.

                 Args:
                     condition (ConditionalFunction): Function returning a ConditionalResult.
                     constant_speed (Speed): Fixed speed to apply while condition holds.

                 Returns:
                     AsyncAlgorithm[int]: An async object handling speed adjustments.

                 Example:
                     >>> device.set_speed_while(lambda: sensor.is_moving(), 15.0)
                )pbdoc")

                      .def("drive_arc", &Device::driveArc,
                           py::arg("condition"),
                           py::arg("radius_cm"),
                           py::arg("max_forward_percentage"),
                           py::arg("direction"),
                           R"pbdoc(
                Moves the device in an arc trajectory for a specified angle.

                Args:
                    condition (ConditionalFunction): Condition to continue movement.
                    radius_cm (float): Radius of the arc in centimeters.
                    max_forward_percentage (float): Maximum forward speed percentage.
                    direction (Direction): Direction of the arc.

                Returns:
                    AsyncAlgorithm[int]: An async object tracking execution.

                Example:
                    >>> device.drive_arc_degrees(while_true(lambda: True), 50, 0.8, Direction.Forward)
                )pbdoc")

                      // Kinematics & PID control
                      .def("__apply_kinematics_model__", &Device::debugApplyKinematicsModel,
                           py::arg("forward"), py::arg("strafe"), py::arg("angular"),
                           R"pbdoc(
                Applies the kinematic model to adjust the device movement.

                Args:
                    forward (float): Forward velocity.
                    strafe (float): Strafe velocity.
                    angular (float): Angular velocity.

                Returns:
                    None

                Example:
                    >>> device.__apply_kinematics_model__(1.0, 0.5, 0.2)
                )pbdoc")

                      .def("set_vx_pid", &Device::setVxPid, py::arg("kp"), py::arg("ki"), py::arg("kd"),
                           R"pbdoc(
                Sets PID gains for the forward speed controller.

                Args:
                    kp (float): Proportional gain.
                    ki (float): Integral gain.
                    kd (float): Derivative gain.

                Returns:
                    None

                Example:
                    >>> device.set_vx_pid(1.2, 0.01, 0.05)
                )pbdoc")

                      .def("set_vy_pid", &Device::setVyPid, py::arg("kp"), py::arg("ki"), py::arg("kd"),
                           R"pbdoc(
                Sets PID gains for the strafing speed controller.

                Args:
                    kp (float): Proportional gain.
                    ki (float): Integral gain.
                    kd (float): Derivative gain.

                Returns:
                    None

                Example:
                    >>> device.set_vy_pid(1.0, 0.02, 0.03)
                )pbdoc")

                      .def("set_w_pid", &Device::setWPid, py::arg("kp"), py::arg("ki"), py::arg("kd"),
                           R"pbdoc(
                Sets PID gains for the angular speed controller.

                Args:
                    kp (float): Proportional gain.
                    ki (float): Integral gain.
                    kd (float): Derivative gain.

                Returns:
                    None

                Example:
                    >>> device.set_w_pid(1.5, 0.05, 0.02)
                )pbdoc")

                      .def("set_heading_pid", &Device::setHeadingPid, py::arg("kp"), py::arg("ki"), py::arg("kd"),
                           R"pbdoc(
                Sets PID gains for the heading control system.

                Args:
                    kp (float): Proportional gain.
                    ki (float): Integral gain.
                    kd (float): Derivative gain.

                Returns:
                    None

                Example:
                    >>> device.set_heading_pid(2.0, 0.03, 0.01)
                )pbdoc")

                      .def("set_max_accel", &Device::setMaxAccel, py::arg("max_forward_accel"),
                           py::arg("max_strafe_accel"), py::arg("max_angular_accel"),
                           R"pbdoc(
                Sets the maximum acceleration values for the device.

                Args:
                    max_forward_accel (float): Maximum forward acceleration.
                    max_strafe_accel (float): Maximum strafing acceleration.
                    max_angular_accel (float): Maximum angular acceleration.

                Returns:
                    None

                Example:
                    >>> device.set_max_accel(0.5, 0.5, 1.0)
                )pbdoc")

                      .def("reset_state", &Device::resetState,
                           R"pbdoc(
                Resets the device state to its initial values.

                Returns:
                    None

                Example:
                    >>> device.reset_state()
                )pbdoc")

                      // Sensor readings
                      .def("set_quaternion", &Device::setQuaternion, py::arg("w"), py::arg("x"), py::arg("y"),
                           py::arg("z"),
                           R"pbdoc(
                Sets the quaternion values for the device orientation.

                Args:
                    w (float): W component of the quaternion.
                    x (float): X component of the quaternion.
                    y (float): Y component of the quaternion.
                    z (float): Z component of the quaternion.

                Returns:
                    None

                Example:
                    >>> device.set_quaternion(0.5, 0.5, 0.5, 0.5)
                )pbdoc")
                      .def("stop", &Device::stopDevice,
                           R"pbdoc(
                Stops the device and halts all movement.

                Returns:
                    None

                Example:
                    >>> device.stop_device()
                )pbdoc")
                      .def("get_max_speeds", &Device::computeMaxSpeeds,
                           R"pbdoc(
                Gets the maximum speeds for the device.
                )pbdoc")
                      .def("get_current_heading", &Device::getCurrentHeading,
                           R"pbdoc(
                             Gets the current heading of the device.
                             )pbdoc")
                      .def("to_absolute_speed", &Device::toAbsoluteSpeed,
                           R"pdoc(
                Converts a Speed object to an AbsoluteSpeed object.
                )pdoc")
                      .def("set_max_speeds", &Device::setMaxSpeeds)
                      // Context manager support in Python
                      .def("__enter__", [](Device& device) -> Device& { return device; })
                      .def("__exit__", [](Device& device, const py::object&, const py::object&, const py::object&)
                      {
                          device.shutdown();
                      });

        // Register motion bindings
        motion::createMotionBindings(device);
    }
}
