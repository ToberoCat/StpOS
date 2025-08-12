//
// Created by tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "imu.h"
#include "sensor.h"
#include "ir_light_sensor.h"

namespace py = pybind11;

namespace libstp::sensor
{
    inline void createSensorBindings(py::module_& m)
    {
        py::class_<Sensor, std::shared_ptr<Sensor>>(m, "Sensor", R"pbdoc(
            Represents a generic sensor.
        )pbdoc")
            .def(py::init<int>(), py::arg("port"), R"pbdoc(
                Initializes a Sensor instance.

                Args:
                    port (int): The port number of the sensor.
            )pbdoc")
            .def("get_value", &Sensor::getValue, R"pbdoc(
                Gets the current value of the sensor.

                Returns:
                    int: The sensor's current value.
            )pbdoc")
            .def("get_port", &Sensor::getPort, R"pbdoc(
                Gets the port number of the sensor.

                Returns:
                    int: The port number.
            )pbdoc");

        py::class_<AnalogSensor, Sensor, std::shared_ptr<AnalogSensor>>(m, "AnalogSensor", R"pbdoc(
            Represents an analog sensor.
        )pbdoc")
            .def(py::init<int>(), py::arg("port"), R"pbdoc(
                Initializes an AnalogSensor instance.

                Args:
                    port (int): The port number of the sensor.
            )pbdoc");

        py::class_<DigitalSensor, Sensor, std::shared_ptr<DigitalSensor>>(m, "DigitalSensor", R"pbdoc(
            Represents a digital sensor.
        )pbdoc")
            .def(py::init<int>(), py::arg("port"), R"pbdoc(
                Initializes a DigitalSensor instance.

                Args:
                    port (int): The port number of the sensor.
            )pbdoc")
            .def("is_clicked", &DigitalSensor::isClicked, R"pbdoc(
                Checks if the sensor is clicked.

                Returns:
                    bool: True if clicked, False otherwise.
            )pbdoc");

        py::class_<ValueSensor, Sensor, std::shared_ptr<ValueSensor>>(m, "ValueSensor", R"pbdoc(
            Represents a value sensor using a custom data provider.
        )pbdoc")
            .def(py::init<std::function<int()>>(), py::arg("provider"), R"pbdoc(
                Initializes a ValueSensor instance with a data provider.

                Args:
                    provider (Callable): A function providing the sensor value.
            )pbdoc");

        py::class_<DistanceSensor, AnalogSensor, std::shared_ptr<DistanceSensor>>(m, "DistanceSensor", R"pbdoc(
            Represents a distance sensor based on analog measurements.
        )pbdoc")
            .def(py::init<int>(), py::arg("port"), R"pbdoc(
                Initializes a DistanceSensor instance.

                Args:
                    port (int): The port number of the sensor.
            )pbdoc")
            .def("get_distance", &DistanceSensor::getDistance, R"pbdoc(
                Gets the measured distance.

                Returns:
                    float: The measured distance in appropriate units.
            )pbdoc");

        py::class_<LightSensor, AnalogSensor, std::shared_ptr<LightSensor>>(m, "LightSensor", R"pbdoc(
            Represents a light sensor with calibration functionality.
        )pbdoc")
            .def(py::init<int, float>(), py::arg("port"), py::arg("calibration_factor") = 0.5, R"pbdoc(
                Initializes a LightSensor instance.

                Args:
                    port (int): The port number of the sensor.
                    calibration_factor (float): The calibration factor.
            )pbdoc")
            .def_property("white_threshold", [](const LightSensor& sensor) { return sensor.whiteThreshold; },
                          [](LightSensor& sensor, const int threshold) { sensor.whiteThreshold = threshold; })
            .def_property("black_threshold", [](const LightSensor& sensor) { return sensor.blackThreshold; },
                          [](LightSensor& sensor, const int threshold) { sensor.blackThreshold = threshold; })
            .def_property("white_mean", [](const LightSensor& sensor) { return sensor.whiteMean; },
                          [](LightSensor& sensor, const float mean) { sensor.whiteMean = mean; })
            .def_property("black_mean", [](const LightSensor& sensor) { return sensor.blackMean; },
                          [](LightSensor& sensor, const float mean) { sensor.blackMean = mean; })
            .def_property("white_std_dev", [](const LightSensor& sensor) { return sensor.whiteStdDev; },
                          [](LightSensor& sensor, const float std_dev) { sensor.whiteStdDev = std_dev; })
            .def_property("black_std_dev", [](const LightSensor& sensor) { return sensor.blackStdDev; },
                          [](LightSensor& sensor, const float std_dev) { sensor.blackStdDev = std_dev; })
            .def("probability_of_black", &LightSensor::probabilityOfBlack, R"pbdoc(
                Calculates the probability of the sensor being on a black surface.

                Returns:
                    float: The probability of being on black.
            )pbdoc")
            .def("probability_of_white", &LightSensor::probabilityOfWhite, R"pbdoc(
                Calculates the probability of the sensor being on a white surface.

                Returns:
                    float: The probability of being on white.
            )pbdoc")
            .def("calibrate", &LightSensor::calibrate, py::arg("white_value"), py::arg("black_value"), R"pbdoc(
                Calibrates the light sensor.

                Args:
                    white_value (int): The white calibration value.
                    black_value (int): The black calibration value.

                Returns:
                    bool: True if successful, False otherwise.
            )pbdoc")
            .def("wait_for_light", &LightSensor::wait_for_light, R"pbdoc(
                Waits for the light sensor to detect light.
            )pbdoc")
            .def("is_on_white", &LightSensor::isOnWhite, R"pbdoc(
                Checks if the sensor is on a white surface.

                Returns:
                    bool: True if on white, False otherwise.
            )pbdoc")
            .def("is_on_black", &LightSensor::isOnBlack, R"pbdoc(
                Checks if the sensor is on a black surface.

                Returns:
                    bool: True if on black, False otherwise.
            )pbdoc");

        py::class_<IrLightSensor, LightSensor, std::shared_ptr<IrLightSensor>>(m, "IrLightSensor", R"pbdoc(
            Represents an IR light sensor with enhanced calibration functionality.
        )pbdoc")
            .def(py::init<int, float>(), py::arg("port"), py::arg("calibration_factor") = 0.3f, R"pbdoc(
                Initializes an IrLightSensor instance.

                Args:
                    port (int): The port number of the sensor.
                    calibration_factor (float): The calibration factor.
            )pbdoc");

        m.def("calibrate_light_sensors", &calibrateLightSensors, py::arg("light_sensors"), R"pbdoc(
            Calibrates a list of light sensors.

            Args:
                light_sensors (List[LightSensor]): The light sensors to calibrate.
        )pbdoc");

        m.def("are_on_black", &areOnBlack, py::arg("left_sensor"), py::arg("right_sensor"), R"pbdoc(
            Checks if both sensors are on black.

            Args:
                left_sensor (LightSensor): The left light sensor.
                right_sensor (LightSensor): The right light sensor.

            Returns:
                bool: True if both are on black, False otherwise.
        )pbdoc");

        m.def("are_on_white", &areOnWhite, py::arg("left_sensor"), py::arg("right_sensor"), R"pbdoc(
            Checks if both sensors are on white.

            Args:
                left_sensor (LightSensor): The left light sensor.
                right_sensor (LightSensor): The right light sensor.

            Returns:
                bool: True if both are on white, False otherwise.
        )pbdoc");

        m.def("wait_for_button_click", &waitForButtonClick, R"pbdoc(
            Waits for a button click.
        )pbdoc");


        m.def("is_button_clicked", &isButtonClicked, R"pbdoc(
            Checks if a button is clicked.

            Returns:
                bool: True if clicked, False otherwise.
        )pbdoc");
    }

    inline void createImuSensorBindings(const py::module_& m)
    {
        py::class_<GyroSensor>(m, "GyroSensor")
            .def(py::init<>())
            .def("calibrate", [](GyroSensor& self, const std::vector<std::vector<double>>& calibMatrix) {
                int rows = calibMatrix.size();
                int cols = rows > 0 ? calibMatrix[0].size() : 0;
                MatrixX3d eigenMatrix(rows, 3);
                for(int i = 0; i < rows; i++) {
                    for(int j = 0; j < std::min(3, (int)calibMatrix[i].size()); j++) {
                        eigenMatrix(i, j) = calibMatrix[i][j];
                    }
                }
                self.calibrate(std::make_shared<MatrixX3d>(eigenMatrix));
            }, "Calibrate the gyroscope sensor")
            .def("get_value", [](const GyroSensor& self) {
                auto val = self.getValue();
                return std::make_tuple((*val)[0], (*val)[1], (*val)[2]);
            }, "Get the calibrated gyroscope value as (x,y,z) tuple")
            .def("get_variance", [](GyroSensor& self) {
                auto var = self.getVariance();
                return std::make_tuple((*var)[0], (*var)[1], (*var)[2]);
            }, "Get the variance of the gyroscope readings as (x,y,z) tuple")
            .def("get_bias", [](GyroSensor& self) {
                auto bias = self.getBias();
                return std::make_tuple((*bias)[0], (*bias)[1], (*bias)[2]);
            }, "Get the gyroscope bias as (x,y,z) tuple");

        py::class_<AccelSensor>(m, "AccelSensor")
            .def(py::init<>())
            .def("calibrate", [](AccelSensor& self, const std::vector<std::vector<double>>& calibMatrix) {
                int rows = calibMatrix.size();
                int cols = rows > 0 ? calibMatrix[0].size() : 0;
                MatrixX3d eigenMatrix(rows, 3);
                for(int i = 0; i < rows; i++) {
                    for(int j = 0; j < std::min(3, (int)calibMatrix[i].size()); j++) {
                        eigenMatrix(i, j) = calibMatrix[i][j];
                    }
                }
                self.calibrate(std::make_shared<MatrixX3d>(eigenMatrix));
            }, "Calibrate the accelerometer sensor")
            .def("get_value", [](const AccelSensor& self) {
                auto val = self.getValue();
                return std::make_tuple((*val)[0], (*val)[1], (*val)[2]);
            }, "Get the calibrated accelerometer value as (x,y,z) tuple")
            .def("get_variance", [](AccelSensor& self) {
                auto var = self.getVariance();
                return std::make_tuple((*var)[0], (*var)[1], (*var)[2]);
            }, "Get the variance of the accelerometer readings as (x,y,z) tuple")
            .def("get_bias", [](AccelSensor& self) {
                auto bias = self.getBias();
                return std::make_tuple((*bias)[0], (*bias)[1], (*bias)[2]);
            }, "Get the accelerometer bias as (x,y,z) tuple")
            .def("get_gravity", [](AccelSensor& self) {
                auto gravity = self.getGravity();
                return std::make_tuple((*gravity)[0], (*gravity)[1], (*gravity)[2]);
            }, "Get the measured gravity vector as (x,y,z) tuple");

        py::class_<MagnetoSensor>(m, "MagnetoSensor")
            .def(py::init<>())
            .def("calibrate", [](MagnetoSensor& self, const std::vector<std::vector<double>>& calibMatrix) {
                int rows = calibMatrix.size();
                int cols = rows > 0 ? calibMatrix[0].size() : 0;
                MatrixX3d eigenMatrix(rows, 3);
                for(int i = 0; i < rows; i++) {
                    for(int j = 0; j < std::min(3, (int)calibMatrix[i].size()); j++) {
                        eigenMatrix(i, j) = calibMatrix[i][j];
                    }
                }
                self.calibrate(std::make_shared<MatrixX3d>(eigenMatrix));
            }, "Calibrate the magnetometer sensor")
            .def("set_hard_iron_offset", [](MagnetoSensor& self, const std::tuple<double, double, double>& offset) {
                Vector3d vec;
                vec << std::get<0>(offset), std::get<1>(offset), std::get<2>(offset);
                self.setHardIronOffset(std::make_shared<Vector3d>(vec));
            }, "Set the hard iron offset for magnetometer calibration")
            .def("set_soft_iron_matrix", [](MagnetoSensor& self, const std::vector<std::vector<double>>& matrix) {
                Matrix3d eigenMatrix = Matrix3d::Identity();
                for(int i = 0; i < std::min(3, (int)matrix.size()); i++) {
                    for(int j = 0; j < std::min(3, (int)matrix[i].size()); j++) {
                        eigenMatrix(i, j) = matrix[i][j];
                    }
                }
                self.setSoftIronMatrix(std::make_shared<Matrix3d>(eigenMatrix));
            }, "Set the soft iron matrix for magnetometer calibration")
            .def("get_value", [](const MagnetoSensor& self) {
                auto val = self.getValue();
                return std::make_tuple((*val)[0], (*val)[1], (*val)[2]);
            }, "Get the calibrated magnetometer value as (x,y,z) tuple")
            .def("get_variance", [](MagnetoSensor& self) {
                auto var = self.getVariance();
                return std::make_tuple((*var)[0], (*var)[1], (*var)[2]);
            }, "Get the variance of the magnetometer readings as (x,y,z) tuple");

        py::class_<IMU>(m, "IMU")
            .def(py::init<>())
            .def_readonly("gyro", &IMU::gyro)
            .def_readonly("accel", &IMU::accel)
            .def_readonly("magneto", &IMU::magneto)
            .def("calibrate", &IMU::calibrate, "Perform IMU calibration with a given sample count")
            .def("get_reading", [](const IMU& self) {
                auto [gyro, accel, magneto] = self.getReading();
                return std::make_tuple(
                    std::make_tuple((*gyro)[0], (*gyro)[1], (*gyro)[2]),
                    std::make_tuple((*accel)[0], (*accel)[1], (*accel)[2]),
                    std::make_tuple((*magneto)[0], (*magneto)[1], (*magneto)[2])
                );
            }, "Get sensor readings as a tuple of (gyro, accel, magneto) where each is a (x,y,z) tuple");
    }
}

