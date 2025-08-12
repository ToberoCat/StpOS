//
// Created by tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>

#include "libstp/math/math.h"

namespace py = pybind11;

namespace libstp::math
{
    inline void createMathBindings(py::module_& m)
    {
        m.attr("RAD_TO_DEG") = RAD_TO_DEG;
        m.attr("DEG_TO_RAD") = DEG_TO_RAD;
        m.attr("M_PIf") = M_PIf;

        m.def("lerp", &lerp, R"pbdoc(
            Linearly interpolates between two values.

            Args:
                a (float): The start value.
                b (float): The end value.
                t (float): The interpolation factor (0 <= t <= 1).

            Returns:
                float: The interpolated value.
        )pbdoc",
              py::arg("a"), py::arg("b"), py::arg("t"));

        m.def("ease_in_out", &easeInOut, R"pbdoc(
            Smoothly interpolates between two values with an ease-in-out curve.

            Args:
                a (float): The start value.
                b (float): The end value.
                t (float): The interpolation factor (0 <= t <= 1).

            Returns:
                float: The interpolated value with easing applied.
        )pbdoc",
              py::arg("a"), py::arg("b"), py::arg("t"));

        m.def("clamp_double", &clampDouble, R"pbdoc(
            Clamps a double value to the specified range.

            Args:
                value (float): The value to clamp.
                min (float): The lower bound.
                max (float): The upper bound.

            Returns:
                float: The clamped value.
        )pbdoc",
              py::arg("value"), py::arg("min"), py::arg("max"));

        m.def("clamp_int", &clampInt, R"pbdoc(
            Clamps an integer value to the specified range.

            Args:
                value (int): The value to clamp.
                min (int): The lower bound.
                max (int): The upper bound.

            Returns:
                int: The clamped value.
        )pbdoc",
              py::arg("value"), py::arg("min"), py::arg("max"));

        m.def("sign", &sign, R"pbdoc(
            Returns the sign of an integer value.

            Args:
                value (int): The input value.

            Returns:
                int: 1 if the value is positive, -1 if negative, and 0 if zero.
        )pbdoc",
              py::arg("value"));

        m.def("signf", &signf, R"pbdoc(
            Returns the sign of a floating-point value.

            Args:
                value (float): The input value.

            Returns:
                float: 1.0 if the value is positive, -1.0 if negative, and 0.0 if zero.
        )pbdoc",
              py::arg("value"));

        m.def("minimal_angle_difference", &minimalAngleDifference, R"pbdoc(
            Computes the shortest difference between two angles.

            Args:
                a (float): The first angle in degrees.
                b (float): The second angle in degrees.

            Returns:
                float: The shortest angle difference in degrees.
        )pbdoc",
              py::arg("a"), py::arg("b"));
    }
}
