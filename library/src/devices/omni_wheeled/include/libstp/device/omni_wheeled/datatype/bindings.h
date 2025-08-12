//
// Created by tobias on 1/12/25.
//
#pragma once

#include <pybind11/pybind11.h>

#include "omni_wheeld_conditions.h"
#include "omni_wheeled_functions.h"

namespace py = pybind11;

namespace libstp::device::omni_wheeled::datatype
{
    inline void createOmniWheeledDatatypeBindings(py::module_& m)
    {
        py::class_<ForwardDistanceConditionalResult, libstp::datatype::DistanceConditionalResult, std::shared_ptr<ForwardDistanceConditionalResult>>(
                m,
                "ForwardDistanceConditionalResult",
                "Represents a forward distance-based condition for omni-wheeled devices."
            )
            .def(py::init<const float>(),
                 py::arg("target_cm"),
                 "Initialize with a forward distance target in centimeters.");

        py::class_<SideDistanceConditionalResult, libstp::datatype::DistanceConditionalResult, std::shared_ptr<SideDistanceConditionalResult>>(
                m,
                "SideDistanceConditionalResult",
                "Represents a side (strafe) distance-based condition for omni-wheeled devices."
            )
            .def(py::init<const float>(),
                 py::arg("target_cm"),
                 "Initialize with a side distance target in centimeters.");

        m.def("for_forward_distance",
              &forForwardDistance,
              py::arg("distance_cm"),
              R"pbdoc(
                  Execute a function for a certain forward distance.

                  Args:
                      distance_cm (float): The distance to cover in centimeters.

                  Returns:
                      ConditionalFunction: A function that returns a ForwardDistanceConditionalResult.
              )pbdoc");

        m.def("for_side_distance",
              &forSideDistance,
              py::arg("distance_cm"),
              R"pbdoc(
                  Execute a function for a certain side (strafe) distance.

                  Args:
                      distance_cm (float): The distance to cover in centimeters.

                  Returns:
                      ConditionalFunction: A function that returns a SideDistanceConditionalResult.
              )pbdoc");
    }
}
