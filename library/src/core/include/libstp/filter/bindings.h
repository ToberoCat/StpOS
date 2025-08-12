//
// Created by tobias on 12/29/24.
//

#pragma once

#include <pybind11/pybind11.h>
#include "libstp/filter/filters.h"
#include <memory>

namespace py = pybind11;

namespace libstp::filter
{
    inline void createFilterBindings(const py::module_& m)
    {
        py::class_<Filter, std::shared_ptr<Filter>>(m, "Filter", R"pbdoc(
            Represents a generic filter with warm-up capabilities.

            This is an abstract base class for specific filter implementations.
        )pbdoc")
            .def(py::init<const unsigned int>(), py::arg("warm_up_cycles"), R"pbdoc(
                Initializes a new instance of the Filter class.

                Args:
                    warm_up_cycles (int): The number of cycles to warm up the filter.
            )pbdoc")

            .def("filter", &Filter::filter, R"pbdoc(
                Applies the filter to a data point.

                Args:
                    data_point (float): The input data point to filter.

                Returns:
                    float: The filtered data point.
            )pbdoc")

            .def("warmup", &Filter::warmup, R"pbdoc(
                Warms up the filter by processing a number of data points.

                Args:
                    warm_up_value (int): The value used to warm up the filter.
            )pbdoc")

            .def("__call__", &Filter::operator(), R"pbdoc(
                Applies the filter to a data point using the call operator.

                Args:
                    data_point (float): The input data point to filter.

                Returns:
                    float: The filtered data point.
            )pbdoc");

        py::class_<FunctionFilter, Filter, std::shared_ptr<FunctionFilter>>(m, "FunctionFilter", R"pbdoc(
            A filter that applies a user-defined function to data points.

            This filter allows custom filtering logic to be defined via a Python function.
        )pbdoc")
            .def(py::init<const std::function<double(double)>&>(), py::arg("filter_function"), R"pbdoc(
                Initializes a new instance of the FunctionFilter class.

                Args:
                    filter_function (function): A Python function that takes a float and returns a float.
            )pbdoc");

        py::class_<NoFilter, Filter, std::shared_ptr<NoFilter>>(m, "NoFilter", R"pbdoc(
            A filter that performs no filtering, returning data points as-is.
        )pbdoc")
            .def(py::init<>(), R"pbdoc(
                Initializes a new instance of the NoFilter class.
            )pbdoc");

        py::class_<AvgFilter, Filter, std::shared_ptr<AvgFilter>>(m, "AvgFilter", R"pbdoc(
            A filter that computes the average of incoming data points incrementally.
        )pbdoc")
            // Constructor
            .def(py::init<>(), R"pbdoc(
                Initializes a new instance of the AvgFilter class.
            )pbdoc");

        py::class_<MovingAverageFilter, Filter, std::shared_ptr<MovingAverageFilter>>(m, "MovingAverageFilter", R"pbdoc(
            A filter that computes the moving average over a specified window size.
        )pbdoc")
            .def(py::init<const unsigned int>(), py::arg("window_size"), R"pbdoc(
                Initializes a new instance of the MovingAverageFilter class.

                Args:
                    window_size (int): The number of data points to include in the moving average.
            )pbdoc")
            .def("get_reading", &MovingAverageFilter::getReading, R"pbdoc(
                Gets the current reading of the moving average.

                Returns:
                    float: The current moving average reading.)pbdoc");

        py::class_<FirstOrderLowPassFilter, Filter, std::shared_ptr<FirstOrderLowPassFilter>>(
                m, "FirstOrderLowPassFilter", R"pbdoc(
            A first-order low-pass filter that smooths data points based on a specified alpha.
        )pbdoc")
            .def(py::init<const double>(), py::arg("alpha"), R"pbdoc(
                Initializes a new instance of the FirstOrderLowPassFilter class.

                Args:
                    alpha (float): The smoothing factor (0 < alpha < 1).
            )pbdoc");

        py::class_<MovingFirstOrderLowPassFilter, Filter, std::shared_ptr<MovingFirstOrderLowPassFilter>>(
                m, "MovingFirstOrderLowPassFilter", R"pbdoc(
            A combined moving average and first-order low-pass filter.
        )pbdoc")
            .def(py::init<const unsigned int, const double>(), py::arg("window_size"), py::arg("alpha"), R"pbdoc(
                Initializes a new instance of the MovingFirstOrderLowPassFilter class.

                Args:
                    window_size (int): The number of data points for the moving average.
                    alpha (float): The smoothing factor for the low-pass filter (0 < alpha < 1).
            )pbdoc");
    }
}
