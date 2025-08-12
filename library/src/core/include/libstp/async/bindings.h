//
// Created by tobias on 1/30/25.
//

#pragma once
#include "algorithm.h"
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/iostream.h>
#include <pybind11/chrono.h>

namespace py = pybind11;

namespace libstp::async
{
    template <typename T>
    py::object start_async_algorithm(AsyncAlgorithm<T> algo,
                                     int frequency)
    {
        py::gil_scoped_acquire acquire;
        auto sharedAlgo = std::make_shared<AsyncAlgorithm<T>>(std::move(algo));
        const py::object asyncio = py::module_::import("asyncio");
        py::object runner = py::module_::import("libstp.asynchronous").attr("_run_algorithm");
        return asyncio.attr("create_task")(runner(sharedAlgo, frequency));
    }

    AsyncAlgorithm<int> sample_algorithm()
    {
        for (int i = 0; i < 10; i++)
        {
            std::cout << "T " << i <<std::endl;
            co_yield i;
        }
    }

    inline void createAlgorithmBindings(py::module_& m)
    {
        py::exec(R"(
import asyncio
import time

async def run_algorithm(algo, frequency):
    '''Repeatedly calls algo.advance() until completion with a fixed frequency.'''
    period = 1 / frequency
    print(f"Running algorithm with period {period}")
    while True:
        print("Advancing algorithm")
        start_time = time.time()
        print("Starting")
        more = algo.advance()
        print("Advanced")
        if not more:
            print("No more")
            break
        print("More")
        elapsed_time = time.time() - start_time
        print(f"Elapsed time: {elapsed_time}")
        sleep_time = max(0, period - elapsed_time)
        print(f"Sleeping for {sleep_time} seconds")
        await asyncio.sleep(sleep_time)  # let other tasks run
        print("Continuing")
)");

        m.attr("_run_algorithm") = py::globals()["run_algorithm"];

        py::class_<AsyncAlgorithm<int>>(m, "AsyncAlgorithmInt")
            .def("advance", &AsyncAlgorithm<int>::advance)
            .def("current", &AsyncAlgorithm<int>::current);

        m.def("sample_algorithm", []
        {
            return sample_algorithm();
        });
    }
}
