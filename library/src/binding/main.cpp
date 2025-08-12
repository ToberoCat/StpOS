//
// Created by tobias on 12/29/24.
// Updated to properly integrate the custom elapsed time formatter
//
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "../devices/omni_wheeled/include/libstp/device/omni_wheeled/bindings.h"
#include "../devices/omni_wheeled/include/libstp/device/omni_wheeled/omni_wheeled_device.h"
#include "../devices/omni_wheeled/include/libstp/device/omni_wheeled/datatype/bindings.h"
#include "libstp/_config.h"
#include "libstp/async/bindings.h"
#include "libstp/datatype/bindings.h"
#include "libstp/device/bindings.h"
#include "libstp/device/two_wheeled/bindings.h"
#include "libstp/filter/bindings.h"
#include "libstp/math/bindings.h"
#include "libstp/motor/bindings.h"
#include "libstp/thread/bindings.h"
#include "libstp/sensor/bindings.h"
#include "libstp/servo/bindings.h"
#include "libstp/utility/bindings.h"

#include <filesystem>
#include <chrono>
#include <memory>
#include <optional>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/pattern_formatter.h>

#ifdef BUILD_CREATE3
#include "libstp/device/create3/bindings.h"
#endif

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

static std::optional<std::chrono::steady_clock::time_point> start_time;

void initialize_timer() {
    start_time = std::chrono::steady_clock::now();
    spdlog::info("Logging timer initialized");
}

class ElapsedTimeFormatter : public spdlog::custom_flag_formatter {
public:
    void format(const spdlog::details::log_msg &msg, const std::tm &tm, spdlog::memory_buf_t &dest) override {
        if (start_time.has_value()) {
            const auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - *start_time).count() / 1000.0;
            fmt::format_to(std::back_inserter(dest), "{:.3f}s", elapsed);
        } else {
            fmt::format_to(std::back_inserter(dest), "0.000s");
        }
    }

    std::unique_ptr<spdlog::custom_flag_formatter> clone() const override {
        return spdlog::details::make_unique<ElapsedTimeFormatter>();
    }
};

PYBIND11_MODULE(libstp, m)
{
    m.doc() = R"pbdoc(
        LibStp: A library containing some commonly used functions for botball
        ---------------------------------------------------------------------

        .. currentmodule:: libstp

        .. autosummary::
           :toctree: _generate

    )pbdoc";

    // Ensure the logs directory exists.
    std::filesystem::path log_dir = "logs";
    if (!std::filesystem::exists(log_dir)) {
        std::filesystem::create_directory(log_dir);
    }

    // Create the console and file sinks.
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        (log_dir / "libstp.log").string(),
        5 * 1024 * 1024,  // 5MB max file size
        3                // keep 3 rotated files
    );

    // Create a pattern formatter and register the custom elapsed time flag using '%E'
    auto pattern_formatter = std::make_unique<spdlog::pattern_formatter>();
    pattern_formatter->add_flag<ElapsedTimeFormatter>('E');
    pattern_formatter->set_pattern("[%H:%M:%S] [+%E] [t%t/%n] [%^%l%$]: %v");

    // Apply the custom formatter to both sinks.
    console_sink->set_formatter(pattern_formatter->clone());
    file_sink->set_formatter(pattern_formatter->clone());

    // Create the logger with the custom sinks.
    auto logger = std::make_shared<spdlog::logger>(
        "default", spdlog::sinks_init_list{console_sink, file_sink});

    // Set the default logger and configure its behavior.
    spdlog::set_default_logger(logger);
    logger->set_level(spdlog::level::debug);
    logger->flush_on(spdlog::level::warn);
    spdlog::flush_every(std::chrono::seconds(3));

    logger->info("Logging to directory: {}", std::filesystem::absolute(log_dir).string());

    // Set up pybind11 submodules.
    py::module_ deviceModule = m.def_submodule("device");
    py::module_ twoWheeledModule = deviceModule.def_submodule("two_wheeled");
    py::module_ omniWheeledModule = deviceModule.def_submodule("omni_wheeled");
    py::module_ create3Module = deviceModule.def_submodule("create3");
    py::module_ filterModule = m.def_submodule("filter");
    py::module_ datatypes = m.def_submodule("datatypes");
    py::module_ sensorModule = m.def_submodule("sensor");
    py::module_ schedulerModule = m.def_submodule("scheduler");
    py::module_ threadModule = m.def_submodule("thread");
    py::module_ servoModule = m.def_submodule("servo");
    py::module_ mathModule = m.def_submodule("math");
    py::module_ logModule = m.def_submodule("logging");
    py::module_ motorModule = m.def_submodule("motor");
    py::module_ asynchronousModule = m.def_submodule("asynchronous");

    m.def("initialize_timer", &initialize_timer, "Initialize the timer for elapsed time logging");

    // Binding calls for different modules.
    libstp::async::createAlgorithmBindings(asynchronousModule);
    libstp::datatype::createAxisBindings(datatypes);
    libstp::datatype::createConditionsBindings(datatypes);
    libstp::datatype::createFunctionsBindings(datatypes);
    libstp::datatype::createSpeedBindings(datatypes);

    libstp::device::createDeviceBindings(deviceModule);
    libstp::device::two_wheeled::createTwoWheeledBindings(twoWheeledModule);
    libstp::device::omni_wheeled::createOmniWheeledBindings(omniWheeledModule);
    libstp::device::omni_wheeled::datatype::createOmniWheeledDatatypeBindings(omniWheeledModule);

    libstp::filter::createFilterBindings(filterModule);
    libstp::math::createMathBindings(mathModule);
    libstp::motor::createMotorBindings(motorModule);
    libstp::motor::createServoLikeMotorBindings(motorModule);
    libstp::threads::createManagedThreadBindings(threadModule);
    libstp::threads::createIntervalBindings(schedulerModule);
    libstp::sensor::createSensorBindings(sensorModule);
    libstp::sensor::createImuSensorBindings(sensorModule);
    libstp::servo::createServoBindings(servoModule);
    libstp::utility::createPidBindings(m);
    libstp::utility::createLoggingBindings(logModule);

#ifdef BUILD_CREATE3
    libstp::device::create3::createCreate3Bindings(create3Module);
    libstp::device::create3::createCreate3SensorBindings(create3Module);
#endif

    m.attr("__version__") = "1.0.0";
}
