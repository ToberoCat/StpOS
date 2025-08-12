//
// Created by tobias on 12/26/24.
//

#ifndef LIBSTP__CONFIG_H
#define LIBSTP__CONFIG_H

// SPDLOG_LEVEL_DEBUG for running the robot
// SPDLOG_LEVEL_TRACE for debugging issues within algorithms
//
// This only sets the macro level - the actual logging level is set in the bindings/main.cpp file
// Macro Level defines the minimum level of logging that will be compiled into the binary
// Actual Level defines the minimum level of logging that will be outputted to the different log writers
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/fmt/ostr.h>

#endif //LIBSTP__CONFIG_H
