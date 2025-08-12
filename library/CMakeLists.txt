cmake_minimum_required(VERSION 3.4...3.18)
project(libstp)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Option to enable CREATE3 support
option(BUILD_CREATE3 "Build with CREATE3 support" OFF)
message(STATUS "BUILD_CREATE3: ${BUILD_CREATE3}")
if(BUILD_CREATE3)
    add_definitions(-DBUILD_CREATE3)
endif()

# Allow user to set build type via command-line; default to Debug
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug, Release)" FORCE)
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -w -fcoroutines")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Find spdlog if not already available
if(NOT TARGET spdlog)
    find_package(spdlog REQUIRED)
endif()

# Add subdirectories for core, devices, and external dependencies
add_subdirectory(src/core)
add_subdirectory(src/devices)
add_subdirectory(lib)

# Gather binding header files (if needed)
file(GLOB_RECURSE BINDINGS_HEADER_FILES "${CMAKE_CURRENT_LIST_DIR}/**/bindings.h")

# Create the pybind11 module
pybind11_add_module(libstp src/binding/main.cpp ${BINDINGS_HEADER_FILES})

# Include directories for our library
target_include_directories(libstp
        PRIVATE
        ${PROJECT_SOURCE_DIR}/src/core/include
        ${PROJECT_SOURCE_DIR}/src/devices/two_wheeled/include
        ${PROJECT_SOURCE_DIR}/src/devices/create3/include
        "${CMAKE_CURRENT_LIST_DIR}/include"
)

# Link with internal libraries
target_link_libraries(libstp PRIVATE libstp_core libstp_devices)