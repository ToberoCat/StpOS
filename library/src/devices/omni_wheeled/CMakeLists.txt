file(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")
file(GLOB_RECURSE HEADER_FILES "${CMAKE_CURRENT_LIST_DIR}/include/*.h" "${CMAKE_CURRENT_LIST_DIR}/include/*.hpp")

list(FILTER HEADER_FILES EXCLUDE REGEX "/bindings\\.h$")

add_library(libstp_device_omni_wheeled ${SRC_FILES} ${HEADER_FILES})

target_link_libraries(libstp_core PRIVATE kipr)
target_include_directories(libstp_device_omni_wheeled
        PRIVATE
        ${PROJECT_SOURCE_DIR}/src/core/include
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(libstp_device_omni_wheeled
        PRIVATE
        libstp_core
)