file(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")
file(GLOB_RECURSE HEADER_FILES "${CMAKE_CURRENT_LIST_DIR}/include/*.h")

list(FILTER HEADER_FILES EXCLUDE REGEX "/bindings\\.h$")

add_library(libstp_core ${SRC_FILES} ${HEADER_FILES})

target_link_libraries(libstp_core PRIVATE kipr)
target_include_directories(libstp_core
        PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/include"
)