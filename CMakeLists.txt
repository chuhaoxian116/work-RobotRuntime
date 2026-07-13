cmake_minimum_required(VERSION 3.16)

project(robot_runtime
    VERSION 0.1.0
    DESCRIPTION "Generic multi-axis robot runtime library"
    LANGUAGES CXX
)

option(ROBOT_RUNTIME_BUILD_SHARED "Build robot_runtime as a shared library" ON)
option(ROBOT_RUNTIME_BUILD_EXAMPLES "Build examples" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CIA402_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/cia402")
set(CIA402_INCLUDE_DIR "${CIA402_SDK_DIR}/include")
set(CIA402_LIBRARY "${CIA402_SDK_DIR}/lib/libcia402.so")

set(ROBOT_RUNTIME_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/sdk")
set(ROBOT_RUNTIME_PUBLIC_INCLUDE_DIR "${ROBOT_RUNTIME_SDK_DIR}/include")
set(ROBOT_RUNTIME_PUBLIC_HEADER
    "${ROBOT_RUNTIME_PUBLIC_INCLUDE_DIR}/robot_runtime/robot_runtime.h"
)
set(ROBOT_RUNTIME_LIBRARY_OUTPUT_DIR "${ROBOT_RUNTIME_SDK_DIR}/lib")

if(NOT EXISTS "${CIA402_INCLUDE_DIR}/cia402/cia402.h" OR
   NOT EXISTS "${CIA402_LIBRARY}" OR
   NOT EXISTS "${ROBOT_RUNTIME_PUBLIC_HEADER}")
    message(FATAL_ERROR
        "SDK submodules are not initialized. Run: "
        "git submodule update --init --recursive"
    )
endif()

add_library(cia402::cia402 SHARED IMPORTED GLOBAL)

set_target_properties(cia402::cia402 PROPERTIES
    IMPORTED_LOCATION "${CIA402_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${CIA402_INCLUDE_DIR}"
)

# Library targets will be added after the public API and implementation are defined.
