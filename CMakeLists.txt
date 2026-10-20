cmake_minimum_required(VERSION 4.0.0)
project(cyberverse_transform CXX)

option(CYBERVERSE_TRANSFORM_BUILD_TESTS "Build unit test cases" ON)
option(CYBERVERSE_TRANSFORM_BUILD_BENCHMARKS "Build benchmarks" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Import engine dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(engine_dep_geometry)
resolve_engine_dep_geometry()

add_subdirectory(src)

if(CYBERVERSE_TRANSFORM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

if(CYBERVERSE_TRANSFORM_BUILD_BENCHMARKS)
  add_subdirectory(benchmark)
endif()
