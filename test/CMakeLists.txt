cmake_minimum_required(VERSION 4.0.0)

include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.5.2
  GIT_SHALLOW ON
)
FetchContent_MakeAvailable(Catch2)

add_executable(
  cyberverse_transform_tests
  matrix_test.cpp
  trs_test.cpp
  camera_test.cpp
  frustum_builder_test.cpp
  transform_geometry_test.cpp
)

target_link_libraries(
  cyberverse_transform_tests
  PRIVATE
  cyberverse::transform::static_lib
  Catch2::Catch2WithMain
)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(cyberverse_transform_tests)
