cmake_minimum_required(VERSION 4.0.0)

set(BENCHMARK_ENABLE_TESTING OFF)

include(FetchContent)

FetchContent_Declare(
  benchmark
  GIT_REPOSITORY https://github.com/google/benchmark
  GIT_TAG v1.9.5
)
FetchContent_MakeAvailable(benchmark)

add_executable(cyberverse_transform_benchmarks transform_benchmark.cpp)

target_link_libraries(
  cyberverse_transform_benchmarks
  PRIVATE
  cyberverse::transform::static_lib
  benchmark::benchmark
)
