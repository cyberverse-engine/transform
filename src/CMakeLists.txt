cmake_minimum_required(VERSION 4.0.0)

add_library(cyberverse_transform_static_lib STATIC)
add_library(cyberverse::transform::static_lib ALIAS cyberverse_transform_static_lib)

target_include_directories(cyberverse_transform_static_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Public headers
set(
  HEADER_FILES
  cyberverse/transform.hpp
  cyberverse/transform/camera.hpp
  cyberverse/transform/frustum_builder.hpp
  cyberverse/transform/matrix.hpp
  cyberverse/transform/transform_geometry.hpp
  cyberverse/transform/trs.hpp
)

# Internal headers
set(
  PRIVATE_HEADER_FILES
  cyberverse/transform/detail/math.hpp
)

# Source files
set(
  SOURCE_FILES
  cyberverse/transform/camera.cpp
  cyberverse/transform/frustum_builder.cpp
  cyberverse/transform/matrix.cpp
  cyberverse/transform/transform_geometry.cpp
  cyberverse/transform/trs.cpp
)

target_sources(
  cyberverse_transform_static_lib
  PUBLIC ${HEADER_FILES}
  PRIVATE ${PRIVATE_HEADER_FILES} ${SOURCE_FILES}
)

target_link_libraries(cyberverse_transform_static_lib PUBLIC cyberverse::geometry::static_lib)

# The projection and frustum APIs assume a right-handed [0, 1] depth range.
# Publish the define so consumers and transform itself use the same GLM clip
# control configuration.
target_compile_definitions(cyberverse_transform_static_lib PUBLIC GLM_FORCE_DEPTH_ZERO_TO_ONE)
