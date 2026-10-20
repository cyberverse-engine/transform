/**
 * @file camera.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Camera projection and view helpers
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "cyberverse/geometry/intersect.hpp"

namespace cyberverse {
namespace transform {

/**
 * @brief Right-handed perspective projection with a [0, 1] depth range
 *
 * The camera looks down -Z in view space. Returns the identity matrix when the
 * parameters are invalid: a field of view outside (0, pi), a non-positive
 * aspect ratio, a non-positive near plane, or a far plane not beyond near.
 *
 * @param fovYRadians Vertical field of view in radians
 * @param aspect Viewport width divided by height
 * @param nearZ Distance to the near plane
 * @param farZ Distance to the far plane
 * @return glm::mat4
 */
glm::mat4 perspective(float fovYRadians, float aspect, float nearZ, float farZ);

/**
 * @brief Right-handed off-center perspective projection with a [0, 1] depth range
 *
 * Useful for asymmetric frusta such as tiled rendering and VR eyes. Returns the
 * identity matrix when @p right is not beyond @p left, @p top is not beyond
 * @p bottom, or @p farZ is not beyond a positive @p nearZ.
 *
 * @param left
 * @param right
 * @param bottom
 * @param top
 * @param nearZ
 * @param farZ
 * @return glm::mat4
 */
glm::mat4 perspective_offcenter(float left, float right, float bottom, float top, float nearZ, float farZ);

/**
 * @brief Right-handed orthographic projection with a [0, 1] depth range
 *
 * Returns the identity matrix when the box is degenerate or @p farZ is not
 * beyond @p nearZ.
 *
 * @param left
 * @param right
 * @param bottom
 * @param top
 * @param nearZ
 * @param farZ
 * @return glm::mat4
 */
glm::mat4 orthographic(float left, float right, float bottom, float top, float nearZ, float farZ);

/**
 * @brief Right-handed view matrix
 *
 * Builds a world-to-view matrix for a camera at @p eye looking at @p target.
 * When the eye and target coincide, the up vector is parallel to the view
 * direction, or any input is non-finite, the identity matrix is returned.
 *
 * @param eye
 * @param target
 * @param up
 * @return glm::mat4
 */
glm::mat4 look_at(const glm::vec3& eye, const glm::vec3& target, const glm::vec3& up);

/**
 * @brief View matrix from a camera position and orientation
 *
 * The orientation is normalized and the camera looks down its local -Z axis.
 * Returns the identity matrix for a non-finite eye position.
 *
 * @param eye
 * @param orientation
 * @return glm::mat4
 */
glm::mat4 view_from_orientation(const glm::vec3& eye, const glm::quat& orientation);

/**
 * @brief World-space forward direction of an orientation
 *
 * Returns the orientation applied to local -Z.
 *
 * @param orientation
 * @return glm::vec3 Unit-length forward direction
 */
glm::vec3 view_direction(const glm::quat& orientation);

/**
 * @brief Project a world-space point into normalized device coordinates
 *
 * Performs the perspective divide after multiplying by @p view_proj. The
 * resulting Z is in [0, 1] for points inside the clip volume. Returns a NaN
 * vector when the point is non-finite, the matrix is non-finite, or the clip
 * w is zero.
 *
 * @param view_proj
 * @param world_point
 * @return glm::vec3
 */
glm::vec3 project_ndc(const glm::mat4& view_proj, const glm::vec3& world_point);

/**
 * @brief Unproject a normalized device coordinate into world space
 *
 * @p ndc uses a [0, 1] Z range. Returns a NaN vector when the matrix or input
 * is non-finite, or the homogeneous w is zero.
 *
 * @param inverse_view_proj
 * @param ndc
 * @return glm::vec3
 */
glm::vec3 unproject_ndc(const glm::mat4& inverse_view_proj, const glm::vec3& ndc);

/**
 * @brief Build a world-space picking ray from a normalized device coordinate
 *
 * The ray starts on the near plane (NDC z = 0) and points towards the far
 * plane (NDC z = 1). Returns an invalid ray when the inverse matrix is
 * non-finite or the points cannot be unprojected.
 *
 * @param inverse_view_proj
 * @param ndc_xy Normalized device coordinates in [-1, 1]
 * @return geometry::ray
 */
geometry::ray ray_from_ndc(const glm::mat4& inverse_view_proj, const glm::vec2& ndc_xy);

} // namespace transform
} // namespace cyberverse
