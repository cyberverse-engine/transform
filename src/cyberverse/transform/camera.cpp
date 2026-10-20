/**
 * @file camera.cpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Camera implementation
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "cyberverse/transform/camera.hpp"

#include <cmath>
#include <limits>
#include <numbers>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/vec4.hpp>

#include "detail/math.hpp"

namespace cyberverse {
namespace transform {
namespace {

glm::vec3 nan_vec3() { return glm::vec3(std::numeric_limits<float>::quiet_NaN()); }

bool valid_perspective(float fov_y_radians, float aspect, float near_z, float far_z) {
  return detail::is_finite(fov_y_radians) && detail::is_finite(aspect) && detail::is_finite(near_z) &&
         detail::is_finite(far_z) && fov_y_radians > 0.0f && fov_y_radians < std::numbers::pi_v<float> &&
         aspect > 0.0f && near_z > 0.0f && far_z > near_z;
}

} // namespace

glm::mat4 perspective(float fovYRadians, float aspect, float nearZ, float farZ) {
  if (!valid_perspective(fovYRadians, aspect, nearZ, farZ)) {
    return glm::mat4(1.0f);
  }
  return glm::perspectiveRH_ZO(fovYRadians, aspect, nearZ, farZ);
}

glm::mat4 perspective_offcenter(float left, float right, float bottom, float top, float nearZ, float farZ) {
  if (!detail::is_finite(left) || !detail::is_finite(right) || !detail::is_finite(bottom) || !detail::is_finite(top) ||
      !detail::is_finite(nearZ) || !detail::is_finite(farZ) || right <= left || top <= bottom || nearZ <= 0.0f ||
      farZ <= nearZ) {
    return glm::mat4(1.0f);
  }
  return glm::frustumRH_ZO(left, right, bottom, top, nearZ, farZ);
}

glm::mat4 orthographic(float left, float right, float bottom, float top, float nearZ, float farZ) {
  if (!detail::is_finite(left) || !detail::is_finite(right) || !detail::is_finite(bottom) || !detail::is_finite(top) ||
      !detail::is_finite(nearZ) || !detail::is_finite(farZ) || right <= left || top <= bottom || farZ <= nearZ) {
    return glm::mat4(1.0f);
  }
  return glm::orthoRH_ZO(left, right, bottom, top, nearZ, farZ);
}

glm::mat4 look_at(const glm::vec3& eye, const glm::vec3& target, const glm::vec3& up) {
  if (!detail::is_finite(eye) || !detail::is_finite(target) || !detail::is_finite(up)) {
    return glm::mat4(1.0f);
  }

  const glm::vec3 to_target = target - eye;
  const float forward_length = glm::length(to_target);
  if (!std::isfinite(forward_length) || forward_length <= detail::k_tolerance) {
    return glm::mat4(1.0f);
  }

  const glm::vec3 forward = to_target / forward_length;

  glm::vec3 up_axis = up;
  float up_length = glm::length(up_axis);
  if (!std::isfinite(up_length) || up_length <= detail::k_tolerance) {
    up_axis = (std::abs(forward.y) < 0.999f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    up_length = 1.0f;
  }
  up_axis /= up_length;

  glm::vec3 right = glm::cross(forward, up_axis);
  float right_length = glm::length(right);
  if (!std::isfinite(right_length) || right_length <= detail::k_tolerance) {
    up_axis = (std::abs(forward.y) < 0.999f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    right = glm::cross(forward, up_axis);
    right_length = glm::length(right);
    if (!std::isfinite(right_length) || right_length <= detail::k_tolerance) {
      return glm::mat4(1.0f);
    }
  }
  right /= right_length;

  const glm::vec3 camera_up = glm::cross(right, forward);

  glm::mat4 result(1.0f);
  result[0][0] = right.x;
  result[1][0] = right.y;
  result[2][0] = right.z;
  result[0][1] = camera_up.x;
  result[1][1] = camera_up.y;
  result[2][1] = camera_up.z;
  result[0][2] = -forward.x;
  result[1][2] = -forward.y;
  result[2][2] = -forward.z;
  result[3][0] = -glm::dot(right, eye);
  result[3][1] = -glm::dot(camera_up, eye);
  result[3][2] = glm::dot(forward, eye);
  return result;
}

glm::mat4 view_from_orientation(const glm::vec3& eye, const glm::quat& orientation) {
  if (!detail::is_finite(eye) || !detail::is_finite(orientation)) {
    return glm::mat4(1.0f);
  }

  const glm::quat rotation = detail::normalized_or_identity(orientation);
  const glm::vec3 right = rotation * glm::vec3(1.0f, 0.0f, 0.0f);
  const glm::vec3 up = rotation * glm::vec3(0.0f, 1.0f, 0.0f);
  const glm::vec3 back = rotation * glm::vec3(0.0f, 0.0f, 1.0f);

  glm::mat4 result(1.0f);
  result[0][0] = right.x;
  result[1][0] = right.y;
  result[2][0] = right.z;
  result[0][1] = up.x;
  result[1][1] = up.y;
  result[2][1] = up.z;
  result[0][2] = back.x;
  result[1][2] = back.y;
  result[2][2] = back.z;
  result[3][0] = -glm::dot(right, eye);
  result[3][1] = -glm::dot(up, eye);
  result[3][2] = -glm::dot(back, eye);
  return result;
}

glm::vec3 view_direction(const glm::quat& orientation) {
  if (!detail::is_finite(orientation)) {
    return glm::vec3(0.0f, 0.0f, -1.0f);
  }
  return detail::normalized_or_identity(orientation) * glm::vec3(0.0f, 0.0f, -1.0f);
}

glm::vec3 project_ndc(const glm::mat4& view_proj, const glm::vec3& world_point) {
  if (!detail::is_finite(view_proj) || !detail::is_finite(world_point)) {
    return nan_vec3();
  }

  const glm::vec4 clip = view_proj * glm::vec4(world_point, 1.0f);
  if (!std::isfinite(clip.w) || std::abs(clip.w) <= detail::k_tolerance) {
    return nan_vec3();
  }

  const glm::vec3 ndc = glm::vec3(clip) / clip.w;
  if (!detail::is_finite(ndc)) {
    return nan_vec3();
  }
  return ndc;
}

glm::vec3 unproject_ndc(const glm::mat4& inverse_view_proj, const glm::vec3& ndc) {
  if (!detail::is_finite(inverse_view_proj) || !detail::is_finite(ndc)) {
    return nan_vec3();
  }

  const glm::vec4 homogeneous = inverse_view_proj * glm::vec4(ndc, 1.0f);
  if (!std::isfinite(homogeneous.w) || std::abs(homogeneous.w) <= detail::k_tolerance) {
    return nan_vec3();
  }

  const glm::vec3 world = glm::vec3(homogeneous) / homogeneous.w;
  if (!detail::is_finite(world)) {
    return nan_vec3();
  }
  return world;
}

geometry::ray ray_from_ndc(const glm::mat4& inverse_view_proj, const glm::vec2& ndc_xy) {
  if (!detail::is_finite(inverse_view_proj) || !detail::is_finite(ndc_xy.x) || !detail::is_finite(ndc_xy.y)) {
    return {};
  }

  const glm::vec3 near_point = unproject_ndc(inverse_view_proj, glm::vec3(ndc_xy.x, ndc_xy.y, 0.0f));
  const glm::vec3 far_point = unproject_ndc(inverse_view_proj, glm::vec3(ndc_xy.x, ndc_xy.y, 1.0f));
  if (!detail::is_finite(near_point) || !detail::is_finite(far_point)) {
    return {};
  }

  return geometry::ray(near_point, far_point - near_point);
}

} // namespace transform
} // namespace cyberverse
