/**
 * @file matrix.cpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Matrix implementation
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "cyberverse/transform/matrix.hpp"

#include <cmath>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/matrix.hpp>
#include <glm/vec4.hpp>

#include "detail/math.hpp"

namespace cyberverse {
namespace transform {
namespace {

glm::mat4 scale_columns(glm::mat4 matrix, const glm::vec3& scale) noexcept {
  matrix[0] *= scale.x;
  matrix[1] *= scale.y;
  matrix[2] *= scale.z;
  return matrix;
}

glm::mat4 rotation_around(const glm::vec3& axis, float radians) {
  if (!detail::is_finite(radians)) {
    return glm::mat4(1.0f);
  }

  const float axis_length = glm::length(axis);
  if (!std::isfinite(axis_length) || axis_length <= detail::k_tolerance) {
    return glm::mat4(1.0f);
  }

  return glm::rotate(glm::mat4(1.0f), radians, axis / axis_length);
}

} // namespace

glm::mat4 compose_trs(const glm::vec3& translation, const glm::quat& rotation, const glm::vec3& scale) {
  glm::mat4 result = glm::mat4_cast(detail::normalized_or_identity(rotation));
  result = scale_columns(result, scale);
  result[3] = glm::vec4(translation, 1.0f);
  return result;
}

glm::mat4 compose_trs(const glm::vec3& translation, const glm::mat4& rotation, const glm::vec3& scale) {
  glm::mat4 result = scale_columns(rotation, scale);
  result[3] = glm::vec4(translation, 1.0f);
  return result;
}

glm::mat4 rotation_x(float radians) { return rotation_around(glm::vec3(1.0f, 0.0f, 0.0f), radians); }

glm::mat4 rotation_y(float radians) { return rotation_around(glm::vec3(0.0f, 1.0f, 0.0f), radians); }

glm::mat4 rotation_z(float radians) { return rotation_around(glm::vec3(0.0f, 0.0f, 1.0f), radians); }

glm::mat4 rotation_axis_angle(const glm::vec3& axis, float radians) { return rotation_around(axis, radians); }

glm::mat3 normal_matrix(const glm::mat4& matrix) {
  glm::mat3 inverse;
  if (!detail::invert(glm::mat3(matrix), inverse)) {
    return glm::mat3(0.0f);
  }
  return glm::transpose(inverse);
}

} // namespace transform
} // namespace cyberverse
