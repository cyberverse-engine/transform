/**
 * @file trs.cpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Translation/rotation/scale implementation
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "cyberverse/transform/trs.hpp"

#include <cmath>

#include <glm/geometric.hpp>
#include <glm/matrix.hpp>

#include "detail/math.hpp"
#include "matrix.hpp"

namespace cyberverse {
namespace transform {
namespace {

constexpr float k_decompose_tolerance = 1e-4f;

} // namespace

glm::mat4 trs::matrix() const { return compose_trs(translation, rotation, scale); }

glm::mat4 compose_trs(const trs& value) { return compose_trs(value.translation, value.rotation, value.scale); }

bool decompose_trs(const glm::mat4& matrix, trs& out) {
  if (!detail::is_finite(matrix)) {
    return false;
  }

  if (std::abs(matrix[0][3]) > detail::k_tolerance || std::abs(matrix[1][3]) > detail::k_tolerance ||
      std::abs(matrix[2][3]) > detail::k_tolerance || std::abs(matrix[3][3] - 1.0f) > detail::k_tolerance) {
    return false;
  }

  const glm::vec3 column0(matrix[0][0], matrix[0][1], matrix[0][2]);
  const glm::vec3 column1(matrix[1][0], matrix[1][1], matrix[1][2]);
  const glm::vec3 column2(matrix[2][0], matrix[2][1], matrix[2][2]);

  float scale_x = glm::length(column0);
  const float scale_y = glm::length(column1);
  const float scale_z = glm::length(column2);
  if (!std::isfinite(scale_x) || !std::isfinite(scale_y) || !std::isfinite(scale_z) || scale_x <= detail::k_tolerance ||
      scale_y <= detail::k_tolerance || scale_z <= detail::k_tolerance) {
    return false;
  }

  glm::mat3 rotation(column0 / scale_x, column1 / scale_y, column2 / scale_z);

  const glm::vec3 axis_x = rotation[0];
  const glm::vec3 axis_y = rotation[1];
  const glm::vec3 axis_z = rotation[2];
  if (std::abs(glm::dot(axis_x, axis_y)) > k_decompose_tolerance ||
      std::abs(glm::dot(axis_x, axis_z)) > k_decompose_tolerance ||
      std::abs(glm::dot(axis_y, axis_z)) > k_decompose_tolerance ||
      std::abs(glm::dot(axis_x, axis_x) - 1.0f) > k_decompose_tolerance ||
      std::abs(glm::dot(axis_y, axis_y) - 1.0f) > k_decompose_tolerance ||
      std::abs(glm::dot(axis_z, axis_z) - 1.0f) > k_decompose_tolerance) {
    return false;
  }

  if (glm::determinant(rotation) < 0.0f) {
    scale_x = -scale_x;
    rotation[0] = -rotation[0];
  }

  const glm::quat rotation_quaternion = glm::normalize(glm::quat_cast(rotation));
  if (!detail::is_finite(rotation_quaternion)) {
    return false;
  }

  out.translation = glm::vec3(matrix[3]);
  out.rotation = rotation_quaternion;
  out.scale = glm::vec3(scale_x, scale_y, scale_z);
  return true;
}

bool inverse_trs(const trs& value, glm::mat4& out) {
  if (!detail::is_finite(value.translation) || !detail::is_finite(value.rotation) || !detail::is_finite(value.scale)) {
    return false;
  }

  return detail::invert(value.matrix(), out);
}

} // namespace transform
} // namespace cyberverse
