/**
 * @file math.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Internal scalar and matrix helpers
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cmath>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/matrix.hpp>
#include <glm/vec3.hpp>

namespace cyberverse {
namespace transform {
namespace detail {

inline constexpr float k_tolerance = 1e-6f;

inline bool is_finite(float value) noexcept { return std::isfinite(value); }

inline bool is_finite(const glm::vec3& value) noexcept {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

inline bool is_finite(const glm::quat& value) noexcept {
  return std::isfinite(value.w) && std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

inline bool is_finite(const glm::mat3& value) noexcept {
  for (int column = 0; column < 3; ++column) {
    for (int row = 0; row < 3; ++row) {
      if (!std::isfinite(value[column][row])) {
        return false;
      }
    }
  }
  return true;
}

inline bool is_finite(const glm::mat4& value) noexcept {
  for (int column = 0; column < 4; ++column) {
    for (int row = 0; row < 4; ++row) {
      if (!std::isfinite(value[column][row])) {
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief Normalize a quaternion, returning identity for invalid input
 */
inline glm::quat normalized_or_identity(const glm::quat& rotation) noexcept {
  if (!is_finite(rotation)) {
    return glm::quat{ 1.0f, 0.0f, 0.0f, 0.0f };
  }

  const float length_squared =
      rotation.w * rotation.w + rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z;
  if (!std::isfinite(length_squared) || length_squared <= k_tolerance) {
    return glm::quat{ 1.0f, 0.0f, 0.0f, 0.0f };
  }

  return rotation / std::sqrt(length_squared);
}

/**
 * @brief Invert a matrix, reporting failure instead of returning NaNs
 */
inline bool invert(const glm::mat3& matrix, glm::mat3& out) noexcept {
  if (!is_finite(matrix)) {
    return false;
  }
  out = glm::inverse(matrix);
  return is_finite(out);
}

inline bool invert(const glm::mat4& matrix, glm::mat4& out) noexcept {
  if (!is_finite(matrix)) {
    return false;
  }
  out = glm::inverse(matrix);
  return is_finite(out);
}

} // namespace detail
} // namespace transform
} // namespace cyberverse
