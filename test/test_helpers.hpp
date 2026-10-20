#pragma once

#include <cmath>

#include <glm/geometric.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace test_support {

inline constexpr float k_epsilon = 1e-4f;

inline bool approx(float lhs, float rhs, float epsilon = k_epsilon) noexcept { return std::abs(lhs - rhs) <= epsilon; }

inline bool approx(const glm::vec3& lhs, const glm::vec3& rhs, float epsilon = k_epsilon) noexcept {
  return glm::length(lhs - rhs) <= epsilon;
}

inline bool approx(const glm::mat4& lhs, const glm::mat4& rhs, float epsilon = k_epsilon) noexcept {
  for (int column = 0; column < 4; ++column) {
    for (int row = 0; row < 4; ++row) {
      if (std::abs(lhs[column][row] - rhs[column][row]) > epsilon) {
        return false;
      }
    }
  }
  return true;
}

inline bool is_unit(const glm::vec3& value, float epsilon = k_epsilon) noexcept {
  return std::abs(glm::length(value) - 1.0f) <= epsilon;
}

} // namespace test_support
