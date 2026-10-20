/**
 * @file frustum_builder.cpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Frustum builder implementation
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "cyberverse/transform/frustum_builder.hpp"

#include <cmath>

#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <glm/vec4.hpp>

#include "detail/math.hpp"

namespace cyberverse {
namespace transform {
namespace {

glm::vec4 matrix_row(const glm::mat4& matrix, int row) noexcept {
  return glm::vec4(matrix[0][row], matrix[1][row], matrix[2][row], matrix[3][row]);
}

} // namespace

geometry::frustum frustum_from_view_proj(const glm::mat4& view_proj) {
  if (!detail::is_finite(view_proj)) {
    return {};
  }

  const glm::vec4 row0 = matrix_row(view_proj, 0);
  const glm::vec4 row1 = matrix_row(view_proj, 1);
  const glm::vec4 row2 = matrix_row(view_proj, 2);
  const glm::vec4 row3 = matrix_row(view_proj, 3);

  // Right-handed clip space with z in [0, 1]:
  //   left:   x + w >= 0, right: w - x >= 0
  //   bottom: y + w >= 0, top:   w - y >= 0
  //   near:   z >= 0,     far:   w - z >= 0
  const glm::vec4 coefficients[6] = {
    row3 + row0, row3 - row0, row3 + row1, row3 - row1, row2, row3 - row2,
  };

  geometry::frustum result;
  for (int index = 0; index < 6; ++index) {
    const glm::vec3 normal(coefficients[index]);
    const float length = glm::length(normal);
    if (!std::isfinite(length) || length <= detail::k_tolerance) {
      return {};
    }

    result.planes[index] = geometry::plane{ normal / length, -coefficients[index].w / length };
  }

  return result;
}

bool frustum_corners(const glm::mat4& view_proj, std::array<glm::vec3, 8>& out_corners) {
  if (!detail::is_finite(view_proj)) {
    return false;
  }

  glm::mat4 inverse;
  if (!detail::invert(view_proj, inverse)) {
    return false;
  }

  const std::array<glm::vec3, 8> ndc_corners = {
    glm::vec3(-1.0f, -1.0f, 0.0f), glm::vec3(1.0f, -1.0f, 0.0f),  glm::vec3(1.0f, 1.0f, 0.0f),
    glm::vec3(-1.0f, 1.0f, 0.0f),  glm::vec3(-1.0f, -1.0f, 1.0f), glm::vec3(1.0f, -1.0f, 1.0f),
    glm::vec3(1.0f, 1.0f, 1.0f),   glm::vec3(-1.0f, 1.0f, 1.0f),
  };

  std::array<glm::vec3, 8> corners{};
  for (size_t index = 0; index < ndc_corners.size(); ++index) {
    const glm::vec4 homogeneous = inverse * glm::vec4(ndc_corners[index], 1.0f);
    if (!std::isfinite(homogeneous.w) || std::abs(homogeneous.w) <= detail::k_tolerance) {
      return false;
    }

    const glm::vec3 corner = glm::vec3(homogeneous) / homogeneous.w;
    if (!detail::is_finite(corner)) {
      return false;
    }
    corners[index] = corner;
  }

  out_corners = corners;
  return true;
}

geometry::aabb frustum_bounds(const glm::mat4& view_proj) {
  std::array<glm::vec3, 8> corners;
  if (!frustum_corners(view_proj, corners)) {
    return {};
  }

  geometry::aabb result;
  for (const glm::vec3& corner : corners) {
    result.expand(corner);
  }
  return result;
}

} // namespace transform
} // namespace cyberverse
