/**
 * @file transform_geometry.cpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Transform geometry implementation
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "cyberverse/transform/transform_geometry.hpp"

#include <algorithm>
#include <cmath>

#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <glm/vec4.hpp>

#include "cyberverse/transform/matrix.hpp"
#include "detail/math.hpp"

namespace cyberverse {
namespace transform {
namespace {

geometry::bounding_sphere invalid_sphere() { return geometry::bounding_sphere(glm::vec3(0.0f), -1.0f); }

geometry::ray ray_from_transformed(const glm::vec3& origin, const glm::vec3& direction, float& out_scale) {
  const float length = glm::length(direction);
  if (!std::isfinite(length) || length <= detail::k_tolerance) {
    out_scale = 0.0f;
    return {};
  }

  out_scale = length;
  return geometry::ray::from_normalized(origin, direction / length);
}

} // namespace

glm::vec3 transform_position(const glm::mat4& m, const glm::vec3& p) { return glm::vec3(m * glm::vec4(p, 1.0f)); }

glm::vec3 transform_direction(const glm::mat4& m, const glm::vec3& d) { return glm::vec3(m * glm::vec4(d, 0.0f)); }

glm::vec3 transform_normal(const glm::mat4& m, const glm::vec3& n) { return normal_matrix(m) * n; }

geometry::aabb transform_aabb(const geometry::aabb& local, const glm::mat4& m) {
  if (!local.is_valid() || !detail::is_finite(m)) {
    return {};
  }

  const glm::vec3& min = local.min();
  const glm::vec3& max = local.max();

  geometry::aabb result;
  for (int corner = 0; corner < 8; ++corner) {
    const glm::vec3 point((corner & 1) ? max.x : min.x, (corner & 2) ? max.y : min.y, (corner & 4) ? max.z : min.z);
    const glm::vec3 transformed = transform_position(m, point);
    if (!detail::is_finite(transformed)) {
      return {};
    }
    result.expand(transformed);
  }

  return result;
}

geometry::bounding_sphere transform_bounding_sphere(const geometry::bounding_sphere& local, const glm::mat4& m) {
  if (!local.is_valid() || !detail::is_finite(local.center()) || !std::isfinite(local.radius()) ||
      !detail::is_finite(m)) {
    return invalid_sphere();
  }

  const glm::vec3 center = transform_position(m, local.center());
  if (!detail::is_finite(center)) {
    return invalid_sphere();
  }

  const glm::mat3 linear(m);
  float max_column_sum = 0.0f;
  float max_row_sum = 0.0f;
  for (int column = 0; column < 3; ++column) {
    const float column_sum = std::abs(linear[column][0]) + std::abs(linear[column][1]) + std::abs(linear[column][2]);
    max_column_sum = std::max(max_column_sum, column_sum);
  }
  for (int row = 0; row < 3; ++row) {
    const float row_sum = std::abs(linear[0][row]) + std::abs(linear[1][row]) + std::abs(linear[2][row]);
    max_row_sum = std::max(max_row_sum, row_sum);
  }

  const float scale = std::sqrt(max_column_sum * max_row_sum);
  if (!std::isfinite(scale)) {
    return invalid_sphere();
  }

  return geometry::bounding_sphere(center, local.radius() * scale);
}

geometry::plane transform_plane(const geometry::plane& local, const glm::mat4& m) {
  if (!detail::is_finite(m) || !detail::is_finite(local.normal) || !std::isfinite(local.d)) {
    return {};
  }

  const glm::vec3 raw_normal = normal_matrix(m) * local.normal;
  const float length = glm::length(raw_normal);
  if (!std::isfinite(length) || length <= detail::k_tolerance) {
    return {};
  }

  const glm::vec3 translation(m[3]);
  const float raw_distance = local.d + glm::dot(raw_normal, translation);
  if (!std::isfinite(raw_distance)) {
    return {};
  }

  return geometry::plane{ raw_normal / length, raw_distance / length };
}

geometry::frustum transform_frustum(const geometry::frustum& local, const glm::mat4& m) {
  if (!detail::is_finite(m)) {
    return {};
  }

  geometry::frustum result;
  for (size_t index = 0; index < result.planes.size(); ++index) {
    result.planes[index] = transform_plane(local.planes[index], m);
  }
  return result;
}

geometry::ray
transform_ray_to_local(const geometry::ray& world_ray, const glm::mat4& world_matrix, float& out_direction_scale) {
  out_direction_scale = 0.0f;
  if (!world_ray.is_valid() || !detail::is_finite(world_matrix)) {
    return {};
  }

  glm::mat4 inverse;
  if (!detail::invert(world_matrix, inverse)) {
    return {};
  }

  const glm::vec3 origin = transform_position(inverse, world_ray.origin());
  const glm::vec3 direction = transform_direction(inverse, world_ray.direction());
  if (!detail::is_finite(origin) || !detail::is_finite(direction)) {
    return {};
  }

  return ray_from_transformed(origin, direction, out_direction_scale);
}

geometry::ray transform_ray_to_local(const geometry::ray& world_ray, const glm::mat4& world_matrix) {
  float direction_scale = 0.0f;
  return transform_ray_to_local(world_ray, world_matrix, direction_scale);
}

geometry::ray
transform_ray_to_world(const geometry::ray& local_ray, const glm::mat4& world_matrix, float& out_direction_scale) {
  out_direction_scale = 0.0f;
  if (!local_ray.is_valid() || !detail::is_finite(world_matrix)) {
    return {};
  }

  glm::mat4 inverse;
  if (!detail::invert(world_matrix, inverse)) {
    return {};
  }

  const glm::vec3 origin = transform_position(world_matrix, local_ray.origin());
  const glm::vec3 direction = transform_direction(world_matrix, local_ray.direction());
  if (!detail::is_finite(origin) || !detail::is_finite(direction)) {
    return {};
  }

  return ray_from_transformed(origin, direction, out_direction_scale);
}

geometry::ray transform_ray_to_world(const geometry::ray& local_ray, const glm::mat4& world_matrix) {
  float direction_scale = 0.0f;
  return transform_ray_to_world(local_ray, world_matrix, direction_scale);
}

} // namespace transform
} // namespace cyberverse
