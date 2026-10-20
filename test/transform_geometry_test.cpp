#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/matrix.hpp>

#include "cyberverse/geometry/intersect.hpp"
#include "cyberverse/transform/camera.hpp"
#include "cyberverse/transform/frustum_builder.hpp"
#include "cyberverse/transform/matrix.hpp"
#include "cyberverse/transform/transform_geometry.hpp"
#include "test_helpers.hpp"

using namespace cyberverse::transform;

namespace {

constexpr float k_half_pi = 1.5707963267948966f;

const glm::quat k_identity(1.0f, 0.0f, 0.0f, 0.0f);

} // namespace

TEST_CASE("transform_position and transform_direction handle translation", "[geometry][transform]") {
  const glm::mat4 matrix = compose_trs(glm::vec3(1.0f, 2.0f, 3.0f), k_identity, glm::vec3(2.0f, 3.0f, 4.0f));

  CHECK(test_support::approx(transform_position(matrix, glm::vec3(1.0f, 1.0f, 1.0f)), glm::vec3(3.0f, 5.0f, 7.0f)));
  CHECK(test_support::approx(transform_direction(matrix, glm::vec3(1.0f, 1.0f, 1.0f)), glm::vec3(2.0f, 3.0f, 4.0f)));
}

TEST_CASE("transform_normal corrects for non-uniform scale", "[geometry][normal]") {
  const glm::mat4 matrix = compose_trs(glm::vec3(0.0f), k_identity, glm::vec3(2.0f, 1.0f, 1.0f));
  CHECK(test_support::approx(transform_normal(matrix, glm::vec3(1.0f, 1.0f, 0.0f)), glm::vec3(0.5f, 1.0f, 0.0f)));
}

TEST_CASE("transform_aabb transforms all eight corners", "[geometry][aabb]") {
  const glm::mat4 matrix = rotation_z(k_half_pi * 0.5f);
  const cyberverse::geometry::aabb local({ -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f });
  const cyberverse::geometry::aabb world = transform_aabb(local, matrix);

  REQUIRE(world.is_valid());
  CHECK(test_support::approx(world.min().x, -std::sqrt(2.0f)));
  CHECK(test_support::approx(world.max().x, std::sqrt(2.0f)));
  CHECK(test_support::approx(world.min().y, -std::sqrt(2.0f)));
  CHECK(test_support::approx(world.max().y, std::sqrt(2.0f)));
  CHECK(test_support::approx(world.min().z, -1.0f));
  CHECK(test_support::approx(world.max().z, 1.0f));
}

TEST_CASE("transform_aabb rejects invalid input", "[geometry][aabb]") {
  const glm::mat4 matrix(1.0f);
  glm::mat4 non_finite(1.0f);
  non_finite[0][0] = std::numeric_limits<float>::quiet_NaN();

  CHECK_FALSE(transform_aabb(cyberverse::geometry::aabb{}, matrix).is_valid());
  CHECK_FALSE(
      transform_aabb(cyberverse::geometry::aabb({ -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f }), non_finite).is_valid());
}

TEST_CASE("transform_bounding_sphere scales the radius conservatively", "[geometry][sphere]") {
  const cyberverse::geometry::bounding_sphere local(glm::vec3(1.0f, 1.0f, 1.0f), 2.0f);
  const glm::mat4 matrix = compose_trs(glm::vec3(5.0f, 0.0f, 0.0f), k_identity, glm::vec3(2.0f, 2.0f, 2.0f));
  const cyberverse::geometry::bounding_sphere world = transform_bounding_sphere(local, matrix);

  REQUIRE(world.is_valid());
  CHECK(test_support::approx(world.center(), glm::vec3(7.0f, 2.0f, 2.0f)));
  CHECK(test_support::approx(world.radius(), 4.0f));
}

TEST_CASE("transform_bounding_sphere rejects invalid input", "[geometry][sphere]") {
  const glm::mat4 matrix(1.0f);
  glm::mat4 non_finite(1.0f);
  non_finite[0][0] = std::numeric_limits<float>::quiet_NaN();

  CHECK_FALSE(
      transform_bounding_sphere(cyberverse::geometry::bounding_sphere(glm::vec3(0.0f), -1.0f), matrix).is_valid());
  CHECK_FALSE(
      transform_bounding_sphere(cyberverse::geometry::bounding_sphere(glm::vec3(0.0f), 1.0f), non_finite).is_valid());
}

TEST_CASE("transform_plane accounts for scale and translation", "[geometry][plane]") {
  const cyberverse::geometry::plane local({ 0.0f, 1.0f, 0.0f }, 2.0f);
  const glm::mat4 matrix = compose_trs(glm::vec3(0.0f, 10.0f, 0.0f), k_identity, glm::vec3(1.0f, 2.0f, 1.0f));
  const cyberverse::geometry::plane world = transform_plane(local, matrix);

  CHECK(test_support::is_unit(world.normal));
  CHECK(test_support::approx(world.normal, glm::vec3(0.0f, 1.0f, 0.0f)));
  CHECK(test_support::approx(world.d, 14.0f));

  const glm::vec3 local_point(3.0f, 2.0f, -1.0f);
  const glm::vec3 world_point = transform_position(matrix, local_point);
  CHECK(test_support::approx(glm::dot(world.normal, world_point), world.d));
}

TEST_CASE("transform_plane rejects singular and degenerate input", "[geometry][plane]") {
  const cyberverse::geometry::plane local({ 0.0f, 1.0f, 0.0f }, 1.0f);
  CHECK(test_support::approx(transform_plane(local, glm::mat4(0.0f)).normal, glm::vec3(0.0f)));
  CHECK(test_support::approx(transform_plane(cyberverse::geometry::plane{}, glm::mat4(1.0f)).normal, glm::vec3(0.0f)));
}

TEST_CASE("transform_frustum keeps planes consistent under a world transform", "[geometry][frustum]") {
  const glm::mat4 view = look_at(glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  const glm::mat4 view_proj = perspective(1.0f, 1.5f, 0.1f, 100.0f) * view;
  const cyberverse::geometry::frustum local = frustum_from_view_proj(view_proj);

  const glm::mat4 world = compose_trs(glm::vec3(10.0f, 0.0f, 0.0f), k_identity, glm::vec3(2.0f, 2.0f, 2.0f));
  const cyberverse::geometry::frustum transformed = transform_frustum(local, world);

  for (const cyberverse::geometry::plane& plane : transformed.planes) {
    CHECK(test_support::is_unit(plane.normal));
  }

  CHECK_FALSE(cyberverse::geometry::is_outside(transformed, glm::vec3(10.0f, 0.0f, 0.0f)));
  CHECK(cyberverse::geometry::is_outside(transformed, glm::vec3(-100.0f, 0.0f, 0.0f)));
}

TEST_CASE("transform_ray_to_local reports the distance scale", "[geometry][ray]") {
  const glm::mat4 matrix = compose_trs(glm::vec3(0.0f), k_identity, glm::vec3(2.0f, 2.0f, 2.0f));
  const cyberverse::geometry::ray world_ray(glm::vec3(-4.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f));

  float direction_scale = 0.0f;
  const cyberverse::geometry::ray local_ray = transform_ray_to_local(world_ray, matrix, direction_scale);

  REQUIRE(local_ray.is_valid());
  CHECK(test_support::approx(direction_scale, 0.5f));
  CHECK(test_support::approx(local_ray.origin(), glm::vec3(-2.0f, 0.0f, 0.0f)));
  CHECK(test_support::approx(local_ray.direction(), glm::vec3(1.0f, 0.0f, 0.0f)));

  // t_world = t_local / direction_scale.
  const float t_local = 3.0f;
  const glm::vec3 local_hit = local_ray.origin() + local_ray.direction() * t_local;
  const glm::vec3 world_hit = world_ray.origin() + world_ray.direction() * (t_local / direction_scale);
  CHECK(test_support::approx(transform_position(matrix, local_hit), world_hit));
}

TEST_CASE("transform_ray_to_world reports the distance scale", "[geometry][ray]") {
  const glm::mat4 matrix = compose_trs(glm::vec3(1.0f, 2.0f, 3.0f), k_identity, glm::vec3(2.0f, 2.0f, 2.0f));
  const cyberverse::geometry::ray local_ray(glm::vec3(-2.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f));

  float direction_scale = 0.0f;
  const cyberverse::geometry::ray world_ray = transform_ray_to_world(local_ray, matrix, direction_scale);

  REQUIRE(world_ray.is_valid());
  CHECK(test_support::approx(direction_scale, 2.0f));
  CHECK(test_support::approx(world_ray.origin(), glm::vec3(-3.0f, 2.0f, 3.0f)));
  CHECK(test_support::approx(world_ray.direction(), glm::vec3(1.0f, 0.0f, 0.0f)));

  float inverse_scale = 0.0f;
  const cyberverse::geometry::ray restored = transform_ray_to_local(world_ray, matrix, inverse_scale);
  REQUIRE(restored.is_valid());
  CHECK(test_support::approx(restored.origin(), local_ray.origin()));
  CHECK(test_support::approx(restored.direction(), local_ray.direction()));
  CHECK(test_support::approx(inverse_scale * direction_scale, 1.0f));
}

TEST_CASE("ray transforms reject singular matrices", "[geometry][ray]") {
  const cyberverse::geometry::ray ray(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
  const glm::mat4 singular = compose_trs(glm::vec3(0.0f), k_identity, glm::vec3(1.0f, 0.0f, 1.0f));

  float direction_scale = 1.0f;
  CHECK_FALSE(transform_ray_to_local(ray, singular, direction_scale).is_valid());
  CHECK(test_support::approx(direction_scale, 0.0f));
  CHECK_FALSE(transform_ray_to_world(ray, singular, direction_scale).is_valid());
  CHECK(test_support::approx(direction_scale, 0.0f));
}
