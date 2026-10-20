#include <catch2/catch_test_macros.hpp>

#include <array>

#include <glm/geometric.hpp>
#include <glm/matrix.hpp>

#include "cyberverse/geometry/intersect.hpp"
#include "cyberverse/transform/camera.hpp"
#include "cyberverse/transform/frustum_builder.hpp"
#include "test_helpers.hpp"

using namespace cyberverse::transform;

namespace {

struct camera_setup {
  glm::mat4 view;
  glm::mat4 projection;
  glm::mat4 view_proj;
};

camera_setup make_camera() {
  camera_setup setup;
  setup.view = look_at(glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  setup.projection = perspective(1.0f, 1.5f, 0.1f, 100.0f);
  setup.view_proj = setup.projection * setup.view;
  return setup;
}

} // namespace

TEST_CASE("frustum_from_view_proj produces inward unit planes", "[frustum][extract]") {
  const camera_setup camera = make_camera();
  const cyberverse::geometry::frustum frustum = frustum_from_view_proj(camera.view_proj);

  for (const cyberverse::geometry::plane& plane : frustum.planes) {
    CHECK(test_support::is_unit(plane.normal));
  }

  CHECK_FALSE(cyberverse::geometry::is_outside(frustum, glm::vec3(0.0f)));
  CHECK_FALSE(cyberverse::geometry::is_outside(frustum, glm::vec3(0.0f, 0.0f, 4.8f)));

  // Behind the eye, beyond the far plane, and far to the side.
  CHECK(cyberverse::geometry::is_outside(frustum, glm::vec3(0.0f, 0.0f, 5.1f)));
  CHECK(cyberverse::geometry::is_outside(frustum, glm::vec3(0.0f, 0.0f, -96.0f)));
  CHECK(cyberverse::geometry::is_outside(frustum, glm::vec3(100.0f, 0.0f, 0.0f)));

  CHECK_FALSE(cyberverse::geometry::is_outside(
      frustum, cyberverse::geometry::aabb({ -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f })));
  CHECK(cyberverse::geometry::is_outside(
      frustum, cyberverse::geometry::aabb({ 100.0f, 100.0f, 100.0f }, { 101.0f, 101.0f, 101.0f })));
}

TEST_CASE("frustum_corners unproject back to the clip cube corners", "[frustum][corners]") {
  const camera_setup camera = make_camera();

  std::array<glm::vec3, 8> corners;
  REQUIRE(frustum_corners(camera.view_proj, corners));

  const std::array<glm::vec3, 8> expected = {
    glm::vec3(-1.0f, -1.0f, 0.0f), glm::vec3(1.0f, -1.0f, 0.0f),  glm::vec3(1.0f, 1.0f, 0.0f),
    glm::vec3(-1.0f, 1.0f, 0.0f),  glm::vec3(-1.0f, -1.0f, 1.0f), glm::vec3(1.0f, -1.0f, 1.0f),
    glm::vec3(1.0f, 1.0f, 1.0f),   glm::vec3(-1.0f, 1.0f, 1.0f),
  };

  for (size_t index = 0; index < corners.size(); ++index) {
    CHECK(test_support::approx(project_ndc(camera.view_proj, corners[index]), expected[index], 1e-3f));
  }
}

TEST_CASE("frustum_bounds contains the frustum corners", "[frustum][bounds]") {
  const camera_setup camera = make_camera();
  const cyberverse::geometry::aabb bounds = frustum_bounds(camera.view_proj);

  REQUIRE(bounds.is_valid());

  std::array<glm::vec3, 8> corners;
  REQUIRE(frustum_corners(camera.view_proj, corners));
  for (const glm::vec3& corner : corners) {
    CHECK(corner.x >= bounds.min().x - test_support::k_epsilon);
    CHECK(corner.y >= bounds.min().y - test_support::k_epsilon);
    CHECK(corner.z >= bounds.min().z - test_support::k_epsilon);
    CHECK(corner.x <= bounds.max().x + test_support::k_epsilon);
    CHECK(corner.y <= bounds.max().y + test_support::k_epsilon);
    CHECK(corner.z <= bounds.max().z + test_support::k_epsilon);
  }

  CHECK_FALSE(cyberverse::geometry::is_outside(frustum_from_view_proj(camera.view_proj), bounds));
}

TEST_CASE("degenerate view-projection matrices fail safely", "[frustum][degenerate]") {
  const glm::mat4 degenerate(0.0f);
  const cyberverse::geometry::frustum frustum = frustum_from_view_proj(degenerate);

  for (const cyberverse::geometry::plane& plane : frustum.planes) {
    CHECK(plane.normal == glm::vec3(0.0f));
    CHECK(plane.d == 0.0f);
  }

  std::array<glm::vec3, 8> corners;
  CHECK_FALSE(frustum_corners(degenerate, corners));
  CHECK_FALSE(frustum_bounds(degenerate).is_valid());
}
