#include <catch2/catch_test_macros.hpp>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/matrix.hpp>
#include <glm/vec4.hpp>

#include "cyberverse/transform/camera.hpp"
#include "test_helpers.hpp"

using namespace cyberverse::transform;

namespace {

constexpr float k_half_pi = 1.5707963267948966f;

} // namespace

TEST_CASE("perspective maps the near and far planes to [0, 1]", "[camera][perspective]") {
  const float near_z = 0.1f;
  const float far_z = 100.0f;
  const glm::mat4 projection = perspective(1.0f, 1.5f, near_z, far_z);

  const glm::vec4 near_clip = projection * glm::vec4(0.0f, 0.0f, -near_z, 1.0f);
  const glm::vec4 far_clip = projection * glm::vec4(0.0f, 0.0f, -far_z, 1.0f);

  CHECK(test_support::approx(near_clip.z / near_clip.w, 0.0f));
  CHECK(test_support::approx(far_clip.z / far_clip.w, 1.0f));

  const glm::vec4 center_clip = projection * glm::vec4(0.0f, 0.0f, -1.0f, 1.0f);
  CHECK(test_support::approx(center_clip.x, 0.0f));
  CHECK(test_support::approx(center_clip.y, 0.0f));
  CHECK(center_clip.z / center_clip.w > 0.0f);
  CHECK(center_clip.z / center_clip.w < 1.0f);
}

TEST_CASE("perspective rejects invalid parameters", "[camera][perspective]") {
  const glm::mat4 identity(1.0f);
  CHECK(test_support::approx(perspective(0.0f, 1.0f, 0.1f, 10.0f), identity));
  CHECK(test_support::approx(perspective(1.0f, 0.0f, 0.1f, 10.0f), identity));
  CHECK(test_support::approx(perspective(1.0f, 1.0f, 0.0f, 10.0f), identity));
  CHECK(test_support::approx(perspective(1.0f, 1.0f, 10.0f, 10.0f), identity));
}

TEST_CASE("perspective_offcenter maps an asymmetric frustum to the clip cube", "[camera][perspective]") {
  const float near_z = 1.0f;
  const float far_z = 10.0f;
  const glm::mat4 projection = perspective_offcenter(-1.0f, 2.0f, -0.5f, 0.5f, near_z, far_z);

  const glm::vec4 near_clip = projection * glm::vec4(-1.0f, -0.5f, -near_z, 1.0f);
  CHECK(test_support::approx(near_clip.x / near_clip.w, -1.0f));
  CHECK(test_support::approx(near_clip.y / near_clip.w, -1.0f));
  CHECK(test_support::approx(near_clip.z / near_clip.w, 0.0f));

  const glm::vec4 far_clip = projection * glm::vec4(2.0f * far_z / near_z, 0.5f * far_z / near_z, -far_z, 1.0f);
  CHECK(test_support::approx(far_clip.x / far_clip.w, 1.0f));
  CHECK(test_support::approx(far_clip.y / far_clip.w, 1.0f));
  CHECK(test_support::approx(far_clip.z / far_clip.w, 1.0f));
}

TEST_CASE("orthographic maps the box corners to the clip cube", "[camera][orthographic]") {
  const glm::mat4 projection = orthographic(-2.0f, 2.0f, -1.0f, 1.0f, 0.5f, 50.0f);

  const glm::vec4 near_clip = projection * glm::vec4(-2.0f, -1.0f, -0.5f, 1.0f);
  CHECK(test_support::approx(near_clip.x, -1.0f));
  CHECK(test_support::approx(near_clip.y, -1.0f));
  CHECK(test_support::approx(near_clip.z, 0.0f));

  const glm::vec4 far_clip = projection * glm::vec4(2.0f, 1.0f, -50.0f, 1.0f);
  CHECK(test_support::approx(far_clip.x, 1.0f));
  CHECK(test_support::approx(far_clip.y, 1.0f));
  CHECK(test_support::approx(far_clip.z, 1.0f));
}

TEST_CASE("look_at places the eye at the origin and looks down -Z", "[camera][look_at]") {
  const glm::vec3 eye(3.0f, 4.0f, 5.0f);
  const glm::vec3 target(0.0f, 1.0f, 0.0f);
  const glm::mat4 view = look_at(eye, target, glm::vec3(0.0f, 1.0f, 0.0f));

  CHECK(test_support::approx(glm::vec3(view * glm::vec4(eye, 1.0f)), glm::vec3(0.0f)));
  CHECK(test_support::approx(glm::vec3(view * glm::vec4(target, 1.0f)),
                             glm::vec3(0.0f, 0.0f, -glm::length(target - eye))));

  const glm::vec3 up_in_view = glm::vec3(view * glm::vec4(eye + glm::vec3(0.0f, 1.0f, 0.0f), 1.0f));
  CHECK(up_in_view.y > 0.0f);
}

TEST_CASE("look_at falls back safely for degenerate input", "[camera][look_at]") {
  const glm::mat4 identity(1.0f);
  CHECK(test_support::approx(look_at(glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f)), identity));

  // Up parallel to the view direction: a fallback axis must still produce a valid basis.
  const glm::mat4 top_down = look_at(glm::vec3(0.0f, 5.0f, 0.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  CHECK(test_support::approx(glm::vec3(top_down * glm::vec4(0.0f, 5.0f, 0.0f, 1.0f)), glm::vec3(0.0f)));
  CHECK(test_support::approx(glm::vec3(top_down * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)), glm::vec3(0.0f, 0.0f, -5.0f)));
}

TEST_CASE("view_from_orientation matches the camera basis", "[camera][view]") {
  const glm::quat orientation = glm::angleAxis(k_half_pi, glm::vec3(0.0f, 1.0f, 0.0f));
  const glm::vec3 eye(0.0f, 0.0f, 5.0f);
  const glm::vec3 forward = view_direction(orientation);

  CHECK(test_support::approx(forward, glm::vec3(-1.0f, 0.0f, 0.0f)));

  const glm::mat4 view = view_from_orientation(eye, orientation);
  CHECK(test_support::approx(glm::vec3(view * glm::vec4(eye, 1.0f)), glm::vec3(0.0f)));
  CHECK(test_support::approx(glm::vec3(view * glm::vec4(eye + forward, 1.0f)), glm::vec3(0.0f, 0.0f, -1.0f)));
}

TEST_CASE("project_ndc and unproject_ndc round-trip", "[camera][project]") {
  const glm::mat4 view =
      look_at(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  const glm::mat4 view_proj = perspective(1.0f, 1.5f, 0.1f, 100.0f) * view;
  const glm::vec3 world(0.3f, -0.2f, -3.0f);

  const glm::vec3 ndc = project_ndc(view_proj, world);
  REQUIRE(ndc.z >= 0.0f);
  REQUIRE(ndc.z <= 1.0f);

  const glm::vec3 restored = unproject_ndc(glm::inverse(view_proj), ndc);
  CHECK(test_support::approx(restored, world, 1e-3f));
}

TEST_CASE("project_ndc returns NaN for a non-invertible view", "[camera][project]") {
  const glm::vec3 ndc = project_ndc(glm::mat4(0.0f), glm::vec3(1.0f, 2.0f, 3.0f));
  CHECK(std::isnan(ndc.x));
}

TEST_CASE("ray_from_ndc starts on the near plane and points forward", "[camera][ray]") {
  const glm::mat4 view_proj = perspective(1.0f, 1.5f, 0.1f, 100.0f) *
                              look_at(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  const glm::mat4 inverse_view_proj = glm::inverse(view_proj);

  const cyberverse::geometry::ray center = ray_from_ndc(inverse_view_proj, glm::vec2(0.0f, 0.0f));
  REQUIRE(center.is_valid());
  CHECK(test_support::approx(center.origin(), glm::vec3(0.0f, 0.0f, -0.1f)));
  CHECK(test_support::approx(center.direction(), glm::vec3(0.0f, 0.0f, -1.0f)));

  const cyberverse::geometry::ray corner = ray_from_ndc(inverse_view_proj, glm::vec2(1.0f, 1.0f));
  REQUIRE(corner.is_valid());
  CHECK(corner.direction().x > 0.0f);
  CHECK(corner.direction().y > 0.0f);
  CHECK(corner.direction().z < 0.0f);

  CHECK_FALSE(ray_from_ndc(glm::mat4(0.0f), glm::vec2(0.0f, 0.0f)).is_valid());
}
