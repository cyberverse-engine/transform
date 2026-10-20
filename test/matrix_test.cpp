#include <catch2/catch_test_macros.hpp>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "cyberverse/transform/matrix.hpp"
#include "cyberverse/transform/transform_geometry.hpp"
#include "test_helpers.hpp"

using namespace cyberverse::transform;

namespace {

constexpr float k_half_pi = 1.5707963267948966f;

glm::quat yaw_90() { return glm::angleAxis(k_half_pi, glm::vec3(0.0f, 1.0f, 0.0f)); }

} // namespace

TEST_CASE("compose_trs applies scale, then rotation, then translation", "[matrix][compose_trs]") {
  const glm::mat4 matrix = compose_trs(glm::vec3(1.0f, 2.0f, 3.0f), yaw_90(), glm::vec3(2.0f, 3.0f, 4.0f));

  CHECK(test_support::approx(transform_position(matrix, glm::vec3(0.0f, 0.0f, 0.0f)), glm::vec3(1.0f, 2.0f, 3.0f)));
  CHECK(test_support::approx(transform_position(matrix, glm::vec3(1.0f, 0.0f, 0.0f)), glm::vec3(1.0f, 2.0f, 1.0f)));
  CHECK(test_support::approx(transform_position(matrix, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(1.0f, 5.0f, 3.0f)));
}

TEST_CASE("compose_trs accepts a rotation matrix", "[matrix][compose_trs]") {
  const glm::vec3 translation(1.0f, 2.0f, 3.0f);
  const glm::vec3 scale(2.0f, 3.0f, 4.0f);
  const glm::mat4 from_quaternion = compose_trs(translation, yaw_90(), scale);
  const glm::mat4 from_matrix = compose_trs(translation, rotation_y(k_half_pi), scale);

  CHECK(test_support::approx(from_matrix, from_quaternion));
}

TEST_CASE("compose_trs normalizes quaternions and ignores invalid ones", "[matrix][compose_trs]") {
  const glm::vec3 translation(0.0f, 0.0f, 0.0f);
  const glm::vec3 scale(1.0f, 1.0f, 1.0f);
  const glm::quat non_unit(2.0f, 0.0f, 0.0f, 0.0f);

  CHECK(test_support::approx(compose_trs(translation, non_unit, scale), glm::mat4(1.0f)));
  CHECK(test_support::approx(compose_trs(translation, glm::quat(0.0f, 0.0f, 0.0f, 0.0f), scale), glm::mat4(1.0f)));
}

TEST_CASE("axis rotations follow the right-handed convention", "[matrix][rotation]") {
  CHECK(test_support::approx(transform_direction(rotation_x(k_half_pi), glm::vec3(0.0f, 1.0f, 0.0f)),
                             glm::vec3(0.0f, 0.0f, 1.0f)));
  CHECK(test_support::approx(transform_direction(rotation_y(k_half_pi), glm::vec3(0.0f, 0.0f, 1.0f)),
                             glm::vec3(1.0f, 0.0f, 0.0f)));
  CHECK(test_support::approx(transform_direction(rotation_z(k_half_pi), glm::vec3(1.0f, 0.0f, 0.0f)),
                             glm::vec3(0.0f, 1.0f, 0.0f)));
}

TEST_CASE("rotation_axis_angle normalizes the axis and rejects degenerate input", "[matrix][rotation]") {
  CHECK(test_support::approx(rotation_axis_angle(glm::vec3(0.0f, 5.0f, 0.0f), k_half_pi), rotation_y(k_half_pi)));
  CHECK(test_support::approx(rotation_axis_angle(glm::vec3(0.0f, 0.0f, 0.0f), k_half_pi), glm::mat4(1.0f)));
}

TEST_CASE("normal_matrix is the inverse transpose of the linear part", "[matrix][normal]") {
  const glm::mat4 matrix =
      compose_trs(glm::vec3(0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(2.0f, 1.0f, 1.0f));
  CHECK(test_support::approx(normal_matrix(matrix) * glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(0.5f, 1.0f, 0.0f)));
  CHECK(test_support::approx(transform_normal(matrix, glm::vec3(1.0f, 1.0f, 0.0f)), glm::vec3(0.5f, 1.0f, 0.0f)));
}

TEST_CASE("normal_matrix and transform_normal handle singular transforms", "[matrix][normal]") {
  const glm::mat4 matrix =
      compose_trs(glm::vec3(0.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 1.0f));
  CHECK(test_support::approx(normal_matrix(matrix) * glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f)));
  CHECK(test_support::approx(transform_normal(matrix, glm::vec3(1.0f, 0.0f, 0.0f)), glm::vec3(0.0f)));
}
