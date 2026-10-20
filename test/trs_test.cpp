#include <catch2/catch_test_macros.hpp>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "cyberverse/transform/matrix.hpp"
#include "cyberverse/transform/transform_geometry.hpp"
#include "cyberverse/transform/trs.hpp"
#include "test_helpers.hpp"

using namespace cyberverse::transform;

namespace {

trs sample_transform() {
  trs value;
  value.translation = glm::vec3(1.0f, -2.0f, 3.0f);
  value.rotation = glm::angleAxis(0.7f, glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f)));
  value.scale = glm::vec3(2.0f, 3.0f, 4.0f);
  return value;
}

} // namespace

TEST_CASE("trs converts to the same matrix as compose_trs", "[trs]") {
  const trs value = sample_transform();
  CHECK(test_support::approx(value.matrix(), compose_trs(value.translation, value.rotation, value.scale)));
  CHECK(test_support::approx(compose_trs(value), value.matrix()));
}

TEST_CASE("decompose_trs round-trips translation, rotation, and scale", "[trs][decompose]") {
  const trs input = sample_transform();
  const glm::mat4 matrix = input.matrix();

  trs output;
  REQUIRE(decompose_trs(matrix, output));

  CHECK(test_support::approx(output.translation, input.translation));
  CHECK(test_support::approx(output.scale, input.scale));
  CHECK(test_support::approx(std::abs(glm::dot(output.rotation, input.rotation)), 1.0f));
  CHECK(test_support::approx(output.matrix(), matrix));
}

TEST_CASE("decompose_trs represents negative determinants with a negative axis", "[trs][decompose]") {
  trs input;
  input.scale = glm::vec3(-2.0f, 1.0f, 1.0f);
  const glm::mat4 matrix = input.matrix();

  trs output;
  REQUIRE(decompose_trs(matrix, output));
  CHECK(test_support::approx(output.scale.x, -2.0f));
  CHECK(test_support::approx(output.matrix(), matrix));
}

TEST_CASE("decompose_trs rejects sheared and projective matrices", "[trs][decompose]") {
  glm::mat4 sheared(1.0f);
  sheared[1][0] = 0.5f;

  glm::mat4 projective(1.0f);
  projective[2][3] = 0.25f;

  trs output;
  CHECK_FALSE(decompose_trs(sheared, output));
  CHECK_FALSE(decompose_trs(projective, output));
  CHECK_FALSE(decompose_trs(glm::mat4(0.0f), output));
}

TEST_CASE("inverse_trs inverts non-uniform scale and translation", "[trs][inverse]") {
  const trs input = sample_transform();
  glm::mat4 inverse;
  REQUIRE(inverse_trs(input, inverse));

  CHECK(test_support::approx(input.matrix() * inverse, glm::mat4(1.0f), 1e-3f));
  CHECK(test_support::approx(inverse * input.matrix(), glm::mat4(1.0f), 1e-3f));

  const glm::vec3 point(0.5f, -1.0f, 2.0f);
  const glm::vec3 world = transform_position(input.matrix(), point);
  CHECK(test_support::approx(transform_position(inverse, world), point, 1e-3f));
}

TEST_CASE("inverse_trs rejects zero scale", "[trs][inverse]") {
  trs input;
  input.scale = glm::vec3(1.0f, 0.0f, 1.0f);

  glm::mat4 output(1.0f);
  CHECK_FALSE(inverse_trs(input, output));
}
