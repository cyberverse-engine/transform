#include <benchmark/benchmark.h>

#include <array>

#include <glm/gtc/quaternion.hpp>
#include <glm/matrix.hpp>

#include "cyberverse/transform.hpp"

using namespace cyberverse::transform;

namespace {

const glm::vec3 k_translation(1.0f, 2.0f, 3.0f);
const glm::vec3 k_scale(2.0f, 3.0f, 4.0f);
const glm::quat k_rotation = glm::angleAxis(0.7f, glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f)));

const glm::mat4 k_view = look_at(glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
const glm::mat4 k_view_proj = perspective(1.0f, 1.5f, 0.1f, 100.0f) * k_view;

} // namespace

static void BM_ComposeTRS(benchmark::State& state) {
  for (auto _ : state) {
    glm::mat4 matrix = compose_trs(k_translation, k_rotation, k_scale);
    benchmark::DoNotOptimize(matrix);
  }
}

static void BM_TransformAABB(benchmark::State& state) {
  const glm::mat4 matrix = compose_trs(k_translation, k_rotation, k_scale);
  const cyberverse::geometry::aabb box({ -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f });

  for (auto _ : state) {
    cyberverse::geometry::aabb result = transform_aabb(box, matrix);
    benchmark::DoNotOptimize(result);
  }
}

static void BM_FrustumFromViewProj(benchmark::State& state) {
  for (auto _ : state) {
    cyberverse::geometry::frustum frustum = frustum_from_view_proj(k_view_proj);
    benchmark::DoNotOptimize(frustum);
  }
}

static void BM_FrustumCorners(benchmark::State& state) {
  std::array<glm::vec3, 8> corners;
  for (auto _ : state) {
    bool valid = frustum_corners(k_view_proj, corners);
    benchmark::DoNotOptimize(valid);
    benchmark::DoNotOptimize(corners);
  }
}

static void BM_RayToLocal(benchmark::State& state) {
  const glm::mat4 matrix = compose_trs(k_translation, k_rotation, k_scale);
  const cyberverse::geometry::ray world_ray(glm::vec3(0.0f, 0.0f, 10.0f), glm::vec3(0.0f, 0.0f, -1.0f));

  for (auto _ : state) {
    float scale = 0.0f;
    cyberverse::geometry::ray local_ray = transform_ray_to_local(world_ray, matrix, scale);
    benchmark::DoNotOptimize(local_ray);
    benchmark::DoNotOptimize(scale);
  }
}

static void BM_ProjectNDC(benchmark::State& state) {
  const glm::vec3 point(0.3f, -0.2f, -3.0f);
  for (auto _ : state) {
    glm::vec3 ndc = project_ndc(k_view_proj, point);
    benchmark::DoNotOptimize(ndc);
  }
}

BENCHMARK(BM_ComposeTRS);
BENCHMARK(BM_TransformAABB);
BENCHMARK(BM_FrustumFromViewProj);
BENCHMARK(BM_FrustumCorners);
BENCHMARK(BM_RayToLocal);
BENCHMARK(BM_ProjectNDC);

BENCHMARK_MAIN();
