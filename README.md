# transform

Cyberverse Engine transform and camera math library.

`transform` is a C++23 static library (`cyberverse::transform::static_lib`) built on top of `cyberverse::geometry::static_lib`. It provides:

- TRS composition (`T * R * S`), axis/angle rotations, and normal matrices.
- A `trs` value type with affine decomposition and matrix-valued inversion.
- Right-handed perspective, off-center perspective, and orthographic projections with a `[0, 1]` depth range.
- `look_at` and orientation-based view matrices.
- NDC project/unproject and world-space picking rays.
- Frustum extraction, world-space corners, and bounds.
- Transform helpers for points, directions, normals, AABBs, bounding spheres, planes, frusta, and rays.

The umbrella header is `cyberverse/transform.hpp`.

## Build

```bash
cmake -S . -B build
cmake --build build
```

The configure step resolves `cyberverse::geometry::static_lib` from an installed package, `CYBERVERSE_GEOMETRY_DIR`, the sibling `../geometry` tree, or FetchContent. GLM is inherited from geometry. The public target defines `GLM_FORCE_DEPTH_ZERO_TO_ONE`, so consumers get the same right-handed `[0, 1]` clip-space convention.

## Test

Tests build and run outside the repository:

```bash
cmake -S . -B /tmp/cyberverse-transform-test
cmake --build /tmp/cyberverse-transform-test
ctest --test-dir /tmp/cyberverse-transform-test --output-on-failure
```

Catch2 v3.5.2 and Google Benchmark v1.9.5 are fetched by default. Disable them with `-DCYBERVERSE_TRANSFORM_BUILD_TESTS=OFF` and `-DCYBERVERSE_TRANSFORM_BUILD_BENCHMARKS=OFF`.

## Example

```cpp
#include "cyberverse/transform.hpp"

using namespace cyberverse::transform;

const glm::mat4 model = compose_trs(glm::vec3(1.0f, 2.0f, 3.0f), rotation_y(1.57f), glm::vec3(1.0f));
const glm::mat4 view = look_at(glm::vec3(0.0f, 2.0f, 5.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
const glm::mat4 projection = perspective(1.05f, 16.0f / 9.0f, 0.1f, 100.0f);
const glm::mat4 view_projection = projection * view;

const cyberverse::geometry::aabb local_bounds({ -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f });
const cyberverse::geometry::frustum frustum = frustum_from_view_proj(view_projection);
const cyberverse::geometry::aabb world_bounds = transform_aabb(local_bounds, model);
const bool visible = !cyberverse::geometry::is_outside(frustum, world_bounds);
```

See `AGENTS.md` for architecture, dependency resolution, conventions, and testing rules.
