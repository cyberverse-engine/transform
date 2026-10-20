# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository

`transform` is the standalone transform/camera math module of Cyberverse Engine: C++23, GLM, built as the static library `cyberverse::transform::static_lib`. It depends on `cyberverse::geometry::static_lib` and exposes its API from `src/cyberverse/transform/`; `src/cyberverse/transform.hpp` is the umbrella header.

The library currently provides:

- `matrix.hpp` — TRS composition, axis/angle rotations, normal matrices.
- `trs.hpp` — a translation/rotation/scale value type, matrix decomposition, and matrix-valued inversion.
- `camera.hpp` — right-handed perspective/off-center/orthographic projections, view matrices, NDC project/unproject, and picking rays.
- `frustum_builder.hpp` — frustum plane extraction, world-space corners, and bounds.
- `transform_geometry.hpp` — transforms for points, directions, normals, AABBs, bounding spheres, planes, frusta, and rays.

## Build and test

Run from this directory. Tests must be built and run under `/tmp`, never inside the repo.

```bash
cmake -S . -B /tmp/cyberverse-transform-test
cmake --build /tmp/cyberverse-transform-test
ctest --test-dir /tmp/cyberverse-transform-test --output-on-failure
```

- The first configure fetches GLM through `geometry`, plus Catch2 v3.5.2 and Google Benchmark v1.9.5. Skip the latter two with `-DCYBERVERSE_TRANSFORM_BUILD_TESTS=OFF -DCYBERVERSE_TRANSFORM_BUILD_BENCHMARKS=OFF`.
- Presets: configure once with `cmake --preset debug`, then build with `cmake --build --preset debug` (same for `release`); outputs go to `build/` and `build-release/`. Tests still must be built and run under `/tmp`.
- `test/` builds `cyberverse_transform_tests` (Catch2, discovered by CTest); `benchmark/` builds `cyberverse_transform_benchmarks` (Google Benchmark).
- Format with the repository `.clang-format` (LLVM base, 2-space indent, 120 columns, attached braces, left-aligned pointers): `clang-format -i <files>`.

### Geometry dependency

`cmake/engine_dep_geometry.cmake` resolves `cyberverse::geometry::static_lib` idempotently. The order is:

1. An already-defined target.
2. An installed `cyberverse_geometry` CMake package.
3. A source tree from `CYBERVERSE_GEOMETRY_DIR` (CMake variable or environment variable), otherwise the sibling `../geometry`.
4. `FetchContent` from `CYBERVERSE_GEOMETRY_GIT_REPOSITORY` (default `git@github.com:cyberverse-engine/geometry.git`) at `CYBERVERSE_GEOMETRY_GIT_TAG` (shallow unless `CYBERVERSE_GEOMETRY_GIT_SHALLOW=OFF`).

`github-simon2077s.com` is a local SSH alias used by developer repo remotes only. Never use it in a FetchContent URL; use the real public host `git@github.com`.

When geometry is vendored, its tests and benchmarks are forced off. GLM is inherited transitively from geometry. The transform target publishes `GLM_FORCE_DEPTH_ZERO_TO_ONE`; consumers must get that definition from the CMake target (or define it before including any GLM header).

## Coordinate and API conventions

- Right-handed, Y-up. A camera looks down its local `-Z` axis. Clip-space depth is `[0,1]`, matching `GLM_FORCE_DEPTH_ZERO_TO_ONE` and `glm::perspectiveRH_ZO` / `glm::orthoRH_ZO`.
- Matrices are column-major GLM matrices. `compose_trs` and `trs::matrix()` mean `M = T * R * S` (scale first, then rotation, then translation).
- `geometry::frustum` stores six inward-facing planes with unit normals in the order Left, Right, Bottom, Top, Near, Far. `dot(normal, p) - d >= 0` is inside.
- `transform_position` applies the full affine matrix and does not perform a perspective divide; use `project_ndc` for clip space.
- `transform_direction` ignores translation and does not normalize. `transform_normal` uses the inverse-transpose and does not normalize; a singular linear part gives a zero vector.
- `transform_aabb` transforms all eight corners. `transform_bounding_sphere` uses a conservative norm bound from the linear part's row and column sums (exact for axis-aligned or uniform scale).
- `transform_plane` maps the normal with the inverse transpose and accounts for both scale and translation in `d`.
- Ray helpers normalize the transformed direction, so the distance parameter changes. The three-argument overloads report the direction scale: `world t = local t / scale` for world-to-local conversion, and `world t = local t * scale` for local-to-world conversion.
- `decompose_trs` only accepts affine, shear-free matrices with three non-zero axes; a negative determinant becomes a negative X scale. `inverse_trs` returns a matrix because the inverse of non-uniform scale followed by rotation can contain shear and has no component-wise TRS form.
- Invalid input is reported, not propagated as NaNs: projections and `look_at` return identity, `frustum_from_view_proj` returns a non-culling default frustum, `transform_aabb` returns an invalid AABB, `transform_bounding_sphere` returns a radius `-1` sphere, ray helpers return invalid rays, and `project_ndc` / `unproject_ndc` return NaN vectors.

## Style

- Public types and functions are `lower_snake_case` (`compose_trs`, `frustum_corners`); private members end with `_`.
- Headers use `#pragma once`, live under `src/cyberverse/transform/`, and are included as `"cyberverse/transform/<name>.hpp"`. Internal helpers live in `src/cyberverse/transform/detail/` and are not part of the public API.
- Add new public headers to `HEADER_FILES` and `src/cyberverse/transform.hpp`; add new sources to `SOURCE_FILES`.
- Keep the Doxygen `@file` header at the top of new files.

## Git

- Existing repo remotes (`origin`) use the local SSH alias `github-simon2077s.com`; don't rewrite them. This alias is not available on other machines or CI.
- Build-time fetches must use the real public host `git@github.com`, never the alias.
- Commit directly on `main`, using Conventional Commits (`feat:`, `fix:`, `docs:`, `chore:`).

`AGENTS.md` is a symlink to this file; edit only `CLAUDE.md`.
