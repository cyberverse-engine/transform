/**
 * @file frustum_builder.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Frustum extraction from a view-projection matrix
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <array>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/geometry/intersect.hpp"

namespace cyberverse {
namespace transform {

/**
 * @brief Extract a frustum from a view-projection matrix
 *
 * The matrix must use a right-handed [0, 1] depth range, as produced by
 * perspective() and orthographic(). The six returned planes have inward-facing
 * unit normals and follow geometry::frustum's order: Left, Right, Bottom, Top,
 * Near, Far. A degenerate or non-finite matrix returns a default frustum whose
 * zero planes cull nothing.
 *
 * @param view_proj
 * @return geometry::frustum
 */
geometry::frustum frustum_from_view_proj(const glm::mat4& view_proj);

/**
 * @brief Extract the eight world-space corners of a view-projection frustum
 *
 * Corners are ordered near-bottom-left, near-bottom-right, near-top-right,
 * near-top-left, then the same four on the far plane. @p out_corners is only
 * modified on success. Returns false when the matrix is degenerate or
 * non-finite.
 *
 * @param view_proj
 * @param out_corners
 * @return true on success
 */
bool frustum_corners(const glm::mat4& view_proj, std::array<glm::vec3, 8>& out_corners);

/**
 * @brief Compute the world-space AABB of a view-projection frustum
 *
 * Returns an invalid AABB when the frustum cannot be extracted.
 *
 * @param view_proj
 * @return geometry::aabb
 */
geometry::aabb frustum_bounds(const glm::mat4& view_proj);

} // namespace transform
} // namespace cyberverse
