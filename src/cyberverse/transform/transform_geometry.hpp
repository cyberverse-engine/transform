/**
 * @file transform_geometry.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Transform geometry primitives between spaces
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "cyberverse/geometry/bounds.hpp"
#include "cyberverse/geometry/intersect.hpp"

namespace cyberverse {
namespace transform {

/**
 * @brief Transform a position (point) into another space
 *
 * Applies the full affine matrix, including translation. No perspective
 * divide is performed; use project_ndc() for clip-space output.
 *
 * @param m
 * @param p
 * @return glm::vec3
 */
glm::vec3 transform_position(const glm::mat4& m, const glm::vec3& p);

/**
 * @brief Transform a direction into another space
 *
 * Uses only the upper 3x3 part, so translation is ignored. The result is not
 * normalized; its length is affected by scale and shear.
 *
 * @param m
 * @param d
 * @return glm::vec3
 */
glm::vec3 transform_direction(const glm::mat4& m, const glm::vec3& d);

/**
 * @brief Transform a normal into another space
 *
 * Uses `transpose(inverse(mat3(m)))` so normals stay perpendicular to
 * transformed surfaces under non-uniform scale. The result is not normalized.
 * A singular linear part produces a zero vector.
 *
 * @param m
 * @param n
 * @return glm::vec3
 */
glm::vec3 transform_normal(const glm::mat4& m, const glm::vec3& n);

/**
 * @brief Transform an axis-aligned bounding box
 *
 * Transforms all eight corners, because a rotated box's transformed min/max
 * corners are not its extrema. Returns an invalid AABB for invalid input or a
 * non-finite matrix.
 *
 * @param local
 * @param m
 * @return geometry::aabb
 */
geometry::aabb transform_aabb(const geometry::aabb& local, const glm::mat4& m);

/**
 * @brief Transform a bounding sphere
 *
 * The center is transformed as a position. The radius is scaled by a
 * conservative bound on the linear part's largest singular value, computed from
 * its maximum row and column sums. The bound is exact for axis-aligned or
 * uniform scale. Returns an invalid sphere (radius -1) for invalid input or a
 * non-finite matrix.
 *
 * @param local
 * @param m
 * @return geometry::bounding_sphere
 */
geometry::bounding_sphere transform_bounding_sphere(const geometry::bounding_sphere& local, const glm::mat4& m);

/**
 * @brief Transform a plane into another space
 *
 * The normal is mapped with the inverse transpose and renormalized; the
 * distance accounts for both scale and translation. A singular matrix or a
 * degenerate plane returns a default (zero-normal) plane.
 *
 * @param local
 * @param m
 * @return geometry::plane
 */
geometry::plane transform_plane(const geometry::plane& local, const glm::mat4& m);

/**
 * @brief Transform all six planes of a frustum
 *
 * @param local
 * @param m
 * @return geometry::frustum
 */
geometry::frustum transform_frustum(const geometry::frustum& local, const glm::mat4& m);

/**
 * @brief Transform a world-space ray into the local space of a matrix
 *
 * Returns an invalid ray when @p world_ray is invalid, @p world_matrix is not
 * invertible, or the transformed direction degenerates. The local direction is
 * unit length; its distance parameter therefore differs from the world-space
 * parameter by the scale factor reported by the three-argument overload.
 *
 * @param world_ray
 * @param world_matrix Local-to-world matrix
 * @return geometry::ray
 */
geometry::ray transform_ray_to_local(const geometry::ray& world_ray, const glm::mat4& world_matrix);

/**
 * @brief Transform a world-space ray into local space and report distance scale
 *
 * @p out_direction_scale is the length of the inverse-transformed world
 * direction. A local hit distance `t_local` maps back to world space as
 * `t_world = t_local / out_direction_scale`. Set to zero on failure.
 *
 * @param world_ray
 * @param world_matrix Local-to-world matrix
 * @param out_direction_scale
 * @return geometry::ray
 */
geometry::ray
transform_ray_to_local(const geometry::ray& world_ray, const glm::mat4& world_matrix, float& out_direction_scale);

/**
 * @brief Transform a local-space ray into world space
 *
 * Returns an invalid ray when @p local_ray is invalid, @p world_matrix is not
 * invertible, or the transformed direction degenerates.
 *
 * @param local_ray
 * @param world_matrix Local-to-world matrix
 * @return geometry::ray
 */
geometry::ray transform_ray_to_world(const geometry::ray& local_ray, const glm::mat4& world_matrix);

/**
 * @brief Transform a local-space ray into world space and report distance scale
 *
 * @p out_direction_scale is the length of the transformed local direction. A
 * local hit distance `t_local` maps to world space as
 * `t_world = t_local * out_direction_scale`. Set to zero on failure.
 *
 * @param local_ray
 * @param world_matrix Local-to-world matrix
 * @param out_direction_scale
 * @return geometry::ray
 */
geometry::ray
transform_ray_to_world(const geometry::ray& local_ray, const glm::mat4& world_matrix, float& out_direction_scale);

} // namespace transform
} // namespace cyberverse
