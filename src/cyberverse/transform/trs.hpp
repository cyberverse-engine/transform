/**
 * @file trs.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Translation/rotation/scale transform
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace cyberverse {
namespace transform {

/**
 * @brief Translation/rotation/scale transform
 *
 * The equivalent matrix is `T * R * S`, matching compose_trs(). The rotation
 * is expected to be a unit quaternion.
 */
struct trs {
  glm::vec3 translation{ 0.0f };
  glm::quat rotation{ 1.0f, 0.0f, 0.0f, 0.0f };
  glm::vec3 scale{ 1.0f };

  /**
   * @brief Convert to a 4x4 matrix
   *
   * @return glm::mat4
   */
  glm::mat4 matrix() const;
};

/**
 * @brief Compose a trs value into a 4x4 matrix
 *
 * @param value
 * @return glm::mat4
 */
glm::mat4 compose_trs(const trs& value);

/**
 * @brief Decompose an affine matrix into translation/rotation/scale
 *
 * The matrix must be affine, have three non-zero scale axes, and have no
 * shear. A negative determinant is represented by a negative X scale. Returns
 * false for singular, sheared, projective, or non-finite matrices.
 *
 * @param matrix
 * @param out Receives the decomposed value only on success
 * @return true on success
 */
bool decompose_trs(const glm::mat4& matrix, trs& out);

/**
 * @brief Inverse of a translation/rotation/scale transform
 *
 * Returns the inverse as a matrix. A non-uniform scale followed by a rotation
 * does not generally have a component-wise TRS inverse, because the inverse
 * linear part can contain shear; returning a matrix avoids inventing a
 * rotation and scale that do not exist. Returns false when the transform is
 * singular or non-finite.
 *
 * @param value
 * @param out Receives the inverse matrix only on success
 * @return true on success
 */
bool inverse_trs(const trs& value, glm::mat4& out);

} // namespace transform
} // namespace cyberverse
