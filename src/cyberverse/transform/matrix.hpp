/**
 * @file matrix.hpp
 * @author simon2077s (simon2077s@gmail.com)
 * @brief Matrix composition and rotation helpers
 * @version 0.1
 * @date 2026-10-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace cyberverse {
namespace transform {

/**
 * @brief Compose a translation/rotation/scale matrix
 *
 * The returned matrix is `T * R * S`, with the rotation supplied as a
 * quaternion. Non-unit quaternions are normalized; an invalid quaternion is
 * treated as identity.
 *
 * @param translation Translation in the parent space
 * @param rotation Rotation in the parent space
 * @param scale Scale applied before rotation
 * @return glm::mat4
 */
glm::mat4 compose_trs(const glm::vec3& translation, const glm::quat& rotation, const glm::vec3& scale);

/**
 * @brief Compose a translation/rotation/scale matrix
 *
 * The rotation matrix is used as a pure rotation: its translation column is
 * replaced by @p translation. The returned matrix is `T * R * S`.
 *
 * @param translation Translation in the parent space
 * @param rotation Rotation matrix in the parent space
 * @param scale Scale applied before rotation
 * @return glm::mat4
 */
glm::mat4 compose_trs(const glm::vec3& translation, const glm::mat4& rotation, const glm::vec3& scale);

/**
 * @brief Rotation matrix around the X axis
 *
 * Right-handed, positive angles rotate +Y towards +Z.
 *
 * @param radians
 * @return glm::mat4
 */
glm::mat4 rotation_x(float radians);

/**
 * @brief Rotation matrix around the Y axis
 *
 * Right-handed, positive angles rotate +Z towards +X.
 *
 * @param radians
 * @return glm::mat4
 */
glm::mat4 rotation_y(float radians);

/**
 * @brief Rotation matrix around the Z axis
 *
 * Right-handed, positive angles rotate +X towards +Y.
 *
 * @param radians
 * @return glm::mat4
 */
glm::mat4 rotation_z(float radians);

/**
 * @brief Rotation matrix around an arbitrary axis
 *
 * The axis is normalized. A zero, non-finite axis or a non-finite angle
 * returns the identity matrix.
 *
 * @param axis
 * @param radians
 * @return glm::mat4
 */
glm::mat4 rotation_axis_angle(const glm::vec3& axis, float radians);

/**
 * @brief Normal matrix of an affine transform
 *
 * Returns `transpose(inverse(mat3(matrix)))`, which maps normals from the
 * source space to the space transformed by @p matrix. A singular linear part
 * yields a zero matrix.
 *
 * @param matrix
 * @return glm::mat3
 */
glm::mat3 normal_matrix(const glm::mat4& matrix);

} // namespace transform
} // namespace cyberverse
