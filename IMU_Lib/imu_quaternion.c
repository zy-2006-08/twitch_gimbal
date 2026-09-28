/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_quaternion.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_attitude.h"

#include <math.h>

imu_quaternionf imu_quaternion_multiply(imu_quaternionf left,
                                        imu_quaternionf right) {
    return (imu_quaternionf){
        .w = left.w * right.w - left.x * right.x - left.y * right.y -
             left.z * right.z,
        .x = left.w * right.x + left.x * right.w + left.y * right.z -
             left.z * right.y,
        .y = left.w * right.y - left.x * right.z + left.y * right.w +
             left.z * right.x,
        .z = left.w * right.z + left.x * right.y - left.y * right.x +
             left.z * right.w,
    };
}

imu_quaternionf imu_quaternion_conjugate(imu_quaternionf quaternion) {
    return (imu_quaternionf){
        .w = quaternion.w,
        .x = -quaternion.x,
        .y = -quaternion.y,
        .z = -quaternion.z,
    };
}

bool imu_quaternion_is_finite(imu_quaternionf quaternion) {
    return isfinite(quaternion.w) && isfinite(quaternion.x) &&
           isfinite(quaternion.y) && isfinite(quaternion.z);
}

imu_quaternionf imu_quaternion_normalize(imu_quaternionf quaternion) {
    if (!imu_quaternion_is_finite(quaternion)) {
        return (imu_quaternionf){1.0f, 0.0f, 0.0f, 0.0f};
    }
    const float norm_squared = quaternion.w * quaternion.w +
                               quaternion.x * quaternion.x +
                               quaternion.y * quaternion.y +
                               quaternion.z * quaternion.z;
    if (!isfinite(norm_squared) || norm_squared < 1.0e-12f) {
        return (imu_quaternionf){1.0f, 0.0f, 0.0f, 0.0f};
    }
    const float norm = sqrtf(norm_squared);
    if (!isfinite(norm) || norm <= 0.0f) {
        return (imu_quaternionf){1.0f, 0.0f, 0.0f, 0.0f};
    }
    const float inverse_norm = 1.0f / norm;
    return (imu_quaternionf){
        quaternion.w * inverse_norm,
        quaternion.x * inverse_norm,
        quaternion.y * inverse_norm,
        quaternion.z * inverse_norm,
    };
}

imu_quaternionf imu_quaternion_from_rotation_vector(
    imu_rotation_vector_rad rotation) {
    const float angle_squared = rotation.x_rad * rotation.x_rad +
                                rotation.y_rad * rotation.y_rad +
                                rotation.z_rad * rotation.z_rad;
    if (!isfinite(angle_squared)) {
        return (imu_quaternionf){1.0f, 0.0f, 0.0f, 0.0f};
    }

    const float half_angle_squared = 0.25f * angle_squared;
    float scalar;
    float vector_scale;
    if (angle_squared < 1.0e-8f) {
        scalar = 1.0f - half_angle_squared * 0.5f +
                 half_angle_squared * half_angle_squared / 24.0f;
        vector_scale = 0.5f - angle_squared / 48.0f +
                       angle_squared * angle_squared / 3840.0f;
    } else {
        const float angle = sqrtf(angle_squared);
        const float half_angle = 0.5f * angle;
        scalar = cosf(half_angle);
        vector_scale = sinf(half_angle) / angle;
    }
    return imu_quaternion_normalize((imu_quaternionf){
        .w = scalar,
        .x = rotation.x_rad * vector_scale,
        .y = rotation.y_rad * vector_scale,
        .z = rotation.z_rad * vector_scale,
    });
}
