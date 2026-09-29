/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_attitude.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_attitude.h"

#include <math.h>
#include <stdbool.h>

static const float imu_pi_f = 3.14159265358979323846f;

static bool imu_vector_is_finite(imu_rotation_vector_rad vector) {
    return isfinite(vector.x_rad) && isfinite(vector.y_rad) &&
           isfinite(vector.z_rad);
}

static imu_rotation_vector_rad imu_rotation_vector_add(
    imu_rotation_vector_rad left, imu_rotation_vector_rad right) {
    return (imu_rotation_vector_rad){
        .x_rad = left.x_rad + right.x_rad,
        .y_rad = left.y_rad + right.y_rad,
        .z_rad = left.z_rad + right.z_rad,
    };
}

static imu_rotation_vector_rad imu_rotation_vector_cross(
    imu_rotation_vector_rad left, imu_rotation_vector_rad right) {
    return (imu_rotation_vector_rad){
        .x_rad = left.y_rad * right.z_rad - left.z_rad * right.y_rad,
        .y_rad = left.z_rad * right.x_rad - left.x_rad * right.z_rad,
        .z_rad = left.x_rad * right.y_rad - left.y_rad * right.x_rad,
    };
}

static imu_rotation_vector_rad imu_rotation_vector_scale(
    imu_rotation_vector_rad vector, float scale) {
    return (imu_rotation_vector_rad){
        .x_rad = vector.x_rad * scale,
        .y_rad = vector.y_rad * scale,
        .z_rad = vector.z_rad * scale,
    };
}

static imu_euler_zyx_deg imu_quaternion_to_euler(
    imu_quaternionf quaternion) {
    const float roll_numerator =
        2.0f * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
    const float roll_denominator =
        1.0f - 2.0f * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
    const float pitch_sine =
        2.0f * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
    const float yaw_numerator =
        2.0f * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
    const float yaw_denominator =
        1.0f - 2.0f * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    float limited_pitch_sine = pitch_sine;
    if (limited_pitch_sine > 1.0f) {
        limited_pitch_sine = 1.0f;
    } else if (limited_pitch_sine < -1.0f) {
        limited_pitch_sine = -1.0f;
    }

    const float radians_to_degrees = 180.0f / imu_pi_f;
    return (imu_euler_zyx_deg){
        .roll_deg = atan2f(roll_numerator, roll_denominator) * radians_to_degrees,
        .pitch_deg = asinf(limited_pitch_sine) * radians_to_degrees,
        .yaw_deg = atan2f(yaw_numerator, yaw_denominator) * radians_to_degrees,
    };
}

static imu_attitude_output imu_attitude_output_for(
    const imu_attitude *attitude) {
    const imu_euler_zyx_deg euler =
        imu_quaternion_to_euler(attitude->quaternion);
    return (imu_attitude_output){
        .quaternion = attitude->quaternion,
        .euler_zyx_deg = euler,
        /* ZYX yaw and roll are not independently defined near vertical pitch. */
        .euler_zyx_singular = fabsf(euler.pitch_deg) >= 85.0f,
    };
}

void imu_attitude_init(imu_attitude *attitude,
                       const imu_attitude_config *config) {
    *attitude = (imu_attitude){
        .config = *config,
        .quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
    };
}

imu_attitude_output imu_attitude_update(imu_attitude *attitude,
                                        imu_gyro_dps gyro_dps,
                                        imu_acceleration_g acceleration_g,
    float dt_s) {
    return imu_attitude_update_timed(
        attitude, gyro_dps,
        (imu_attitude_acceleration){
            .acceleration_g = acceleration_g,
            .age_s = 0.0f,
            .sample_dt_s = dt_s,
            .valid = true,
            .fresh = true,
            .allow_integral_feedback = true,
        },
        dt_s);
}

imu_attitude_output imu_attitude_update_timed(
    imu_attitude *attitude, imu_gyro_dps gyro_dps,
    imu_attitude_acceleration acceleration, float dt_s) {
    if (dt_s <= 0.0f) {
        return imu_attitude_output_for(attitude);
    }

    const float acceleration_squared_g2 =
        acceleration.acceleration_g.x_g * acceleration.acceleration_g.x_g +
        acceleration.acceleration_g.y_g * acceleration.acceleration_g.y_g +
        acceleration.acceleration_g.z_g * acceleration.acceleration_g.z_g;
    const float acceleration_magnitude_g = sqrtf(acceleration_squared_g2);
    const float gyro_squared_dps2 = gyro_dps.x_dps * gyro_dps.x_dps +
                                    gyro_dps.y_dps * gyro_dps.y_dps +
                                    gyro_dps.z_dps * gyro_dps.z_dps;
    const float gyro_magnitude_dps = sqrtf(gyro_squared_dps2);
    const bool age_is_valid =
        attitude->config.max_acceleration_age_s <= 0.0f ||
        (acceleration.age_s >= 0.0f &&
         acceleration.age_s <= attitude->config.max_acceleration_age_s);
    const bool rate_is_valid =
        attitude->config.max_acceleration_correction_gyro_dps <= 0.0f ||
        gyro_magnitude_dps <=
            attitude->config.max_acceleration_correction_gyro_dps;
    const bool acceleration_is_valid = acceleration.valid && acceleration.fresh &&
        age_is_valid && rate_is_valid && isfinite(acceleration_magnitude_g) &&
        acceleration_magnitude_g >=
            attitude->config.min_acceleration_magnitude_g &&
        acceleration_magnitude_g <=
            attitude->config.max_acceleration_magnitude_g &&
        acceleration_magnitude_g > 0.0f;

    float corrected_x_rad_s =
        gyro_dps.x_dps * (imu_pi_f / 180.0f);
    float corrected_y_rad_s =
        gyro_dps.y_dps * (imu_pi_f / 180.0f);
    float corrected_z_rad_s =
        gyro_dps.z_dps * (imu_pi_f / 180.0f);

    if (acceleration_is_valid) {
        const float feedback_scale =
            acceleration.sample_dt_s > dt_s
                ? acceleration.sample_dt_s / dt_s
                : 1.0f;
        const float inverse_acceleration_magnitude =
            1.0f / acceleration_magnitude_g;
        const float ax = acceleration.acceleration_g.x_g *
                         inverse_acceleration_magnitude;
        const float ay = acceleration.acceleration_g.y_g *
                         inverse_acceleration_magnitude;
        const float az = acceleration.acceleration_g.z_g *
                         inverse_acceleration_magnitude;
        const imu_quaternionf q = attitude->quaternion;
        const float estimated_gravity_x = 2.0f * (q.x * q.z - q.w * q.y);
        const float estimated_gravity_y = 2.0f * (q.w * q.x + q.y * q.z);
        const float estimated_gravity_z =
            1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        const float error_x = ay * estimated_gravity_z -
                              az * estimated_gravity_y;
        const float error_y = az * estimated_gravity_x -
                              ax * estimated_gravity_z;
        const float error_z = ax * estimated_gravity_y -
                              ay * estimated_gravity_x;

        if (acceleration.allow_integral_feedback) {
            attitude->integral_error_x_rad_s +=
                attitude->config.integral_gain * error_x *
                acceleration.sample_dt_s;
            attitude->integral_error_y_rad_s +=
                attitude->config.integral_gain * error_y *
                acceleration.sample_dt_s;
            attitude->integral_error_z_rad_s +=
                attitude->config.integral_gain * error_z *
                acceleration.sample_dt_s;
        }
        corrected_x_rad_s += attitude->config.proportional_gain * error_x *
                                 feedback_scale +
                             attitude->integral_error_x_rad_s;
        corrected_y_rad_s += attitude->config.proportional_gain * error_y *
                                 feedback_scale +
                             attitude->integral_error_y_rad_s;
        corrected_z_rad_s += attitude->config.proportional_gain * error_z *
                                 feedback_scale +
                             attitude->integral_error_z_rad_s;
    }

    const imu_rotation_vector_rad delta_angle = {
        .x_rad = corrected_x_rad_s * dt_s,
        .y_rad = corrected_y_rad_s * dt_s,
        .z_rad = corrected_z_rad_s * dt_s,
    };
    imu_rotation_vector_rad coning_delta = delta_angle;
    if (attitude->previous_delta_angle_valid) {
        coning_delta = imu_rotation_vector_add(
            delta_angle,
            imu_rotation_vector_scale(
                imu_rotation_vector_cross(attitude->previous_delta_angle_rad,
                                          delta_angle),
                1.0f / 12.0f));
    }
    attitude->previous_delta_angle_rad = delta_angle;
    attitude->previous_delta_angle_valid = imu_vector_is_finite(delta_angle);
    const imu_quaternionf increment =
        imu_quaternion_from_rotation_vector(coning_delta);
    const imu_quaternionf next =
        imu_quaternion_multiply(attitude->quaternion, increment);
    if (imu_quaternion_is_finite(next)) {
        attitude->quaternion = imu_quaternion_normalize(next);
    } else {
        attitude->quaternion = (imu_quaternionf){1.0f, 0.0f, 0.0f, 0.0f};
        attitude->previous_delta_angle_valid = false;
    }

    return imu_attitude_output_for(attitude);
}
