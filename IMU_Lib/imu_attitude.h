/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_attitude.h
 * Do not edit locally; re-sync from the donor instead.
 */
/* Six-axis Mahony attitude estimation in the sensor frame. */
#ifndef MIDDLEWARE_IMU_ATTITUDE_H
#define MIDDLEWARE_IMU_ATTITUDE_H

#include <stdbool.h>

#include "imu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Proportional and integral gains for the accelerometer correction. */
    float proportional_gain;
    float integral_gain;
    /* Inclusive acceleration magnitude gate in standard gravity units (g). */
    float min_acceleration_magnitude_g;
    float max_acceleration_magnitude_g;
    /* Zero disables the rate gate. */
    float max_acceleration_correction_gyro_dps;
    /* Zero disables the age gate. */
    float max_acceleration_age_s;
} imu_attitude_config;

typedef struct {
    imu_acceleration_g acceleration_g;
    float age_s;
    bool valid;
    bool fresh;
    bool allow_integral_feedback;
} imu_attitude_acceleration;

typedef struct {
    imu_attitude_config config;
    imu_quaternionf quaternion;
    float integral_error_x_rad_s;
    float integral_error_y_rad_s;
    float integral_error_z_rad_s;
    imu_rotation_vector_rad previous_delta_angle_rad;
    bool previous_delta_angle_valid;
} imu_attitude;

typedef struct {
    imu_quaternionf quaternion;
    imu_euler_zyx_deg euler_zyx_deg;
} imu_attitude_output;

/* Starts at the identity quaternion. The quaternion is scalar-first and
 * represents sensor-frame orientation relative to the reference frame.
 * Sensor axes are used unchanged; board axis/sign mapping is unvalidated.
 */
void imu_attitude_init(imu_attitude *attitude,
                       const imu_attitude_config *config);

imu_quaternionf imu_quaternion_multiply(imu_quaternionf left,
                                        imu_quaternionf right);

imu_quaternionf imu_quaternion_conjugate(imu_quaternionf quaternion);

imu_quaternionf imu_quaternion_from_rotation_vector(
    imu_rotation_vector_rad rotation);

bool imu_quaternion_is_finite(imu_quaternionf quaternion);

imu_quaternionf imu_quaternion_normalize(imu_quaternionf quaternion);

/* Updates with angular rate in dps, acceleration in g, and caller-supplied
 * fixed dt_s. Acceleration correction is applied only inside the magnitude
 * gate; this rejects dynamic acceleration and makes zero acceleration safe.
 *
 * Axis-sign convention: with the sensor level the accelerometer reads
 * (0, 0, +1) g and all angles are zero. A right-hand rotation about the
 * sensor +x axis tips +y upward and reports positive roll; a right-hand
 * rotation about +y tips +x downward and reports positive pitch. Board
 * mounting may invert or permute axes; that mapping is the caller's and is
 * not applied here. Roll and pitch converge toward the measured gravity
 * vector. Yaw has no accelerometer reference and therefore only integrates
 * gyro z rate, with gyro bias causing yaw drift. Non-positive dt_s leaves
 * state unchanged.
 */
imu_attitude_output imu_attitude_update(imu_attitude *attitude,
                                        imu_gyro_dps gyro_dps,
                                        imu_acceleration_g acceleration_g,
                                        float dt_s);

/* Timed update with explicit acceleration freshness and high-rate rejection. */
imu_attitude_output imu_attitude_update_timed(
    imu_attitude *attitude, imu_gyro_dps gyro_dps,
    imu_attitude_acceleration acceleration, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* MIDDLEWARE_IMU_ATTITUDE_H */
