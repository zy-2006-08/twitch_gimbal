/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_types.h
 * Do not edit locally; re-sync from the donor instead.
 */
/* Shared value types for the pure IMU middleware.
 *
 * Sensor-frame identity is the only convention implemented here: x, y, and
 * z are consumed exactly as supplied by the caller. The board-level axis and
 * sign mapping is intentionally unvalidated and is not applied in middleware.
 */
#ifndef MIDDLEWARE_IMU_TYPES_H
#define MIDDLEWARE_IMU_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Angular rate in degrees per second (dps), in sensor-frame x/y/z. */
typedef struct {
    float x_dps;
    float y_dps;
    float z_dps;
} imu_gyro_dps;

typedef struct {
    float x_rad;
    float y_rad;
    float z_rad;
} imu_rotation_vector_rad;

/* Linear acceleration in standard gravity units (g), in sensor-frame x/y/z. */
typedef struct {
    float x_g;
    float y_g;
    float z_g;
} imu_acceleration_g;

/* Angular acceleration in degrees per second squared (dps2). */
typedef struct {
    float x_dps2;
    float y_dps2;
    float z_dps2;
} imu_angular_acceleration_dps2;

/* Unit quaternion, stored as scalar-first (w, x, y, z). */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} imu_quaternionf;

/* ZYX yaw-pitch-roll angles in degrees: roll x, pitch y, yaw z. */
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} imu_euler_zyx_deg;

#ifdef __cplusplus
}
#endif

#endif /* MIDDLEWARE_IMU_TYPES_H */
