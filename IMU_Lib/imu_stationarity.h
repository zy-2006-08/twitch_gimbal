/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_stationarity.h
 * Do not edit locally; re-sync from the donor instead.
 */
/* Stationarity checks shared by calibration and drift statistics. */
#ifndef MIDDLEWARE_IMU_STATIONARITY_H
#define MIDDLEWARE_IMU_STATIONARITY_H

#include <stdbool.h>

#include "imu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Maximum allowed gyro vector magnitude in degrees per second (dps). */
    float max_gyro_magnitude_dps;
    /* Inclusive acceleration magnitude gate in standard gravity units (g). */
    float min_acceleration_magnitude_g;
    float max_acceleration_magnitude_g;
} imu_stationarity_config;

/* Returns true only when the gyro is quiet and acceleration is near 1 g.
 * Zero acceleration is safe and is rejected when the minimum gate is positive.
 */
bool imu_stationarity_is_stationary(const imu_stationarity_config *config,
                                    imu_gyro_dps gyro_dps,
                                    imu_acceleration_g acceleration_g);

#ifdef __cplusplus
}
#endif

#endif /* MIDDLEWARE_IMU_STATIONARITY_H */
