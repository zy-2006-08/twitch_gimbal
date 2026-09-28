/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_calibration.h
 * Do not edit locally; re-sync from the donor instead.
 */
/* Gyroscope bias calibration using stationary samples only. */
#ifndef MIDDLEWARE_IMU_CALIBRATION_H
#define MIDDLEWARE_IMU_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_stationarity.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IMU_CALIBRATION_COLLECTING = 0,
    IMU_CALIBRATION_COMPLETE
} imu_calibration_state;

typedef struct {
    imu_stationarity_config stationarity;
    uint32_t required_samples;
    uint32_t accepted_samples;
    imu_calibration_state state;
    /* Mean stationary angular rate, interpreted as the gyro bias in dps. */
    imu_gyro_dps bias_dps;
} imu_gyro_calibration;

/* Initializes calibration. A required sample count of zero completes
 * immediately with a zero bias. No timing is stored or inferred. */
void imu_gyro_calibration_init(imu_gyro_calibration *calibration,
                               const imu_stationarity_config *stationarity,
                               uint32_t required_samples);

bool imu_gyro_calibration_update(imu_gyro_calibration *calibration,
                                 imu_gyro_dps gyro_dps,
                                 imu_acceleration_g acceleration_g);

#ifdef __cplusplus
}
#endif

#endif /* MIDDLEWARE_IMU_CALIBRATION_H */
