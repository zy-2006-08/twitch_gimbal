/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_drift.h
 * Do not edit locally; re-sync from the donor instead.
 */
/* Stationary gyroscope drift statistics. */
#ifndef MIDDLEWARE_IMU_DRIFT_H
#define MIDDLEWARE_IMU_DRIFT_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_stationarity.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    imu_stationarity_config stationarity;
    uint32_t sample_count;
    double duration_s;
    double mean_x_dps;
    double mean_y_dps;
    double mean_z_dps;
    double sum_squared_deviation_x_dps2;
    double sum_squared_deviation_y_dps2;
    double sum_squared_deviation_z_dps2;
    double yaw_drift_deg;
} imu_gyro_drift;

typedef struct {
    uint32_t sample_count;
    float observed_duration_s;
    imu_gyro_dps mean_dps;
    /* Per-axis root-mean-square rate noise about the mean in dps. A pure
     * constant bias therefore reports zero noise; see mean_dps for the bias.
     */
    imu_gyro_dps rms_dps;
    /* Signed integrated z-axis drift in degrees. */
    float yaw_drift_deg;
    /* Signed z-axis drift rate in degrees per minute (deg/min). */
    float yaw_drift_deg_per_min;
} imu_drift_statistics;

void imu_gyro_drift_init(imu_gyro_drift *drift,
                         const imu_stationarity_config *stationarity);

/* Accepts a sample only while stationary and with a positive caller-supplied
 * dt_s. The z-axis rate is integrated as signed yaw drift. */
bool imu_gyro_drift_update(imu_gyro_drift *drift,
                           imu_gyro_dps gyro_dps,
                           imu_acceleration_g acceleration_g,
                           float dt_s);

/* Records every valid rate sample without applying a stationarity gate. Use
 * this for a wall-clock hold-out where motion must not disappear from yaw. */
bool imu_gyro_drift_update_ungated(imu_gyro_drift *drift,
                                  imu_gyro_dps gyro_dps, float dt_s);

imu_drift_statistics imu_gyro_drift_statistics(const imu_gyro_drift *drift);

#ifdef __cplusplus
}
#endif

#endif /* MIDDLEWARE_IMU_DRIFT_H */
