/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_stationary_experiment.h
 * Do not edit locally; re-sync from the donor instead.
 */
#ifndef MIDDLEWARE_IMU_STATIONARY_EXPERIMENT_H
#define MIDDLEWARE_IMU_STATIONARY_EXPERIMENT_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_stationarity.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} imu_gyro_raw_counts;

typedef enum {
    IMU_EXPERIMENT_SETTLING = 0,
    IMU_EXPERIMENT_CALIBRATING,
    IMU_EXPERIMENT_HOLD_OUT,
    IMU_EXPERIMENT_COMPLETE
} imu_stationary_experiment_phase;

typedef struct {
    imu_stationarity_config stationarity;
    uint32_t raw_to_dps_numerator;
    uint32_t raw_to_dps_denominator;
    uint32_t sample_rate_hz;
    float settling_duration_s;
    float calibration_duration_s;
    float hold_out_duration_s;
} imu_stationary_experiment_config;

typedef struct {
    imu_gyro_raw_counts gyro_raw;
    imu_acceleration_g acceleration_g;
    float dt_s;
    uint32_t skipped_cycles;
    bool acceleration_valid;
} imu_stationary_experiment_input;

typedef struct {
    imu_stationary_experiment_config config;
    imu_stationary_experiment_phase phase;
    uint32_t settling_required_samples;
    uint32_t calibration_required_samples;
    uint32_t settling_accepted_samples;
    uint32_t calibration_accepted_samples;
    uint32_t calibration_reset_count;
    uint32_t hold_out_accepted_samples;
    int64_t raw_sum_x;
    int64_t raw_sum_y;
    int64_t raw_sum_z;
    double settling_elapsed_s;
    double calibration_elapsed_s;
    double hold_out_wall_duration_s;
    double hold_out_accepted_duration_s;
    double hold_out_unobserved_duration_s;
    double hold_out_nonstationary_duration_s;
    imu_gyro_dps frozen_bias_dps;
    bool bias_frozen;
    bool hold_out_valid;
    bool sample_stationary;
} imu_stationary_experiment;

void imu_stationary_experiment_init(
    imu_stationary_experiment *experiment,
    const imu_stationary_experiment_config *config);

bool imu_stationary_experiment_update(
    imu_stationary_experiment *experiment,
    const imu_stationary_experiment_input *input);

void imu_stationary_experiment_reject(imu_stationary_experiment *experiment,
                                      float dt_s);

#ifdef __cplusplus
}
#endif

#endif
