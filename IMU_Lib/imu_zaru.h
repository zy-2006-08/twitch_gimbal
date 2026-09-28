/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_zaru.h
 * Do not edit locally; re-sync from the donor instead.
 */
#ifndef MIDDLEWARE_IMU_ZARU_H
#define MIDDLEWARE_IMU_ZARU_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float minimum_window_s;
    uint32_t minimum_samples;
    float max_window_stddev_dps;
    float bias_update_gain;
} imu_zaru_config;

typedef enum {
    IMU_ZARU_WINDOW_IN_PROGRESS = 0,
    IMU_ZARU_WINDOW_ACCEPTED,
    IMU_ZARU_WINDOW_REJECTED,
} imu_zaru_update_result;

typedef struct {
    imu_gyro_dps gyro_dps;
    float dt_s;
    uint32_t skipped_cycles;
    bool sample_valid;
    bool confirmed_stationary;
} imu_zaru_input;

typedef struct {
    imu_zaru_config config;
    imu_gyro_dps bias_dps;
    imu_gyro_dps window_mean_dps;
    imu_gyro_dps window_m2_dps2;
    float window_duration_s;
    uint32_t window_sample_count;
    imu_gyro_dps last_window_bias_dps;
    float last_window_stddev_dps;
    float last_window_duration_s;
    uint32_t last_window_sample_count;
    double cumulative_sum_x_dps_s;
    double cumulative_sum_y_dps_s;
    double cumulative_sum_z_dps_s;
    uint32_t cumulative_sample_count;
    uint32_t accepted_window_count;
    uint32_t rejected_window_count;
    bool bias_valid;
} imu_zaru_observer;

void imu_zaru_init(imu_zaru_observer *observer,
                   const imu_zaru_config *config);

void imu_zaru_set_bias(imu_zaru_observer *observer, imu_gyro_dps bias_dps);

imu_zaru_update_result imu_zaru_update(imu_zaru_observer *observer,
                                       const imu_zaru_input *input);

#ifdef __cplusplus
}
#endif

#endif
