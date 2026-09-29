/* Ported verbatim from the seven_code donor repo.
 * Donor origin: apps/rm_c_blinky/src/imu_pipeline.h
 * Do not edit locally; re-sync from the donor instead.
 */
#ifndef APPS_RM_C_BLINKY_IMU_PIPELINE_H
#define APPS_RM_C_BLINKY_IMU_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_angular_acceleration.h"
#include "imu_attitude.h"
#include "imu_drift.h"
#include "imu_stationary_experiment.h"
#include "imu_temperature_gate.h"
#include "imu_zaru.h"
#include "imu_pipeline_heading.h"

#define IMU_PIPELINE_SAMPLE_PERIOD_S 0.001f
#define IMU_PIPELINE_SAMPLE_RATE_HZ 1000u

/* Drift statistics are bench diagnostics built on double-precision Welford
 * accumulators, which are software-emulated on the Cortex-M4F. They feed no
 * gimbal control output, so they stay off inside the 1 kHz TIM7 ISR. */
#ifndef IMU_PIPELINE_ENABLE_DRIFT_DIAGNOSTICS
#define IMU_PIPELINE_ENABLE_DRIFT_DIAGNOSTICS 0
#endif
#define IMU_PIPELINE_SETTLING_DURATION_S 0.0f
#define IMU_PIPELINE_CALIBRATION_DURATION_S 2.0f
#define IMU_PIPELINE_HOLD_OUT_DURATION_S 600.0f

typedef struct {
    imu_gyro_raw_counts gyro_raw;
    imu_gyro_dps gyro_dps;
    imu_acceleration_g acceleration_g;
    float dt_s;
    uint32_t skipped_cycles;
    bool gyro_raw_valid;
    bool gyro_clipped;
    bool acceleration_valid;
    bool acceleration_fresh;
    bool acceleration_sampled;
    float acceleration_age_s;
    float acceleration_sample_dt_s;
    bool temperature_sampled;
    bool temperature_sample_valid;
    float temperature_degc;
} imu_pipeline_input;

typedef struct {
    imu_gyro_dps gyro_bias_dps;
    imu_gyro_dps zaru_bias_dps;
    imu_gyro_dps hold_out_bias_dps;
    imu_gyro_dps angular_velocity_dps;
    imu_gyro_dps attitude_angular_velocity_dps;
    imu_acceleration_g acceleration_g;
    imu_angular_acceleration_dps2 angular_acceleration_dps2;
    imu_attitude_output attitude;
    float attitude_heading_drift_deg;
    uint32_t gyro_clip_count;
    bool heading_valid;
    imu_drift_statistics drift;
    imu_drift_statistics calibrated_drift;
    imu_stationary_experiment_phase experiment_phase;
    uint32_t settling_accepted_samples;
    uint32_t calibration_accepted_samples;
    uint32_t calibration_reset_count;
    uint32_t hold_out_accepted_samples;
    uint32_t skipped_cycles;
    float experiment_phase_elapsed_s;
    float experiment_phase_required_s;
    float hold_out_wall_duration_s;
    float hold_out_accepted_duration_s;
    float hold_out_unobserved_duration_s;
    float hold_out_nonstationary_duration_s;
    float sample_dt_s;
    float temperature_degc;
    float temperature_slope_degc_per_s;
    bool calibration_complete;
    bool bias_frozen;
    bool hold_out_valid;
    bool temperature_valid;
    bool sample_stationary;
} imu_pipeline_output;

typedef struct {
    imu_stationary_experiment experiment;
    imu_angular_acceleration_filter angular_acceleration;
    imu_attitude attitude;
    imu_zaru_observer zaru;
    imu_pipeline_heading_metrics heading;
    bool zaru_enabled;
    imu_gyro_drift hold_out_raw_drift;
    imu_gyro_drift hold_out_calibrated_drift;
    imu_temperature_gate temperature_gate;
    bool calibration_enabled;
    uint32_t gyro_clip_count;
    bool heading_valid;
} imu_pipeline;

void imu_pipeline_init(imu_pipeline *pipeline);

void imu_pipeline_init_with_experiment_config(
    imu_pipeline *pipeline,
    const imu_stationary_experiment_config *experiment_config);

void imu_pipeline_set_calibration_enabled(imu_pipeline *pipeline, bool enabled);

void imu_pipeline_set_zaru_enabled(imu_pipeline *pipeline, bool enabled);

imu_pipeline_output imu_pipeline_update_timed(imu_pipeline *pipeline,
                                              const imu_pipeline_input *input);

imu_pipeline_output imu_pipeline_advance_time(imu_pipeline *pipeline,
                                              float dt_s);

#endif
