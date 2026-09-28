/* Ported verbatim from the seven_code donor repo.
 * Donor origin: apps/rm_c_blinky/src/imu_pipeline.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_pipeline.h"

static const imu_stationarity_config experiment_stationarity = {
    .max_gyro_magnitude_dps = 2.0f,
    .min_acceleration_magnitude_g = 0.95f,
    .max_acceleration_magnitude_g = 1.05f,
};

static const imu_temperature_gate_config temperature_gate_config = {
    .max_temperature_age_s = 2.5f,
    .max_temperature_slope_degc_per_s = 0.1f,
};

static const imu_zaru_config zaru_config = {
    .minimum_window_s = 3.0f,
    .minimum_samples = 3000u,
    .max_window_stddev_dps = 0.5f,
    .bias_update_gain = 0.3143f,
};

static imu_gyro_dps subtract_bias(imu_gyro_dps gyro_dps,
                                  imu_gyro_dps bias_dps) {
    return (imu_gyro_dps){
        .x_dps = gyro_dps.x_dps - bias_dps.x_dps,
        .y_dps = gyro_dps.y_dps - bias_dps.y_dps,
        .z_dps = gyro_dps.z_dps - bias_dps.z_dps,
    };
}

static imu_gyro_dps apply_static_yaw_deadband(imu_gyro_dps gyro_dps,
                                              bool stationary,
                                              bool enabled) {
    const float deadband_dps = 1.0f;
    if (enabled && stationary) {
        if (gyro_dps.x_dps > -deadband_dps && gyro_dps.x_dps < deadband_dps) {
            gyro_dps.x_dps = 0.0f;
        }
        if (gyro_dps.y_dps > -deadband_dps && gyro_dps.y_dps < deadband_dps) {
            gyro_dps.y_dps = 0.0f;
        }
        if (gyro_dps.z_dps > -deadband_dps && gyro_dps.z_dps < deadband_dps) {
            gyro_dps.z_dps = 0.0f;
        }
    }
    return gyro_dps;
}

static imu_gyro_dps input_gyro_dps(const imu_pipeline *pipeline,
                                   const imu_pipeline_input *input) {
    if (!input->gyro_raw_valid) {
        return input->gyro_dps;
    }
    const float scale =
        (float)pipeline->experiment.config.raw_to_dps_numerator /
        (float)pipeline->experiment.config.raw_to_dps_denominator;
    return (imu_gyro_dps){
        .x_dps = (float)input->gyro_raw.x * scale,
        .y_dps = (float)input->gyro_raw.y * scale,
        .z_dps = (float)input->gyro_raw.z * scale,
    };
}

static float phase_elapsed_s(const imu_stationary_experiment *experiment) {
    switch (experiment->phase) {
        case IMU_EXPERIMENT_SETTLING:
            return (float)experiment->settling_elapsed_s;
        case IMU_EXPERIMENT_CALIBRATING:
            return (float)experiment->calibration_elapsed_s;
        case IMU_EXPERIMENT_HOLD_OUT:
        case IMU_EXPERIMENT_COMPLETE:
            return (float)experiment->hold_out_wall_duration_s;
    }
    return 0.0f;
}

static float phase_required_s(const imu_stationary_experiment *experiment) {
    switch (experiment->phase) {
        case IMU_EXPERIMENT_SETTLING:
            return experiment->config.settling_duration_s;
        case IMU_EXPERIMENT_CALIBRATING:
            return experiment->config.calibration_duration_s;
        case IMU_EXPERIMENT_HOLD_OUT:
        case IMU_EXPERIMENT_COMPLETE:
            return experiment->config.hold_out_duration_s;
    }
    return 0.0f;
}

void imu_pipeline_init_with_experiment_config(
    imu_pipeline *pipeline,
    const imu_stationary_experiment_config *experiment_config) {
    const imu_attitude_config attitude_config = {
        .proportional_gain = 2.0f,
        .integral_gain = 0.1f,
        .min_acceleration_magnitude_g = 0.8f,
        .max_acceleration_magnitude_g = 1.2f,
        .max_acceleration_correction_gyro_dps = 100.0f,
        .max_acceleration_age_s = 0.002f,
    };
    *pipeline = (imu_pipeline){0};
    imu_stationary_experiment_init(&pipeline->experiment, experiment_config);
    imu_angular_acceleration_init(&pipeline->angular_acceleration, 0.25f);
    imu_attitude_init(&pipeline->attitude, &attitude_config);
    imu_zaru_init(&pipeline->zaru, &zaru_config);
    imu_pipeline_heading_init(&pipeline->heading);
    pipeline->zaru_enabled = true;
    imu_gyro_drift_init(&pipeline->hold_out_raw_drift,
                        &experiment_config->stationarity);
    imu_gyro_drift_init(&pipeline->hold_out_calibrated_drift,
                        &experiment_config->stationarity);
    imu_temperature_gate_init(&pipeline->temperature_gate,
                              &temperature_gate_config);
    pipeline->calibration_enabled = true;
}

void imu_pipeline_init(imu_pipeline *pipeline) {
    const imu_stationary_experiment_config config = {
        .stationarity = experiment_stationarity,
        .raw_to_dps_numerator = 2000u,
        .raw_to_dps_denominator = 32768u,
        .sample_rate_hz = IMU_PIPELINE_SAMPLE_RATE_HZ,
        .settling_duration_s = IMU_PIPELINE_SETTLING_DURATION_S,
        .calibration_duration_s = IMU_PIPELINE_CALIBRATION_DURATION_S,
        .hold_out_duration_s = IMU_PIPELINE_HOLD_OUT_DURATION_S,
    };
    imu_pipeline_init_with_experiment_config(pipeline, &config);
}

void imu_pipeline_set_calibration_enabled(imu_pipeline *pipeline,
                                          bool enabled) {
    if (pipeline->calibration_enabled == enabled) {
        return;
    }
    if (enabled) {
        const imu_stationary_experiment_config config =
            pipeline->experiment.config;
        imu_pipeline_init_with_experiment_config(pipeline, &config);
        return;
    }
    pipeline->calibration_enabled = false;
}

void imu_pipeline_set_zaru_enabled(imu_pipeline *pipeline, bool enabled) {
    pipeline->zaru_enabled = enabled;
}

static imu_pipeline_output pipeline_output(const imu_pipeline *pipeline,
                                           imu_pipeline_output output) {
    const imu_stationary_experiment *experiment = &pipeline->experiment;
    output.gyro_bias_dps = experiment->bias_frozen
                               ? experiment->frozen_bias_dps
                               : (imu_gyro_dps){0};
    output.zaru_bias_dps = pipeline->zaru.bias_dps;
    output.attitude_heading_drift_deg = pipeline->heading.drift_deg;
    output.hold_out_bias_dps = experiment->frozen_bias_dps;
    output.drift = imu_gyro_drift_statistics(&pipeline->hold_out_raw_drift);
    output.calibrated_drift =
        imu_gyro_drift_statistics(&pipeline->hold_out_calibrated_drift);
    output.experiment_phase = experiment->phase;
    output.settling_accepted_samples = experiment->settling_accepted_samples;
    output.calibration_accepted_samples = experiment->calibration_accepted_samples;
    output.calibration_reset_count = experiment->calibration_reset_count;
    output.hold_out_accepted_samples = experiment->hold_out_accepted_samples;
    output.experiment_phase_elapsed_s = phase_elapsed_s(experiment);
    output.experiment_phase_required_s = phase_required_s(experiment);
    output.hold_out_wall_duration_s = (float)experiment->hold_out_wall_duration_s;
    output.hold_out_accepted_duration_s =
        (float)experiment->hold_out_accepted_duration_s;
    output.hold_out_unobserved_duration_s =
        (float)experiment->hold_out_unobserved_duration_s;
    output.hold_out_nonstationary_duration_s =
        (float)experiment->hold_out_nonstationary_duration_s;
    output.temperature_degc = pipeline->temperature_gate.temperature_degc;
    output.temperature_slope_degc_per_s =
        pipeline->temperature_gate.slope_degc_per_s;
    output.calibration_complete = experiment->bias_frozen;
    output.bias_frozen = experiment->bias_frozen;
    output.hold_out_valid = experiment->hold_out_valid;
    output.temperature_valid =
        imu_temperature_gate_is_valid(&pipeline->temperature_gate);
    output.sample_stationary = experiment->sample_stationary;
    return output;
}

imu_pipeline_output imu_pipeline_advance_time(imu_pipeline *pipeline,
                                              float dt_s) {
    imu_temperature_gate_advance(&pipeline->temperature_gate, dt_s);
    if (pipeline->calibration_enabled) {
        imu_stationary_experiment_reject(&pipeline->experiment, dt_s);
    }
    return pipeline_output(pipeline, (imu_pipeline_output){.sample_dt_s = dt_s});
}

imu_pipeline_output imu_pipeline_update_timed(imu_pipeline *pipeline,
                                              const imu_pipeline_input *input) {
    imu_temperature_gate_advance(&pipeline->temperature_gate, input->dt_s);
    if (input->temperature_sampled) {
        imu_temperature_gate_record(&pipeline->temperature_gate,
                                    input->temperature_sample_valid,
                                    input->temperature_degc);
    }

    const imu_gyro_dps gyro_dps = input_gyro_dps(pipeline, input);
    const imu_stationary_experiment_phase phase_before =
        pipeline->experiment.phase;
    bool accepted = false;
    if (pipeline->calibration_enabled && input->gyro_raw_valid) {
        const imu_stationary_experiment_input experiment_input = {
            .gyro_raw = input->gyro_raw,
            .acceleration_g = input->acceleration_g,
            .dt_s = input->dt_s,
            .skipped_cycles = input->skipped_cycles,
            .acceleration_valid = input->acceleration_valid &&
                                  input->acceleration_age_s <= 0.02f,
        };
        accepted = imu_stationary_experiment_update(&pipeline->experiment,
                                                    &experiment_input);
    } else if (pipeline->calibration_enabled) {
        imu_stationary_experiment_reject(&pipeline->experiment, input->dt_s);
    }

    const imu_gyro_dps bias_dps = pipeline->experiment.bias_frozen
                                      ? pipeline->experiment.frozen_bias_dps
                                      : (imu_gyro_dps){0};
    const imu_gyro_dps calibrated_gyro_dps = subtract_bias(gyro_dps, bias_dps);
    if (pipeline->zaru_enabled && pipeline->experiment.bias_frozen &&
        !pipeline->zaru.bias_valid) {
        imu_zaru_set_bias(&pipeline->zaru, bias_dps);
    }
    if (pipeline->zaru_enabled && pipeline->experiment.bias_frozen &&
        phase_before == IMU_EXPERIMENT_HOLD_OUT) {
        const imu_zaru_input zaru_input = {
            .gyro_dps = gyro_dps,
            .dt_s = input->dt_s,
            .skipped_cycles = input->skipped_cycles,
            .sample_valid = input->gyro_raw_valid,
            .confirmed_stationary = pipeline->experiment.sample_stationary,
        };
        (void)imu_zaru_update(&pipeline->zaru, &zaru_input);
    }
    const imu_gyro_dps attitude_bias_dps = pipeline->zaru_enabled &&
                                                   pipeline->zaru.bias_valid
                                                ? pipeline->zaru.bias_dps
                                                : bias_dps;
    const imu_gyro_dps attitude_gyro_dps = apply_static_yaw_deadband(
        subtract_bias(gyro_dps, attitude_bias_dps),
        pipeline->experiment.sample_stationary, pipeline->zaru_enabled);
    const imu_angular_acceleration_dps2 angular_acceleration_dps2 =
        imu_angular_acceleration_update(&pipeline->angular_acceleration,
                                        calibrated_gyro_dps, input->dt_s);
    const imu_attitude_output attitude = imu_attitude_update_timed(
        &pipeline->attitude, attitude_gyro_dps,
        (imu_attitude_acceleration){
            .acceleration_g = input->acceleration_g,
            .age_s = input->acceleration_age_s,
            .valid = input->acceleration_valid,
            .fresh = input->acceleration_fresh,
            .allow_integral_feedback = pipeline->experiment.sample_stationary,
        },
        input->dt_s);

    imu_pipeline_heading_update(
        &pipeline->heading,
        (imu_pipeline_heading_input){
            .phase = phase_before,
            .attitude = attitude,
            .dt_s = input->dt_s,
        });

#if IMU_PIPELINE_ENABLE_DRIFT_DIAGNOSTICS
    /* Diagnostics only: Welford drift statistics use double accumulators, which
     * the Cortex-M4F emulates in software (~58 __aeabi_d* calls per cycle here).
     * None of the outputs the gimbal actually consumes depend on this, so it is
     * compiled out by default to protect the 1 kHz TIM7 budget. Define
     * IMU_PIPELINE_ENABLE_DRIFT_DIAGNOSTICS=1 to re-enable it for bench runs. */
    if (phase_before == IMU_EXPERIMENT_HOLD_OUT && accepted) {
        const float integration_dt_s = input->skipped_cycles == 0u
                                           ? input->dt_s
                                           : IMU_PIPELINE_SAMPLE_PERIOD_S;
        (void)imu_gyro_drift_update_ungated(&pipeline->hold_out_raw_drift,
                                           gyro_dps, integration_dt_s);
        (void)imu_gyro_drift_update_ungated(
            &pipeline->hold_out_calibrated_drift, calibrated_gyro_dps,
            integration_dt_s);
    }
#else
    (void)accepted;
#endif

    return pipeline_output(
        pipeline, (imu_pipeline_output){
                      .angular_velocity_dps = calibrated_gyro_dps,
                      .attitude_angular_velocity_dps = attitude_gyro_dps,
                      .acceleration_g = input->acceleration_g,
                      .angular_acceleration_dps2 = angular_acceleration_dps2,
                      .attitude = attitude,
                      .sample_dt_s = input->dt_s,
                      .skipped_cycles = input->skipped_cycles,
                  });
}
