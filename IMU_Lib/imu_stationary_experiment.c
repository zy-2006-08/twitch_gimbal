/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_stationary_experiment.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_stationary_experiment.h"

#include <math.h>

static imu_gyro_dps raw_to_dps(
    const imu_stationary_experiment_config *config,
    imu_gyro_raw_counts raw) {
    const double scale = (double)config->raw_to_dps_numerator /
                         (double)config->raw_to_dps_denominator;
    return (imu_gyro_dps){
        .x_dps = (float)((double)raw.x * scale),
        .y_dps = (float)((double)raw.y * scale),
        .z_dps = (float)((double)raw.z * scale),
    };
}

static bool duration_complete(double elapsed_s, float required_s,
                              uint32_t sample_rate_hz) {
    const double half_sample_s = 0.5 / (double)sample_rate_hz;
    return elapsed_s + half_sample_s >= (double)required_s;
}

static const double hold_out_nonstationary_tolerance_s = 0.25;

static bool acceleration_is_plausible(
    const imu_stationary_experiment *experiment,
    imu_acceleration_g acceleration_g) {
    const float squared = acceleration_g.x_g * acceleration_g.x_g +
                          acceleration_g.y_g * acceleration_g.y_g +
                          acceleration_g.z_g * acceleration_g.z_g;
    const float magnitude = sqrtf(squared);
    return magnitude >= experiment->config.stationarity.min_acceleration_magnitude_g &&
           magnitude <= experiment->config.stationarity.max_acceleration_magnitude_g;
}

static void reset_calibration(imu_stationary_experiment *experiment) {
    experiment->calibration_accepted_samples = 0u;
    experiment->calibration_elapsed_s = 0.0;
    experiment->raw_sum_x = 0;
    experiment->raw_sum_y = 0;
    experiment->raw_sum_z = 0;
    experiment->calibration_reset_count++;
}

static void freeze_bias(imu_stationary_experiment *experiment) {
    const double scale =
        (double)experiment->config.raw_to_dps_numerator /
        (double)experiment->config.raw_to_dps_denominator;
    const double sample_count =
        (double)experiment->calibration_accepted_samples;
    experiment->frozen_bias_dps = (imu_gyro_dps){
        .x_dps =
            (float)(((double)experiment->raw_sum_x / sample_count) * scale),
        .y_dps =
            (float)(((double)experiment->raw_sum_y / sample_count) * scale),
        .z_dps =
            (float)(((double)experiment->raw_sum_z / sample_count) * scale),
    };
    experiment->bias_frozen = true;
    experiment->hold_out_valid = true;
    experiment->phase = IMU_EXPERIMENT_HOLD_OUT;
}

void imu_stationary_experiment_init(
    imu_stationary_experiment *experiment,
    const imu_stationary_experiment_config *config) {
    *experiment = (imu_stationary_experiment){
        .config = *config,
        .phase = IMU_EXPERIMENT_SETTLING,
    };
    experiment->settling_required_samples = (uint32_t)(
        config->settling_duration_s * (float)config->sample_rate_hz + 0.5f);
    experiment->calibration_required_samples = (uint32_t)(
        config->calibration_duration_s * (float)config->sample_rate_hz + 0.5f);
    if (experiment->settling_required_samples == 0u) {
        experiment->phase = IMU_EXPERIMENT_CALIBRATING;
    }
    if (experiment->calibration_required_samples == 0u) {
        experiment->phase = IMU_EXPERIMENT_HOLD_OUT;
        experiment->bias_frozen = true;
        experiment->hold_out_valid = true;
    }
}

void imu_stationary_experiment_reject(imu_stationary_experiment *experiment,
                                      float dt_s) {
    switch (experiment->phase) {
        case IMU_EXPERIMENT_SETTLING:
            experiment->settling_accepted_samples = 0u;
            experiment->settling_elapsed_s = 0.0;
            break;
        case IMU_EXPERIMENT_CALIBRATING:
            reset_calibration(experiment);
            break;
        case IMU_EXPERIMENT_HOLD_OUT:
            experiment->hold_out_valid = false;
            if (dt_s > 0.0f) {
                experiment->hold_out_wall_duration_s += (double)dt_s;
                experiment->hold_out_unobserved_duration_s += (double)dt_s;
            }
            if (experiment->hold_out_wall_duration_s >=
                (double)experiment->config.hold_out_duration_s) {
                experiment->phase = IMU_EXPERIMENT_COMPLETE;
            }
            break;
        case IMU_EXPERIMENT_COMPLETE:
            break;
    }
}

bool imu_stationary_experiment_update(
    imu_stationary_experiment *experiment,
    const imu_stationary_experiment_input *input) {
    experiment->sample_stationary = false;
    if (input->dt_s <= 0.0f) {
        imu_stationary_experiment_reject(experiment, input->dt_s);
        return false;
    }

    const imu_gyro_dps gyro_dps = raw_to_dps(&experiment->config,
                                              input->gyro_raw);
    const bool stationary = input->acceleration_valid &&
        imu_stationarity_is_stationary(&experiment->config.stationarity,
                                       gyro_dps, input->acceleration_g);
    experiment->sample_stationary = stationary;
    if (experiment->phase == IMU_EXPERIMENT_HOLD_OUT) {
        const double observed_s = input->skipped_cycles == 0u
                                      ? (double)input->dt_s
                                      : 1.0 / (double)experiment->config.sample_rate_hz;
        experiment->hold_out_wall_duration_s += (double)input->dt_s;
        if (input->skipped_cycles > 0u) {
            experiment->hold_out_unobserved_duration_s +=
                (double)input->dt_s > observed_s
                    ? (double)input->dt_s - observed_s
                    : 0.0;
            experiment->hold_out_valid = false;
        }
        if (stationary) {
            experiment->hold_out_accepted_duration_s += observed_s;
            experiment->hold_out_accepted_samples++;
            experiment->hold_out_nonstationary_duration_s = 0.0;
        } else {
            if (!input->acceleration_valid ||
                !acceleration_is_plausible(experiment, input->acceleration_g)) {
                experiment->hold_out_valid = false;
            }
            experiment->hold_out_nonstationary_duration_s +=
                (double)input->dt_s;
            if (experiment->hold_out_nonstationary_duration_s >=
                hold_out_nonstationary_tolerance_s) {
                experiment->hold_out_valid = false;
            }
        }
        if (duration_complete(experiment->hold_out_wall_duration_s,
                              experiment->config.hold_out_duration_s,
                              experiment->config.sample_rate_hz)) {
            experiment->phase = IMU_EXPERIMENT_COMPLETE;
        }
        return true;
    }

    if (!stationary || input->skipped_cycles > 0u) {
        imu_stationary_experiment_reject(experiment, input->dt_s);
        return false;
    }

    switch (experiment->phase) {
        case IMU_EXPERIMENT_SETTLING:
            experiment->settling_accepted_samples++;
            experiment->settling_elapsed_s += (double)input->dt_s;
            if (experiment->settling_accepted_samples >=
                    experiment->settling_required_samples &&
                duration_complete(experiment->settling_elapsed_s,
                                  experiment->config.settling_duration_s,
                                  experiment->config.sample_rate_hz)) {
                experiment->phase = IMU_EXPERIMENT_CALIBRATING;
            }
            return true;
        case IMU_EXPERIMENT_CALIBRATING:
            experiment->raw_sum_x += (int64_t)input->gyro_raw.x;
            experiment->raw_sum_y += (int64_t)input->gyro_raw.y;
            experiment->raw_sum_z += (int64_t)input->gyro_raw.z;
            experiment->calibration_accepted_samples++;
            experiment->calibration_elapsed_s += (double)input->dt_s;
            if (experiment->calibration_accepted_samples >=
                    experiment->calibration_required_samples &&
                duration_complete(experiment->calibration_elapsed_s,
                                  experiment->config.calibration_duration_s,
                                  experiment->config.sample_rate_hz)) {
                freeze_bias(experiment);
            }
            return true;
        case IMU_EXPERIMENT_HOLD_OUT:
            return false;
        case IMU_EXPERIMENT_COMPLETE:
            return false;
    }
    return false;
}
