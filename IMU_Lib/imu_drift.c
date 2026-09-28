/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_drift.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_drift.h"

#include <math.h>

void imu_gyro_drift_init(imu_gyro_drift *drift,
                         const imu_stationarity_config *stationarity) {
    *drift = (imu_gyro_drift){
        .stationarity = *stationarity,
    };
}

bool imu_gyro_drift_update_ungated(imu_gyro_drift *drift,
                                  imu_gyro_dps gyro_dps, float dt_s) {
    if (dt_s <= 0.0f) {
        return false;
    }

    const uint32_t sample_number = drift->sample_count + 1u;
    const double weight = 1.0 / (double)sample_number;
    const double previous_mean_x_dps = drift->mean_x_dps;
    const double previous_mean_y_dps = drift->mean_y_dps;
    const double previous_mean_z_dps = drift->mean_z_dps;
    drift->mean_x_dps +=
        ((double)gyro_dps.x_dps - drift->mean_x_dps) * weight;
    drift->mean_y_dps +=
        ((double)gyro_dps.y_dps - drift->mean_y_dps) * weight;
    drift->mean_z_dps +=
        ((double)gyro_dps.z_dps - drift->mean_z_dps) * weight;
    /* Welford variance accumulator: deviation from the old mean times
     * deviation from the new mean is always non-negative, so the squared
     * sum cannot cancel and the RMS noise floor stays exact for a constant
     * signal. */
    drift->sum_squared_deviation_x_dps2 +=
        ((double)gyro_dps.x_dps - previous_mean_x_dps) *
        ((double)gyro_dps.x_dps - drift->mean_x_dps);
    drift->sum_squared_deviation_y_dps2 +=
        ((double)gyro_dps.y_dps - previous_mean_y_dps) *
        ((double)gyro_dps.y_dps - drift->mean_y_dps);
    drift->sum_squared_deviation_z_dps2 +=
        ((double)gyro_dps.z_dps - previous_mean_z_dps) *
        ((double)gyro_dps.z_dps - drift->mean_z_dps);
    drift->sample_count = sample_number;
    drift->duration_s += (double)dt_s;
    drift->yaw_drift_deg += (double)gyro_dps.z_dps * (double)dt_s;
    return true;
}

bool imu_gyro_drift_update(imu_gyro_drift *drift,
                           imu_gyro_dps gyro_dps,
                           imu_acceleration_g acceleration_g,
                           float dt_s) {
    if (!imu_stationarity_is_stationary(&drift->stationarity, gyro_dps,
                                        acceleration_g)) {
        return false;
    }
    return imu_gyro_drift_update_ungated(drift, gyro_dps, dt_s);
}

imu_drift_statistics imu_gyro_drift_statistics(const imu_gyro_drift *drift) {
    if (drift->sample_count == 0u) {
        return (imu_drift_statistics){0};
    }

    const double inverse_sample_count =
        1.0 / (double)drift->sample_count;
    const imu_gyro_dps rms_dps = {
        (float)sqrt(drift->sum_squared_deviation_x_dps2 * inverse_sample_count),
        (float)sqrt(drift->sum_squared_deviation_y_dps2 * inverse_sample_count),
        (float)sqrt(drift->sum_squared_deviation_z_dps2 * inverse_sample_count),
    };
    float yaw_drift_deg_per_min = 0.0f;
    if (drift->duration_s > 0.0) {
        yaw_drift_deg_per_min = (float)(
            drift->yaw_drift_deg * 60.0 / drift->duration_s);
    }
    return (imu_drift_statistics){
        .sample_count = drift->sample_count,
        .observed_duration_s = (float)drift->duration_s,
        .mean_dps = {
            .x_dps = (float)drift->mean_x_dps,
            .y_dps = (float)drift->mean_y_dps,
            .z_dps = (float)drift->mean_z_dps,
        },
        .rms_dps = rms_dps,
        .yaw_drift_deg = (float)drift->yaw_drift_deg,
        .yaw_drift_deg_per_min = yaw_drift_deg_per_min,
    };
}
