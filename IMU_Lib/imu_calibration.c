/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_calibration.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_calibration.h"

void imu_gyro_calibration_init(imu_gyro_calibration *calibration,
                               const imu_stationarity_config *stationarity,
                               uint32_t required_samples) {
    *calibration = (imu_gyro_calibration){
        .stationarity = *stationarity,
        .required_samples = required_samples,
        .state = required_samples == 0u ? IMU_CALIBRATION_COMPLETE
                                        : IMU_CALIBRATION_COLLECTING,
    };
}

bool imu_gyro_calibration_update(imu_gyro_calibration *calibration,
                                 imu_gyro_dps gyro_dps,
                                 imu_acceleration_g acceleration_g) {
    if (calibration->state == IMU_CALIBRATION_COMPLETE ||
        !imu_stationarity_is_stationary(&calibration->stationarity, gyro_dps,
                                        acceleration_g)) {
        return false;
    }

    const uint32_t sample_number = calibration->accepted_samples + 1u;
    const float weight = 1.0f / (float)sample_number;
    calibration->bias_dps.x_dps +=
        (gyro_dps.x_dps - calibration->bias_dps.x_dps) * weight;
    calibration->bias_dps.y_dps +=
        (gyro_dps.y_dps - calibration->bias_dps.y_dps) * weight;
    calibration->bias_dps.z_dps +=
        (gyro_dps.z_dps - calibration->bias_dps.z_dps) * weight;
    calibration->accepted_samples = sample_number;
    if (sample_number >= calibration->required_samples) {
        calibration->state = IMU_CALIBRATION_COMPLETE;
    }
    return true;
}
