/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_stationarity.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_stationarity.h"

#include <math.h>

bool imu_stationarity_is_stationary(const imu_stationarity_config *config,
                                    imu_gyro_dps gyro_dps,
                                    imu_acceleration_g acceleration_g) {
    const float gyro_squared_dps2 =
        gyro_dps.x_dps * gyro_dps.x_dps +
        gyro_dps.y_dps * gyro_dps.y_dps +
        gyro_dps.z_dps * gyro_dps.z_dps;
    const float max_gyro_squared_dps2 =
        config->max_gyro_magnitude_dps * config->max_gyro_magnitude_dps;
    const float acceleration_squared_g2 =
        acceleration_g.x_g * acceleration_g.x_g +
        acceleration_g.y_g * acceleration_g.y_g +
        acceleration_g.z_g * acceleration_g.z_g;
    const float acceleration_magnitude_g = sqrtf(acceleration_squared_g2);

    return gyro_squared_dps2 <= max_gyro_squared_dps2 &&
           acceleration_magnitude_g >=
               config->min_acceleration_magnitude_g &&
            acceleration_magnitude_g <=
                config->max_acceleration_magnitude_g;
}
