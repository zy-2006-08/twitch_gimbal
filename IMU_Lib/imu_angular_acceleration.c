/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_angular_acceleration.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_angular_acceleration.h"

void imu_angular_acceleration_init(imu_angular_acceleration_filter *filter,
                                   float smoothing_alpha) {
    *filter = (imu_angular_acceleration_filter){
        .smoothing_alpha = smoothing_alpha,
    };
}

imu_angular_acceleration_dps2 imu_angular_acceleration_update(
    imu_angular_acceleration_filter *filter,
    imu_gyro_dps gyro_dps,
    float dt_s) {
    if (dt_s <= 0.0f) {
        filter->has_previous_rate = false;
        return filter->acceleration_dps2;
    }

    if (!filter->has_previous_rate) {
        filter->previous_rate_dps = gyro_dps;
        filter->has_previous_rate = true;
        return filter->acceleration_dps2;
    }

    float alpha = filter->smoothing_alpha;
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    const float raw_x_dps2 =
        (gyro_dps.x_dps - filter->previous_rate_dps.x_dps) / dt_s;
    const float raw_y_dps2 =
        (gyro_dps.y_dps - filter->previous_rate_dps.y_dps) / dt_s;
    const float raw_z_dps2 =
        (gyro_dps.z_dps - filter->previous_rate_dps.z_dps) / dt_s;
    filter->acceleration_dps2.x_dps2 +=
        alpha * (raw_x_dps2 - filter->acceleration_dps2.x_dps2);
    filter->acceleration_dps2.y_dps2 +=
        alpha * (raw_y_dps2 - filter->acceleration_dps2.y_dps2);
    filter->acceleration_dps2.z_dps2 +=
        alpha * (raw_z_dps2 - filter->acceleration_dps2.z_dps2);
    filter->previous_rate_dps = gyro_dps;
    return filter->acceleration_dps2;
}
