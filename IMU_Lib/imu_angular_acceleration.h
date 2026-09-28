/* Ported verbatim from the seven_code donor repo.
 * Donor origin: middleware/imu/imu_angular_acceleration.h
 * Do not edit locally; re-sync from the donor instead.
 */
/* Fixed-dt angular acceleration from sampled gyro rates. */
#ifndef MIDDLEWARE_IMU_ANGULAR_ACCELERATION_H
#define MIDDLEWARE_IMU_ANGULAR_ACCELERATION_H

#include <stdbool.h>

#include "imu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float smoothing_alpha;
    bool has_previous_rate;
    imu_gyro_dps previous_rate_dps;
    imu_angular_acceleration_dps2 acceleration_dps2;
} imu_angular_acceleration_filter;

/* alpha is the first-order low-pass blend in [0, 1]. alpha=1 disables
 * smoothing. The caller supplies every sample interval to update(). */
void imu_angular_acceleration_init(imu_angular_acceleration_filter *filter,
                                   float smoothing_alpha);

/* Returns filtered angular acceleration in dps2. The first sample returns
 * zero acceleration. A non-positive dt_s clears temporal continuity. */
imu_angular_acceleration_dps2 imu_angular_acceleration_update(
    imu_angular_acceleration_filter *filter,
    imu_gyro_dps gyro_dps,
    float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* MIDDLEWARE_IMU_ANGULAR_ACCELERATION_H */
