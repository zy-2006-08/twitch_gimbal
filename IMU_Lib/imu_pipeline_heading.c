/* Ported verbatim from the seven_code donor repo.
 * Donor origin: apps/rm_c_blinky/src/imu_pipeline_heading.c
 * Do not edit locally; re-sync from the donor instead.
 */
#include "imu_pipeline_heading.h"

static float heading_delta_deg(float current_deg, float previous_deg) {
    float delta_deg = current_deg - previous_deg;
    if (delta_deg > 180.0f) {
        delta_deg -= 360.0f;
    } else if (delta_deg < -180.0f) {
        delta_deg += 360.0f;
    }
    return delta_deg;
}

void imu_pipeline_heading_init(imu_pipeline_heading_metrics *metrics) {
    *metrics = (imu_pipeline_heading_metrics){.initialized = true};
}

void imu_pipeline_heading_update(imu_pipeline_heading_metrics *metrics,
                                 imu_pipeline_heading_input input) {
    const float yaw_deg = input.attitude.euler_zyx_deg.yaw_deg;
    if (!metrics->initialized) {
        metrics->previous_yaw_deg = yaw_deg;
        metrics->initialized = true;
        return;
    }

    const float delta_deg = heading_delta_deg(yaw_deg, metrics->previous_yaw_deg);
    if (input.phase == IMU_EXPERIMENT_HOLD_OUT && input.dt_s > 0.0f) {
        metrics->drift_deg += delta_deg;
    }
    metrics->previous_yaw_deg = yaw_deg;
}
