/* Ported verbatim from the seven_code donor repo.
 * Donor origin: apps/rm_c_blinky/src/imu_pipeline_heading.h
 * Do not edit locally; re-sync from the donor instead.
 */
#ifndef APPS_RM_C_BLINKY_IMU_PIPELINE_HEADING_H
#define APPS_RM_C_BLINKY_IMU_PIPELINE_HEADING_H

#include <stdbool.h>

#include "imu_attitude.h"
#include "imu_stationary_experiment.h"

typedef struct {
    float drift_deg;
    float previous_yaw_deg;
    bool initialized;
} imu_pipeline_heading_metrics;

typedef struct {
    imu_stationary_experiment_phase phase;
    imu_attitude_output attitude;
    float dt_s;
} imu_pipeline_heading_input;

void imu_pipeline_heading_init(imu_pipeline_heading_metrics *metrics);

void imu_pipeline_heading_update(imu_pipeline_heading_metrics *metrics,
                                 imu_pipeline_heading_input input);

#endif
