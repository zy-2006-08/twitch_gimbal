/* C facade over the ported donor IMU stack.
 *
 * Purpose: give the C++ BMI088 class a single, small, C-linkage surface to call,
 * so the donor sources underneath stay byte-identical to their origin and future
 * re-syncs are trivial. All orchestration that gyro_task.c did under FreeRTOS in
 * the donor repo lives here instead, minus the RTOS and minus the heater.
 *
 * Donor reference for the call sequencing, sample-rate dividers and input
 * construction (read, not copied, because it is FreeRTOS-coupled):
 *   D:\RM27_zimiao\seven_code\apps\rm_c_blinky\src\gyro_task.c
 *
 * Timing model: this project drives the pipeline from the TIM7 1 kHz ISR, which
 * is a stable hardware timer, so dt is the fixed IMU_FACADE_DT_S below and
 * skipped_cycles is always zero. There is exactly ONE definition of the sample
 * period here; nothing else in the port hardcodes 0.001f.
 */
#ifndef IMU_LIB_IMU_FACADE_H
#define IMU_LIB_IMU_FACADE_H

#include <stdbool.h>
#include <stdint.h>

/* These carry their own extern "C" guards. */
#include "bmi088_accel.h"
#include "bmi088_gyro.h"
#include "bmi088_spi_adapter.h"

/* imu_pipeline.h and imu_pipeline_heading.h are the only ported headers that
 * lack extern "C" guards, so they must be wrapped when pulled in from C++.
 * Every middleware header they include does carry its own guard, and the
 * standard headers were already included above, so no system declaration ends
 * up inside this linkage block. */
#ifdef __cplusplus
extern "C" {
#endif
#include "imu_pipeline.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* The single source of truth for the sample period. TIM7 runs at 1 kHz. */
#define IMU_FACADE_DT_S IMU_PIPELINE_SAMPLE_PERIOD_S

/* Cycle dividers, mirroring the donor: gyro every cycle, accel every 10th
 * (100 Hz, matching the accelerometer's configured ODR), temperature every
 * 1000th (~1 Hz). */
#define IMU_FACADE_ACCEL_DIVIDER 10u
#define IMU_FACADE_TEMPERATURE_DIVIDER 1000u

typedef enum {
    IMU_FACADE_OK = 0,
    IMU_FACADE_ERR_ACCEL,
    IMU_FACADE_ERR_GYRO
} imu_facade_status;

/* Everything the port needs, held by value so the owning C++ object carries its
 * own state and no file-scope globals are involved. */
typedef struct {
    /* Hardware binding. */
    bmi088_spi_adapter accel_adapter;
    bmi088_spi_adapter gyro_adapter;
    bmi088_accel accel_device;
    bmi088_gyro gyro_device;

    /* Pure algorithm state. */
    imu_pipeline pipeline;

    /* Scheduling state for the dividers. */
    uint32_t sequence;
    bmi088_accel_sample accel_sample;
    float acceleration_age_s;
    bool acceleration_valid;

    /* ---- Outputs, refreshed by imu_facade_tick ---- */

    /* Unit quaternion, scalar-first. */
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    /* Euler angles in DEGREES, wrapped, ZYX convention. Identical formulas to
     * the ones the old implementation used, so these drop straight in. */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    /* Bias-corrected body angular rates straight off the gyro, in DEG/S.
     * These are the un-deadbanded control-grade rates, not the deadbanded
     * rates that drive the attitude filter. */
    float rate_x_dps;
    float rate_y_dps;
    float rate_z_dps;

    /* Observability / health. */
    float temperature_degc;
    bool calibration_complete;
    bool sample_stationary;
    bool gyro_read_ok;
    uint32_t gyro_error_count;
    uint32_t accel_error_count;
    uint32_t calibration_reset_count;
    imu_stationary_experiment_phase experiment_phase;
    /* Gyro hit the int16 rail at least once; yaw can no longer be trusted. */
    uint32_t gyro_clip_count;
    bool heading_valid;
    /* |pitch| >= 85 deg: ZYX yaw/roll are not independently defined. */
    bool euler_zyx_singular;
} imu_facade;

/* Brings up both sensors and resets the pipeline.
 *
 * Returns IMU_FACADE_OK only when the accelerometer AND the gyroscope were both
 * identified and configured successfully, which is what the caller maps onto
 * BMI088_OK. The chip-select arguments are GPIO_TypeDef* / pin pairs. */
imu_facade_status imu_facade_init(imu_facade *facade, void *hspi,
                                 void *accel_cs_port, uint16_t accel_cs_pin,
                                 void *gyro_cs_port, uint16_t gyro_cs_pin);

/* One 1 kHz cycle: reads the gyro every call, the accelerometer and temperature
 * on their dividers, advances the pipeline, and refreshes the output fields.
 * Safe to call from an ISR; all SPI access is blocking with a short timeout. */
void imu_facade_tick(imu_facade *facade);

#ifdef __cplusplus
}
#endif

#endif /* IMU_LIB_IMU_FACADE_H */
