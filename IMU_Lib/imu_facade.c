/* C facade implementation. See imu_facade.h for the contract.
 *
 * Call sequencing mirrors the donor's gyro_task.c loop body, with the FreeRTOS
 * tick arithmetic replaced by the fixed TIM7 period and the heater removed.
 */
#include "imu_facade.h"

imu_facade_status imu_facade_init(imu_facade *facade, void *hspi,
                                 void *accel_cs_port, uint16_t accel_cs_pin,
                                 void *gyro_cs_port, uint16_t gyro_cs_pin)
{
    *facade = (imu_facade){0};

    facade->accel_adapter.hspi = hspi;
    facade->accel_adapter.cs_port = accel_cs_port;
    facade->accel_adapter.cs_pin = accel_cs_pin;
    facade->gyro_adapter.hspi = hspi;
    facade->gyro_adapter.cs_port = gyro_cs_port;
    facade->gyro_adapter.cs_pin = gyro_cs_pin;

    /* Identity quaternion until the first update, so a consumer reading the
     * outputs before the first tick sees a valid unit quaternion. */
    facade->q_w = 1.0f;

    imu_pipeline_init(&facade->pipeline);

    const bmi088_accel_bus accel_bus =
        bmi088_spi_adapter_accel_bus(&facade->accel_adapter);
    const bmi088_gyro_bus gyro_bus =
        bmi088_spi_adapter_gyro_bus(&facade->gyro_adapter);

    /* Both sensors must come up. The accelerometer is brought up first because
     * its soft reset drops the part back to I2C mode and the driver's own init
     * handles the SPI re-wake; doing it first keeps the gyro's 80 ms reset
     * settle from overlapping that sequence. */
    const bmi088_accel_status accel_status =
        bmi088_accel_init(&facade->accel_device, &accel_bus);
    if (accel_status != BMI088_ACCEL_OK) {
        return IMU_FACADE_ERR_ACCEL;
    }

    const bmi088_gyro_status gyro_status =
        bmi088_gyro_init(&facade->gyro_device, &gyro_bus);
    if (gyro_status != BMI088_GYRO_OK) {
        return IMU_FACADE_ERR_GYRO;
    }

    return IMU_FACADE_OK;
}

void imu_facade_tick(imu_facade *facade)
{
    const float dt_s = IMU_FACADE_DT_S;

    bmi088_gyro_sample gyro_sample = {0};
    const bmi088_gyro_status gyro_status =
        bmi088_gyro_read(&facade->gyro_device, &gyro_sample);

    if (gyro_status != BMI088_GYRO_OK) {
        /* No sample this cycle, but time still passed: the pipeline must see the
         * tick so the temperature gate and the calibration phases stay on a
         * correct clock. Outputs keep their last good values. */
        facade->gyro_read_ok = false;
        facade->gyro_error_count++;
        const imu_pipeline_output output =
            imu_pipeline_advance_time(&facade->pipeline, dt_s);
        facade->calibration_complete = output.calibration_complete;
        facade->sample_stationary = output.sample_stationary;
        facade->experiment_phase = output.experiment_phase;
        facade->calibration_reset_count = output.calibration_reset_count;
        return;
    }

    facade->gyro_read_ok = true;
    facade->sequence++;

    bool acceleration_fresh = false;
    bool acceleration_sampled = false;
    float acceleration_sample_dt_s = 0.0f;
    facade->acceleration_age_s += dt_s;

    /* The sequence == 1 case primes acceleration before the divider first
     * fires, so the attitude filter is not blind for the first 9 cycles. */
    if (facade->sequence == 1u ||
        (facade->sequence % IMU_FACADE_ACCEL_DIVIDER) == 0u) {
        const bmi088_accel_status accel_status =
            bmi088_accel_read(&facade->accel_device, &facade->accel_sample);
        acceleration_sampled = true;
        facade->acceleration_valid = (accel_status == BMI088_ACCEL_OK);
        if (facade->acceleration_valid) {
            acceleration_fresh = true;
            acceleration_sample_dt_s = facade->acceleration_age_s;
            facade->acceleration_age_s = 0.0f;
        } else {
            facade->accel_error_count++;
        }
    }

    imu_pipeline_input input = {
        .gyro_raw =
            {
                .x = gyro_sample.x_raw,
                .y = gyro_sample.y_raw,
                .z = gyro_sample.z_raw,
            },
        .gyro_dps =
            {
                .x_dps = bmi088_gyro_raw_to_dps(gyro_sample.x_raw),
                .y_dps = bmi088_gyro_raw_to_dps(gyro_sample.y_raw),
                .z_dps = bmi088_gyro_raw_to_dps(gyro_sample.z_raw),
            },
        .acceleration_g =
            {
                .x_g = (float)facade->accel_sample.x_ug / 1000000.0f,
                .y_g = (float)facade->accel_sample.y_ug / 1000000.0f,
                .z_g = (float)facade->accel_sample.z_ug / 1000000.0f,
            },
        .dt_s = dt_s,
        .skipped_cycles = 0u,
        .gyro_raw_valid = true,
        .gyro_clipped = bmi088_gyro_sample_is_clipped(&gyro_sample),
        .acceleration_valid = facade->acceleration_valid,
        .acceleration_fresh = acceleration_fresh,
        .acceleration_sampled = acceleration_sampled,
        .acceleration_age_s = facade->acceleration_age_s,
        .acceleration_sample_dt_s = acceleration_sample_dt_s,
    };

    if ((facade->sequence % IMU_FACADE_TEMPERATURE_DIVIDER) == 0u) {
        int32_t temperature_mdeg_c = 0;
        const bmi088_accel_status temperature_status =
            bmi088_accel_read_temperature(&facade->accel_device,
                                          &temperature_mdeg_c);
        input.temperature_sampled = true;
        if (temperature_status == BMI088_ACCEL_OK) {
            input.temperature_sample_valid = true;
            input.temperature_degc = (float)temperature_mdeg_c / 1000.0f;
        } else {
            /* A failed read deliberately closes the gate rather than reusing
             * stale data; that is the donor middleware's contract. */
            input.temperature_sample_valid = false;
        }
    }

    const imu_pipeline_output output =
        imu_pipeline_update_timed(&facade->pipeline, &input);

    facade->q_w = output.attitude.quaternion.w;
    facade->q_x = output.attitude.quaternion.x;
    facade->q_y = output.attitude.quaternion.y;
    facade->q_z = output.attitude.quaternion.z;

    facade->roll_deg = output.attitude.euler_zyx_deg.roll_deg;
    facade->pitch_deg = output.attitude.euler_zyx_deg.pitch_deg;
    facade->yaw_deg = output.attitude.euler_zyx_deg.yaw_deg;
    facade->euler_zyx_singular = output.attitude.euler_zyx_singular;
    facade->gyro_clip_count = output.gyro_clip_count;
    facade->heading_valid = output.heading_valid;

    /* angular_velocity_dps, not attitude_angular_velocity_dps: the latter has a
     * 1 dps static deadband applied when stationary, which would hide real slow
     * motion from a rate controller. */
    facade->rate_x_dps = output.angular_velocity_dps.x_dps;
    facade->rate_y_dps = output.angular_velocity_dps.y_dps;
    facade->rate_z_dps = output.angular_velocity_dps.z_dps;

    facade->temperature_degc = output.temperature_degc;
    facade->calibration_complete = output.calibration_complete;
    facade->sample_stationary = output.sample_stationary;
    facade->experiment_phase = output.experiment_phase;
    facade->calibration_reset_count = output.calibration_reset_count;
}
