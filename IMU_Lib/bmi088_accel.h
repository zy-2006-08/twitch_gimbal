/* Ported verbatim from the seven_code donor repo.
 * Donor origin: drivers/imu/bmi088/bmi088_accel.h
 * Do not edit locally; re-sync from the donor instead.
 */
/* Portable BMI088 accelerometer and temperature driver.
 *
 * The hardware boundary is an injected SPI bus: this driver has no vendor
 * HAL, board pin map, interrupt, DMA, or RTOS dependency. It therefore builds
 * unchanged for the target MCU and the host test harness.
 */
#ifndef DRIVERS_IMU_BMI088_BMI088_ACCEL_H
#define DRIVERS_IMU_BMI088_BMI088_ACCEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BMI088_ACCEL_OK = 0,
    BMI088_ACCEL_ERR_BUS,
    BMI088_ACCEL_ERR_CHIP_ID,
    BMI088_ACCEL_ERR_CONF_READBACK
} bmi088_accel_status;

/* Callbacks own the full chip-select-framed SPI exchange. transfer returns
 * zero on success and non-zero when the exchange failed. delay_ms blocks for
 * at least the requested duration. */
typedef struct {
    void *ctx;
    void (*select)(void *ctx);
    void (*deselect)(void *ctx);
    int (*transfer)(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len);
    void (*delay_ms)(void *ctx, uint32_t ms);
} bmi088_accel_bus;

/* Linear acceleration in micro-g, where 1 g is exactly 1,000,000 ug. */
typedef struct {
    int32_t x_ug;
    int32_t y_ug;
    int32_t z_ug;
} bmi088_accel_sample;

/* Driver handle. Treat its contents as private after bmi088_accel_init. */
typedef struct {
    bmi088_accel_bus bus;
} bmi088_accel;

/* Verifies the BMI088 accelerometer chip ID, resets the accelerometer, and
 * configures it enabled and active at a conservative 100 Hz normal-filter,
 * +/-12 g setting. Each configuration register is read back before success is
 * returned. */
bmi088_accel_status bmi088_accel_init(bmi088_accel *dev,
                                      const bmi088_accel_bus *bus);

/* Reads acceleration registers 0x12 through 0x17. The BMI088 accelerometer
 * emits a mandatory dummy byte after the read command, so data begins at
 * rx[2]. On failure sample remains exactly as supplied by the caller. */
bmi088_accel_status bmi088_accel_read(const bmi088_accel *dev,
                                      bmi088_accel_sample *sample);

/* Reads temperature registers 0x22 through 0x23 and returns milli-degrees
 * Celsius. On failure temperature_mdeg_c remains exactly as supplied. */
bmi088_accel_status bmi088_accel_read_temperature(
    const bmi088_accel *dev, int32_t *temperature_mdeg_c);

/* Converts one raw axis count to micro-g for the configured +/-12 g range. */
int32_t bmi088_accel_raw_to_ug(int16_t raw);

/* A short, stable, non-NULL description of a status value. */
const char *bmi088_accel_status_text(bmi088_accel_status status);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_IMU_BMI088_BMI088_ACCEL_H */
