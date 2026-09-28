/* Ported verbatim from the seven_code donor repo.
 * Donor origin: drivers/imu/bmi088/bmi088_gyro.h
 * Do not edit locally; re-sync from the donor instead.
 */
/* Portable BMI088 gyroscope driver.
 *
 * Hardware independent by construction: the caller injects the SPI bus as a
 * value object, so this driver contains no chip vendor HAL, no board pin
 * map, and no RTOS dependency. The same source compiles for an MCU target
 * and for a host test harness.
 *
 * The gyroscope is treated as read-only after configuration. The driver
 * never touches accelerometer registers, data-ready interrupt registers, or
 * any interrupt routing. Samples retain both raw counts and integer
 * milli-degrees per second: raw counts are the precision source for long bias
 * estimation, while mdps remains convenient debugger observability.
 */
#ifndef DRIVERS_IMU_BMI088_BMI088_GYRO_H
#define DRIVERS_IMU_BMI088_BMI088_GYRO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Every way the driver can fail, each naming exactly one cause so a caller
 * can distinguish a dead bus from a wrong part from a refused write. */
typedef enum {
    BMI088_GYRO_OK = 0,
    BMI088_GYRO_ERR_BUS,
    BMI088_GYRO_ERR_CHIP_ID,
    BMI088_GYRO_ERR_RANGE_READBACK,
    BMI088_GYRO_ERR_BANDWIDTH_READBACK,
    BMI088_GYRO_ERR_POWER_READBACK
} bmi088_gyro_status;

/* The SPI bus the caller injects.
 *
 * ctx is passed back to every callback so an adapter can carry its own
 * peripheral handle and chip-select pin without a global. select and
 * deselect drive chip select; transfer performs one full-duplex exchange of
 * len bytes and returns 0 on success, non-zero on any bus error; delay_ms
 * blocks for at least the requested milliseconds.
 */
typedef struct {
    void *ctx;
    void (*select)(void *ctx);
    void (*deselect)(void *ctx);
    int (*transfer)(void *ctx, const uint8_t *tx, uint8_t *rx, uint16_t len);
    void (*delay_ms)(void *ctx, uint32_t ms);
} bmi088_gyro_bus;

/* One angular-rate reading, in milli-degrees per second. */
typedef struct {
    int32_t x_mdps;
    int32_t y_mdps;
    int32_t z_mdps;
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
} bmi088_gyro_sample;

/* Driver handle. Holds the injected bus by value; treat the contents as
 * private and construct only through bmi088_gyro_init. */
typedef struct {
    bmi088_gyro_bus bus;
} bmi088_gyro;

/* Brings the gyroscope up: verifies the chip ID, soft resets, then writes
 * and reads back the range, bandwidth, and power configuration. Returns
 * BMI088_GYRO_OK only when every step is confirmed against the device. */
bmi088_gyro_status bmi088_gyro_init(bmi088_gyro *dev,
                                    const bmi088_gyro_bus *bus);

/* Reads one angular-rate sample. On any failure the sample is left exactly
 * as the caller passed it, so a caller may keep using its last good value. */
bmi088_gyro_status bmi088_gyro_read(const bmi088_gyro *dev,
                                    bmi088_gyro_sample *sample);

/* Converts a raw axis count to milli-degrees per second.
 *
 * At the configured +/-2000 dps range one LSB is 2000/32768 dps, which is
 * exactly 15625/256 mdps, so the conversion is integer-exact and needs no
 * floating point. */
int32_t bmi088_gyro_raw_to_mdps(int16_t raw);

/* Converts a raw count without passing through integer mdps quantization. */
float bmi088_gyro_raw_to_dps(int16_t raw);

/* A short, stable, non-NULL description of a status value. */
const char *bmi088_gyro_status_text(bmi088_gyro_status status);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_IMU_BMI088_BMI088_GYRO_H */
