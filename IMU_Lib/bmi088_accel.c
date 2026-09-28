/* Ported verbatim from the seven_code donor repo.
 * Donor origin: drivers/imu/bmi088/bmi088_accel.c
 * Do not edit locally; re-sync from the donor instead.
 */
/* Portable BMI088 accelerometer and temperature driver implementation. */
#include "bmi088_accel.h"

/* BMI088 accelerometer register map. */
#define REG_CHIP_ID 0x00u
#define REG_ACCEL_X_LSB 0x12u
#define REG_TEMP_MSB 0x22u
#define REG_ACCEL_CONF 0x40u
#define REG_ACCEL_RANGE 0x41u
#define REG_PWR_CONF 0x7Cu
#define REG_PWR_CTRL 0x7Du
#define REG_SOFTRESET 0x7Eu

#define CHIP_ID_ACCEL 0x1Eu
#define SOFTRESET_CMD 0xB6u
#define SPI_READ_FLAG 0x80u

/* The accelerometer requires one command byte and one dummy byte before its
 * first payload byte. Every accel read below indexes received payload at 2. */
#define ACCEL_READ_OVERHEAD 2u

#define ACCEL_AXIS_COUNT 3u
#define ACCEL_AXIS_BYTE_COUNT 2u
#define ACCEL_SAMPLE_BYTE_COUNT (ACCEL_AXIS_COUNT * ACCEL_AXIS_BYTE_COUNT)
#define TEMP_BYTE_COUNT 2u

/* Datasheet-minimum reset delay is one millisecond. Two milliseconds is a
 * conservative board-independent choice. Power transitions need their own
 * documented settling times before the next SPI exchange. */
#define RESET_SETTLE_MS 2u
#define POWER_ENABLE_SETTLE_MS 50u
#define POWER_ACTIVE_SETTLE_MS 1u
#define REGISTER_SETTLE_MS 1u

/* ACC_CONF 0xA8 selects normal bandwidth filtering with 100 Hz ODR. Together
 * with +/-12 g it is deliberately conservative: low enough to start polling
 * safely, without configuring interrupts or any data-ready routing. */
#define ACC_CONF_NORMAL_100HZ 0xA8u
#define ACC_RANGE_12G 0x02u
#define PWR_CTRL_ACCEL_ENABLE 0x04u
#define PWR_CONF_ACTIVE 0x00u

/* At +/-12 g, 32,768 counts span 12 g, so each count is exactly
 * 375,000 / 1,024 ug. The intermediate needs int64_t at full scale. */
#define UG_NUMERATOR 375000
#define UG_DENOMINATOR 1024

static bmi088_accel_status bus_transfer(const bmi088_accel_bus *bus,
                                        const uint8_t *tx, uint8_t *rx,
                                        uint16_t len) {
    bus->select(bus->ctx);
    const int result = bus->transfer(bus->ctx, tx, rx, len);
    bus->deselect(bus->ctx);
    return (result == 0) ? BMI088_ACCEL_OK : BMI088_ACCEL_ERR_BUS;
}

/* Reads consecutive registers using the BMI088 accel's mandatory dummy byte.
 * data is assigned only after the full transaction has succeeded. */
static bmi088_accel_status read_registers(const bmi088_accel_bus *bus,
                                          uint8_t reg, uint8_t *data,
                                          uint16_t count) {
    uint8_t tx[ACCEL_READ_OVERHEAD + ACCEL_SAMPLE_BYTE_COUNT] = {
        (uint8_t)(reg | SPI_READ_FLAG),
    };
    uint8_t rx[ACCEL_READ_OVERHEAD + ACCEL_SAMPLE_BYTE_COUNT] = {0u};
    const uint16_t length = (uint16_t)(ACCEL_READ_OVERHEAD + count);
    const bmi088_accel_status status = bus_transfer(bus, tx, rx, length);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    for (uint16_t index = 0u; index < count; index++) {
        data[index] = rx[ACCEL_READ_OVERHEAD + index];
    }
    return BMI088_ACCEL_OK;
}

static bmi088_accel_status read_register(const bmi088_accel_bus *bus,
                                         uint8_t reg, uint8_t *value) {
    uint8_t data = 0u;
    const bmi088_accel_status status = read_registers(bus, reg, &data, 1u);
    if (status == BMI088_ACCEL_OK) {
        *value = data;
    }
    return status;
}

static bmi088_accel_status write_register(const bmi088_accel_bus *bus,
                                          uint8_t reg, uint8_t value) {
    const uint8_t tx[2] = {(uint8_t)(reg & (uint8_t)~SPI_READ_FLAG), value};
    uint8_t rx[2] = {0u, 0u};
    return bus_transfer(bus, tx, rx, 2u);
}

static bmi088_accel_status verify_chip_id(const bmi088_accel_bus *bus) {
    uint8_t chip_id = 0u;
    const bmi088_accel_status status =
        read_register(bus, REG_CHIP_ID, &chip_id);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    return (chip_id == CHIP_ID_ACCEL) ? BMI088_ACCEL_OK
                                      : BMI088_ACCEL_ERR_CHIP_ID;
}

static bmi088_accel_status write_verified(const bmi088_accel_bus *bus,
                                          uint8_t reg, uint8_t value,
                                          uint32_t settle_ms) {
    bmi088_accel_status status = write_register(bus, reg, value);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    bus->delay_ms(bus->ctx, settle_ms);

    uint8_t readback = 0u;
    status = read_register(bus, reg, &readback);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    return (readback == value) ? BMI088_ACCEL_OK
                               : BMI088_ACCEL_ERR_CONF_READBACK;
}

bmi088_accel_status bmi088_accel_init(bmi088_accel *dev,
                                      const bmi088_accel_bus *bus) {
    dev->bus = *bus;
    const bmi088_accel_bus *active_bus = &dev->bus;

    /* The accelerometer selects SPI only after chip-select returns high from
     * an initial transaction. Discard the first result, then identify it. */
    uint8_t discarded = 0u;
    bmi088_accel_status status =
        read_register(active_bus, REG_CHIP_ID, &discarded);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    status = verify_chip_id(active_bus);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    status = write_register(active_bus, REG_SOFTRESET, SOFTRESET_CMD);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    active_bus->delay_ms(active_bus->ctx, RESET_SETTLE_MS);

    /* Soft reset returns the accel interface to I2C mode, so wake SPI again
     * before consuming the first post-reset response. */
    status = read_register(active_bus, REG_CHIP_ID, &discarded);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    status = verify_chip_id(active_bus);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    status = write_verified(active_bus, REG_PWR_CTRL, PWR_CTRL_ACCEL_ENABLE,
                            POWER_ENABLE_SETTLE_MS);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    status = write_verified(active_bus, REG_PWR_CONF, PWR_CONF_ACTIVE,
                            POWER_ACTIVE_SETTLE_MS);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    status = write_verified(active_bus, REG_ACCEL_CONF, ACC_CONF_NORMAL_100HZ,
                            REGISTER_SETTLE_MS);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }
    return write_verified(active_bus, REG_ACCEL_RANGE, ACC_RANGE_12G,
                          REGISTER_SETTLE_MS);
}

static int16_t decode_little_endian_i16(const uint8_t *data) {
    const uint16_t raw = (uint16_t)((uint16_t)data[0] |
                                    ((uint16_t)data[1] << 8));
    return (int16_t)raw;
}

bmi088_accel_status bmi088_accel_read(const bmi088_accel *dev,
                                      bmi088_accel_sample *sample) {
    uint8_t data[ACCEL_SAMPLE_BYTE_COUNT] = {0u};
    const bmi088_accel_status status =
        read_registers(&dev->bus, REG_ACCEL_X_LSB, data,
                       ACCEL_SAMPLE_BYTE_COUNT);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    const bmi088_accel_sample decoded = {
        .x_ug = bmi088_accel_raw_to_ug(decode_little_endian_i16(&data[0])),
        .y_ug = bmi088_accel_raw_to_ug(decode_little_endian_i16(&data[2])),
        .z_ug = bmi088_accel_raw_to_ug(decode_little_endian_i16(&data[4])),
    };
    *sample = decoded;
    return BMI088_ACCEL_OK;
}

bmi088_accel_status bmi088_accel_read_temperature(
    const bmi088_accel *dev, int32_t *temperature_mdeg_c) {
    uint8_t data[TEMP_BYTE_COUNT] = {0u};
    const bmi088_accel_status status =
        read_registers(&dev->bus, REG_TEMP_MSB, data, TEMP_BYTE_COUNT);
    if (status != BMI088_ACCEL_OK) {
        return status;
    }

    /* Temperature is an 11-bit signed value: bits 10..3 are 0x22 and bits
     * 2..0 are 0x23[7:5]. Each count is 0.125 C with a 23 C offset. */
    int16_t raw_temperature =
        (int16_t)(((uint16_t)data[0] << 3) | ((uint16_t)data[1] >> 5));
    if (raw_temperature > 1023) {
        raw_temperature -= 2048;
    }
    *temperature_mdeg_c = 23000 + (int32_t)raw_temperature * 125;
    return BMI088_ACCEL_OK;
}

int32_t bmi088_accel_raw_to_ug(int16_t raw) {
    const int64_t scaled_ug = (int64_t)raw * (int64_t)UG_NUMERATOR;
    return (int32_t)(scaled_ug / (int64_t)UG_DENOMINATOR);
}

const char *bmi088_accel_status_text(bmi088_accel_status status) {
    switch (status) {
        case BMI088_ACCEL_OK:
            return "ok";
        case BMI088_ACCEL_ERR_BUS:
            return "spi bus transfer failed";
        case BMI088_ACCEL_ERR_CHIP_ID:
            return "unexpected chip id";
        case BMI088_ACCEL_ERR_CONF_READBACK:
            return "configuration register readback mismatch";
    }
    return "unknown status";
}
