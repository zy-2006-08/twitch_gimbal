/* Ported verbatim from the seven_code donor repo.
 * Donor origin: drivers/imu/bmi088/bmi088_gyro.c
 * Do not edit locally; re-sync from the donor instead.
 */
/* Portable BMI088 gyroscope driver implementation.
 *
 * Contains no vendor HAL, no board header, and no RTOS call: every hardware
 * effect goes through the injected bus value object. All arithmetic is
 * integer, and nothing here formats or prints.
 */
#include "bmi088_gyro.h"

/* Gyroscope register map, from the BMI088 datasheet. Only these are used;
 * accelerometer and interrupt registers are deliberately absent. */
#define REG_CHIP_ID 0x00u
#define REG_RATE_X_LSB 0x02u
#define REG_RANGE 0x0Fu
#define REG_BANDWIDTH 0x10u
#define REG_POWER 0x11u
#define REG_SOFTRESET 0x14u

#define CHIP_ID_GYRO 0x0Fu
#define SOFTRESET_CMD 0xB6u

/* RoboMaster operation keeps the full dynamic range and reads every 1 kHz
 * output. The 116 Hz filter is selected by 0x82 because bit 7 reads back as
 * one in addition to the 0x02 mode code. */
#define RANGE_2000_DPS 0x00u
#define BANDWIDTH_1000HZ_ODR_116HZ 0x82u
#define POWER_NORMAL 0x00u

/* SPI framing: bit 7 of the command byte selects a read. */
#define SPI_READ_FLAG 0x80u

/* Reset settling time, in milliseconds, and the generic register settle. */
#define RESET_DELAY_MS 80u
#define REG_SETTLE_MS 1u

/* A burst covering chip ID through RATE_Z_MSB (0x00..0x07): one command
 * byte plus eight data bytes shifted out one index later. */
#define BURST_REG_COUNT 8u
#define BURST_LEN (BURST_REG_COUNT + 1u)

/* Byte offsets of each axis LSB within the burst receive buffer. */
#define BURST_X_LSB (1u + REG_RATE_X_LSB)
#define BURST_Y_LSB (BURST_X_LSB + 2u)
#define BURST_Z_LSB (BURST_Y_LSB + 2u)

/* Milli-dps per LSB expressed exactly as a rational number: at +/-2000 dps
 * one LSB is 2000/32768 dps == 15625/256 mdps. */
#define MDPS_NUMERATOR 15625
#define MDPS_DENOMINATOR 256
#define DPS_NUMERATOR 2000.0f
#define DPS_DENOMINATOR 32768.0f

/* Performs one chip-select-framed transfer, keeping select and deselect
 * paired even when the bus reports an error. */
static bmi088_gyro_status bus_transfer(const bmi088_gyro_bus *bus,
                                       const uint8_t *tx, uint8_t *rx,
                                       uint16_t len) {
    bus->select(bus->ctx);
    const int result = bus->transfer(bus->ctx, tx, rx, len);
    bus->deselect(bus->ctx);
    return (result == 0) ? BMI088_GYRO_OK : BMI088_GYRO_ERR_BUS;
}

/* Reads a single register. The BMI088 shifts the value out one byte after
 * the command, so rx[1] holds the data and rx[0] is discarded. */
static bmi088_gyro_status read_reg(const bmi088_gyro_bus *bus, uint8_t reg,
                                   uint8_t *value) {
    const uint8_t tx[2] = {(uint8_t)(reg | SPI_READ_FLAG), 0x00u};
    uint8_t rx[2] = {0u, 0u};
    const bmi088_gyro_status status = bus_transfer(bus, tx, rx, 2u);
    if (status != BMI088_GYRO_OK) {
        return status;
    }
    *value = rx[1];
    return BMI088_GYRO_OK;
}

static bmi088_gyro_status write_reg(const bmi088_gyro_bus *bus, uint8_t reg,
                                    uint8_t value) {
    const uint8_t tx[2] = {(uint8_t)(reg & (uint8_t)~SPI_READ_FLAG), value};
    uint8_t rx[2] = {0u, 0u};
    return bus_transfer(bus, tx, rx, 2u);
}

/* Confirms the part on the bus is the gyroscope and not the co-packaged
 * accelerometer or an unrelated device. */
static bmi088_gyro_status verify_chip_id(const bmi088_gyro_bus *bus) {
    uint8_t id = 0u;
    const bmi088_gyro_status status = read_reg(bus, REG_CHIP_ID, &id);
    if (status != BMI088_GYRO_OK) {
        return status;
    }
    return (id == CHIP_ID_GYRO) ? BMI088_GYRO_OK : BMI088_GYRO_ERR_CHIP_ID;
}

/* Writes a configuration register, lets it settle, then reads it back.
 * A silent refusal is reported using the caller-supplied status so the
 * failure names the specific register. */
static bmi088_gyro_status write_verified(const bmi088_gyro_bus *bus,
                                         uint8_t reg, uint8_t value,
                                         bmi088_gyro_status mismatch) {
    bmi088_gyro_status status = write_reg(bus, reg, value);
    if (status != BMI088_GYRO_OK) {
        return status;
    }
    bus->delay_ms(bus->ctx, REG_SETTLE_MS);

    uint8_t readback = 0u;
    status = read_reg(bus, reg, &readback);
    if (status != BMI088_GYRO_OK) {
        return status;
    }
    return (readback == value) ? BMI088_GYRO_OK : mismatch;
}

bmi088_gyro_status bmi088_gyro_init(bmi088_gyro *dev,
                                    const bmi088_gyro_bus *bus) {
    dev->bus = *bus;
    const bmi088_gyro_bus *b = &dev->bus;

    /* The first SPI access after power-up wakes the interface and may read
     * back garbage, so the ID is read twice and only the second counts. */
    uint8_t discarded = 0u;
    bmi088_gyro_status status = read_reg(b, REG_CHIP_ID, &discarded);
    if (status != BMI088_GYRO_OK) {
        return status;
    }
    b->delay_ms(b->ctx, REG_SETTLE_MS);

    status = verify_chip_id(b);
    if (status != BMI088_GYRO_OK) {
        return status;
    }

    /* Soft reset to a known state, then wait out the datasheet start-up
     * time before touching the device again. */
    status = write_reg(b, REG_SOFTRESET, SOFTRESET_CMD);
    if (status != BMI088_GYRO_OK) {
        return status;
    }
    b->delay_ms(b->ctx, RESET_DELAY_MS);

    /* The reset drops the SPI interface, so confirm the part came back. */
    status = verify_chip_id(b);
    if (status != BMI088_GYRO_OK) {
        return status;
    }

    status = write_verified(b, REG_RANGE, RANGE_2000_DPS,
                            BMI088_GYRO_ERR_RANGE_READBACK);
    if (status != BMI088_GYRO_OK) {
        return status;
    }
    status = write_verified(b, REG_BANDWIDTH, BANDWIDTH_1000HZ_ODR_116HZ,
                            BMI088_GYRO_ERR_BANDWIDTH_READBACK);
    if (status != BMI088_GYRO_OK) {
        return status;
    }
    return write_verified(b, REG_POWER, POWER_NORMAL,
                          BMI088_GYRO_ERR_POWER_READBACK);
}

/* Rebuilds one signed axis count from its little-endian register pair. */
static int16_t decode_axis(const uint8_t *rx, uint8_t lsb_offset) {
    const uint16_t raw =
        (uint16_t)((uint16_t)rx[lsb_offset] |
                   (uint16_t)((uint16_t)rx[lsb_offset + 1u] << 8));
    return (int16_t)raw;
}

bmi088_gyro_status bmi088_gyro_read(const bmi088_gyro *dev,
                                    bmi088_gyro_sample *sample) {
    const bmi088_gyro_bus *b = &dev->bus;

    /* One transaction starting at the chip ID: the rate registers stream
     * out contiguously, and the ID byte rides along as a framing canary. */
    uint8_t tx[BURST_LEN] = {(uint8_t)(REG_CHIP_ID | SPI_READ_FLAG)};
    uint8_t rx[BURST_LEN] = {0u};
    const bmi088_gyro_status status = bus_transfer(b, tx, rx, BURST_LEN);
    if (status != BMI088_GYRO_OK) {
        return status;
    }

    /* A wrong ID here means the frame slipped, so the rate bytes in this
     * same transaction are untrustworthy and the sample is left alone. */
    if (rx[1] != CHIP_ID_GYRO) {
        return BMI088_GYRO_ERR_CHIP_ID;
    }

    const int16_t x_raw = decode_axis(rx, BURST_X_LSB);
    const int16_t y_raw = decode_axis(rx, BURST_Y_LSB);
    const int16_t z_raw = decode_axis(rx, BURST_Z_LSB);
    *sample = (bmi088_gyro_sample){
        .x_mdps = bmi088_gyro_raw_to_mdps(x_raw),
        .y_mdps = bmi088_gyro_raw_to_mdps(y_raw),
        .z_mdps = bmi088_gyro_raw_to_mdps(z_raw),
        .x_raw = x_raw,
        .y_raw = y_raw,
        .z_raw = z_raw,
    };
    return BMI088_GYRO_OK;
}

int32_t bmi088_gyro_raw_to_mdps(int16_t raw) {
    /* int32 is sufficient: the widest intermediate is 32768 * 15625, which
     * is about 5.1e8 and well inside the int32 range. */
    const int32_t scaled = (int32_t)raw * (int32_t)MDPS_NUMERATOR;
    return scaled / (int32_t)MDPS_DENOMINATOR;
}

float bmi088_gyro_raw_to_dps(int16_t raw) {
    return (float)raw * DPS_NUMERATOR / DPS_DENOMINATOR;
}

const char *bmi088_gyro_status_text(bmi088_gyro_status status) {
    switch (status) {
        case BMI088_GYRO_OK:
            return "ok";
        case BMI088_GYRO_ERR_BUS:
            return "spi bus transfer failed";
        case BMI088_GYRO_ERR_CHIP_ID:
            return "unexpected chip id";
        case BMI088_GYRO_ERR_RANGE_READBACK:
            return "range register readback mismatch";
        case BMI088_GYRO_ERR_BANDWIDTH_READBACK:
            return "bandwidth register readback mismatch";
        case BMI088_GYRO_ERR_POWER_READBACK:
            return "power register readback mismatch";
    }
    return "unknown status";
}
