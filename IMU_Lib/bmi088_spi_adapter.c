/* SPI bus adapter implementation. See bmi088_spi_adapter.h for the contract. */
#include "bmi088_spi_adapter.h"

#include "main.h"
#include "spi.h"

/* Short on purpose: a 14 byte burst at ~5.25 MBit/s is well under 30 us, so
 * anything past a couple of milliseconds means the sensor or bus is dead and we
 * must return to the 1 kHz ISR rather than block on it. */
#define BMI088_SPI_TIMEOUT_MS 3u

static void bmi088_spi_select(void *ctx)
{
    bmi088_spi_adapter *adapter = (bmi088_spi_adapter *)ctx;
    HAL_GPIO_WritePin((GPIO_TypeDef *)adapter->cs_port, adapter->cs_pin,
                      GPIO_PIN_RESET);
}

static void bmi088_spi_deselect(void *ctx)
{
    bmi088_spi_adapter *adapter = (bmi088_spi_adapter *)ctx;
    HAL_GPIO_WritePin((GPIO_TypeDef *)adapter->cs_port, adapter->cs_pin,
                      GPIO_PIN_SET);
}

/* One full-duplex exchange of len bytes. Returns 0 on success, non-zero on any
 * bus error, exactly as the donor driver's transfer contract requires. */
static int bmi088_spi_transfer(void *ctx, const uint8_t *tx, uint8_t *rx,
                               uint16_t len)
{
    bmi088_spi_adapter *adapter = (bmi088_spi_adapter *)ctx;
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        (SPI_HandleTypeDef *)adapter->hspi, (uint8_t *)tx, rx, len,
        BMI088_SPI_TIMEOUT_MS);
    return (status == HAL_OK) ? 0 : -1;
}

static void bmi088_spi_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    HAL_Delay(ms);
}

bmi088_gyro_bus bmi088_spi_adapter_gyro_bus(bmi088_spi_adapter *adapter)
{
    bmi088_gyro_bus bus;
    bus.ctx = adapter;
    bus.select = bmi088_spi_select;
    bus.deselect = bmi088_spi_deselect;
    bus.transfer = bmi088_spi_transfer;
    bus.delay_ms = bmi088_spi_delay_ms;
    return bus;
}

bmi088_accel_bus bmi088_spi_adapter_accel_bus(bmi088_spi_adapter *adapter)
{
    bmi088_accel_bus bus;
    bus.ctx = adapter;
    bus.select = bmi088_spi_select;
    bus.deselect = bmi088_spi_deselect;
    bus.transfer = bmi088_spi_transfer;
    bus.delay_ms = bmi088_spi_delay_ms;
    return bus;
}
