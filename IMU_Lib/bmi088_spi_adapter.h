/* SPI bus adapter binding the portable BMI088 drivers to this project's HAL.
 *
 * Target specific by design: this is the ONLY file in IMU_Lib that knows about
 * STM32 HAL, SPI1, or the PA4 / PC4 chip-select pins. Everything else in
 * IMU_Lib is the portable donor code and stays hardware agnostic.
 *
 * Wiring on this board:
 *   SPI1 (hspi1), prescaler 16 -> ~5.25 MBit/s, mode 0, 8-bit, MSB first
 *   accelerometer chip select = PA4
 *   gyroscope     chip select = PC4
 *
 * Transfers are blocking and synchronous on purpose. analyse() runs inside the
 * TIM7 1 kHz ISR and the whole point of keeping HAL_SPI_TransmitReceive is that
 * it is predictable; no DMA and no interrupt-driven SPI is introduced here.
 * The timeout is deliberately short so a dead sensor cannot stall the ISR.
 */
#ifndef IMU_LIB_BMI088_SPI_ADAPTER_H
#define IMU_LIB_BMI088_SPI_ADAPTER_H

#include <stdint.h>

#include "bmi088_accel.h"
#include "bmi088_gyro.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Per-device adapter context: which SPI peripheral and which chip-select pin.
 * Held by value inside the driver handles, so no globals are required. */
typedef struct {
    void *hspi;           /* SPI_HandleTypeDef*, opaque here to stay HAL-free */
    void *cs_port;        /* GPIO_TypeDef* */
    uint16_t cs_pin;
} bmi088_spi_adapter;

/* Builds a gyroscope bus value object bound to the supplied adapter context.
 * The adapter must outlive the driver handle. */
bmi088_gyro_bus bmi088_spi_adapter_gyro_bus(bmi088_spi_adapter *adapter);

/* Builds an accelerometer bus value object bound to the supplied context. */
bmi088_accel_bus bmi088_spi_adapter_accel_bus(bmi088_spi_adapter *adapter);

#ifdef __cplusplus
}
#endif

#endif /* IMU_LIB_BMI088_SPI_ADAPTER_H */
