/**
 * @file bsp_i2c.h
 * @brief Declare board I2C access helpers for sensor drivers.
 */

#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdint.h>

#include "bsp_status.h"

/**
 * @brief Initialize the shared board I2C bus.
 */
BSP_Status_t BSP_I2C_Init(void);

/**
 * @brief Release a hung I2C bus and re-initialize the peripheral.
 */
BSP_Status_t BSP_I2C_Recover(void);

/**
 * @brief Release the I2C peripheral so it consumes no resources.
 */
BSP_Status_t BSP_I2C_DeInit(void);

/**
 * @brief Probe an I2C device address.
 */
BSP_Status_t BSP_I2C_IsDeviceReady(uint16_t dev_addr,
                                   uint32_t timeout_ms);

/**
 * @brief Read bytes from an 8-bit register address.
 */
BSP_Status_t BSP_I2C_MemRead(uint16_t dev_addr,
                             uint16_t reg_addr,
                             uint8_t *data,
                             uint16_t len,
                             uint32_t timeout_ms);

/**
 * @brief Write bytes to an 8-bit register address.
 */
BSP_Status_t BSP_I2C_MemWrite(uint16_t dev_addr,
                              uint16_t reg_addr,
                              const uint8_t *data,
                              uint16_t len,
                              uint32_t timeout_ms);

#endif
