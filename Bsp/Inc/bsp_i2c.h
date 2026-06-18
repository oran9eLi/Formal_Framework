/**
 * @file bsp_i2c.h
 * @brief 声明传感器驱动使用的板级 I2C 访问接口。
 */

#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdint.h>

#include "bsp_status.h"

/**
 * @brief 初始化共享板级 I2C 总线。
 */
BSP_Status_t BSP_I2C_Init(void);

/**
 * @brief 释放卡死的 I2C 总线并重新初始化外设。
 */
BSP_Status_t BSP_I2C_Recover(void);

/**
 * @brief 释放 I2C 外设资源。
 */
BSP_Status_t BSP_I2C_DeInit(void);

/**
 * @brief 探测一个 I2C 设备地址是否响应。
 */
BSP_Status_t BSP_I2C_IsDeviceReady(uint16_t dev_addr, uint32_t timeout_ms);

/**
 * @brief 从 8-bit 寄存器地址读取字节。
 */
BSP_Status_t BSP_I2C_MemRead(uint16_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
 * @brief 向 8-bit 寄存器地址写入字节。
 */
BSP_Status_t BSP_I2C_MemWrite(uint16_t dev_addr, uint16_t reg_addr, const uint8_t *data, uint16_t len, uint32_t timeout_ms);

#endif
