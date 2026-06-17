/**
 * @file bsp_spi.h
 * @brief 声明 SD 卡日志使用的板级 SPI 接口。
 *
 * @details
 * 当前接口服务于 SD/FatFs 路径，负责 SPI3 初始化、速率切换、阻塞收发和片选控制。
 * 文件系统协议逻辑不放在 BSP 层。
 */

#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdint.h>
#include "bsp_status.h"

/**
 * @brief 初始化 SD 卡 SPI 总线，默认低速。
 */
void BSP_SPI1_Init(void);
/**
 * @brief 将 SD 卡 SPI 总线切换到初始化低速。
 */
BSP_Status_t BSP_SPI1_SetSpeedLow(void);
/**
 * @brief 将 SD 卡 SPI 总线切换到传输高速。
 */
BSP_Status_t BSP_SPI1_SetSpeedHigh(void);
/**
 * @brief 通过 SD 卡 SPI 总线发送字节。
 */
BSP_Status_t BSP_SPI1_Transmit(const uint8_t *tx,
                               uint16_t length,
                               uint32_t timeout_ms);
/**
 * @brief 通过 SD 卡 SPI 总线接收字节。
 */
BSP_Status_t BSP_SPI1_Receive(uint8_t *rx,
                              uint16_t length,
                              uint32_t timeout_ms);
/**
 * @brief 通过 SD 卡 SPI 总线同时发送和接收字节。
 */
BSP_Status_t BSP_SPI1_TransmitReceive(const uint8_t *tx,
                                      uint8_t *rx,
                                      uint16_t length,
                                      uint32_t timeout_ms);
/**
 * @brief 拉低 SD 卡片选。
 */
void BSP_SPI1_SD_Select(void);
/**
 * @brief 拉高 SD 卡片选。
 */
void BSP_SPI1_SD_Deselect(void);

#endif
