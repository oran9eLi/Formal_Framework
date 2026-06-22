/**
 * @file bsp_uart.h
 * @brief 声明板级调试 UART 接口。
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

#include "bsp_status.h"

/**
 * @brief 按配置引脚和波特率初始化调试 UART。
 */
BSP_Status_t BSP_UART_Init(void);
/**
 * @brief 通过调试 UART 在超时时间内发送完整字节缓冲区。
 */
BSP_Status_t BSP_UART_Send(const uint8_t *data, uint16_t length, uint32_t timeout_ms);

#endif
