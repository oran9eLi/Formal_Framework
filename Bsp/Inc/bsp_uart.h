/**
 * @file bsp_uart.h
 * @brief Declare the debug UART board interface.
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

#include "bsp_status.h"

/**
 * @brief Initialize the board debug UART with the configured pins and baud rate.
 */
BSP_Status_t BSP_UART_Init(void);
/**
 * @brief Send a complete byte buffer through the debug UART with a timeout.
 */
BSP_Status_t BSP_UART_Send(const uint8_t *data, uint16_t length, uint32_t timeout_ms);

#endif
