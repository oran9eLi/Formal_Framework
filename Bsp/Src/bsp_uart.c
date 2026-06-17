/**
 * @file bsp_uart.c
 * @brief Implement USART1 debug console transmission.
 */

#include "bsp_uart.h"
#include "bsp_config.h"
#include "stm32f4xx_hal.h"

/***************DEBUG***************/
static UART_HandleTypeDef huart1;

/*
 * Map a HAL status onto the unified board status type so the public UART API
 * never leaks HAL_StatusTypeDef to callers.
 */
static BSP_Status_t BSP_UART_MapHalStatus(HAL_StatusTypeDef status)
{
  if (status == HAL_OK)
  {
    return BSP_STATUS_OK;
  }
  if (status == HAL_BUSY)
  {
    return BSP_STATUS_BUSY;
  }
  if (status == HAL_TIMEOUT)
  {
    return BSP_STATUS_TIMEOUT;
  }
  return BSP_STATUS_ERROR;
}

/**
 * @brief Initialize the board debug UART with the configured pins and baud rate.
 */
BSP_Status_t BSP_UART_Init(void)
{
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  return BSP_UART_MapHalStatus(HAL_UART_Init(&huart1));
}

/**
 * @brief Send a complete byte buffer through the debug UART with a timeout.
 */
BSP_Status_t BSP_UART_Send(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
  return BSP_UART_MapHalStatus(
      HAL_UART_Transmit(&huart1, (uint8_t *)data, length, timeout_ms));
}
/*************DEBUG END*************/
