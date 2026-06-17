/**
 * @file bsp_uart.c
 * @brief 实现 USART1 调试控制台发送接口。
 */

#include "bsp_uart.h"
#include "bsp_config.h"
#include "stm32f4xx_hal.h"

/***************DEBUG***************/
static UART_HandleTypeDef huart1;

/**
 * @brief 将 HAL UART 状态映射为 BSP 通用返回码。
 *
 * @param[in] status HAL 返回状态。
 *
 * @return BSP 通用返回码，避免向上层泄漏 HAL_StatusTypeDef。
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
 * @brief 按配置引脚和波特率初始化调试 UART。
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
 * @brief 通过调试 UART 在超时时间内发送完整字节缓冲区。
 */
BSP_Status_t BSP_UART_Send(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
  return BSP_UART_MapHalStatus(
      HAL_UART_Transmit(&huart1, (uint8_t *)data, length, timeout_ms));
}
/*************DEBUG END*************/
