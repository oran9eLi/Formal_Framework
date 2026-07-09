/**
 * @file bsp_uart.c
 * @brief 实现板级 UART 原始发送接口（调试 USART1 + 树莓派 MAVLink USART6）。
 *
 * @details
 * 本文件只封装串口的 HAL 初始化和阻塞发送，不做日志格式化、不做业务判断。
 * 调试输出必须通过 DebugConsole 互斥后调用；树莓派 UART 与调试口物理分离，
 * 两个句柄各自独立，互不影响。
 */

#include "bsp_uart.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

static UART_HandleTypeDef huart1;
#if (BSP_ENABLE_RPI_UART == 1U)
static UART_HandleTypeDef huart6;
#endif

/**
 * @brief 将 HAL UART 状态映射为 BSP 通用返回码。
 *
 * @param[in] status HAL 返回状态。
 *
 * @return BSP 通用返回码，避免向上层泄漏 HAL 类型。
 */
static BSP_Status_t BSP_UART_MapHalStatus(HAL_StatusTypeDef status)
{
  if (status == HAL_OK) { return BSP_STATUS_OK; }
  if (status == HAL_BUSY) { return BSP_STATUS_BUSY; }
  if (status == HAL_TIMEOUT) { return BSP_STATUS_TIMEOUT; }
  return BSP_STATUS_ERROR;
}

/**
 * @brief 按 `bsp_config.h` 配置初始化调试 UART。
 *
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_UART_Init(void)
{
  huart1.Instance          = BSP_DBG_UART;
  huart1.Init.BaudRate     = BSP_DBG_UART_BAUD;
  huart1.Init.WordLength   = BSP_DBG_UART_WORD;
  huart1.Init.StopBits     = BSP_DBG_UART_STOP;
  huart1.Init.Parity       = BSP_DBG_UART_PARITY;
  huart1.Init.Mode         = BSP_DBG_UART_MODE;
  huart1.Init.HwFlowCtl    = BSP_DBG_UART_HWCTL;
  huart1.Init.OverSampling = BSP_DBG_UART_OVERSAMP;
  return BSP_UART_MapHalStatus(HAL_UART_Init(&huart1));
}

/**
 * @brief 通过调试 UART 阻塞发送字节缓冲区。
 *
 * @param[in] data 待发送数据缓冲区，不能为 NULL。
 * @param[in] length 待发送字节数。
 * @param[in] timeout_ms HAL 阻塞发送超时时间，单位 ms。
 *
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_UART_Send(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
  return BSP_UART_MapHalStatus(HAL_UART_Transmit(&huart1, (uint8_t *)data, length, timeout_ms));
}

/**
 * @brief 复制调试 UART 当前 HAL 配置。
 *
 * @param[out] out 输出缓冲区，允许为 NULL。
 */
void BSP_UART_GetDebugInfo(BSP_UART_DebugInfo_t *out)
{
  if (out == 0) { return; }

  out->instance      = (uint32_t)(uintptr_t)huart1.Instance;
  out->baud_rate     = huart1.Init.BaudRate;
  out->word_length   = huart1.Init.WordLength;
  out->stop_bits     = huart1.Init.StopBits;
  out->parity        = huart1.Init.Parity;
  out->mode          = huart1.Init.Mode;
  out->hw_flow_ctl   = huart1.Init.HwFlowCtl;
  out->over_sampling = huart1.Init.OverSampling;
  out->initialized   = (huart1.gState != HAL_UART_STATE_RESET) ? 1U : 0U;
}

#if (BSP_ENABLE_RPI_UART == 1U)
/**
 * @brief 按 `bsp_config.h` 配置初始化树莓派 MAVLink UART（USART6）。
 *
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_RpiUART_Init(void)
{
  huart6.Instance          = BSP_RPI_UART;
  huart6.Init.BaudRate     = BSP_RPI_UART_BAUD;
  huart6.Init.WordLength   = BSP_RPI_UART_WORD;
  huart6.Init.StopBits     = BSP_RPI_UART_STOP;
  huart6.Init.Parity       = BSP_RPI_UART_PARITY;
  huart6.Init.Mode         = BSP_RPI_UART_MODE;
  huart6.Init.HwFlowCtl    = BSP_RPI_UART_HWCTL;
  huart6.Init.OverSampling = BSP_RPI_UART_OVERSAMP;
  return BSP_UART_MapHalStatus(HAL_UART_Init(&huart6));
}

/**
 * @brief 通过树莓派 UART 阻塞发送字节缓冲区。
 *
 * @param[in] data 待发送数据缓冲区，不能为 NULL。
 * @param[in] length 待发送字节数。
 * @param[in] timeout_ms HAL 阻塞发送超时时间，单位 ms。
 *
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_RpiUART_Send(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
  return BSP_UART_MapHalStatus(HAL_UART_Transmit(&huart6, (uint8_t *)data, length, timeout_ms));
}
#endif
