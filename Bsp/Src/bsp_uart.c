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
#include "bsp_critical.h"
#include "stm32f4xx_hal.h"

static UART_HandleTypeDef huart1;
#if (BSP_ENABLE_RPI_UART == 1U)
static UART_HandleTypeDef huart6;
static uint8_t s_rpi_rx_buf[BSP_RPI_RX_BUF_SIZE];
static volatile uint16_t s_rpi_rx_head;
static volatile uint16_t s_rpi_rx_tail;
static volatile uint16_t s_rpi_rx_count;
static volatile uint32_t s_rpi_rx_drop_count;
static volatile uint32_t s_rpi_rx_error_count;
static volatile uint8_t s_rpi_initialized;
static volatile uint8_t s_rpi_reconfiguring;
static uint8_t s_rpi_rx_byte;
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
static void BSP_RpiUART_ResetRxState(uint8_t reset_counters)
{
  uint32_t primask;

  primask = BSP_Critical_Enter();
  s_rpi_rx_head        = 0U;
  s_rpi_rx_tail        = 0U;
  s_rpi_rx_count       = 0U;
  if (reset_counters != 0U) {
    s_rpi_rx_drop_count  = 0U;
    s_rpi_rx_error_count = 0U;
  }
  s_rpi_rx_byte        = 0U;
  BSP_Critical_Exit(primask);
}

/** @brief 填写 USART6 固定 8N1 参数，波特率由调用方传入。 */
static void BSP_RpiUART_FillConfig(uint32_t baud_bps)
{
  huart6.Instance          = BSP_RPI_UART;
  huart6.Init.BaudRate     = baud_bps;
  huart6.Init.WordLength   = BSP_RPI_UART_WORD;
  huart6.Init.StopBits     = BSP_RPI_UART_STOP;
  huart6.Init.Parity       = BSP_RPI_UART_PARITY;
  huart6.Init.Mode         = BSP_RPI_UART_MODE;
  huart6.Init.HwFlowCtl    = BSP_RPI_UART_HWCTL;
  huart6.Init.OverSampling = BSP_RPI_UART_OVERSAMP;
}

/** @brief 初始化指定速率并挂接第一个接收字节。 */
static HAL_StatusTypeDef BSP_RpiUART_Start(uint32_t baud_bps)
{
  HAL_StatusTypeDef status;

  BSP_RpiUART_FillConfig(baud_bps);
  status = HAL_UART_Init(&huart6);
  if (status != HAL_OK) { return status; }
  status = HAL_UART_Receive_IT(&huart6, &s_rpi_rx_byte, 1U);
  if (status != HAL_OK) { (void)HAL_UART_DeInit(&huart6); }
  return status;
}

static void BSP_RpiUART_PushRxByteFromIsr(uint8_t byte)
{
  if (s_rpi_rx_count >= BSP_RPI_RX_BUF_SIZE) {
    s_rpi_rx_drop_count++;
    return;
  }

  s_rpi_rx_buf[s_rpi_rx_head] = byte;
  s_rpi_rx_head = (uint16_t)((s_rpi_rx_head + 1U) % BSP_RPI_RX_BUF_SIZE);
  s_rpi_rx_count++;
}

/**
 * @brief 按 `bsp_config.h` 配置初始化树莓派 MAVLink UART（USART6）。
 *
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_RpiUART_Init(void)
{
  HAL_StatusTypeDef hal_status;

  s_rpi_reconfiguring = 1U;
  s_rpi_initialized = 0U;
  BSP_RpiUART_ResetRxState(1U);
  hal_status = BSP_RpiUART_Start(BSP_RPI_UART_BAUD);
  if (hal_status == HAL_OK) { s_rpi_initialized = 1U; }
  s_rpi_reconfiguring = 0U;
  return BSP_UART_MapHalStatus(hal_status);
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
  if ((data == 0) || (length == 0U)) { return BSP_STATUS_ERROR; }
  if ((s_rpi_initialized == 0U) || (s_rpi_reconfiguring != 0U)) { return BSP_STATUS_BUSY; }
  return BSP_UART_MapHalStatus(HAL_UART_Transmit(&huart6, (uint8_t *)data, length, timeout_ms));
}

/**
 * @brief 读取 USART6 当前实际波特率。
 * @param[out] baud_bps 输出波特率，单位 bit/s。
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_RpiUART_GetBaudRate(uint32_t *baud_bps)
{
  if (baud_bps == 0) { return BSP_STATUS_ERROR; }
  if ((s_rpi_initialized == 0U) || (s_rpi_reconfiguring != 0U)) { return BSP_STATUS_BUSY; }
  *baud_bps = huart6.Init.BaudRate;
  return BSP_STATUS_OK;
}

/**
 * @brief 同步重配 USART6 波特率，失败时尝试恢复旧速率。
 * @param[in] baud_bps 目标波特率，单位 bit/s。
 * @return BSP 通用返回码；目标配置失败时即使回滚成功也返回原失败结果。
 */
BSP_Status_t BSP_RpiUART_SetBaudRate(uint32_t baud_bps)
{
  uint32_t previous_baud_bps;
  HAL_StatusTypeDef target_status;
  HAL_StatusTypeDef rollback_status;

  if (baud_bps == 0U) { return BSP_STATUS_ERROR; }
  if ((s_rpi_initialized == 0U) || (s_rpi_reconfiguring != 0U)) { return BSP_STATUS_BUSY; }
  if (huart6.Init.BaudRate == baud_bps) { return BSP_STATUS_OK; }

  previous_baud_bps = huart6.Init.BaudRate;
  s_rpi_reconfiguring = 1U;
  s_rpi_initialized = 0U;
  (void)HAL_UART_AbortReceive(&huart6);
  (void)HAL_UART_DeInit(&huart6);
  BSP_RpiUART_ResetRxState(0U);

  target_status = BSP_RpiUART_Start(baud_bps);
  if (target_status == HAL_OK) {
    s_rpi_initialized = 1U;
    s_rpi_reconfiguring = 0U;
    return BSP_STATUS_OK;
  }

  /* 目标速率启动失败时尽力回滚，函数仍返回目标配置失败，供上层结构化上报。 */
  (void)HAL_UART_AbortReceive(&huart6);
  (void)HAL_UART_DeInit(&huart6);
  BSP_RpiUART_ResetRxState(0U);
  rollback_status = BSP_RpiUART_Start(previous_baud_bps);
  if (rollback_status == HAL_OK) { s_rpi_initialized = 1U; }
  s_rpi_reconfiguring = 0U;
  return BSP_UART_MapHalStatus(target_status);
}

/**
 * @brief 从 USART6 RX 环形缓冲复制原始字节。
 * @param[out] data 输出缓冲区。
 * @param[in] max_length 最大读取长度。
 * @return 实际读取字节数。
 */
uint16_t BSP_RpiUART_Read(uint8_t *data, uint16_t max_length)
{
  uint16_t copied = 0U;

  if ((data == 0) || (max_length == 0U) || (s_rpi_initialized == 0U) ||
      (s_rpi_reconfiguring != 0U)) { return 0U; }

  while (copied < max_length) {
    uint32_t primask;

    primask = BSP_Critical_Enter();
    if (s_rpi_rx_count == 0U) {
      BSP_Critical_Exit(primask);
      break;
    }

    data[copied] = s_rpi_rx_buf[s_rpi_rx_tail];
    s_rpi_rx_tail = (uint16_t)((s_rpi_rx_tail + 1U) % BSP_RPI_RX_BUF_SIZE);
    s_rpi_rx_count--;
    BSP_Critical_Exit(primask);
    copied++;
  }

  return copied;
}

/** @brief 转发 USART6 IRQ 到 HAL，并累计硬件错误。 */
void BSP_RpiUART_IrqHandler(void)
{
  uint32_t sr;

  if ((s_rpi_initialized == 0U) || (s_rpi_reconfiguring != 0U)) { return; }
  sr = huart6.Instance->SR;
  if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U) { s_rpi_rx_error_count++; }

  HAL_UART_IRQHandler(&huart6);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != 0) && (huart->Instance == BSP_RPI_UART) &&
      (s_rpi_initialized != 0U) && (s_rpi_reconfiguring == 0U)) {
    BSP_RpiUART_PushRxByteFromIsr(s_rpi_rx_byte);
    if (HAL_UART_Receive_IT(&huart6, &s_rpi_rx_byte, 1U) != HAL_OK) { s_rpi_rx_error_count++; }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != 0) && (huart->Instance == BSP_RPI_UART) &&
      (s_rpi_initialized != 0U) && (s_rpi_reconfiguring == 0U)) {
    s_rpi_rx_error_count++;
    if (HAL_UART_Receive_IT(&huart6, &s_rpi_rx_byte, 1U) != HAL_OK) { s_rpi_rx_error_count++; }
  }
}
#endif
