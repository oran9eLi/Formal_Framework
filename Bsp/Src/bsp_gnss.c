/**
 * @file bsp_gnss.c
 * @brief Implement USART2 DMA circular reception for the GNSS module.
 */

#include "debug_config.h"
#include "bsp_gnss.h"
#include "bsp_config.h"
#include "stm32f4xx_hal.h"

static UART_HandleTypeDef s_gnss_uart;
static DMA_HandleTypeDef hdma_gnss_rx;

/**
 * @brief Return the private GNSS UART handle for interrupt and MSP use only.
 */
UART_HandleTypeDef *BSP_GNSS_GetUartHandle(void)
{
  return &s_gnss_uart;
}

static uint8_t s_rx_buf[BSP_GNSS_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;
static volatile uint8_t s_rx_ovf = 0U;
static volatile uint32_t s_rx_drop_count = 0U;

#if DEBUG_GNSS_BSP_MONITOR_ENABLE
static volatile uint32_t s_dbg_usart2_irq_count = 0U;
static volatile uint32_t s_dbg_idle_irq_count = 0U;
static volatile uint32_t s_dbg_rx_idle_callback_count = 0U;
static volatile uint32_t s_dbg_dma_irq_count = 0U;
static volatile uint32_t s_dbg_sr_snapshot = 0U;
static volatile uint32_t s_dbg_cr1_snapshot = 0U;
static volatile uint32_t s_dbg_cr3_snapshot = 0U;
static volatile uint16_t s_dbg_dma_pos = 0U;

/**
 * @brief Record one USART2 interrupt and its register snapshot for diagnostics.
 */
void BSP_GNSS_DebugMarkUsart2Irq(uint32_t sr, uint32_t cr1, uint32_t cr3)
{
  s_dbg_usart2_irq_count++;
  s_dbg_sr_snapshot = sr;
  s_dbg_cr1_snapshot = cr1;
  s_dbg_cr3_snapshot = cr3;
}

/**
 * @brief Record one detected USART2 IDLE interrupt for diagnostics.
 */
void BSP_GNSS_DebugMarkIdleIrq(void)
{
  s_dbg_idle_irq_count++;
}

/**
 * @brief Copy the current GNSS BSP diagnostic counters and DMA state.
 */
void BSP_GNSS_DebugGetInfo(BSP_GNSS_DebugInfo_t *info)
{
  if (info == 0) {
    return;
  }

  info->usart2_irq_count = s_dbg_usart2_irq_count;
  info->idle_irq_count = s_dbg_idle_irq_count;
  info->rx_idle_callback_count = s_dbg_rx_idle_callback_count;
  info->dma_irq_count = s_dbg_dma_irq_count;
  info->sr_snapshot = s_dbg_sr_snapshot;
  info->cr1_snapshot = s_dbg_cr1_snapshot;
  info->cr3_snapshot = s_dbg_cr3_snapshot;
  info->dma_ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_gnss_rx);
  info->dma_pos = s_dbg_dma_pos;
  info->rx_head = s_rx_head;
  info->rx_tail = s_rx_tail;
  info->rx_count = BSP_GNSS_GetRxCount();
  info->rx_drop_count = s_rx_drop_count;
}
#endif


/**
 * @brief Configure USART2 and circular DMA reception for the GNSS module.
 */
BSP_Status_t BSP_GNSS_Init(void)
{
  s_gnss_uart.Instance = BSP_GNSS_UART;
  s_gnss_uart.Init.BaudRate = BSP_GNSS_UART_BAUD;
  s_gnss_uart.Init.WordLength = BSP_GNSS_UART_WORD;
  s_gnss_uart.Init.StopBits = BSP_GNSS_UART_STOP;
  s_gnss_uart.Init.Parity = BSP_GNSS_UART_PARITY;
  s_gnss_uart.Init.Mode = BSP_GNSS_UART_MODE;
  s_gnss_uart.Init.HwFlowCtl = BSP_GNSS_UART_HWCTL;
  s_gnss_uart.Init.OverSampling = BSP_GNSS_UART_OVERSAMP;
  if (HAL_UART_Init(&s_gnss_uart) != HAL_OK)
  {
    return BSP_STATUS_ERROR;
  }

  hdma_gnss_rx.Instance = BSP_GNSS_RX_DMA_STREAM;
  hdma_gnss_rx.Init.Channel = BSP_GNSS_RX_DMA_CHANNEL;
  hdma_gnss_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_gnss_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_gnss_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_gnss_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_gnss_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_gnss_rx.Init.Mode = DMA_CIRCULAR;
  hdma_gnss_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
  hdma_gnss_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_gnss_rx) != HAL_OK)
  {
    return BSP_STATUS_ERROR;
  }

  __HAL_LINKDMA(&s_gnss_uart, hdmarx, hdma_gnss_rx);

  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_ovf = 0U;
  s_rx_drop_count = 0U;

  if (HAL_UART_Receive_DMA(&s_gnss_uart, s_rx_buf, BSP_GNSS_RX_BUF_SIZE) != HAL_OK)
  {
    return BSP_STATUS_ERROR;
  }
  __HAL_UART_ENABLE_IT(&s_gnss_uart, UART_IT_IDLE);
  return BSP_STATUS_OK;
}




/**
 * @brief Release the GNSS UART and DMA so the peripheral consumes no resources.
 */
BSP_Status_t BSP_GNSS_DeInit(void)
{
  __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_IDLE);
  (void)HAL_UART_DMAStop(&s_gnss_uart);
  (void)HAL_DMA_DeInit(&hdma_gnss_rx);

  if (HAL_UART_DeInit(&s_gnss_uart) != HAL_OK)
  {
    return BSP_STATUS_ERROR;
  }

  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_ovf = 0U;
  return BSP_STATUS_OK;
}

/**
 * @brief Recover the GNSS UART and DMA receive path after an error.
 */
BSP_Status_t BSP_GNSS_RecoverRx(void)
{
  volatile uint32_t tmp;

  __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_IDLE);

  tmp = s_gnss_uart.Instance->SR;
  tmp = s_gnss_uart.Instance->DR;
  (void)tmp;

  s_gnss_uart.ErrorCode = HAL_UART_ERROR_NONE;

  (void)HAL_UART_DMAStop(&s_gnss_uart);

  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_ovf = 0U;

  if (HAL_UART_Receive_DMA(&s_gnss_uart, s_rx_buf, BSP_GNSS_RX_BUF_SIZE) != HAL_OK)
  {
    return BSP_STATUS_ERROR;
  }
  __HAL_UART_ENABLE_IT(&s_gnss_uart, UART_IT_IDLE);
  return BSP_STATUS_OK;
}

/**
 * @brief Copy available GNSS bytes from the BSP ring buffer to the caller.
 */
uint16_t BSP_GNSS_GetRxData(uint8_t *dst, uint16_t max_len)
{
  uint16_t count = 0U;
  uint32_t primask;

  if (dst == 0 || max_len == 0U) {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  while ((s_rx_tail != s_rx_head) && (count < max_len)) {
    dst[count] = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % BSP_GNSS_RX_BUF_SIZE);
    count++;
  }

  if (primask == 0U) {
    __enable_irq();
  }

  return count;
}

/**
 * @brief Return the number of unread bytes in the GNSS receive ring.
 */
uint16_t BSP_GNSS_GetRxCount(void)
{
  uint16_t count;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();

  count = (s_rx_head >= s_rx_tail)
        ? (uint16_t)(s_rx_head - s_rx_tail)
        : (uint16_t)(BSP_GNSS_RX_BUF_SIZE - s_rx_tail + s_rx_head);

  if (primask == 0U) {
    __enable_irq();
  }

  return count;
}

/**
 * @brief Return the cumulative number of GNSS bytes dropped on ring overflow.
 */
uint32_t BSP_GNSS_GetDropCount(void)
{
  uint32_t count;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  count = s_rx_drop_count;
  if (primask == 0U) {
    __enable_irq();
  }

  return count;
}

/**
 * @brief Advance the receive ring when UART IDLE reports newly arrived DMA bytes.
 */
void BSP_GNSS_RxIdleCallback(uint16_t dummy)
{
  uint16_t pos;
  uint16_t advanced;
  uint16_t free_before;

  (void)dummy;

#if DEBUG_GNSS_BSP_MONITOR_ENABLE
  s_dbg_rx_idle_callback_count++;
#endif

  pos = (uint16_t)(BSP_GNSS_RX_BUF_SIZE -
                   (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_gnss_rx));

  if (pos >= BSP_GNSS_RX_BUF_SIZE) {
    pos = 0U;
  }

#if DEBUG_GNSS_BSP_MONITOR_ENABLE
  s_dbg_dma_pos = pos;
#endif

  /*
   * Detect circular DMA overflow. The DMA write index advancing past the
   * unread tail means the oldest unread bytes were overwritten. Count the lost
   * bytes (drop_count) and apply a drop-oldest policy by keeping the tail one
   * slot behind the newest data, instead of silently overwriting.
   */
  advanced = (uint16_t)((pos - s_rx_head + BSP_GNSS_RX_BUF_SIZE) %
                        BSP_GNSS_RX_BUF_SIZE);
  free_before = (uint16_t)((s_rx_tail - s_rx_head - 1U + BSP_GNSS_RX_BUF_SIZE) %
                           BSP_GNSS_RX_BUF_SIZE);

  if (advanced > free_before) {
    s_rx_drop_count += (uint32_t)(advanced - free_before);
    s_rx_tail = (uint16_t)((pos + 1U) % BSP_GNSS_RX_BUF_SIZE);
    s_rx_ovf = 1U;
  }

  s_rx_head = pos;
}

/**
 * @brief Handle GNSS DMA receive interrupts and maintain the circular receive path.
 */
void BSP_GNSS_DmaIrqHandler(void)
{
#if DEBUG_GNSS_BSP_MONITOR_ENABLE
  s_dbg_dma_irq_count++;
#endif

  HAL_DMA_IRQHandler(&hdma_gnss_rx);
}
