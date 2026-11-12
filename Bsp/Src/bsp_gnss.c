/**
 * @file bsp_gnss.c
 * @brief BSP层GNSS原始UART transport：RX循环DMA、静态复制TX和owner硬件恢复。
 * @details 不解析NMEA/UBX、不判定定位/模块最终健康状态；阻塞重建仅由sensor owner执行。
 *          ISR只服务HAL、推进原始位置和记录计数/恢复请求，不打印、不等待或重新初始化。
 */
#include "bsp_gnss.h"
#include "bsp_config.h"
#include <string.h>

static UART_HandleTypeDef s_gnss_uart;
static DMA_HandleTypeDef hdma_gnss_rx;
static uint8_t s_rx_buf[BSP_GNSS_RX_BUF_SIZE]; /**< 主SRAM内的DMA缓冲，禁止放入CCM。 */
static uint8_t s_tx_buf[BSP_GNSS_TX_MAX_BYTES]; /**< HAL_IT持有的私有发送副本，不依赖调用者缓冲寿命。 */
static volatile uint16_t s_rx_head, s_rx_tail;
static volatile uint8_t s_rx_ovf;
static volatile uint32_t s_rx_drop_count;
static volatile uint8_t s_rx_recover_request;
static volatile uint8_t s_initialized, s_recovering, s_tx_busy;
static uint8_t s_dma_error_latched;
static uint32_t s_target_baud_bps = BSP_GNSS_UART_BAUD;
static volatile BSP_GNSS_TransportInfo_t s_transport; /**< 上电累计，warm重建不得清零。 */

#if DEBUG_GNSS_BSP_MONITOR_ENABLE
static volatile uint32_t s_dbg_usart2_irq_count       = 0U;
static volatile uint32_t s_dbg_idle_irq_count         = 0U;
static volatile uint32_t s_dbg_rx_idle_callback_count = 0U;
static volatile uint32_t s_dbg_dma_irq_count          = 0U;
static volatile uint32_t s_dbg_sr_snapshot            = 0U;
static volatile uint32_t s_dbg_cr1_snapshot           = 0U;
static volatile uint32_t s_dbg_cr3_snapshot           = 0U;
static volatile uint16_t s_dbg_dma_pos                = 0U;

/**
 * @brief 记录一次 USART2 中断和寄存器快照。
 */
void BSP_GNSS_DebugMarkUsart2Irq(uint32_t sr, uint32_t cr1, uint32_t cr3)
{
  s_dbg_usart2_irq_count++;
  s_dbg_sr_snapshot  = sr;
  s_dbg_cr1_snapshot = cr1;
  s_dbg_cr3_snapshot = cr3;
}

/**
 * @brief 记录一次 USART2 IDLE 中断。
 */
void BSP_GNSS_DebugMarkIdleIrq(void)
{
  s_dbg_idle_irq_count++;
}

/**
 * @brief 复制当前 GNSS BSP 诊断计数和 DMA 状态。
 */
void BSP_GNSS_DebugGetInfo(BSP_GNSS_DebugInfo_t *info)
{
  if (info == 0) { return; }

  info->usart2_irq_count       = s_dbg_usart2_irq_count;
  info->idle_irq_count         = s_dbg_idle_irq_count;
  info->rx_idle_callback_count = s_dbg_rx_idle_callback_count;
  info->dma_irq_count          = s_dbg_dma_irq_count;
  info->sr_snapshot            = s_dbg_sr_snapshot;
  info->cr1_snapshot           = s_dbg_cr1_snapshot;
  info->cr3_snapshot           = s_dbg_cr3_snapshot;
  info->dma_ndtr               = (hdma_gnss_rx.Instance != 0) ? (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_gnss_rx) : 0U;
  info->dma_pos                = s_dbg_dma_pos;
  info->rx_head                = s_rx_head;
  info->rx_tail                = s_rx_tail;
  info->rx_count               = BSP_GNSS_GetRxCount();
  info->rx_drop_count          = s_rx_drop_count;
}
#endif

/** @brief 返回私有句柄，仅用于HAL/MSP集成，不向上层暴露原始缓冲。 */
UART_HandleTypeDef *BSP_GNSS_GetUartHandle(void) { return &s_gnss_uart; }

/** @brief 保存并关闭全局中断，仅用于有界位置/快照复制。 */
static uint32_t BspGnss_Lock(void)
{
  uint32_t key = __get_PRIMASK();
  __disable_irq();
  return key;
}
/** @brief 只恢复进入前允许的中断状态，不无条件开中断。 */
static void BspGnss_Unlock(uint32_t key) { if (key == 0U) { __enable_irq(); } }

/** @brief caller已互斥时计算已提交环位置的未读长度，单位byte。 */
static uint16_t BspGnss_Queued(void)
{
  return (uint16_t)((s_rx_head + BSP_GNSS_RX_BUF_SIZE - s_rx_tail) % BSP_GNSS_RX_BUF_SIZE);
}

/**
 * @brief 在同一个短临界窗口观察NDTR并提交head/drop，任务轮询和HT/TC共用。
 * @note 同一NDTR位置不重复累计；不能还原被长时间屏蔽中断漏过的完整DMA圈数。
 */
static void BspGnss_UpdateRxPosition(void)
{
  uint16_t pos, advanced, free_before;
  uint32_t key = BspGnss_Lock();
  if (hdma_gnss_rx.Instance == 0) { BspGnss_Unlock(key); return; }
  pos = (uint16_t)(BSP_GNSS_RX_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_gnss_rx));
  if (pos >= BSP_GNSS_RX_BUF_SIZE) { pos = 0U; }
  advanced = (uint16_t)((pos + BSP_GNSS_RX_BUF_SIZE - s_rx_head) % BSP_GNSS_RX_BUF_SIZE);
  free_before = (uint16_t)((s_rx_tail + BSP_GNSS_RX_BUF_SIZE - s_rx_head - 1U) % BSP_GNSS_RX_BUF_SIZE);
  s_transport.rx_bytes += advanced;
  if (advanced > free_before) {
    s_rx_drop_count += (uint32_t)(advanced - free_before);
    s_rx_tail = (uint16_t)((pos + 1U) % BSP_GNSS_RX_BUF_SIZE);
    s_rx_ovf = 1U;
  }
  s_rx_head = pos;
#if DEBUG_GNSS_BSP_MONITOR_ENABLE
  s_dbg_dma_pos = pos;
#endif
  BspGnss_Unlock(key);
}

/** @brief 清除已观察到的待消费队列并只计一次丢弃，保留旧DMA观测原点。 */
static void BspGnss_DiscardQueued(void)
{
  uint32_t key = BspGnss_Lock();
  s_rx_drop_count += BspGnss_Queued();
  s_rx_tail = s_rx_head;
  s_rx_ovf = 0U;
  BspGnss_Unlock(key);
}

/** @brief 结束一笔未完成TX的失败事实；重复错误/后续重建不重复计数。 */
static void BspGnss_FailTx(void)
{
  if (s_tx_busy != 0U) { s_transport.tx_failures++; s_tx_busy = 0U; }
}

/**
 * @brief 只核对RX硬件链与HAL事实，不依赖恢复阶段或NVIC临时屏蔽状态。
 * @details Start重开中断前及AbortTx暂时关闭UART IRQ时均可使用；不读SR/DR、
 * 不推进NDTR、不消费接收字节，也不清除任何错误码。
 */
static uint8_t BspGnss_RxHardwareReady(void)
{
  return (uint8_t)(s_gnss_uart.Instance != 0 && hdma_gnss_rx.Instance != 0 &&
      (s_gnss_uart.Instance->CR1 & (USART_CR1_UE | USART_CR1_RE | USART_CR1_IDLEIE)) ==
          (USART_CR1_UE | USART_CR1_RE | USART_CR1_IDLEIE) &&
      (s_gnss_uart.Instance->CR3 & (USART_CR3_DMAR | USART_CR3_EIE)) ==
          (USART_CR3_DMAR | USART_CR3_EIE) &&
      (hdma_gnss_rx.Instance->CR & DMA_SxCR_EN) &&
      s_gnss_uart.RxState == HAL_UART_STATE_BUSY_RX &&
      s_gnss_uart.ErrorCode == HAL_UART_ERROR_NONE &&
      HAL_DMA_GetState(&hdma_gnss_rx) == HAL_DMA_STATE_BUSY &&
      HAL_DMA_GetError(&hdma_gnss_rx) == HAL_DMA_ERROR_NONE);
}

/** @brief owner正常轮询要求整个会话可运行，包含两路NVIC和已挂起恢复事实。 */
static uint8_t BspGnss_RxSessionReady(void)
{
  return (uint8_t)(s_initialized && !s_recovering && !s_rx_recover_request &&
      BspGnss_RxHardwareReady() && NVIC_GetEnableIRQ(BSP_GNSS_IRQn) &&
      NVIC_GetEnableIRQ(BSP_GNSS_RX_DMA_IRQn));
}

/** @brief 出错后只关闭本transport中断源和请求，不在ISR执行重建。 */
static void BspGnss_RequestFaultRecovery(void)
{
  s_initialized = 0U;
  s_rx_recover_request = 1U;
  BspGnss_FailTx();
  if (s_gnss_uart.Instance != 0) {
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_IDLE);
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_PE);
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_TXE);
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_TC);
    CLEAR_BIT(s_gnss_uart.Instance->CR3, USART_CR3_DMAR);
  }
  HAL_NVIC_DisableIRQ(BSP_GNSS_IRQn);
  HAL_NVIC_DisableIRQ(BSP_GNSS_RX_DMA_IRQn);
}

/**
 * @brief owner同步结束旧会话，显式清理HAL异步DMA ABORT/Lock/Callback后释放UART。
 * @note DMA BUSY同步Abort使用HAL既定5ms超时；失败保持IRQ关闭且保留NDTR观测原点供重试。
 */
static BSP_Status_t BspGnss_Stop(void)
{
  HAL_NVIC_DisableIRQ(BSP_GNSS_IRQn);
  HAL_NVIC_DisableIRQ(BSP_GNSS_RX_DMA_IRQn);
  s_initialized = 0U;
  BspGnss_FailTx();
  if (s_gnss_uart.Instance != 0) {
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_IDLE);
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_PE);
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_TXE);
    __HAL_UART_DISABLE_IT(&s_gnss_uart, UART_IT_TC);
    CLEAR_BIT(s_gnss_uart.Instance->CR3, USART_CR3_DMAR | USART_CR3_DMAT);
  }
  BspGnss_UpdateRxPosition();
  BspGnss_DiscardQueued();
  if (hdma_gnss_rx.Instance != 0) {
    if (HAL_DMA_GetState(&hdma_gnss_rx) == HAL_DMA_STATE_BUSY) {
      if (HAL_DMA_Abort(&hdma_gnss_rx) != HAL_OK) { return BSP_STATUS_ERROR; }
    }
    BspGnss_UpdateRxPosition();
    BspGnss_DiscardQueued();
    if (HAL_DMA_DeInit(&hdma_gnss_rx) != HAL_OK) { return BSP_STATUS_ERROR; }
    /* 只有NDTR确已重置才能归零软件原点；否则重试会重复累计旧前缀。 */
    s_rx_head = 0U; s_rx_tail = 0U;
  }
  if (s_gnss_uart.Instance != 0 && HAL_UART_DeInit(&s_gnss_uart) != HAL_OK) { return BSP_STATUS_ERROR; }
  HAL_NVIC_ClearPendingIRQ(BSP_GNSS_IRQn);
  HAL_NVIC_ClearPendingIRQ(BSP_GNSS_RX_DMA_IRQn);
  return BSP_STATUS_OK;
}

/** @brief owner完整挂接UART、RX DMA、HAL callback及NVIC；末尾才开放中断。 */
static BSP_Status_t BspGnss_Start(void)
{
  s_gnss_uart.Instance = BSP_GNSS_UART;
  s_gnss_uart.Init.BaudRate = s_target_baud_bps;
  s_gnss_uart.Init.WordLength = BSP_GNSS_UART_WORD;
  s_gnss_uart.Init.StopBits = BSP_GNSS_UART_STOP;
  s_gnss_uart.Init.Parity = BSP_GNSS_UART_PARITY;
  s_gnss_uart.Init.Mode = BSP_GNSS_UART_MODE;
  s_gnss_uart.Init.HwFlowCtl = BSP_GNSS_UART_HWCTL;
  s_gnss_uart.Init.OverSampling = BSP_GNSS_UART_OVERSAMP;
  if (HAL_UART_Init(&s_gnss_uart) != HAL_OK) { return BSP_STATUS_ERROR; }
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
  if (HAL_DMA_Init(&hdma_gnss_rx) != HAL_OK) { return BSP_STATUS_ERROR; }
  __HAL_LINKDMA(&s_gnss_uart, hdmarx, hdma_gnss_rx);
  s_rx_head = 0U; s_rx_tail = 0U; s_rx_ovf = 0U;
  s_dma_error_latched = 0U;
  s_rx_recover_request = 0U;
  __HAL_UART_CLEAR_OREFLAG(&s_gnss_uart);
  if (HAL_UART_Receive_DMA(&s_gnss_uart, s_rx_buf, BSP_GNSS_RX_BUF_SIZE) != HAL_OK) { return BSP_STATUS_ERROR; }
  __HAL_UART_ENABLE_IT(&s_gnss_uart, UART_IT_IDLE);
  s_initialized = 1U;
  if (!BspGnss_RxHardwareReady()) { return BSP_STATUS_ERROR; }
  HAL_NVIC_ClearPendingIRQ(BSP_GNSS_IRQn);
  HAL_NVIC_ClearPendingIRQ(BSP_GNSS_RX_DMA_IRQn);
  HAL_NVIC_EnableIRQ(BSP_GNSS_IRQn);
  /* 第一个IRQ可能立即发现接收错误；不得继续开放已被ISR停用的另一个IRQ。 */
  if (!s_initialized || s_rx_recover_request || !BspGnss_RxHardwareReady() ||
      !NVIC_GetEnableIRQ(BSP_GNSS_IRQn)) { return BSP_STATUS_ERROR; }
  HAL_NVIC_EnableIRQ(BSP_GNSS_RX_DMA_IRQn);
  /* DMA错误也可在重开瞬间同步交付，只有整条接收链仍健康才报告本次成功。 */
  if (!s_initialized || s_rx_recover_request || !BspGnss_RxHardwareReady() ||
      !NVIC_GetEnableIRQ(BSP_GNSS_IRQn) || !NVIC_GetEnableIRQ(BSP_GNSS_RX_DMA_IRQn)) { return BSP_STATUS_ERROR; }
  return BSP_STATUS_OK;
}

/** @brief owner在不影响其他外设的条件下重建；失败由后续限速恢复继续处理。 */
static BSP_Status_t BspGnss_Rebuild(void)
{
  BSP_Status_t result;
  s_recovering = 1U;
  result = BspGnss_Stop();
  if (result == BSP_STATUS_OK) { result = BspGnss_Start(); }
  if (result != BSP_STATUS_OK) { BspGnss_RequestFaultRecovery(); }
  s_recovering = 0U;
  return result;
}

/** @brief 调度前初始化或owner修复当前会话；不清本次上电累计计数。 */
BSP_Status_t BSP_GNSS_Init(void)
{
  if (s_tx_busy) { return BSP_STATUS_BUSY; }
  if (BspGnss_RxSessionReady()) { return BSP_STATUS_OK; }
  return BspGnss_Rebuild();
}
/** @brief owner释放硬件，未完成TX按失败结束；累计事实保留。 */
BSP_Status_t BSP_GNSS_DeInit(void)
{
  BSP_Status_t result;
  s_recovering = 1U;
  result = BspGnss_Stop();
  if (result == BSP_STATUS_OK) { s_rx_recover_request = 0U; }
  s_recovering = 0U;
  return result;
}
/** @brief owner切换波特率；TX未完成时拒绝，不截断配置帧。 */
BSP_Status_t BSP_GNSS_SetBaudRate(uint32_t baud_bps)
{
  if (baud_bps < 4800U || baud_bps > 921600U) { return BSP_STATUS_ERROR; }
  if (s_tx_busy) { return BSP_STATUS_BUSY; }
  s_target_baud_bps = baud_bps;
  if (BspGnss_RxSessionReady() && s_gnss_uart.Init.BaudRate == baud_bps) { return BSP_STATUS_OK; }
  return BspGnss_Rebuild();
}
/** @brief owner修复当前目标波特率的接收会话，只有成功才推进recoveries。 */
BSP_Status_t BSP_GNSS_RecoverRx(void)
{
  BSP_Status_t result = BspGnss_Rebuild();
  if (result == BSP_STATUS_OK) { s_transport.recoveries++; }
  return result;
}
/** @brief 仅登记恢复请求，ISR不访问总线重初始化。 */
void BSP_GNSS_RequestRecoverRx(void) { s_rx_recover_request = 1U; }
/** @brief owner原子取走请求标志，实际恢复由其自行限速执行。 */
uint8_t BSP_GNSS_ConsumeRecoverRxRequest(void)
{
  uint8_t pending;
  uint32_t key = BspGnss_Lock();
  pending = s_rx_recover_request; s_rx_recover_request = 0U;
  BspGnss_Unlock(key);
  return pending;
}

/**
 * @brief owner将发送数据复制到静态槽，再以HAL IT挂接；仅短暂屏蔽本transport的两个IRQ。
 * @note 不关全局中断，不等待物理完成；实际TC由UART IRQ包装器确认，避免重复定义全局HAL回调。
 */
BSP_Status_t BSP_GNSS_Transmit(const uint8_t *data, uint16_t len)
{
  uint32_t uart_enabled, dma_enabled;
  HAL_StatusTypeDef result;
  BSP_Status_t status;
  if (data == 0 || len == 0U || len > sizeof(s_tx_buf)) { return BSP_STATUS_ERROR; }
  uart_enabled = NVIC_GetEnableIRQ(BSP_GNSS_IRQn);
  dma_enabled = NVIC_GetEnableIRQ(BSP_GNSS_RX_DMA_IRQn);
  HAL_NVIC_DisableIRQ(BSP_GNSS_IRQn); HAL_NVIC_DisableIRQ(BSP_GNSS_RX_DMA_IRQn);
  if (!uart_enabled || !s_initialized || s_recovering || s_rx_recover_request || s_gnss_uart.Instance == 0 ||
      (s_gnss_uart.Instance->CR1 & (USART_CR1_UE | USART_CR1_TE)) != (USART_CR1_UE | USART_CR1_TE)) {
    status = BSP_STATUS_ERROR;
  } else if (s_tx_busy || s_gnss_uart.gState != HAL_UART_STATE_READY) {
    status = BSP_STATUS_BUSY;
  } else {
    memcpy(s_tx_buf, data, len);
    s_tx_busy = 1U;
    result = HAL_UART_Transmit_IT(&s_gnss_uart, s_tx_buf, len);
    if (result == HAL_OK) { status = BSP_STATUS_OK; }
    else {
      s_tx_busy = 0U;
      if (result != HAL_BUSY) { s_transport.tx_failures++; }
      status = (result == HAL_BUSY) ? BSP_STATUS_BUSY : BSP_STATUS_ERROR;
    }
  }
  /* 开放第一个IRQ可能立即进入错误处理；每一步重新核对，不能把故障关闭的另一个IRQ再打开。 */
  if (dma_enabled && s_initialized && !s_rx_recover_request) { HAL_NVIC_EnableIRQ(BSP_GNSS_RX_DMA_IRQn); }
  if (uart_enabled && s_initialized && !s_rx_recover_request) { HAL_NVIC_EnableIRQ(BSP_GNSS_IRQn); }
  return status;
}

/**
 * @brief owner中止独立IT发送，RX故障才退回完整重建请求。
 * @details 本项目TX不用DMA；拒绝意外DMAT避免HAL中止其它通道。
 * HAL_UART_AbortTransmit在此路径只清TX中断/计数/gState，不访问RX DMA或SR/DR。
 */
BSP_Status_t BSP_GNSS_AbortTx(void)
{
  uint32_t uart_enabled=NVIC_GetEnableIRQ(BSP_GNSS_IRQn);
  BSP_Status_t status=BSP_STATUS_ERROR;
  HAL_NVIC_DisableIRQ(BSP_GNSS_IRQn);
  if(s_initialized && !s_recovering && !s_rx_recover_request && BspGnss_RxHardwareReady() &&
      !(s_gnss_uart.Instance->CR3 & USART_CR3_DMAT)) {
    if(HAL_UART_AbortTransmit(&s_gnss_uart)==HAL_OK && BspGnss_RxHardwareReady()) {
      BspGnss_FailTx();
      status=BSP_STATUS_OK;
    }
  }
  if(status!=BSP_STATUS_OK) { BspGnss_RequestFaultRecovery(); }
  /* 恢复使能可能立即交付新的UART错误；不得把其停收结果当作成功。 */
  if(uart_enabled && s_initialized && !s_rx_recover_request) { HAL_NVIC_EnableIRQ(BSP_GNSS_IRQn); }
  if(!s_initialized || s_recovering || s_rx_recover_request || !BspGnss_RxHardwareReady()) { status=BSP_STATUS_ERROR; }
  return status;
}

/** @brief 复制状态不推进接收、不读取SR/DR；未初始化也能返回已发生的累计事实。 */
BSP_Status_t BSP_GNSS_CopyTransportInfo(BSP_GNSS_TransportInfo_t *out)
{
  uint32_t key;
  if (out == 0) { return BSP_STATUS_ERROR; }
  key = BspGnss_Lock();
  *out = s_transport;
  out->baud_bps = s_gnss_uart.Init.BaudRate;
  out->rx_drop_bytes = s_rx_drop_count;
  out->queued_bytes = BspGnss_Queued();
  out->ndtr = (hdma_gnss_rx.Instance != 0) ? (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_gnss_rx) : 0U;
  out->initialized = s_initialized; out->tx_busy = s_tx_busy;
  BspGnss_Unlock(key);
  return BSP_STATUS_OK;
}
/** @brief owner复制缓存原始字节，既不解析协议也不重新采样硬件位置。 */
uint16_t BSP_GNSS_GetRxData(uint8_t *dst, uint16_t max_len)
{
  uint16_t count = 0U;
  uint32_t key;
  if (dst == 0 || max_len == 0U) { return 0U; }
  key = BspGnss_Lock();
  while (s_rx_tail != s_rx_head && count < max_len) {
    dst[count++] = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % BSP_GNSS_RX_BUF_SIZE);
  }
  BspGnss_Unlock(key);
  return count;
}
/** @brief 只读未消费字节数量，不推进NDTR位置。 */
uint16_t BSP_GNSS_GetRxCount(void)
{
  uint16_t count;
  uint32_t key = BspGnss_Lock(); count = BspGnss_Queued(); BspGnss_Unlock(key); return count;
}
/** @brief 只读本次上电累计丢字节数量。 */
uint32_t BSP_GNSS_GetDropCount(void)
{
  uint32_t count;
  uint32_t key = BspGnss_Lock(); count = s_rx_drop_count; BspGnss_Unlock(key); return count;
}
/** @brief owner轮询与中断共用位置推进；发现停收只请求恢复，不在此重建。 */
void BSP_GNSS_RxIdleCallback(uint16_t dummy)
{
  (void)dummy;
  if (!s_recovering && !BspGnss_RxSessionReady()) {
    BspGnss_RequestFaultRecovery();
  }
#if DEBUG_GNSS_BSP_MONITOR_ENABLE
  s_dbg_rx_idle_callback_count++;
#endif
  BspGnss_UpdateRxPosition();
}

/** @brief UART ISR服务HAL；TC实际完成后释放TX，错误时静默源并交owner重建。 */
void BSP_GNSS_UartIrqHandler(void)
{
  uint32_t sr, cr1;
  uint8_t completed;
  if (s_gnss_uart.Instance == 0) { return; }
  sr = s_gnss_uart.Instance->SR; cr1 = s_gnss_uart.Instance->CR1;
  completed = (uint8_t)((sr & USART_SR_TC) && (cr1 & USART_CR1_TCIE));
#if DEBUG_GNSS_BSP_MONITOR_ENABLE
  BSP_GNSS_DebugMarkUsart2Irq(sr, cr1, s_gnss_uart.Instance->CR3);
#endif
  if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    s_transport.uart_errors++;
    BspGnss_UpdateRxPosition();
    HAL_UART_IRQHandler(&s_gnss_uart);
    BspGnss_RequestFaultRecovery();
    return;
  }
  if ((sr & USART_SR_IDLE) && (cr1 & USART_CR1_IDLEIE)) {
    __HAL_UART_CLEAR_IDLEFLAG(&s_gnss_uart);
#if DEBUG_GNSS_BSP_MONITOR_ENABLE
    BSP_GNSS_DebugMarkIdleIrq();
#endif
    BspGnss_UpdateRxPosition();
  }
  HAL_UART_IRQHandler(&s_gnss_uart);
  if (completed && s_tx_busy && s_gnss_uart.gState == HAL_UART_STATE_READY) {
    s_tx_busy = 0U; s_transport.tx_packets++;
  }
}
/** @brief RX DMA HT/TC/错误ISR；HAL回调后统一推进原始位置并锁存错误。 */
void BSP_GNSS_DmaIrqHandler(void)
{
  if (hdma_gnss_rx.Instance == 0) { return; }
#if DEBUG_GNSS_BSP_MONITOR_ENABLE
  s_dbg_dma_irq_count++;
#endif
  HAL_DMA_IRQHandler(&hdma_gnss_rx);
  BspGnss_UpdateRxPosition();
  if (HAL_DMA_GetError(&hdma_gnss_rx) != HAL_DMA_ERROR_NONE || (s_gnss_uart.ErrorCode & HAL_UART_ERROR_DMA)) {
    if (!s_dma_error_latched) { s_dma_error_latched = 1U; s_transport.dma_errors++; }
    BspGnss_RequestFaultRecovery();
  }
}
