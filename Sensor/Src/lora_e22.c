/**
 * @file lora_e22.c
 * @brief Implement typed E22 driver on top of BSP_LoRa and MAVLink parser.
 */

#include "lora_e22.h"
#include "bsp_lora.h"
#include "bsp_time.h"
#include "stm32f4xx.h"

#if defined(__CC_ARM)
/*
 * ARMCC5 must use MAVLink's field-by-field wire encoder.  This avoids
 * depending on packed C structure layout and packed member pointer types.
 */
#define MAVLINK_ALIGNED_FIELDS   0
#define MAVLINK_COMM_NUM_BUFFERS 1
#pragma diag_suppress 66
#endif

#include "common/mavlink.h"
#if defined(__CC_ARM)
#pragma diag_default 66
#endif
#include <string.h>

/* 接收帧有界队列：一个 comm 周期内可能解析出多帧，必须全部入队由 comm 任务
   排空消费。单帧缓冲会在多帧到达时只保留最后一帧，导致低频字段长期不刷新。 */
#define LORA_RX_QUEUE_LEN 8U
/* 单次 Service 最多处理的 RX 字节数，避免极端积压独占 comm 周期。 */
#define LORA_E22_RX_SERVICE_BYTE_BUDGET 128U
static Lora_RxFrame_t s_rx_queue[LORA_RX_QUEUE_LEN];
static volatile uint16_t s_rx_q_head;
static volatile uint16_t s_rx_q_tail;
static volatile uint16_t s_rx_q_count;

static mavlink_status_t s_parse_status;
static mavlink_message_t s_parse_msg;

static uint32_t s_last_rx_ms;
static uint32_t s_rx_frame_count;
static uint32_t s_tx_frame_count;
static uint32_t s_tx_busy_count;
static uint32_t s_crc_error_count;
static uint32_t s_send_error_count;
static uint32_t s_parse_error_count;
static uint32_t s_rx_byte_count;
static uint32_t s_rx_drop_count;
static uint32_t s_last_tx_ms;
static uint32_t s_last_ready_ms;
static uint32_t s_last_msg_id;
static uint8_t s_initialized;
static uint8_t s_present; /* 模块是否在位(配置寄存器读回握手判定) */
static volatile uint8_t s_reinit_request;

/* Non-blocking transmit state machine, advanced from Lora_E22_Service().
   The comm task never blocks on the air interface: a frame is staged here,
   the machine waits (without spinning) for AUX to go idle, starts a USART3
   TX DMA, then returns to IDLE when the DMA + UART transfer completes. */
#define LORA_E22_BITS_PER_BYTE          10U
#define LORA_E22_TIMEOUT_MIN_MS         20U
#define LORA_E22_AIR_TIMEOUT_MARGIN_MS  200U
#define LORA_E22_UART_TIMEOUT_MARGIN_MS 100U

typedef enum {
  LORA_TX_IDLE = 0,
  LORA_TX_WAIT_AUX,
  LORA_TX_SENDING
} Lora_TxState_t;

static Lora_TxState_t s_tx_state;
/* Owner: comm task only. Staged TX frame; must not be re-encoded until the
   in-flight frame returns to LORA_TX_IDLE (see Lora_E22_Send / Lora_E22_TxStep). */
static uint8_t s_tx_pending[LORA_E22_TX_BUF_SIZE];
static uint16_t s_tx_pending_len;
static uint32_t s_tx_state_ms;

static uint32_t Lora_E22_CalcTimeoutMs(uint16_t length_bytes, uint32_t bitrate_bps, uint32_t margin_ms);
static uint32_t Lora_E22_GetAuxWaitTimeoutMs(void);
static uint32_t Lora_E22_GetUartDmaTimeoutMs(uint16_t length_bytes);
static void Lora_E22_ResetRuntimeState(uint32_t now_ms);
static void Lora_E22_TxStep(uint32_t now_ms);

static uint8_t Lora_E22_RecordAuxReady(uint32_t now_ms)
{
  if (BSP_LoRa_IsReady() != 0U) {
    s_last_ready_ms = now_ms;
    return 1U;
  }
  return 0U;
}

static void Lora_E22_ResetRuntimeState(uint32_t now_ms)
{
  memset(s_rx_queue, 0, sizeof(s_rx_queue));
  s_rx_q_head = 0U;
  s_rx_q_tail = 0U;
  s_rx_q_count = 0U;
  memset(&s_parse_status, 0, sizeof(s_parse_status));
  memset(&s_parse_msg, 0, sizeof(s_parse_msg));

  s_last_rx_ms        = 0U;
  s_rx_frame_count    = 0U;
  s_tx_frame_count    = 0U;
  s_tx_busy_count     = 0U;
  s_crc_error_count   = 0U;
  s_send_error_count  = 0U;
  s_parse_error_count = 0U;
  s_rx_byte_count     = 0U;
  s_rx_drop_count     = 0U;
  s_last_tx_ms        = 0U;
  s_last_ready_ms     = now_ms;
  s_last_msg_id       = 0U;
  s_tx_state          = LORA_TX_IDLE;
  s_tx_pending_len    = 0U;
  s_tx_state_ms       = 0U;
  s_initialized       = 1U;
}

Lora_Result_t Lora_E22_Init(void)
{
  uint32_t start_ms;
  uint32_t ready_ms;

  s_initialized = 0U;
  s_present     = 0U;
  BSP_LoRa_SetMode(0U); /* normal mode M0=0 M1=0 */
  start_ms = BSP_Time_GetTickMs();
  while (BSP_LoRa_IsReady() == 0U) {
    /* AUX 下拉：无模块时恒低，超时即判定未接入并让 init 失败。 */
    if ((uint32_t)(BSP_Time_GetTickMs() - start_ms) > 500U) { return LORA_RESULT_BUSY; }
  }

  ready_ms = BSP_Time_GetTickMs();
  Lora_E22_ResetRuntimeState(ready_ms);

  /* 在位检测(基于 AUX 引脚)：E22 的 AUX 为推挽输出，模块在位且就绪时主动把 AUX 拉高，
     BSP 把 AUX 配成内部下拉输入。能走出上面的等待环就说明 AUX 已被模块驱动为高，即模块
     在位；若模块未接入则 AUX 下拉恒低，等待环超时返回 BUSY、s_present 保持 0。
     这里直接判为在位，不再额外做 30ms 采样——采样窗口若撞上模块瞬时忙(AUX 暂低)会把
     在位模块误判为未接入，而 ResetRuntimeState 已先置 s_initialized=1，Service 的补判
     分支以 s_initialized==0 为前提，会导致 s_present 永久卡 0、收发被整体关闭。 */
  s_present = 1U;

  return LORA_RESULT_OK;
}

/* 模块是否在位。 */
uint8_t Lora_E22_IsPresent(void)
{
  return s_present;
}

void Lora_E22_RequestReinit(void)
{
  s_reinit_request = 1U;
}

Lora_Result_t Lora_E22_Service(uint32_t now_ms)
{
  uint8_t temp[128];
  uint16_t available;
  uint16_t received;
  uint16_t rx_budget = LORA_E22_RX_SERVICE_BYTE_BUDGET;
  uint16_t i;

  if (s_initialized == 0U) {
    if (Lora_E22_RecordAuxReady(now_ms) == 0U) { return LORA_RESULT_IO_ERROR; }
    s_present = 1U;
    Lora_E22_ResetRuntimeState(now_ms);
  } else if (s_present == 0U) {
    /* 已初始化但在位标志意外落在 0(如 init 撞上瞬时忙的历史路径)：运行期一旦 AUX 就绪
       即补判为在位，避免一次性闩锁把收发永久关闭。 */
    if (BSP_LoRa_IsReady() != 0U) { s_present = 1U; }
  }

  if (s_reinit_request != 0U) {
    BSP_LoRa_SetMode(0U);
    if (BSP_LoRa_IsReady() != 0U) {
      s_reinit_request = 0U;
      s_present = 1U;
      Lora_E22_ResetRuntimeState(now_ms);
    }
  } else {
    (void)Lora_E22_RecordAuxReady(now_ms);
  }

  while ((rx_budget != 0U) && ((available = BSP_LoRa_GetRxCount()) > 0U)) {
    uint16_t chunk_len = (available > sizeof(temp)) ? (uint16_t)sizeof(temp) : available;
    if (chunk_len > rx_budget) { chunk_len = rx_budget; }

    received = BSP_LoRa_GetRxData(temp, chunk_len);

    if (received == 0U) { break; }
    rx_budget = (received >= rx_budget) ? 0U : (uint16_t)(rx_budget - received);
    s_rx_byte_count += received;

    for (i = 0U; i < received; i++) {
      uint8_t parse_error_before = s_parse_status.parse_error;

      if (mavlink_parse_char(MAVLINK_COMM_0, temp[i], &s_parse_msg, &s_parse_status) != 0) {
        /* --- complete MAVLink frame received: 入队，不覆盖 --- */
        uint32_t primask;
        Lora_RxFrame_t *slot;

        primask = __get_PRIMASK();
        __disable_irq();

        if (s_rx_q_count >= LORA_RX_QUEUE_LEN) {
          s_rx_q_tail = (uint16_t)((s_rx_q_tail + 1U) % LORA_RX_QUEUE_LEN);
          s_rx_q_count--;
          s_rx_drop_count++;
        }

        slot               = &s_rx_queue[s_rx_q_head];
        slot->frame_len    = s_parse_msg.len + MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES;
        slot->system_id    = s_parse_msg.sysid;
        slot->component_id = s_parse_msg.compid;
        slot->sequence     = s_parse_msg.seq;
        slot->payload_len  = s_parse_msg.len;
        slot->msg_id       = s_parse_msg.msgid;
        memcpy(slot->data, _MAV_PAYLOAD(&s_parse_msg), s_parse_msg.len);
        s_rx_q_head = (uint16_t)((s_rx_q_head + 1U) % LORA_RX_QUEUE_LEN);
        s_rx_q_count++;

        s_rx_frame_count++;
        s_last_rx_ms  = now_ms;
        s_last_msg_id = s_parse_msg.msgid;

        if (primask == 0U) { __enable_irq(); }
      }

      if (s_parse_status.parse_error != parse_error_before) {
        uint8_t delta = (uint8_t)(s_parse_status.parse_error - parse_error_before);
        s_parse_error_count += delta;
        s_crc_error_count += delta;
      }
    }
  }

  Lora_E22_TxStep(now_ms);
  return LORA_RESULT_OK;
}

Lora_Result_t Lora_E22_CopyRxFrame(Lora_RxFrame_t *out)
{
  uint32_t primask;

  if (out == 0) { return LORA_RESULT_INVALID_PARAM; }

  primask = __get_PRIMASK();
  __disable_irq();

  if (s_rx_q_count == 0U) {
    if (primask == 0U) { __enable_irq(); }
    return LORA_RESULT_NO_DATA;
  }

  *out        = s_rx_queue[s_rx_q_tail];
  s_rx_q_tail = (uint16_t)((s_rx_q_tail + 1U) % LORA_RX_QUEUE_LEN);
  s_rx_q_count--;

  if (primask == 0U) { __enable_irq(); }
  return LORA_RESULT_OK;
}

Lora_State_t Lora_E22_GetState(uint32_t now_ms, uint32_t offline_timeout_ms)
{
  if (s_initialized == 0U) { return LORA_STATE_NOT_READY; }
  /* LoRa 模块状态按 AUX ready 判定，远端数据是否有效由 RemoteTelemetry 负责判定。 */
  if (Lora_E22_RecordAuxReady(now_ms) != 0U) { return LORA_STATE_ONLINE; }
  if (s_last_ready_ms == 0U) { return LORA_STATE_NOT_READY; }
  if ((uint32_t)(now_ms - s_last_ready_ms) <= offline_timeout_ms) { return LORA_STATE_ONLINE; }
  return LORA_STATE_OFFLINE;
}

static uint32_t Lora_E22_CalcTimeoutMs(uint16_t length_bytes, uint32_t bitrate_bps, uint32_t margin_ms)
{
  uint32_t bits;
  uint32_t transfer_ms;
  uint32_t timeout_ms;

  if ((length_bytes == 0U) || (bitrate_bps == 0U)) { return LORA_E22_TIMEOUT_MIN_MS; }

  bits        = (uint32_t)length_bytes * LORA_E22_BITS_PER_BYTE;
  transfer_ms = ((bits * 1000U) + bitrate_bps - 1U) / bitrate_bps;
  timeout_ms  = transfer_ms + margin_ms;

  return (timeout_ms < LORA_E22_TIMEOUT_MIN_MS) ? LORA_E22_TIMEOUT_MIN_MS : timeout_ms;
}

static uint32_t Lora_E22_GetAuxWaitTimeoutMs(void)
{
  return Lora_E22_CalcTimeoutMs(LORA_E22_TX_BUF_SIZE, BSP_LoRa_GetAirBps(), LORA_E22_AIR_TIMEOUT_MARGIN_MS);
}

static uint32_t Lora_E22_GetUartDmaTimeoutMs(uint16_t length_bytes)
{
  return Lora_E22_CalcTimeoutMs(length_bytes, BSP_LoRa_GetUartBaud(), LORA_E22_UART_TIMEOUT_MARGIN_MS);
}

/* Advance the transmit state machine. Called every comm cycle from
   Lora_E22_Service() and once from Lora_E22_Send(). Never blocks. */
static void Lora_E22_TxStep(uint32_t now_ms)
{
  uint32_t timeout_ms;

  switch (s_tx_state) {
    case LORA_TX_WAIT_AUX:
      if (BSP_LoRa_IsBusy() == 0U) {
        int32_t r = BSP_LoRa_StartSend(s_tx_pending, s_tx_pending_len);
        if (r == 0) {
          s_tx_state    = LORA_TX_SENDING;
          s_tx_state_ms = now_ms;
        } else if (r < 0) {
          s_send_error_count++;
          s_tx_state = LORA_TX_IDLE;
        }
        /* r == 1: DMA still busy, stay and retry (bounded below). */
      }
      timeout_ms = Lora_E22_GetAuxWaitTimeoutMs();
      if ((s_tx_state == LORA_TX_WAIT_AUX) && ((uint32_t)(now_ms - s_tx_state_ms) > timeout_ms)) {
        s_tx_busy_count++;
        s_tx_state = LORA_TX_IDLE;
      }
      break;

    case LORA_TX_SENDING:
      if (BSP_LoRa_IsTxBusy() == 0U) {
        s_tx_frame_count++;
        s_last_tx_ms = now_ms;
        s_tx_state   = LORA_TX_IDLE;
      } else if ((uint32_t)(now_ms - s_tx_state_ms) > Lora_E22_GetUartDmaTimeoutMs(s_tx_pending_len)) {
        BSP_LoRa_AbortTx();
        s_send_error_count++;
        s_tx_state = LORA_TX_IDLE;
      }
      break;

    case LORA_TX_IDLE:
    default:
      break;
  }
}

Lora_Result_t Lora_E22_Send(const uint8_t *data, uint16_t len)
{
  if (data == 0 || len == 0U || len > LORA_E22_TX_BUF_SIZE) { return LORA_RESULT_INVALID_PARAM; }

  /* 模块未接入时不发送，发送计数保持为 0。 */
  if (s_present == 0U) { return LORA_RESULT_BUSY; }

  /* One frame in flight at a time. While a frame is staged or sending,
     report BUSY so the MAVLink scheduler retries after its short backoff
     instead of overwriting the in-flight frame. */
  if (s_tx_state != LORA_TX_IDLE) {
    s_tx_busy_count++;
    return LORA_RESULT_BUSY;
  }

  memcpy(s_tx_pending, data, len);
  s_tx_pending_len = len;
  s_tx_state       = LORA_TX_WAIT_AUX;
  s_tx_state_ms    = BSP_Time_GetTickMs();

  /* Start immediately if the module is already idle; otherwise the state
     machine in Lora_E22_Service() carries it forward without ever blocking
     the comm task. */
  Lora_E22_TxStep(BSP_Time_GetTickMs());
  return LORA_RESULT_OK;
}

void Lora_E22_GetDebugInfo(Lora_DebugInfo_t *info)
{
  uint32_t primask;

  if (info == 0) { return; }

  primask = __get_PRIMASK();
  __disable_irq();
  info->rx_frame_count    = s_rx_frame_count;
  info->tx_frame_count    = s_tx_frame_count;
  info->tx_busy_count     = s_tx_busy_count;
  info->crc_error_count   = s_crc_error_count;
  info->send_error_count  = s_send_error_count;
  info->parse_error_count = s_parse_error_count;
  info->rx_byte_count     = s_rx_byte_count;
  info->rx_drop_count     = s_rx_drop_count;
  info->last_rx_ms        = s_last_rx_ms;
  info->last_tx_ms        = s_last_tx_ms;
  info->last_ready_ms     = s_last_ready_ms;
  info->last_msg_id       = s_last_msg_id;
  if (primask == 0U) { __enable_irq(); }

  info->rx_overflow_count = BSP_LoRa_GetRxOverflowCount();
}
