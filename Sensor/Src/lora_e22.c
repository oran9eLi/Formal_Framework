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

static Lora_RxFrame_t s_rx_frame;
static volatile uint8_t s_rx_ready;

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
static uint32_t s_last_msg_id;
static uint8_t s_initialized;
static uint8_t s_present; /* 模块是否在位(配置寄存器读回握手判定) */
static volatile uint8_t s_reinit_request;

/* Non-blocking transmit state machine, advanced from Lora_E22_Service().
   The comm task never blocks on the air interface: a frame is staged here,
   the machine waits (without spinning) for AUX to go idle, starts a USART3
   TX DMA, then returns to IDLE when the DMA + UART transfer completes. */
#define LORA_E22_TX_AUX_TIMEOUT_MS 200U /* give up waiting for AUX idle */
#define LORA_E22_TX_DMA_TIMEOUT_MS 500U /* backstop for a stuck DMA */

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

static void Lora_E22_TxStep(uint32_t now_ms);

/* E22 在位检测（基于 AUX 引脚）：
   E22 的 AUX 为推挽输出，模块在位且就绪时主动把 AUX 拉高。BSP 把 AUX 配成内部下拉
   输入，于是：模块在位 -> AUX 被模块驱动为高；模块未接入 -> 下拉到低。
   因此 init 顶部的“等待 AUX 就绪”循环若在超时内变高，即说明模块在位；若超时(无模块
   下拉恒低)则 init 返回 BUSY，上层据此判 FAILED(红)。本函数在就绪等待通过后调用，
   再多采样几次以避开模块忙(AUX 暂时拉低)的瞬态。 */
static uint8_t Lora_E22_AuxPresent(void)
{
  uint32_t t0 = BSP_Time_GetTickMs();
  while ((uint32_t)(BSP_Time_GetTickMs() - t0) < 30U) {
    if (BSP_LoRa_IsReady() != 0U) { return 1U; }
  }
  return 0U;
}

Lora_Result_t Lora_E22_Init(void)
{
  uint32_t start_ms;

  s_initialized = 0U;
  s_present     = 0U;
  BSP_LoRa_SetMode(0U); /* normal mode M0=0 M1=0 */
  start_ms = BSP_Time_GetTickMs();
  while (BSP_LoRa_IsReady() == 0U) {
    /* AUX 下拉：无模块时恒低，超时即判定未接入并让 init 失败。 */
    if ((uint32_t)(BSP_Time_GetTickMs() - start_ms) > 500U) { return LORA_RESULT_BUSY; }
  }

  memset(&s_rx_frame, 0, sizeof(s_rx_frame));
  s_rx_ready = 0U;
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
  s_last_msg_id       = 0U;
  s_tx_state          = LORA_TX_IDLE;
  s_tx_pending_len    = 0U;
  s_tx_state_ms       = 0U;
  s_initialized       = 1U;

  /* 走到这里说明 AUX 已就绪(变高)，即模块在位；再采样确认避开瞬态忙。 */
  s_present = Lora_E22_AuxPresent();

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
  uint16_t i;

  if (s_reinit_request != 0U) {
    s_reinit_request = 0U;
    (void)Lora_E22_Init();
  }

  while ((available = BSP_LoRa_GetRxCount()) > 0U) {
    received = BSP_LoRa_GetRxData(temp, (available > sizeof(temp)) ? (uint16_t)sizeof(temp) : available);

    if (received == 0U) { break; }
    s_rx_byte_count += received;

    for (i = 0U; i < received; i++) {
      uint8_t parse_error_before = s_parse_status.parse_error;
      uint16_t drop_before       = s_parse_status.packet_rx_drop_count;

      if (mavlink_parse_char(MAVLINK_COMM_0, temp[i], &s_parse_msg, &s_parse_status) != 0) {
        /* --- complete MAVLink frame received --- */
        uint32_t primask;

        primask = __get_PRIMASK();
        __disable_irq();

        s_rx_frame.frame_len    = s_parse_msg.len + MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES;
        s_rx_frame.system_id    = s_parse_msg.sysid;
        s_rx_frame.component_id = s_parse_msg.compid;
        s_rx_frame.sequence     = s_parse_msg.seq;
        s_rx_frame.payload_len  = s_parse_msg.len;
        s_rx_frame.msg_id       = s_parse_msg.msgid;
        memcpy(s_rx_frame.data, _MAV_PAYLOAD(&s_parse_msg), s_parse_msg.len);
        s_rx_ready = 1U;
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
      if (s_parse_status.packet_rx_drop_count != drop_before) { s_rx_drop_count += (uint16_t)(s_parse_status.packet_rx_drop_count - drop_before); }
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

  if (s_rx_ready == 0U) {
    if (primask == 0U) { __enable_irq(); }
    return LORA_RESULT_NO_DATA;
  }

  *out       = s_rx_frame;
  s_rx_ready = 0U;

  if (primask == 0U) { __enable_irq(); }
  return LORA_RESULT_OK;
}

Lora_State_t Lora_E22_GetState(uint32_t now_ms, uint32_t offline_timeout_ms)
{
  if (s_initialized == 0U) { return LORA_STATE_NOT_READY; }
  /* 在线判定只认实际收到对端帧(RX)。本机 TX 是开环串口发送，不插模块/无对端时
     也会"发出去"，不能作为通信在线的依据，否则会误判为在线并随 AUX 悬空来回抖动。 */
  if (s_last_rx_ms == 0U) { return LORA_STATE_NOT_READY; }
  if ((uint32_t)(now_ms - s_last_rx_ms) <= offline_timeout_ms) { return LORA_STATE_ONLINE; }
  return LORA_STATE_OFFLINE;
}

/* Advance the transmit state machine. Called every comm cycle from
   Lora_E22_Service() and once from Lora_E22_Send(). Never blocks. */
static void Lora_E22_TxStep(uint32_t now_ms)
{
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
      if ((s_tx_state == LORA_TX_WAIT_AUX) && ((uint32_t)(now_ms - s_tx_state_ms) > LORA_E22_TX_AUX_TIMEOUT_MS)) {
        s_tx_busy_count++;
        s_tx_state = LORA_TX_IDLE;
      }
      break;

    case LORA_TX_SENDING:
      if (BSP_LoRa_IsTxBusy() == 0U) {
        s_tx_frame_count++;
        s_last_tx_ms = now_ms;
        s_tx_state   = LORA_TX_IDLE;
      } else if ((uint32_t)(now_ms - s_tx_state_ms) > LORA_E22_TX_DMA_TIMEOUT_MS) {
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
  info->last_msg_id       = s_last_msg_id;
  if (primask == 0U) { __enable_irq(); }

  info->rx_overflow_count = BSP_LoRa_GetRxOverflowCount();
}
