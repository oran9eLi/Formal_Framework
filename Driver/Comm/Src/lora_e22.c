/**
 * @file lora_e22.c
 * @brief Implement typed E22 driver on top of BSP_LoRa and MAVLink parser.
 */

#include "lora_e22.h"
#include "bsp_critical.h"
#include "bsp_lora.h"
#include "bsp_time.h"

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

#define LORA_E22_MAVLINK_SYSID_MAX 256U

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
static uint32_t s_rx_sequence_lost_count;
static uint32_t s_last_tx_ms;
static uint32_t s_last_msg_id;
static uint32_t s_last_aux_ready_ms;
static uint8_t s_initialized;
static volatile uint8_t s_reinit_request;
static uint8_t s_rx_sequence_seen[LORA_E22_MAVLINK_SYSID_MAX];
static uint8_t s_rx_sequence_last[LORA_E22_MAVLINK_SYSID_MAX];

/* Non-blocking transmit state machine, advanced from Lora_E22_Service().
   The comm task never blocks on the air interface: a frame is staged here,
   the machine waits (without spinning) for AUX to go idle, starts a USART3
   TX DMA, then waits for AUX to return idle before counting local TX done.
   TX completion is local evidence only; peer online is proven by RX. */
#define LORA_E22_TX_AUX_TIMEOUT_MS      200U  /* give up waiting for AUX idle before TX */
#define LORA_E22_TX_DMA_TIMEOUT_MS      500U  /* backstop for a stuck DMA */
#define LORA_E22_TX_AIR_DONE_TIMEOUT_MS 2000U /* backstop for AUX after UART DMA TC */

typedef enum {
  LORA_TX_IDLE = 0,
  LORA_TX_WAIT_AUX,
  LORA_TX_SENDING,
  LORA_TX_WAIT_AIR_DONE
} Lora_TxState_t;

static Lora_TxState_t s_tx_state;
/* Owner: comm task only. Staged TX frame; must not be re-encoded until the
   in-flight frame returns to LORA_TX_IDLE (see Lora_E22_Send / Lora_E22_TxStep). */
static uint8_t s_tx_pending[LORA_E22_TX_BUF_SIZE];
static uint16_t s_tx_pending_len;
static uint32_t s_tx_state_ms;

static uint32_t Lora_E22_SaturatedAddU32(uint32_t a, uint32_t b);
static uint16_t Lora_E22_CalcLossRateX10(uint32_t lost_count, uint32_t expected_count);
static void Lora_E22_UpdateSequenceStats(uint8_t system_id, uint8_t sequence);
static void Lora_E22_RecordAuxReady(uint32_t now_ms);
static uint8_t Lora_E22_LocalUnavailable(uint32_t now_ms, uint32_t timeout_ms);
static void Lora_E22_TxStep(uint32_t now_ms);

static uint32_t Lora_E22_SaturatedAddU32(uint32_t a, uint32_t b)
{
  if (a > (0xFFFFFFFFUL - b)) { return 0xFFFFFFFFUL; }
  return a + b;
}

static uint16_t Lora_E22_CalcLossRateX10(uint32_t lost_count, uint32_t expected_count)
{
  uint64_t scaled;

  if (expected_count == 0U) { return 0U; }

  scaled = (((uint64_t)lost_count * 1000ULL) + ((uint64_t)expected_count / 2ULL)) / (uint64_t)expected_count;
  if (scaled > 1000ULL) { return 1000U; }
  return (uint16_t)scaled;
}

static void Lora_E22_UpdateSequenceStats(uint8_t system_id, uint8_t sequence)
{
  uint8_t delta;

  if (s_rx_sequence_seen[system_id] == 0U) {
    s_rx_sequence_seen[system_id] = 1U;
    s_rx_sequence_last[system_id] = sequence;
    return;
  }

  delta = (uint8_t)(sequence - s_rx_sequence_last[system_id]);
  if (delta == 0U) { return; }

  if (delta < 128U) {
    s_rx_sequence_lost_count += (uint32_t)(delta - 1U);
    s_rx_sequence_last[system_id] = sequence;
  } else {
    s_rx_sequence_last[system_id] = sequence;
  }
}

/**
 * @brief 记录 E22 AUX 最近一次处于高电平就绪的时间。
 *
 * @param[in] now_ms 当前 comm 周期时间，单位：ms。
 *
 * @note AUX 在空中发送期间会短暂拉低，因此这里只记录硬件事实，不直接判定 OFFLINE。
 */
static void Lora_E22_RecordAuxReady(uint32_t now_ms)
{
  if (BSP_LoRa_IsReady() != 0U) {
    s_last_aux_ready_ms = now_ms;
  }
}

/**
 * @brief 判断本机 E22 是否长期不可用。
 *
 * @param[in] now_ms 当前时间，单位：ms。
 * @param[in] timeout_ms AUX 长期不就绪超时，单位：ms。
 *
 * @return 1 表示本机模块疑似拔掉或硬件不可用；0 表示未形成离线事实。
 *
 * @note 发送状态机未超时前，AUX 低电平属于半双工正常忙状态，不作为拔掉证据。
 */
static uint8_t Lora_E22_LocalUnavailable(uint32_t now_ms, uint32_t timeout_ms)
{
  uint32_t tx_guard_ms = LORA_E22_TX_AUX_TIMEOUT_MS + LORA_E22_TX_DMA_TIMEOUT_MS + LORA_E22_TX_AIR_DONE_TIMEOUT_MS;

  if (BSP_LoRa_IsReady() != 0U) { return 0U; }

  if ((s_tx_state != LORA_TX_IDLE) && ((uint32_t)(now_ms - s_tx_state_ms) <= tx_guard_ms)) {
    return 0U;
  }

  if (s_last_aux_ready_ms == 0U) { return 0U; }

  return ((uint32_t)(now_ms - s_last_aux_ready_ms) > timeout_ms) ? 1U : 0U;
}

Lora_Result_t Lora_E22_Init(void)
{
  uint32_t start_ms;

  s_initialized = 0U;
  BSP_LoRa_SetMode(0U); /* normal mode M0=0 M1=0 */
  start_ms = BSP_Time_GetTickMs();
  while (BSP_LoRa_IsReady() == 0U) {
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
  s_rx_sequence_lost_count = 0U;
  s_last_tx_ms        = 0U;
  s_last_msg_id       = 0U;
  s_last_aux_ready_ms = BSP_Time_GetTickMs();
  memset(s_rx_sequence_seen, 0, sizeof(s_rx_sequence_seen));
  memset(s_rx_sequence_last, 0, sizeof(s_rx_sequence_last));
  s_tx_state          = LORA_TX_IDLE;
  s_tx_pending_len    = 0U;
  s_tx_state_ms       = 0U;
  s_initialized       = 1U;

  return LORA_RESULT_OK;
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
    if (Lora_E22_Init() != LORA_RESULT_OK) {
      return LORA_RESULT_IO_ERROR;
    }
  }

  if (s_initialized == 0U) { return LORA_RESULT_IO_ERROR; }

  Lora_E22_RecordAuxReady(now_ms);

  if (BSP_LoRa_ConsumeRecoverRxRequest() != 0U) {
    BSP_LoRa_RecoverRx();
    memset(&s_parse_status, 0, sizeof(s_parse_status));
    memset(&s_parse_msg, 0, sizeof(s_parse_msg));
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

        primask = BSP_Critical_Enter();

        s_rx_frame.rx_time_ms   = now_ms;
        s_rx_frame.frame_len    = s_parse_msg.len + MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES;
        s_rx_frame.system_id    = s_parse_msg.sysid;
        s_rx_frame.component_id = s_parse_msg.compid;
        s_rx_frame.sequence     = s_parse_msg.seq;
        s_rx_frame.payload_len  = s_parse_msg.len;
        s_rx_frame.msg_id       = s_parse_msg.msgid;
        memcpy(s_rx_frame.data, _MAV_PAYLOAD(&s_parse_msg), s_parse_msg.len);
        s_rx_ready = 1U;
        Lora_E22_UpdateSequenceStats(s_parse_msg.sysid, s_parse_msg.seq);
        s_rx_frame_count++;
        s_last_rx_ms  = now_ms;
        s_last_msg_id = s_parse_msg.msgid;

        BSP_Critical_Exit(primask);
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
  Lora_E22_RecordAuxReady(now_ms);
  return LORA_RESULT_OK;
}

Lora_Result_t Lora_E22_CopyRxFrame(Lora_RxFrame_t *out)
{
  uint32_t primask;

  if (out == 0) { return LORA_RESULT_INVALID_PARAM; }

  primask = BSP_Critical_Enter();

  if (s_rx_ready == 0U) {
    BSP_Critical_Exit(primask);
    return LORA_RESULT_NO_DATA;
  }

  *out       = s_rx_frame;
  s_rx_ready = 0U;

  BSP_Critical_Exit(primask);
  return LORA_RESULT_OK;
}

Lora_State_t Lora_E22_GetState(uint32_t now_ms, uint32_t offline_timeout_ms)
{
  if (s_initialized == 0U) { return LORA_STATE_NOT_READY; }
  if (Lora_E22_LocalUnavailable(now_ms, offline_timeout_ms) != 0U) { return LORA_STATE_FAILED; }
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
        s_tx_state    = LORA_TX_WAIT_AIR_DONE;
        s_tx_state_ms = now_ms;
      } else if ((uint32_t)(now_ms - s_tx_state_ms) > LORA_E22_TX_DMA_TIMEOUT_MS) {
        BSP_LoRa_AbortTx();
        s_send_error_count++;
        s_tx_state = LORA_TX_IDLE;
      }
      break;

    case LORA_TX_WAIT_AIR_DONE:
      if (BSP_LoRa_IsBusy() == 0U) {
        s_tx_frame_count++;
        s_last_tx_ms = now_ms;
        s_tx_state   = LORA_TX_IDLE;
      } else if ((uint32_t)(now_ms - s_tx_state_ms) > LORA_E22_TX_AIR_DONE_TIMEOUT_MS) {
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

  primask = BSP_Critical_Enter();
  info->rx_frame_count    = s_rx_frame_count;
  info->tx_frame_count    = s_tx_frame_count;
  info->tx_busy_count     = s_tx_busy_count;
  info->crc_error_count   = s_crc_error_count;
  info->send_error_count  = s_send_error_count;
  info->parse_error_count = s_parse_error_count;
  info->rx_byte_count     = s_rx_byte_count;
  info->rx_drop_count     = s_rx_drop_count;
  info->rx_sequence_lost_count = s_rx_sequence_lost_count;
  info->rx_sequence_expected_count = Lora_E22_SaturatedAddU32(s_rx_frame_count, s_rx_sequence_lost_count);
  info->last_rx_ms        = s_last_rx_ms;
  info->last_tx_ms        = s_last_tx_ms;
  info->last_msg_id       = s_last_msg_id;
  BSP_Critical_Exit(primask);

  info->rx_loss_rate_x10  = Lora_E22_CalcLossRateX10(info->rx_sequence_lost_count, info->rx_sequence_expected_count);
  info->rx_overflow_count = BSP_LoRa_GetRxOverflowCount();
}
