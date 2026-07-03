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
/* 接收帧有界队列：一个 comm 周期内可能解析出多帧，不能只保留最后一帧。 */
#define LORA_E22_RX_QUEUE_LEN 8U
/* 单次 Service 最大处理 RX 字节数，避免极端积压独占 comm 周期。 */
#define LORA_E22_RX_SERVICE_BYTE_BUDGET 128U

static Lora_RxFrame_t s_rx_queue[LORA_E22_RX_QUEUE_LEN];
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
static uint32_t s_rx_sequence_lost_count;
static uint32_t s_last_tx_ms;
static uint32_t s_last_msg_id;
static uint32_t s_last_aux_ready_ms;
static uint32_t s_init_wait_start_ms;
static uint32_t s_parse_last_byte_ms;
static uint8_t s_initialized;
static uint8_t s_local_failed;
static uint8_t s_bsp_reinit_needed;
static volatile uint8_t s_reinit_request;
static uint8_t s_rx_sequence_seen[LORA_E22_MAVLINK_SYSID_MAX];
static uint8_t s_rx_sequence_last[LORA_E22_MAVLINK_SYSID_MAX];

/* Non-blocking transmit state machine, advanced from Lora_E22_Service().
   The comm task never blocks on the air interface: a frame is staged here,
   the machine waits (without spinning) for AUX to go idle, starts a USART3
   TX DMA, then counts local TX done when the UART DMA transfer completes.
   Peer link online is still proven by RX. */
#define LORA_E22_BITS_PER_BYTE          10U
#define LORA_E22_TIMEOUT_MIN_MS         20U
#define LORA_E22_AIR_TIMEOUT_MARGIN_MS  200U
#define LORA_E22_UART_TIMEOUT_MARGIN_MS 100U
#define LORA_E22_PRESENT_TIMEOUT_MS     2000U
#define LORA_E22_LOSS_MIN_EXPECTED      32U
#define LORA_E22_INIT_AUX_TIMEOUT_MS    500U
#define LORA_E22_PARSE_IDLE_RESET_MS    200U
#define LORA_E22_SEQUENCE_JUMP_MAX      64U
/* 本机发射占空上限，与 px4lite_config.h 的 PX4LITE_LORA_TX_DUTY_LIMIT_PCT 预算一致。
   连续满占空发射会让 E22 持续大电流+发热，诱发模块内部死锁(只能断电恢复)，
   同时半双工链路也需要给对端和本机接收留出空口窗口。 */
#define LORA_E22_TX_DUTY_LIMIT_PCT      40U

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
/* 上一帧发完后强制静默到该时刻，保证发射占空 <= LORA_E22_TX_DUTY_LIMIT_PCT。 */
static uint32_t s_tx_gap_until_ms;

static uint32_t Lora_E22_SaturatedAddU32(uint32_t a, uint32_t b);
static uint16_t Lora_E22_CalcLossRateX10(uint32_t lost_count, uint32_t expected_count);
static uint32_t Lora_E22_CalcTimeoutMs(uint16_t length_bytes, uint32_t bitrate_bps, uint32_t margin_ms);
static uint32_t Lora_E22_GetAuxWaitTimeoutMs(void);
static uint32_t Lora_E22_GetUartDmaTimeoutMs(uint16_t length_bytes);
static uint32_t Lora_E22_GetAirDoneTimeoutMs(uint16_t length_bytes);
static uint8_t Lora_E22_UpdateSequenceStats(uint8_t system_id, uint8_t sequence, uint32_t now_ms);
static void Lora_E22_RecordAuxReady(uint32_t now_ms);
static uint8_t Lora_E22_LocalUnavailable(uint32_t now_ms);
static void Lora_E22_ResetParser(void);
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
  if (expected_count < LORA_E22_LOSS_MIN_EXPECTED) { return 0U; }

  scaled = (((uint64_t)lost_count * 1000ULL) + ((uint64_t)expected_count / 2ULL)) / (uint64_t)expected_count;
  if (scaled > 1000ULL) { return 1000U; }
  return (uint16_t)scaled;
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

static uint32_t Lora_E22_GetAirDoneTimeoutMs(uint16_t length_bytes)
{
  return Lora_E22_CalcTimeoutMs(length_bytes, BSP_LoRa_GetAirBps(), LORA_E22_AIR_TIMEOUT_MARGIN_MS);
}

static uint8_t Lora_E22_UpdateSequenceStats(uint8_t system_id, uint8_t sequence, uint32_t now_ms)
{
  uint8_t delta;

  (void)now_ms;

  if (s_rx_sequence_seen[system_id] == 0U) {
    s_rx_sequence_seen[system_id] = 1U;
    s_rx_sequence_last[system_id] = sequence;
    return 0U;
  }

  delta = (uint8_t)(sequence - s_rx_sequence_last[system_id]);
  if (delta == 0U) {
    return 0U;
  }

  /* 只按序号跳变幅度判断，不再按静默时长判断：静默再久也不能免罚，
     否则"走远失联很久再回来"这种真实丢包会被当成正常重新同步而永远
     算作 0% 丢包。序号跳变过大(通常是对端序号被重置/重启)才不计入，
     真实的连续丢包(哪怕跨越很长静默时间)必须计数。 */
  if (delta <= LORA_E22_SEQUENCE_JUMP_MAX) {
    s_rx_sequence_lost_count += (uint32_t)(delta - 1U);
  }
  s_rx_sequence_last[system_id] = sequence;
  return 0U;
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
 * @brief 判断本机 E22 模块是否已经长期不可用。
 *
 * @param[in] now_ms 当前 comm 周期时间，单位：ms。
 *
 * @return 1 表示本机模块不可用，0 表示仍可认为本机模块存在。
 *
 * @note AUX 在 DMA 发送和空中发送期间会短暂拉低，因此发送状态机未超时时不把 AUX 低电平当作拔掉。
 */
static uint8_t Lora_E22_LocalUnavailable(uint32_t now_ms)
{
  uint32_t tx_guard_ms;
  uint32_t aux_low_ms;

  if (BSP_LoRa_IsReady() != 0U) { return 0U; }
  if (s_last_aux_ready_ms == 0U) { return 1U; }

  aux_low_ms = (uint32_t)(now_ms - s_last_aux_ready_ms);

  tx_guard_ms = Lora_E22_GetAuxWaitTimeoutMs();
  tx_guard_ms = Lora_E22_SaturatedAddU32(tx_guard_ms, Lora_E22_GetUartDmaTimeoutMs(LORA_E22_TX_BUF_SIZE));
  tx_guard_ms = Lora_E22_SaturatedAddU32(tx_guard_ms, Lora_E22_GetAirDoneTimeoutMs(LORA_E22_TX_BUF_SIZE));
  if ((s_tx_state != LORA_TX_IDLE) && ((uint32_t)(now_ms - s_tx_state_ms) <= tx_guard_ms)) { return 0U; }

  return (aux_low_ms > LORA_E22_PRESENT_TIMEOUT_MS) ? 1U : 0U;
}

static void Lora_E22_ResetParser(void)
{
  memset(&s_parse_status, 0, sizeof(s_parse_status));
  memset(&s_parse_msg, 0, sizeof(s_parse_msg));
  s_parse_last_byte_ms = 0U;
}

Lora_Result_t Lora_E22_Init(void)
{
  uint32_t now_ms;

  s_initialized = 0U;
  BSP_LoRa_SetMode(0U); /* normal mode M0=0 M1=0 */
  now_ms = BSP_Time_GetTickMs();
  if (BSP_LoRa_IsReady() == 0U) {
    if (s_init_wait_start_ms == 0U) { s_init_wait_start_ms = now_ms; }
    if ((uint32_t)(now_ms - s_init_wait_start_ms) > LORA_E22_INIT_AUX_TIMEOUT_MS) {
      s_init_wait_start_ms = now_ms;
      s_local_failed       = 1U;
      /* E22 可能在冷启动自检窗口错过了 M0/M1 的下降沿而滞留休眠态。
         重试路径每次都写低电平，同电平重写不产生沿，模块永远唤不醒。
         这里先驱动回休眠模式电平，下次重试开头的 SetMode(0) 就是一个
         真实下降沿，强制 E22 重新锁存工作模式。 */
      BSP_LoRa_SetMode(3U);
      return LORA_RESULT_IO_ERROR;
    }
    return LORA_RESULT_BUSY;
  }
  s_init_wait_start_ms = 0U;
  s_local_failed       = 0U;

  memset(s_rx_queue, 0, sizeof(s_rx_queue));
  s_rx_q_head = 0U;
  s_rx_q_tail = 0U;
  s_rx_q_count = 0U;
  Lora_E22_ResetParser();

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
  s_last_aux_ready_ms = now_ms;
  memset(s_rx_sequence_seen, 0, sizeof(s_rx_sequence_seen));
  memset(s_rx_sequence_last, 0, sizeof(s_rx_sequence_last));
  s_tx_state          = LORA_TX_IDLE;
  s_tx_pending_len    = 0U;
  s_tx_state_ms       = 0U;
  s_tx_gap_until_ms   = 0U;
  s_initialized       = 1U;

  return LORA_RESULT_OK;
}

void Lora_E22_RequestReinit(void)
{
  s_bsp_reinit_needed = 1U;
  s_reinit_request    = 1U;
}

Lora_Result_t Lora_E22_Service(uint32_t now_ms)
{
  uint8_t temp[128];
  uint16_t available;
  uint16_t received;
  uint16_t rx_budget = LORA_E22_RX_SERVICE_BYTE_BUDGET;
  uint16_t i;
  Lora_Result_t init_result;

  if (s_reinit_request != 0U) {
    s_reinit_request = 0U;
    if (s_bsp_reinit_needed != 0U) {
      s_bsp_reinit_needed = 0U;
      if (BSP_LoRa_Init() != 0) {
        s_bsp_reinit_needed = 1U;
        s_reinit_request    = 1U;
        s_initialized       = 0U;
        s_local_failed      = 1U;
        return LORA_RESULT_IO_ERROR;
      }
      s_local_failed = 0U;
    }
    init_result = Lora_E22_Init();
    if (init_result != LORA_RESULT_OK) {
      if (init_result == LORA_RESULT_IO_ERROR) { s_bsp_reinit_needed = 1U; }
      s_reinit_request = 1U;
      return init_result;
    }
  }

  if (s_initialized == 0U) { return LORA_RESULT_IO_ERROR; }

  Lora_E22_RecordAuxReady(now_ms);

  if (BSP_LoRa_ConsumeRecoverRxRequest() != 0U) {
    BSP_LoRa_RecoverRx();
    Lora_E22_ResetParser();
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
      uint16_t drop_before       = s_parse_status.packet_rx_drop_count;

      if ((s_parse_last_byte_ms != 0U) && ((uint32_t)(now_ms - s_parse_last_byte_ms) > LORA_E22_PARSE_IDLE_RESET_MS)) {
        Lora_E22_ResetParser();
      }
      s_parse_last_byte_ms = now_ms;

      if (mavlink_parse_char(MAVLINK_COMM_0, temp[i], &s_parse_msg, &s_parse_status) != 0) {
        /* --- complete MAVLink frame received --- */
        uint32_t primask;
        Lora_RxFrame_t *slot;

        primask = BSP_Critical_Enter();

        if (s_rx_q_count >= LORA_E22_RX_QUEUE_LEN) {
          s_rx_q_tail = (uint16_t)((s_rx_q_tail + 1U) % LORA_E22_RX_QUEUE_LEN);
          s_rx_q_count--;
          s_rx_drop_count++;
        }

        slot               = &s_rx_queue[s_rx_q_head];
        slot->rx_time_ms   = now_ms;
        slot->frame_len    = s_parse_msg.len + MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES;
        slot->system_id    = s_parse_msg.sysid;
        slot->component_id = s_parse_msg.compid;
        slot->sequence     = s_parse_msg.seq;
        slot->payload_len  = s_parse_msg.len;
        slot->msg_id       = s_parse_msg.msgid;
        memcpy(slot->data, _MAV_PAYLOAD(&s_parse_msg), s_parse_msg.len);
        s_rx_q_head = (uint16_t)((s_rx_q_head + 1U) % LORA_E22_RX_QUEUE_LEN);
        s_rx_q_count++;
        (void)Lora_E22_UpdateSequenceStats(s_parse_msg.sysid, s_parse_msg.seq, now_ms);
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

  if (s_rx_q_count == 0U) {
    BSP_Critical_Exit(primask);
    return LORA_RESULT_NO_DATA;
  }

  *out = s_rx_queue[s_rx_q_tail];
  s_rx_q_tail = (uint16_t)((s_rx_q_tail + 1U) % LORA_E22_RX_QUEUE_LEN);
  s_rx_q_count--;

  BSP_Critical_Exit(primask);
  return LORA_RESULT_OK;
}

Lora_State_t Lora_E22_GetState(uint32_t now_ms, uint32_t offline_timeout_ms)
{
  (void)offline_timeout_ms;

  if (s_initialized == 0U) {
    Lora_E22_RecordAuxReady(now_ms);
    if (BSP_LoRa_IsReady() != 0U) {
      s_local_failed       = 0U;
      s_bsp_reinit_needed = 1U;
      s_reinit_request    = 1U;
      return LORA_STATE_NOT_READY;
    }
    return (s_local_failed != 0U) ? LORA_STATE_FAILED : LORA_STATE_NOT_READY;
  }
  Lora_E22_RecordAuxReady(now_ms);

  if (Lora_E22_LocalUnavailable(now_ms) != 0U) {
    s_local_failed = 1U;
    return LORA_STATE_FAILED;
  }

  return LORA_STATE_ONLINE;
}

uint8_t Lora_E22_IsPresent(void)
{
  uint32_t now_ms;

  if (BSP_LoRa_IsReady() != 0U) { return 1U; }
  if (s_initialized == 0U) { return 0U; }
  now_ms = BSP_Time_GetTickMs();
  Lora_E22_RecordAuxReady(now_ms);
  return (Lora_E22_LocalUnavailable(now_ms) == 0U) ? 1U : 0U;
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
        uint32_t air_ms;
        uint32_t bps = BSP_LoRa_GetAirBps();

        s_tx_frame_count++;
        s_last_tx_ms = now_ms;
        s_tx_state   = LORA_TX_IDLE;
        /* 按帧空口时间换算强制静默期：gap = air * (100 - duty) / duty。 */
        if (bps != 0U) {
          air_ms = (((uint32_t)s_tx_pending_len * LORA_E22_BITS_PER_BYTE * 1000U) + bps - 1U) / bps;
          s_tx_gap_until_ms = now_ms + ((air_ms * (100U - LORA_E22_TX_DUTY_LIMIT_PCT)) / LORA_E22_TX_DUTY_LIMIT_PCT);
        }
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

uint8_t Lora_E22_IsTxIdle(void)
{
  if (s_tx_state != LORA_TX_IDLE) { return 0U; }
  /* 强制静默期内同样报忙，MAVLink 调度器不编码、不消耗序号。 */
  if ((int32_t)(BSP_Time_GetTickMs() - s_tx_gap_until_ms) < 0) { return 0U; }
  return 1U;
}

Lora_Result_t Lora_E22_Send(const uint8_t *data, uint16_t len)
{
  if (data == 0 || len == 0U || len > LORA_E22_TX_BUF_SIZE) { return LORA_RESULT_INVALID_PARAM; }
  if (s_initialized == 0U) { return LORA_RESULT_BUSY; }

  /* One frame in flight at a time. While a frame is staged or sending,
     report BUSY so the MAVLink scheduler retries after its short backoff
     instead of overwriting the in-flight frame. */
  if (s_tx_state != LORA_TX_IDLE) {
    s_tx_busy_count++;
    return LORA_RESULT_BUSY;
  }
  /* 占空静默期内拒绝新帧，双保险(调度器正常情况下已被 IsTxIdle 挡住)。 */
  if ((int32_t)(BSP_Time_GetTickMs() - s_tx_gap_until_ms) < 0) {
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
