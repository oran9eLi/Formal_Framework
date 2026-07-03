/**
 * @file test_mavlink_tx_log_replay.c
 * @brief TX 低频全量重播：不依赖对端请求，定期把本机日志缓冲重发一遍。
 * @details
 * 替代已删除 px4lite_remote_tunnel 协议下的 test_mavlink_tx_log_full_replay.c：
 * 日志改为逐条 LOGSYNC NAMED_VALUE_INT 增量发送，TX 端另有一个独立的低频
 * 重播游标(s_log_replay_cursor)，每隔 PX4LITE_MAVLINK_LOG_REPLAY_PERIOD_MS
 * 把游标拉回缓冲区最早一条重新发一遍，验证：首次全部增量发出后，等到重播
 * 周期到达，之前发过的旧消息会被重新发送一遍（接收端按 sequence 去重，
 * 重发是安全的）。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common/mavlink.h"
#include "px4lite_config.h"
#include "px4lite_local_msglog.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_topics.h"

static int g_fail;
static void Eq(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

#define MAX_SEEN 16U
static uint16_t s_seen_seq[MAX_SEEN];
static uint16_t s_seen_count;

/* 抓取每一帧 LOGSYNC，记录其携带的 sequence(打包在 value 低 16 位)。 */
Px4Lite_Result_t Px4Lite_LoRaSend(const uint8_t *data, uint16_t length)
{
  mavlink_message_t msg;
  mavlink_status_t status;
  uint16_t i;

  memset(&msg, 0, sizeof(msg));
  memset(&status, 0, sizeof(status));
  for (i = 0U; i < length; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status) != 0U) {
      if (msg.msgid == MAVLINK_MSG_ID_NAMED_VALUE_INT) {
        mavlink_named_value_int_t nv;
        mavlink_msg_named_value_int_decode(&msg, &nv);
        if (strncmp(nv.name, "LOGSYNC", 7) == 0) {
          if (s_seen_count < MAX_SEEN) { s_seen_seq[s_seen_count++] = (uint16_t)((uint32_t)nv.value & 0xFFFFU); }
        }
      }
    }
  }
  return PX4LITE_OK;
}

uint8_t Px4Lite_LoRaIsTxIdle(void) { return 1U; }

/* 本测试只关心日志重播路径，其余数据源全部置为未就绪。 */
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyHealth(Px4Lite_SystemHealth_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmSummary(uint32_t *publish_time_ms, uint32_t *sequence, uint16_t *active_count, uint16_t *highest_fault_code, uint16_t *highest_source_id, Px4Lite_AlarmSeverity_t *highest_severity)
{ (void)publish_time_ms; (void)sequence; (void)active_count; (void)highest_fault_code; (void)highest_source_id; (void)highest_severity; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmRecord(uint16_t index, Px4Lite_AlarmRecord_t *out) { (void)index; (void)out; return PX4LITE_NOT_READY; }

static void RunUntilSeenGrows(uint32_t *now, uint32_t step_ms, uint32_t max_steps)
{
  uint32_t i;
  uint16_t before = s_seen_count;
  for (i = 0U; (i < max_steps) && (s_seen_count == before); ++i) {
    *now += step_ms;
    (void)Px4Lite_MavlinkTxRun(*now);
  }
}

int main(void)
{
  uint32_t now = 1000U;
  uint16_t i;

  Px4Lite_LocalMsgLogReset();
  (void)Px4Lite_MavlinkTxInit(now);

  Px4Lite_LocalMsgLogPush(10U, 101010U, 0U, 0U, 0U, 0U);
  Px4Lite_LocalMsgLogPush(11U, 101011U, 0U, 0U, 0U, 0U);
  Px4Lite_LocalMsgLogPush(12U, 101012U, 0U, 0U, 0U, 0U);

  /* 首轮增量：三条应依次发出，序号 1,2,3。 */
  for (i = 0U; i < 3U; ++i) { RunUntilSeenGrows(&now, 200U, 30U); }
  Eq("first pass count", s_seen_count, 3U);
  Eq("first seq0", s_seen_seq[0], 1U);
  Eq("first seq1", s_seen_seq[1], 2U);
  Eq("first seq2", s_seen_seq[2], 3U);

  /* 增量发完之后，没有新消息，不应该再有增量帧，直到重播周期到达。 */
  {
    uint16_t before = s_seen_count;
    uint32_t j;
    for (j = 0U; j < 5U; ++j) { now += 500U; (void)Px4Lite_MavlinkTxRun(now); }
    Eq("idle before replay window", s_seen_count, before);
  }

  /* 推进到低频全量重播周期，三条旧消息应被重新发送一遍(序号不变)。 */
  s_seen_count = 0U;
  for (i = 0U; i < 3U; ++i) { RunUntilSeenGrows(&now, 300U, (PX4LITE_MAVLINK_LOG_REPLAY_PERIOD_MS / 300U) + 20U); }
  Eq("replay count", s_seen_count, 3U);
  Eq("replay seq0", s_seen_seq[0], 1U);
  Eq("replay seq1", s_seen_seq[1], 2U);
  Eq("replay seq2", s_seen_seq[2], 3U);

  if (g_fail == 0) { printf("PASS test_mavlink_tx_log_replay\n"); return 0; }
  printf("FAIL test_mavlink_tx_log_replay fails=%d\n", g_fail);
  return 1;
}
