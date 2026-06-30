/**
 * @file test_mavlink_tx_log_full_replay.c
 * @brief TX 消息日志低频全量重播：新接入接收端可补到最近日志。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common/mavlink.h"
#include "px4lite_config.h"
#include "px4lite_local_msglog.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_remote_tunnel.h"
#include "px4lite_time.h"
#include "px4lite_topics.h"

static int g_fail;
static uint8_t s_seen_tunnel;
static uint8_t s_seen_count;
static uint16_t s_seen_latest;
static Px4Lite_LogEntry_t s_seen_entries[PX4LITE_TUNNEL_LOG_MAX_ENTRIES];

static void Eq(const char *name, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)actual, (unsigned long)expected);
    g_fail++;
  }
}

static void ResetSeen(void)
{
  s_seen_tunnel = 0U;
  s_seen_count = 0U;
  s_seen_latest = 0U;
  memset(s_seen_entries, 0, sizeof(s_seen_entries));
}

Px4Lite_Result_t Px4Lite_LoRaSend(const uint8_t *data, uint16_t length)
{
  mavlink_message_t msg;
  mavlink_status_t status;
  uint16_t i;

  memset(&msg, 0, sizeof(msg));
  memset(&status, 0, sizeof(status));
  for (i = 0U; i < length; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status) != 0U) {
      if (msg.msgid == MAVLINK_MSG_ID_TUNNEL) {
        mavlink_tunnel_t tun;
        mavlink_msg_tunnel_decode(&msg, &tun);
        if (tun.payload_type == PX4LITE_TUNNEL_PT_MESSAGE_LOG) {
          s_seen_tunnel = 1U;
          (void)Px4Lite_UnpackMessageLog(tun.payload, tun.payload_length,
                                          s_seen_entries, PX4LITE_TUNNEL_LOG_MAX_ENTRIES,
                                          &s_seen_count, &s_seen_latest);
        }
      }
    }
  }
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyHealth(Px4Lite_SystemHealth_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmSnapshot(Px4Lite_AlarmSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }

static void RunUntilLog(uint32_t now_ms)
{
  uint8_t i;

  ResetSeen();
  for (i = 0U; (i < 24U) && (s_seen_tunnel == 0U); ++i) {
    (void)Px4Lite_MavlinkTxRun(now_ms);
  }
}

int main(void)
{
  uint32_t now = 1000U;

  Px4Lite_LocalMsgLogReset();
  (void)Px4Lite_MavlinkTxInit(now);

  Px4Lite_LocalMsgLogPush(10U, 101010U, 0U, 0U, 0U, 0U);
  Px4Lite_LocalMsgLogPush(11U, 101011U, 0U, 0U, 0U, 0U);
  Px4Lite_LocalMsgLogPush(12U, 101012U, 0U, 0U, 0U, 0U);

  RunUntilLog(now + 700U);
  Eq("first tunnel", s_seen_tunnel, 1U);
  Eq("first count", s_seen_count, 3U);
  Eq("first latest", s_seen_latest, 3U);

  RunUntilLog(now + 10700U);
  Eq("replay tunnel", s_seen_tunnel, 1U);
  Eq("replay count", s_seen_count, 3U);
  Eq("replay oldest seq", s_seen_entries[0].sequence, 1U);
  Eq("replay newest seq", s_seen_entries[2].sequence, 3U);
  Eq("replay original time", s_seen_entries[2].time_hhmmss, 101012U);

  if (g_fail == 0) { printf("PASS test_mavlink_tx_log_full_replay\n"); return 0; }
  printf("FAIL test_mavlink_tx_log_full_replay fails=%d\n", g_fail);
  return 1;
}
