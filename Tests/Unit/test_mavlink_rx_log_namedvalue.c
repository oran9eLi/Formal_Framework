/**
 * @file test_mavlink_rx_log_namedvalue.c
 * @brief RX 收到 NAMED_VALUE_INT("LOGSYNC") 后按 sequence 去重追加远端日志。
 * @details
 * 替代已删除 px4lite_remote_tunnel 协议下的 test_mavlink_rx_log_tunnel.c：
 * 日志不再走 TUNNEL 整表下发，改为逐条 LOGSYNC，接收端在
 * MavRx_AppendRemoteLog 里按 sequence 去重（重复收到同一条只原地更新，
 * 不会产生重复行），这也是 TX 端低频全量重播能安全复用同一条消息的前提。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common/mavlink.h"
#include "px4lite_command.h"
#include "px4lite_mavlink_rx.h"
#include "px4lite_modules.h"
#include "px4lite_remote_telemetry.h"

static int g_fail;
static void Eq(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

static Px4Lite_CommRxFrame_t s_queued_frame;
static uint8_t s_queued_valid;

Px4Lite_Result_t Px4Lite_CopyCommRxFrame(Px4Lite_CommRxFrame_t *out)
{
  if (s_queued_valid == 0U) { return PX4LITE_NOT_READY; }
  *out = s_queued_frame;
  s_queued_valid = 0U;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_CommandHandleMavlinkLong(uint16_t command, uint8_t sysid, uint8_t compid, uint8_t target_system, uint8_t target_component, float p1, float p2, float p3, float p4, uint32_t now_ms)
{ (void)command; (void)sysid; (void)compid; (void)target_system; (void)target_component; (void)p1; (void)p2; (void)p3; (void)p4; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CommandHandleMavlinkAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, uint32_t now_ms)
{ (void)command; (void)result; (void)target_system; (void)target_component; (void)now_ms; return PX4LITE_NOT_READY; }

#define TEST_REMOTE_SYSID 3U

/* 与 MavTx_SendMessageLog 的打包方式保持一致，序号/消息号/时间/等级/激活/来源。 */
static void QueueLogSync(uint16_t sequence, uint16_t message_id, uint32_t time_hhmmss, uint8_t severity, uint32_t now_ms)
{
  mavlink_named_value_int_t packet;
  mavlink_message_t msg;
  uint32_t packed;

  packed = ((uint32_t)sequence & 0xFFFFUL) |
           (((uint32_t)message_id & 0xFFUL) << 16U) |
           (((uint32_t)severity & 0x0FUL) << 24U);

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = time_hhmmss;
  packet.value        = (int32_t)packed;
  memcpy(packet.name, "LOGSYNC", 7U);

  memset(&msg, 0, sizeof(msg));
  (void)mavlink_msg_named_value_int_encode_chan(TEST_REMOTE_SYSID, 1U, MAVLINK_COMM_0, &msg, &packet);

  memset(&s_queued_frame, 0, sizeof(s_queued_frame));
  s_queued_frame.rx_time_ms   = now_ms;
  s_queued_frame.msg_id       = msg.msgid;
  s_queued_frame.frame_len    = (uint16_t)(msg.len + 12U);
  s_queued_frame.system_id    = msg.sysid;
  s_queued_frame.component_id = msg.compid;
  s_queued_frame.sequence     = msg.seq;
  s_queued_frame.payload_len  = (uint8_t)msg.len;
  memcpy(s_queued_frame.payload, _MAV_PAYLOAD(&msg), msg.len);
  s_queued_valid = 1U;
}

int main(void)
{
  uint32_t now = 8000U;
  Px4Lite_RemoteTelemetry_t out;

  Px4Lite_RemoteTelemetryInit(now);
  (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now);
  (void)Px4Lite_SelectRemoteNode(TEST_REMOTE_SYSID);

  QueueLogSync(1U, 0U, 100U, 0U, now);
  Eq("run seq1", Px4Lite_MavlinkRxRun(now), PX4LITE_OK);
  QueueLogSync(2U, 4U, 200U, 0U, now + 50U);
  Eq("run seq2", Px4Lite_MavlinkRxRun(now + 50U), PX4LITE_OK);

  Eq("copy ok", Px4Lite_CopyRemoteTelemetry(&out), PX4LITE_OK);
  Eq("count", out.remote_log_count, 2U);
  Eq("latest", out.remote_log_latest_seq, 2U);
  Eq("e1 mid", out.remote_log_entries[1].message_id, 4U);

  /* 重播场景：重发 seq1/2(重复) + 新 seq3 -> 只追加 seq3，不产生重复行。 */
  QueueLogSync(1U, 0U, 100U, 0U, now + 100U);
  Eq("run replay seq1", Px4Lite_MavlinkRxRun(now + 100U), PX4LITE_OK);
  QueueLogSync(2U, 4U, 200U, 0U, now + 150U);
  Eq("run replay seq2", Px4Lite_MavlinkRxRun(now + 150U), PX4LITE_OK);
  QueueLogSync(3U, 6U, 300U, 0U, now + 200U);
  Eq("run seq3", Px4Lite_MavlinkRxRun(now + 200U), PX4LITE_OK);

  Eq("copy ok2", Px4Lite_CopyRemoteTelemetry(&out), PX4LITE_OK);
  Eq("dedup count", out.remote_log_count, 3U);
  Eq("dedup latest", out.remote_log_latest_seq, 3U);
  Eq("e2 mid", out.remote_log_entries[2].message_id, 6U);
  Eq("e0 still seq1", out.remote_log_entries[0].sequence, 1U);

  if (g_fail == 0) { printf("PASS test_mavlink_rx_log_namedvalue\n"); return 0; }
  printf("FAIL test_mavlink_rx_log_namedvalue fails=%d\n", g_fail);
  return 1;
}
