/**
 * @file test_mavlink_rx_log_tunnel.c
 * @brief RX 收到 TUNNEL(0x8002) 后去重追加远端日志。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common/mavlink.h"
#include "px4lite_mavlink_rx.h"
#include "px4lite_remote_telemetry.h"
#include "px4lite_remote_tunnel.h"

static int g_fail;
static void Eq(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

/* 仿真 LoRa 驱动产出的帧：data = MAVLink 负载(非整帧)，由 RX 重建消息。
   远端 sysid 取 本机+1 以保证 != 本机(PX4LITE_MAVLINK_SYSTEM_ID)，否则被自收过滤。 */
static void SendLog(const Px4Lite_LogEntry_t *e, uint8_t cnt, uint16_t latest, uint32_t now)
{
  uint8_t payload[PX4LITE_TUNNEL_LOG_MAX_BYTES];
  uint16_t plen = Px4Lite_PackMessageLog(e, cnt, latest, payload, sizeof(payload));
  uint8_t remote_sysid = (uint8_t)(PX4LITE_MAVLINK_SYSTEM_ID + 1U);
  mavlink_message_t msg;
  Px4Lite_LoRaRxFrame_t frame;

  (void)mavlink_msg_tunnel_pack_chan(remote_sysid, 1U, MAVLINK_COMM_0, &msg, PX4LITE_MAVLINK_SYSTEM_ID, 0U,
                                     PX4LITE_TUNNEL_PT_MESSAGE_LOG, (uint8_t)plen, payload);
  memset(&frame, 0, sizeof(frame));
  frame.frame_len    = (uint16_t)(msg.len + 12U);
  frame.system_id    = msg.sysid;
  frame.component_id = msg.compid;
  frame.sequence     = msg.seq;
  frame.payload_len  = (uint8_t)msg.len;
  frame.msg_id       = msg.msgid;
  memcpy(frame.data, _MAV_PAYLOAD(&msg), msg.len);
  (void)Px4Lite_MavlinkRxHandleFrame(&frame, now);
}

int main(void)
{
  uint32_t now = 8000U;
  Px4Lite_LogEntry_t e[3];
  Px4Lite_RemoteTelemetrySnapshot_t out;

  Px4Lite_RemoteTelemetryInit(now);
  (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now);
  Px4Lite_MavlinkRxInit(now);

  memset(e, 0, sizeof(e));
  e[0].sequence = 1U; e[0].message_id = 0U; e[0].time_hhmmss = 100U;
  e[1].sequence = 2U; e[1].message_id = 4U; e[1].time_hhmmss = 200U;
  SendLog(e, 2U, 2U, now);

  Eq("copy ok", Px4Lite_RemoteTelemetryCopySnapshot(&out, now), PX4LITE_OK);
  Eq("count", out.log_count, 2U);
  Eq("last_seq", out.log_last_seq, 2U);
  Eq("e1 mid", out.log_entries[1].message_id, 4U);

  /* 重发 seq1/2(重复) + 新 seq3 → 只追加 seq3。 */
  e[2].sequence = 3U; e[2].message_id = 6U; e[2].time_hhmmss = 300U;
  SendLog(e, 3U, 3U, now + 100U);
  (void)Px4Lite_RemoteTelemetryCopySnapshot(&out, now + 100U);
  Eq("dedup count", out.log_count, 3U);
  Eq("dedup last", out.log_last_seq, 3U);
  Eq("e2 mid", out.log_entries[2].message_id, 6U);

  /* 心跳 count=0：不改数据。 */
  SendLog(0, 0U, 3U, now + 200U);
  (void)Px4Lite_RemoteTelemetryCopySnapshot(&out, now + 200U);
  Eq("hb count unchanged", out.log_count, 3U);
  Eq("hb last unchanged", out.log_last_seq, 3U);

  if (g_fail == 0) { printf("PASS test_mavlink_rx_log_tunnel\n"); return 0; }
  printf("FAIL test_mavlink_rx_log_tunnel fails=%d\n", g_fail); return 1;
}
