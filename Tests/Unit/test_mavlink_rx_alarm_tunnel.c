/**
 * @file test_mavlink_rx_alarm_tunnel.c
 * @brief RX 收到 TUNNEL(0x8001) 后写入远端告警行。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common/mavlink.h"
#include "px4lite_mavlink_rx.h"
#include "px4lite_remote_telemetry.h"
#include "px4lite_remote_tunnel.h"

static int g_fail;
static void ExpectU32(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

int main(void)
{
  uint32_t now = 8000U;
  Px4Lite_AlarmRecord_t rows[2];
  uint8_t payload[PX4LITE_TUNNEL_ALARM_MAX_BYTES];
  uint16_t plen;
  mavlink_message_t msg;
  Px4Lite_LoRaRxFrame_t frame;
  Px4Lite_RemoteTelemetrySnapshot_t out;

  Px4Lite_RemoteTelemetryInit(now);
  (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now);
  Px4Lite_MavlinkRxInit(now);

  memset(rows, 0, sizeof(rows));
  rows[0].source_id = 4U; rows[0].fault_code = 0x55U; rows[0].severity = (Px4Lite_AlarmSeverity_t)2U; rows[0].active = 1U; rows[0].raised_ms = now - 2000U;
  rows[1].source_id = 6U; rows[1].fault_code = 0x66U; rows[1].severity = (Px4Lite_AlarmSeverity_t)1U; rows[1].active = 1U; rows[1].raised_ms = now - 1000U;
  plen = Px4Lite_PackAlarmTable(rows, 2U, 9U, now, payload, sizeof(payload));

  /* 远端 sysid(2) 必须 != 本机(PX4LITE_MAVLINK_SYSTEM_ID)，否则被自收过滤。 */
  (void)mavlink_msg_tunnel_pack_chan(2U, 1U, MAVLINK_COMM_0, &msg,
                                     PX4LITE_MAVLINK_SYSTEM_ID, 0U,
                                     PX4LITE_TUNNEL_PT_ALARM_TABLE, (uint8_t)plen, payload);

  /* 仿真 LoRa 驱动产出的帧：data = MAVLink 负载(非整帧)，由 RX 重建消息。 */
  memset(&frame, 0, sizeof(frame));
  frame.frame_len    = (uint16_t)(msg.len + 12U);
  frame.system_id    = msg.sysid;
  frame.component_id = msg.compid;
  frame.sequence     = msg.seq;
  frame.payload_len  = (uint8_t)msg.len;
  frame.msg_id       = msg.msgid;
  memcpy(frame.data, _MAV_PAYLOAD(&msg), msg.len);

  ExpectU32("handle ok", Px4Lite_MavlinkRxHandleFrame(&frame, now), PX4LITE_OK);
  ExpectU32("copy ok", Px4Lite_RemoteTelemetryCopySnapshot(&out, now), PX4LITE_OK);
  ExpectU32("count", out.alarm_table_count, 2U);
  ExpectU32("ver", out.alarm_table_ver, 9U);
  ExpectU32("rec0 src", out.alarm_records[0].source_id, 4U);
  ExpectU32("rec1 code", out.alarm_records[1].fault_code, 0x66U);

  if (g_fail == 0) { printf("PASS test_mavlink_rx_alarm_tunnel\n"); return 0; }
  printf("FAIL test_mavlink_rx_alarm_tunnel fails=%d\n", g_fail);
  return 1;
}
