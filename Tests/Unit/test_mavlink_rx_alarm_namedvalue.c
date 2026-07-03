/**
 * @file test_mavlink_rx_alarm_namedvalue.c
 * @brief RX 收到 NAMED_VALUE_INT("ALRMHI"/"ALRMMSK") 后写入远端最高告警与活动位图。
 * @details
 * 替代已删除 px4lite_remote_tunnel 协议下的 test_mavlink_rx_alarm_tunnel.c：
 * 告警不再走 TUNNEL 整表下发，改为两条轻量 NAMED_VALUE_INT
 * (ALRMHI=最高故障码+来源+等级，ALRMMSK=活动告警位图)。
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

/* Px4Lite_MavlinkRxRun 每次调用只处理一帧；这里用一个单帧槽位模拟通信层 RX 队列。 */
static Px4Lite_CommRxFrame_t s_queued_frame;
static uint8_t s_queued_valid;

Px4Lite_Result_t Px4Lite_CopyCommRxFrame(Px4Lite_CommRxFrame_t *out)
{
  if (s_queued_valid == 0U) { return PX4LITE_NOT_READY; }
  *out = s_queued_frame;
  s_queued_valid = 0U;
  return PX4LITE_OK;
}

/* 本测试不触达命令处理路径，仅提供链接桩。 */
Px4Lite_Result_t Px4Lite_CommandHandleMavlinkLong(uint16_t command, uint8_t sysid, uint8_t compid, uint8_t target_system, uint8_t target_component, float p1, float p2, float p3, float p4, uint32_t now_ms)
{ (void)command; (void)sysid; (void)compid; (void)target_system; (void)target_component; (void)p1; (void)p2; (void)p3; (void)p4; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CommandHandleMavlinkAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, uint32_t now_ms)
{ (void)command; (void)result; (void)target_system; (void)target_component; (void)now_ms; return PX4LITE_NOT_READY; }

/* 远端 sysid 取 本机(DCDW-002=2)+1=3，避免被自收过滤。 */
#define TEST_REMOTE_SYSID 3U

static void QueueNamedValueInt(const char *name, int32_t value, uint32_t now_ms)
{
  mavlink_named_value_int_t packet;
  mavlink_message_t msg;

  memset(&packet, 0, sizeof(packet));
  packet.time_boot_ms = now_ms;
  packet.value        = value;
  memcpy(packet.name, name, strlen(name));

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
  uint32_t packed_hi;

  Px4Lite_RemoteTelemetryInit(now);
  (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now);
  (void)Px4Lite_SelectRemoteNode(TEST_REMOTE_SYSID);

  /* ALRMHI: fault_code=0x0155, source_id=4, severity=2。 */
  packed_hi = (0x0155UL & 0xFFFFUL) | ((4UL & 0xFFUL) << 16U) | ((2UL & 0x0FUL) << 24U);
  QueueNamedValueInt("ALRMHI", (int32_t)packed_hi, now);
  Eq("run alrmhi", Px4Lite_MavlinkRxRun(now), PX4LITE_OK);

  Eq("copy after alrmhi", Px4Lite_CopyRemoteTelemetry(&out), PX4LITE_OK);
  Eq("highest fault", out.highest_fault_code, 0x0155U);
  Eq("highest source", out.highest_source_id, 4U);
  Eq("highest severity", (uint32_t)out.highest_severity, 2U);
  Eq("alarm valid bit", (out.valid_mask & PX4LITE_REMOTE_VALID_ALARM) != 0U, 1U);

  /* ALRMMSK: 活动位图 = bit3|bit6。 */
  QueueNamedValueInt("ALRMMSK", (int32_t)((1UL << 3) | (1UL << 6)), now + 100U);
  Eq("run alrmmsk", Px4Lite_MavlinkRxRun(now + 100U), PX4LITE_OK);

  Eq("copy after alrmmsk", Px4Lite_CopyRemoteTelemetry(&out), PX4LITE_OK);
  Eq("active mask", out.alarm_active_mask, (1UL << 3) | (1UL << 6));
  /* ALRMHI 写入的字段应保持不变，两条消息互不覆盖。 */
  Eq("highest fault kept", out.highest_fault_code, 0x0155U);

  if (g_fail == 0) { printf("PASS test_mavlink_rx_alarm_namedvalue\n"); return 0; }
  printf("FAIL test_mavlink_rx_alarm_namedvalue fails=%d\n", g_fail);
  return 1;
}
