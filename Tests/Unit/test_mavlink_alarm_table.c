/**
 * @file test_mavlink_alarm_table.c
 * @brief 使用真实 MAVLink 解码验证告警表边界、整表拒绝和空表清除。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Framework/Src/px4lite_mavlink_rx.c"

static mavlink_message_t message;
static mavlink_tunnel_t packet;
static Px4Lite_RemoteTelemetry_t remote;

/** @brief 编码真实 TUNNEL 帧后送入实际告警解码器。 */
static uint8_t Decode(void)
{
  mavlink_msg_tunnel_encode(42U, 191U, &message, &packet);
  return MavRx_DecodeTunnel(&remote, &message);
}

/** @brief 非法报文不得更改任何旧告警字段或更新时间。 */
static void ExpectRejected(void)
{
  Px4Lite_RemoteTelemetry_t before = remote;
  assert(Decode() == 0U);
  assert(memcmp(&before, &remote, sizeof(remote)) == 0);
}

int main(void)
{
  static const uint8_t invalid_lengths[] = {0U, 1U, 129U, 255U};
  unsigned i;
  memset(&remote, 0, sizeof(remote));
  remote.last_rx_ms = 10000U;
  remote.alarm_active_mask = 1U;
  remote.alarm_fault_code[0] = 0x1234U;
  remote.alarm_update_ms = 123U;
  memset(&packet, 0, sizeof(packet));
  packet.payload_type = 0x8001U;
  packet.payload[0] = 1U;
  packet.payload[1] = 19U;
  for (i = 0; i < sizeof(invalid_lengths); ++i) {
    packet.payload_length = invalid_lengths[i];
    ExpectRejected();
  }
  packet.payload_length = 9U;
  packet.payload[1] = 2U; /* 完整一行后截断，不能发布部分表。 */
  ExpectRejected();
  packet.payload[1] = 1U;
  packet.payload[0] = 2U;
  ExpectRejected();
  packet.payload[0] = 1U;
  packet.payload_length = 10U; /* 不匹配行数的尾部数据。 */
  ExpectRejected();
  packet.payload_length = 9U;
  packet.payload[2] = 1U;
  packet.payload[3] = 0x78U;
  packet.payload[4] = 0x56U;
  packet.payload[5] = 2U;
  packet.payload[6] = 1U;
  assert(Decode() == 1U);
  assert(remote.alarm_active_mask == 2U);
  assert(remote.alarm_fault_code[1] == 0x5678U);
  assert(remote.alarm_fault_code[0] == 0U);
  assert(remote.highest_fault_code == 0x5678U);
  assert(remote.alarm_update_ms == 10000U);
  /* 最大合法表：2 + 18 * 7 = 128；未知来源按既有兼容策略跳过。 */
  packet.payload_length = 128U;
  packet.payload[1] = 18U;
  for (i = 1U; i < 18U; ++i) { packet.payload[2U + 7U * i] = 255U; }
  assert(Decode() == 1U);
  assert(remote.alarm_active_mask == 2U);
  packet.payload_length = 2U;
  packet.payload[1] = 0U;
  assert(Decode() == 1U);
  assert(remote.alarm_active_mask == 0U && remote.highest_fault_code == 0U);
  for (i = 0U; i < PX4LITE_MODULE_COUNT; ++i) {
    assert(remote.alarm_fault_code[i] == 0U && remote.alarm_severity[i] == 0U);
  }
  assert((remote.valid_mask & PX4LITE_REMOTE_VALID_ALARM) != 0U);
  puts("mavlink alarm table tests passed");
  return 0;
}
