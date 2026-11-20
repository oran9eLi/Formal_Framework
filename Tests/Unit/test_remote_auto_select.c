/**
 * @file test_remote_auto_select.c
 * @brief 验证箱体移除 LoRa 节点列表后仍可直接观察到已确认身份的对端。
 * @details 模拟接收队列，使用真实 MAVLink 编解码、发现和快照代码，不启动界面。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Framework/Src/px4lite_mavlink_rx.c"
#include "../../Framework/Src/px4lite_remote_telemetry.c"

static uint32_t s_test_now_ms = 1000U;
static Px4Lite_CommRxFrame_t s_frame;

uint8_t Px4Lite_IdentityGetNodeId(void) { return 250U; }
uint32_t Px4Lite_PlatformGetMs(void) { return s_test_now_ms; }

void Px4Lite_IdentityFormatVendorProductId(char *out, uint8_t capacity)
{
  assert(capacity >= 21U);
  memcpy(out, "DCDWCNS1000000000000", 21U);
}

void Px4Lite_LocalMsgLogEvent(uint16_t message, uint32_t now, uint16_t fault, uint8_t source, uint8_t active)
{ (void)message; (void)now; (void)fault; (void)source; (void)active; }

Px4Lite_Result_t Px4Lite_CopyCommRxFrame(Px4Lite_CommRxFrame_t *out)
{
  *out = s_frame;
  return PX4LITE_OK;
}

/** @brief 按接收队列的字段格式下发已校验的 MAVLink 内容。 */
static Px4Lite_Result_t Receive(const mavlink_message_t *message)
{
  s_frame.system_id = message->sysid;
  s_frame.component_id = message->compid;
  s_frame.sequence = message->seq;
  s_frame.msg_id = message->msgid;
  s_frame.payload_len = message->len;
  s_frame.rx_time_ms = ++s_test_now_ms;
  memcpy(s_frame.payload, _MAV_PAYLOAD(message), message->len);
  return Px4Lite_MavlinkRxRun(s_test_now_ms);
}

static void Identify(uint8_t node, const char *product)
{
  mavlink_message_t message;
  mavlink_open_drone_id_basic_id_t packet = {0};
  packet.id_type = MAV_ODID_ID_TYPE_SERIAL_NUMBER;
  memcpy(packet.uas_id, product, sizeof(packet.uas_id));
  mavlink_msg_open_drone_id_basic_id_encode(node, 193U, &message, &packet);
  assert(Receive(&message) == PX4LITE_IDLE);
}

static Px4Lite_Result_t ReceiveGps(uint8_t node)
{
  mavlink_message_t message;
  mavlink_gps_raw_int_t packet = {0};
  packet.fix_type = 3U;
  packet.lat = 225000000;
  packet.lon = 1135000000;
  mavlink_msg_gps_raw_int_encode(node, 193U, &message, &packet);
  return Receive(&message);
}

int main(void)
{
  Px4Lite_RemoteNodeStatus_t status;
  Px4Lite_RemoteTelemetry_t snapshot;

  Px4Lite_RemoteTelemetryInit(s_test_now_ms);
  assert(ReceiveGps(42U) == PX4LITE_IDLE);
  Identify(250U, "DCDWCNS1000000000001");
  assert(Px4Lite_CopySelectedRemoteStatus(&status, s_test_now_ms) == PX4LITE_NOT_READY);

  Identify(42U, "DCDWCNS1000000000002");
  assert(Px4Lite_CopySelectedRemoteStatus(&status, s_test_now_ms) == PX4LITE_OK);
  assert(status.node_id == 42U && status.identity_known && status.state == PX4LITE_REMOTE_NODE_DISCOVERED);
  assert(ReceiveGps(42U) == PX4LITE_OK);
  assert(Px4Lite_CopyRemoteTelemetry(&snapshot) == PX4LITE_OK);
  assert(snapshot.system_id == 42U && (snapshot.navigation_valid_mask & PX4LITE_NAV_VALID_POSITION));
  assert(Px4Lite_CopySelectedRemoteStatus(&status, s_test_now_ms) == PX4LITE_OK);
  assert(status.state == PX4LITE_REMOTE_NODE_ACTIVE && status.rx_frame_count == 3U);

  Identify(43U, "DCDWCNS1000000000003");
  assert(ReceiveGps(43U) == PX4LITE_IDLE);
  Identify(42U, "DCDWCNS1000000000004");
  assert(ReceiveGps(42U) == PX4LITE_IDLE);
  assert(Px4Lite_CopyRemoteTelemetry(&snapshot) == PX4LITE_NOT_READY);
  assert(Px4Lite_CopySelectedRemoteStatus(&status, s_test_now_ms) == PX4LITE_OK && status.id_conflict);

  Px4Lite_RemoteTelemetryResetLink(s_test_now_ms);
  Identify(43U, "DCDWCNS1000000000003");
  assert(ReceiveGps(43U) == PX4LITE_OK);
  assert(Px4Lite_CopySelectedRemoteStatus(&status, s_test_now_ms) == PX4LITE_OK && status.node_id == 43U);
  puts("LoRa auto select: identity, decoded GPS, conflict and link reset PASS");
  return 0;
}
