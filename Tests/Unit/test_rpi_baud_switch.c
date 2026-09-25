/**
 * @file test_rpi_baud_switch.c
 * @brief 验证 UART6 B0 到 B1 切换协议、寻址、去重及整帧发送时序。
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define MAVLINK_COMM_NUM_BUFFERS 1
#if defined(_MSC_VER)
#pragma warning(push, 0)
#endif
#include "common/mavlink.h"
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include "../../Framework/Src/px4lite_rpi_baud_switch.c"

static uint32_t s_mock_baud_bps = 115200U;
static Px4Lite_Result_t s_mock_get_result = PX4LITE_OK;
static Px4Lite_Result_t s_mock_set_result = PX4LITE_OK;
static uint32_t s_set_call_count;

Px4Lite_Result_t Px4Lite_RpiMavlinkGetBaudRate(uint32_t *baud_bps)
{
  if (baud_bps == 0) { return PX4LITE_INVALID_PARAM; }
  if (s_mock_get_result != PX4LITE_OK) { return s_mock_get_result; }
  *baud_bps = s_mock_baud_bps;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_RpiMavlinkSetBaudRate(uint32_t baud_bps)
{
  s_set_call_count++;
  if (s_mock_set_result != PX4LITE_OK) { return s_mock_set_result; }
  s_mock_baud_bps = baud_bps;
  return PX4LITE_OK;
}

static void BuildRequest(uint8_t payload[PX4LITE_RPI_BAUD_PAYLOAD_LENGTH],
                         const uint8_t nonce[PX4LITE_RPI_BAUD_NONCE_LENGTH],
                         uint32_t target_baud_bps)
{
  payload[0] = PX4LITE_RPI_BAUD_VERSION;
  payload[1] = PX4LITE_RPI_BAUD_REQUEST_TYPE;
  memcpy(&payload[2], nonce, PX4LITE_RPI_BAUD_NONCE_LENGTH);
  RpiBaudSwitch_PutU32Le(&payload[10], target_baud_bps);
}

static Px4Lite_Result_t Handle(const uint8_t *payload,
                               uint8_t payload_length,
                               uint8_t wire_length)
{
  return Px4Lite_RpiBaudSwitchHandleTunnel(42U, 191U, 42U, 193U, 42U,
                                           PX4LITE_RPI_BAUD_TUNNEL_TYPE,
                                           payload, payload_length, wire_length);
}

static void TestGoldenVectorAndSendOrdering(void)
{
  static const uint8_t nonce[PX4LITE_RPI_BAUD_NONCE_LENGTH] = {
    0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U
  };
  static const uint8_t expected[PX4LITE_RPI_BAUD_PAYLOAD_LENGTH] = {
    0x01U, 0x04U, 0x08U, 0x07U, 0x06U, 0x05U, 0x04U,
    0x03U, 0x02U, 0x01U, 0x00U, 0xE1U, 0x00U, 0x00U
  };
  uint8_t request[PX4LITE_RPI_BAUD_PAYLOAD_LENGTH];
  uint8_t duplicate_nonce[PX4LITE_RPI_BAUD_NONCE_LENGTH];
  Px4Lite_RpiBaudSwitchResponseView_t response;
  Px4Lite_RpiBaudSwitchStats_t stats;

  BuildRequest(request, nonce, 57600U);
  assert(Px4Lite_RpiBaudSwitchInit() == PX4LITE_OK);
  assert(Handle(request, sizeof(request), 12U) == PX4LITE_OK);
  assert(Px4Lite_RpiBaudSwitchIsPending() == 1U);
  assert(s_mock_baud_bps == 115200U);
  assert(s_set_call_count == 0U);
  assert(Px4Lite_RpiBaudSwitchPeekResponse(&response) == PX4LITE_OK);
  assert(response.payload_type == 0x8003U);
  assert(response.payload_length == 14U);
  assert(response.target_system == 42U && response.target_component == 191U);
  assert(memcmp(response.payload, expected, sizeof(expected)) == 0);

  /* 同一待发 nonce 去重；其他请求不能覆盖已经排队的接受帧。 */
  assert(Handle(request, sizeof(request), 12U) == PX4LITE_OK);
  memcpy(duplicate_nonce, nonce, sizeof(nonce));
  duplicate_nonce[0]++;
  BuildRequest(request, duplicate_nonce, 57600U);
  assert(Handle(request, sizeof(request), 12U) == PX4LITE_BUSY);
  Px4Lite_RpiBaudSwitchGetStats(&stats);
  assert(stats.accepted_count == 1U);
  assert(stats.duplicate_count == 1U);
  assert(stats.busy_count == 1U);

  /* 只有接受帧完整发送后，回调才可重配 UART6。 */
  assert(Px4Lite_RpiBaudSwitchResponseSent() == PX4LITE_OK);
  assert(s_mock_baud_bps == 57600U);
  assert(s_set_call_count == 1U);
  assert(Px4Lite_RpiBaudSwitchIsPending() == 0U);
  Px4Lite_RpiBaudSwitchGetStats(&stats);
  assert(stats.response_sent_count == 1U);
  assert(stats.apply_success_count == 1U);
  assert(stats.current_baud_bps == 57600U);
  assert(stats.state == PX4LITE_RPI_BAUD_STATE_APPLIED);

  /* 同一启动周期内不接受第二次改参，阻止重放 nonce 造成二次切换。 */
  BuildRequest(request, nonce, 57600U);
  assert(Handle(request, sizeof(request), 12U) == PX4LITE_INVALID_PARAM);
  assert(s_set_call_count == 1U);
}

static void TestMavlinkV2FrameVector(void)
{
  static const uint8_t nonce[PX4LITE_RPI_BAUD_NONCE_LENGTH] = {
    0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U
  };
  static const uint8_t expected_response[PX4LITE_RPI_BAUD_PAYLOAD_LENGTH] = {
    0x01U, 0x04U, 0x08U, 0x07U, 0x06U, 0x05U, 0x04U,
    0x03U, 0x02U, 0x01U, 0x00U, 0xE1U, 0x00U, 0x00U
  };
  uint8_t request[PX4LITE_RPI_BAUD_PAYLOAD_LENGTH];
  uint8_t tx_frame[280];
  uint16_t tx_length;
  mavlink_message_t tx_message;
  mavlink_message_t parsed_message;
  mavlink_status_t parse_status;
  mavlink_tunnel_t tunnel;
  mavlink_tunnel_t response_tunnel;
  Px4Lite_RpiBaudSwitchResponseView_t response;
  size_t i;
  uint8_t parsed = 0U;

  BuildRequest(request, nonce, 57600U);
  memset(&tunnel, 0, sizeof(tunnel));
  tunnel.target_system = 42U;
  tunnel.target_component = 193U;
  tunnel.payload_type = PX4LITE_RPI_BAUD_TUNNEL_TYPE;
  tunnel.payload_length = PX4LITE_RPI_BAUD_PAYLOAD_LENGTH;
  memcpy(tunnel.payload, request, sizeof(request));
  (void)mavlink_msg_tunnel_encode_chan(42U, 191U, MAVLINK_COMM_0, &tx_message, &tunnel);
  tx_length = mavlink_msg_to_send_buffer(tx_frame, &tx_message);
  assert(tx_message.len == 17U); /* 尾随两个零字节由 MAVLink 2 裁剪。 */
  assert(tx_length == 29U);

  memset(&parse_status, 0, sizeof(parse_status));
  for (i = 0U; i < tx_length; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_0, tx_frame[i], &parsed_message, &parse_status) != 0) {
      parsed++;
    }
  }
  assert(parsed == 1U && parsed_message.msgid == MAVLINK_MSG_ID_TUNNEL);
  mavlink_msg_tunnel_decode(&parsed_message, &tunnel);
  assert(tunnel.payload_length == PX4LITE_RPI_BAUD_PAYLOAD_LENGTH);
  assert(parsed_message.len == 17U);
  assert(Handle(tunnel.payload, tunnel.payload_length,
                (uint8_t)(parsed_message.len - 5U)) == PX4LITE_OK);
  assert(Px4Lite_RpiBaudSwitchPeekResponse(&response) == PX4LITE_OK);

  memset(&response_tunnel, 0, sizeof(response_tunnel));
  response_tunnel.target_system = response.target_system;
  response_tunnel.target_component = response.target_component;
  response_tunnel.payload_type = response.payload_type;
  response_tunnel.payload_length = response.payload_length;
  memcpy(response_tunnel.payload, response.payload, response.payload_length);
  (void)mavlink_msg_tunnel_encode_chan(42U, 193U, MAVLINK_COMM_0,
                                       &tx_message, &response_tunnel);
  assert(tx_message.len == 17U);
  mavlink_msg_tunnel_decode(&tx_message, &response_tunnel);
  assert(response_tunnel.payload_length == PX4LITE_RPI_BAUD_PAYLOAD_LENGTH);
  assert(memcmp(response_tunnel.payload, expected_response, sizeof(expected_response)) == 0);
  assert(s_mock_baud_bps == 115200U);
  assert(Px4Lite_RpiBaudSwitchResponseSent() == PX4LITE_OK);
  assert(s_mock_baud_bps == 57600U);
}

static void TestStrictValidation(void)
{
  static const uint8_t nonce[PX4LITE_RPI_BAUD_NONCE_LENGTH] = {
    0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U
  };
  uint8_t request[PX4LITE_RPI_BAUD_PAYLOAD_LENGTH];
  Px4Lite_RpiBaudSwitchResponseView_t response;

  assert(Px4Lite_RpiBaudSwitchInit() == PX4LITE_OK);
  BuildRequest(request, nonce, 57600U);
  assert(Px4Lite_RpiBaudSwitchHandleTunnel(42U, 191U, 42U, 193U, 42U,
                                           0x8001U, request, sizeof(request), 12U) == PX4LITE_IDLE);
  assert(Px4Lite_RpiBaudSwitchHandleTunnel(42U, 191U, 42U, 193U, 42U,
                                           0x8003U, request, sizeof(request), 11U) == PX4LITE_INVALID_PARAM);
  assert(Handle(request, (uint8_t)(sizeof(request) - 1U), 12U) == PX4LITE_INVALID_PARAM);
  assert(Handle(request, (uint8_t)(sizeof(request) + 1U), 12U) == PX4LITE_INVALID_PARAM);
  assert(Handle(request, sizeof(request), 15U) == PX4LITE_INVALID_PARAM);
  assert(Px4Lite_RpiBaudSwitchHandleTunnel(42U, 191U, 43U, 193U, 42U,
                                           0x8003U, request, sizeof(request), 12U) == PX4LITE_INVALID_PARAM);
  assert(Px4Lite_RpiBaudSwitchHandleTunnel(42U, 191U, 42U, 191U, 42U,
                                           0x8003U, request, sizeof(request), 12U) == PX4LITE_INVALID_PARAM);
  request[0] = 2U;
  assert(Handle(request, sizeof(request), 12U) == PX4LITE_INVALID_PARAM);
  request[0] = 1U;
  request[1] = 2U;
  assert(Handle(request, sizeof(request), 12U) == PX4LITE_INVALID_PARAM);
  request[1] = 3U;
  RpiBaudSwitch_PutU32Le(&request[10], 38400U);
  assert(Handle(request, sizeof(request), 12U) == PX4LITE_INVALID_PARAM);
  BuildRequest(request, nonce, 57600U);
  memset(&request[2], 0, PX4LITE_RPI_BAUD_NONCE_LENGTH);
  assert(Handle(request, sizeof(request), 12U) == PX4LITE_INVALID_PARAM);
  assert(Px4Lite_RpiBaudSwitchPeekResponse(&response) == PX4LITE_NOT_READY);
}

static void TestApplyFailureAndWrongStartingRate(void)
{
  static const uint8_t nonce[PX4LITE_RPI_BAUD_NONCE_LENGTH] = {
    0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U
  };
  uint8_t request[PX4LITE_RPI_BAUD_PAYLOAD_LENGTH];
  Px4Lite_RpiBaudSwitchStats_t stats;

  s_mock_baud_bps = 57600U;
  assert(Px4Lite_RpiBaudSwitchInit() == PX4LITE_NOT_READY);
  s_mock_baud_bps = 115200U;
  assert(Px4Lite_RpiBaudSwitchInit() == PX4LITE_OK);
  BuildRequest(request, nonce, 57600U);
  assert(Handle(request, sizeof(request), 12U) == PX4LITE_OK);
  s_mock_set_result = PX4LITE_IO_ERROR;
  assert(Px4Lite_RpiBaudSwitchResponseSent() == PX4LITE_IO_ERROR);
  assert(s_mock_baud_bps == 115200U);
  Px4Lite_RpiBaudSwitchGetStats(&stats);
  assert(stats.apply_failure_count == 1U);
  assert(stats.state == PX4LITE_RPI_BAUD_STATE_APPLY_FAILED);
  assert(stats.current_baud_bps == 115200U);
  assert(Handle(request, sizeof(request), 12U) == PX4LITE_INVALID_PARAM);
  s_mock_set_result = PX4LITE_OK;
}

int main(void)
{
  s_mock_baud_bps = 115200U;
  s_mock_get_result = PX4LITE_OK;
  s_mock_set_result = PX4LITE_OK;
  s_set_call_count = 0U;
  assert(Px4Lite_RpiBaudSwitchInit() == PX4LITE_OK);
  TestMavlinkV2FrameVector();
  s_mock_baud_bps = 115200U;
  s_mock_get_result = PX4LITE_OK;
  s_mock_set_result = PX4LITE_OK;
  s_set_call_count = 0U;
  TestGoldenVectorAndSendOrdering();
  s_mock_baud_bps = 115200U;
  s_mock_get_result = PX4LITE_OK;
  TestStrictValidation();
  s_mock_baud_bps = 115200U;
  s_mock_get_result = PX4LITE_OK;
  TestApplyFailureAndWrongStartingRate();
  puts("RPi UART6 波特率切换测试通过");
  return 0;
}
