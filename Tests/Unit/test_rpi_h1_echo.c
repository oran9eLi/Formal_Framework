/**
 * @file test_rpi_h1_echo.c
 * @brief 验证箱子端 H1 回显的字节契约、精确长度及无改参路径。
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define MAVLINK_COMM_NUM_BUFFERS 1
#if defined(_MSC_VER)
#pragma warning(push, 0) /* 第三方 MAVLink 生成头保留其原有 MSVC 扩展告警。 */
#endif
#include "common/mavlink.h"
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include "../../Framework/Src/px4lite_rpi_h1_echo.c"

static uint32_t s_mock_baud_bps = 57600U;
static Px4Lite_Result_t s_mock_get_result = PX4LITE_OK;

Px4Lite_Result_t Px4Lite_RpiMavlinkGetBaudRate(uint32_t *baud_bps)
{
  if (baud_bps == 0) { return PX4LITE_INVALID_PARAM; }
  if (s_mock_get_result != PX4LITE_OK) { return s_mock_get_result; }
  *baud_bps = s_mock_baud_bps;
  return PX4LITE_OK;
}

static uint8_t BuildRequest(uint8_t *out, const uint8_t nonce[8],
                            const uint8_t *text, uint8_t text_length)
{
  memset(out, 0, 128U);
  out[0] = PX4LITE_RPI_H1_VERSION;
  out[1] = PX4LITE_RPI_H1_REQUEST_TYPE;
  memcpy(&out[2], nonce, 8U);
  out[10] = text_length;
  if ((text != 0) && (text_length > 0U)) { memcpy(&out[11], text, text_length); }
  return (uint8_t)(PX4LITE_RPI_H1_REQUEST_PREFIX + text_length);
}

static Px4Lite_Result_t Handle(uint8_t source_system, uint8_t source_component,
                               uint8_t target_system, uint8_t target_component,
                               const uint8_t *payload, uint8_t declared_length,
                               uint8_t wire_length)
{
  return Px4Lite_RpiH1EchoHandleTunnel(source_system, source_component,
                                       target_system, target_component, 42U,
                                       PX4LITE_RPI_H1_TUNNEL_TYPE,
                                       payload, declared_length, wire_length);
}

static void AssertNoResponse(void)
{
  Px4Lite_RpiH1EchoResponseView_t response;
  assert(Px4Lite_RpiH1EchoPeekResponse(&response) == PX4LITE_NOT_READY);
}

static void AssertEcho(const uint8_t *request, uint8_t request_length,
                       uint8_t source_system, uint8_t source_component,
                       uint32_t baud_bps)
{
  Px4Lite_RpiH1EchoResponseView_t response;
  uint8_t expected[PX4LITE_RPI_H1_RESPONSE_MAX];

  assert(Handle(source_system, source_component, 42U, 193U,
                request, request_length, request_length) == PX4LITE_OK);
  assert(Px4Lite_RpiH1EchoPeekResponse(&response) == PX4LITE_OK);
  assert(response.payload_type == 0x8003U);
  assert(response.target_system == source_system);
  assert(response.target_component == source_component);
  assert(response.payload_length == (uint8_t)(request_length + 4U));
  memcpy(expected, request, request_length);
  expected[1] = PX4LITE_RPI_H1_RESPONSE_TYPE;
  RpiH1Echo_PutU32Le(&expected[request_length], baud_bps);
  assert(memcmp(response.payload, expected, response.payload_length) == 0);
  Px4Lite_RpiH1EchoResponseSent();
  AssertNoResponse();
}

/** @brief 用候选完整帧检查 MAVLink 2 裁剪与 H1 数据区的真实边界。 */
static void CheckCandidateMavlinkFrame(void)
{
  static const uint8_t request_frame[] = {
    0xFDU, 0x19U, 0x00U, 0x00U, 0x00U, 0x2AU, 0xBFU, 0x81U, 0x01U, 0x00U,
    0x03U, 0x80U, 0x2AU, 0xC1U, 0x14U, 0x01U, 0x01U, 0x08U, 0x07U, 0x06U,
    0x05U, 0x04U, 0x03U, 0x02U, 0x01U, 0x09U, 0x48U, 0x45U, 0x4CU, 0x4CU,
    0x4FU, 0x20U, 0x43U, 0x4EU, 0x53U, 0xFCU, 0x90U
  };
  mavlink_message_t message;
  mavlink_message_t reply;
  mavlink_status_t status;
  mavlink_tunnel_t tunnel;
  mavlink_tunnel_t response_packet;
  Px4Lite_RpiH1EchoResponseView_t response;
  uint8_t frame[280];
  uint16_t frame_length;
  size_t i;
  uint8_t parsed = 0U;

  memset(&message, 0, sizeof(message));
  memset(&status, 0, sizeof(status));
  for (i = 0U; i < sizeof(request_frame); ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_0, request_frame[i], &message, &status) != 0) {
      parsed++;
    }
  }
  assert(parsed == 1U);
  assert(message.msgid == MAVLINK_MSG_ID_TUNNEL);
  assert(message.len == 25U); /* 固定字段 5 + 请求数据区 20。 */
  assert(message.sysid == 42U && message.compid == 191U);
  mavlink_msg_tunnel_decode(&message, &tunnel);
  assert(tunnel.payload_length == 20U && tunnel.payload_type == 0x8003U);
  assert(Handle(message.sysid, message.compid, tunnel.target_system,
                tunnel.target_component, tunnel.payload, tunnel.payload_length,
                (uint8_t)(message.len - 5U)) == PX4LITE_OK);
  assert(Px4Lite_RpiH1EchoPeekResponse(&response) == PX4LITE_OK);
  assert(response.payload_length == 24U);

  memset(&response_packet, 0, sizeof(response_packet));
  response_packet.target_system = response.target_system;
  response_packet.target_component = response.target_component;
  response_packet.payload_type = response.payload_type;
  response_packet.payload_length = response.payload_length;
  memcpy(response_packet.payload, response.payload, response.payload_length);
  (void)mavlink_msg_tunnel_encode_chan(42U, 193U, MAVLINK_COMM_0, &reply,
                                       &response_packet);
  frame_length = mavlink_msg_to_send_buffer(frame, &reply);
  assert(frame_length == 39U); /* 尾部两个零字节可由 MAVLink 2 裁剪。 */
  assert(reply.sysid == 42U && reply.compid == 193U);
  assert(reply.len == 27U);
  Px4Lite_RpiH1EchoResponseSent();
}

int main(void)
{
  static const uint8_t nonce[8] = {0x08U, 0x07U, 0x06U, 0x05U,
                                   0x04U, 0x03U, 0x02U, 0x01U};
  static const uint8_t hello[] = {'H', 'E', 'L', 'L', 'O', ' ', 'C', 'N', 'S'};
  static const uint8_t chinese[] = {0xE4U, 0xBDU, 0xA0U, 0xE5U, 0xA5U, 0xBDU};
  static const uint8_t mixed[] = {'A', 0xE4U, 0xBDU, 0xA0U, 'B'};
  static const uint8_t opaque[] = {'A', 0x00U, 'B'};
  uint8_t text48[48];
  uint8_t request[128];
  uint8_t length;
  uint8_t i;
  Px4Lite_RpiH1EchoResponseView_t response;
  Px4Lite_RpiH1EchoStats_t stats;

  assert(Px4Lite_RpiH1EchoInit() == PX4LITE_OK);
  CheckCandidateMavlinkFrame();
  length = BuildRequest(request, nonce, hello, (uint8_t)sizeof(hello));
  assert(length == 20U);
  AssertEcho(request, length, 42U, 191U, 57600U);
  /* 9 月 23 日候选文档中的数据区向量是合成例，不代表现场 B0。 */
  {
    static const uint8_t expected_request[] = {
      0x01U, 0x01U, 0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U,
      0x09U, 0x48U, 0x45U, 0x4CU, 0x4CU, 0x4FU, 0x20U, 0x43U, 0x4EU, 0x53U
    };
    assert(memcmp(request, expected_request, sizeof(expected_request)) == 0);
  }

  length = BuildRequest(request, nonce, chinese, (uint8_t)sizeof(chinese));
  AssertEcho(request, length, 42U, 191U, 57600U);
  length = BuildRequest(request, nonce, mixed, (uint8_t)sizeof(mixed));
  AssertEcho(request, length, 42U, 191U, 57600U);
  /* 来源组件不写死为某个 Pi 测试值；响应回填实际来源身份。 */
  AssertEcho(request, length, 43U, 1U, 57600U);
  /* F407 只复制字节；输入有效性由上游保证，不在此解析 UTF-8。 */
  length = BuildRequest(request, nonce, opaque, (uint8_t)sizeof(opaque));
  AssertEcho(request, length, 42U, 191U, 57600U);
  length = BuildRequest(request, nonce, hello, 1U);
  AssertEcho(request, length, 42U, 191U, 57600U);
  for (i = 0U; i < 48U; ++i) { text48[i] = (uint8_t)('A' + (i % 26U)); }
  length = BuildRequest(request, nonce, text48, 48U);
  assert(length == 59U);
  AssertEcho(request, length, 42U, 191U, 57600U);

  length = BuildRequest(request, nonce, hello, (uint8_t)sizeof(hello));
  assert(Px4Lite_RpiH1EchoHandleTunnel(42U, 191U, 42U, 193U, 42U,
                                      0x8001U, request, length, length) == PX4LITE_IDLE);
  assert(Px4Lite_RpiH1EchoHandleTunnel(42U, 191U, 42U, 193U, 42U,
                                      0x8002U, request, length, length) == PX4LITE_IDLE);
  assert(Handle(42U, 191U, 0U, 193U, request, length, length) == PX4LITE_INVALID_PARAM);
  assert(Handle(42U, 191U, 42U, 0U, request, length, length) == PX4LITE_INVALID_PARAM);
  assert(Handle(42U, 191U, 43U, 193U, request, length, length) == PX4LITE_INVALID_PARAM);
  assert(Handle(42U, 191U, 42U, 191U, request, length, length) == PX4LITE_INVALID_PARAM);
  assert(Handle(0U, 191U, 42U, 193U, request, length, length) == PX4LITE_INVALID_PARAM);
  assert(Handle(42U, 0U, 42U, 193U, request, length, length) == PX4LITE_INVALID_PARAM);
  AssertNoResponse();
  /* MAVLink TUNNEL 允许外层带固定数组零填充，但内层声明长度仍须精确。 */
  assert(Handle(42U, 191U, 42U, 193U, request, length, 128U) == PX4LITE_OK);
  Px4Lite_RpiH1EchoResponseSent();

  request[0] = 2U;
  assert(Handle(42U, 191U, 42U, 193U, request, length, length) == PX4LITE_INVALID_PARAM);
  request[0] = 1U;
  request[1] = PX4LITE_RPI_H1_RESPONSE_TYPE;
  assert(Handle(42U, 191U, 42U, 193U, request, length, length) == PX4LITE_INVALID_PARAM);
  request[1] = 3U; /* 旧改参操作不得重新进入箱子端。 */
  assert(Handle(42U, 191U, 42U, 193U, request, length, length) == PX4LITE_INVALID_PARAM);
  request[1] = PX4LITE_RPI_H1_REQUEST_TYPE;

  request[10] = 0U;
  assert(Handle(42U, 191U, 42U, 193U, request, 11U, 11U) == PX4LITE_INVALID_PARAM);
  request[10] = 49U;
  assert(Handle(42U, 191U, 42U, 193U, request, 60U, 60U) == PX4LITE_INVALID_PARAM);
  request[10] = (uint8_t)sizeof(hello);
  assert(Handle(42U, 191U, 42U, 193U, request, (uint8_t)(length - 1U),
                (uint8_t)(length - 1U)) == PX4LITE_INVALID_PARAM);
  assert(Handle(42U, 191U, 42U, 193U, request, (uint8_t)(length + 1U),
                (uint8_t)(length + 1U)) == PX4LITE_INVALID_PARAM);
  assert(Handle(42U, 191U, 42U, 193U, request, length,
                (uint8_t)(length - 1U)) == PX4LITE_INVALID_PARAM);
  AssertNoResponse();

  /* 静态单槽：相同请求在待发期间去重，不同文本/nonce 不覆盖原响应。 */
  assert(Handle(42U, 191U, 42U, 193U, request, length, length) == PX4LITE_OK);
  assert(Handle(42U, 191U, 42U, 193U, request, length, length) == PX4LITE_OK);
  request[19] ^= 1U;
  assert(Handle(42U, 191U, 42U, 193U, request, length, length) == PX4LITE_BUSY);
  request[19] ^= 1U;
  request[2] ^= 1U;
  assert(Handle(42U, 191U, 42U, 193U, request, length, length) == PX4LITE_BUSY);
  request[2] ^= 1U;
  assert(Px4Lite_RpiH1EchoPeekResponse(&response) == PX4LITE_OK);
  assert(response.payload[19] == 'S');
  Px4Lite_RpiH1EchoResponseSent();
  AssertNoResponse();

  /* 不限制本地实际速率为旧草案的两档；响应必须以读取值为准。 */
  s_mock_baud_bps = 38400U;
  assert(Px4Lite_RpiH1EchoInit() == PX4LITE_OK);
  AssertEcho(request, length, 42U, 191U, 38400U);
  s_mock_get_result = PX4LITE_BUSY;
  assert(Handle(42U, 191U, 42U, 193U, request, length, length) == PX4LITE_BUSY);
  AssertNoResponse();
  Px4Lite_RpiH1EchoGetStats(&stats);
  assert(stats.current_baud_bps == 0U);
  assert(stats.rejected_request_count > 0U);
  s_mock_get_result = PX4LITE_OK;
  AssertEcho(request, length, 42U, 191U, 38400U);

  puts("H1 回显测试通过");
  return 0;
}
