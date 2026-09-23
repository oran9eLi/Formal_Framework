/**
 * @file px4lite_rpi_h1_echo.c
 * @brief 实现 USART6 上的 H1 最小受控回显。
 *
 * @details
 * CommTask 单写静态响应槽；MAVLink CRC 由上游解析器校验。请求中 8 字节 nonce
 * 和 1—48 字节文本均原样复制，只有尾部波特率按小端编码。无改参和持久化路径。
 */

#include "px4lite_rpi_h1_echo.h"

#include <string.h>

#include "px4lite_config.h"
#include "px4lite_platform.h"

/** @brief CommTask 独占的单槽响应和待发请求身份。 */
typedef struct {
  uint8_t valid;
  uint8_t source_system;
  uint8_t source_component;
  uint8_t payload_length;
  uint8_t response[PX4LITE_RPI_H1_RESPONSE_MAX];
} RpiH1Echo_ResponseSlot_t;

static RpiH1Echo_ResponseSlot_t s_response;
static Px4Lite_RpiH1EchoStats_t s_stats;

/** @brief 将已应用波特率逐字节写入响应尾部，避免发送结构体内存。 */
static void RpiH1Echo_PutU32Le(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/** @brief 统一计数并拒绝非法请求；不得产生错误响应或其他副作用。 */
static Px4Lite_Result_t RpiH1Echo_Reject(void)
{
  s_stats.rejected_request_count++;
  return PX4LITE_INVALID_PARAM;
}

/** @brief 检查响应槽是否已有同来源、同 nonce 和同文本的请求。 */
static uint8_t RpiH1Echo_MatchesPending(uint8_t source_system,
                                        uint8_t source_component,
                                        const uint8_t *payload,
                                        uint8_t payload_length)
{
  if ((s_response.valid == 0U) || (s_response.source_system != source_system) ||
      (s_response.source_component != source_component) ||
      (s_response.payload_length != (uint8_t)(payload_length + PX4LITE_RPI_H1_BAUD_BYTES))) {
    return 0U;
  }
  /* version 相同、type 已校验；从 nonce 到文本末尾逐字节比较。 */
  return (memcmp(&s_response.response[2], &payload[2], (size_t)payload_length - 2U) == 0) ? 1U : 0U;
}

Px4Lite_Result_t Px4Lite_RpiH1EchoInit(void)
{
  uint32_t baud_bps = 0U;

  memset(&s_response, 0, sizeof(s_response));
  memset(&s_stats, 0, sizeof(s_stats));
  if ((Px4Lite_RpiMavlinkGetBaudRate(&baud_bps) != PX4LITE_OK) || (baud_bps == 0U)) {
    return PX4LITE_NOT_READY;
  }
  s_stats.current_baud_bps = baud_bps;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_RpiH1EchoHandleTunnel(uint8_t source_system,
                                               uint8_t source_component,
                                               uint8_t target_system,
                                               uint8_t target_component,
                                               uint8_t local_system,
                                               uint16_t payload_type,
                                               const uint8_t *payload,
                                               uint8_t payload_length,
                                               uint8_t wire_payload_length)
{
  uint8_t text_length;
  uint32_t baud_bps = 0U;
  Px4Lite_Result_t result;

  if (payload_type != PX4LITE_RPI_H1_TUNNEL_TYPE) { return PX4LITE_IDLE; }
  if ((source_system == 0U) || (source_component == 0U) || (local_system == 0U) ||
      (target_system != local_system) ||
      (target_component != PX4LITE_RPI_MAVLINK_COMPONENT_ID)) {
    return RpiH1Echo_Reject();
  }
  if ((payload == 0) ||
      (payload_length < (PX4LITE_RPI_H1_REQUEST_PREFIX + 1U)) ||
      (payload_length > (PX4LITE_RPI_H1_REQUEST_PREFIX + PX4LITE_RPI_H1_TEXT_MAX_BYTES)) ||
      (wire_payload_length < payload_length)) {
    return RpiH1Echo_Reject();
  }
  if ((payload[0] != PX4LITE_RPI_H1_VERSION) ||
      (payload[1] != PX4LITE_RPI_H1_REQUEST_TYPE)) {
    return RpiH1Echo_Reject();
  }
  text_length = payload[10];
  if ((text_length == 0U) || (text_length > PX4LITE_RPI_H1_TEXT_MAX_BYTES) ||
      (payload_length != (uint8_t)(PX4LITE_RPI_H1_REQUEST_PREFIX + text_length))) {
    return RpiH1Echo_Reject();
  }
  if (s_response.valid != 0U) {
    if (RpiH1Echo_MatchesPending(source_system, source_component, payload, payload_length) != 0U) {
      s_stats.duplicate_count++;
      return PX4LITE_OK;
    }
    s_stats.busy_count++;
    return PX4LITE_BUSY;
  }

  result = Px4Lite_RpiMavlinkGetBaudRate(&baud_bps);
  if ((result != PX4LITE_OK) || (baud_bps == 0U)) {
    s_stats.current_baud_bps = 0U;
    s_stats.rejected_request_count++;
    return (result == PX4LITE_OK) ? PX4LITE_NOT_READY : result;
  }

  memset(&s_response, 0, sizeof(s_response));
  s_response.valid = 1U;
  s_response.source_system = source_system;
  s_response.source_component = source_component;
  s_response.payload_length = (uint8_t)(payload_length + PX4LITE_RPI_H1_BAUD_BYTES);
  /* 版本、nonce、N 和文本保持原字节；只有类型从请求 1 改为响应 2。 */
  memcpy(s_response.response, payload, payload_length);
  s_response.response[1] = PX4LITE_RPI_H1_RESPONSE_TYPE;
  RpiH1Echo_PutU32Le(&s_response.response[payload_length], baud_bps);

  s_stats.current_baud_bps = baud_bps;
  s_stats.handled_request_count++;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_RpiH1EchoPeekResponse(Px4Lite_RpiH1EchoResponseView_t *out)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  if (s_response.valid == 0U) { return PX4LITE_NOT_READY; }
  out->payload = s_response.response;
  out->payload_type = PX4LITE_RPI_H1_TUNNEL_TYPE;
  out->payload_length = s_response.payload_length;
  out->target_system = s_response.source_system;
  out->target_component = s_response.source_component;
  return PX4LITE_OK;
}

void Px4Lite_RpiH1EchoResponseSent(void)
{
  if (s_response.valid == 0U) { return; }
  memset(&s_response, 0, sizeof(s_response));
  s_stats.response_sent_count++;
}

void Px4Lite_RpiH1EchoGetStats(Px4Lite_RpiH1EchoStats_t *out)
{
  uint32_t baud_bps = 0U;

  if (out == 0) { return; }
  if (Px4Lite_RpiMavlinkGetBaudRate(&baud_bps) == PX4LITE_OK) {
    s_stats.current_baud_bps = baud_bps;
  } else {
    s_stats.current_baud_bps = 0U;
  }
  *out = s_stats;
}
