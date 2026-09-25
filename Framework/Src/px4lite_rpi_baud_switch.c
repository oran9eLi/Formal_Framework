/**
 * @file px4lite_rpi_baud_switch.c
 * @brief 实现 UART6 受控的 B0 到 B1 临时波特率切换。
 *
 * @details
 * Framework 通信任务是状态唯一写者。请求仅在当前已应用速率为 115200 bit/s、
 * 目标恰为 57600 bit/s 且 TUNNEL 地址和字节长度均有效时入队。响应发送完成前
 * 不重配 UART6；一次启动最多处理一项切换，复位后 BSP 回到编译期 B0。
 */

#include "px4lite_rpi_baud_switch.h"

#include <string.h>

#include "px4lite_config.h"
#include "px4lite_platform.h"

#define RPI_BAUD_B0_BPS 115200U
#define RPI_BAUD_B1_BPS 57600U

typedef struct {
  uint8_t source_system;
  uint8_t source_component;
  uint8_t response[PX4LITE_RPI_BAUD_PAYLOAD_LENGTH];
} RpiBaudSwitchSlot_t;

static RpiBaudSwitchSlot_t s_slot;
static Px4Lite_RpiBaudSwitchStats_t s_stats;

/** @brief 从协议字节数组读取小端 32 位无符号整数。 */
static uint32_t RpiBaudSwitch_GetU32Le(const uint8_t *src)
{
  return (uint32_t)src[0] |
         ((uint32_t)src[1] << 8U) |
         ((uint32_t)src[2] << 16U) |
         ((uint32_t)src[3] << 24U);
}

/** @brief 将 32 位无符号整数按小端逐字节写入协议缓冲区。 */
static void RpiBaudSwitch_PutU32Le(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/** @brief 判定 8 字节 nonce 是否全零；全零值不作为有效动作标识。 */
static uint8_t RpiBaudSwitch_IsZeroNonce(const uint8_t *nonce)
{
  uint8_t i;

  for (i = 0U; i < PX4LITE_RPI_BAUD_NONCE_LENGTH; ++i) {
    if (nonce[i] != 0U) { return 0U; }
  }
  return 1U;
}

/** @brief 判断请求是否与当前待发槽的来源、nonce 和目标波特率完全相同。 */
static uint8_t RpiBaudSwitch_IsSamePendingRequest(uint8_t source_system,
                                                 uint8_t source_component,
                                                 const uint8_t *payload)
{
  return (uint8_t)((s_stats.state == PX4LITE_RPI_BAUD_STATE_PENDING_TX) &&
                   (s_slot.source_system == source_system) &&
                   (s_slot.source_component == source_component) &&
                   (memcmp(&s_slot.response[2], &payload[2], PX4LITE_RPI_BAUD_NONCE_LENGTH) == 0) &&
                   (memcmp(&s_slot.response[10], &payload[10], 4U) == 0));
}

/** @brief 初始化切换状态，并确认 BSP 已将 UART6 配置为基准速率 B0。 */
Px4Lite_Result_t Px4Lite_RpiBaudSwitchInit(void)
{
  uint32_t baud_bps = 0U;
  Px4Lite_Result_t result;

  memset(&s_slot, 0, sizeof(s_slot));
  memset(&s_stats, 0, sizeof(s_stats));
  result = Px4Lite_RpiMavlinkGetBaudRate(&baud_bps);
  if ((result != PX4LITE_OK) || (baud_bps != RPI_BAUD_B0_BPS)) {
    s_stats.current_baud_bps = (result == PX4LITE_OK) ? baud_bps : 0U;
    return (result == PX4LITE_OK) ? PX4LITE_NOT_READY : result;
  }
  s_stats.current_baud_bps = baud_bps;
  s_stats.state = PX4LITE_RPI_BAUD_STATE_IDLE;
  return PX4LITE_OK;
}

/** @brief 校验目标和定长载荷，仅排队接受帧，不在接收路径中重配外设。 */
Px4Lite_Result_t Px4Lite_RpiBaudSwitchHandleTunnel(uint8_t source_system,
                                                   uint8_t source_component,
                                                   uint8_t target_system,
                                                   uint8_t target_component,
                                                   uint8_t local_system,
                                                   uint16_t payload_type,
                                                   const uint8_t *payload,
                                                   uint8_t payload_length,
                                                   uint8_t wire_payload_length)
{
  uint32_t target_baud_bps;
  uint32_t current_baud_bps = 0U;
  uint8_t i;
  Px4Lite_Result_t result;

  if (payload_type != PX4LITE_RPI_BAUD_TUNNEL_TYPE) { return PX4LITE_IDLE; }
  if ((source_system == 0U) || (source_component == 0U) || (local_system == 0U) ||
      (target_system != local_system) ||
      (target_component != PX4LITE_RPI_MAVLINK_COMPONENT_ID) ||
      (payload == 0) || (payload_length != PX4LITE_RPI_BAUD_PAYLOAD_LENGTH) ||
      (wire_payload_length > payload_length) || (wire_payload_length < 12U)) {
    s_stats.rejected_count++;
    return PX4LITE_INVALID_PARAM;
  }

  /* MAVLink 2 可省略 TUNNEL 数据区末尾的零；省略的字节必须确为协议规定的零。 */
  for (i = wire_payload_length; i < payload_length; ++i) {
    if (payload[i] != 0U) {
      s_stats.rejected_count++;
      return PX4LITE_INVALID_PARAM;
    }
  }

  if ((payload[0] != PX4LITE_RPI_BAUD_VERSION) ||
      (payload[1] != PX4LITE_RPI_BAUD_REQUEST_TYPE) ||
      (RpiBaudSwitch_IsZeroNonce(&payload[2]) != 0U)) {
    s_stats.rejected_count++;
    return PX4LITE_INVALID_PARAM;
  }
  target_baud_bps = RpiBaudSwitch_GetU32Le(&payload[10]);
  if (target_baud_bps != RPI_BAUD_B1_BPS) {
    s_stats.rejected_count++;
    return PX4LITE_INVALID_PARAM;
  }

  if (s_stats.state == PX4LITE_RPI_BAUD_STATE_PENDING_TX) {
    if (RpiBaudSwitch_IsSamePendingRequest(source_system, source_component, payload) != 0U) {
      s_stats.duplicate_count++;
      return PX4LITE_OK;
    }
    s_stats.busy_count++;
    return PX4LITE_BUSY;
  }
  if (s_stats.state != PX4LITE_RPI_BAUD_STATE_IDLE) {
    s_stats.rejected_count++;
    return PX4LITE_INVALID_PARAM;
  }

  result = Px4Lite_RpiMavlinkGetBaudRate(&current_baud_bps);
  if ((result != PX4LITE_OK) || (current_baud_bps != RPI_BAUD_B0_BPS)) {
    s_stats.current_baud_bps = (result == PX4LITE_OK) ? current_baud_bps : 0U;
    s_stats.rejected_count++;
    return (result == PX4LITE_OK) ? PX4LITE_NOT_READY : result;
  }

  s_slot.source_system = source_system;
  s_slot.source_component = source_component;
  s_slot.response[0] = PX4LITE_RPI_BAUD_VERSION;
  s_slot.response[1] = PX4LITE_RPI_BAUD_ACCEPT_TYPE;
  memcpy(&s_slot.response[2], &payload[2], PX4LITE_RPI_BAUD_NONCE_LENGTH);
  RpiBaudSwitch_PutU32Le(&s_slot.response[10], target_baud_bps);
  s_stats.current_baud_bps = current_baud_bps;
  s_stats.state = PX4LITE_RPI_BAUD_STATE_PENDING_TX;
  s_stats.accepted_count++;
  return PX4LITE_OK;
}

/** @brief 返回接受响应是否等待 CommTask 在 B0 下发送。 */
uint8_t Px4Lite_RpiBaudSwitchIsPending(void)
{
  return (uint8_t)(s_stats.state == PX4LITE_RPI_BAUD_STATE_PENDING_TX);
}

/** @brief 暴露待发响应视图供 MAVLink 发送器逐字段编码。 */
Px4Lite_Result_t Px4Lite_RpiBaudSwitchPeekResponse(Px4Lite_RpiBaudSwitchResponseView_t *out)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  if (s_stats.state != PX4LITE_RPI_BAUD_STATE_PENDING_TX) { return PX4LITE_NOT_READY; }
  out->payload = s_slot.response;
  out->payload_type = PX4LITE_RPI_BAUD_TUNNEL_TYPE;
  out->payload_length = PX4LITE_RPI_BAUD_PAYLOAD_LENGTH;
  out->target_system = s_slot.source_system;
  out->target_component = s_slot.source_component;
  return PX4LITE_OK;
}

/** @brief 旧速率整帧发送成功后应用 B1，并记录实际配置结果。 */
Px4Lite_Result_t Px4Lite_RpiBaudSwitchResponseSent(void)
{
  uint32_t actual_baud_bps = 0U;
  Px4Lite_Result_t result;

  if (s_stats.state != PX4LITE_RPI_BAUD_STATE_PENDING_TX) { return PX4LITE_NOT_READY; }
  s_stats.response_sent_count++;
  result = Px4Lite_RpiMavlinkSetBaudRate(RPI_BAUD_B1_BPS);
  if (result == PX4LITE_OK) {
    s_stats.state = PX4LITE_RPI_BAUD_STATE_APPLIED;
    s_stats.apply_success_count++;
  } else {
    s_stats.state = PX4LITE_RPI_BAUD_STATE_APPLY_FAILED;
    s_stats.apply_failure_count++;
  }
  if (Px4Lite_RpiMavlinkGetBaudRate(&actual_baud_bps) == PX4LITE_OK) {
    s_stats.current_baud_bps = actual_baud_bps;
  } else {
    s_stats.current_baud_bps = 0U;
  }
  memset(&s_slot, 0, sizeof(s_slot));
  return result;
}

/** @brief 复制切换计数和状态，并刷新 UART6 当前已应用波特率。 */
void Px4Lite_RpiBaudSwitchGetStats(Px4Lite_RpiBaudSwitchStats_t *out)
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
