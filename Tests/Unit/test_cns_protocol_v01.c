/**
 * @file test_cns_protocol_v01.c
 * @brief CNS V0.1真实MAVLink编解码、命令受理、FIFO应答与armed心跳回归。
 * @details 仅替换Control状态机及UART物理边界；真实电机/声光时序由独立集成测试覆盖。
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "px4lite_config.h"
#include "px4lite_control.h"
#include "px4lite_startup_tone.h"
#define MAVLINK_COMM_NUM_BUFFERS 1
#include "../../Framework/Src/px4lite_command.c"
#include "../../Framework/Src/px4lite_mavlink_tx.c"
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "../../Framework/Src/px4lite_mavlink_rx.c"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

static uint8_t test_system = 42U, test_armed, test_reason;
static uint8_t test_action, test_motor, test_notifications;
static uint16_t test_pulse;
static unsigned test_control_calls, test_uart_calls, test_released;
static uint32_t test_token = 123U, test_last_release;
static uint32_t test_now;
static uint32_t test_uart_elapsed_ms, test_last_release_ms;
static Px4Lite_Result_t test_control_result = PX4LITE_OK, test_uart_result = PX4LITE_OK;
static uint8_t test_wire[MAVLINK_MAX_PACKET_LEN];
static uint16_t test_wire_len;
static unsigned test_switch_requests, test_switch_acks;
static Px4Lite_ModuleId_t test_switch_id, test_switch_ack_id;
static uint8_t test_switch_enabled, test_switch_ack_enabled;
static Px4Lite_Result_t test_switch_result = PX4LITE_OK;
static uint8_t test_request_mavlink1;
static uint32_t test_disabled_modules;
static Px4Lite_State_t test_lora_state = PX4LITE_STATE_ONLINE;
static Px4Lite_MavlinkLink_t test_request_link = PX4LITE_MAVLINK_LINK_RPI;

/** @brief 模拟独立Control事实，测试边界不以ACK受理推断真实执行。 */
Px4Lite_Result_t Px4Lite_ControlCnsRequest(Px4Lite_CnsAction_t action, uint8_t motor,
    uint16_t pulse, uint8_t *reason, uint32_t *token)
{
  ++test_control_calls; test_action = (uint8_t)action; test_motor = motor; test_pulse = pulse;
  *reason = test_reason; *token = test_control_result == PX4LITE_OK ? test_token : 0U;
  return test_control_result;
}
void Px4Lite_ControlCnsAckSent(uint32_t token)
{ ++test_released; test_last_release = token; test_last_release_ms = test_now; }
uint8_t Px4Lite_ControlCnsArmed(void) { return test_armed; }
/** @brief 记录命令边界传递的模块请求，由独立测试覆盖实际模块服务。 */
Px4Lite_Result_t Px4Lite_ModuleExperimentRequest(Px4Lite_ModuleId_t id, uint8_t enabled, uint32_t now_ms)
{
  ++test_switch_requests;
  test_switch_id = id;
  test_switch_enabled = enabled;
  assert(now_ms == test_now);
  return test_switch_result;
}

/** @brief 记录串口发送成功才释放的切屏和声光事件。 */
void Px4Lite_ModuleExperimentAck(Px4Lite_ModuleId_t id, uint8_t enabled)
{
  ++test_switch_acks;
  test_switch_ack_id = id;
  test_switch_ack_enabled = enabled;
}

uint8_t Px4Lite_ModuleExperimentEnabled(Px4Lite_ModuleId_t id)
{ return (uint8_t)((test_disabled_modules & (1UL << id)) == 0U); }

/** @brief 模拟健康任务尚未来得及发布 OFF 的上一帧。 */
Px4Lite_Result_t Px4Lite_CopyHealth(Px4Lite_SystemHealth_t *health)
{
  memset(health, 0, sizeof(*health));
  health->header.valid = 1U;
  health->header.sample_time_ms = test_now;
  health->module_state[PX4LITE_MODULE_GNSS] = PX4LITE_STATE_ONLINE;
  health->module_state[PX4LITE_MODULE_IMU] = PX4LITE_STATE_ONLINE;
  health->module_state[PX4LITE_MODULE_BARO] = PX4LITE_STATE_ONLINE;
  return PX4LITE_OK;
}

void Px4Lite_LoRaGetDebugInfo(Px4Lite_CommDebugInfo_t *info)
{ memset(info, 0, sizeof(*info)); info->rx_loss_rate_x10 = 321U; }
Px4Lite_State_t Px4Lite_LoRaGetState(uint32_t now_ms) { (void)now_ms; return test_lora_state; }
uint8_t Px4Lite_LoRaIsPresent(void) { return 1U; }
uint8_t Px4Lite_IdentityGetNodeId(void) { return 13U; }

uint8_t Px4Lite_IdentityGetMavlinkSystemId(void) { return test_system; }
uint32_t Px4Lite_PlatformGetMs(void) { return test_now; }
/** @brief 保留完整线上帧；BUSY/错误不能被当作ACK发送成功。 */
Px4Lite_Result_t Px4Lite_RpiMavlinkSend(const uint8_t *data, uint16_t len)
{
  assert(len <= sizeof(test_wire)); memcpy(test_wire, data, len); test_wire_len = len;
  ++test_uart_calls; test_now += test_uart_elapsed_ms;
  return test_uart_result;
}
Px4Lite_Result_t Px4Lite_LoraSend(const uint8_t *data, uint16_t len)
{ (void)data; (void)len; assert(0); return PX4LITE_IO_ERROR; }
void Px4Lite_StartupToneNotifyRpiResult(uint8_t succeeded) { (void)succeeded; ++test_notifications; }
Px4Lite_Result_t Px4Lite_ControlSetMotorThrottles(const uint8_t p[4]) { (void)p; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ControlSetRpiMotorThrottles(const uint8_t p[4]) { (void)p; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ControlSetMotorPulses(const uint16_t p[4]) { (void)p; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ControlSetRpiMotorPulses(const uint16_t p[4]) { (void)p; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ControlSetModeAndAttitudeTarget(Px4Lite_ControlMode_t m,
    int32_t r, int32_t p, int32_t y)
{ (void)m; (void)r; (void)p; (void)y; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ControlStartAutoTakeoff(void) { return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ControlStartRpiAutoTakeoff(void) { return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ControlStartAutoLanding(void) { return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ControlStartRpiAutoLanding(void) { return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ControlEmergencyStop(uint32_t ms) { (void)ms; return PX4LITE_OK; }

/** @brief 使用独立parser重新检查UART帧CRC，不以发送缓存结构体当线上正确。 */
static mavlink_message_t DecodeWire(void)
{
  mavlink_message_t parser, decoded;
  mavlink_status_t state, status;
  unsigned i, valid = 0U;
  memset(&parser, 0, sizeof(parser)); memset(&decoded, 0, sizeof(decoded));
  memset(&state, 0, sizeof(state)); memset(&status, 0, sizeof(status));
  for (i = 0U; i < test_wire_len; ++i) {
    if (mavlink_frame_char_buffer(&parser, &state, test_wire[i], &decoded, &status) == MAVLINK_FRAMING_OK) { ++valid; }
  }
  assert(valid == 1U && decoded.magic == MAVLINK_STX && decoded.sysid == test_system && decoded.compid == 193U);
  return decoded;
}

/** @brief 通过真实标准COMMAND_LONG编码和生产RX完整参数入口。 */
static uint8_t Request(uint16_t cmd, uint8_t source, uint8_t comp, uint8_t target,
    uint8_t target_comp, uint8_t confirmation, const float p[7])
{
  mavlink_command_long_t payload;
  mavlink_message_t message;
  mavlink_message_t rebuilt;
  Px4Lite_CommRxFrame_t frame;
  mavlink_status_t *channel = mavlink_get_channel_status(MAVLINK_COMM_0);
  uint8_t flags = channel->flags;
  memset(&payload, 0, sizeof(payload));
  payload.command = cmd; payload.target_system = target; payload.target_component = target_comp;
  payload.confirmation = confirmation; payload.param1 = p[0]; payload.param2 = p[1];
  payload.param3 = p[2]; payload.param4 = p[3]; payload.param5 = p[4]; payload.param6 = p[5]; payload.param7 = p[6];
  if (test_request_mavlink1) { channel->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1; }
  (void)mavlink_msg_command_long_encode_chan(source, comp, MAVLINK_COMM_0, &message, &payload);
  channel->flags = flags;
  /* 实际RX经过紧凑帧快照后重建，不能只用原编码message绕过生产包装链。 */
  memset(&frame, 0, sizeof(frame));
  frame.rx_time_ms = test_now; frame.msg_id = message.msgid;
  frame.system_id = message.sysid; frame.component_id = message.compid;
  frame.sequence = message.seq; frame.wire_magic = message.magic; frame.payload_len = message.len;
  memcpy(frame.payload, _MAV_PAYLOAD(&message), message.len);
  MavRx_MessageFromFrame(&frame, &rebuilt);
  return MavRx_DecodeCommandLong(&rebuilt, test_request_link, test_now);
}

/** @brief 检查完整ACK字段及发送门禁；不接受IN_PROGRESS或虚假完成事件。 */
static void Ack(uint16_t cmd, uint8_t result, uint8_t reason)
{
  mavlink_message_t decoded;
  mavlink_command_ack_t packet;
  s_send_target = MAV_TX_TARGET_RPI;
  assert(MavTx_SendCnsAck(test_now) == PX4LITE_OK);
  decoded = DecodeWire(); assert(decoded.msgid == MAVLINK_MSG_ID_COMMAND_ACK);
  mavlink_msg_command_ack_decode(&decoded, &packet);
  assert(packet.command == cmd && packet.result == result && packet.progress == 255U);
  assert(packet.result_param2 == reason && packet.target_system == test_system && packet.target_component == 191U);
}

/** @brief 每次场景清空真实应答FIFO；不影响生产初始化函数。 */
static void Reset(void)
{
  s_cns_ack_read = s_cns_ack_write = s_cns_ack_count = 0U;
  s_cns_ack_overflow = s_cns_ack_expired = 0U;
  memset(s_cns_ack, 0, sizeof(s_cns_ack));
  test_control_calls = test_uart_calls = test_released = test_notifications = 0U;
  test_switch_requests = test_switch_acks = 0U;
  test_switch_result = PX4LITE_OK;
  test_request_mavlink1 = 0U;
  test_request_link = PX4LITE_MAVLINK_LINK_RPI;
  test_control_result = test_uart_result = PX4LITE_OK;
  test_reason = test_armed = 0U; test_now = 100U; test_system = 42U; test_token = 123U;
  test_uart_elapsed_ms = test_last_release_ms = test_last_release = 0U;
}

/** @brief 普通应答占满15项后，锁定仍进入Control并使用安全保留项，不覆盖原FIFO。 */
static void TestDisarmUsesSafetyAckReserve(void)
{
  const float params[7] = {0}; unsigned i;
  Reset();
  for (i = 0U; i < PX4LITE_CNS_ACK_QUEUE_DEPTH - 1U; ++i) {
    assert(Px4Lite_MavlinkQueueCnsAck((uint16_t)(31000U+i),1U,2U,42U,0U,test_now) == PX4LITE_OK);
  }
  assert(!Px4Lite_MavlinkCnsAckAvailable());
  assert(Request(31094U,42U,191U,42U,193U,0U,params));
  assert(test_control_calls == 1U && test_action == PX4LITE_CNS_DISARM);
  assert(s_cns_ack_count == PX4LITE_CNS_ACK_QUEUE_DEPTH && !test_released);
  for (i = 0U; i < PX4LITE_CNS_ACK_QUEUE_DEPTH - 1U; ++i) { Ack((uint16_t)(31000U+i),1U,2U); }
  assert(!test_released); Ack(31094U,0U,0U);
  assert(test_released == 1U && test_last_release == test_token && !s_cns_ack_count);
}

/** @brief 真实平台时钟覆盖过时调用参数；发送预算不足时不发ACK、不释放动作令牌。 */
static void TestAckDeadlineUsesCurrentClockAndUartBudget(void)
{
  const float params[7] = {0};
  const uint32_t ages[] = {1980U,1999U};
  unsigned i; uint32_t received_ms;
  assert(PX4LITE_CNS_ACK_TIMEOUT_MS - PX4LITE_RPI_MAVLINK_UART_TIMEOUT_MS == ages[0]);
  for (i = 0U; i < sizeof(ages)/sizeof(ages[0]); ++i) {
    Reset(); received_ms = test_now;
    assert(Request(31093U,42U,191U,42U,193U,0U,params));
    test_now = received_ms + ages[i];
    assert(MavTx_SendCnsAck(received_ms) == PX4LITE_IDLE);
    assert(!test_uart_calls && !test_released && s_cns_ack_expired == 1U && !s_cns_ack_count);
  }
  Reset(); received_ms = test_now;
  assert(Request(31093U,42U,191U,42U,193U,0U,params));
  test_now = received_ms + 1979U;
  test_uart_elapsed_ms = PX4LITE_RPI_MAVLINK_UART_TIMEOUT_MS;
  /* 1979ms尚有完整20ms发送预算；阻塞返回后1999ms，才提交同一令牌。 */
  s_send_target = MAV_TX_TARGET_RPI;
  assert(MavTx_SendCnsAck(received_ms) == PX4LITE_OK);
  assert(test_uart_calls == 1U && test_released == 1U && test_last_release == test_token);
  assert(test_last_release_ms == received_ms + 1999U && test_last_release_ms < received_ms + PX4LITE_CNS_ACK_TIMEOUT_MS);
  assert(!s_cns_ack_expired && !s_cns_ack_count);
  {
    mavlink_message_t decoded = DecodeWire(); mavlink_command_ack_t packet;
    mavlink_msg_command_ack_decode(&decoded,&packet);
    assert(decoded.msgid == MAVLINK_MSG_ID_COMMAND_ACK && packet.command == 31093U);
    assert(packet.result == MAV_RESULT_ACCEPTED && packet.result_param2 == 0 && packet.progress == 255U);
  }
}

/** @brief 五个模块编号及开启和关闭都应返回原路受理，发送ACK后只释放一次事件。 */
static void TestModuleSwitchAccepted(void)
{
  static const uint8_t ids[] = {0U, 1U, 2U, 4U, 7U};
  float p[7] = {0};
  unsigned i, enabled, count = 0U;
  Reset();
  test_system = 57U;
  for (i = 0U; i < sizeof(ids) / sizeof(ids[0]); ++i) {
    p[0] = (float)ids[i];
    for (enabled = 0U; enabled <= 1U; ++enabled) {
      p[1] = (float)enabled;
      assert(Request(31015U, 57U, 191U, 57U, 193U, 0U, p));
      assert(test_switch_requests == count + 1U && test_switch_id == ids[i] && test_switch_enabled == enabled);
      assert(test_switch_acks == count && test_control_calls == 0U);
      Ack(31015U, 0U, 0U);
      ++count;
      assert(test_switch_acks == count && test_switch_ack_id == ids[i] && test_switch_ack_enabled == enabled);
    }
  }
}

/** @brief 地址、参数、模块无法受理均不释放切屏或声光，明确拒绝后仍由Pi通知失败。 */
static void TestModuleSwitchRejected(void)
{
  static const float bad_ids[] = {-1.0f, 1.5f, 3.0f, 5.0f, 6.0f, 8.0f, NAN, INFINITY};
  static const float bad_values[] = {-1.0f, 0.5f, 2.0f, NAN, INFINITY};
  float p[7] = {0};
  unsigned i;
  Reset();
  assert(!Request(31015U, 41U, 191U, 42U, 193U, 0U, p));
  assert(!Request(31015U, 42U, 192U, 42U, 193U, 0U, p));
  assert(!Request(31015U, 42U, 191U, 0U, 193U, 0U, p));
  assert(!Request(31015U, 42U, 191U, 42U, 0U, 0U, p));
  test_request_mavlink1 = 1U;
  assert(!Request(31015U, 42U, 191U, 42U, 193U, 0U, p));
  test_request_mavlink1 = 0U;
  test_request_link = PX4LITE_MAVLINK_LINK_LORA;
  assert(!Request(31015U, 42U, 191U, 42U, 193U, 0U, p));
  test_request_link = PX4LITE_MAVLINK_LINK_RPI;
  assert(!s_cns_ack_count);
  assert(Request(31015U, 42U, 191U, 42U, 193U, 1U, p)); Ack(31015U, 2U, 3U);
  for (i = 0U; i < sizeof(bad_ids) / sizeof(bad_ids[0]); ++i) {
    p[0] = bad_ids[i];
    assert(Request(31015U, 42U, 191U, 42U, 193U, 0U, p)); Ack(31015U, 2U, 3U);
  }
  p[0] = 4.0f;
  for (i = 0U; i < sizeof(bad_values) / sizeof(bad_values[0]); ++i) {
    p[1] = bad_values[i];
    assert(Request(31015U, 42U, 191U, 42U, 193U, 0U, p)); Ack(31015U, 2U, 3U);
  }
  p[1] = 1.0f;
  for (i = 2U; i < 7U; ++i) {
    p[i] = 1.0f;
    assert(Request(31015U, 42U, 191U, 42U, 193U, 0U, p)); Ack(31015U, 2U, 3U);
    p[i] = 0.0f;
  }
  assert(!test_switch_requests && !test_switch_acks && !test_notifications);
  test_switch_result = PX4LITE_NOT_READY;
  assert(Request(31015U, 42U, 191U, 42U, 193U, 0U, p)); Ack(31015U, 2U, 4U);
  test_switch_result = PX4LITE_BUSY;
  assert(Request(31015U, 42U, 191U, 42U, 193U, 0U, p)); Ack(31015U, 1U, 2U);
  test_switch_result = PX4LITE_IO_ERROR;
  assert(Request(31015U, 42U, 191U, 42U, 193U, 0U, p)); Ack(31015U, 4U, 0U);
  assert(test_switch_requests == 3U && !test_switch_acks && !test_notifications);
}

/** @brief 串口失败可重试且过期静默，FIFO拥塞不能启动新动作。 */
static void TestModuleSwitchAckGate(void)
{
  const float p[7] = {7.0f, 1.0f, 0.0f};
  unsigned i;
  Reset();
  assert(Request(31015U, 42U, 191U, 42U, 193U, 0U, p));
  test_uart_result = PX4LITE_IO_ERROR;
  s_send_target = MAV_TX_TARGET_RPI;
  assert(MavTx_SendCnsAck(test_now) == PX4LITE_IO_ERROR && !test_switch_acks && s_cns_ack_count == 1U);
  test_uart_result = PX4LITE_OK;
  Ack(31015U, 0U, 0U);
  assert(test_switch_acks == 1U && MavTx_SendCnsAck(test_now) == PX4LITE_IDLE);
  Reset();
  assert(Request(31015U, 42U, 191U, 42U, 193U, 0U, p));
  test_now += PX4LITE_CNS_ACK_TIMEOUT_MS;
  assert(MavTx_SendCnsAck(test_now) == PX4LITE_IDLE && !test_uart_calls && !test_switch_acks);
  Reset();
  assert(Request(31015U, 42U, 191U, 42U, 193U, 0U, p));
  test_uart_elapsed_ms = PX4LITE_CNS_ACK_TIMEOUT_MS;
  s_send_target = MAV_TX_TARGET_RPI;
  assert(MavTx_SendCnsAck(test_now) == PX4LITE_OK && !test_switch_acks);
  Reset();
  for (i = 0U; i < PX4LITE_CNS_ACK_QUEUE_DEPTH - 1U; ++i) {
    assert(Px4Lite_MavlinkQueueCnsAck(31093U, 1U, 2U, 42U, 0U, test_now) == PX4LITE_OK);
  }
  assert(!Request(31015U, 42U, 191U, 42U, 193U, 0U, p));
  assert(!test_switch_requests && !test_switch_acks);
}

/** @brief 真实串口线帧区分关闭、在位和通信故障。 */
static void TestModuleSwitchStatus(void)
{
  static const Px4Lite_ModuleId_t sensors[] = {PX4LITE_MODULE_GNSS, PX4LITE_MODULE_IMU, PX4LITE_MODULE_BARO};
  static const uint32_t flags[] = {MAV_SYS_STATUS_SENSOR_GPS, MAV_SYS_STATUS_SENSOR_3D_GYRO |
    MAV_SYS_STATUS_SENSOR_3D_ACCEL, MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE};
  const uint32_t all_sensors = flags[0] | flags[1] | flags[2];
  const uint32_t lora_metadata = 321U | (13UL << 16U) | (1UL << 24U);
  mavlink_message_t decoded;
  mavlink_sys_status_t system;
  mavlink_named_value_int_t named;
  unsigned i;

  Reset();
  s_send_target = MAV_TX_TARGET_RPI;
  for (i = 0U; i < sizeof(sensors) / sizeof(sensors[0]); ++i) {
    test_disabled_modules = 1UL << sensors[i];
    assert(MavTx_SendSystemStatus(test_now) == PX4LITE_OK);
    decoded = DecodeWire();
    assert(decoded.msgid == MAVLINK_MSG_ID_SYS_STATUS);
    mavlink_msg_sys_status_decode(&decoded, &system);
    assert(system.onboard_control_sensors_present == all_sensors);
    assert(system.onboard_control_sensors_enabled == (all_sensors & ~flags[i]));
    assert(system.onboard_control_sensors_health == (all_sensors & ~flags[i]));
  }

  test_disabled_modules = 1UL << PX4LITE_MODULE_LORA;
  test_lora_state = PX4LITE_STATE_FAILED;
  assert(MavTx_SendRpiLoraStatus(test_now) == PX4LITE_OK);
  decoded = DecodeWire();
  assert(decoded.msgid == MAVLINK_MSG_ID_NAMED_VALUE_INT);
  mavlink_msg_named_value_int_decode(&decoded, &named);
  assert(memcmp(named.name, "LORASTAT", 8U) == 0);
  assert((uint32_t)named.value == (lora_metadata | ((uint32_t)PX4LITE_STATE_DISABLED << 25U)));

  test_disabled_modules = 0U;
  assert(MavTx_SendRpiLoraStatus(test_now) == PX4LITE_OK);
  decoded = DecodeWire();
  mavlink_msg_named_value_int_decode(&decoded, &named);
  assert((uint32_t)named.value == (lora_metadata | ((uint32_t)PX4LITE_STATE_FAILED << 25U)));
}

int main(void)
{
  static const uint16_t commands[] = {31093U,31091U,31092U,31094U,31090U,31014U};
  float p[7] = {0};
  unsigned i, j;
  mavlink_message_t decoded;
  mavlink_heartbeat_t heartbeat;
  Reset();
  for (i = 0U; i < 6U; ++i) {
    memset(p, 0, sizeof(p)); if (i == 5U) { p[0] = 2.0f; p[1] = 1300.0f; }
    assert(Request(commands[i],42U,191U,42U,193U,0U,p));
    assert(test_action == i+1U && test_notifications == 0U);
    assert(test_released == i); Ack(commands[i],0U,0U); assert(test_released == i+1U);
  }
  assert(test_motor == 1U && test_pulse == 1300U);
  memset(p, 0, sizeof(p)); Reset();
  assert(!Request(31093U,9U,191U,42U,193U,0U,p));
  assert(!Request(31093U,42U,7U,42U,193U,0U,p));
  assert(!Request(31093U,42U,191U,0U,193U,0U,p));
  assert(!Request(31093U,42U,191U,42U,0U,0U,p));
  assert(!Request(31093U,42U,191U,42U,191U,0U,p));
  assert(test_control_calls == 0U && s_cns_ack_count == 0U);
  assert(Request(31093U,42U,191U,42U,193U,1U,p)); Ack(31093U,2U,3U);
  for (j = 0U; j < 7U; ++j) {
    p[j] = 1.0f; assert(Request(31091U,42U,191U,42U,193U,0U,p)); Ack(31091U,2U,3U); p[j] = 0.0f;
  }
  {
    const float bad[] = {0.0f, 5.0f, 1.5f, NAN, INFINITY};
    for (j = 0U; j < sizeof(bad)/sizeof(bad[0]); ++j) {
      p[0] = bad[j]; p[1] = 1300.0f;
      assert(Request(31014U,42U,191U,42U,193U,0U,p)); Ack(31014U,2U,3U);
    }
    p[0] = 1.0f;
    p[1] = 1300.5f; assert(Request(31014U,42U,191U,42U,193U,0U,p)); Ack(31014U,2U,3U);
    p[1] = NAN; assert(Request(31014U,42U,191U,42U,193U,0U,p)); Ack(31014U,2U,3U);
  }
  assert(test_control_calls == 0U && test_notifications == 0U);
  memset(p, 0, sizeof(p));
  test_control_result = PX4LITE_NOT_READY; test_reason = 1U;
  assert(Request(31091U,42U,191U,42U,193U,0U,p)); Ack(31091U,2U,1U);
  test_control_result = PX4LITE_BUSY; test_reason = 2U;
  assert(Request(31091U,42U,191U,42U,193U,0U,p)); Ack(31091U,1U,2U);
  test_control_result = PX4LITE_IO_ERROR; test_reason = 5U;
  assert(Request(31091U,42U,191U,42U,193U,0U,p)); Ack(31091U,4U,5U);
  assert(Request(31999U,42U,191U,42U,193U,0U,p)); Ack(31999U,3U,0U);
  Reset(); assert(Request(31093U,42U,191U,42U,193U,0U,p));
  test_uart_result = PX4LITE_BUSY; s_send_target = MAV_TX_TARGET_RPI;
  assert(MavTx_SendCnsAck(test_now) == PX4LITE_BUSY && test_released == 0U && s_cns_ack_count == 1U);
  test_uart_result = PX4LITE_IO_ERROR;
  assert(MavTx_SendCnsAck(test_now) == PX4LITE_IO_ERROR && test_released == 0U && s_cns_ack_count == 1U);
  test_uart_result = PX4LITE_OK; Ack(31093U,0U,0U); assert(test_released == 1U);
  Reset();
  for (i=0U; i<PX4LITE_CNS_ACK_QUEUE_DEPTH-1U; ++i) {
    assert(Px4Lite_MavlinkQueueCnsAck((uint16_t)(31000U+i),1U,2U,42U,0U,test_now) == PX4LITE_OK);
  }
  assert(!Px4Lite_MavlinkCnsAckAvailable());
  assert(!Request(31093U,42U,191U,42U,193U,0U,p) && test_control_calls == 0U);
  assert(Request(31090U,42U,191U,42U,193U,0U,p) && test_action == PX4LITE_CNS_EMERGENCY_STOP);
  for (i=0U; i<PX4LITE_CNS_ACK_QUEUE_DEPTH-1U; ++i) { Ack((uint16_t)(31000U+i),1U,2U); }
  Ack(31090U,0U,0U);
  Reset(); assert(Request(31093U,42U,191U,42U,193U,0U,p));
  test_now += PX4LITE_CNS_ACK_TIMEOUT_MS; assert(MavTx_SendCnsAck(test_now) == PX4LITE_IDLE);
  assert(!test_uart_calls && !test_released && s_cns_ack_expired == 1U);
  Reset(); s_send_target = MAV_TX_TARGET_RPI;
  assert(MavTx_SendHeartbeat(test_now) == PX4LITE_OK); decoded = DecodeWire();
  mavlink_msg_heartbeat_decode(&decoded,&heartbeat); assert(!(heartbeat.base_mode & 128U));
  test_armed = 1U; assert(MavTx_SendHeartbeat(test_now) == PX4LITE_OK); decoded = DecodeWire();
  mavlink_msg_heartbeat_decode(&decoded,&heartbeat); assert(heartbeat.base_mode & 128U);
  {
    const uint8_t failure[128] = {1U,0U};
    assert(Px4Lite_CommandHandleRpiResultTunnel(42U,191U,42U,193U,0x8040U,failure,2U,1U,PX4LITE_MAVLINK_LINK_RPI) == PX4LITE_OK);
    assert(test_notifications == 1U);
  }
  TestDisarmUsesSafetyAckReserve(); TestAckDeadlineUsesCurrentClockAndUartBudget();
  TestModuleSwitchAccepted(); TestModuleSwitchRejected(); TestModuleSwitchAckGate();
  TestModuleSwitchStatus();
  puts("CNS V0.1 protocol: motor controls, module switch/status, strict parameters/endpoints, ACK CRC/FIFO/deadline and safety reserve PASS");
  return 0;
}
