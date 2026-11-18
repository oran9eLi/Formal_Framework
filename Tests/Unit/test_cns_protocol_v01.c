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
Px4Lite_Result_t Px4Lite_ControlSetMode(Px4Lite_ControlMode_t m) { (void)m; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ControlSetAttitudeTarget(int32_t r, int32_t p, int32_t y)
{ (void)r; (void)p; (void)y; return PX4LITE_OK; }
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
  memset(&payload, 0, sizeof(payload));
  payload.command = cmd; payload.target_system = target; payload.target_component = target_comp;
  payload.confirmation = confirmation; payload.param1 = p[0]; payload.param2 = p[1];
  payload.param3 = p[2]; payload.param4 = p[3]; payload.param5 = p[4]; payload.param6 = p[5]; payload.param7 = p[6];
  (void)mavlink_msg_command_long_encode_chan(source, comp, MAVLINK_COMM_0, &message, &payload);
  /* 实际RX经过紧凑帧快照后重建，不能只用原编码message绕过生产包装链。 */
  memset(&frame, 0, sizeof(frame));
  frame.rx_time_ms = test_now; frame.msg_id = message.msgid;
  frame.system_id = message.sysid; frame.component_id = message.compid;
  frame.sequence = message.seq; frame.payload_len = message.len;
  memcpy(frame.payload, _MAV_PAYLOAD(&message), message.len);
  MavRx_MessageFromFrame(&frame, &rebuilt);
  return MavRx_DecodeCommandLong(&rebuilt, PX4LITE_MAVLINK_LINK_RPI, test_now);
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
  puts("CNS V0.1 protocol: strict parameters/endpoints, ACK CRC/FIFO/deadline, disarm safety reserve, stale-clock 1979/1980/1999ms boundary, real armed bit PASS");
  return 0;
}
