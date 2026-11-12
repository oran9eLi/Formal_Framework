/**
 * @file test_single_supply_mavlink.c
 * @brief 使用真实 MAVLink 编码与 LoRa 实验组调度验证单路供电协议。
 * @details 平台出口在本测试内捕获串行帧，再用 MAVLink 解析器核验 CRC 和字段；不访问硬件。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Framework/Src/px4lite_mavlink_tx.c"

/** @brief 本夹具仅验证电源编码与初始化；GNSS统一出口由独立真实模块回归覆盖。 */
void Px4Lite_GnssUplinkInit(uint32_t now_ms) { (void)now_ms; }
/** @brief 电源独立夹具不启动原始帧通道，其真实生命周期由RAW专项覆盖。 */
void Px4Lite_GnssRawUplinkInit(uint32_t now_ms) { (void)now_ms; }

static uint32_t s_now = 1000U;
static unsigned s_copies[2], s_lora_frames, s_rpi_frames, s_result_calls, s_complete_calls;
static uint8_t s_pending, s_current_valid[2] = {1U, 1U};
static uint8_t s_available[2] = {1U, 1U};
static Px4Lite_Result_t s_last_result;
static mavlink_message_t s_captured;
static uint8_t s_lora_ids[8];

uint8_t Px4Lite_IdentityGetMavlinkSystemId(void) { return 42U; }
void Px4Lite_IdentityFormatVendorProductId(char *out, uint8_t capacity)
{ memset(out, 0, capacity); }
Px4Lite_Result_t Px4Lite_RpiMavlinkInit(void) { return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RpiH1EchoInit(void) { return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RpiBaudSwitchInit(void) { return PX4LITE_OK; }
void Px4Lite_LoraLabInit(uint32_t now_ms) { (void)now_ms; }
uint8_t Px4Lite_LoRaIsTxIdle(void) { return 1U; }
uint8_t Px4Lite_LoraLabPending(Px4Lite_LabData_t *data)
{ *data = PX4LITE_LAB_POWER; return s_pending; }
void Px4Lite_LoraLabTxResult(Px4Lite_Result_t result, uint8_t complete, uint32_t now_ms)
{
  (void)now_ms; ++s_result_calls; s_last_result = result;
  if (complete) { ++s_complete_calls; s_pending = 0U; }
}

/** @brief 故意让禁用组仍有缓存，用调用计数证明禁用门控不会借用旧数据。 */
static Px4Lite_Result_t TestCopy(unsigned channel, Px4Lite_BatteryStatus_t *out)
{
  ++s_copies[channel];
  if (!s_available[channel]) { return PX4LITE_NOT_READY; }
  memset(out, 0, sizeof(*out));
  out->header.valid = 1U;
  out->header.flags = PX4LITE_DATA_VALID;
  out->header.sample_time_ms = s_now;
  out->header.sequence = s_now + 1U;
  out->voltage_mv = 12000U + 500U * channel;
  out->current_ma = 100 + (int32_t)channel * 300;
  out->current_valid = s_current_valid[channel];
  out->percent = 77U;
  return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out) { return TestCopy(0U, out); }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out) { return TestCopy(1U, out); }
Px4Lite_Result_t Px4Lite_CopyHealth(Px4Lite_SystemHealth_t *out)
{
  unsigned i;
  memset(out, 0, sizeof(*out));
  out->header.valid = 1U; out->header.sample_time_ms = s_now;
  for (i = 0U; i < PX4LITE_MODULE_COUNT; ++i) { out->module_state[i] = PX4LITE_STATE_ONLINE; }
  return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out) { (void)out; return PX4LITE_NOT_READY; }

/** @brief 从实际线帧重新解析，只有完整 CRC 有效的唯一消息才允许测试继续。 */
static void TestCapture(const uint8_t *data, uint16_t length)
{
  mavlink_message_t parser;
  mavlink_status_t parser_status, result_status;
  unsigned i, complete = 0U;
  memset(&parser, 0, sizeof(parser));
  memset(&parser_status, 0, sizeof(parser_status));
  memset(&result_status, 0, sizeof(result_status));
  for (i = 0U; i < length; ++i) {
    uint8_t result = mavlink_frame_char_buffer(&parser, &parser_status, data[i], &s_captured, &result_status);
    assert(result != MAVLINK_FRAMING_BAD_CRC && result != MAVLINK_FRAMING_BAD_SIGNATURE);
    if (result == MAVLINK_FRAMING_OK) { ++complete; }
  }
  assert(complete == 1U && s_captured.sysid == 42U);
}
Px4Lite_Result_t Px4Lite_RpiMavlinkSend(const uint8_t *data, uint16_t length)
{
  TestCapture(data, length); ++s_rpi_frames;
  assert(s_captured.compid == PX4LITE_RPI_MAVLINK_COMPONENT_ID);
  return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_LoRaSend(const uint8_t *data, uint16_t length)
{
  mavlink_battery_status_t packet;
  TestCapture(data, length);
  assert(s_captured.compid == PX4LITE_MAVLINK_COMPONENT_ID);
  assert(s_captured.msgid == MAVLINK_MSG_ID_BATTERY_STATUS);
  mavlink_msg_battery_status_decode(&s_captured, &packet);
  assert(s_lora_frames < sizeof(s_lora_ids));
  s_lora_ids[s_lora_frames++] = packet.id;
  return PX4LITE_OK;
}

/** @brief 每组被冻结后必须恰好提交启用且新鲜的输入，并结束会话。 */
static void TestPowerGroup(unsigned expected_frames, uint8_t expected_first_id)
{
  unsigned frames_before = s_lora_frames;
  unsigned complete_before = s_complete_calls;
  unsigned result_before = s_result_calls;
  unsigned i;
  s_pending = 1U;
  for (i = 0U; i < 3U; ++i) { assert(Px4Lite_MavlinkTxRun(s_now) == PX4LITE_IDLE); }
  assert(s_pending == 0U && s_lab_group_active == 0U);
  assert(s_lora_frames - frames_before == expected_frames);
  assert(s_complete_calls == complete_before + 1U);
  assert(s_result_calls - result_before == (expected_frames ? expected_frames : 1U));
  assert(s_last_result == (expected_frames ? PX4LITE_OK : PX4LITE_NOT_READY));
  if (expected_frames) { assert(s_lora_ids[frames_before] == expected_first_id); }
  if (expected_frames == 2U) { assert(s_lora_ids[frames_before + 1U] == 1U); }
}

int main(void)
{
  mavlink_battery_status_t battery;
  mavlink_sys_status_t system;
  unsigned before, copies1;
  assert(Px4Lite_MavlinkTxInit(s_now) == PX4LITE_OK);
  s_send_target = MAV_TX_TARGET_RPI;
  before = s_rpi_frames;
  assert(MavTx_SendBatteryStatus(s_now) == (PX4LITE_ENABLE_BATTERY1 ? PX4LITE_OK : PX4LITE_NOT_READY));
  assert(s_rpi_frames - before == (PX4LITE_ENABLE_BATTERY1 ? 1U : 0U));
  before = s_rpi_frames;
  assert(MavTx_SendBattery2(s_now) == (PX4LITE_ENABLE_BATTERY2 ? PX4LITE_OK : PX4LITE_NOT_READY));
  assert(s_rpi_frames - before == (PX4LITE_ENABLE_BATTERY2 ? 1U : 0U));
  if (PX4LITE_ENABLE_BATTERY2) {
    mavlink_msg_battery_status_decode(&s_captured, &battery);
    assert(battery.id == 1U && battery.voltages[0] == 12500U && battery.current_battery == 40);
    assert(battery.battery_remaining == -1 && battery.charge_state == MAV_BATTERY_CHARGE_STATE_UNDEFINED);
    assert(battery.current_consumed == -1 && battery.energy_consumed == -1);
    ++s_now; s_current_valid[1] = 0U;
    assert(MavTx_SendBattery2(s_now) == PX4LITE_OK);
    mavlink_msg_battery_status_decode(&s_captured, &battery);
    assert(battery.voltages[0] == 12500U && battery.current_battery == -1 && battery.battery_remaining == -1);
    assert(battery.charge_state == MAV_BATTERY_CHARGE_STATE_UNDEFINED);
    s_current_valid[1] = 1U;
  }
  copies1 = s_copies[0];
  assert(MavTx_SendSystemStatus(s_now) == PX4LITE_OK);
  assert(s_captured.msgid == MAVLINK_MSG_ID_SYS_STATUS);
  mavlink_msg_sys_status_decode(&s_captured, &system);
  if (PX4LITE_ENABLE_BATTERY1) {
    assert(system.voltage_battery == 12000U && system.current_battery == 10 && system.battery_remaining == 77);
    assert(s_copies[0] == copies1 + 1U);
  } else {
    assert(system.voltage_battery == UINT16_MAX && system.current_battery == -1 && system.battery_remaining == -1);
    assert(s_copies[0] == copies1);
  }

  /* 基础广播尚未到期，执行真实 LoRa 工作函数的 POWER 组调度分支。 */
  s_next_heartbeat_ms = s_now + 10000U;
  s_next_basic_id_ms = s_now + 10000U;
  s_next_sys_status_ms = s_now + 10000U;
  TestPowerGroup((unsigned)PX4LITE_ENABLE_BATTERY1 + (unsigned)PX4LITE_ENABLE_BATTERY2,
                 PX4LITE_ENABLE_BATTERY1 ? 0U : 1U);
  if (PX4LITE_ENABLE_BATTERY1 && PX4LITE_ENABLE_BATTERY2) {
    /* 双路配置中另一组暂缺时也须直接完成，不能先发送失效的 id0。 */
    s_available[0] = 0U;
    TestPowerGroup(1U, 1U);
    s_available[0] = 1U; s_available[1] = 0U;
    TestPowerGroup(1U, 0U);
  }
  s_available[0] = 0U; s_available[1] = 0U;
  TestPowerGroup(0U, 0U);
  if (!PX4LITE_ENABLE_BATTERY1) { assert(s_copies[0] == 0U); }
  if (!PX4LITE_ENABLE_BATTERY2) { assert(s_copies[1] == 0U); }
  printf("single supply MAVLink: enable1=%u enable2=%u passed\n",
         (unsigned)PX4LITE_ENABLE_BATTERY1, (unsigned)PX4LITE_ENABLE_BATTERY2);
  return 0;
}
