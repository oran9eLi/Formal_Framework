/** @file motor_battery_fixture.h
 * @brief Control host 测试的本机动力电池输入；默认连续采样，可单独冻结样本年龄。
 */
#ifndef MOTOR_BATTERY_FIXTURE_H
#define MOTOR_BATTERY_FIXTURE_H
static uint32_t g_battery_mv = 12000U;
static uint32_t g_battery_sample_ms;
static uint8_t g_battery_ready = 1U;
static uint8_t g_battery_valid = 1U;
static uint8_t g_battery_live = 1U;
static void ResetBatteryFixture(void)
{
  g_battery_mv = 12000U;
  g_battery_ready = 1U;
  g_battery_valid = 1U;
  g_battery_live = 1U;
}
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out)
{
  if (!g_battery_ready) { return PX4LITE_NOT_READY; }
  memset(out, 0, sizeof(*out));
  out->header.valid = g_battery_valid;
  out->header.sample_time_ms = g_battery_live ? g_now_ms : g_battery_sample_ms;
  out->voltage_mv = g_battery_mv;
  return PX4LITE_OK;
}
static Px4Lite_Result_t SetFixturePowerInhibit(uint8_t inhibit)
{
  g_battery_mv = inhibit ? 9800U : 12000U;
  Px4Lite_ControlRun(g_now_ms);
  return PX4LITE_OK;
}
#endif
