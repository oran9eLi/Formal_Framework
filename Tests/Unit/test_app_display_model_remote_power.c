/**
 * @file test_app_display_model_remote_power.c
 * @brief 验证 REMOTE 显示环境快照包含远端电机电池字段。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_display_model.h"

static Px4Lite_RemoteMode_t s_mode;
static Px4Lite_RemoteTelemetrySnapshot_t s_remote;

Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void)
{
  return s_mode;
}

Px4Lite_Result_t App_CopyRemoteTelemetry(Px4Lite_RemoteTelemetrySnapshot_t *out, uint32_t now_ms)
{
  (void)now_ms;
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  *out = s_remote;
  return PX4LITE_OK;
}

Px4Lite_Result_t App_CopyEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now_ms)
{
  (void)out;
  (void)now_ms;
  return PX4LITE_NOT_READY;
}

Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms) { (void)out; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopySystem(App_SystemSnapshot_t *out, uint32_t now_ms) { (void)out; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyAlarm(App_AlarmSnapshot_t *out, uint32_t now_ms) { (void)out; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyMotor(App_MotorSnapshot_t *out, uint32_t now_ms) { (void)out; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyDateTime(App_DateTimeSnapshot_t *out, uint32_t now_ms) { (void)out; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleStatus_t *out) { (void)module_id; (void)out; return PX4LITE_NOT_READY; }
void App_GetCommStats(Px4Lite_CommDebugInfo_t *out) { if (out != 0) { memset(out, 0, sizeof(*out)); } }

static int ExpectU32(const char *name, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)actual, (unsigned long)expected);
    return 1;
  }
  return 0;
}

static int ExpectU8(const char *name, uint8_t actual, uint8_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%u expected=%u\n", name, (unsigned int)actual, (unsigned int)expected);
    return 1;
  }
  return 0;
}

static int ExpectResult(const char *name, Px4Lite_Result_t actual, Px4Lite_Result_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%d expected=%d\n", name, (int)actual, (int)expected);
    return 1;
  }
  return 0;
}

static int TestRemoteEnvironmentCarriesMotorBattery(void)
{
  App_EnvironmentSnapshot_t env;
  int failures = 0;

  memset(&s_remote, 0, sizeof(s_remote));
  memset(&env, 0, sizeof(env));
  s_mode = PX4LITE_REMOTE_MODE_REMOTE;

  s_remote.valid_mask = PX4LITE_REMOTE_VALID_POWER;
  s_remote.voltage_mv = 11800U;
  s_remote.battery_percent = 86U;
  s_remote.voltage2_mv = 12050U;
  s_remote.battery2_percent = 91U;
  s_remote.low_voltage2 = 1U;

  failures += ExpectResult("remote env result", App_GetDisplayEnvironment(&env, 1000U), PX4LITE_OK);
  failures += ExpectU32("main voltage", env.voltage_mv, 11800U);
  failures += ExpectU8("main percent", env.battery_percent, 86U);
  failures += ExpectU32("motor voltage", env.voltage2_mv, 12050U);
  failures += ExpectU8("motor percent", env.battery2_percent, 91U);
  failures += ExpectU8("motor low voltage", env.low_voltage2, 1U);

  return failures;
}

int main(void)
{
  int failures = 0;

  failures += TestRemoteEnvironmentCarriesMotorBattery();

  if (failures == 0) {
    printf("PASS test_app_display_model_remote_power\n");
    return 0;
  }
  printf("FAIL test_app_display_model_remote_power failures=%d\n", failures);
  return 1;
}
