/**
 * @file test_display_remote_control.c
 * @brief PC 回归：真实显示请求边界隔离远端浏览与本机控制；应用/屏幕外设用桩记录副作用。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Display/Src/display.c"

static Px4Lite_RemoteMode_t view_mode;
static unsigned throttle_calls, takeoff_calls, landing_calls, calibration_calls, stop_calls;
static Px4Lite_Result_t calibration_result = PX4LITE_OK;
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void) { return view_mode; }
uint32_t Px4Lite_PlatformGetMs(void) { return 1000U; }
Px4Lite_Result_t App_SetMotorThrottlePercent(uint8_t i, uint8_t value)
{ assert(i < 4U); assert(value <= 100U); throttle_calls++; return PX4LITE_OK; }
Px4Lite_Result_t App_StartMotorAutoTakeoff(void) { takeoff_calls++; return PX4LITE_OK; }
Px4Lite_Result_t App_StartMotorAutoLanding(void) { landing_calls++; return PX4LITE_OK; }
Px4Lite_Result_t App_RequestAttitudeLevelCalibration(void) { calibration_calls++; return calibration_result; }
Px4Lite_Result_t App_EmergencyStopMotors(void) { stop_calls++; return PX4LITE_OK; }
uint8_t App_IsMotorPowerInhibited(void) { return 0U; }
void App_MessageLogPushThrottlePowerFail(uint32_t now) { (void)now; }
void App_MessageLogPushLandingThrottleReject(uint32_t now) { (void)now; }
void App_MessageLogPushTakeoffPowerFail(uint32_t now) { (void)now; }
void App_MessageLogPushTakeoffSensorFail(uint32_t now) { (void)now; }
void App_MessageLogPushLandingFail(uint32_t now) { (void)now; }
Display_Result_t Display_LvglSetValue(Display_HmiVariableId_t id, uint32_t value)
{ (void)id; (void)value; return DISPLAY_OK; }
Display_Result_t Display_LvglSetMotorPulseUs(uint8_t i, uint16_t value)
{ (void)i; (void)value; return DISPLAY_OK; }

/** @brief 远端禁止控制且不伪造成功；本机急停不能篡改远端遥测。 */
int main(void)
{
  unsigned i;
  s_display_initialized = 1U;
  s_display_ready = 1U;
  view_mode = PX4LITE_REMOTE_MODE_REMOTE;
  for (i = 0U; i < 4U; i++) {
    Display_HmiVariableId_t id = (Display_HmiVariableId_t)(DISPLAY_HMI_VAR_MOTOR_PWM_1 + i);
    assert(Display_SetHmiValueU16(id, 42U) == DISPLAY_OK);
    assert(Display_RequestMotorThrottle(id, 60U) == DISPLAY_NOT_READY);
    assert(Display_RequestMotorThrottle(id, 0U) == DISPLAY_NOT_READY);
    assert(s_hmi_values[id].value == 42U);
  }
  assert(Display_RequestMotorAutoTakeoff() == DISPLAY_NOT_READY);
  assert(Display_RequestMotorAutoLanding() == DISPLAY_NOT_READY);
  assert(Display_RequestAttitudeLevelCalibration() == DISPLAY_NOT_READY);
  assert(throttle_calls == 0U && takeoff_calls == 0U && landing_calls == 0U && calibration_calls == 0U);
  assert(Display_RequestMotorEmergencyStop() == DISPLAY_OK);
  assert(stop_calls == 1U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_PWM_1].value == 42U);

  view_mode = PX4LITE_REMOTE_MODE_LOCAL;
  assert(Display_RequestMotorThrottle(DISPLAY_HMI_VAR_MOTOR_PWM_1, 60U) == DISPLAY_OK);
  assert(Display_RequestMotorAutoTakeoff() == DISPLAY_OK);
  assert(Display_RequestMotorAutoLanding() == DISPLAY_OK);
  assert(Display_RequestAttitudeLevelCalibration() == DISPLAY_OK);
  assert(throttle_calls == 1U && takeoff_calls == 1U && landing_calls == 1U && calibration_calls == 1U);
  calibration_result = PX4LITE_NOT_READY;
  assert(Display_RequestAttitudeLevelCalibration() == DISPLAY_NOT_READY);
  assert(Display_RequestMotorEmergencyStop() == DISPLAY_OK);
  assert(stop_calls == 2U && s_hmi_values[DISPLAY_HMI_VAR_MOTOR_PWM_1].value == 0U);
  puts("display remote control tests passed");
  return 0;
}
