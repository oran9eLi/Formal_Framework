/**
 * @file test_display_remote_control.c
 * @brief PC 回归：真实显示请求边界隔离远端浏览与本机控制；应用/屏幕外设用桩记录副作用。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Display/Src/display.c"

static Px4Lite_RemoteMode_t view_mode;
static unsigned throttle_calls, takeoff_calls, landing_calls, attitude_calls, mode_calls, stop_calls;
static Px4Lite_AttitudeAction_t last_attitude_action;
static Px4Lite_ControlMode_t last_control_mode;
static App_AttitudeStatus_t attitude_status;
static uint16_t displayed_pulse[PX4LITE_MOTOR_COUNT];
static Px4Lite_Result_t calibration_result = PX4LITE_OK;
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void) { return view_mode; }
uint32_t Px4Lite_PlatformGetMs(void) { return 1000U; }
Px4Lite_Result_t App_SetMotorThrottlePercent(uint8_t i, uint8_t value)
{ assert(i < 4U); assert(value <= 100U); throttle_calls++; return PX4LITE_OK; }
Px4Lite_Result_t App_StartMotorAutoTakeoff(void) { takeoff_calls++; return PX4LITE_OK; }
Px4Lite_Result_t App_StartMotorAutoLanding(void) { landing_calls++; return PX4LITE_OK; }
Px4Lite_Result_t App_RequestAttitudeAction(Px4Lite_AttitudeAction_t action)
{ attitude_calls++; last_attitude_action = action; return calibration_result; }
Px4Lite_Result_t App_SetMotorControlMode(Px4Lite_ControlMode_t mode)
{ mode_calls++; last_control_mode = mode; return PX4LITE_OK; }
Px4Lite_Result_t App_CopyAttitudeStatus(App_AttitudeStatus_t *out)
{ *out = attitude_status; return PX4LITE_OK; }
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
{ assert(i < PX4LITE_MOTOR_COUNT); displayed_pulse[i] = value; return DISPLAY_OK; }

/** @brief 远端禁止控制且不伪造成功；本机急停不能篡改远端遥测。 */
int main(void)
{
  unsigned i;
  App_MotorSnapshot_t motor;
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
  assert(Display_RequestAttitudeAction(PX4LITE_ATTITUDE_ACTION_BIAS) == DISPLAY_NOT_READY);
  assert(Display_RequestMotorControlMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST) == DISPLAY_NOT_READY);
  assert(throttle_calls == 0U && takeoff_calls == 0U && landing_calls == 0U && attitude_calls == 0U && mode_calls == 0U);
  assert(Display_RequestMotorEmergencyStop() == DISPLAY_OK);
  assert(stop_calls == 1U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_PWM_1].value == 42U);
  assert(Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE, PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST) == DISPLAY_OK);
  assert(Display_SetHmiValueU16(DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE, PX4LITE_MOTOR_OP_SYNC_RAMP_UP) == DISPLAY_OK);
  Display_ClearMotorFields();
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE].value == 2U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE].value == 255U);

  view_mode = PX4LITE_REMOTE_MODE_LOCAL;
  assert(Display_RequestMotorThrottle(DISPLAY_HMI_VAR_MOTOR_PWM_1, 60U) == DISPLAY_OK);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_TARGET_1].value == 60U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_PWM_1].value == 0U);
  assert(Display_RequestMotorAutoTakeoff() == DISPLAY_OK);
  assert(Display_RequestMotorAutoLanding() == DISPLAY_OK);
  assert(Display_RequestAttitudeAction(PX4LITE_ATTITUDE_ACTION_BIAS) == DISPLAY_OK);
  assert(last_attitude_action == PX4LITE_ATTITUDE_ACTION_BIAS);
  assert(Display_RequestMotorControlMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST) == DISPLAY_OK);
  assert(last_control_mode == PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
  assert(throttle_calls == 1U && takeoff_calls == 1U && landing_calls == 1U && attitude_calls == 1U && mode_calls == 1U);
  calibration_result = PX4LITE_NOT_READY;
  assert(Display_RequestAttitudeAction(PX4LITE_ATTITUDE_ACTION_ZERO) == DISPLAY_NOT_READY);
  assert(Display_RequestMotorEmergencyStop() == DISPLAY_OK);
  assert(stop_calls == 2U && s_hmi_values[DISPLAY_HMI_VAR_MOTOR_PWM_1].value == 0U);

  memset(&motor, 0, sizeof(motor));
  motor.control_mode_valid = 1U;
  motor.operation_state_valid = 1U;
  motor.control_mode = PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST;
  motor.operation_state = PX4LITE_MOTOR_OP_SYNC_RAMP_UP;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    motor.base_percent[i] = (uint8_t)(20U + i);
    motor.duty_percent[i] = (uint8_t)(30U + i);
    motor.pulse_us[i] = (uint16_t)(1300U + i);
  }
  Display_LoadMotorSnapshot(&motor);
  Display_LoadMotorPulseSnapshot(&motor);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_TARGET_1].value == 20U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_PWM_1].value == 30U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_CONTROL_MODE].value == PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_OPERATION_STATE].value == PX4LITE_MOTOR_OP_SYNC_RAMP_UP);
  assert(displayed_pulse[0] == 1300U);

  memset(&attitude_status, 0, sizeof(attitude_status));
  attitude_status.action = PX4LITE_ATTITUDE_ACTION_BIAS;
  attitude_status.phase = PX4LITE_ATTITUDE_COLLECTING;
  attitude_status.progress = 55U;
  attitude_status.reference_active = 1U;
  Display_LoadAttitudeOperationStatus(PX4LITE_REMOTE_MODE_LOCAL);
  assert(s_hmi_values[DISPLAY_HMI_VAR_ATTITUDE_ACTION].value == PX4LITE_ATTITUDE_ACTION_BIAS);
  assert(s_hmi_values[DISPLAY_HMI_VAR_ATTITUDE_PHASE].value == PX4LITE_ATTITUDE_COLLECTING);
  assert(s_hmi_values[DISPLAY_HMI_VAR_ATTITUDE_PROGRESS].value == 55U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_ATTITUDE_REFERENCE].value == 1U);
  Display_LoadAttitudeOperationStatus(PX4LITE_REMOTE_MODE_REMOTE);
  assert(s_hmi_values[DISPLAY_HMI_VAR_ATTITUDE_PHASE].value == 255U);
  puts("display remote control tests passed");
  return 0;
}
