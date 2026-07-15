/**
 * @file test_control_motor_logic.c
 * @brief 验证 Control 模块的电机油门映射、急停和失效保护逻辑。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "px4lite_config.h"
#include "px4lite_control.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"

static uint32_t g_now_ms;
static uint16_t g_pulse_us[PX4LITE_MOTOR_COUNT];
static uint8_t g_button_pressed[PX4LITE_BUTTON_COUNT];
static Px4Lite_MotorOutputs_t g_last_motor;
static Px4Lite_VehicleNavigation_t g_navigation;
static uint8_t g_navigation_ready;
static uint32_t g_publish_count;
static Px4Lite_State_t g_control_state;
static uint16_t g_control_fault;

static void ResetHarness(void)
{
  memset(g_pulse_us, 0, sizeof(g_pulse_us));
  memset(g_button_pressed, 0, sizeof(g_button_pressed));
  memset(&g_last_motor, 0, sizeof(g_last_motor));
  memset(&g_navigation, 0, sizeof(g_navigation));
  g_navigation_ready = 0U;
  g_publish_count = 0U;
  g_control_state = PX4LITE_STATE_UNINITIALIZED;
  g_control_fault = 0U;
  g_now_ms        = 1000U;
}

static int ExpectU32(const char *name, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)actual, (unsigned long)expected);
    return 1;
  }
  return 0;
}

uint32_t Px4Lite_PlatformGetMs(void)
{
  return g_now_ms;
}

Px4Lite_Result_t Px4Lite_MotorInit(void)
{
  return Px4Lite_MotorDisarmAll();
}

Px4Lite_Result_t Px4Lite_MotorWritePulseUs(uint8_t channel, uint16_t pulse_us)
{
  if (channel >= PX4LITE_MOTOR_COUNT) { return PX4LITE_INVALID_PARAM; }
  g_pulse_us[channel] = pulse_us;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_MotorDisarmAll(void)
{
  uint8_t i;

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    g_pulse_us[i] = PX4LITE_CONTROL_ESC_MIN_PULSE_US;
  }
  return PX4LITE_OK;
}

uint8_t Px4Lite_ButtonPressed(Px4Lite_ButtonId_t button)
{
  if (button >= PX4LITE_BUTTON_COUNT) { return 0U; }
  return g_button_pressed[button];
}

void Px4Lite_PublishMotor(const Px4Lite_MotorOutputs_t *data)
{
  g_last_motor = *data;
  g_publish_count++;
}

Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *data)
{
  if (data == 0) { return PX4LITE_INVALID_PARAM; }
  if (g_navigation_ready == 0U) { return PX4LITE_NOT_READY; }
  *data = g_navigation;
  return PX4LITE_OK;
}

void Px4Lite_SetExternalModuleState(Px4Lite_ModuleId_t module_id, Px4Lite_State_t state, uint16_t fault_code, uint32_t now_ms)
{
  (void)now_ms;
  if (module_id == PX4LITE_MODULE_CONTROL) {
    g_control_state = state;
    g_control_fault = fault_code;
  }
}

#include "../../Framework/Src/px4lite_control.c"

static void ArmControl(void)
{
  uint32_t elapsed;

  (void)Px4Lite_ControlModuleInit();
  for (elapsed = 0U; elapsed <= PX4LITE_CONTROL_ESC_ARM_TIME_MS; elapsed += PX4LITE_CONTROL_PERIOD_MS) {
    Px4Lite_ControlRun(g_now_ms);
    g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
  }
}

static void SeedAttitude(uint32_t sample_time_ms, int32_t roll_deg100, int32_t pitch_deg100, int32_t roll_rate_dps100, int32_t pitch_rate_dps100)
{
  memset(&g_navigation, 0, sizeof(g_navigation));
  g_navigation.header.valid          = 1U;
  g_navigation.header.sample_time_ms = sample_time_ms;
  g_navigation.valid_mask            = PX4LITE_NAV_VALID_ATTITUDE;
  g_navigation.roll_deg100           = roll_deg100;
  g_navigation.pitch_deg100          = pitch_deg100;
  g_navigation.roll_rate_dps100      = roll_rate_dps100;
  g_navigation.pitch_rate_dps100     = pitch_rate_dps100;
  g_navigation_ready                 = 1U;
}

static int TestThrottleMappingAndPublish(void)
{
  uint16_t expected_mid = (uint16_t)(PX4LITE_CONTROL_ESC_MIN_PULSE_US + ((PX4LITE_CONTROL_ESC_MAX_PULSE_US - PX4LITE_CONTROL_ESC_MIN_PULSE_US) / 2U));
  int failures          = 0;

  ResetHarness();
  ArmControl();
  failures += ExpectU32("set motor0 50", Px4Lite_ControlSetMotorThrottlePercent(0U, 50U), PX4LITE_OK);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("motor0 pulse", g_pulse_us[0], expected_mid);
  failures += ExpectU32("motor1 pulse", g_pulse_us[1], PX4LITE_CONTROL_ESC_MIN_PULSE_US);
  failures += ExpectU32("published duty0", g_last_motor.duty_percent[0], 50U);
  failures += ExpectU32("published run", g_last_motor.run_state, 1U);
  failures += ExpectU32("control state", g_control_state, PX4LITE_STATE_ONLINE);
  failures += ExpectU32("control fault", g_control_fault, 0U);
  failures += ExpectU32("publish count nonzero", (g_publish_count > 0U) ? 1U : 0U, 1U);
  return failures;
}

static int TestThrottleClampAndInvalidIndex(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  failures += ExpectU32("invalid index", Px4Lite_ControlSetMotorThrottlePercent(PX4LITE_MOTOR_COUNT, 10U), PX4LITE_INVALID_PARAM);
  failures += ExpectU32("clamp motor1", Px4Lite_ControlSetMotorThrottlePercent(1U, 120U), PX4LITE_OK);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("motor1 pulse max", g_pulse_us[1], PX4LITE_CONTROL_ESC_MAX_PULSE_US);
  failures += ExpectU32("published duty1 max", g_last_motor.duty_percent[1], 100U);
  return failures;
}

static int TestAttitudeAssistMixesFreshRoll(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(1U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(2U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(3U, 40U);
  SeedAttitude(g_now_ms, 1000, 0, 0, 0);
  failures += ExpectU32("assist target", Px4Lite_ControlSetAttitudeTarget(0, 0, 0), PX4LITE_OK);
  failures += ExpectU32("assist mode", Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST), PX4LITE_OK);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("assist duty0", g_last_motor.duty_percent[0], 37U);
  failures += ExpectU32("assist duty1", g_last_motor.duty_percent[1], 43U);
  failures += ExpectU32("assist duty2", g_last_motor.duty_percent[2], 37U);
  failures += ExpectU32("assist duty3", g_last_motor.duty_percent[3], 43U);
  failures += ExpectU32("assist pulse0", g_pulse_us[0], 1370U);
  failures += ExpectU32("assist pulse1", g_pulse_us[1], 1430U);
  return failures;
}

static int TestAttitudeAssistDefaultsOnForSingleActiveOutput(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 40U);
  SeedAttitude(g_now_ms, 1000, 0, 0, 0);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("default assist duty0", g_last_motor.duty_percent[0], 37U);
  failures += ExpectU32("default assist pulse0", g_pulse_us[0], 1370U);
  failures += ExpectU32("inactive duty1", g_last_motor.duty_percent[1], 0U);
  failures += ExpectU32("inactive pulse1", g_pulse_us[1], PX4LITE_CONTROL_ESC_MIN_PULSE_US);
  return failures;
}

static int TestAttitudeAssistPreservesUnequalBaseThrottle(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 20U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(1U, 30U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(2U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(3U, 50U);
  SeedAttitude(g_now_ms, 1000, 0, 0, 0);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("unequal base duty0", g_last_motor.duty_percent[0], 17U);
  failures += ExpectU32("unequal base duty1", g_last_motor.duty_percent[1], 33U);
  failures += ExpectU32("unequal base duty2", g_last_motor.duty_percent[2], 37U);
  failures += ExpectU32("unequal base duty3", g_last_motor.duty_percent[3], 53U);
  return failures;
}

static int TestAttitudeAssistCorrectsPositivePitch(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(1U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(2U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(3U, 40U);
  SeedAttitude(g_now_ms, 0, 1000, 0, 0);
  (void)Px4Lite_ControlSetAttitudeTarget(0, 0, 0);
  (void)Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("pitch front-left", g_last_motor.duty_percent[0], 43U);
  failures += ExpectU32("pitch front-right", g_last_motor.duty_percent[1], 43U);
  failures += ExpectU32("pitch rear-left", g_last_motor.duty_percent[2], 37U);
  failures += ExpectU32("pitch rear-right", g_last_motor.duty_percent[3], 37U);
  return failures;
}

static int TestAttitudeAssistCorrectsPositiveYaw(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(1U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(2U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(3U, 40U);
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  g_navigation.yaw_deg100 = 1000;
  g_navigation.yaw_rate_dps100 = 0;
  (void)Px4Lite_ControlSetAttitudeTarget(0, 0, 0);
  (void)Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("yaw cw motor1", g_last_motor.duty_percent[0], 43U);
  failures += ExpectU32("yaw ccw motor2", g_last_motor.duty_percent[1], 37U);
  failures += ExpectU32("yaw ccw motor3", g_last_motor.duty_percent[2], 37U);
  failures += ExpectU32("yaw cw motor4", g_last_motor.duty_percent[3], 43U);
  return failures;
}

static int TestAttitudeTargetRejectsUnsafeRange(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  failures += ExpectU32("target positive limit", Px4Lite_ControlSetAttitudeTarget(PX4LITE_CONTROL_ATTITUDE_TARGET_LIMIT_DEG100, 0, 0), PX4LITE_OK);
  failures += ExpectU32("target roll overflow", Px4Lite_ControlSetAttitudeTarget(PX4LITE_CONTROL_ATTITUDE_TARGET_LIMIT_DEG100 + 1, 0, 0), PX4LITE_INVALID_PARAM);
  failures += ExpectU32("target pitch overflow", Px4Lite_ControlSetAttitudeTarget(0, -PX4LITE_CONTROL_ATTITUDE_TARGET_LIMIT_DEG100 - 1, 0), PX4LITE_INVALID_PARAM);
  failures += ExpectU32("target yaw overflow", Px4Lite_ControlSetAttitudeTarget(0, 0, PX4LITE_CONTROL_ATTITUDE_YAW_TARGET_LIMIT_DEG100 + 1), PX4LITE_INVALID_PARAM);
  return failures;
}

static int TestAttitudeAssistClampsCorruptNavigation(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(1U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(2U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(3U, 40U);
  SeedAttitude(g_now_ms, (int32_t)0x80000000UL, (int32_t)0x80000000UL, (int32_t)0x80000000UL, (int32_t)0x80000000UL);
  (void)Px4Lite_ControlSetAttitudeTarget(0, 0, 0);
  (void)Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("corrupt nav duty0", g_last_motor.duty_percent[0], 40U);
  failures += ExpectU32("corrupt nav duty1", g_last_motor.duty_percent[1], 0U);
  failures += ExpectU32("corrupt nav duty2", g_last_motor.duty_percent[2], 80U);
  failures += ExpectU32("corrupt nav duty3", g_last_motor.duty_percent[3], 40U);
  return failures;
}

static int TestAttitudeAssistStaleFallsBackToDirect(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(1U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(2U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(3U, 40U);
  SeedAttitude(g_now_ms - PX4LITE_IMU_MAX_AGE_MS - 1U, 1000, 0, 0, 0);
  (void)Px4Lite_ControlSetAttitudeTarget(0, 0, 0);
  (void)Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("stale duty0", g_last_motor.duty_percent[0], 40U);
  failures += ExpectU32("stale duty1", g_last_motor.duty_percent[1], 40U);
  failures += ExpectU32("stale pulse0", g_pulse_us[0], 1400U);
  failures += ExpectU32("stale pulse1", g_pulse_us[1], 1400U);
  return failures;
}

static int TestKey1EmergencyStopClearsTargets(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(2U, 60U);
  Px4Lite_ControlRun(g_now_ms);
  g_button_pressed[PX4LITE_BUTTON_KEY1] = 1U;
  g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
  Px4Lite_ControlRun(g_now_ms);
  g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("motor2 estop pulse", g_pulse_us[2], PX4LITE_CONTROL_ESC_MIN_PULSE_US);
  failures += ExpectU32("motor2 estop duty", g_last_motor.duty_percent[2], 0U);
  return failures;
}

static int TestFailsafeGapRearmsAtMinimum(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(3U, 80U);
  Px4Lite_ControlRun(g_now_ms);
  g_now_ms += PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS + 1U;
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("failsafe pulse", g_pulse_us[3], PX4LITE_CONTROL_ESC_MIN_PULSE_US);
  failures += ExpectU32("failsafe duty", g_last_motor.duty_percent[3], 0U);
  failures += ExpectU32("failsafe state", g_control_state, PX4LITE_STATE_STARTING);
  return failures;
}

int main(void)
{
  int failures = 0;

  failures += TestThrottleMappingAndPublish();
  failures += TestThrottleClampAndInvalidIndex();
  failures += TestAttitudeAssistMixesFreshRoll();
  failures += TestAttitudeAssistDefaultsOnForSingleActiveOutput();
  failures += TestAttitudeAssistPreservesUnequalBaseThrottle();
  failures += TestAttitudeAssistCorrectsPositivePitch();
  failures += TestAttitudeAssistCorrectsPositiveYaw();
  failures += TestAttitudeTargetRejectsUnsafeRange();
  failures += TestAttitudeAssistClampsCorruptNavigation();
  failures += TestAttitudeAssistStaleFallsBackToDirect();
  failures += TestKey1EmergencyStopClearsTargets();
  failures += TestFailsafeGapRearmsAtMinimum();

  if (failures != 0) {
    printf("control motor tests failed: %d\n", failures);
    return 1;
  }

  printf("control motor tests passed\n");
  return 0;
}
