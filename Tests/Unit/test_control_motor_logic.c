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
static uint32_t g_publish_count;
static Px4Lite_State_t g_control_state;
static uint16_t g_control_fault;

static void ResetHarness(void)
{
  memset(g_pulse_us, 0, sizeof(g_pulse_us));
  memset(g_button_pressed, 0, sizeof(g_button_pressed));
  memset(&g_last_motor, 0, sizeof(g_last_motor));
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
  failures += TestKey1EmergencyStopClearsTargets();
  failures += TestFailsafeGapRearmsAtMinimum();

  if (failures != 0) {
    printf("control motor tests failed: %d\n", failures);
    return 1;
  }

  printf("control motor tests passed\n");
  return 0;
}
