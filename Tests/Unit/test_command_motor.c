/**
 * @file test_command_motor.c
 * @brief 使用真实 Command 与 Control 实现验证四路电机命令的事务语义。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Host 测试不启动调度器，但仍记录临界区是否成对进入。 */
#define INC_FREERTOS_H
#define INC_TASK_H
static uint32_t g_critical_depth;
static uint32_t g_critical_enter_count;
#define taskENTER_CRITICAL() do { g_critical_depth++; g_critical_enter_count++; } while (0)
#define taskEXIT_CRITICAL()  do { g_critical_depth--; } while (0)
static unsigned g_scheduler_depth;
#define vTaskSuspendAll() do { g_scheduler_depth++; } while (0)
#define xTaskResumeAll() (--g_scheduler_depth)

#include "px4lite_config.h"
#include "px4lite_control.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"
#include "common/mavlink.h"

static uint32_t g_now_ms;
static uint16_t g_pulse_us[PX4LITE_MOTOR_COUNT];
static Px4Lite_MotorOutputs_t g_last_motor;
static uint8_t g_last_ack_result;
static uint8_t g_navigation_ready;
static uint8_t g_require_cycle_lock;
static uint8_t g_unprotected_write;

static int ExpectU32(const char *name, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)actual, (unsigned long)expected);
    return 1;
  }
  return 0;
}

uint32_t Px4Lite_PlatformGetMs(void) { return g_now_ms; }
Px4Lite_Result_t Px4Lite_MotorInit(void) { return Px4Lite_MotorDisarmAll(); }
Px4Lite_Result_t Px4Lite_MotorWritePulseUs(uint8_t channel, uint16_t pulse_us)
{
  if (channel >= PX4LITE_MOTOR_COUNT) { return PX4LITE_INVALID_PARAM; }
  if ((g_require_cycle_lock != 0U) && (g_scheduler_depth == 0U)) { g_unprotected_write = 1U; }
  g_pulse_us[channel] = pulse_us;
  return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_MotorDisarmAll(void)
{
  uint8_t i;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) { g_pulse_us[i] = PX4LITE_CONTROL_ESC_MIN_PULSE_US; }
  return PX4LITE_OK;
}
uint8_t Px4Lite_ButtonPressed(Px4Lite_ButtonId_t button) { (void)button; return 0U; }
void Px4Lite_PublishMotor(const Px4Lite_MotorOutputs_t *data) { g_last_motor = *data; }
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *data)
{
  if (g_navigation_ready == 0U) { return PX4LITE_NOT_READY; }
  memset(data, 0, sizeof(*data));
  data->header.valid = 1U;
  data->header.sample_time_ms = g_now_ms;
  data->valid_mask = PX4LITE_NAV_VALID_ATTITUDE;
  data->roll_deg100 = 1000;
  return PX4LITE_OK;
}
void Px4Lite_SetExternalModuleState(Px4Lite_ModuleId_t module_id, Px4Lite_State_t state, uint16_t fault_code, uint32_t now_ms)
{ (void)module_id; (void)state; (void)fault_code; (void)now_ms; }
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleStatus_t *out)
{ (void)module_id; (void)out; return PX4LITE_NOT_READY; }
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void) { return 1U; }
void Px4Lite_LocalMsgLogPush(uint16_t message_id, uint32_t time_hhmmss, uint16_t fault_code, uint8_t severity, uint8_t source_id, uint8_t active)
{ (void)message_id; (void)time_hhmmss; (void)fault_code; (void)severity; (void)source_id; (void)active; }
void Px4Lite_MavlinkQueueCommandAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, Px4Lite_MavlinkLink_t link)
{ (void)command; (void)target_system; (void)target_component; (void)link; g_last_ack_result = result; }
void Px4Lite_MavlinkRecordCommandAck(uint16_t command, uint8_t result, uint32_t now_ms)
{ (void)command; (void)result; (void)now_ms; }

#include "../../Framework/Src/px4lite_control.c"
#include "../../Framework/Src/px4lite_command.c"

static void ResetHarness(void)
{
  memset(g_pulse_us, 0, sizeof(g_pulse_us));
  memset(&g_last_motor, 0, sizeof(g_last_motor));
  g_now_ms = 4000U;
  g_last_ack_result = 0xFFU;
  g_critical_depth = 0U;
  g_critical_enter_count = 0U;
  g_navigation_ready = 0U;
  g_require_cycle_lock = 0U;
  g_unprotected_write = 0U;
  (void)Px4Lite_ControlSetPowerInhibit(0U);
  (void)Px4Lite_ControlModuleInit();
}

static void SendMotorCommand(uint16_t command, float p1, float p2, float p3, float p4)
{
  (void)Px4Lite_CommandHandleMavlinkLong(command, 42U, 7U, 1U, 191U,
                                         p1, p2, p3, p4,
                                         PX4LITE_MAVLINK_LINK_LORA, g_now_ms);
}

/** @brief 被拒的脉宽命令不得提前切换模式或破坏降落曲线和目标。 */
static int TestRejectedPulseCommandIsAtomic(void)
{
  uint8_t i;
  uint32_t old_update[PX4LITE_MOTOR_COUNT];
  uint16_t old_pulse[PX4LITE_MOTOR_COUNT];
  int failures = 0;

  ResetHarness();
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) { (void)Px4Lite_ControlSetMotorThrottlePercent(i, 60U); }
  (void)Px4Lite_ControlSetAttitudeTarget(0, 0, 0);
  (void)Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
  g_navigation_ready = 1U;
  g_require_cycle_lock = 1U;
  for (i = 0U; i < 70U; i++) { g_now_ms += 50U; Px4Lite_ControlRun(g_now_ms); }
  (void)Px4Lite_ControlStartAutoLanding();
  Px4Lite_ControlRun(g_now_ms);
  memcpy(old_pulse, g_pulse_us, sizeof(old_pulse));
  failures += ExpectU32("assist output before rejection", old_pulse[0], 1570U);
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) { old_update[i] = s_target_update_ms[i]; }

  SendMotorCommand(31013U, 1600.0f, 1600.0f, 1600.0f, 1600.0f);

  failures += ExpectU32("pulse refused", g_last_ack_result, MAV_RESULT_TEMPORARILY_REJECTED);
  failures += ExpectU32("mode preserved", s_control_mode, PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
  failures += ExpectU32("landing preserved", s_auto_profile, PX4LITE_CONTROL_AUTO_LANDING);
  failures += ExpectU32("landing ownership preserved", s_auto_landing_user, 1U);
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    failures += ExpectU32("target preserved", s_target_throttle_percent[i], 60U);
    failures += ExpectU32("timestamp preserved", s_target_update_ms[i], old_update[i]);
    failures += ExpectU32("target type preserved", s_target_is_pulse[i], 0U);
  }
  Px4Lite_ControlRun(g_now_ms); /* 固定时刻隔离命令效果，不混入斜坡自然变化。 */
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    failures += ExpectU32("rejected command leaves physical output", g_pulse_us[i], old_pulse[i]);
  }
  failures += ExpectU32("cycle output serialization", g_unprotected_write, 0U);
  SendMotorCommand(31013U, 1000.0f, 1000.0f, 1000.0f, 1000.0f);
  failures += ExpectU32("zero during landing accepted", g_last_ack_result, MAV_RESULT_ACCEPTED);
  failures += ExpectU32("zero retains assist and landing", s_control_mode, PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
  failures += ExpectU32("zero retains landing", s_auto_profile, PX4LITE_CONTROL_AUTO_LANDING);
  return failures;
}

/** @brief 第三路被闭锁拒绝时前两路也不能变更；NaN/Inf/边界值拒绝不能改变目标。 */
static int TestRejectedBatchLeavesAllTargets(void)
{
  uint8_t i;
  static const float bad[] = {-1.0f, 101.0f, NAN, INFINITY};
  int failures = 0;
  ResetHarness();
  (void)Px4Lite_ControlSetPowerInhibit(1U);
  g_now_ms += 10U;
  SendMotorCommand(31011U, 0.0f, 0.0f, 20.0f, 0.0f);
  failures += ExpectU32("inhibited batch refused", g_last_ack_result, MAV_RESULT_TEMPORARILY_REJECTED);
  for (i = 0U; i < 4U; ++i) {
    failures += ExpectU32("no partial valid target", s_target_valid[i], 0U);
    failures += ExpectU32("no partial timestamp", s_target_update_ms[i], 0U);
  }
  (void)Px4Lite_ControlSetPowerInhibit(0U);
  SendMotorCommand(31011U, 10.0f, 20.0f, 30.0f, 40.0f);
  for (i = 0U; i < sizeof(bad)/sizeof(bad[0]); ++i) {
    SendMotorCommand(31011U, 50.0f, bad[i], 60.0f, 70.0f);
    failures += ExpectU32("bad percent rejected", g_last_ack_result, MAV_RESULT_DENIED);
    failures += ExpectU32("bad percent preserves first", s_target_throttle_percent[0], 10U);
  }
  SendMotorCommand(31013U, 1200.0f, 999.0f, 1300.0f, 1400.0f);
  failures += ExpectU32("bad pulse rejected", g_last_ack_result, MAV_RESULT_DENIED);
  failures += ExpectU32("bad pulse preserves first", s_target_throttle_percent[0], 10U);
  return failures;
}

/** @brief 成功的脉宽命令在一个临界区内提交四路目标和 DIRECT 模式。 */
static int TestAcceptedPulseCommandCommitsOnce(void)
{
  static const uint16_t expected[PX4LITE_MOTOR_COUNT] = {1200U, 1300U, 1400U, 1500U};
  uint8_t i;
  int failures = 0;

  ResetHarness();
  SendMotorCommand(31013U, 1200.0f, 1300.0f, 1400.0f, 1500.0f);
  failures += ExpectU32("pulse accepted", g_last_ack_result, MAV_RESULT_ACCEPTED);
  failures += ExpectU32("pulse direct mode", s_control_mode, PX4LITE_CONTROL_MODE_DIRECT);
  failures += ExpectU32("single commit", g_critical_enter_count, 1U);
  failures += ExpectU32("critical balanced", g_critical_depth, 0U);
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    failures += ExpectU32("pulse target", s_target_pulse_us[i], expected[i]);
    failures += ExpectU32("pulse type", s_target_is_pulse[i], 1U);
    failures += ExpectU32("same timestamp", s_target_update_ms[i], g_now_ms);
  }
  return failures;
}

/** @brief 百分比命令先校验全部参数，再在一个临界区内提交四路。 */
static int TestThrottleCommandValidationAndCommit(void)
{
  uint8_t i;
  int failures = 0;

  ResetHarness();
  SendMotorCommand(31011U, 10.0f, 20.0f, 101.0f, 40.0f);
  failures += ExpectU32("invalid throttle denied", g_last_ack_result, MAV_RESULT_DENIED);
  failures += ExpectU32("invalid no commit", g_critical_enter_count, 0U);
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) { failures += ExpectU32("invalid target untouched", s_target_valid[i], 0U); }

  SendMotorCommand(31011U, 10.0f, 20.0f, 30.0f, 40.0f);
  failures += ExpectU32("throttle accepted", g_last_ack_result, MAV_RESULT_ACCEPTED);
  failures += ExpectU32("throttle single commit", g_critical_enter_count, 1U);
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    failures += ExpectU32("throttle target", s_target_throttle_percent[i], (uint32_t)((i + 1U) * 10U));
    failures += ExpectU32("throttle type", s_target_is_pulse[i], 0U);
    failures += ExpectU32("throttle timestamp", s_target_update_ms[i], g_now_ms);
  }
  return failures;
}

int main(void)
{
  int failures = 0;
  failures += TestRejectedPulseCommandIsAtomic();
  failures += TestAcceptedPulseCommandCommitsOnce();
  failures += TestThrottleCommandValidationAndCommit();
  failures += TestRejectedBatchLeavesAllTargets();
  /* 控制周期在可调度边界内消费目标，结束后必须恢复调度。 */
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("scheduler balanced", g_scheduler_depth, 0U);
  if (failures != 0) { printf("command motor tests failed: %d\n", failures); return 1; }
  printf("command motor tests passed\n");
  return 0;
}
