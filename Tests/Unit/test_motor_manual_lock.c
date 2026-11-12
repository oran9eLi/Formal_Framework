/**
 * @file test_motor_manual_lock.c
 * @brief 真实 Control、Command 和 App API 的人工锁与电压门禁回归。
 * @details 仅平台 I/O、输入快照和日志出口由本文件替代；不模拟真实 ESC 转速，不代替实板验证。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef int BaseType_t;
typedef void *TaskHandle_t;
static uint32_t s_test_now;
static unsigned s_test_scheduler_depth;
static unsigned s_test_critical_depth;
#define taskENTER_CRITICAL() (++s_test_critical_depth)
#define taskEXIT_CRITICAL() (--s_test_critical_depth)
#define vTaskSuspendAll() (++s_test_scheduler_depth)
#define xTaskResumeAll() (--s_test_scheduler_depth)

#include "../../Framework/Src/px4lite_control.c"
#include "../../Framework/Src/px4lite_command.c"
/** @brief 上锁/电压门禁回归不运行声光输出，反馈仅保留独立任务通知语义。 */
void Px4Lite_StartupToneNotifyRpiCommand(void) {}
#include "../../Business/Src/app_data_api.c"

static uint16_t s_test_pwm[PX4LITE_MOTOR_COUNT];
static Px4Lite_MotorOutputs_t s_test_motor;
static Px4Lite_Result_t s_test_init_result = PX4LITE_OK;
static Px4Lite_Result_t s_test_disarm_result = PX4LITE_OK;
static uint8_t s_test_write_fail;
static uint8_t s_test_key1;
static uint8_t s_test_battery_ready = 1U;
static uint8_t s_test_battery_valid = 1U;
static uint8_t s_test_battery_stale;
static uint32_t s_test_battery_mv = 12000U;
static unsigned s_test_battery_copies;
static uint8_t s_test_attitude_ready = 1U;
static Px4Lite_State_t s_test_imu_state = PX4LITE_STATE_ONLINE;
static Px4Lite_State_t s_test_baro_state = PX4LITE_STATE_ONLINE;
static Px4Lite_State_t s_test_control_state;
static uint16_t s_test_control_fault;
static uint8_t s_test_ack;
static unsigned s_test_log_count[PX4LITE_LOGMSG_COUNT];

uint32_t Px4Lite_PlatformGetMs(void) { return s_test_now; }
Px4Lite_Result_t Px4Lite_MotorInit(void)
{
  if (s_test_init_result != PX4LITE_OK) { return s_test_init_result; }
  return Px4Lite_MotorDisarmAll();
}
Px4Lite_Result_t Px4Lite_MotorWritePulseUs(uint8_t channel, uint16_t pulse_us)
{
  assert(s_test_scheduler_depth != 0U);
  if (s_test_write_fail && channel == 1U) { return PX4LITE_IO_ERROR; }
  s_test_pwm[channel] = pulse_us;
  return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_MotorDisarmAll(void)
{
  unsigned i;
  if (s_test_disarm_result != PX4LITE_OK) { return s_test_disarm_result; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) { s_test_pwm[i] = PX4LITE_CONTROL_ESC_MIN_PULSE_US; }
  return PX4LITE_OK;
}
uint8_t Px4Lite_ButtonPressed(Px4Lite_ButtonId_t button)
{ return (button == PX4LITE_BUTTON_KEY1) ? s_test_key1 : 0U; }
void Px4Lite_PublishMotor(const Px4Lite_MotorOutputs_t *out) { s_test_motor = *out; }
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out)
{ *out = s_test_motor; return out->header.valid ? PX4LITE_OK : PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out)
{
  ++s_test_battery_copies;
  if (!s_test_battery_ready) { return PX4LITE_NOT_READY; }
  memset(out, 0, sizeof(*out));
  out->header.valid = s_test_battery_valid;
  out->header.sample_time_ms = s_test_now - (s_test_battery_stale ? PX4LITE_BATTERY_MAX_AGE_MS + 1U : 0U);
  out->voltage_mv = s_test_battery_mv;
  return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *out)
{
  if (!s_test_attitude_ready) { return PX4LITE_NOT_READY; }
  memset(out, 0, sizeof(*out)); out->header.valid = 1U;
  out->header.sample_time_ms = s_test_now; out->valid_mask = PX4LITE_NAV_VALID_ATTITUDE;
  return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *out)
{
  memset(out, 0, sizeof(*out));
  if (id == PX4LITE_MODULE_IMU) { out->state = s_test_imu_state; return PX4LITE_OK; }
  if (id == PX4LITE_MODULE_BARO) { out->state = s_test_baro_state; return PX4LITE_OK; }
  return PX4LITE_NOT_READY;
}
void Px4Lite_SetExternalModuleState(Px4Lite_ModuleId_t id, Px4Lite_State_t state, uint16_t fault, uint32_t now_ms)
{ (void)now_ms; assert(id == PX4LITE_MODULE_CONTROL); s_test_control_state = state; s_test_control_fault = fault; }
uint8_t Px4Lite_LogMessageSeverity(uint16_t message) { (void)message; return 0U; }
void Px4Lite_LocalMsgLogEvent(uint16_t message, uint32_t now_ms, uint16_t fault, uint8_t source, uint8_t active)
{ (void)now_ms; (void)fault; (void)source; (void)active; assert(message < PX4LITE_LOGMSG_COUNT); ++s_test_log_count[message]; }
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void) { return 1U; }
void Px4Lite_MavlinkQueueCommandAck(uint16_t command, uint8_t result, uint8_t target_system,
                                  uint8_t target_component, Px4Lite_MavlinkLink_t link)
{ (void)command; (void)target_system; (void)target_component; (void)link; s_test_ack = result; }
void Px4Lite_MavlinkRecordCommandAck(uint16_t command, uint8_t result, uint32_t now_ms)
{ (void)command; (void)result; (void)now_ms; }

/** @brief 校验四路真实平台输出均为最小值。 */
static void TestMinimum(void)
{
  unsigned i;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) { assert(s_test_pwm[i] == PX4LITE_CONTROL_ESC_MIN_PULSE_US); }
}
/** @brief 通过真实 App 边界读取人工锁。 */
static uint8_t TestLocked(void)
{
  uint8_t locked = 0xFFU;
  assert(App_GetMotorLocked(&locked) == PX4LITE_OK);
  return locked;
}
/** @brief 每拍按 20 ms 真实控制周期推进，避免把任务失步混入正常状态测试。 */
static void TestAdvance(uint32_t duration_ms)
{
  uint32_t elapsed;
  for (elapsed = 0U; elapsed < duration_ms; elapsed += PX4LITE_CONTROL_PERIOD_MS) {
    s_test_now += PX4LITE_CONTROL_PERIOD_MS; Px4Lite_ControlRun(s_test_now);
  }
}
/** @brief 复位受控平台后执行真实启动；所有测试均从人工上锁开始。 */
static void TestReset(void)
{
  s_test_now = 1000U;
  s_test_key1 = 0U; s_test_write_fail = 0U;
  s_test_init_result = PX4LITE_OK; s_test_disarm_result = PX4LITE_OK;
  s_test_battery_ready = 1U; s_test_battery_valid = 1U; s_test_battery_stale = 0U;
  s_test_battery_mv = 12000U; s_test_battery_copies = 0U;
  s_test_attitude_ready = 1U;
  s_test_imu_state = PX4LITE_STATE_ONLINE; s_test_baro_state = PX4LITE_STATE_ONLINE;
  memset(&s_test_motor, 0, sizeof(s_test_motor));
  assert(Px4Lite_ControlModuleInit() == PX4LITE_OK);
  assert(TestLocked() == 1U);
  TestMinimum();
}
/** @brief 显式解锁完成后仍应无自动非零输出。 */
static void TestUnlockReady(void)
{
  assert(App_SetMotorLocked(0U) == PX4LITE_OK);
  assert(TestLocked() == 0U && s_test_motor.run_state == 0U);
  assert(s_test_motor.operation_state == PX4LITE_MOTOR_OP_ESC_PREPARING);
  TestMinimum();
  TestAdvance(PX4LITE_CONTROL_ESC_ARM_TIME_MS);
  TestMinimum();
  assert(s_esc_armed != 0U);
}
/** @brief 真实 MAVLink 命令边界保留 ACK 语义，拒绝不能改变人工锁。 */
static void TestCommand(uint16_t command, float p1, float p2, float p3, float p4, uint8_t expected_ack)
{
  assert(Px4Lite_CommandHandleMavlinkLong(command, 42U, 7U, 1U, 191U,
    p1, p2, p3, p4, PX4LITE_MAVLINK_LINK_LORA, s_test_now) == PX4LITE_OK);
  assert(s_test_ack == expected_ack);
}

/** @brief 开机等待、模式、姿态、单路/整体/下行命令和起降都不能绕过人工锁。 */
static void TestStartupAndEntries(void)
{
  uint8_t percent[4] = {30U, 30U, 30U, 30U};
  uint16_t pulse[4] = {1500U, 1500U, 1500U, 1500U};
  TestReset(); TestAdvance(3000U);
  assert(TestLocked() == 1U && s_esc_armed == 0U);
  assert(s_test_motor.operation_state == PX4LITE_MOTOR_OP_LOCKED && s_test_motor.run_state == 0U);
  assert(App_SetMotorLocked(2U) == PX4LITE_INVALID_PARAM);
  assert(App_GetMotorLocked(0) == PX4LITE_INVALID_PARAM);
  assert(App_SetMotorControlMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST) == PX4LITE_OK);
  assert(Px4Lite_ControlSetAttitudeTarget(100, -100, 100) == PX4LITE_OK);
  assert(App_SetMotorThrottlePercent(0U, 30U) == PX4LITE_NOT_READY);
  assert(Px4Lite_ControlSetMotorPulseUs(0U, 1500U) == PX4LITE_NOT_READY);
  assert(Px4Lite_ControlSetMotorThrottles(percent) == PX4LITE_NOT_READY);
  assert(Px4Lite_ControlSetMotorPulses(pulse) == PX4LITE_NOT_READY);
  assert(App_StartMotorAutoTakeoff() == PX4LITE_NOT_READY);
  TestCommand(31011U, 30, 30, 30, 30, MAV_RESULT_TEMPORARILY_REJECTED);
  TestCommand(31013U, 1500, 1500, 1500, 1500, MAV_RESULT_TEMPORARILY_REJECTED);
  TestCommand(31012U, 0, 0, 0, 0, MAV_RESULT_ACCEPTED);
  TestCommand(31091U, 0, 0, 0, 0, MAV_RESULT_TEMPORARILY_REJECTED);
  TestCommand(31092U, 0, 0, 0, 0, MAV_RESULT_ACCEPTED);
  assert(App_SetMotorThrottlePercent(0U, 0U) == PX4LITE_OK);
  assert(TestLocked() == 1U); TestMinimum();
}

/** @brief 默认旁路覆盖缺失/无效/过期/低压/高压，配置恢复后同样的输入必须闭锁。 */
static void TestVoltagePolicy(void)
{
  unsigned condition;
  for (condition = 0U; condition < 6U; ++condition) {
    TestReset();
    if (condition == 0U) { s_test_battery_ready = 0U; }
    if (condition == 1U) { s_test_battery_valid = 0U; }
    if (condition == 2U) { s_test_battery_stale = 1U; }
    if (condition == 3U) { s_test_battery_mv = 9800U; }
    if (condition == 4U) { s_test_battery_mv = 14000U; }
    if (condition == 5U) { s_test_battery_mv = 0U; }
#if PX4LITE_CONTROL_POWER_GATE_ENABLE
    assert(App_SetMotorLocked(0U) == PX4LITE_NOT_READY);
    assert(TestLocked() == 1U && App_IsMotorPowerInhibited() == 1U);
    assert(App_SetMotorThrottlePercent(0U, 50U) == PX4LITE_NOT_READY);
#else
    TestUnlockReady();
    assert(App_SetMotorThrottlePercent(0U, 50U) == PX4LITE_OK);
    Px4Lite_ControlRun(s_test_now);
    assert(s_test_pwm[0] == 1500U && App_IsMotorPowerInhibited() == 0U);
    assert(s_test_battery_copies == 0U);
#endif
  }
#if PX4LITE_CONTROL_POWER_GATE_ENABLE
  TestReset(); TestUnlockReady();
  assert(App_SetMotorThrottlePercent(0U, 40U) == PX4LITE_OK);
  Px4Lite_ControlRun(s_test_now); assert(s_test_pwm[0] == 1400U);
  s_test_battery_mv = 9800U; Px4Lite_ControlRun(s_test_now); TestMinimum();
  assert(App_IsMotorPowerInhibited() == 1U && s_target_valid[0] == 0U);
  s_test_battery_mv = 12000U; TestAdvance(PX4LITE_CONTROL_ESC_ARM_TIME_MS + PX4LITE_CONTROL_PERIOD_MS);
  TestMinimum(); /* 恢复电压不重放旧油门。 */
#endif
}

/** @brief 上锁同步撤收并发布真实零输出，解锁和重复解锁均不自动旋转。 */
static void TestLockAndEmergency(void)
{
  App_MotorSnapshot_t app_motor;
  TestReset(); TestUnlockReady();
  assert(App_SetMotorThrottlePercent(0U, 40U) == PX4LITE_OK);
  Px4Lite_ControlRun(s_test_now); assert(s_test_pwm[0] == 1400U);
  assert(App_SetMotorLocked(0U) == PX4LITE_OK); /* 幂等，不改变正在运行的明确目标。 */
  assert(s_test_pwm[0] == 1400U && s_target_throttle_percent[0] == 40U);
  assert(App_SetMotorLocked(1U) == PX4LITE_OK); TestMinimum();
  assert(TestLocked() == 1U && s_target_valid[0] == 0U);
  assert(App_CopyMotor(&app_motor, s_test_now) == PX4LITE_OK);
  assert(app_motor.duty_percent[0] == 0U && app_motor.base_percent[0] == 0U);
  assert(app_motor.operation_state == PX4LITE_MOTOR_OP_LOCKED && app_motor.run_state == 0U);
  TestUnlockReady(); TestMinimum();
  assert(App_SetMotorThrottlePercent(0U, 35U) == PX4LITE_OK);
  Px4Lite_ControlRun(s_test_now); assert(s_test_pwm[0] == 1350U);
  TestCommand(31090U, 0, 0, 0, 0, MAV_RESULT_ACCEPTED);
  assert(TestLocked() == 1U && s_estop_latched == 1U); TestMinimum();
  TestCommand(31011U, 60, 60, 60, 60, MAV_RESULT_TEMPORARILY_REJECTED);
  assert(App_StartMotorAutoTakeoff() == PX4LITE_NOT_READY);
  assert(App_SetMotorThrottlePercent(0U, 0U) == PX4LITE_OK && s_estop_latched == 1U);
  s_test_key1 = 1U; assert(App_SetMotorLocked(0U) == PX4LITE_NOT_READY);
  s_test_key1 = 0U; TestUnlockReady(); assert(s_estop_latched == 0U); TestMinimum();
  s_test_key1 = 1U; TestAdvance(2U * PX4LITE_CONTROL_PERIOD_MS);
  assert(TestLocked() == 1U && s_estop_latched == 1U); TestMinimum();
}

/** @brief 控制失步、写入失败和恢复仍撤收并重新上锁，不受电压旁路影响。 */
static void TestNonVoltageFaultsAndRecovery(void)
{
  uint32_t confirmed_sequence;
  TestReset(); TestUnlockReady();
  assert(App_SetMotorThrottlePercent(0U, 50U) == PX4LITE_OK);
  Px4Lite_ControlRun(s_test_now); assert(s_test_pwm[0] == 1500U);
  s_test_now += PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS; Px4Lite_ControlRun(s_test_now);
  assert(TestLocked() == 0U && s_test_pwm[0] == 1500U);
  s_test_now += PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS + 1U; Px4Lite_ControlRun(s_test_now);
  assert(TestLocked() == 1U && s_target_valid[0] == 0U); TestMinimum();
  assert(App_SetMotorThrottlePercent(0U, 50U) == PX4LITE_NOT_READY);
  TestUnlockReady();
  assert(App_SetMotorThrottlePercent(0U, 50U) == PX4LITE_OK);
  s_test_write_fail = 1U; Px4Lite_ControlRun(s_test_now);
  assert(TestLocked() == 1U && s_test_motor.run_state == 0U); TestMinimum();
  assert(s_test_control_state == PX4LITE_STATE_DEGRADED && s_test_control_fault == PX4LITE_FAULT_SYSTEM_SELF_CHECK);
  s_test_write_fail = 0U; TestUnlockReady();
  assert(App_SetMotorThrottlePercent(0U, 30U) == PX4LITE_OK);
  Px4Lite_ControlRun(s_test_now); assert(s_test_pwm[0] == 1300U);
  s_test_disarm_result = PX4LITE_IO_ERROR;
  assert(App_SetMotorLocked(1U) == PX4LITE_IO_ERROR && TestLocked() == 1U);
  assert(s_test_motor.duty_percent[0] == 30U); /* 撤收失败不伪造已成功写入零输出的 topic。 */
  confirmed_sequence = s_test_motor.header.sequence;
  s_test_write_fail = 1U; TestAdvance(3U * PX4LITE_CONTROL_PERIOD_MS);
  assert(s_test_motor.header.sequence == confirmed_sequence && s_test_motor.duty_percent[0] == 30U);
  assert(TestLocked() == 1U && s_test_control_state == PX4LITE_STATE_DEGRADED);
  assert(App_SetMotorLocked(0U) == PX4LITE_IO_ERROR && TestLocked() == 1U);
  s_test_write_fail = 0U;
  s_test_disarm_result = PX4LITE_OK; TestUnlockReady(); TestMinimum();

  /* 运行中 PWM 部分失败且撤收也失败：不能发布无法确认的部分输出或零输出。 */
  assert(App_SetMotorThrottlePercent(0U, 40U) == PX4LITE_OK);
  Px4Lite_ControlRun(s_test_now); confirmed_sequence = s_test_motor.header.sequence;
  assert(App_SetMotorThrottlePercent(0U, 60U) == PX4LITE_OK);
  s_test_write_fail = 1U; s_test_disarm_result = PX4LITE_IO_ERROR;
  Px4Lite_ControlRun(s_test_now);
  assert(TestLocked() == 1U && s_test_motor.header.sequence == confirmed_sequence);
  assert(s_test_motor.duty_percent[0] == 40U);
  TestAdvance(3U * PX4LITE_CONTROL_PERIOD_MS);
  assert(s_test_motor.header.sequence == confirmed_sequence);
  s_test_write_fail = 0U; TestAdvance(PX4LITE_CONTROL_PERIOD_MS);
  assert(s_test_motor.header.sequence != confirmed_sequence && s_test_motor.duty_percent[0] == 0U);
  s_test_disarm_result = PX4LITE_OK; TestUnlockReady();
  assert(App_SetMotorThrottlePercent(0U, 30U) == PX4LITE_OK);
  Px4Lite_ControlRun(s_test_now);
  assert(Px4Lite_ControlRecover() == PX4LITE_OK && TestLocked() == 1U);
  Px4Lite_ControlRun(s_test_now); TestMinimum();
  assert(s_target_valid[0] == 0U && App_SetMotorThrottlePercent(0U, 30U) == PX4LITE_NOT_READY);
}

/** @brief 起降保留传感器/撤收规则，手动上锁中止斜坡且不重放；维护也拒绝解锁。 */
static void TestRampAndMaintenance(void)
{
  TestReset(); TestUnlockReady();
  assert(App_SetMotorControlMode(PX4LITE_CONTROL_MODE_DIRECT) == PX4LITE_OK);
  s_test_imu_state = PX4LITE_STATE_OFFLINE;
  assert(App_StartMotorAutoTakeoff() == PX4LITE_NOT_READY);
  assert(App_SetMotorThrottlePercent(0U, 20U) == PX4LITE_OK); /* 手动直控保留原来不依赖姿态的规则。 */
  s_test_imu_state = PX4LITE_STATE_ONLINE;
  assert(App_StartMotorAutoTakeoff() == PX4LITE_OK); TestAdvance(1000U);
  assert(s_auto_profile == PX4LITE_CONTROL_AUTO_TAKEOFF && s_test_pwm[0] > 1000U);
  assert(App_SetMotorLocked(1U) == PX4LITE_OK && s_auto_profile == PX4LITE_CONTROL_AUTO_NONE); TestMinimum();
  assert(s_control_mode == PX4LITE_CONTROL_MODE_DIRECT);
  TestUnlockReady(); TestMinimum();
  assert(App_StartMotorAutoTakeoff() == PX4LITE_OK); TestAdvance(1000U);
  assert(App_StartMotorAutoLanding() == PX4LITE_OK);
  assert(App_SetMotorThrottlePercent(0U, 50U) == PX4LITE_NOT_READY);
  assert(App_SetMotorLocked(1U) == PX4LITE_OK && s_auto_profile == PX4LITE_CONTROL_AUTO_NONE); TestMinimum();
  Px4Lite_ControlRun(s_test_now);
  assert(Px4Lite_ControlBeginConfigMaintenance() == PX4LITE_OK);
  assert(App_SetMotorLocked(0U) == PX4LITE_NOT_READY);
  assert(App_SetMotorThrottlePercent(0U, 30U) == PX4LITE_NOT_READY);
  assert(Px4Lite_ControlCheckConfigMaintenance() == PX4LITE_OK);
  Px4Lite_ControlEndConfigMaintenance(); assert(TestLocked() == 1U);
  TestUnlockReady(); TestMinimum();
}

int main(void)
{
  uint8_t unknown = 0xA5U;
  assert(App_GetMotorLocked(&unknown) == PX4LITE_NOT_READY && unknown == 0xA5U);
  assert(App_SetMotorLocked(0U) == PX4LITE_NOT_READY);
  TestStartupAndEntries();
  TestVoltagePolicy();
  TestLockAndEmergency();
  TestNonVoltageFaultsAndRecovery();
  TestRampAndMaintenance();
  TestReset(); s_test_now = UINT32_MAX - 999U;
  assert(Px4Lite_ControlRecover() == PX4LITE_OK); TestUnlockReady(); TestMinimum();
  assert(s_test_scheduler_depth == 0U && s_test_critical_depth == 0U);
  assert(s_test_log_count[PX4LITE_LOGMSG_MOTOR_MANUAL_UNLOCKED] != 0U);
  assert(s_test_log_count[PX4LITE_LOGMSG_MOTOR_MANUAL_LOCKED] != 0U);
  assert(s_test_log_count[PX4LITE_LOGMSG_THROTTLE_LOCKED] != 0U);
  s_test_init_result = PX4LITE_IO_ERROR;
  assert(Px4Lite_ControlModuleInit() == PX4LITE_IO_ERROR);
  assert(App_GetMotorLocked(&unknown) == PX4LITE_NOT_READY);
  assert(App_SetMotorLocked(0U) == PX4LITE_NOT_READY);
  printf("manual motor lock: voltage_gate=%u passed\n", (unsigned)PX4LITE_CONTROL_POWER_GATE_ENABLE);
  return 0;
}
