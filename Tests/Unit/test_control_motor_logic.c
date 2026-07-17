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
static Px4Lite_State_t g_imu_state;
static Px4Lite_State_t g_baro_state;

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
  g_imu_state     = PX4LITE_STATE_ONLINE;
  g_baro_state    = PX4LITE_STATE_ONLINE;
  g_now_ms        = 1000U;
  /* 动力电池闭锁是外部供电条件，Px4Lite_ControlReset() 故意不清它(见该函数说明)，
     因此必须由测试夹具显式复位，否则用例之间会互相污染。 */
  (void)Px4Lite_ControlSetPowerInhibit(0U);
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

Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleStatus_t *out)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  memset(out, 0, sizeof(*out));
  if (module_id == PX4LITE_MODULE_IMU) {
    out->state = g_imu_state;
    return PX4LITE_OK;
  }
  if (module_id == PX4LITE_MODULE_BARO) {
    out->state = g_baro_state;
    return PX4LITE_OK;
  }
  return PX4LITE_NOT_READY;
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

/**
 * @brief 按 Control 真实任务周期推进时间，并在每周期提供水平姿态。
 *
 * @param[in] duration_ms 推进时长，单位 ms，必须为 Control 周期的整数倍。
 */
static void RunControlWithLevelAttitude(uint32_t duration_ms)
{
  uint32_t elapsed_ms;

  for (elapsed_ms = 0U; elapsed_ms < duration_ms; elapsed_ms += PX4LITE_CONTROL_PERIOD_MS) {
    g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
    SeedAttitude(g_now_ms, 0, 0, 0, 0);
    Px4Lite_ControlRun(g_now_ms);
  }
}

/**
 * @brief 按 Control 真实任务周期推进时间，不补充新的姿态快照。
 *
 * @param[in] duration_ms 推进时长，单位 ms，必须为 Control 周期的整数倍。
 */
static void RunControlWithoutAttitude(uint32_t duration_ms)
{
  uint32_t elapsed_ms;

  for (elapsed_ms = 0U; elapsed_ms < duration_ms; elapsed_ms += PX4LITE_CONTROL_PERIOD_MS) {
    g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
    Px4Lite_ControlRun(g_now_ms);
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
  failures += ExpectU32("published pulse0", g_last_motor.pulse_us[0], expected_mid);
  failures += ExpectU32("published run", g_last_motor.run_state, 1U);
  failures += ExpectU32("control state", g_control_state, PX4LITE_STATE_ONLINE);
  failures += ExpectU32("control fault", g_control_fault, 0U);
  failures += ExpectU32("publish count nonzero", (g_publish_count > 0U) ? 1U : 0U, 1U);
  return failures;
}

static int TestThrottleTargetPublishesDuringArm(void)
{
  int failures = 0;

  ResetHarness();
  (void)Px4Lite_ControlModuleInit();
  failures += ExpectU32("set motor0 while arming", Px4Lite_ControlSetMotorThrottlePercent(0U, 40U), PX4LITE_OK);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("arming pulse stays min", g_pulse_us[0], PX4LITE_CONTROL_ESC_MIN_PULSE_US);
  failures += ExpectU32("arming publishes target", g_last_motor.duty_percent[0], 40U);
  failures += ExpectU32("arming publishes actual min pulse", g_last_motor.pulse_us[0], PX4LITE_CONTROL_ESC_MIN_PULSE_US);
  failures += ExpectU32("arming run state", g_last_motor.run_state, 0U);
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
  failures += ExpectU32("single active corrected duty0", g_last_motor.duty_percent[0], 37U);
  failures += ExpectU32("single active corrected pulse0", g_pulse_us[0], 1370U);
  failures += ExpectU32("inactive duty1", g_last_motor.duty_percent[1], 0U);
  failures += ExpectU32("inactive pulse1", g_pulse_us[1], PX4LITE_CONTROL_ESC_MIN_PULSE_US);
  return failures;
}

static int TestAttitudeAssistAdjustsPulseForSmallWindDisturbance(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(1U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(2U, 40U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(3U, 40U);
  SeedAttitude(g_now_ms, 100, 0, 0, 0);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("wind roll left pulse0", g_pulse_us[0], 1397U);
  failures += ExpectU32("wind roll right pulse1", g_pulse_us[1], 1403U);
  failures += ExpectU32("wind roll left pulse2", g_pulse_us[2], 1397U);
  failures += ExpectU32("wind roll right pulse3", g_pulse_us[3], 1403U);
  failures += ExpectU32("wind published pulse0", g_last_motor.pulse_us[0], 1397U);
  failures += ExpectU32("wind published pulse1", g_last_motor.pulse_us[1], 1403U);

  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("level pulse0 restored", g_pulse_us[0], 1400U);
  failures += ExpectU32("level pulse1 restored", g_pulse_us[1], 1400U);
  failures += ExpectU32("level published pulse0 restored", g_last_motor.pulse_us[0], 1400U);
  return failures;
}

/**
 * @brief 校验姿态修正不污染发布的基础油门。
 *
 * @details base_percent 是"用户锁存目标"的真值，遥测和地面站按它判断用户意图；duty_percent
 * 才含修正。本用例锁住这一分离：无论修正跑多久，base 都必须等于用户设定。
 *
 * 注意滑轨本身跟随 duty_percent(实验要求从四路显示上读出倾斜差异)，把显示值回灌成新目标的
 * 棘轮风险由 Display_LvglMotorSliderEventCb() 的触摸锁存消除，不由本字段承担。
 */
static int TestAttitudeAssistKeepsBaseThrottleStable(void)
{
  int failures = 0;
  uint8_t cycle;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 30U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(1U, 30U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(2U, 30U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(3U, 30U);

  /* 持续给一个固定的 10 度横滚偏差，模拟未做水平校准的姿态零点误差。 */
  for (cycle = 0U; cycle < 50U; cycle++) {
    g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
    SeedAttitude(g_now_ms, 1000, 0, 0, 0);
    Px4Lite_ControlRun(g_now_ms);
  }

  /* 无论跑多少周期，四路基础油门都必须原样保持 30%。 */
  failures += ExpectU32("base0 stable under assist", g_last_motor.base_percent[0], 30U);
  failures += ExpectU32("base1 stable under assist", g_last_motor.base_percent[1], 30U);
  failures += ExpectU32("base2 stable under assist", g_last_motor.base_percent[2], 30U);
  failures += ExpectU32("base3 stable under assist", g_last_motor.base_percent[3], 30U);
  /* 而实际输出必须确实带上修正(roll 混控 {+1,-1,+1,-1}，10 度 -> 3%)。 */
  failures += ExpectU32("duty0 carries correction", g_last_motor.duty_percent[0], 27U);
  failures += ExpectU32("duty1 carries correction", g_last_motor.duty_percent[1], 33U);

  /* 预解锁期间硬件停在 1000us，但 base 仍须反映已接受的目标，不能被当成掉档。 */
  ResetHarness();
  (void)Px4Lite_ControlModuleInit();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 45U);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("base kept while esc arming", g_last_motor.base_percent[0], 45U);
  failures += ExpectU32("pulse safe while esc arming", g_last_motor.pulse_us[0], PX4LITE_CONTROL_ESC_MIN_PULSE_US);
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

static int TestAttitudeAssistBridgesTransientNavigationGap(void)
{
  uint8_t i;
  int failures = 0;

  ResetHarness();
  ArmControl();
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    (void)Px4Lite_ControlSetMotorThrottlePercent(i, 40U);
  }
  SeedAttitude(g_now_ms, 1000, 0, 0, 0);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("gap seed corrected duty0", g_last_motor.duty_percent[0], 37U);

  g_navigation_ready = 0U;
  g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("short navigation gap keeps correction", g_last_motor.duty_percent[0], 37U);

  for (i = 0U; i <= (PX4LITE_IMU_MAX_AGE_MS / PX4LITE_CONTROL_PERIOD_MS); i++) {
    g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
    Px4Lite_ControlRun(g_now_ms);
  }
  failures += ExpectU32("expired attitude cache returns direct", g_last_motor.duty_percent[0], 40U);
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

static int TestManualThrottleReleasesEmergencyStop(void)
{
  uint32_t elapsed;
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
  failures += ExpectU32("estop clears before release", g_last_motor.duty_percent[2], 0U);

  g_button_pressed[PX4LITE_BUTTON_KEY1] = 0U;
  failures += ExpectU32("manual throttle after estop", Px4Lite_ControlSetMotorThrottlePercent(2U, 35U), PX4LITE_OK);
  for (elapsed = 0U; elapsed <= PX4LITE_CONTROL_ESC_ARM_TIME_MS; elapsed += PX4LITE_CONTROL_PERIOD_MS) {
    g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
    Px4Lite_ControlRun(g_now_ms);
  }
  g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("manual throttle restored duty", g_last_motor.duty_percent[2], 35U);
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
  failures += ExpectU32("failsafe accepts new throttle", Px4Lite_ControlSetMotorThrottlePercent(3U, 20U), PX4LITE_OK);
  return failures;
}

static int TestManualThrottleIgnoresAttitudeOffline(void)
{
  uint16_t expected = (uint16_t)(PX4LITE_CONTROL_ESC_MIN_PULSE_US + (((PX4LITE_CONTROL_ESC_MAX_PULSE_US - PX4LITE_CONTROL_ESC_MIN_PULSE_US) * 45U) / 100U));
  int failures = 0;

  ResetHarness();
  ArmControl();
  g_imu_state = PX4LITE_STATE_OFFLINE;
  g_navigation_ready = 0U;
  failures += ExpectU32("manual attitude offline set", Px4Lite_ControlSetMotorThrottlePercent(0U, 45U), PX4LITE_OK);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("manual attitude offline duty", g_last_motor.duty_percent[0], 45U);
  failures += ExpectU32("manual attitude offline pulse", g_pulse_us[0], expected);
  return failures;
}

static int TestAutoTakeoffDropsToLandingOnAttitudeLoss(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("auto takeoff start", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_OK);
  Px4Lite_ControlRun(g_now_ms);

  RunControlWithLevelAttitude(PX4LITE_CONTROL_AUTO_TAKEOFF_RAMP_MS / 2U);
  failures += ExpectU32("auto takeoff half duty", g_last_motor.duty_percent[0], PX4LITE_CONTROL_AUTO_TAKEOFF_TARGET_PERCENT / 2U);

  g_imu_state = PX4LITE_STATE_OFFLINE;
  g_navigation_ready = 0U;
  g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("attitude loss enters landing", s_auto_profile, PX4LITE_CONTROL_AUTO_LANDING);
  failures += ExpectU32("attitude loss holds current duty", g_last_motor.duty_percent[0], PX4LITE_CONTROL_AUTO_TAKEOFF_TARGET_PERCENT / 2U);

  RunControlWithoutAttitude(1000U);
  failures += ExpectU32("forced landing ramps down", (g_last_motor.duty_percent[0] < (PX4LITE_CONTROL_AUTO_TAKEOFF_TARGET_PERCENT / 2U)) ? 1U : 0U, 1U);

  failures += ExpectU32("manual override during forced landing", Px4Lite_ControlSetMotorThrottlePercent(0U, 45U), PX4LITE_OK);
  failures += ExpectU32("manual override cancels auto landing", s_auto_profile, PX4LITE_CONTROL_AUTO_NONE);
  g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("manual override works with attitude offline", g_last_motor.duty_percent[0], 45U);
  return failures;
}

static int TestAutoTakeoffCompletesAndHoldsTarget(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  failures += ExpectU32("set direct before takeoff", Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_DIRECT), PX4LITE_OK);
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("completed takeoff start", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_OK);
  failures += ExpectU32("takeoff restores attitude mode", s_control_mode, PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);
  Px4Lite_ControlRun(g_now_ms);

  RunControlWithLevelAttitude(PX4LITE_CONTROL_AUTO_TAKEOFF_RAMP_MS);
  failures += ExpectU32("takeoff profile completes", s_auto_profile, PX4LITE_CONTROL_AUTO_NONE);
  failures += ExpectU32("takeoff holds target duty", g_last_motor.duty_percent[0], PX4LITE_CONTROL_AUTO_TAKEOFF_TARGET_PERCENT);
  failures += ExpectU32("takeoff holds target pulse", g_last_motor.pulse_us[0], 1600U);

  g_navigation_ready = 0U;
  g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("completed takeoff does not re-enter landing", s_auto_profile, PX4LITE_CONTROL_AUTO_NONE);
  failures += ExpectU32("completed takeoff remains at target", g_last_motor.duty_percent[0], PX4LITE_CONTROL_AUTO_TAKEOFF_TARGET_PERCENT);
  return failures;
}

static int TestAutoTakeoffRequiresAttitudeAndEnvironmentOnline(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("takeoff sensors online", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_OK);

  ResetHarness();
  ArmControl();
  g_imu_state = PX4LITE_STATE_DEGRADED;
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("takeoff degraded imu with fresh attitude", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_OK);

  ResetHarness();
  ArmControl();
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  Px4Lite_ControlRun(g_now_ms);
  g_navigation_ready = 0U;
  failures += ExpectU32("takeoff uses periodically cached attitude", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_OK);

  ResetHarness();
  ArmControl();
  failures += ExpectU32("takeoff attitude not fresh", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_NOT_READY);

  ResetHarness();
  ArmControl();
  g_imu_state = PX4LITE_STATE_OFFLINE;
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("takeoff attitude offline", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_NOT_READY);

  ResetHarness();
  ArmControl();
  g_baro_state = PX4LITE_STATE_DEGRADED;
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("takeoff environment not online", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_NOT_READY);

  ResetHarness();
  ArmControl();
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  (void)Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_DIRECT);
  failures += ExpectU32("landing attitude online", Px4Lite_ControlStartAutoLanding(), PX4LITE_OK);
  failures += ExpectU32("landing restores attitude mode", s_control_mode, PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST);

  ResetHarness();
  ArmControl();
  g_imu_state = PX4LITE_STATE_STARTING;
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("landing starting imu with fresh attitude", Px4Lite_ControlStartAutoLanding(), PX4LITE_OK);

  /* 撤收动作不设姿态准入：姿态过期或 IMU 离线时更需要能把油门收回 0，
     否则飞机会卡在当前油门上。 */
  ResetHarness();
  ArmControl();
  failures += ExpectU32("landing without any attitude", Px4Lite_ControlStartAutoLanding(), PX4LITE_OK);

  ResetHarness();
  ArmControl();
  g_imu_state = PX4LITE_STATE_OFFLINE;
  failures += ExpectU32("landing with imu offline", Px4Lite_ControlStartAutoLanding(), PX4LITE_OK);

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 60U);
  g_imu_state = PX4LITE_STATE_FAILED;
  failures += ExpectU32("landing with imu failed", Px4Lite_ControlStartAutoLanding(), PX4LITE_OK);
  failures += ExpectU32("landing profile started", s_auto_profile, PX4LITE_CONTROL_AUTO_LANDING);
  RunControlWithoutAttitude(PX4LITE_CONTROL_AUTO_LANDING_RAMP_MS + PX4LITE_CONTROL_PERIOD_MS);
  failures += ExpectU32("landing reaches zero without attitude", g_last_motor.duty_percent[0], 0U);
  failures += ExpectU32("landing clears auto profile", s_auto_profile, PX4LITE_CONTROL_AUTO_NONE);
  return failures;
}

static int TestAutoLandingBlocksTakeoffAndEmergencyStopsImmediately(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("auto takeoff start for transition", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_OK);
  Px4Lite_ControlRun(g_now_ms);

  RunControlWithLevelAttitude(PX4LITE_CONTROL_AUTO_TAKEOFF_RAMP_MS / 2U);
  failures += ExpectU32("takeoff before landing duty", g_last_motor.duty_percent[0], PX4LITE_CONTROL_AUTO_TAKEOFF_TARGET_PERCENT / 2U);

  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("landing overrides takeoff", Px4Lite_ControlStartAutoLanding(), PX4LITE_OK);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("takeoff blocked while landing", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_BUSY);
  g_imu_state = PX4LITE_STATE_OFFLINE;
  failures += ExpectU32("takeoff busy wins sensor gate", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_BUSY);
  g_imu_state = PX4LITE_STATE_ONLINE;

  RunControlWithLevelAttitude(1000U);
  failures += ExpectU32("landing ramps down", (g_last_motor.duty_percent[0] < (PX4LITE_CONTROL_AUTO_TAKEOFF_TARGET_PERCENT / 2U)) ? 1U : 0U, 1U);

  failures += ExpectU32("estop during landing", Px4Lite_ControlEmergencyStop(g_now_ms), PX4LITE_OK);
  failures += ExpectU32("estop pulse immediate", g_pulse_us[0], PX4LITE_CONTROL_ESC_MIN_PULSE_US);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("estop duty zero", g_last_motor.duty_percent[0], 0U);
  return failures;
}

/**
 * @brief 校验动力电池闭锁：清零已有目标、拒绝新的非零油门、解除后不自动恢复。
 *
 * @details 《无人机动力系统安全控制实验》4.3 要求断开/没电/供电不足三档"四路油门清零"。
 * 只在输入口拒绝新命令是不够的：电机已经转起来之后电压才跌破门限时，Control 锁存的旧目标
 * 不会自己消失。本用例锁住"闭锁沿必须清零已锁存目标"这一条。
 */
static int TestPowerInhibitClearsAndRefusesThrottle(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 30U);
  (void)Px4Lite_ControlSetMotorThrottlePercent(1U, 30U);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("duty before inhibit", g_last_motor.duty_percent[0], 30U);

  /* 闭锁沿：已经在跑的四路必须立刻掉到 0，硬件回到最小脉宽。 */
  failures += ExpectU32("set inhibit", Px4Lite_ControlSetPowerInhibit(1U), PX4LITE_OK);
  failures += ExpectU32("inhibit reported", Px4Lite_ControlIsPowerInhibited(), 1U);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("inhibit zeroes duty0", g_last_motor.duty_percent[0], 0U);
  failures += ExpectU32("inhibit zeroes duty1", g_last_motor.duty_percent[1], 0U);
  failures += ExpectU32("inhibit zeroes base0", g_last_motor.base_percent[0], 0U);
  failures += ExpectU32("inhibit pulse safe", g_pulse_us[0], PX4LITE_CONTROL_ESC_MIN_PULSE_US);

  /* 闭锁期间只接受 0，一键起飞也必须被拒；一键降落不受限制。 */
  failures += ExpectU32("inhibit refuses throttle", Px4Lite_ControlSetMotorThrottlePercent(0U, 30U), PX4LITE_NOT_READY);
  failures += ExpectU32("inhibit accepts zero", Px4Lite_ControlSetMotorThrottlePercent(0U, 0U), PX4LITE_OK);
  failures += ExpectU32("inhibit refuses pulse", Px4Lite_ControlSetMotorPulseUs(0U, 1500U), PX4LITE_NOT_READY);
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("inhibit refuses takeoff", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_NOT_READY);
  failures += ExpectU32("inhibit allows landing", Px4Lite_ControlStartAutoLanding(), PX4LITE_OK);

  /* 被拒的油门命令不能顺带解锁急停：拒绝发生在解锁之前。 */
  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetPowerInhibit(1U);
  (void)Px4Lite_ControlEmergencyStop(g_now_ms);
  failures += ExpectU32("refused throttle keeps estop", Px4Lite_ControlSetMotorThrottlePercent(0U, 30U), PX4LITE_NOT_READY);
  failures += ExpectU32("estop still latched", s_estop_latched, 1U);

  /* 解除闭锁只是允许输出，不自动恢复任何油门；必须用户重新下发。 */
  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 30U);
  (void)Px4Lite_ControlSetPowerInhibit(1U);
  (void)Px4Lite_ControlSetPowerInhibit(0U);
  failures += ExpectU32("inhibit released", Px4Lite_ControlIsPowerInhibited(), 0U);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("release restores nothing", g_last_motor.duty_percent[0], 0U);
  failures += ExpectU32("throttle accepted after release", Px4Lite_ControlSetMotorThrottlePercent(0U, 30U), PX4LITE_OK);
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("duty after release", g_last_motor.duty_percent[0], 30U);
  return failures;
}

/**
 * @brief 校验一键降落进行中拒绝非零手动油门，且不被其打断。
 *
 * @details 《无人机动力系统安全控制实验》4.4.2 要求"降落过程中不应接受新的非零手动油门"，
 * 步骤 7 的口径是按降落按钮后观察手动输入被拒。因此只有用户发起的一键降落设此限制。
 *
 * 姿态丢失触发的强制降落不在此列：那是内部保护，锁住手动接管会让 IMU 掉线时操作员失去出口，
 * 该口径由 TestAutoTakeoffDropsToLandingOnAttitudeLoss() 锁住。一键起飞同样不设限制。
 */
static int TestLandingRefusesNonZeroManualThrottle(void)
{
  int failures = 0;

  ResetHarness();
  ArmControl();
  (void)Px4Lite_ControlSetMotorThrottlePercent(0U, 60U);
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("landing started", Px4Lite_ControlStartAutoLanding(), PX4LITE_OK);
  RunControlWithLevelAttitude(PX4LITE_CONTROL_AUTO_LANDING_RAMP_MS / 2U);

  /* 非零手动油门必须被拒，且降落斜坡不能被撤掉。 */
  failures += ExpectU32("landing refuses manual throttle", Px4Lite_ControlSetMotorThrottlePercent(0U, 40U), PX4LITE_NOT_READY);
  failures += ExpectU32("landing profile survives refusal", s_auto_profile, PX4LITE_CONTROL_AUTO_LANDING);
  failures += ExpectU32("landing refuses manual pulse", Px4Lite_ControlSetMotorPulseUs(0U, 1600U), PX4LITE_NOT_READY);
  failures += ExpectU32("landing profile survives pulse refusal", s_auto_profile, PX4LITE_CONTROL_AUTO_LANDING);
  /* 0 与降落终点一致，接受但同样不打断斜坡。 */
  failures += ExpectU32("landing accepts zero", Px4Lite_ControlSetMotorThrottlePercent(0U, 0U), PX4LITE_OK);
  failures += ExpectU32("landing profile survives zero", s_auto_profile, PX4LITE_CONTROL_AUTO_LANDING);

  /* 斜坡跑完必须落到 0 并自行结束。 */
  RunControlWithLevelAttitude(PX4LITE_CONTROL_AUTO_LANDING_RAMP_MS);
  failures += ExpectU32("landing completes to zero", g_last_motor.duty_percent[0], 0U);
  failures += ExpectU32("landing cleared", s_auto_profile, PX4LITE_CONTROL_AUTO_NONE);
  failures += ExpectU32("throttle accepted after landing", Px4Lite_ControlSetMotorThrottlePercent(0U, 40U), PX4LITE_OK);

  /* 一键起飞相反：手动接管可以随时打断上升。 */
  ResetHarness();
  ArmControl();
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("takeoff started", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_OK);
  RunControlWithLevelAttitude(PX4LITE_CONTROL_AUTO_TAKEOFF_RAMP_MS / 2U);
  failures += ExpectU32("takeoff accepts manual throttle", Px4Lite_ControlSetMotorThrottlePercent(0U, 10U), PX4LITE_OK);
  failures += ExpectU32("manual throttle cancels takeoff", s_auto_profile, PX4LITE_CONTROL_AUTO_NONE);

  /* 强制降落(姿态丢失)同样不设限制：这是内部保护，不能把操作员锁在外面。 */
  ResetHarness();
  ArmControl();
  SeedAttitude(g_now_ms, 0, 0, 0, 0);
  failures += ExpectU32("takeoff for forced landing", Px4Lite_ControlStartAutoTakeoff(), PX4LITE_OK);
  RunControlWithLevelAttitude(PX4LITE_CONTROL_AUTO_TAKEOFF_RAMP_MS / 2U);
  g_imu_state = PX4LITE_STATE_OFFLINE;
  g_navigation_ready = 0U;
  g_now_ms += PX4LITE_CONTROL_PERIOD_MS;
  Px4Lite_ControlRun(g_now_ms);
  failures += ExpectU32("forced landing entered", s_auto_profile, PX4LITE_CONTROL_AUTO_LANDING);
  failures += ExpectU32("forced landing is not user landing", s_auto_landing_user, 0U);
  failures += ExpectU32("forced landing accepts manual throttle", Px4Lite_ControlSetMotorThrottlePercent(0U, 45U), PX4LITE_OK);
  failures += ExpectU32("manual throttle cancels forced landing", s_auto_profile, PX4LITE_CONTROL_AUTO_NONE);
  return failures;
}

int main(void)
{
  int failures = 0;

  failures += TestThrottleMappingAndPublish();
  failures += TestThrottleTargetPublishesDuringArm();
  failures += TestThrottleClampAndInvalidIndex();
  failures += TestAttitudeAssistMixesFreshRoll();
  failures += TestAttitudeAssistDefaultsOnForSingleActiveOutput();
  failures += TestAttitudeAssistAdjustsPulseForSmallWindDisturbance();
  failures += TestAttitudeAssistPreservesUnequalBaseThrottle();
  failures += TestAttitudeAssistKeepsBaseThrottleStable();
  failures += TestAttitudeAssistCorrectsPositivePitch();
  failures += TestAttitudeAssistCorrectsPositiveYaw();
  failures += TestAttitudeTargetRejectsUnsafeRange();
  failures += TestAttitudeAssistClampsCorruptNavigation();
  failures += TestAttitudeAssistStaleFallsBackToDirect();
  failures += TestAttitudeAssistBridgesTransientNavigationGap();
  failures += TestKey1EmergencyStopClearsTargets();
  failures += TestManualThrottleReleasesEmergencyStop();
  failures += TestFailsafeGapRearmsAtMinimum();
  failures += TestManualThrottleIgnoresAttitudeOffline();
  failures += TestAutoTakeoffDropsToLandingOnAttitudeLoss();
  failures += TestAutoTakeoffCompletesAndHoldsTarget();
  failures += TestAutoTakeoffRequiresAttitudeAndEnvironmentOnline();
  failures += TestAutoLandingBlocksTakeoffAndEmergencyStopsImmediately();
  failures += TestPowerInhibitClearsAndRefusesThrottle();
  failures += TestLandingRefusesNonZeroManualThrottle();

  if (failures != 0) {
    printf("control motor tests failed: %d\n", failures);
    return 1;
  }

  printf("control motor tests passed\n");
  return 0;
}
