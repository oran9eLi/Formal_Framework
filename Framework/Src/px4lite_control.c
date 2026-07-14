/**
 * @file px4lite_control.c
 * @brief 实现电机油门命令、按键急停、ESC 预解锁和失效保护。
 *
 * @details
 * 本文件位于 Framework 层，只通过 Platform Adapter 写入 PWM 和读取按键。
 * Business/Display 通过 `App_SetMotorThrottlePercent()` 写入目标油门，Control
 * 任务按固定周期统一下发，避免多个任务同时拥有同一组电机输出。
 */

#include "px4lite_control.h"

#include <string.h>
#include "px4lite_config.h"
#include "px4lite_faults.h"
#include "px4lite_modules.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"

#if PX4LITE_ENABLE_CONTROL

#if PX4LITE_CONTROL_ESC_MAX_PULSE_US <= PX4LITE_CONTROL_ESC_MIN_PULSE_US
#error "PX4LITE_CONTROL_ESC_MAX_PULSE_US must be greater than PX4LITE_CONTROL_ESC_MIN_PULSE_US"
#endif

#if PX4LITE_MOTOR_COUNT != 4U
#error "Attitude assist mixer requires exactly four motor channels"
#endif

#if (PX4LITE_CONTROL_ATTITUDE_MIN_BASE_PERCENT < 1U) || (PX4LITE_CONTROL_ATTITUDE_MIN_BASE_PERCENT > 100U)
#error "PX4LITE_CONTROL_ATTITUDE_MIN_BASE_PERCENT must be within 1..100"
#endif

#define PX4LITE_CONTROL_DEBOUNCE_CYCLES 2U

static uint8_t s_esc_armed;
static uint8_t s_last_run_valid;
static volatile uint8_t s_target_throttle_percent[PX4LITE_MOTOR_COUNT];
static volatile uint32_t s_target_update_ms[PX4LITE_MOTOR_COUNT];
static volatile uint8_t s_target_valid[PX4LITE_MOTOR_COUNT];
static volatile Px4Lite_ControlMode_t s_control_mode;
static volatile int32_t s_target_roll_deg100;
static volatile int32_t s_target_pitch_deg100;
static volatile int32_t s_target_yaw_deg100;
static volatile uint8_t s_estop_latched;
static uint32_t s_arm_start_ms;
static uint32_t s_last_run_ms;
static uint32_t s_sequence;
static uint8_t s_button_stable[PX4LITE_BUTTON_COUNT];
static uint8_t s_button_last_raw[PX4LITE_BUTTON_COUNT];
static uint8_t s_button_count[PX4LITE_BUTTON_COUNT];
/*
 * X 机架通道约定：1=前左、2=前右、3=后左、4=后右。
 * 正 roll 表示右侧下沉，正 pitch 表示抬头，正 yaw 按 NED 约定为俯视顺时针。
 * 俯视旋转方向约定为 1/4 顺时针、2/3 逆时针，但物理转向由电调配置或电机相线决定。
 */
static const int8_t s_attitude_roll_mix[PX4LITE_MOTOR_COUNT]  = {1, -1, 1, -1};
static const int8_t s_attitude_pitch_mix[PX4LITE_MOTOR_COUNT] = {-1, -1, 1, 1};
static const int8_t s_attitude_yaw_mix[PX4LITE_MOTOR_COUNT]   = {-1, 1, 1, -1};

/**
 * @brief 将 0 到 100 的油门百分比映射为 ESC 高电平脉宽。
 *
 * @param[in] throttle_percent 油门百分比，超过 100 时按 100 限幅。
 *
 * @return PWM 高电平脉宽，单位：us。
 */
static uint16_t Px4Lite_ControlThrottleToPulseUs(uint8_t throttle_percent)
{
  uint32_t range;
  uint32_t pulse;

  if (throttle_percent > 100U) { throttle_percent = 100U; }

  range = (uint32_t)PX4LITE_CONTROL_ESC_MAX_PULSE_US - (uint32_t)PX4LITE_CONTROL_ESC_MIN_PULSE_US;
  pulse = (uint32_t)PX4LITE_CONTROL_ESC_MIN_PULSE_US + ((range * (uint32_t)throttle_percent) / 100U);
  return (uint16_t)pulse;
}

/**
 * @brief 将有符号定点输入限制到闭区间，避免后续乘加越界。
 */
static int32_t Px4Lite_ControlClampI32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value) { return min_value; }
  if (value > max_value) { return max_value; }
  return value;
}

/**
 * @brief 将混控结果限制到 0~100% 油门范围。
 */
static uint8_t Px4Lite_ControlClampDuty(int32_t duty_percent)
{
  if (duty_percent <= 0) { return 0U; }
  if (duty_percent >= 100) { return 100U; }
  return (uint8_t)duty_percent;
}

/**
 * @brief 计算单轴定点 PD 修正量并按百分比限幅。
 */
static int32_t Px4Lite_ControlAttitudeCorrection(int32_t angle_error_deg100, int32_t rate_error_dps100)
{
  int32_t correction;
  int32_t max_correction = (int32_t)PX4LITE_CONTROL_ATTITUDE_MAX_CORRECTION_PERCENT;

  angle_error_deg100 = Px4Lite_ControlClampI32(angle_error_deg100, -36000, 36000);
  rate_error_dps100 = Px4Lite_ControlClampI32(rate_error_dps100, -200000, 200000);
  correction = ((angle_error_deg100 * (int32_t)PX4LITE_CONTROL_ATTITUDE_KP_X100) +
                (rate_error_dps100 * (int32_t)PX4LITE_CONTROL_ATTITUDE_KD_X100)) / 10000;
  return Px4Lite_ControlClampI32(correction, -max_correction, max_correction);
}

/**
 * @brief 将偏航角误差折返到 -180~180 degree，避免跨越边界时走长路径。
 */
static int32_t Px4Lite_ControlWrapYawErrorDeg100(int32_t target_deg100, int32_t measured_deg100)
{
  int32_t error = target_deg100 - measured_deg100;

  if (error > 18000) { error -= 36000; }
  if (error < -18000) { error += 36000; }
  return error;
}

/**
 * @brief 复制新鲜且已标记有效的姿态快照。
 */
static uint8_t Px4Lite_ControlCopyFreshAttitude(Px4Lite_VehicleNavigation_t *navigation, uint32_t now_ms)
{
  if (navigation == 0) { return 0U; }
  if (Px4Lite_CopyNavigation(navigation) != PX4LITE_OK) { return 0U; }
  if (Px4Lite_IsFresh(&navigation->header, now_ms, PX4LITE_IMU_MAX_AGE_MS) == 0U) { return 0U; }
  if ((navigation->valid_mask & PX4LITE_NAV_VALID_ATTITUDE) == 0U) { return 0U; }
  return 1U;
}

/**
 * @brief 在全部安全前提满足时把 roll/pitch/yaw 修正叠加到四路基础油门。
 */
static void Px4Lite_ControlApplyAttitudeAssist(uint32_t now_ms, uint8_t duty_percent[PX4LITE_MOTOR_COUNT])
{
  Px4Lite_VehicleNavigation_t navigation;
  int32_t roll_correction;
  int32_t pitch_correction;
  int32_t yaw_correction;
  int32_t measured_roll_deg100;
  int32_t measured_pitch_deg100;
  int32_t measured_yaw_deg100;
  int32_t measured_roll_rate_dps100;
  int32_t measured_pitch_rate_dps100;
  int32_t measured_yaw_rate_dps100;
  int32_t mixed;
  uint8_t has_active_output = 0U;
  uint8_t i;

  if (s_control_mode != PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST) { return; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (duty_percent[i] >= PX4LITE_CONTROL_ATTITUDE_MIN_BASE_PERCENT) {
      has_active_output = 1U;
    }
  }
  if (has_active_output == 0U) { return; }
  if (Px4Lite_ControlCopyFreshAttitude(&navigation, now_ms) == 0U) { return; }

  measured_roll_deg100 = Px4Lite_ControlClampI32(navigation.roll_deg100, -18000, 18000);
  measured_pitch_deg100 = Px4Lite_ControlClampI32(navigation.pitch_deg100, -9000, 9000);
  measured_yaw_deg100 = Px4Lite_ControlClampI32(navigation.yaw_deg100, -18000, 18000);
  measured_roll_rate_dps100 = Px4Lite_ControlClampI32(navigation.roll_rate_dps100, -200000, 200000);
  measured_pitch_rate_dps100 = Px4Lite_ControlClampI32(navigation.pitch_rate_dps100, -200000, 200000);
  measured_yaw_rate_dps100 = Px4Lite_ControlClampI32(navigation.yaw_rate_dps100, -200000, 200000);
  roll_correction = Px4Lite_ControlAttitudeCorrection(s_target_roll_deg100 - measured_roll_deg100,
                                                       -measured_roll_rate_dps100);
  pitch_correction = Px4Lite_ControlAttitudeCorrection(s_target_pitch_deg100 - measured_pitch_deg100,
                                                        -measured_pitch_rate_dps100);
  yaw_correction = Px4Lite_ControlAttitudeCorrection(Px4Lite_ControlWrapYawErrorDeg100(s_target_yaw_deg100,
                                                                                       measured_yaw_deg100),
                                                      -measured_yaw_rate_dps100);

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    /* 仅修正已有基础油门的通道，避免单路台架测试时意外启动其余零油门电机。 */
    if (duty_percent[i] < PX4LITE_CONTROL_ATTITUDE_MIN_BASE_PERCENT) { continue; }
    mixed = (int32_t)duty_percent[i] +
            ((int32_t)s_attitude_roll_mix[i] * roll_correction) +
            ((int32_t)s_attitude_pitch_mix[i] * pitch_correction) +
            ((int32_t)s_attitude_yaw_mix[i] * yaw_correction);
    duty_percent[i] = Px4Lite_ControlClampDuty(mixed);
  }
}

/**
 * @brief 写入全部电机 PWM 脉宽。
 *
 * @param[in] pulse_us 每个电机的高电平脉宽数组，单位：us。
 *
 * @return 写入结果。
 */
static Px4Lite_Result_t Px4Lite_ControlWriteAll(const uint16_t pulse_us[PX4LITE_MOTOR_COUNT])
{
  uint8_t i;
  Px4Lite_Result_t result = PX4LITE_OK;

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (Px4Lite_MotorWritePulseUs(i, pulse_us[i]) != PX4LITE_OK) { result = PX4LITE_IO_ERROR; }
  }

  return result;
}

/**
 * @brief 清零所有目标油门。
 */
static void Px4Lite_ControlClearTargets(void)
{
  uint8_t i;

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    s_target_throttle_percent[i] = 0U;
    s_target_update_ms[i]        = 0U;
    s_target_valid[i]            = 0U;
  }
}

/**
 * @brief 复位控制状态并把电机输出降到最小脉宽。
 */
static void Px4Lite_ControlReset(uint32_t now_ms)
{
  s_esc_armed      = 0U;
  s_last_run_valid = 0U;
  s_estop_latched  = 0U;
  s_arm_start_ms   = now_ms;
  s_last_run_ms    = now_ms;
  s_control_mode    = (PX4LITE_CONTROL_ATTITUDE_ASSIST_DEFAULT != 0U) ? PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST : PX4LITE_CONTROL_MODE_DIRECT;
  s_target_roll_deg100  = PX4LITE_CONTROL_ATTITUDE_TARGET_ROLL_DEG100;
  s_target_pitch_deg100 = PX4LITE_CONTROL_ATTITUDE_TARGET_PITCH_DEG100;
  s_target_yaw_deg100   = PX4LITE_CONTROL_ATTITUDE_TARGET_YAW_DEG100;
  Px4Lite_ControlClearTargets();
  memset(s_button_stable, 0, sizeof(s_button_stable));
  memset(s_button_last_raw, 0, sizeof(s_button_last_raw));
  memset(s_button_count, 0, sizeof(s_button_count));
  (void)Px4Lite_MotorDisarmAll();
}

/**
 * @brief 对按键做简易去抖并返回按下边沿。
 *
 * @param[in] button 按键编号。
 *
 * @return 1 表示检测到稳定按下边沿，0 表示没有边沿。
 */
static uint8_t Px4Lite_ControlPressEdge(Px4Lite_ButtonId_t button)
{
  uint8_t raw;
  uint8_t edge = 0U;

  raw = Px4Lite_ButtonPressed(button);
  if (raw == s_button_last_raw[button]) {
    if (s_button_count[button] < PX4LITE_CONTROL_DEBOUNCE_CYCLES) { s_button_count[button]++; }
  } else {
    s_button_last_raw[button] = raw;
    s_button_count[button]    = 1U;
  }

  if ((s_button_count[button] >= PX4LITE_CONTROL_DEBOUNCE_CYCLES) && (raw != s_button_stable[button])) {
    s_button_stable[button] = raw;
    if (raw != 0U) { edge = 1U; }
  }

  return edge;
}

/**
 * @brief 发布当前电机输出状态。
 */
static void Px4Lite_ControlPublish(uint32_t now_ms, const uint8_t duty_percent[PX4LITE_MOTOR_COUNT], uint8_t run_state)
{
  Px4Lite_MotorOutputs_t outputs;
  uint8_t i;
  uint8_t max_throttle = 0U;

  memset(&outputs, 0, sizeof(outputs));
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    outputs.duty_percent[i] = duty_percent[i];
    if (duty_percent[i] > max_throttle) { max_throttle = duty_percent[i]; }
  }
  outputs.run_state                = run_state;
  outputs.speed_level              = max_throttle;
  outputs.header.sequence          = ++s_sequence;
  outputs.header.sample_time_ms    = now_ms;
  outputs.header.publish_time_ms   = now_ms;
  outputs.header.device_id         = (uint16_t)PX4LITE_MODULE_CONTROL;
  outputs.header.valid             = 1U;
  outputs.header.quality           = 100U;
  outputs.header.flags             = PX4LITE_DATA_VALID;
  Px4Lite_PublishMotor(&outputs);
}

Px4Lite_Result_t Px4Lite_ControlSetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent)
{
  if (motor_index >= PX4LITE_MOTOR_COUNT) { return PX4LITE_INVALID_PARAM; }
  if (throttle_percent > 100U) { throttle_percent = 100U; }
  if ((s_estop_latched != 0U) && (throttle_percent != 0U)) { return PX4LITE_BUSY; }

  s_target_throttle_percent[motor_index] = throttle_percent;
  s_target_update_ms[motor_index]        = Px4Lite_PlatformGetMs();
  s_target_valid[motor_index]            = 1U;
  return PX4LITE_OK;
}

/**
 * @brief 设置直控或姿态辅助输出模式。
 */
Px4Lite_Result_t Px4Lite_ControlSetMode(Px4Lite_ControlMode_t mode)
{
  if ((mode != PX4LITE_CONTROL_MODE_DIRECT) && (mode != PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST)) {
    return PX4LITE_INVALID_PARAM;
  }

  s_control_mode = mode;
  return PX4LITE_OK;
}

/**
 * @brief 复制当前输出模式。
 */
Px4Lite_Result_t Px4Lite_ControlGetMode(Px4Lite_ControlMode_t *mode)
{
  if (mode == 0) { return PX4LITE_INVALID_PARAM; }
  *mode = s_control_mode;
  return PX4LITE_OK;
}

/**
 * @brief 校验并设置姿态辅助目标角。
 */
Px4Lite_Result_t Px4Lite_ControlSetAttitudeTarget(int32_t roll_deg100, int32_t pitch_deg100, int32_t yaw_deg100)
{
  if ((roll_deg100 < -(int32_t)PX4LITE_CONTROL_ATTITUDE_TARGET_LIMIT_DEG100) ||
      (roll_deg100 > (int32_t)PX4LITE_CONTROL_ATTITUDE_TARGET_LIMIT_DEG100) ||
      (pitch_deg100 < -(int32_t)PX4LITE_CONTROL_ATTITUDE_TARGET_LIMIT_DEG100) ||
      (pitch_deg100 > (int32_t)PX4LITE_CONTROL_ATTITUDE_TARGET_LIMIT_DEG100) ||
      (yaw_deg100 < -(int32_t)PX4LITE_CONTROL_ATTITUDE_YAW_TARGET_LIMIT_DEG100) ||
      (yaw_deg100 > (int32_t)PX4LITE_CONTROL_ATTITUDE_YAW_TARGET_LIMIT_DEG100)) {
    return PX4LITE_INVALID_PARAM;
  }

  s_target_roll_deg100  = roll_deg100;
  s_target_pitch_deg100 = pitch_deg100;
  s_target_yaw_deg100   = yaw_deg100;
  return PX4LITE_OK;
}

/**
 * @brief 锁存急停状态、清零目标油门，并立即写入 ESC 最小脉宽。
 */
Px4Lite_Result_t Px4Lite_ControlEmergencyStop(uint32_t now_ms)
{
  Px4Lite_Result_t result;

  s_estop_latched  = 1U;
  s_esc_armed      = 0U;
  s_arm_start_ms   = now_ms;
  s_last_run_ms    = now_ms;
  s_last_run_valid = 1U;
  Px4Lite_ControlClearTargets();

  result = Px4Lite_MotorDisarmAll();
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL, PX4LITE_STATE_DEGRADED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SYSTEM_SELF_CHECK, now_ms);
  return result;
}

Px4Lite_Result_t Px4Lite_ControlModuleInit(void)
{
  uint32_t now_ms = Px4Lite_PlatformGetMs();
  Px4Lite_Result_t result;

  result = Px4Lite_MotorInit();
  Px4Lite_ControlReset(now_ms);
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL, (result == PX4LITE_OK) ? PX4LITE_STATE_STARTING : PX4LITE_STATE_DEGRADED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SYSTEM_SELF_CHECK, now_ms);
  return result;
}

Px4Lite_Result_t Px4Lite_ControlRecover(void)
{
  uint32_t now_ms = Px4Lite_PlatformGetMs();

  Px4Lite_ControlReset(now_ms);
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL, PX4LITE_STATE_STARTING, PX4LITE_FAULT_NONE, now_ms);
  return PX4LITE_OK;
}

void Px4Lite_ControlRun(uint32_t now_ms)
{
  uint16_t pulse_us[PX4LITE_MOTOR_COUNT];
  uint8_t duty_percent[PX4LITE_MOTOR_COUNT];
  uint8_t i;
  Px4Lite_Result_t result;

  if ((s_last_run_valid != 0U) && (Px4Lite_ElapsedMs(now_ms, s_last_run_ms) > PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS)) {
    s_estop_latched = 1U;
    s_esc_armed     = 0U;
    s_arm_start_ms  = now_ms;
    Px4Lite_ControlClearTargets();
    (void)Px4Lite_MotorDisarmAll();
  }
  s_last_run_valid = 1U;
  s_last_run_ms    = now_ms;

  if (Px4Lite_ControlPressEdge(PX4LITE_BUTTON_KEY1) != 0U) {
    s_estop_latched = 1U;
    Px4Lite_ControlClearTargets();
    (void)Px4Lite_MotorDisarmAll();
  }

  if (s_esc_armed == 0U) {
    for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
      duty_percent[i] = 0U;
      pulse_us[i]     = PX4LITE_CONTROL_ESC_MIN_PULSE_US;
    }
    result = Px4Lite_ControlWriteAll(pulse_us);
    if (Px4Lite_ElapsedMs(now_ms, s_arm_start_ms) >= PX4LITE_CONTROL_ESC_ARM_TIME_MS) { s_esc_armed = 1U; }
    Px4Lite_ControlPublish(now_ms, duty_percent, 0U);
    Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL, (result == PX4LITE_OK) ? PX4LITE_STATE_STARTING : PX4LITE_STATE_DEGRADED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SYSTEM_SELF_CHECK, now_ms);
    return;
  }

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if ((s_estop_latched != 0U) || (s_target_valid[i] == 0U)) {
      s_target_throttle_percent[i] = 0U;
      s_target_update_ms[i]        = 0U;
      s_target_valid[i]            = 0U;
      duty_percent[i]              = 0U;
    } else {
      duty_percent[i] = s_target_throttle_percent[i];
      if (duty_percent[i] > 100U) { duty_percent[i] = 100U; }
    }
  }

  if (s_estop_latched == 0U) { Px4Lite_ControlApplyAttitudeAssist(now_ms, duty_percent); }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    pulse_us[i] = Px4Lite_ControlThrottleToPulseUs(duty_percent[i]);
  }

  result = Px4Lite_ControlWriteAll(pulse_us);
  Px4Lite_ControlPublish(now_ms, duty_percent, (s_estop_latched == 0U) ? 1U : 0U);
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL, ((result == PX4LITE_OK) && (s_estop_latched == 0U)) ? PX4LITE_STATE_ONLINE : PX4LITE_STATE_DEGRADED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SYSTEM_SELF_CHECK, now_ms);
}

#else

Px4Lite_Result_t Px4Lite_ControlSetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent)
{
  (void)motor_index;
  (void)throttle_percent;
  return PX4LITE_OK;
}

/**
 * @brief Control 模块关闭时的模式设置空实现。
 */
Px4Lite_Result_t Px4Lite_ControlSetMode(Px4Lite_ControlMode_t mode)
{
  (void)mode;
  return PX4LITE_OK;
}

/**
 * @brief Control 模块关闭时返回直控模式。
 */
Px4Lite_Result_t Px4Lite_ControlGetMode(Px4Lite_ControlMode_t *mode)
{
  if (mode == 0) { return PX4LITE_INVALID_PARAM; }
  *mode = PX4LITE_CONTROL_MODE_DIRECT;
  return PX4LITE_OK;
}

/**
 * @brief Control 模块关闭时的姿态目标设置空实现。
 */
Px4Lite_Result_t Px4Lite_ControlSetAttitudeTarget(int32_t roll_deg100, int32_t pitch_deg100, int32_t yaw_deg100)
{
  (void)roll_deg100;
  (void)pitch_deg100;
  (void)yaw_deg100;
  return PX4LITE_OK;
}

/**
 * @brief Control 模块关闭时的急停空实现。
 */
Px4Lite_Result_t Px4Lite_ControlEmergencyStop(uint32_t now_ms)
{
  (void)now_ms;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_ControlModuleInit(void)
{
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_ControlRecover(void)
{
  return PX4LITE_OK;
}

void Px4Lite_ControlRun(uint32_t now_ms)
{
  (void)now_ms;
}

#endif
