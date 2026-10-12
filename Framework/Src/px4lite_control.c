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
#include "px4lite_local_msglog.h"
#include "px4lite_log_message_ids.h"
#include "FreeRTOS.h"
#include "task.h"

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
#define PX4LITE_CONTROL_AUTO_TAKEOFF_TARGET_PERCENT 60U
#define PX4LITE_CONTROL_AUTO_TAKEOFF_RAMP_MS        6000U
#define PX4LITE_CONTROL_AUTO_LANDING_RAMP_MS        8000U
#define PX4LITE_CONTROL_PERCENT_X100_SCALE          100
#define PX4LITE_CONTROL_PERCENT_X100_MAX            10000

typedef enum {
  PX4LITE_CONTROL_AUTO_NONE = 0,
  PX4LITE_CONTROL_AUTO_TAKEOFF,
  PX4LITE_CONTROL_AUTO_LANDING
} Px4Lite_ControlAutoProfile_t;

static uint8_t s_esc_armed;
static uint8_t s_last_run_valid;
static volatile uint8_t s_target_throttle_percent[PX4LITE_MOTOR_COUNT];
static volatile uint16_t s_target_pulse_us[PX4LITE_MOTOR_COUNT];
static volatile uint8_t s_target_is_pulse[PX4LITE_MOTOR_COUNT];
static volatile uint32_t s_target_update_ms[PX4LITE_MOTOR_COUNT];
static volatile uint8_t s_target_valid[PX4LITE_MOTOR_COUNT];
static volatile Px4Lite_ControlAutoProfile_t s_auto_profile;
/* 当前降落斜坡是否由用户按一键降落发起。用户降落是明确的撤收意图，进行中拒绝非零手动油门；
   姿态丢失触发的强制降落是内部保护，必须允许手动接管，否则 IMU 挂掉时操作员被锁在外面。 */
static volatile uint8_t s_auto_landing_user;
static volatile uint8_t s_auto_start_percent[PX4LITE_MOTOR_COUNT];
static volatile uint8_t s_auto_target_percent[PX4LITE_MOTOR_COUNT];
static volatile uint32_t s_auto_start_ms;
static volatile uint32_t s_auto_duration_ms;
static volatile Px4Lite_ControlMode_t s_control_mode;
static volatile int32_t s_target_roll_deg100;
static volatile int32_t s_target_pitch_deg100;
static volatile int32_t s_target_yaw_deg100;
static volatile uint8_t s_estop_latched;
/**< 本机供电闭锁，启动默认关闭输出；只有 Control 根据 Battery2 更新。 */
static volatile uint8_t s_power_inhibited = 1U;
static volatile uint8_t s_power_low_voltage;
static Px4Lite_VehicleNavigation_t s_last_attitude_navigation;
static uint8_t s_last_attitude_valid;
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

/** @brief 日志辅助状态仅由已串行化的控制路径访问。 */
static uint8_t s_auto_logged_start;
static uint8_t s_output_above_minimum;
static uint8_t s_output_write_failed;
static uint8_t s_power_observed;
static uint8_t s_power_cutoff_pending;
static Px4Lite_ControlAutoProfile_t s_auto_complete_pending;
static uint16_t s_last_command_log = PX4LITE_LOGMSG_COUNT;
static uint32_t s_last_command_log_ms;
/** @brief 立即记录所属状态机事实，保护动作不参与状态去抖。 */
static void Px4Lite_ControlLog(uint16_t message, uint32_t now_ms)
{
  Px4Lite_LocalMsgLogEvent(message, now_ms, 0U, PX4LITE_MODULE_CONTROL,
                           (Px4Lite_LogMessageSeverity(message) != 0U) ? 1U : 0U);
}
/** @brief 合并同一请求反馈的连续重复；原因变化立即写入。 */
static void Px4Lite_ControlLogCommand(uint16_t message, uint32_t now_ms)
{
  if ((message == s_last_command_log) && ((uint32_t)(now_ms - s_last_command_log_ms) < 3000U)) { return; }
  s_last_command_log = message;
  s_last_command_log_ms = now_ms;
  Px4Lite_ControlLog(message, now_ms);
}
/** @brief PWM 写入结果是输出事实边界；仅全部通道成功才能记录最小输出。 */
static void Px4Lite_ControlLogOutput(Px4Lite_Result_t result, uint8_t above_minimum, uint32_t now_ms)
{
  if (result != PX4LITE_OK) {
    if (s_output_write_failed == 0U) { Px4Lite_ControlLog(PX4LITE_LOGMSG_OUTPUT_WRITE_FAILED, now_ms); }
    s_output_write_failed = 1U;
    return;
  }
  s_output_write_failed = 0U;
  if (above_minimum == 0U) {
    if (s_power_cutoff_pending != 0U) {
      Px4Lite_ControlLog(PX4LITE_LOGMSG_MOTOR_POWER_CUTOFF, now_ms);
      s_power_cutoff_pending = 0U;
    }
    if (s_output_above_minimum != 0U) { Px4Lite_ControlLog(PX4LITE_LOGMSG_OUTPUT_MINIMUM, now_ms); }
  }
  s_output_above_minimum = above_minimum;
}
/** @brief 撤收输出并记录实际写入结果；不将命令提交冒充硬件成功。 */
static Px4Lite_Result_t Px4Lite_ControlDisarmLogged(uint32_t now_ms)
{
  Px4Lite_Result_t result = Px4Lite_MotorDisarmAll();
  Px4Lite_ControlLogOutput(result, 0U, now_ms);
  return result;
}

static int32_t Px4Lite_ControlClampI32(int32_t value, int32_t min_value, int32_t max_value);

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
 * @brief 将 0.01% 分辨率的油门映射为 ESC 脉宽。
 *
 * @details 姿态闭环在百分比整数以下仍需产生有效差动输出，因此混控内部保留
 * 0.01% 分辨率，最后一步才四舍五入到真实微秒脉宽。
 *
 * @param[in] throttle_percent_x100 油门百分比乘 100，范围 0 到 10000。
 * @return PWM 高电平脉宽，单位：us。
 */
static uint16_t Px4Lite_ControlThrottleX100ToPulseUs(int32_t throttle_percent_x100)
{
  uint32_t range;
  uint32_t pulse_offset;

  throttle_percent_x100 = Px4Lite_ControlClampI32(throttle_percent_x100, 0, PX4LITE_CONTROL_PERCENT_X100_MAX);
  range = (uint32_t)PX4LITE_CONTROL_ESC_MAX_PULSE_US - (uint32_t)PX4LITE_CONTROL_ESC_MIN_PULSE_US;
  pulse_offset = ((range * (uint32_t)throttle_percent_x100) +
                  ((uint32_t)PX4LITE_CONTROL_PERCENT_X100_MAX / 2U)) /
                 (uint32_t)PX4LITE_CONTROL_PERCENT_X100_MAX;
  return (uint16_t)((uint32_t)PX4LITE_CONTROL_ESC_MIN_PULSE_US + pulse_offset);
}

static uint8_t Px4Lite_ControlPulseToThrottlePercent(uint16_t pulse_us)
{
  uint32_t range;
  uint32_t offset;

  if (pulse_us <= PX4LITE_CONTROL_ESC_MIN_PULSE_US) { return 0U; }
  if (pulse_us >= PX4LITE_CONTROL_ESC_MAX_PULSE_US) { return 100U; }
  range = (uint32_t)PX4LITE_CONTROL_ESC_MAX_PULSE_US - (uint32_t)PX4LITE_CONTROL_ESC_MIN_PULSE_US;
  offset = (uint32_t)pulse_us - (uint32_t)PX4LITE_CONTROL_ESC_MIN_PULSE_US;
  return (uint8_t)(((offset * 100U) + (range / 2U)) / range);
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
static int32_t Px4Lite_ControlAttitudeCorrectionX100(int32_t angle_error_deg100, int32_t rate_error_dps100)
{
  int32_t correction;
  int32_t max_correction = (int32_t)PX4LITE_CONTROL_ATTITUDE_MAX_CORRECTION_PERCENT *
                           PX4LITE_CONTROL_PERCENT_X100_SCALE;

  angle_error_deg100 = Px4Lite_ControlClampI32(angle_error_deg100, -36000, 36000);
  rate_error_dps100 = Px4Lite_ControlClampI32(rate_error_dps100, -200000, 200000);
  correction = ((angle_error_deg100 * (int32_t)PX4LITE_CONTROL_ATTITUDE_KP_X100) +
                (rate_error_dps100 * (int32_t)PX4LITE_CONTROL_ATTITUDE_KD_X100)) / 100;
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
 *
 * @details Navigation 同时承载 GNSS 与姿态域，GNSS 单独发布时可能短暂不带姿态有效位。
 * Control 缓存最后一帧真实姿态并只在 IMU 新鲜度窗口内复用，避免把域切换空帧误判为
 * 姿态掉线；缓存超过 `PX4LITE_IMU_MAX_AGE_MS` 后仍会按掉线处理。
 */
static uint8_t Px4Lite_ControlCopyFreshAttitude(Px4Lite_VehicleNavigation_t *navigation, uint32_t now_ms)
{
  Px4Lite_VehicleNavigation_t current;

  if (navigation == 0) { return 0U; }
  if ((Px4Lite_CopyNavigation(&current) == PX4LITE_OK) &&
      (current.header.valid != 0U) &&
      (Px4Lite_IsFresh(&current.header, now_ms, PX4LITE_IMU_MAX_AGE_MS) != 0U) &&
      ((current.valid_mask & PX4LITE_NAV_VALID_ATTITUDE) != 0U)) {
    s_last_attitude_navigation = current;
    s_last_attitude_valid      = 1U;
    *navigation                = current;
    return 1U;
  }
  if ((s_last_attitude_valid != 0U) &&
      (Px4Lite_IsFresh(&s_last_attitude_navigation.header, now_ms, PX4LITE_IMU_MAX_AGE_MS) != 0U)) {
    *navigation = s_last_attitude_navigation;
    return 1U;
  }
  return 0U;
}

/**
 * @brief 判断姿态链路是否允许自动起降。
 *
 * @details 自动起飞/降落拒绝明确离线、失败、未初始化或关闭的 IMU，同时要求 Navigation
 * 姿态快照新鲜。STARTING/DEGRADED 期间只要姿态持续有效仍允许起降，避免状态切换瞬间
 * 吞掉按钮命令；手动滑杆不调用该门控，姿态掉线时仍允许独立油门测试和人工撤收。
 */
static uint8_t Px4Lite_ControlAutoAttitudeReady(uint32_t now_ms)
{
  Px4Lite_ModuleStatus_t attitude_status;
  Px4Lite_VehicleNavigation_t navigation;

  if (Px4Lite_GetModuleStatus(PX4LITE_MODULE_IMU, &attitude_status) != PX4LITE_OK) {
    return 0U;
  }
  if ((attitude_status.state == PX4LITE_STATE_UNINITIALIZED) ||
      (attitude_status.state == PX4LITE_STATE_OFFLINE) ||
      (attitude_status.state == PX4LITE_STATE_FAILED) ||
      (attitude_status.state == PX4LITE_STATE_DISABLED)) {
    return 0U;
  }
  return Px4Lite_ControlCopyFreshAttitude(&navigation, now_ms);
}

/**
 * @brief 在无电机输出时也持续接收最新姿态，供一键命令稳定判定。
 *
 * @details 姿态辅助混控在基础油门为 0 时会提前返回，不能依赖混控路径顺带更新缓存；
 * Control 每周期主动观察一次 Navigation，避免按钮点击恰逢 GNSS 单域帧时误报姿态不可用。
 */
static void Px4Lite_ControlRefreshAttitudeCache(uint32_t now_ms)
{
  Px4Lite_VehicleNavigation_t navigation;

  (void)Px4Lite_ControlCopyFreshAttitude(&navigation, now_ms);
}

/**
 * @brief 在全部安全前提满足时把 roll/pitch/yaw 修正叠加到四路基础油门。
 */
static void Px4Lite_ControlApplyAttitudeAssist(uint32_t now_ms, int32_t duty_percent_x100[PX4LITE_MOTOR_COUNT])
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
    if (duty_percent_x100[i] >= ((int32_t)PX4LITE_CONTROL_ATTITUDE_MIN_BASE_PERCENT *
                                 PX4LITE_CONTROL_PERCENT_X100_SCALE)) {
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
  roll_correction = Px4Lite_ControlAttitudeCorrectionX100(s_target_roll_deg100 - measured_roll_deg100,
                                                          -measured_roll_rate_dps100);
  pitch_correction = Px4Lite_ControlAttitudeCorrectionX100(s_target_pitch_deg100 - measured_pitch_deg100,
                                                           -measured_pitch_rate_dps100);
  yaw_correction = Px4Lite_ControlAttitudeCorrectionX100(Px4Lite_ControlWrapYawErrorDeg100(s_target_yaw_deg100,
                                                                                           measured_yaw_deg100),
                                                         -measured_yaw_rate_dps100);

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    /* 按旧工程口径只修正已有基础油门的通道，避免单路台架测试时启动零油门电机。 */
    if (duty_percent_x100[i] < ((int32_t)PX4LITE_CONTROL_ATTITUDE_MIN_BASE_PERCENT *
                                PX4LITE_CONTROL_PERCENT_X100_SCALE)) { continue; }
    mixed = duty_percent_x100[i] +
            ((int32_t)s_attitude_roll_mix[i] * roll_correction) +
            ((int32_t)s_attitude_pitch_mix[i] * pitch_correction) +
            ((int32_t)s_attitude_yaw_mix[i] * yaw_correction);
    duty_percent_x100[i] = Px4Lite_ControlClampI32(mixed, 0, PX4LITE_CONTROL_PERCENT_X100_MAX);
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
  uint8_t above = 0U;
  Px4Lite_Result_t result = PX4LITE_OK;

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (pulse_us[i] > PX4LITE_CONTROL_ESC_MIN_PULSE_US) { above = 1U; }
    if (Px4Lite_MotorWritePulseUs(i, pulse_us[i]) != PX4LITE_OK) { result = PX4LITE_IO_ERROR; }
  }

  Px4Lite_ControlLogOutput(result, above, Px4Lite_PlatformGetMs());
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
    s_target_pulse_us[i]          = PX4LITE_CONTROL_ESC_MIN_PULSE_US;
    s_target_is_pulse[i]          = 0U;
    s_target_update_ms[i]        = 0U;
    s_target_valid[i]            = 0U;
  }
}

/**
 * @brief 清除自动斜坡参数；本函数不写 PWM，不能单独证明输出已撤收。
 */
static void Px4Lite_ControlClearAutoThrottle(void)
{
  uint8_t i;

  s_auto_profile      = PX4LITE_CONTROL_AUTO_NONE;
  s_auto_logged_start = 0U;
  s_auto_landing_user = 0U;
  s_auto_start_ms    = 0U;
  s_auto_duration_ms = 0U;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    s_auto_start_percent[i]  = 0U;
    s_auto_target_percent[i] = 0U;
  }
}

/** @brief 因手动接管、保护、急停或恢复中止自动过程，保留中止事实。 */
static void Px4Lite_ControlAbortAuto(uint32_t now_ms)
{
  if (s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) {
    Px4Lite_ControlLog((s_auto_profile == PX4LITE_CONTROL_AUTO_TAKEOFF) ?
      PX4LITE_LOGMSG_RAMP_UP_ABORTED : PX4LITE_LOGMSG_RAMP_DOWN_ABORTED, now_ms);
  }
  s_auto_complete_pending = PX4LITE_CONTROL_AUTO_NONE;
  Px4Lite_ControlClearAutoThrottle();
}
/** @brief 明确非零新请求解除急停后，输出重新经历预解锁。 */
static void Px4Lite_ControlReleaseEmergencyStop(uint32_t now_ms)
{
  if (s_estop_latched == 0U) { return; }
  s_estop_latched = 0U;
  Px4Lite_ControlLog(PX4LITE_LOGMSG_ESTOP_RELEASED, now_ms);
  s_esc_armed = 0U;
  s_arm_start_ms = now_ms;
  s_last_run_ms = now_ms;
  s_last_run_valid = 1U;
}

static uint8_t Px4Lite_ControlCurrentTargetPercent(uint8_t motor_index)
{
  uint8_t value;

  if (motor_index >= PX4LITE_MOTOR_COUNT) { return 0U; }
  value = (s_target_valid[motor_index] != 0U) ? s_target_throttle_percent[motor_index] : 0U;
  return (value > 100U) ? 100U : value;
}

/** @brief 建立同步升速/缓降目标；接受不代表 ESC 已解锁或开始输出。 */
static Px4Lite_Result_t Px4Lite_ControlStartAutoThrottle(Px4Lite_ControlAutoProfile_t profile)
{
  uint8_t i;
  uint8_t target;
  uint32_t duration;

  if ((profile != PX4LITE_CONTROL_AUTO_TAKEOFF) && (profile != PX4LITE_CONTROL_AUTO_LANDING)) {
    return PX4LITE_INVALID_PARAM;
  }
  if ((s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING) && (profile == PX4LITE_CONTROL_AUTO_TAKEOFF)) {
    return PX4LITE_BUSY;
  }
  if (s_auto_profile == profile) { return PX4LITE_BUSY; }
  if (s_estop_latched != 0U) {
    if (profile == PX4LITE_CONTROL_AUTO_TAKEOFF) {
      Px4Lite_ControlReleaseEmergencyStop(Px4Lite_PlatformGetMs());
    } else {
      return PX4LITE_OK;
    }
  }

  if (s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) { Px4Lite_ControlAbortAuto(Px4Lite_PlatformGetMs()); }
  s_auto_logged_start = 0U;
  target   = (profile == PX4LITE_CONTROL_AUTO_TAKEOFF) ? PX4LITE_CONTROL_AUTO_TAKEOFF_TARGET_PERCENT : 0U;
  duration = (profile == PX4LITE_CONTROL_AUTO_TAKEOFF) ? PX4LITE_CONTROL_AUTO_TAKEOFF_RAMP_MS : PX4LITE_CONTROL_AUTO_LANDING_RAMP_MS;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    s_auto_start_percent[i]  = Px4Lite_ControlCurrentTargetPercent(i);
    s_auto_target_percent[i] = target;
  }
  s_auto_start_ms    = Px4Lite_PlatformGetMs();
  s_auto_duration_ms = duration;
  s_auto_profile     = profile;
  return PX4LITE_OK;
}

/**
 * @brief 在自动起飞姿态丢失时强制切换到降落斜坡。
 *
 * @details 这是自动流程内部保护，不走公开一键降落门控；否则姿态已经丢失时会因
 * “禁止一键降落”而无法撤收自动油门。手动滑杆路径不受该保护触发。
 */
static void Px4Lite_ControlForceAutoLanding(uint32_t now_ms)
{
  uint8_t i;

  Px4Lite_ControlLog(PX4LITE_LOGMSG_RAMP_SENSOR_ABORT, now_ms);
  s_auto_logged_start = 0U;

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    s_auto_start_percent[i]  = Px4Lite_ControlCurrentTargetPercent(i);
    s_auto_target_percent[i] = 0U;
  }
  s_auto_start_ms    = now_ms;
  s_auto_duration_ms = PX4LITE_CONTROL_AUTO_LANDING_RAMP_MS;
  s_auto_profile     = PX4LITE_CONTROL_AUTO_LANDING;
  /* 内部保护发起的降落：手动油门仍可随时接管。 */
  s_auto_landing_user = 0U;
}

static uint8_t Px4Lite_ControlAutoTakeoffSensorsReady(uint32_t now_ms)
{
  Px4Lite_ModuleStatus_t environment_status;

  if ((Px4Lite_ControlAutoAttitudeReady(now_ms) == 0U) ||
      (Px4Lite_GetModuleStatus(PX4LITE_MODULE_BARO, &environment_status) != PX4LITE_OK) ||
      (environment_status.state != PX4LITE_STATE_ONLINE)) {
    return 0U;
  }
  return 1U;
}

/**
 * @brief 自动起飞期间监控姿态有效性，姿态丢失立即转入自动降落。
 */
static void Px4Lite_ControlMonitorAutoTakeoff(uint32_t now_ms)
{
  if (s_auto_profile != PX4LITE_CONTROL_AUTO_TAKEOFF) { return; }
  if (Px4Lite_ControlAutoAttitudeReady(now_ms) != 0U) { return; }
  Px4Lite_ControlForceAutoLanding(now_ms);
}

/** @brief 计算本周期斜坡目标；结束事件延迟到四路 PWM 成功写入后确认。 */
static void Px4Lite_ControlApplyAutoThrottle(uint32_t now_ms)
{
  uint8_t i;
  uint32_t elapsed_ms;
  uint32_t duration_ms;

  if (s_auto_profile == PX4LITE_CONTROL_AUTO_NONE) { return; }
  if ((s_estop_latched != 0U) || (s_power_inhibited != 0U)) {
    Px4Lite_ControlAbortAuto(now_ms);
    return;
  }
  elapsed_ms = Px4Lite_ElapsedMs(now_ms, s_auto_start_ms);
  duration_ms = (s_auto_duration_ms == 0U) ? 1U : s_auto_duration_ms;
  if (elapsed_ms > duration_ms) { elapsed_ms = duration_ms; }

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    int32_t start = (int32_t)s_auto_start_percent[i];
    int32_t target = (int32_t)s_auto_target_percent[i];
    int32_t delta = target - start;
    int32_t value = start + ((delta * (int32_t)elapsed_ms) / (int32_t)duration_ms);

    s_target_throttle_percent[i] = Px4Lite_ControlClampDuty(value);
    s_target_is_pulse[i]          = 0U;
    s_target_update_ms[i]        = now_ms;
    s_target_valid[i]            = 1U;
  }

  if (elapsed_ms >= duration_ms) {
    s_auto_complete_pending = s_auto_profile;
    Px4Lite_ControlClearAutoThrottle();
  }
}

/**
 * @brief 复位 Control 内部状态并把电机输出降到最小脉宽。
 *
 * @details 恢复时清零目标并重新闭锁，下一次本机有效采样才允许解除。
 */
static void Px4Lite_ControlReset(uint32_t now_ms)
{
  s_power_inhibited = 1U;
  s_power_low_voltage = 0U;
  s_esc_armed      = 0U;
  s_last_run_valid = 0U;
  s_estop_latched  = 0U;
  s_arm_start_ms   = now_ms;
  s_last_run_ms    = now_ms;
  s_control_mode    = (PX4LITE_CONTROL_ATTITUDE_ASSIST_DEFAULT != 0U) ? PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST : PX4LITE_CONTROL_MODE_DIRECT;
  s_target_roll_deg100  = PX4LITE_CONTROL_ATTITUDE_TARGET_ROLL_DEG100;
  s_target_pitch_deg100 = PX4LITE_CONTROL_ATTITUDE_TARGET_PITCH_DEG100;
  s_target_yaw_deg100   = PX4LITE_CONTROL_ATTITUDE_TARGET_YAW_DEG100;
  memset(&s_last_attitude_navigation, 0, sizeof(s_last_attitude_navigation));
  s_last_attitude_valid = 0U;
  Px4Lite_ControlClearTargets();
  Px4Lite_ControlAbortAuto(now_ms);
  memset(s_button_stable, 0, sizeof(s_button_stable));
  memset(s_button_last_raw, 0, sizeof(s_button_last_raw));
  memset(s_button_count, 0, sizeof(s_button_count));
  (void)Px4Lite_ControlDisarmLogged(now_ms);
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
 * @brief 发布当前电机百分比和最终写入 PWM 的真实脉宽。
 *
 * @param[in] duty_percent 已叠加姿态修正的实际输出百分比。
 * @param[in] base_percent 修正前的基础油门，供滑块跟随，避免修正量被回灌成新目标。
 * @param[in] pulse_us     最终写入 PWM 的高电平脉宽。
 * @param[in] run_state    1 表示 ESC 已解锁且允许输出。
 */
static void Px4Lite_ControlPublish(uint32_t now_ms,
                                   const uint8_t duty_percent[PX4LITE_MOTOR_COUNT],
                                   const uint8_t base_percent[PX4LITE_MOTOR_COUNT],
                                   const uint16_t pulse_us[PX4LITE_MOTOR_COUNT],
                                   uint8_t run_state)
{
  Px4Lite_MotorOutputs_t outputs;
  uint8_t i;
  uint8_t max_throttle = 0U;

  memset(&outputs, 0, sizeof(outputs));
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    outputs.duty_percent[i] = duty_percent[i];
    outputs.base_percent[i] = base_percent[i];
    outputs.pulse_us[i]     = pulse_us[i];
    if (duty_percent[i] > max_throttle) { max_throttle = duty_percent[i]; }
  }
  outputs.run_state                = run_state;
  outputs.speed_level              = max_throttle;
  outputs.control_mode             = (uint8_t)s_control_mode;
  if (s_power_inhibited != 0U) {
    outputs.operation_state = (uint8_t)PX4LITE_MOTOR_OP_INHIBITED;
  } else if (s_esc_armed == 0U) {
    outputs.operation_state = (uint8_t)PX4LITE_MOTOR_OP_ESC_PREPARING;
  } else if (s_auto_profile == PX4LITE_CONTROL_AUTO_TAKEOFF) {
    outputs.operation_state = (uint8_t)PX4LITE_MOTOR_OP_SYNC_RAMP_UP;
  } else if (s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING) {
    outputs.operation_state = (uint8_t)PX4LITE_MOTOR_OP_RAMP_DOWN;
  } else if ((run_state != 0U) && (max_throttle != 0U)) {
    outputs.operation_state = (uint8_t)PX4LITE_MOTOR_OP_MANUAL_RUNNING;
  } else {
    outputs.operation_state = (uint8_t)PX4LITE_MOTOR_OP_STOPPED;
  }
  outputs.header.sequence          = ++s_sequence;
  outputs.header.sample_time_ms    = now_ms;
  outputs.header.publish_time_ms   = now_ms;
  outputs.header.device_id         = (uint16_t)PX4LITE_MODULE_CONTROL;
  outputs.header.valid             = 1U;
  outputs.header.quality           = 100U;
  outputs.header.flags             = PX4LITE_DATA_VALID;
  Px4Lite_PublishMotor(&outputs);
}

static void Px4Lite_ControlCopyTargetPercent(uint8_t duty_percent[PX4LITE_MOTOR_COUNT])
{
  uint8_t i;

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    duty_percent[i] = ((s_target_valid[i] != 0U) && (s_power_inhibited == 0U)) ? s_target_throttle_percent[i] : 0U;
    if (duty_percent[i] > 100U) { duty_percent[i] = 100U; }
  }
}

/**
 * @brief 在控制周期或命令提交前，依据本机 Battery2 更新供电门禁。
 * @param[in] now_ms 当前真实毫秒时间。
 * @note 调用者已挂起任务调度；快照内部短临界区，不依赖显示或远端数据。
 *       闭锁同时清目标、自动过程及预解锁进度，恢复不会重放旧命令。
 */
static void Px4Lite_ControlRefreshPowerGate(uint32_t now_ms)
{
  Px4Lite_BatteryStatus_t battery;
  uint8_t inhibit = 1U;
  s_power_low_voltage = 0U;
  if ((Px4Lite_CopyBattery2(&battery) == PX4LITE_OK) &&
      (battery.header.valid != 0U) &&
      (Px4Lite_ElapsedMs(now_ms, battery.header.sample_time_ms) <= PX4LITE_BATTERY_MAX_AGE_MS)) {
    s_power_low_voltage = (battery.voltage_mv < PX4LITE_CONTROL_POWER_CUTOFF_MV) ? 1U : 0U;
    if ((battery.voltage_mv <= PX4LITE_CONTROL_POWER_MAX_MV) &&
        (battery.voltage_mv >= ((s_power_inhibited != 0U) ? PX4LITE_CONTROL_POWER_RECOVER_MV : PX4LITE_CONTROL_POWER_CUTOFF_MV))) {
      inhibit = 0U;
    }
  }
  if ((s_power_observed == 0U) || (inhibit != s_power_inhibited)) {
    Px4Lite_ControlLog((inhibit != 0U) ? PX4LITE_LOGMSG_POWER_INHIBITED : PX4LITE_LOGMSG_POWER_RELEASED, now_ms);
    s_power_observed = 1U;
    if ((inhibit != 0U) && (s_output_above_minimum != 0U)) { s_power_cutoff_pending = 1U; }
  }
  if (inhibit != 0U) {
    Px4Lite_ControlClearTargets();
    Px4Lite_ControlAbortAuto(now_ms);
    s_esc_armed = 0U;
    s_arm_start_ms = now_ms;
  }
  s_power_inhibited = inhibit;
}

/** @brief 只读查询：最近一次本机有效新鲜采样是否低于停机门限。 */
uint8_t Px4Lite_ControlIsPowerLowVoltage(void)
{
  return s_power_low_voltage;
}

/** @brief 查询本机供电闭锁，不允许上层写入解除。 */
uint8_t Px4Lite_ControlIsPowerInhibited(void)
{
  return s_power_inhibited;
}

/**
 * @brief 提交完整四路目标；暂停任务调度期间检查本机供电并整体更新状态。
 * @details 百分比保持当前模式，脉宽命令同时选择 DIRECT。用户缓降中的全零命令
 *          只确认已有撤收意图，不改变曲线或模式；被拒命令不解除急停。
 */
static Px4Lite_Result_t Px4Lite_ControlCommitMotorTargets(const uint8_t percent[PX4LITE_MOTOR_COUNT],
                                                         const uint16_t pulse[PX4LITE_MOTOR_COUNT],
                                                         uint8_t is_pulse, uint8_t nonzero)
{
  uint8_t i;
  uint32_t now_ms;

  vTaskSuspendAll();
  now_ms = Px4Lite_PlatformGetMs();
  Px4Lite_ControlRefreshPowerGate(now_ms);
  if ((s_power_inhibited != 0U) && (nonzero != 0U)) {
    Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_POWER_FAIL, now_ms);
    (void)xTaskResumeAll();
    return PX4LITE_NOT_READY;
  }
  if ((s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING) && (s_auto_landing_user != 0U)) {
    if (nonzero != 0U) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_LANDING_THROTTLE_REJECT, now_ms); }
    (void)xTaskResumeAll();
    return (nonzero != 0U) ? PX4LITE_NOT_READY : PX4LITE_OK;
  }
  if (nonzero != 0U) { Px4Lite_ControlReleaseEmergencyStop(now_ms); }
  if (s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) { Px4Lite_ControlAbortAuto(Px4Lite_PlatformGetMs()); }
  if (is_pulse != 0U) { s_control_mode = PX4LITE_CONTROL_MODE_DIRECT; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    s_target_throttle_percent[i] = percent[i];
    s_target_pulse_us[i] = pulse[i];
    s_target_is_pulse[i] = is_pulse;
    s_target_update_ms[i] = now_ms;
    s_target_valid[i] = 1U;
  }
  Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_ACCEPTED, now_ms);
  (void)xTaskResumeAll();
  return PX4LITE_OK;
}

/** @brief 完整校验四路百分比，保留模式后整体提交；单位 %，范围 0~100。 */
Px4Lite_Result_t Px4Lite_ControlSetMotorThrottles(const uint8_t percent[PX4LITE_MOTOR_COUNT])
{
  uint8_t i;
  uint8_t nonzero = 0U;
  uint16_t pulse[PX4LITE_MOTOR_COUNT];

  if (percent == 0) { return PX4LITE_INVALID_PARAM; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (percent[i] > 100U) { return PX4LITE_INVALID_PARAM; }
    pulse[i] = Px4Lite_ControlThrottleToPulseUs(percent[i]);
    if (percent[i] != 0U) { nonzero = 1U; }
  }
  return Px4Lite_ControlCommitMotorTargets(percent, pulse, 0U, nonzero);
}

/** @brief 完整校验四路脉宽，与 DIRECT 模式整体提交；单位 us。 */
Px4Lite_Result_t Px4Lite_ControlSetMotorPulses(const uint16_t pulse[PX4LITE_MOTOR_COUNT])
{
  uint8_t i;
  uint8_t nonzero = 0U;
  uint8_t percent[PX4LITE_MOTOR_COUNT];

  if (pulse == 0) { return PX4LITE_INVALID_PARAM; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if ((pulse[i] < PX4LITE_CONTROL_ESC_MIN_PULSE_US) ||
        (pulse[i] > PX4LITE_CONTROL_ESC_MAX_PULSE_US)) { return PX4LITE_INVALID_PARAM; }
    percent[i] = Px4Lite_ControlPulseToThrottlePercent(pulse[i]);
    if (pulse[i] != PX4LITE_CONTROL_ESC_MIN_PULSE_US) { nonzero = 1U; }
  }
  return Px4Lite_ControlCommitMotorTargets(percent, pulse, 1U, nonzero);
}

/** @brief 单路百分比目标准入；保护、缓降拒绝和人工接管均在此处成为事实。 */
static Px4Lite_Result_t Px4Lite_ControlSetMotorThrottlePercentImpl(uint8_t motor_index, uint8_t throttle_percent)
{
  if (motor_index >= PX4LITE_MOTOR_COUNT) { return PX4LITE_INVALID_PARAM; }
  if (throttle_percent > 100U) { throttle_percent = 100U; }
  /* 动力电池不满足运行条件时只接受 0：拒绝要发生在解除急停和撤销自动油门之前，
     否则一次被拒的滑条命令会顺带把急停解锁、把降落斜坡撤掉。 */
  if ((s_power_inhibited != 0U) && (throttle_percent != 0U)) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_POWER_FAIL, Px4Lite_PlatformGetMs()); return PX4LITE_NOT_READY; }
  /* 一键降落是用户明确的撤收动作，进行中不接受新的非零手动油门；0 与降落终点一致，接受但
     不打断斜坡。姿态丢失触发的强制降落不在此列——那是内部保护，必须留出手动接管的出口。
     一键起飞也不做此限制，手动接管随时可以打断上升。 */
  if ((s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING) && (s_auto_landing_user != 0U)) {
    if (throttle_percent != 0U) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_LANDING_THROTTLE_REJECT, Px4Lite_PlatformGetMs()); return PX4LITE_NOT_READY; }
    return PX4LITE_OK;
  }
  if ((s_estop_latched != 0U) && (throttle_percent != 0U)) {
    Px4Lite_ControlReleaseEmergencyStop(Px4Lite_PlatformGetMs());
  }
  if (s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) { Px4Lite_ControlAbortAuto(Px4Lite_PlatformGetMs()); }

  s_target_throttle_percent[motor_index] = throttle_percent;
  s_target_pulse_us[motor_index]          = Px4Lite_ControlThrottleToPulseUs(throttle_percent);
  s_target_is_pulse[motor_index]          = 0U;
  s_target_update_ms[motor_index]        = Px4Lite_PlatformGetMs();
  s_target_valid[motor_index]            = 1U;
  return PX4LITE_OK;
}

/** @brief 单路脉宽准入；最小输出语义独立于动力电压的状态观察。 */
static Px4Lite_Result_t Px4Lite_ControlSetMotorPulseUsImpl(uint8_t motor_index, uint16_t pulse_us)
{
  if (motor_index >= PX4LITE_MOTOR_COUNT) { return PX4LITE_INVALID_PARAM; }
  if ((pulse_us < PX4LITE_CONTROL_ESC_MIN_PULSE_US) ||
      (pulse_us > PX4LITE_CONTROL_ESC_MAX_PULSE_US)) { return PX4LITE_INVALID_PARAM; }
  /* 与百分比入口同口径：闭锁和一键降落期间只接受最小脉宽。 */
  if ((s_power_inhibited != 0U) && (pulse_us != PX4LITE_CONTROL_ESC_MIN_PULSE_US)) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_POWER_FAIL, Px4Lite_PlatformGetMs()); return PX4LITE_NOT_READY; }
  if ((s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING) && (s_auto_landing_user != 0U)) {
    if (pulse_us != PX4LITE_CONTROL_ESC_MIN_PULSE_US) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_LANDING_THROTTLE_REJECT, Px4Lite_PlatformGetMs()); return PX4LITE_NOT_READY; }
    return PX4LITE_OK;
  }
  if ((s_estop_latched != 0U) && (pulse_us != PX4LITE_CONTROL_ESC_MIN_PULSE_US)) {
    Px4Lite_ControlReleaseEmergencyStop(Px4Lite_PlatformGetMs());
  }
  if (s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) { Px4Lite_ControlAbortAuto(Px4Lite_PlatformGetMs()); }

  s_target_pulse_us[motor_index]          = pulse_us;
  s_target_throttle_percent[motor_index] = Px4Lite_ControlPulseToThrottlePercent(pulse_us);
  s_target_is_pulse[motor_index]          = 1U;
  s_target_update_ms[motor_index]         = Px4Lite_PlatformGetMs();
  s_target_valid[motor_index]             = 1U;
  return PX4LITE_OK;
}

/** @brief 在状态机内给出精确拒绝原因；重复过程不重置已有斜坡计时。 */
static Px4Lite_Result_t Px4Lite_ControlStartAutoTakeoffImpl(void)
{
  if (s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) {
    Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_RAMP_UP_BUSY, Px4Lite_PlatformGetMs());
    return PX4LITE_BUSY;
  }
  /* 起飞是加油门动作，动力电池闭锁时一律拒绝；降落不设此门槛，撤收永远允许。 */
  if (s_power_inhibited != 0U) {
    Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_TAKEOFF_POWER_FAIL, Px4Lite_PlatformGetMs());
    return PX4LITE_NOT_READY;
  }
  if (Px4Lite_ControlAutoTakeoffSensorsReady(Px4Lite_PlatformGetMs()) == 0U) {
    Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_TAKEOFF_SENSOR_FAIL, Px4Lite_PlatformGetMs());
    return PX4LITE_NOT_READY;
  }

  /* 教学操作只改变四路同步目标，不替学生切换控制模式。需要姿态辅助时，
     学生必须先在电机页明确选择，便于观察直控与闭环修正的差异。 */
  return Px4Lite_ControlStartAutoThrottle(PX4LITE_CONTROL_AUTO_TAKEOFF);
}

/**
 * @brief 启动一键降落斜坡。
 *
 * @details 降落是撤收动作，不设姿态或环境准入：姿态掉线恰恰是最需要把油门收回 0 的时刻，
 * 用传感器状态阻止降落会让飞机在异常时卡在当前油门。自动起飞期间姿态丢失由
 * `Px4Lite_ControlMonitorAutoTakeoff()` 自动转入本斜坡；手动油门下用户也可随时按降落。
 * 当前控制模式在缓降过程中保持不变；姿态辅助只有在学生已经明确选择时才生效。
 */
static Px4Lite_Result_t Px4Lite_ControlStartAutoLandingImpl(void)
{
  Px4Lite_Result_t result;

  if ((s_power_inhibited != 0U) || (s_estop_latched != 0U)) {
    Px4Lite_ControlAbortAuto(Px4Lite_PlatformGetMs());
    Px4Lite_ControlClearTargets();
    return Px4Lite_ControlDisarmLogged(Px4Lite_PlatformGetMs());
  }
  result = Px4Lite_ControlStartAutoThrottle(PX4LITE_CONTROL_AUTO_LANDING);
  if (result == PX4LITE_OK) {
    /* 用户明确要求撤收：斜坡跑完之前不再接受非零手动油门。 */
    s_auto_landing_user = 1U;
  }
  return result;
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
static Px4Lite_Result_t Px4Lite_ControlEmergencyStopImpl(uint32_t now_ms)
{
  Px4Lite_Result_t result;

  Px4Lite_ControlLog(PX4LITE_LOGMSG_ESTOP, now_ms);
  s_estop_latched  = 1U;
  s_esc_armed      = 0U;
  s_arm_start_ms   = now_ms;
  s_last_run_ms    = now_ms;
  s_last_run_valid = 1U;
  Px4Lite_ControlClearTargets();
  Px4Lite_ControlAbortAuto(now_ms);

  result = Px4Lite_ControlDisarmLogged(now_ms);
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

static Px4Lite_Result_t Px4Lite_ControlRecoverImpl(void)
{
  uint32_t now_ms = Px4Lite_PlatformGetMs();

  Px4Lite_ControlReset(now_ms);
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL, PX4LITE_STATE_STARTING, PX4LITE_FAULT_NONE, now_ms);
  return PX4LITE_OK;
}

/** @brief 无阻塞地完成一次目标消费和输出；由外层保持任务之间的状态一致性。 */
static void Px4Lite_ControlRunCycle(uint32_t now_ms)
{
  uint16_t pulse_us[PX4LITE_MOTOR_COUNT];
  uint8_t duty_percent[PX4LITE_MOTOR_COUNT];
  uint8_t base_percent[PX4LITE_MOTOR_COUNT];
  int32_t duty_percent_x100[PX4LITE_MOTOR_COUNT];
  uint8_t i;
  Px4Lite_Result_t result;

  if ((s_last_run_valid != 0U) && (Px4Lite_ElapsedMs(now_ms, s_last_run_ms) > PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS)) {
    Px4Lite_ControlLog(PX4LITE_LOGMSG_CONTROL_TIMEOUT, now_ms);
    s_esc_armed = 0U;
    s_arm_start_ms = now_ms;
    Px4Lite_ControlClearTargets();
    Px4Lite_ControlAbortAuto(now_ms);
    (void)Px4Lite_ControlDisarmLogged(now_ms);
  }
  s_last_run_valid = 1U;
  s_last_run_ms    = now_ms;
  Px4Lite_ControlRefreshAttitudeCache(now_ms);

  if (Px4Lite_ControlPressEdge(PX4LITE_BUTTON_KEY1) != 0U) {
    (void)Px4Lite_ControlEmergencyStopImpl(now_ms);
  }

  if (s_esc_armed == 0U) {
    for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
      duty_percent[i] = 0U;
      pulse_us[i]     = PX4LITE_CONTROL_ESC_MIN_PULSE_US;
    }
    result = Px4Lite_ControlWriteAll(pulse_us);
    if (Px4Lite_ElapsedMs(now_ms, s_arm_start_ms) >= PX4LITE_CONTROL_ESC_ARM_TIME_MS) { s_esc_armed = 1U; }
    if (s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) { s_auto_start_ms = now_ms; }
    /* 预解锁期间硬件停在 1000us，因此实际输出为 0；学生设定单独保存在
       base_percent，页面可同时解释“目标已接受”和“ESC 尚未输出”。 */
    Px4Lite_ControlCopyTargetPercent(base_percent);
    Px4Lite_ControlPublish(now_ms, duty_percent, base_percent, pulse_us, 0U);
    Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL, (result == PX4LITE_OK) ? PX4LITE_STATE_STARTING : PX4LITE_STATE_DEGRADED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SYSTEM_SELF_CHECK, now_ms);
    return;
  }

  Px4Lite_ControlMonitorAutoTakeoff(now_ms);
  Px4Lite_ControlApplyAutoThrottle(now_ms);

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if ((s_estop_latched != 0U) || (s_power_inhibited != 0U) || (s_target_valid[i] == 0U)) {
      s_target_throttle_percent[i] = 0U;
      s_target_pulse_us[i]          = PX4LITE_CONTROL_ESC_MIN_PULSE_US;
      s_target_is_pulse[i]          = 0U;
      s_target_update_ms[i]        = 0U;
      s_target_valid[i]            = 0U;
      duty_percent[i]              = 0U;
    } else {
      duty_percent[i] = s_target_throttle_percent[i];
      if (duty_percent[i] > 100U) { duty_percent[i] = 100U; }
    }
  }

  /* 锁住修正前的基础油门再做混控：滑块跟随 base_percent，姿态修正只体现在 duty/pulse 上，
     否则修正量会随滑块回推变成新的目标。 */
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    base_percent[i]      = duty_percent[i];
    duty_percent_x100[i] = (int32_t)duty_percent[i] * PX4LITE_CONTROL_PERCENT_X100_SCALE;
  }
  if ((s_estop_latched == 0U) && (s_power_inhibited == 0U)) { Px4Lite_ControlApplyAttitudeAssist(now_ms, duty_percent_x100); }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if ((s_target_is_pulse[i] != 0U) &&
        (s_control_mode == PX4LITE_CONTROL_MODE_DIRECT) &&
        (s_estop_latched == 0U)) {
      pulse_us[i] = s_target_pulse_us[i];
    } else {
      pulse_us[i] = Px4Lite_ControlThrottleX100ToPulseUs(duty_percent_x100[i]);
      duty_percent[i] = Px4Lite_ControlClampDuty((duty_percent_x100[i] + 50) /
                                                 PX4LITE_CONTROL_PERCENT_X100_SCALE);
    }
  }

  result = Px4Lite_ControlWriteAll(pulse_us);
  if (result == PX4LITE_OK) {
    if ((s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) && (s_auto_logged_start == 0U)) {
      Px4Lite_ControlLog((s_auto_profile == PX4LITE_CONTROL_AUTO_TAKEOFF) ? PX4LITE_LOGMSG_RAMP_UP_STARTED :
        ((s_auto_landing_user != 0U) ? PX4LITE_LOGMSG_RAMP_DOWN_STARTED : PX4LITE_LOGMSG_PROTECT_DOWN_STARTED), now_ms);
      s_auto_logged_start = 1U;
    }
    if (s_auto_complete_pending != PX4LITE_CONTROL_AUTO_NONE) {
      Px4Lite_ControlLog((s_auto_complete_pending == PX4LITE_CONTROL_AUTO_TAKEOFF) ?
        PX4LITE_LOGMSG_RAMP_UP_COMPLETED : PX4LITE_LOGMSG_RAMP_DOWN_COMPLETED, now_ms);
      s_auto_complete_pending = PX4LITE_CONTROL_AUTO_NONE;
    }
  } else if ((s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) || (s_auto_complete_pending != PX4LITE_CONTROL_AUTO_NONE)) {
    if (s_auto_profile == PX4LITE_CONTROL_AUTO_NONE) { s_auto_profile = s_auto_complete_pending; }
    Px4Lite_ControlAbortAuto(now_ms);
    Px4Lite_ControlClearTargets();
  }
  Px4Lite_ControlPublish(now_ms, duty_percent, base_percent, pulse_us, ((s_estop_latched == 0U) && (s_power_inhibited == 0U)) ? 1U : 0U);
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL, ((result == PX4LITE_OK) && (s_estop_latched == 0U)) ? PX4LITE_STATE_ONLINE : PX4LITE_STATE_DEGRADED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SYSTEM_SELF_CHECK, now_ms);
}

/**
 * @brief 控制周期与任务中的命令提交串行，避免计算中途被另一任务切换目标/模式。
 * @details 保持中断可用；本周期只允许快照、计算和同步 PWM 寄存器写入，禁止阻塞
 *          I/O 或等待调度器。健康状态和 topic 发布均为短临界区操作。M4 测量周期耗时。
 * @param[in] now_ms 调用者调度时刻，单位 ms；进入调度互斥后重新取时，避免锁前抢占使用旧年龄。
 */
void Px4Lite_ControlRun(uint32_t now_ms)
{
  vTaskSuspendAll();
  now_ms = Px4Lite_PlatformGetMs();
  Px4Lite_ControlRefreshPowerGate(now_ms);
  Px4Lite_ControlRunCycle(now_ms);
  (void)xTaskResumeAll();
}

/** @brief 与控制周期串行提交，确保门禁和目标变更不可被其他任务交错。 */
Px4Lite_Result_t Px4Lite_ControlSetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent)
{
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  Px4Lite_ControlRefreshPowerGate(Px4Lite_PlatformGetMs());
  result = Px4Lite_ControlSetMotorThrottlePercentImpl(motor_index, throttle_percent);
  if (result == PX4LITE_OK) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_ACCEPTED, Px4Lite_PlatformGetMs()); }
  else if (result == PX4LITE_INVALID_PARAM) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_INVALID, Px4Lite_PlatformGetMs()); }
  (void)xTaskResumeAll();
  return result;
}

/** @brief 与控制周期串行提交，确保门禁和目标变更不可被其他任务交错。 */
Px4Lite_Result_t Px4Lite_ControlSetMotorPulseUs(uint8_t motor_index, uint16_t pulse_us)
{
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  Px4Lite_ControlRefreshPowerGate(Px4Lite_PlatformGetMs());
  result = Px4Lite_ControlSetMotorPulseUsImpl(motor_index, pulse_us);
  if (result == PX4LITE_OK) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_ACCEPTED, Px4Lite_PlatformGetMs()); }
  else if (result == PX4LITE_INVALID_PARAM) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_INVALID, Px4Lite_PlatformGetMs()); }
  (void)xTaskResumeAll();
  return result;
}

/** @brief 与控制周期串行提交，确保门禁和目标变更不可被其他任务交错。 */
Px4Lite_Result_t Px4Lite_ControlStartAutoTakeoff(void)
{
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  Px4Lite_ControlRefreshPowerGate(Px4Lite_PlatformGetMs());
  result = Px4Lite_ControlStartAutoTakeoffImpl();
  if (result == PX4LITE_OK) { Px4Lite_ControlLog(PX4LITE_LOGMSG_RAMP_UP_ACCEPTED, Px4Lite_PlatformGetMs()); s_last_command_log = PX4LITE_LOGMSG_COUNT; }
  (void)xTaskResumeAll();
  return result;
}

/** @brief 与控制周期串行提交，确保门禁和目标变更不可被其他任务交错。 */
Px4Lite_Result_t Px4Lite_ControlStartAutoLanding(void)
{
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  Px4Lite_ControlRefreshPowerGate(Px4Lite_PlatformGetMs());
  result = Px4Lite_ControlStartAutoLandingImpl();
  if (result == PX4LITE_OK) {
    Px4Lite_ControlLog(PX4LITE_LOGMSG_RAMP_DOWN_ACCEPTED, Px4Lite_PlatformGetMs());
    if (s_auto_profile == PX4LITE_CONTROL_AUTO_NONE) {
      Px4Lite_ControlLog(PX4LITE_LOGMSG_OUTPUT_MINIMUM, Px4Lite_PlatformGetMs());
      Px4Lite_ControlLog(PX4LITE_LOGMSG_RAMP_DOWN_COMPLETED, Px4Lite_PlatformGetMs());
    }
    s_last_command_log = PX4LITE_LOGMSG_COUNT;
  } else { Px4Lite_ControlLogCommand((result == PX4LITE_BUSY) ? PX4LITE_LOGMSG_RAMP_DOWN_BUSY : PX4LITE_LOGMSG_LANDING_FAIL, Px4Lite_PlatformGetMs()); }
  (void)xTaskResumeAll();
  return result;
}

/** @brief 与控制周期串行提交，确保门禁和目标变更不可被其他任务交错。 */
Px4Lite_Result_t Px4Lite_ControlEmergencyStop(uint32_t now_ms)
{
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  result = Px4Lite_ControlEmergencyStopImpl(now_ms);
  (void)xTaskResumeAll();
  return result;
}

/** @brief 与控制周期串行提交，确保门禁和目标变更不可被其他任务交错。 */
Px4Lite_Result_t Px4Lite_ControlRecover(void)
{
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  result = Px4Lite_ControlRecoverImpl();
  (void)xTaskResumeAll();
  return result;
}

#else

Px4Lite_Result_t Px4Lite_ControlSetMotorThrottles(const uint8_t percent[PX4LITE_MOTOR_COUNT])
{
  (void)percent;
  return PX4LITE_NOT_READY;
}

Px4Lite_Result_t Px4Lite_ControlSetMotorPulses(const uint16_t pulse[PX4LITE_MOTOR_COUNT])
{
  (void)pulse;
  return PX4LITE_NOT_READY;
}

Px4Lite_Result_t Px4Lite_ControlSetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent)
{
  (void)motor_index;
  (void)throttle_percent;
  return PX4LITE_NOT_READY;
}

Px4Lite_Result_t Px4Lite_ControlSetMotorPulseUs(uint8_t motor_index, uint16_t pulse_us)
{
  (void)motor_index;
  (void)pulse_us;
  return PX4LITE_NOT_READY;
}

Px4Lite_Result_t Px4Lite_ControlStartAutoTakeoff(void)
{
  return PX4LITE_NOT_READY;
}

Px4Lite_Result_t Px4Lite_ControlStartAutoLanding(void)
{
  return PX4LITE_NOT_READY;
}

/**
 * @brief Control 模块关闭时的模式设置空实现。
 */
Px4Lite_Result_t Px4Lite_ControlSetMode(Px4Lite_ControlMode_t mode)
{
  (void)mode;
  return PX4LITE_NOT_READY;
}

/**
 * @brief Control 模块关闭时返回直控模式。
 */
Px4Lite_Result_t Px4Lite_ControlGetMode(Px4Lite_ControlMode_t *mode)
{
  if (mode == 0) { return PX4LITE_INVALID_PARAM; }
  *mode = PX4LITE_CONTROL_MODE_DIRECT;
  return PX4LITE_NOT_READY;
}

/**
 * @brief Control 模块关闭时的姿态目标设置空实现。
 */
Px4Lite_Result_t Px4Lite_ControlSetAttitudeTarget(int32_t roll_deg100, int32_t pitch_deg100, int32_t yaw_deg100)
{
  (void)roll_deg100;
  (void)pitch_deg100;
  (void)yaw_deg100;
  return PX4LITE_NOT_READY;
}

/**
 * @brief Control 模块关闭时的急停空实现。
 */
Px4Lite_Result_t Px4Lite_ControlEmergencyStop(uint32_t now_ms)
{
  (void)now_ms;
  return PX4LITE_NOT_READY;
}

/**
 * @brief Control 模块关闭时的动力电池闭锁空实现。
 */
uint8_t Px4Lite_ControlIsPowerLowVoltage(void)
{
  return 0U;
}

uint8_t Px4Lite_ControlIsPowerInhibited(void)
{
  return 1U;
}

Px4Lite_Result_t Px4Lite_ControlModuleInit(void)
{
  return PX4LITE_NOT_READY;
}

Px4Lite_Result_t Px4Lite_ControlRecover(void)
{
  return PX4LITE_NOT_READY;
}

void Px4Lite_ControlRun(uint32_t now_ms)
{
  (void)now_ms;
}

#endif
