/**
 * @file px4lite_control.c
 * @brief 实现电机人工锁、油门命令、按键急停、ESC 预备和失效保护。
 *
 * @details
 * 本文件位于 Framework 层，只通过 Platform Adapter 写入 PWM 和读取按键。
 * Business/Display 通过 `App_SetMotorThrottlePercent()` 写入目标油门，Control
 * 任务按固定周期统一下发，避免多个任务同时拥有同一组电机输出。
 */

#include "px4lite_control.h"
#include "px4lite_startup_tone.h"

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

#if PX4LITE_ENABLE_CNS_CONTROL && ((PX4LITE_CNS_IDLE_PULSE_US <= PX4LITE_CONTROL_ESC_MIN_PULSE_US) || \
    (PX4LITE_CNS_IDLE_PULSE_US > PX4LITE_CONTROL_ESC_MAX_PULSE_US) || (PX4LITE_CNS_ACK_TIMEOUT_MS == 0U))
#error "CNS idle pulse must exceed stopped pulse and ACK timeout must be positive"
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
static uint8_t s_control_initialized; /**< 1 表示已执行控制初始化，可查询人工锁。 */
static volatile uint8_t s_manual_locked = 1U; /**< 人工锁，启动/恢复/急停/控制超时均置 1；只有显式解锁可解除。 */
static uint8_t s_config_maintenance;
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
/**< 电压门禁事实；关闭门禁时恒为 0，不承担人工锁职责。 */
static volatile uint8_t s_power_inhibited = (PX4LITE_CONTROL_POWER_GATE_ENABLE != 0U) ? 1U : 0U;
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
#if PX4LITE_CONTROL_POWER_GATE_ENABLE
static uint8_t s_power_observed;
#endif
static uint8_t s_power_cutoff_pending;
static Px4Lite_ControlAutoProfile_t s_auto_complete_pending;
static uint16_t s_last_command_log = PX4LITE_LOGMSG_COUNT;
static uint32_t s_last_command_log_ms;

/** @brief CNS 阶段；WAIT_ACK 尚不执行，SOUND 等有限提示，RAMP/OUTPUT 等真实输出。 */
typedef enum {
  CONTROL_CNS_NONE = 0, CONTROL_CNS_WAIT_ACK, CONTROL_CNS_SOUND,
  CONTROL_CNS_RAMP, CONTROL_CNS_OUTPUT
} ControlCnsPhase_t;
/** @brief 单项受理槽；在既有控制调度互斥内访问，禁止迟到 ACK 恢复已取消动作。 */
typedef struct {
  uint32_t token; /**< 非零令牌，uint32 回绕时跳过 0。 */
  uint32_t accepted_ms; /**< 受理真实时刻，单位 ms，用于 ACK 过期。 */
  uint16_t pulse_us; /**< 指定通道脉宽，单位 us。 */
  uint16_t held_pulse_us[PX4LITE_MOTOR_COUNT]; /**< 调速受理时四路已确认输出，单位 us。 */
  uint8_t action; /**< Px4Lite_CnsAction_t，0 为无动作。 */
  uint8_t phase; /**< ControlCnsPhase_t。 */
  uint8_t motor_index; /**< 调速通道，0～3。 */
  uint8_t ack_sent; /**< 同令牌 ACK 已实际提交 UART 为 1。 */
} ControlCnsRequest_t;
static ControlCnsRequest_t s_cns;
#if PX4LITE_ENABLE_CNS_CONTROL
static uint32_t s_cns_next_token; /**< 恢复不清零，防止旧 ACK 与新动作复用令牌。 */
#endif
static uint8_t s_cns_exact_hold; /**< CNS 手动精确输出保持；旧入口接管即解除，不改变模式配置。 */
static uint16_t s_last_actual_pulse_us[PX4LITE_MOTOR_COUNT]; /**< 最近全部写入成功的实际 PWM，单位 us。 */
static void ControlCnsCancel(uint8_t all);

#if PX4LITE_RPI_LOCAL_RESULT_FEEDBACK
/** @brief 一项树莓派请求的结果观察种类；不改变控制目标或状态策略。 */
typedef enum {
  CONTROL_RPI_FEEDBACK_NONE = 0,
  CONTROL_RPI_FEEDBACK_OUTPUT,
  CONTROL_RPI_FEEDBACK_TAKEOFF,
  CONTROL_RPI_FEEDBACK_LANDING
} ControlRpiFeedbackKind_t;
/** @brief 与现有调度互斥共用的单项观察器；新实际目标覆盖时旧请求只报未执行。 */
typedef struct {
  uint8_t kind; /**< ControlRpiFeedbackKind_t；0为空，其余等待实际输出事实。 */
  uint8_t mode; /**< 接受时输出模式，Px4Lite_ControlMode_t。 */
  uint8_t is_pulse; /**< 1为微秒脉宽请求，0为百分比请求。 */
  uint8_t minimum_only; /**< 请求四路最小输出为1，锁/预备期成功写最小也可完成。 */
  uint8_t percent[PX4LITE_MOTOR_COUNT]; /**< 已提交的四路基础油门，单位%。 */
  uint16_t pulse_us[PX4LITE_MOTOR_COUNT]; /**< 已提交的四路脉宽，单位us。 */
} ControlRpiFeedback_t;
static ControlRpiFeedback_t s_rpi_feedback;
/** @brief 在已持有调度互斥的owner事实边界终结一次观察，只登记提示，不驱动声光。 */
static void ControlRpiFeedbackFinish(uint8_t succeeded)
{
  if (s_rpi_feedback.kind == CONTROL_RPI_FEEDBACK_NONE) { return; }
  s_rpi_feedback.kind = CONTROL_RPI_FEEDBACK_NONE;
  Px4Lite_StartupToneNotifyRpiResult(succeeded);
}
/** @brief 仅中止原树莓派自动过程的观察，保护缓降不继承成功资格。 */
static void ControlRpiFeedbackAbortAuto(void)
{
  if ((s_rpi_feedback.kind == CONTROL_RPI_FEEDBACK_TAKEOFF) ||
      (s_rpi_feedback.kind == CONTROL_RPI_FEEDBACK_LANDING)) { ControlRpiFeedbackFinish(0U); }
}
/** @brief 目标模式/姿态被真实改写时取消尚未消费的手动请求，不影响自动控制策略。 */
static void ControlRpiFeedbackContextChanged(void)
{
  if (s_rpi_feedback.kind == CONTROL_RPI_FEEDBACK_OUTPUT) { ControlRpiFeedbackFinish(0U); }
}
/** @brief 在同一提交锁内记录已接受目标；暂不把接受误当PWM执行成功。 */
static void ControlRpiFeedbackTrackOutput(uint8_t is_pulse)
{
  uint8_t i;
  ControlRpiFeedbackFinish(0U);
  s_rpi_feedback.kind = CONTROL_RPI_FEEDBACK_OUTPUT;
  s_rpi_feedback.mode = (uint8_t)s_control_mode;
  s_rpi_feedback.is_pulse = is_pulse;
  s_rpi_feedback.minimum_only = 1U;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) {
    s_rpi_feedback.percent[i] = s_target_throttle_percent[i];
    s_rpi_feedback.pulse_us[i] = s_target_pulse_us[i];
    if (s_target_pulse_us[i] != PX4LITE_CONTROL_ESC_MIN_PULSE_US || s_target_throttle_percent[i] != 0U) {
      s_rpi_feedback.minimum_only = 0U;
    }
  }
}
/** @brief 四路真实PWM写入后核对请求仍被消费；预备期非零目标继续等候。 */
static void ControlRpiFeedbackOutput(Px4Lite_Result_t result,
                                    const uint16_t pulse_us[PX4LITE_MOTOR_COUNT], uint8_t preparing)
{
  uint8_t i;
  if (s_rpi_feedback.kind != CONTROL_RPI_FEEDBACK_OUTPUT) { return; }
  if (result != PX4LITE_OK) { ControlRpiFeedbackFinish(0U); return; }
  if (s_rpi_feedback.minimum_only != 0U) {
    for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) {
      if (pulse_us[i] != PX4LITE_CONTROL_ESC_MIN_PULSE_US) { ControlRpiFeedbackFinish(0U); return; }
    }
    ControlRpiFeedbackFinish(1U);
    return;
  }
  if ((uint8_t)s_control_mode != s_rpi_feedback.mode) { ControlRpiFeedbackFinish(0U); return; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) {
    if (!s_target_valid[i] || s_target_is_pulse[i] != s_rpi_feedback.is_pulse ||
        s_target_throttle_percent[i] != s_rpi_feedback.percent[i] ||
        s_target_pulse_us[i] != s_rpi_feedback.pulse_us[i]) { ControlRpiFeedbackFinish(0U); return; }
  }
  if (!preparing) { ControlRpiFeedbackFinish(1U); }
}
/** @brief 自动斜坡达到终点且四路PWM成功后，才结束对应树莓派请求。 */
static void ControlRpiFeedbackAutoCompleted(Px4Lite_ControlAutoProfile_t profile)
{
  if (((profile == PX4LITE_CONTROL_AUTO_TAKEOFF) && s_rpi_feedback.kind == CONTROL_RPI_FEEDBACK_TAKEOFF) ||
      ((profile == PX4LITE_CONTROL_AUTO_LANDING) && s_rpi_feedback.kind == CONTROL_RPI_FEEDBACK_LANDING)) {
    ControlRpiFeedbackFinish(1U);
  }
}
#else
#define ControlRpiFeedbackFinish(succeeded) ((void)0)
#define ControlRpiFeedbackAbortAuto() ((void)0)
#define ControlRpiFeedbackContextChanged() ((void)0)
#define ControlRpiFeedbackOutput(result, pulse, preparing) ((void)0)
#define ControlRpiFeedbackAutoCompleted(profile) ((void)0)
#endif

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
  uint8_t i;
  Px4Lite_Result_t result = Px4Lite_MotorDisarmAll();
  if (result == PX4LITE_OK) {
    for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) { s_last_actual_pulse_us[i] = PX4LITE_CONTROL_ESC_MIN_PULSE_US; }
  }
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
  if (result == PX4LITE_OK) { memcpy(s_last_actual_pulse_us, pulse_us, sizeof(s_last_actual_pulse_us)); }
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
  ControlRpiFeedbackAbortAuto();
  if (s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) {
    Px4Lite_ControlLog((s_auto_profile == PX4LITE_CONTROL_AUTO_TAKEOFF) ?
      PX4LITE_LOGMSG_RAMP_UP_ABORTED : PX4LITE_LOGMSG_RAMP_DOWN_ABORTED, now_ms);
  }
  s_auto_complete_pending = PX4LITE_CONTROL_AUTO_NONE;
  Px4Lite_ControlClearAutoThrottle();
}

/** @brief 撤销 CNS 执行资格及专属提示；旧自动过程仅在由 CNS 持有时撤销。 */
static void ControlCnsCancel(uint8_t all)
{
#if PX4LITE_ENABLE_CNS_CONTROL
  uint8_t active = (uint8_t)(s_cns.phase != CONTROL_CNS_NONE);
  if (active && s_cns.phase == CONTROL_CNS_RAMP && s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) {
    Px4Lite_ControlAbortAuto(Px4Lite_PlatformGetMs());
  }
  memset(&s_cns, 0, sizeof(s_cns));
  s_cns_exact_hold = 0U;
  if (active || all) { Px4Lite_StartupToneCnsCancel(all); }
#else
  (void)all;
#endif
}

#if PX4LITE_ENABLE_CNS_CONTROL
/** @brief ACK 未放行时撤销的只是新请求；原已确认精确保持不能因超时而切回混控。 */
static void ControlCnsExpirePending(void)
{
  uint8_t hold = s_cns_exact_hold;
  ControlCnsCancel(0U);
  s_cns_exact_hold = hold;
}

/** @brief 动作实际完成；保留已提交的精确手动输出，不发送完成 ACK。 */
static void ControlCnsFinish(void)
{
  Px4Lite_StartupToneCnsCancel(0U);
  memset(&s_cns, 0, sizeof(s_cns));
}

/** @brief 在同一输出事务内建立四路精确保持，不改变学生选择的模式。 */
static void ControlCnsSetExactTargets(const uint16_t pulse_us[PX4LITE_MOTOR_COUNT], uint32_t now_ms)
{
  uint8_t i;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) {
    s_target_pulse_us[i] = pulse_us[i];
    s_target_throttle_percent[i] = Px4Lite_ControlPulseToThrottlePercent(pulse_us[i]);
    s_target_is_pulse[i] = 1U;
    s_target_update_ms[i] = now_ms;
    s_target_valid[i] = 1U;
  }
  s_cns_exact_hold = 1U;
}
#endif
/** @brief 仅供显式人工解锁解除急停，输出重新经历预解锁。 */
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
    return (profile == PX4LITE_CONTROL_AUTO_TAKEOFF) ? PX4LITE_NOT_READY : PX4LITE_OK;
  }

  ControlRpiFeedbackFinish(0U);
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

  ControlCnsCancel(0U);
  ControlRpiFeedbackAbortAuto();
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
  if ((s_estop_latched != 0U) || (s_power_inhibited != 0U) || (s_manual_locked != 0U)) {
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
 * @details 启动和恢复均清目标并人工上锁；有效采样或普通油门命令不会解除人工锁。
 */
static void Px4Lite_ControlReset(uint32_t now_ms)
{
  ControlCnsCancel(0U);
  ControlRpiFeedbackFinish(0U);
  s_manual_locked = 1U;
  s_power_inhibited = (PX4LITE_CONTROL_POWER_GATE_ENABLE != 0U) ? 1U : 0U;
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
  if (s_manual_locked != 0U) {
    outputs.operation_state = (uint8_t)PX4LITE_MOTOR_OP_LOCKED;
  } else if (s_power_inhibited != 0U) {
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
    duty_percent[i] = ((s_target_valid[i] != 0U) && (s_power_inhibited == 0U) && (s_manual_locked == 0U)) ? s_target_throttle_percent[i] : 0U;
    if (duty_percent[i] > 100U) { duty_percent[i] = 100U; }
  }
}

/**
 * @brief 在控制周期或命令提交前更新可配置的电压门禁；关闭时只旁路电压条件。
 * @param[in] now_ms 当前真实毫秒时间。
 * @note 调用者已挂起任务调度；快照内部短临界区，不依赖显示或远端数据。
 *       闭锁同时清目标、自动过程及预解锁进度，恢复不会重放旧命令。
 */
static void Px4Lite_ControlRefreshPowerGate(uint32_t now_ms)
{
#if PX4LITE_CONTROL_POWER_GATE_ENABLE
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
    ControlCnsCancel(0U);
    Px4Lite_ControlClearTargets();
    Px4Lite_ControlAbortAuto(now_ms);
    s_esc_armed = 0U;
    s_arm_start_ms = now_ms;
  }
  s_power_inhibited = inhibit;
#else
  /* 教学期间临时旁路；完整电压保护算法和阈值保留在上方，可由配置恢复。 */
  (void)now_ms;
  s_power_inhibited = 0U;
  s_power_low_voltage = 0U;
#endif
}

/** @brief 查询启用电压门禁时的低压事实；旁路时返回 0，采样低压事实仍由电源 topic 提供。 */
uint8_t Px4Lite_ControlIsPowerLowVoltage(void)
{
  return s_power_low_voltage;
}

/** @brief 查询本机供电闭锁，不允许上层写入解除。 */
uint8_t Px4Lite_ControlIsPowerInhibited(void)
{
  return s_power_inhibited;
}

/** @brief 在最小 PWM 已成功写入之后发布停止快照，不用命令接受冒充输出成功。 */
static void Px4Lite_ControlPublishMinimum(uint32_t now_ms)
{
  uint8_t zero[PX4LITE_MOTOR_COUNT] = {0U};
  uint16_t pulse[PX4LITE_MOTOR_COUNT];
  uint8_t i;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) { pulse[i] = PX4LITE_CONTROL_ESC_MIN_PULSE_US; }
  Px4Lite_ControlPublish(now_ms, zero, zero, pulse, 0U);
}

/**
 * @brief 显式修改人工锁；解锁先确认最小输出并丢弃旧目标，不替用户提交油门。
 * @note 与 Control 周期串行，保持中断可用；急停键按下、维护或启用的电压门禁拒绝解锁。
 */
Px4Lite_Result_t Px4Lite_ControlSetMotorLocked(uint8_t locked)
{
  Px4Lite_Result_t result;
  uint32_t now_ms;
  uint8_t changed;
  if (locked > 1U) { return PX4LITE_INVALID_PARAM; }
  vTaskSuspendAll();
  now_ms = Px4Lite_PlatformGetMs();
  if (s_control_initialized == 0U) { (void)xTaskResumeAll(); return PX4LITE_NOT_READY; }
  Px4Lite_ControlRefreshPowerGate(now_ms);
  ControlCnsCancel(0U);
  if (locked == 0U) {
    if ((s_config_maintenance != 0U) || (s_power_inhibited != 0U) ||
        (Px4Lite_ButtonPressed(PX4LITE_BUTTON_KEY1) != 0U)) {
      (void)xTaskResumeAll(); return PX4LITE_NOT_READY;
    }
    if (s_manual_locked == 0U) { (void)xTaskResumeAll(); return PX4LITE_OK; }
  }
  changed = (s_manual_locked != locked) ? 1U : 0U;
  ControlRpiFeedbackFinish(0U);
  /* 撤收失败也先保持人工锁；只有最小脉宽写入成功才能转为解锁。 */
  s_manual_locked = 1U;
  s_esc_armed = 0U;
  s_arm_start_ms = now_ms;
  s_last_run_ms = now_ms;
  s_last_run_valid = 1U;
  Px4Lite_ControlClearTargets();
  Px4Lite_ControlAbortAuto(now_ms);
  result = Px4Lite_ControlDisarmLogged(now_ms);
  if ((locked == 0U) && (result == PX4LITE_OK)) {
    Px4Lite_ControlReleaseEmergencyStop(now_ms);
    s_manual_locked = 0U;
    Px4Lite_ControlLog(PX4LITE_LOGMSG_MOTOR_MANUAL_UNLOCKED, now_ms);
  } else if ((locked != 0U) && (changed != 0U)) {
    Px4Lite_ControlLog(PX4LITE_LOGMSG_MOTOR_MANUAL_LOCKED, now_ms);
  }
  if (result == PX4LITE_OK) { Px4Lite_ControlPublishMinimum(now_ms); }
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL,
    (result != PX4LITE_OK || s_estop_latched) ? PX4LITE_STATE_DEGRADED :
      ((s_manual_locked != 0U) ? PX4LITE_STATE_ONLINE : PX4LITE_STATE_STARTING),
    (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SYSTEM_SELF_CHECK, now_ms);
  (void)xTaskResumeAll();
  return result;
}

/** @brief 在任务同步边界读取人工锁；未初始化时不向 UI 伪造可用状态。 */
Px4Lite_Result_t Px4Lite_ControlGetMotorLocked(uint8_t *locked)
{
  Px4Lite_Result_t result;
  if (locked == 0) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  result = s_control_initialized ? PX4LITE_OK : PX4LITE_NOT_READY;
  if (result == PX4LITE_OK) { *locked = s_manual_locked; }
  taskEXIT_CRITICAL();
  return result;
}

/** @brief 所有非零油门共用准入，人工锁和急停不能被普通命令解除。 */
static uint8_t Px4Lite_ControlRejectNonzero(uint32_t now_ms)
{
  if ((s_manual_locked != 0U) || (s_estop_latched != 0U) || (s_config_maintenance != 0U)) {
    Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_LOCKED, now_ms);
    return 1U;
  }
  if (s_power_inhibited != 0U) {
    Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_POWER_FAIL, now_ms);
    return 1U;
  }
  return 0U;
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
  if ((nonzero != 0U) && (Px4Lite_ControlRejectNonzero(now_ms) != 0U)) {
    (void)xTaskResumeAll();
    return PX4LITE_NOT_READY;
  }
  if ((s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING) && (s_auto_landing_user != 0U)) {
    if (nonzero != 0U) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_LANDING_THROTTLE_REJECT, now_ms); }
    (void)xTaskResumeAll();
    return (nonzero != 0U) ? PX4LITE_NOT_READY : PX4LITE_OK;
  }
  ControlCnsCancel(0U);
  ControlRpiFeedbackFinish(0U);
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
  /* 拒绝先于目标和自动过程变更；非零普通命令不能解除人工锁或急停。 */
  if ((throttle_percent != 0U) && (Px4Lite_ControlRejectNonzero(Px4Lite_PlatformGetMs()) != 0U)) { return PX4LITE_NOT_READY; }
  /* 一键降落是用户明确的撤收动作，进行中不接受新的非零手动油门；0 与降落终点一致，接受但
     不打断斜坡。姿态丢失触发的强制降落不在此列——那是内部保护，必须留出手动接管的出口。
     一键起飞也不做此限制，手动接管随时可以打断上升。 */
  if ((s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING) && (s_auto_landing_user != 0U)) {
    if (throttle_percent != 0U) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_LANDING_THROTTLE_REJECT, Px4Lite_PlatformGetMs()); return PX4LITE_NOT_READY; }
    return PX4LITE_OK;
  }
  ControlCnsCancel(0U);
  ControlRpiFeedbackFinish(0U);
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
  if ((pulse_us != PX4LITE_CONTROL_ESC_MIN_PULSE_US) && (Px4Lite_ControlRejectNonzero(Px4Lite_PlatformGetMs()) != 0U)) { return PX4LITE_NOT_READY; }
  if ((s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING) && (s_auto_landing_user != 0U)) {
    if (pulse_us != PX4LITE_CONTROL_ESC_MIN_PULSE_US) { Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_LANDING_THROTTLE_REJECT, Px4Lite_PlatformGetMs()); return PX4LITE_NOT_READY; }
    return PX4LITE_OK;
  }
  ControlCnsCancel(0U);
  ControlRpiFeedbackFinish(0U);
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
  /* 起飞是加油门动作，人工锁/急停不能被起飞命令解除；降落撤收永远允许。 */
  if ((s_manual_locked != 0U) || (s_estop_latched != 0U)) {
    Px4Lite_ControlLogCommand(PX4LITE_LOGMSG_THROTTLE_LOCKED, Px4Lite_PlatformGetMs());
    return PX4LITE_NOT_READY;
  }
  if ((s_power_inhibited != 0U) || (s_config_maintenance != 0U)) {
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

  if ((s_power_inhibited != 0U) || (s_estop_latched != 0U) || (s_config_maintenance != 0U) || (s_manual_locked != 0U)) {
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

  /* 配置赋值与旧观察失效属于同一短事务，防止新RPi请求插入后被旧设置误取消。 */
  vTaskSuspendAll();
  if (s_control_mode != mode) { ControlCnsCancel(0U); ControlRpiFeedbackContextChanged(); }
  s_control_mode = mode;
  (void)xTaskResumeAll();
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

  /* 三轴赋值和观察失效保持原子；只序列化已有配置写入，不新增输出或准入条件。 */
  vTaskSuspendAll();
  if (s_target_roll_deg100 != roll_deg100 || s_target_pitch_deg100 != pitch_deg100 || s_target_yaw_deg100 != yaw_deg100) {
    ControlCnsCancel(0U);
    ControlRpiFeedbackContextChanged();
  }
  s_target_roll_deg100  = roll_deg100;
  s_target_pitch_deg100 = pitch_deg100;
  s_target_yaw_deg100   = yaw_deg100;
  (void)xTaskResumeAll();
  return PX4LITE_OK;
}

/**
 * @brief 锁存急停状态、清零目标油门，并立即写入 ESC 最小脉宽。
 */
static Px4Lite_Result_t Px4Lite_ControlEmergencyStopImpl(uint32_t now_ms)
{
  Px4Lite_Result_t result;

  ControlCnsCancel(0U);
  ControlRpiFeedbackFinish(0U);
  Px4Lite_ControlLog(PX4LITE_LOGMSG_ESTOP, now_ms);
  s_manual_locked = 1U;
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
  /* 首次owner初始化也原子撤销旧观察与目标，避免初始化前被接受的零目标漏报未执行。 */
  vTaskSuspendAll();
  Px4Lite_ControlReset(now_ms);
  s_control_initialized = (result == PX4LITE_OK) ? 1U : 0U;
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL, (result == PX4LITE_OK) ? PX4LITE_STATE_STARTING : PX4LITE_STATE_DEGRADED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SYSTEM_SELF_CHECK, now_ms);
  (void)xTaskResumeAll();
  return result;
}

static Px4Lite_Result_t Px4Lite_ControlRecoverImpl(void)
{
  uint32_t now_ms = Px4Lite_PlatformGetMs();

  Px4Lite_ControlReset(now_ms);
  Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL, PX4LITE_STATE_STARTING, PX4LITE_FAULT_NONE, now_ms);
  return PX4LITE_OK;
}

#if PX4LITE_ENABLE_CNS_CONTROL
/** @brief 限时等待 ACK，随后推进声音与动作；调用时已持有控制调度互斥。 */
static void ControlCnsService(uint32_t now_ms)
{
  Px4Lite_Result_t tone_result;
  uint16_t pulse_us[PX4LITE_MOTOR_COUNT];
  uint8_t i;
  if (s_cns.phase == CONTROL_CNS_NONE) { return; }
  if (s_cns.phase == CONTROL_CNS_WAIT_ACK) {
    if (!s_cns.ack_sent) {
      if ((uint32_t)(now_ms - s_cns.accepted_ms) >= PX4LITE_CNS_ACK_TIMEOUT_MS) { ControlCnsExpirePending(); }
      return;
    }
    /* ACK 已在真实截止时间内完成；本次 Control 的调度延后不撤销已确认的受理。 */
    if (s_cns.action == PX4LITE_CNS_ARM) {
      /* ARM 无锁状态前提；重复解锁也先撤销旧目标，声音期间必须保持真实锁定。 */
      s_manual_locked = 1U;
      s_esc_armed = 0U;
      s_arm_start_ms = now_ms;
      s_cns_exact_hold = 0U;
      Px4Lite_ControlClearTargets();
      Px4Lite_ControlAbortAuto(now_ms);
      if (Px4Lite_ControlDisarmLogged(now_ms) != PX4LITE_OK) {
        ControlCnsCancel(0U); return;
      }
    }
    s_cns.phase = CONTROL_CNS_SOUND;
    Px4Lite_StartupToneCnsStart(s_cns.action, s_cns.token);
    if (s_cns.action == PX4LITE_CNS_LAND) {
      if (s_manual_locked || s_estop_latched || s_config_maintenance || s_power_inhibited ||
          Px4Lite_ControlStartAutoThrottle(PX4LITE_CONTROL_AUTO_LANDING) != PX4LITE_OK) {
        ControlCnsCancel(0U); return;
      }
      s_cns_exact_hold = 0U;
      s_auto_landing_user = 1U;
      for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) {
        s_auto_target_percent[i] = Px4Lite_ControlPulseToThrottlePercent(PX4LITE_CNS_IDLE_PULSE_US);
      }
      s_cns.phase = CONTROL_CNS_RAMP;
      return;
    }
  }
  if (s_cns.phase != CONTROL_CNS_SOUND) { return; }
  tone_result = Px4Lite_StartupToneCnsResult(s_cns.token);
  if (tone_result == PX4LITE_BUSY) { return; }
  if (tone_result != PX4LITE_OK) { ControlCnsCancel(0U); return; }
  if (s_cns.action == PX4LITE_CNS_DISARM || s_cns.action == PX4LITE_CNS_EMERGENCY_STOP) {
    ControlCnsFinish(); return;
  }
  if (s_config_maintenance || s_power_inhibited || Px4Lite_ButtonPressed(PX4LITE_BUTTON_KEY1)) {
    ControlCnsCancel(0U); return;
  }
  if (s_cns.action == PX4LITE_CNS_ARM) {
    for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) { pulse_us[i] = PX4LITE_CNS_IDLE_PULSE_US; }
    /* 四路全部成功前人工锁保持为 1；逐路输出回调也不能提前看到 armed。 */
    if (Px4Lite_ControlWriteAll(pulse_us) != PX4LITE_OK) {
      ControlCnsCancel(0U);
      s_manual_locked = 1U; s_esc_armed = 0U;
      Px4Lite_ControlClearTargets();
      (void)Px4Lite_ControlDisarmLogged(now_ms);
      return;
    }
    Px4Lite_ControlReleaseEmergencyStop(now_ms);
    ControlCnsSetExactTargets(pulse_us, now_ms);
    s_manual_locked = 0U;
    s_esc_armed = 1U;
    Px4Lite_ControlLog(PX4LITE_LOGMSG_MOTOR_MANUAL_UNLOCKED, now_ms);
    ControlCnsFinish();
  } else if (s_manual_locked || s_estop_latched) {
    ControlCnsCancel(0U);
  } else if (s_cns.action == PX4LITE_CNS_TAKEOFF) {
    /* 前置声响期间依赖可能失效，不能仅依赖受理时的一次检查。 */
    if (!Px4Lite_ControlAutoTakeoffSensorsReady(now_ms) ||
        Px4Lite_ControlStartAutoThrottle(PX4LITE_CONTROL_AUTO_TAKEOFF) != PX4LITE_OK) {
      ControlCnsCancel(0U); return;
    }
    s_cns_exact_hold = 0U;
    s_cns.phase = CONTROL_CNS_RAMP;
  } else if (s_cns.action == PX4LITE_CNS_SET_MOTOR_PWM) {
    memcpy(pulse_us, s_cns.held_pulse_us, sizeof(pulse_us));
    pulse_us[s_cns.motor_index] = s_cns.pulse_us;
    ControlCnsSetExactTargets(pulse_us, now_ms);
    s_cns.phase = CONTROL_CNS_OUTPUT;
  }
}

/** @brief CNS 降落终点使用精确怠速，不受辅助混控改变；其他旧降落仍保持原最小输出。 */
static void ControlCnsPrepareLandingEnd(uint32_t now_ms)
{
  uint16_t pulse_us[PX4LITE_MOTOR_COUNT];
  uint8_t i;
  if (s_cns.phase != CONTROL_CNS_RAMP || s_cns.action != PX4LITE_CNS_LAND ||
      s_auto_complete_pending != PX4LITE_CONTROL_AUTO_LANDING) { return; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) { pulse_us[i] = PX4LITE_CNS_IDLE_PULSE_US; }
  ControlCnsSetExactTargets(pulse_us, now_ms);
}

/** @brief 实际四路输出成功后才能结束调速/起降；失败取消执行资格交由既有撤收分支处理。 */
static void ControlCnsAfterOutput(Px4Lite_Result_t result)
{
  if (s_cns.phase == CONTROL_CNS_NONE) { return; }
  if (result != PX4LITE_OK) { ControlCnsCancel(0U); return; }
  if (s_cns.phase == CONTROL_CNS_OUTPUT ||
      (s_cns.phase == CONTROL_CNS_RAMP &&
       ((s_cns.action == PX4LITE_CNS_TAKEOFF && s_auto_complete_pending == PX4LITE_CONTROL_AUTO_TAKEOFF) ||
        (s_cns.action == PX4LITE_CNS_LAND && s_auto_complete_pending == PX4LITE_CONTROL_AUTO_LANDING)))) {
    ControlCnsFinish();
  }
}

/** @brief 受理一项 CNS 请求；拒绝不更改既有目标，停车动作不依赖 ACK 排队。 */
Px4Lite_Result_t Px4Lite_ControlCnsRequest(Px4Lite_CnsAction_t action, uint8_t motor_index,
                                         uint16_t pulse_us, uint8_t *reason, uint32_t *token)
{
  uint32_t now_ms;
  Px4Lite_Result_t result = PX4LITE_OK;
  if (reason == 0 || token == 0) { return PX4LITE_INVALID_PARAM; }
  *reason = 0U; *token = 0U;
  if (action < PX4LITE_CNS_ARM || action > PX4LITE_CNS_SET_MOTOR_PWM ||
      (action == PX4LITE_CNS_SET_MOTOR_PWM && (motor_index >= PX4LITE_MOTOR_COUNT ||
       pulse_us < PX4LITE_CONTROL_ESC_MIN_PULSE_US || pulse_us > PX4LITE_CONTROL_ESC_MAX_PULSE_US))) {
    *reason = 3U; return PX4LITE_INVALID_PARAM;
  }
  vTaskSuspendAll();
  now_ms = Px4Lite_PlatformGetMs();
  Px4Lite_ControlRefreshPowerGate(now_ms);
  if (action == PX4LITE_CNS_EMERGENCY_STOP) {
    result = Px4Lite_ControlEmergencyStopImpl(now_ms);
    Px4Lite_StartupToneCnsCancel(1U);
  } else if (!s_control_initialized) {
    *reason = 4U; result = PX4LITE_NOT_READY;
  } else if (action != PX4LITE_CNS_ARM && s_manual_locked) {
    *reason = 1U; result = PX4LITE_NOT_READY;
  } else if (action == PX4LITE_CNS_DISARM) {
    result = Px4Lite_ControlSetMotorLocked(1U);
  } else if (s_cns.phase != CONTROL_CNS_NONE || s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) {
    *reason = 2U; result = PX4LITE_BUSY;
  } else if (s_config_maintenance || s_power_inhibited || Px4Lite_ButtonPressed(PX4LITE_BUTTON_KEY1) ||
             (action != PX4LITE_CNS_ARM && (s_estop_latched || !s_esc_armed))) {
    *reason = 4U; result = PX4LITE_NOT_READY;
  } else if (action == PX4LITE_CNS_TAKEOFF && !Px4Lite_ControlAutoTakeoffSensorsReady(now_ms)) {
    *reason = 5U; result = PX4LITE_IO_ERROR;
  } else if (s_output_write_failed) {
    result = PX4LITE_IO_ERROR;
  }
  if (result == PX4LITE_OK) {
    /* 普通动作受理前已确认槽为空；保留前一 CNS 动作成功建立的精确怠速/手动保持。 */
    memset(&s_cns, 0, sizeof(s_cns));
    ++s_cns_next_token;
    if (!s_cns_next_token) { ++s_cns_next_token; }
    s_cns.token = s_cns_next_token;
    s_cns.accepted_ms = now_ms;
    s_cns.action = (uint8_t)action;
    s_cns.phase = CONTROL_CNS_WAIT_ACK;
    s_cns.motor_index = motor_index;
    s_cns.pulse_us = pulse_us;
    memcpy(s_cns.held_pulse_us, s_last_actual_pulse_us, sizeof(s_cns.held_pulse_us));
    *token = s_cns.token;
  }
  (void)xTaskResumeAll();
  return result;
}

/** @brief 只解锁同一项未过期请求的执行资格；旧令牌不能恢复中断的动作。 */
void Px4Lite_ControlCnsAckSent(uint32_t token)
{
  uint32_t now_ms;
  vTaskSuspendAll();
  now_ms = Px4Lite_PlatformGetMs();
  if (token && s_cns.phase == CONTROL_CNS_WAIT_ACK && s_cns.token == token) {
    if ((uint32_t)(now_ms - s_cns.accepted_ms) < PX4LITE_CNS_ACK_TIMEOUT_MS) { s_cns.ack_sent = 1U; }
    else { ControlCnsExpirePending(); }
  }
  (void)xTaskResumeAll();
}
#else
/** @brief CNS 配置关闭时不推进动作，不依赖专属声光实现。 */
static void ControlCnsService(uint32_t now_ms) { (void)now_ms; }
/** @brief CNS 配置关闭时保留旧降落最小输出语义。 */
static void ControlCnsPrepareLandingEnd(uint32_t now_ms) { (void)now_ms; }
/** @brief CNS 配置关闭时没有待完成动作。 */
static void ControlCnsAfterOutput(Px4Lite_Result_t result) { (void)result; }
/** @brief CNS 配置关闭时明确拒绝请求，避免受理后永远等待缺席声光。 */
Px4Lite_Result_t Px4Lite_ControlCnsRequest(Px4Lite_CnsAction_t action, uint8_t motor_index,
                                         uint16_t pulse_us, uint8_t *reason, uint32_t *token)
{
  (void)action; (void)motor_index; (void)pulse_us;
  if (reason == 0 || token == 0) { return PX4LITE_INVALID_PARAM; }
  *reason = 4U; *token = 0U; return PX4LITE_NOT_READY;
}
/** @brief CNS 配置关闭时忽略晚到 ACK。 */
void Px4Lite_ControlCnsAckSent(uint32_t token) { (void)token; }
#endif

/** @brief 实际锁事实的短临界区快照，不根据 PWM、run_state 或已受理请求推测。 */
uint8_t Px4Lite_ControlCnsArmed(void)
{
  uint8_t armed;
  taskENTER_CRITICAL();
  armed = (uint8_t)(s_control_initialized && !s_manual_locked);
  taskEXIT_CRITICAL();
  return armed;
}

/** @brief 无阻塞地完成一次目标消费和输出；由外层保持任务之间的状态一致性。 */
static void Px4Lite_ControlRunCycle(uint32_t now_ms)
{
  uint16_t pulse_us[PX4LITE_MOTOR_COUNT];
  uint8_t duty_percent[PX4LITE_MOTOR_COUNT];
  uint8_t base_percent[PX4LITE_MOTOR_COUNT];
  int32_t duty_percent_x100[PX4LITE_MOTOR_COUNT];
  uint8_t output_confirmed = 1U;
  uint8_t i;
  Px4Lite_Result_t result;

  if ((s_last_run_valid != 0U) && (Px4Lite_ElapsedMs(now_ms, s_last_run_ms) > PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS)) {
    ControlCnsCancel(0U);
    Px4Lite_ControlLog(PX4LITE_LOGMSG_CONTROL_TIMEOUT, now_ms);
    s_manual_locked = 1U;
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

  ControlCnsService(now_ms);

  if (s_config_maintenance || s_manual_locked || s_estop_latched || s_power_inhibited) {
    s_esc_armed=0U; s_arm_start_ms=now_ms; Px4Lite_ControlClearTargets();
  }
  if (s_esc_armed == 0U) {
    for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
      duty_percent[i] = 0U;
      pulse_us[i]     = PX4LITE_CONTROL_ESC_MIN_PULSE_US;
    }
    result = Px4Lite_ControlWriteAll(pulse_us);
    ControlRpiFeedbackOutput(result, pulse_us, 1U);
    if ((result == PX4LITE_OK) && !s_config_maintenance && !s_manual_locked && !s_estop_latched &&
        !s_power_inhibited && Px4Lite_ElapsedMs(now_ms, s_arm_start_ms) >= PX4LITE_CONTROL_ESC_ARM_TIME_MS) { s_esc_armed = 1U; }
    if (result != PX4LITE_OK) {
      ControlCnsCancel(0U);
      s_manual_locked = 1U;
      Px4Lite_ControlClearTargets();
      Px4Lite_ControlAbortAuto(now_ms);
    }
    if (s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) { s_auto_start_ms = now_ms; }
    /* 预解锁期间硬件停在 1000us，因此实际输出为 0；学生设定单独保存在
       base_percent，页面可同时解释“目标已接受”和“ESC 尚未输出”。 */
    Px4Lite_ControlCopyTargetPercent(base_percent);
    /* 本轮最小 PWM 写入失败时保留最后一次已确认快照，不能把撤收请求当成停机事实。 */
    if (result == PX4LITE_OK) { Px4Lite_ControlPublish(now_ms, duty_percent, base_percent, pulse_us, 0U); }
    Px4Lite_SetExternalModuleState(PX4LITE_MODULE_CONTROL,
      (result != PX4LITE_OK || s_estop_latched) ? PX4LITE_STATE_DEGRADED :
        (s_manual_locked ? PX4LITE_STATE_ONLINE : PX4LITE_STATE_STARTING),
      (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SYSTEM_SELF_CHECK, now_ms);
    return;
  }

  Px4Lite_ControlMonitorAutoTakeoff(now_ms);
  Px4Lite_ControlApplyAutoThrottle(now_ms);
  ControlCnsPrepareLandingEnd(now_ms);

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if ((s_estop_latched != 0U) || (s_power_inhibited != 0U) || (s_manual_locked != 0U) || (s_target_valid[i] == 0U)) {
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
  if ((s_estop_latched == 0U) && (s_power_inhibited == 0U) && !s_cns_exact_hold) { Px4Lite_ControlApplyAttitudeAssist(now_ms, duty_percent_x100); }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if ((s_target_is_pulse[i] != 0U) &&
        ((s_control_mode == PX4LITE_CONTROL_MODE_DIRECT) || s_cns_exact_hold) &&
        (s_estop_latched == 0U)) {
      pulse_us[i] = s_target_pulse_us[i];
    } else {
      pulse_us[i] = Px4Lite_ControlThrottleX100ToPulseUs(duty_percent_x100[i]);
      duty_percent[i] = Px4Lite_ControlClampDuty((duty_percent_x100[i] + 50) /
                                                 PX4LITE_CONTROL_PERCENT_X100_SCALE);
    }
  }

  if (s_cns.action == PX4LITE_CNS_SET_MOTOR_PWM &&
      (s_cns.phase == CONTROL_CNS_WAIT_ACK || s_cns.phase == CONTROL_CNS_SOUND)) {
    /* 声光前后均以受理时的真实输出为基准，不让姿态混控悄悄改变另三路。 */
    for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) {
      pulse_us[i] = s_cns.held_pulse_us[i];
      duty_percent[i] = Px4Lite_ControlPulseToThrottlePercent(pulse_us[i]);
    }
  }

  result = Px4Lite_ControlWriteAll(pulse_us);
  ControlCnsAfterOutput(result);
  ControlRpiFeedbackOutput(result, pulse_us, 0U);
  if (result == PX4LITE_OK) {
    if ((s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) && (s_auto_logged_start == 0U)) {
      Px4Lite_ControlLog((s_auto_profile == PX4LITE_CONTROL_AUTO_TAKEOFF) ? PX4LITE_LOGMSG_RAMP_UP_STARTED :
        ((s_auto_landing_user != 0U) ? PX4LITE_LOGMSG_RAMP_DOWN_STARTED : PX4LITE_LOGMSG_PROTECT_DOWN_STARTED), now_ms);
      s_auto_logged_start = 1U;
    }
    if (s_auto_complete_pending != PX4LITE_CONTROL_AUTO_NONE) {
      ControlRpiFeedbackAutoCompleted(s_auto_complete_pending);
      Px4Lite_ControlLog((s_auto_complete_pending == PX4LITE_CONTROL_AUTO_TAKEOFF) ?
        PX4LITE_LOGMSG_RAMP_UP_COMPLETED : PX4LITE_LOGMSG_RAMP_DOWN_COMPLETED, now_ms);
      s_auto_complete_pending = PX4LITE_CONTROL_AUTO_NONE;
    }
  } else {
    /* 任意 PWM 写入失败都撤收并重新上锁，普通油门不能重放失败前的目标。 */
    s_manual_locked = 1U;
    s_esc_armed = 0U;
    s_arm_start_ms = now_ms;
    if ((s_auto_profile == PX4LITE_CONTROL_AUTO_NONE) && (s_auto_complete_pending != PX4LITE_CONTROL_AUTO_NONE)) { s_auto_profile = s_auto_complete_pending; }
    Px4Lite_ControlAbortAuto(now_ms);
    Px4Lite_ControlClearTargets();
    if (Px4Lite_ControlDisarmLogged(now_ms) == PX4LITE_OK) {
      for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) {
        duty_percent[i] = 0U; base_percent[i] = 0U; pulse_us[i] = PX4LITE_CONTROL_ESC_MIN_PULSE_US;
      }
    } else { output_confirmed = 0U; }
  }
  if (output_confirmed != 0U) {
    Px4Lite_ControlPublish(now_ms, duty_percent, base_percent, pulse_us,
      ((result == PX4LITE_OK) && (s_estop_latched == 0U) && (s_power_inhibited == 0U) && (s_manual_locked == 0U)) ? 1U : 0U);
  }
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
  if (result == PX4LITE_OK) {
    /* 原门控先决定是否接受；不能先清掉正在降落的 profile 再误受理旧起飞。 */
    ControlCnsCancel(0U);
    Px4Lite_ControlLog(PX4LITE_LOGMSG_RAMP_UP_ACCEPTED, Px4Lite_PlatformGetMs()); s_last_command_log = PX4LITE_LOGMSG_COUNT;
  }
  (void)xTaskResumeAll();
  return result;
}

/** @brief 与控制周期串行提交，确保门禁和目标变更不可被其他任务交错。 */
Px4Lite_Result_t Px4Lite_ControlStartAutoLanding(void)
{
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  Px4Lite_ControlRefreshPowerGate(Px4Lite_PlatformGetMs());
  /* 重复降落按旧逻辑返回 BUSY，不能为了取消 CNS 而重置原曲线。 */
  if (s_auto_profile != PX4LITE_CONTROL_AUTO_LANDING) { ControlCnsCancel(0U); }
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

/**
 * @brief 确认最近 PWM 写入成功、全部目标停止且没有自动过程。
 * @note 调用者必须已挂起任务调度；只检查 Control 输出事实，不要求接入动力电源。
 */
static uint8_t Px4Lite_ControlConfigOutputsStopped(void)
{
  uint8_t i;
  if (!s_last_run_valid || s_output_above_minimum || s_output_write_failed ||
      s_auto_profile != PX4LITE_CONTROL_AUTO_NONE) { return 0U; }
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (s_target_valid[i] && (s_target_throttle_percent[i] != 0U ||
        s_target_pulse_us[i] > PX4LITE_CONTROL_ESC_MIN_PULSE_US)) { return 0U; }
  }
  return 1U;
}

/**
 * @brief 在调度互斥内确认停止并建立配置维护锁，允许未接动力电源时保存。
 * @note 本接口假定主控供电符合 Flash 编程要求；动力电压闭锁仍只约束电机运行。
 */
Px4Lite_Result_t Px4Lite_ControlBeginConfigMaintenance(void)
{
  uint8_t safe;
  uint32_t now;
  vTaskSuspendAll(); now=Px4Lite_PlatformGetMs();
  safe=(!s_config_maintenance && Px4Lite_ControlConfigOutputsStopped())?1U:0U;
  if(safe) { ControlCnsCancel(0U); s_config_maintenance=1U; s_esc_armed=0U; s_arm_start_ms=now; Px4Lite_ControlClearTargets(); }
  (void)xTaskResumeAll(); return safe?PX4LITE_OK:PX4LITE_NOT_READY;
}
/** @brief 每步复核维护锁与停止事实，不以动力电压闭锁拒绝配置保存。 */
Px4Lite_Result_t Px4Lite_ControlCheckConfigMaintenance(void)
{
  uint8_t safe;
  vTaskSuspendAll();
  safe=(s_config_maintenance && Px4Lite_ControlConfigOutputsStopped())?1U:0U;
  (void)xTaskResumeAll(); return safe?PX4LITE_OK:PX4LITE_NOT_READY;
}
/** @brief 撤销旧命令并恢复预解锁，结束维护不启动电机。 */
void Px4Lite_ControlEndConfigMaintenance(void)
{
  vTaskSuspendAll(); ControlCnsCancel(0U); Px4Lite_ControlClearTargets(); s_esc_armed=0U;
  s_arm_start_ms=Px4Lite_PlatformGetMs(); s_config_maintenance=0U; (void)xTaskResumeAll();
}
#else
/** @brief Control 关闭时不接受 CNS 请求，也不伪造解锁状态。 */
Px4Lite_Result_t Px4Lite_ControlCnsRequest(Px4Lite_CnsAction_t action, uint8_t motor_index,
                                         uint16_t pulse_us, uint8_t *reason, uint32_t *token)
{
  (void)action; (void)motor_index; (void)pulse_us;
  if (reason == 0 || token == 0) { return PX4LITE_INVALID_PARAM; }
  *reason = 4U; *token = 0U; return PX4LITE_NOT_READY;
}
/** @brief Control 关闭时忽略 ACK 令牌。 */
void Px4Lite_ControlCnsAckSent(uint32_t token) { (void)token; }
/** @brief Control 关闭时始终锁定。 */
uint8_t Px4Lite_ControlCnsArmed(void) { return 0U; }
/** @brief Control 关闭时无法执行人工锁命令。 */
Px4Lite_Result_t Px4Lite_ControlSetMotorLocked(uint8_t locked)
{ return (locked > 1U) ? PX4LITE_INVALID_PARAM : PX4LITE_NOT_READY; }
/** @brief Control 关闭时人工锁状态不可用，不伪造已解锁。 */
Px4Lite_Result_t Px4Lite_ControlGetMotorLocked(uint8_t *locked)
{ return (locked == 0) ? PX4LITE_INVALID_PARAM : PX4LITE_NOT_READY; }
/** @brief 无 Control 时无法证明电机停止，拒绝维护。 */
Px4Lite_Result_t Px4Lite_ControlBeginConfigMaintenance(void) { return PX4LITE_NOT_READY; }
/** @brief 无 Control 时拒绝维护门禁。 */
Px4Lite_Result_t Px4Lite_ControlCheckConfigMaintenance(void) { return PX4LITE_NOT_READY; }
/** @brief 模块关闭时无锁可释放。 */
void Px4Lite_ControlEndConfigMaintenance(void) { }

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


/** @brief 树莓派四路百分比提交；本地模式原子登记结果观察，Pi模式只转交原准入/执行。 */
Px4Lite_Result_t Px4Lite_ControlSetRpiMotorThrottles(const uint8_t percent[PX4LITE_MOTOR_COUNT])
{
#if PX4LITE_ENABLE_CONTROL && PX4LITE_RPI_LOCAL_RESULT_FEEDBACK
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  result = Px4Lite_ControlSetMotorThrottles(percent);
  if (result == PX4LITE_OK) {
    /* 用户缓降期间全零只确认旧意图，没有执行新目标，不能替原过程提前报成功。 */
    if (s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING && s_auto_landing_user) { Px4Lite_StartupToneNotifyRpiResult(0U); }
    else { ControlRpiFeedbackTrackOutput(0U); }
  }
  (void)xTaskResumeAll();
  return result;
#else
  return Px4Lite_ControlSetMotorThrottles(percent);
#endif
}
/** @brief 树莓派四路微秒脉宽提交；仅本地模式附加结果观察，不新增硬件写入。 */
Px4Lite_Result_t Px4Lite_ControlSetRpiMotorPulses(const uint16_t pulse[PX4LITE_MOTOR_COUNT])
{
#if PX4LITE_ENABLE_CONTROL && PX4LITE_RPI_LOCAL_RESULT_FEEDBACK
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  result = Px4Lite_ControlSetMotorPulses(pulse);
  if (result == PX4LITE_OK) {
    if (s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING && s_auto_landing_user) { Px4Lite_StartupToneNotifyRpiResult(0U); }
    else { ControlRpiFeedbackTrackOutput(1U); }
  }
  (void)xTaskResumeAll();
  return result;
#else
  return Px4Lite_ControlSetMotorPulses(pulse);
#endif
}
/** @brief 树莓派自动起飞仅本地模式登记终点观察；原返回OK仍只表示斜坡已接受。 */
Px4Lite_Result_t Px4Lite_ControlStartRpiAutoTakeoff(void)
{
#if PX4LITE_ENABLE_CONTROL && PX4LITE_RPI_LOCAL_RESULT_FEEDBACK
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  result = Px4Lite_ControlStartAutoTakeoff();
  if (result == PX4LITE_OK) {
    ControlRpiFeedbackFinish(0U);
    s_rpi_feedback.kind = CONTROL_RPI_FEEDBACK_TAKEOFF;
  }
  (void)xTaskResumeAll();
  return result;
#else
  return Px4Lite_ControlStartAutoTakeoff();
#endif
}
/** @brief 树莓派降落仅本地模式登记真实终点/同步撤收结果，Pi模式不产生自动声光。 */
Px4Lite_Result_t Px4Lite_ControlStartRpiAutoLanding(void)
{
#if PX4LITE_ENABLE_CONTROL && PX4LITE_RPI_LOCAL_RESULT_FEEDBACK
  Px4Lite_Result_t result;
  vTaskSuspendAll();
  result = Px4Lite_ControlStartAutoLanding();
  if (result == PX4LITE_OK) {
    ControlRpiFeedbackFinish(0U);
    if (s_auto_profile == PX4LITE_CONTROL_AUTO_LANDING) { s_rpi_feedback.kind = CONTROL_RPI_FEEDBACK_LANDING; }
    else { Px4Lite_StartupToneNotifyRpiResult(1U); }
  }
  (void)xTaskResumeAll();
  return result;
#else
  return Px4Lite_ControlStartAutoLanding();
#endif
}
