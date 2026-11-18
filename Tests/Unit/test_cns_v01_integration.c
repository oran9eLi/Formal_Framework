/**
 * @file test_cns_v01_integration.c
 * @brief CNS V0.1真实Control与真实sensor声光播放器的动作时序集成测试。
 * @details 仅替换平台I/O、时钟和输入快照，不替换控制或声光状态机；
 *          不连接硬件，不将PWM请求成功解释为电机转速或实际声光验收。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef int BaseType_t;
typedef void *TaskHandle_t;
static unsigned s_test_critical, s_test_scheduler, s_test_sensor_owner;
#define taskENTER_CRITICAL() (++s_test_critical)
#define taskEXIT_CRITICAL() do { assert(s_test_critical); --s_test_critical; } while (0)
#define vTaskSuspendAll() (++s_test_scheduler)
#define xTaskResumeAll() (assert(s_test_scheduler), --s_test_scheduler, 0)

#include "px4lite_config.h"
#include "px4lite_control.h"
#include "px4lite_startup_tone.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"
#include "px4lite_log_message_ids.h"

/** @brief 平台替身的实际输出与注入条件；时刻均为真实毫秒单位。 */
static struct {
  uint32_t now_ms; /**< 单调模拟平台时钟，单位ms。 */
  uint32_t low_started_ms; /**< 本场景首次低音输出时刻，单位ms，0为未发生。 */
  uint32_t above_minimum_ms; /**< 本场景首次非最小PWM写入时刻，单位ms。 */
  uint16_t pwm[PX4LITE_MOTOR_COUNT]; /**< 最后成功平台PWM，单位us。 */
  uint16_t hz; /**< 最后成功蜂鸣器频率，单位Hz。 */
  uint8_t rgb[3]; /**< 最后成功RGB原始通道，范围0..255。 */
  uint8_t fail_write; /**< 1表示第二路PWM写入失败。 */
  uint8_t guard_arm_write; /**< ARM四路怠速写入过程中，真实armed必须仍为0。 */
  int32_t roll_deg100; /**< 可注入的姿态辅助横滚角，单位degree*100。 */
  unsigned audio_calls; /**< sensor声光平台调用次数。 */
  unsigned writes; /**< PWM平台写入次数。 */
  Px4Lite_State_t control_state; /**< 真实Control发布的模块状态。 */
  uint16_t control_fault; /**< 真实Control发布的故障码。 */
  Px4Lite_MotorOutputs_t motor; /**< 真实Control发布的Motor快照。 */
} s_test;

/** @brief 返回测试平台毫秒，不借用RTOS tick。 */
uint32_t Px4Lite_PlatformGetMs(void) { return s_test.now_ms; }
/** @brief 返回与毫秒同源的微秒时刻。 */
uint32_t Px4Lite_PlatformGetUs(void) { return s_test.now_ms * 1000U; }
/** @brief 硬件启动仅建立安全最小输出。 */
Px4Lite_Result_t Px4Lite_MotorInit(void) { return Px4Lite_MotorDisarmAll(); }
/** @brief 原始PWM由真实Control串行调用；声光owner禁止直接改变电机。 */
Px4Lite_Result_t Px4Lite_MotorWritePulseUs(uint8_t channel, uint16_t pulse_us)
{
  assert(s_test_scheduler && !s_test_sensor_owner);
  assert(channel < PX4LITE_MOTOR_COUNT);
  if (s_test.guard_arm_write && pulse_us == PX4LITE_CNS_IDLE_PULSE_US) {
    assert(Px4Lite_ControlCnsArmed() == 0U);
    if (channel == PX4LITE_MOTOR_COUNT - 1U) { s_test.guard_arm_write = 0U; }
  }
  if (s_test.fail_write && channel == 1U) { return PX4LITE_IO_ERROR; }
  s_test.pwm[channel] = pulse_us;
  if (pulse_us > PX4LITE_CONTROL_ESC_MIN_PULSE_US && !s_test.above_minimum_ms) {
    s_test.above_minimum_ms = s_test.now_ms;
  }
  ++s_test.writes;
  return PX4LITE_OK;
}
/** @brief 停车接口始终可用，使PWM写入失败场景仍可验证后续锁定和撤收。 */
Px4Lite_Result_t Px4Lite_MotorDisarmAll(void)
{
  uint8_t i;
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) { s_test.pwm[i] = PX4LITE_CONTROL_ESC_MIN_PULSE_US; }
  return PX4LITE_OK;
}
/** @brief 物理急停键未按下；命令急停由真实Control执行。 */
uint8_t Px4Lite_ButtonPressed(Px4Lite_ButtonId_t button) { (void)button; return 0U; }
/** @brief 保存真实输出快照，不根据目标伪造实际输出。 */
void Px4Lite_PublishMotor(const Px4Lite_MotorOutputs_t *out) { s_test.motor = *out; }
/** @brief 播放器仅能读取真实Control发布的Motor快照。 */
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out)
{ *out = s_test.motor; return out->header.valid ? PX4LITE_OK : PX4LITE_NOT_READY; }
/** @brief 默认有效12V输入仅用于保留启用电压门禁时的测试能力。 */
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out)
{
  memset(out, 0, sizeof(*out)); out->header.valid = 1U;
  out->header.sample_time_ms = s_test.now_ms; out->voltage_mv = 12000U;
  return PX4LITE_OK;
}
/** @brief 生成新鲜姿态事实，供真实起飞准入和姿态修正使用。 */
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *out)
{
  memset(out, 0, sizeof(*out)); out->header.valid = 1U;
  out->header.sample_time_ms = s_test.now_ms; out->valid_mask = PX4LITE_NAV_VALID_ATTITUDE;
  out->roll_deg100 = s_test.roll_deg100;
  return PX4LITE_OK;
}
/** @brief IMU/Baro在线；Control状态必须来自真实状态发布。 */
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *out)
{
  memset(out, 0, sizeof(*out)); out->module_id = id;
  if (id == PX4LITE_MODULE_CONTROL) {
    out->state = s_test.control_state; out->fault_code = s_test.control_fault; return PX4LITE_OK;
  }
  if (id == PX4LITE_MODULE_IMU || id == PX4LITE_MODULE_BARO) { out->state = PX4LITE_STATE_ONLINE; return PX4LITE_OK; }
  return PX4LITE_NOT_READY;
}
/** @brief 保留真实Control对外的生命周期事实。 */
void Px4Lite_SetExternalModuleState(Px4Lite_ModuleId_t id, Px4Lite_State_t state, uint16_t fault, uint32_t now_ms)
{ assert(id == PX4LITE_MODULE_CONTROL && now_ms == s_test.now_ms); s_test.control_state = state; s_test.control_fault = fault; }
/** @brief 日志出口不参与被测控制和声光决策。 */
uint8_t Px4Lite_LogMessageSeverity(uint16_t message) { (void)message; return 0U; }
/** @brief 日志仅接收真实框架事件，不执行存储I/O。 */
void Px4Lite_LocalMsgLogEvent(uint16_t message, uint32_t now_ms, uint16_t fault, uint8_t source, uint8_t active)
{ (void)message; (void)now_ms; (void)fault; (void)source; (void)active; }

/** @brief 所有蜂鸣器/RGB I/O只允许sensor上下文，且不处于临界区/调度互斥内。 */
static void AudioIo(void)
{ assert(s_test_sensor_owner && !s_test_critical && !s_test_scheduler); ++s_test.audio_calls; }
/** @brief 蜂鸣器初始化只记录平台结果。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void) { AudioIo(); s_test.hz = 0U; return PX4LITE_OK; }
/** @brief 捕获真实声光播放器输出的频率与首次ARM低音时刻。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t hz)
{
  AudioIo(); s_test.hz = hz;
  if (hz == PX4LITE_CNS_LOW_TONE_HZ && !s_test.low_started_ms) { s_test.low_started_ms = s_test.now_ms; }
  return PX4LITE_OK;
}
/** @brief RGB接口只在sensor初始化。 */
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void) { AudioIo(); return PX4LITE_OK; }
/** @brief RGB撤收同样不可来自Control或Comm。 */
void Px4Lite_PlatformRgbDeInit(void) { AudioIo(); }
/** @brief 保存真实请求颜色，不伪造DMA完成。 */
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t r, uint8_t g, uint8_t b)
{ AudioIo(); s_test.rgb[0] = r; s_test.rgb[1] = g; s_test.rgb[2] = b; return PX4LITE_OK; }
/** @brief RGB服务使用与控制同源的真实毫秒。 */
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t now_ms)
{ AudioIo(); assert(now_ms == s_test.now_ms); return PX4LITE_OK; }

#include "../../Framework/Src/px4lite_control.c"
#include "../../Framework/Src/px4lite_startup_tone.c"

/** @brief 推进真实sensor一次，并确认蜂鸣器播放不能直接改电机PWM。 */
static void Sensor(void)
{
  uint16_t before[PX4LITE_MOTOR_COUNT];
  memcpy(before, s_test.pwm, sizeof(before)); s_test_sensor_owner = 1U;
  Px4Lite_StartupToneRun(s_test.now_ms * 1000U); s_test_sensor_owner = 0U;
  assert(!memcmp(before, s_test.pwm, sizeof(before)) && !s_test_critical && !s_test_scheduler);
}
/** @brief 按sensor10ms/Control20ms节拍交错推进，避免引入任务失步保护。 */
static void Advance(uint32_t duration_ms)
{
  uint32_t elapsed;
  assert(duration_ms % 10U == 0U);
  for (elapsed = 0U; elapsed < duration_ms; elapsed += 10U) {
    s_test.now_ms += 10U;
    if (s_test.now_ms % PX4LITE_CONTROL_PERIOD_MS == 0U) { Px4Lite_ControlRun(s_test.now_ms); }
    Sensor();
  }
}
/** @brief 校验四路平台确认的最小输出，而非只检查锁标志。 */
static void Minimum(void)
{ uint8_t i; for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) { assert(s_test.pwm[i] == PX4LITE_CONTROL_ESC_MIN_PULSE_US); } }
/** @brief 每个场景启动真实模块；待原启动音结束后才发CNS请求。 */
static void Fresh(void)
{
  memset(&s_test, 0, sizeof(s_test)); s_test.now_ms = 1000U;
  assert(Px4Lite_ControlModuleInit() == PX4LITE_OK);
  s_test_sensor_owner = 1U; Px4Lite_StartupToneInit(s_test.now_ms * 1000U); s_test_sensor_owner = 0U;
  Advance(2800U); assert(!Px4Lite_ControlCnsArmed()); Minimum();
  s_test.low_started_ms = 0U; s_test.above_minimum_ms = 0U;
}
/** @brief 仅调用公开受理边界；请求本身不能同步驱动声光。 */
static uint32_t Request(Px4Lite_CnsAction_t action, uint8_t motor, uint16_t pwm)
{
  uint8_t reason = 255U; uint32_t token = 0U; unsigned before = s_test.audio_calls;
  assert(Px4Lite_ControlCnsRequest(action, motor, pwm, &reason, &token) == PX4LITE_OK);
  assert(reason == 0U && token && before == s_test.audio_calls);
  assert(!s_test_critical && !s_test_scheduler);
  return token;
}
/** @brief 使用真实ARM流程建立怠速，并对照先声音后PWM的外部时序。 */
static void ArmReady(void)
{
  uint32_t token = Request(PX4LITE_CNS_ARM, 0U, 0U); uint8_t i;
  Px4Lite_ControlCnsAckSent(token); s_test.guard_arm_write = 1U;
  Advance(900U); assert(!Px4Lite_ControlCnsArmed()); Minimum();
  Advance(300U); s_test.guard_arm_write = 0U;
  assert(Px4Lite_ControlCnsArmed() && s_test.low_started_ms && s_test.above_minimum_ms);
  assert(s_test.above_minimum_ms - s_test.low_started_ms >= PX4LITE_CNS_ARM_TONE_MS);
  for (i = 0U; i < PX4LITE_MOTOR_COUNT; ++i) { assert(s_test.pwm[i] == PX4LITE_CNS_IDLE_PULSE_US); }
}
/** @brief 未送出受理ACK时不执行；过2s的迟到ACK不能重新启动ARM。 */
static void TestAckAdmissionAndExpiry(void)
{
  uint32_t token;
  Fresh(); token = Request(PX4LITE_CNS_ARM, 0U, 0U);
  Advance(500U); assert(!Px4Lite_ControlCnsArmed() && !s_test.hz); Minimum();
  Px4Lite_ControlCnsAckSent(token); Advance(900U); assert(!Px4Lite_ControlCnsArmed()); Minimum();
  Advance(300U); assert(Px4Lite_ControlCnsArmed());
  Fresh(); token = Request(PX4LITE_CNS_ARM, 0U, 0U);
  Advance(PX4LITE_CNS_ACK_TIMEOUT_MS + 100U); Px4Lite_ControlCnsAckSent(token);
  Advance(1500U); assert(!Px4Lite_ControlCnsArmed() && !s_test.hz); Minimum();
  /* 1979ms开始的同步发送若被额外抢占，完成回调来到2000ms也不能再启动。 */
  Fresh(); token = Request(PX4LITE_CNS_ARM, 0U, 0U); Advance(1960U);
  s_test.now_ms += 19U; s_test.now_ms += 21U; Px4Lite_ControlCnsAckSent(token);
  Advance(1500U); assert(!Px4Lite_ControlCnsArmed() && !s_test.hz); Minimum();
}
/** @brief 四路idle写入有一路失败时，不能仅根据声音结束就宣布已解锁。 */
static void TestArmedAfterActualIdleOnly(void)
{
  uint32_t token;
  Fresh(); ArmReady();
  Fresh(); token = Request(PX4LITE_CNS_ARM, 0U, 0U); Px4Lite_ControlCnsAckSent(token);
  Advance(900U); s_test.fail_write = 1U; Advance(400U);
  assert(!Px4Lite_ControlCnsArmed()); Minimum();
  s_test.fail_write = 0U; Advance(1500U); assert(!Px4Lite_ControlCnsArmed()); Minimum();
}
/** @brief 单路调速先完成成功声光，再变更指定PWM；其余三路保持实际输出。 */
static void TestSingleMotorAfterSuccessTone(void)
{
  static const uint16_t base[4] = {1234U, 1357U, 1478U, 1591U};
  uint16_t old[4]; uint32_t token; uint8_t i;
  Fresh(); ArmReady(); s_test.roll_deg100 = 1000;
  assert(Px4Lite_ControlSetMotorPulses(base) == PX4LITE_OK);
  assert(Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST) == PX4LITE_OK);
  Advance(40U); memcpy(old, s_test.pwm, sizeof(old));
  token = Request(PX4LITE_CNS_SET_MOTOR_PWM, 1U, 1755U); Px4Lite_ControlCnsAckSent(token);
  Advance(100U); assert(!memcmp(old, s_test.pwm, sizeof(old)) && Px4Lite_ControlCnsArmed());
  Advance(900U); assert(s_test.pwm[1] == 1755U && Px4Lite_ControlCnsArmed());
  for (i = 0U; i < 4U; ++i) { if (i != 1U) { assert(s_test.pwm[i] == old[i]); } }
  Advance(200U);
  for (i = 0U; i < 4U; ++i) { if (i != 1U) { assert(s_test.pwm[i] == old[i]); } }
}
/** @brief 急停抢占尚未完成的解锁和普通提示；旧ARM令牌不允许复活。 */
static void TestEmergencyPreemptsArmingAndFeedback(void)
{
  uint32_t arm, stop; Px4Lite_StartupToneStatus_t status;
  Fresh(); arm = Request(PX4LITE_CNS_ARM, 0U, 0U); Px4Lite_ControlCnsAckSent(arm); Advance(300U);
  assert(!Px4Lite_ControlCnsArmed() && s_test.hz == PX4LITE_CNS_LOW_TONE_HZ);
  Px4Lite_StartupToneNotifyRpiResult(1U); Px4Lite_StartupToneNotifyModuleConnection(1U);
  stop = Request(PX4LITE_CNS_EMERGENCY_STOP, 0U, 0U); Minimum(); assert(!Px4Lite_ControlCnsArmed());
  Px4Lite_ControlCnsAckSent(stop); Advance(40U); assert(s_test.hz == PX4LITE_CNS_HIGH_TONE_HZ);
  Px4Lite_ControlCnsAckSent(arm); Advance(2000U); Minimum(); assert(!Px4Lite_ControlCnsArmed());
  assert(Px4Lite_StartupToneCopyStatus(&status) == PX4LITE_OK);
  assert(!status.feedback_active && !status.feedback_pending && !status.music_running && !s_test.hz);
}
/** @brief 上锁立即撤收正在起飞的电机；迟到旧ACK和旧蜂鸣完成不能恢复提速。 */
static void TestDisarmInterruptsTakeoff(void)
{
  uint32_t takeoff, lock;
  Fresh(); ArmReady(); takeoff = Request(PX4LITE_CNS_TAKEOFF, 0U, 0U); Px4Lite_ControlCnsAckSent(takeoff);
  Advance(200U); assert(s_test.pwm[0] == PX4LITE_CNS_IDLE_PULSE_US);
  Advance(2200U); assert(s_test.pwm[0] > PX4LITE_CNS_IDLE_PULSE_US && Px4Lite_ControlCnsArmed());
  lock = Request(PX4LITE_CNS_DISARM, 0U, 0U); Minimum(); assert(!Px4Lite_ControlCnsArmed());
  Px4Lite_ControlCnsAckSent(takeoff); Px4Lite_ControlCnsAckSent(lock); Advance(9000U);
  Minimum(); assert(!Px4Lite_ControlCnsArmed());
}
/** @brief 降落伴随间歇高音，实际到idle后结束；全程和终点均保持真实解锁。 */
static void TestLandingEndsAtArmedIdle(void)
{
  uint32_t token; uint8_t i, saw_high = 0U, saw_silence = 0U;
  Fresh(); ArmReady(); token = Request(PX4LITE_CNS_TAKEOFF, 0U, 0U); Px4Lite_ControlCnsAckSent(token);
  Advance(7500U); assert(s_test.pwm[0] > PX4LITE_CNS_IDLE_PULSE_US && Px4Lite_ControlCnsArmed());
  token = Request(PX4LITE_CNS_LAND, 0U, 0U); Px4Lite_ControlCnsAckSent(token);
  for (i = 0U; i < 120U; ++i) {
    Advance(10U); assert(Px4Lite_ControlCnsArmed());
    if (s_test.hz == PX4LITE_CNS_HIGH_TONE_HZ) { saw_high = 1U; }
    if (!s_test.hz) { saw_silence = 1U; }
  }
  assert(saw_high && saw_silence); Advance(7500U);
  for (i = 0U; i < 4U; ++i) { assert(s_test.pwm[i] == PX4LITE_CNS_IDLE_PULSE_US); }
  assert(Px4Lite_ControlCnsArmed() && Px4Lite_StartupToneCnsResult(token) != PX4LITE_BUSY);
}

/** @brief 执行真实组件的动作时序、安全中断和输出事实验证。 */
int main(void)
{
  TestAckAdmissionAndExpiry(); TestArmedAfterActualIdleOnly(); TestSingleMotorAfterSuccessTone();
  TestEmergencyPreemptsArmingAndFeedback(); TestDisarmInterruptsTakeoff(); TestLandingEndsAtArmedIdle();
  assert(!s_test_sensor_owner && !s_test_critical && !s_test_scheduler);
  puts("CNS V0.1 integration: real Control+Tone / ACK gate+expiry / actual-idle armed / cue-before-PWM / unchanged other channels / emergency+lock cancellation / armed-idle landing PASS");
  return 0;
}
