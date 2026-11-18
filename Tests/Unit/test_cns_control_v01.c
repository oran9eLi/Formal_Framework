/**
 * @file test_cns_control_v01.c
 * @brief CNS V0.1 真实控制状态机边界回归；平台输出与声光完成事实为受控桩。
 * @details 不连接硬件、不模拟电机转速；覆盖 ACK、锁、抢占、输出失败及精确单路保持。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_now;
static unsigned s_sched, s_critical;
#define vTaskSuspendAll() (++s_sched)
#define xTaskResumeAll() (--s_sched)
#define taskENTER_CRITICAL() (++s_critical)
#define taskEXIT_CRITICAL() (--s_critical)
#include "../../Framework/Src/px4lite_control.c"

static uint16_t s_pwm[4];
static Px4Lite_MotorOutputs_t s_motor;
static uint8_t s_key, s_write_fail, s_disarm_fail, s_nav_ready = 1U;
static int32_t s_nav_roll;
static uint8_t s_tone_action, s_tone_done, s_tone_error;
static uint32_t s_tone_token;
static unsigned s_cancel_all, s_start_count;
static uint8_t s_watch_armed_during_idle;

uint32_t Px4Lite_PlatformGetMs(void) { return s_now; }
Px4Lite_Result_t Px4Lite_MotorDisarmAll(void)
{ unsigned i;if(s_disarm_fail) { return PX4LITE_IO_ERROR; }for(i=0U;i<4U;++i) { s_pwm[i]=1000U; }return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_MotorInit(void) { return Px4Lite_MotorDisarmAll(); }
Px4Lite_Result_t Px4Lite_MotorWritePulseUs(uint8_t channel, uint16_t pulse)
{
  assert(s_sched);
  if (s_watch_armed_during_idle && pulse==PX4LITE_CNS_IDLE_PULSE_US) { assert(!Px4Lite_ControlCnsArmed()); }
  if (s_write_fail && channel==1U) { return PX4LITE_IO_ERROR; }
  s_pwm[channel]=pulse; return PX4LITE_OK;
}
uint8_t Px4Lite_ButtonPressed(Px4Lite_ButtonId_t button)
{ return button==PX4LITE_BUTTON_KEY1?s_key:0U; }
void Px4Lite_PublishMotor(const Px4Lite_MotorOutputs_t *motor) { s_motor=*motor; }
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *motor)
{ *motor=s_motor; return motor->header.valid?PX4LITE_OK:PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out)
{ memset(out,0,sizeof(*out));out->header.valid=1U;out->header.sample_time_ms=s_now;out->voltage_mv=12000U;return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *nav)
{
  if(!s_nav_ready) { return PX4LITE_NOT_READY; }
  memset(nav,0,sizeof(*nav));nav->header.valid=1U;nav->header.sample_time_ms=s_now;
  nav->valid_mask=PX4LITE_NAV_VALID_ATTITUDE;nav->roll_deg100=s_nav_roll;return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id,Px4Lite_ModuleStatus_t *out)
{ (void)id;memset(out,0,sizeof(*out));out->state=PX4LITE_STATE_ONLINE;return PX4LITE_OK; }
void Px4Lite_SetExternalModuleState(Px4Lite_ModuleId_t id,Px4Lite_State_t state,uint16_t fault,uint32_t now)
{ (void)id;(void)state;(void)fault;(void)now; }
uint8_t Px4Lite_LogMessageSeverity(uint16_t message) { (void)message;return 0U; }
void Px4Lite_LocalMsgLogEvent(uint16_t message,uint32_t now,uint16_t fault,uint8_t source,uint8_t active)
{ (void)message;(void)now;(void)fault;(void)source;(void)active; }
void Px4Lite_StartupToneNotifyRpiResult(uint8_t success) { (void)success; }
void Px4Lite_StartupToneCnsStart(uint8_t action,uint32_t token)
{ s_tone_action=action;s_tone_token=token;s_tone_done=0U;s_tone_error=0U;++s_start_count; }
void Px4Lite_StartupToneCnsCancel(uint8_t all)
{ s_tone_token=0U;s_tone_action=0U;if(all) { ++s_cancel_all; } }
Px4Lite_Result_t Px4Lite_StartupToneCnsResult(uint32_t token)
{
  if(!token || token!=s_tone_token) { return PX4LITE_IDLE; }
  if(s_tone_error) { return PX4LITE_IO_ERROR; }
  return s_tone_done?PX4LITE_OK:PX4LITE_BUSY;
}

/** @brief 按真实控制周期推进，避免将未调度故障混入正常动作。 */
static void Advance(uint32_t duration)
{ uint32_t i;for(i=0U;i<duration;i+=PX4LITE_CONTROL_PERIOD_MS) { s_now+=PX4LITE_CONTROL_PERIOD_MS;Px4Lite_ControlRun(s_now); } }
/** @brief 复位实际控制模块；令牌序列按生产恢复规则不清零。 */
static void Reset(void)
{
  s_now=1000U;s_key=0U;s_write_fail=0U;s_disarm_fail=0U;s_nav_ready=1U;s_nav_roll=0;
  s_watch_armed_during_idle=0U;s_tone_done=0U;s_tone_error=0U;
  assert(Px4Lite_ControlModuleInit()==PX4LITE_OK);Advance(20U);assert(!Px4Lite_ControlCnsArmed());
}
/** @brief 比较四路已确认的实际平台输出。 */
static void All(uint16_t pulse)
{ unsigned i;for(i=0U;i<4U;++i) { assert(s_pwm[i]==pulse); } }
/** @brief 请求实际状态机，检查拒绝原因和令牌的独立语义。 */
#if PX4LITE_ENABLE_CNS_CONTROL
static uint32_t Request(Px4Lite_CnsAction_t action,uint8_t channel,uint16_t pulse,Px4Lite_Result_t expected,uint8_t why)
{
  uint32_t token=99U;uint8_t reason=99U;
  Px4Lite_Result_t result=Px4Lite_ControlCnsRequest(action,channel,pulse,&reason,&token);
  if(result!=expected || reason!=why) {
    fprintf(stderr,"action=%u result=%d expected=%d reason=%u expected_reason=%u locked=%u phase=%u auto=%u now=%lu\n",
            (unsigned)action,(int)result,(int)expected,(unsigned)reason,(unsigned)why,
            (unsigned)s_manual_locked,(unsigned)s_cns.phase,(unsigned)s_auto_profile,(unsigned long)s_now);
  }
  assert(result==expected);
  assert(reason==why);assert((expected==PX4LITE_OK)==(token!=0U));return token;
}
/** @brief 完成真实受理/ACK/声音后转换，解锁只在全部怠速写入成功后成立。 */
static void Arm(void)
{
  uint32_t token=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);
  Px4Lite_ControlCnsAckSent(token);Advance(20U);assert(!Px4Lite_ControlCnsArmed());All(1000U);
  s_tone_done=1U;
  /* 只观察首次四路怠速事务，后续正常刷新时 armed 应已为 1。 */
  s_watch_armed_during_idle=1U;
  vTaskSuspendAll();
  ControlCnsService(s_now);
  (void)xTaskResumeAll();
  s_watch_armed_during_idle=0U;
  Advance(20U);assert(Px4Lite_ControlCnsArmed());All(PX4LITE_CNS_IDLE_PULSE_US);
}

/** @brief ACK 不到达、过期、恢复和晚到令牌都不能启动声音或电机。 */
static void TestAckAndLock(void)
{
  uint32_t token, newer;
  unsigned starts;
  Reset();
  (void)Request(PX4LITE_CNS_TAKEOFF,0U,0U,PX4LITE_NOT_READY,1U);
  (void)Request(PX4LITE_CNS_LAND,0U,0U,PX4LITE_NOT_READY,1U);
  (void)Request(PX4LITE_CNS_DISARM,0U,0U,PX4LITE_NOT_READY,1U);
  (void)Request(PX4LITE_CNS_SET_MOTOR_PWM,4U,1300U,PX4LITE_INVALID_PARAM,3U);
  starts=s_start_count;
  token=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);
  Advance(1000U);All(1000U);assert(s_start_count==starts);
  (void)Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_BUSY,2U);
  Advance(PX4LITE_CNS_ACK_TIMEOUT_MS);Px4Lite_ControlCnsAckSent(token);Advance(100U);
  assert(s_start_count==starts && !Px4Lite_ControlCnsArmed());All(1000U);
  newer=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);assert(newer!=token);
  assert(Px4Lite_ControlRecover()==PX4LITE_OK);Px4Lite_ControlCnsAckSent(newer);Advance(100U);All(1000U);
  Arm();All(1200U);
}

/** @brief ARM 无锁状态前提；重复解锁在ACK后先停车锁定，声音完成后才重新进入怠速。 */
static void TestRearmFromUnlocked(void)
{
  uint16_t previous[4]={1400U,1500U,1600U,1700U};
  uint32_t token;unsigned i, starts;
  Reset();Arm();assert(Px4Lite_ControlSetMotorPulses(previous)==PX4LITE_OK);Advance(20U);
  token=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);
  Advance(100U);assert(Px4Lite_ControlCnsArmed());
  for(i=0U;i<4U;++i) { assert(s_pwm[i]==previous[i]); }
  (void)Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_BUSY,2U);
  Px4Lite_ControlCnsAckSent(token);Advance(20U);
  assert(!Px4Lite_ControlCnsArmed() && s_tone_action==PX4LITE_CNS_ARM);All(1000U);
  Advance(900U);assert(!Px4Lite_ControlCnsArmed());All(1000U);
  Advance(100U);s_tone_done=1U;Advance(20U);
  assert(Px4Lite_ControlCnsArmed());All(1200U);
  token=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);starts=s_start_count;
  s_disarm_fail=1U;Px4Lite_ControlCnsAckSent(token);Advance(20U);
  assert(!Px4Lite_ControlCnsArmed() && s_start_count==starts && s_cns.phase==CONTROL_CNS_NONE);
  All(1000U);s_disarm_fail=0U;Advance(100U);assert(!Px4Lite_ControlCnsArmed());All(1000U);
}

/** @brief 2秒约束的是ACK发送完成时刻，而不是下一次Control调度；截止点ACK仍拒绝。 */
static void TestAckDeadlineBoundary(void)
{
  uint32_t token, accepted;
  unsigned starts, extra;
  for(extra=0U;extra<=10U;extra+=10U) {
    Reset();
    token=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);accepted=s_now;starts=s_start_count;
    Advance(PX4LITE_CNS_ACK_TIMEOUT_MS-20U);
    s_now=accepted+PX4LITE_CNS_ACK_TIMEOUT_MS-1U;
    Px4Lite_ControlCnsAckSent(token);
    s_now=accepted+PX4LITE_CNS_ACK_TIMEOUT_MS+extra;
    Px4Lite_ControlRun(s_now);
    assert(s_start_count==starts+1U && s_tone_token==token);
    assert(!Px4Lite_ControlCnsArmed());All(1000U);
  }
  Reset();token=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);accepted=s_now;starts=s_start_count;
  Advance(PX4LITE_CNS_ACK_TIMEOUT_MS-20U);
  s_now=accepted+PX4LITE_CNS_ACK_TIMEOUT_MS;
  Px4Lite_ControlCnsAckSent(token);Px4Lite_ControlRun(s_now);
  assert(s_start_count==starts && s_cns.phase==CONTROL_CNS_NONE);
  assert(!Px4Lite_ControlCnsArmed());All(1000U);
}

/** @brief 任一通道写入失败不能提前发布 armed，也不能残留半次怠速输出。 */
static void TestArmOutputFailure(void)
{
  uint32_t token;
  Reset();token=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);
  Px4Lite_ControlCnsAckSent(token);Advance(20U);s_tone_done=1U;s_write_fail=1U;
  Advance(20U);assert(!Px4Lite_ControlCnsArmed());All(1000U);
  s_write_fail=0U;Px4Lite_ControlCnsAckSent(token);Advance(100U);All(1000U);
}

/** @brief 单通道精确调速保持另外三路的实际值，姿态变化和重复刷新不改变它们。 */
static void TestSinglePwmAndBusy(void)
{
  uint16_t initial[4]={1231U,1342U,1453U,1564U},held[4];
  uint32_t token;unsigned i;
  Reset();Arm();assert(Px4Lite_ControlSetMotorPulses(initial)==PX4LITE_OK);
  assert(Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST)==PX4LITE_OK);
  s_nav_roll=1000;Advance(20U);memcpy(held,s_pwm,sizeof(held));
  token=Request(PX4LITE_CNS_SET_MOTOR_PWM,1U,1755U,PX4LITE_OK,0U);
  Px4Lite_ControlCnsAckSent(token);s_nav_roll=-2000;Advance(80U);
  for(i=0U;i<4U;++i) { assert(s_pwm[i]==held[i]); }
  (void)Request(PX4LITE_CNS_TAKEOFF,0U,0U,PX4LITE_BUSY,2U);
  s_tone_done=1U;Advance(20U);held[1]=1755U;
  for(i=0U;i<4U;++i) { assert(s_pwm[i]==held[i]); }
  s_nav_roll=3000;Advance(100U);
  for(i=0U;i<4U;++i) { assert(s_pwm[i]==held[i]); }
  assert(Px4Lite_ControlCnsArmed());
  /* 等待 ACK 没有改目标，过期不得丢掉前一动作已经建立的精确输出保持。 */
  token=Request(PX4LITE_CNS_TAKEOFF,0U,0U,PX4LITE_OK,0U);
  s_nav_roll=-3000;Advance(PX4LITE_CNS_ACK_TIMEOUT_MS+40U);
  assert(s_cns.phase==CONTROL_CNS_NONE && s_cns_exact_hold);
  for(i=0U;i<4U;++i) { assert(s_pwm[i]==held[i]); }
  token=Request(PX4LITE_CNS_TAKEOFF,0U,0U,PX4LITE_OK,0U);
  Advance(PX4LITE_CNS_ACK_TIMEOUT_MS-20U);s_now+=20U;
  Px4Lite_ControlCnsAckSent(token);Px4Lite_ControlRun(s_now);
  assert(s_cns.phase==CONTROL_CNS_NONE && s_cns_exact_hold);
  for(i=0U;i<4U;++i) { assert(s_pwm[i]==held[i]); }
}

/** @brief 上锁立即打断起飞；降落原曲线保留但 CNS 终点为怠速，锁保持解锁。 */
static void TestRampsAndStops(void)
{
  uint32_t token, stop;
  uint16_t pulse[4]={1600U,1600U,1600U,1600U};
  Reset();Arm();s_nav_ready=0U;Advance(PX4LITE_IMU_MAX_AGE_MS+40U);
  (void)Request(PX4LITE_CNS_TAKEOFF,0U,0U,PX4LITE_IO_ERROR,5U);
  s_nav_ready=1U;token=Request(PX4LITE_CNS_TAKEOFF,0U,0U,PX4LITE_OK,0U);
  Px4Lite_ControlCnsAckSent(token);Advance(100U);All(1200U);
  s_tone_done=1U;Advance(1000U);assert(s_pwm[0]>1200U);
  stop=Request(PX4LITE_CNS_DISARM,0U,0U,PX4LITE_OK,0U);All(1000U);assert(!Px4Lite_ControlCnsArmed());
  Px4Lite_ControlCnsAckSent(token);Advance(1000U);All(1000U);
  Px4Lite_ControlCnsAckSent(stop);Advance(20U);s_tone_done=1U;Advance(20U);Arm();
  assert(Px4Lite_ControlSetMotorPulses(pulse)==PX4LITE_OK);Advance(20U);
  token=Request(PX4LITE_CNS_LAND,0U,0U,PX4LITE_OK,0U);Px4Lite_ControlCnsAckSent(token);
  Advance(20U);assert(s_tone_action==PX4LITE_CNS_LAND);
  assert(Px4Lite_ControlStartAutoTakeoff()==PX4LITE_BUSY);
  assert(Px4Lite_ControlStartAutoLanding()==PX4LITE_BUSY);
  assert(s_cns.token==token && s_cns.phase==CONTROL_CNS_RAMP);
  Advance(8100U);All(1200U);assert(Px4Lite_ControlCnsArmed());assert(!s_tone_token);
}

/** @brief 急停取消解锁序列和全部声光，屏幕锁、按键、维护和任务失步均不可留下可复活请求。 */
static void TestPreemption(void)
{
  uint32_t old, stop;unsigned cancels;
  Reset();old=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);Px4Lite_ControlCnsAckSent(old);Advance(20U);
  cancels=s_cancel_all;stop=Request(PX4LITE_CNS_EMERGENCY_STOP,0U,0U,PX4LITE_OK,0U);
  assert(s_cancel_all==cancels+1U);All(1000U);assert(!Px4Lite_ControlCnsArmed());
  Px4Lite_ControlCnsAckSent(old);s_tone_done=1U;Advance(100U);All(1000U);
  Px4Lite_ControlCnsAckSent(stop);Advance(20U);s_tone_done=1U;Advance(20U);
  old=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);Px4Lite_ControlCnsAckSent(old);Advance(20U);
  assert(Px4Lite_ControlSetMotorLocked(1U)==PX4LITE_OK);s_tone_done=1U;Advance(100U);All(1000U);
  old=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);
  assert(Px4Lite_ControlBeginConfigMaintenance()==PX4LITE_OK);Px4Lite_ControlCnsAckSent(old);Advance(100U);All(1000U);
  Px4Lite_ControlEndConfigMaintenance();old=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);
  Px4Lite_ControlCnsAckSent(old);Advance(20U);s_key=1U;Advance(40U);s_tone_done=1U;Advance(20U);All(1000U);
  s_key=0U;Advance(40U);old=Request(PX4LITE_CNS_ARM,0U,0U,PX4LITE_OK,0U);Px4Lite_ControlCnsAckSent(old);Advance(20U);
  s_now+=PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS+1U;Px4Lite_ControlRun(s_now);s_tone_done=1U;Advance(40U);All(1000U);
  assert(!Px4Lite_ControlCnsArmed());
}

int main(void)
{
  TestAckAndLock();TestRearmFromUnlocked();TestAckDeadlineBoundary();TestArmOutputFailure();TestSinglePwmAndBusy();TestRampsAndStops();TestPreemption();
  assert(!s_sched && !s_critical);puts("CNS V0.1 control: PASS (ACK, lock, output, ramps, preemption)");return 0;
}
#else
/** @brief 关闭 CNS 后真实旧人工锁路径可用，新接口明确拒绝且不启动专属声光。 */
int main(void)
{
  uint8_t reason=99U;
  uint32_t token=99U;
  Reset();
  assert(Px4Lite_ControlCnsRequest(PX4LITE_CNS_ARM,0U,0U,&reason,&token)==PX4LITE_NOT_READY);
  assert(reason==4U && token==0U);
  Px4Lite_ControlCnsAckSent(1U);Advance(100U);All(1000U);assert(!s_start_count);
  assert(Px4Lite_ControlSetMotorLocked(0U)==PX4LITE_OK);
  assert(Px4Lite_ControlCnsArmed());Advance(PX4LITE_CONTROL_ESC_ARM_TIME_MS);All(1000U);
  assert(!s_sched && !s_critical);puts("CNS disabled: PASS (reject, no-op ACK, original manual lock)");return 0;
}
#endif
