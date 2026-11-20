/**
 * @file test_rpi_result_feedback_control.c
 * @brief 真实Command/Control的树莓派执行结果提示验证。
 * @details 复用原人工锁平台夹具；不播放声光、不伪造电机物理转速，只核验真实PWM API结果。
 */
#include <stdint.h>
static uint8_t s_feedback_events[128];
static unsigned s_feedback_count;
static uint8_t s_require_feedback_lock;
static uint8_t s_resume_probe;
static uint8_t s_resume_expected_mode;
static int32_t s_resume_expected_roll,s_resume_expected_pitch,s_resume_expected_yaw;
static uint32_t s_cns_ack_token;
static int TestRpiResultResumeAll(void);
#define TEST_RPI_RESULT_RESUME_ALL 1
#define TEST_RPI_RESULT_NOTIFY 1
#define main RpiFeedbackLegacyManualLockMain
#include "test_motor_manual_lock.c"
#undef main

/** @brief 捕获实际生产通知出口；不替换Command或Control决策。 */
void Px4Lite_StartupToneNotifyRpiResult(uint8_t succeeded)
{
  if (s_require_feedback_lock) { assert(s_test_scheduler_depth != 0U); }
  assert(s_feedback_count < sizeof(s_feedback_events));
  s_feedback_events[s_feedback_count++] = succeeded;
}
/** @brief 丢弃场景之间已真实终结的旧观察，随后清测试计数。 */
static void Fresh(uint8_t ready)
{
  TestReset();
  if (ready) { TestUnlockReady(); }
  s_feedback_count = 0U;
  memset(s_feedback_events, 0xFF, sizeof(s_feedback_events));
}
/** @brief 按真实RPi地址发送，并维持原COMMAND_ACK语义。 */
static void Rpi(uint16_t command,float a,float b,float c,float d,uint8_t ack)
{
  assert(Px4Lite_CommandHandleMavlinkLong(command,42U,7U,1U,193U,a,b,c,d,
    PX4LITE_MAVLINK_LINK_RPI,s_test_now)==PX4LITE_OK);
  assert(s_test_ack==ack && s_test_scheduler_depth==0U && s_test_critical_depth==0U);
}
/** @brief 仅替代旧夹具未合编的 UART6 发送队列和声光硬件。 */
uint8_t Px4Lite_MavlinkCnsAckAvailable(void) { return 1U; }
Px4Lite_Result_t Px4Lite_MavlinkQueueCnsAck(uint16_t command, uint8_t result,
    uint8_t reason, uint8_t target, uint32_t token, uint32_t now_ms)
{
  (void)command;
  assert(reason == 0U && target == 1U && now_ms == s_test_now);
  s_test_ack = result;
  s_cns_ack_token = token;
  return PX4LITE_OK;
}
void Px4Lite_StartupToneCnsStart(uint8_t action, uint32_t token)
{ assert(action == PX4LITE_CNS_LAND && token != 0U); }

/** @brief 经真实 UART6 Command V0.1 入口发送和取得应答，ACK 仍由测试按时提交。 */
static uint32_t RpiV01(uint16_t command, float mode, float roll, float pitch, float yaw,
                       uint8_t expected_ack)
{
  const float params[7] = {mode, roll, pitch, yaw, 0.0f, 0.0f, 0.0f};
  s_test_ack = 0xFFU;
  s_cns_ack_token = 0U;
  assert(Px4Lite_CommandHandleRpiLongV01(command, 1U, PX4LITE_RPI_RESULT_PI_COMPONENT_ID,
    1U, PX4LITE_RPI_MAVLINK_COMPONENT_ID, 0U, params, s_test_now) == PX4LITE_OK);
  if (s_test_ack != expected_ack) {
    fprintf(stderr, "UART6 command %u ACK %u, expected %u\n",
            (unsigned)command, (unsigned)s_test_ack, (unsigned)expected_ack);
  }
  assert(s_test_ack == expected_ack && s_test_scheduler_depth == 0U && s_test_critical_depth == 0U);
  return s_cns_ack_token;
}
/** @brief 在调度恢复点模拟新RPi抢占；恢复之前原配置必须已完整提交。 */
static int TestRpiResultResumeAll(void)
{
  assert(s_test_scheduler_depth);
  --s_test_scheduler_depth;
  if (!s_test_scheduler_depth && s_resume_probe) {
    s_resume_probe = 0U;
    assert((uint8_t)s_control_mode == s_resume_expected_mode);
    assert(s_target_roll_deg100 == s_resume_expected_roll && s_target_pitch_deg100 == s_resume_expected_pitch && s_target_yaw_deg100 == s_resume_expected_yaw);
    Rpi(31011U,25,25,25,25,MAV_RESULT_ACCEPTED);
  }
  return (int)s_test_scheduler_depth;
}
/** @brief 关闭结果提示时所有真实控制流程仍运行，只要求通知出口保持静默。 */
static void Expect(const char *pattern)
{
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  unsigned i,length=(unsigned)strlen(pattern);
  assert(s_feedback_count==length);
  for(i=0U;i<length;++i) { assert(s_feedback_events[i]==(uint8_t)(pattern[i]-'0')); }
#else
  (void)pattern;assert(s_feedback_count==0U);
#endif
}
/** @brief 拒绝/不支持/非法参数低音；错目标、LoRa及本地控制不产生自己的RPi提示。 */
static void TestClassification(void)
{
  Fresh(0U);
  Rpi(31011U,20,20,20,20,MAV_RESULT_TEMPORARILY_REJECTED);Expect("0");
  Rpi(31011U,-1,20,20,20,MAV_RESULT_DENIED);Expect("00");
  Rpi(60000U,0,0,0,0,MAV_RESULT_UNSUPPORTED);Expect("000");
  assert(Px4Lite_CommandHandleMavlinkLong(31012U,42U,7U,2U,193U,0,0,0,0,PX4LITE_MAVLINK_LINK_RPI,s_test_now)==PX4LITE_IDLE);
  Expect("000");
  TestCommand(31012U,0,0,0,0,MAV_RESULT_ACCEPTED);Expect("000");
  Rpi(31012U,0,1,2,3,MAV_RESULT_ACCEPTED);Expect("0001");
  s_test_disarm_result=PX4LITE_IO_ERROR;
  Rpi(31090U,0,0,0,0,MAV_RESULT_FAILED);Expect("00010");
  s_test_disarm_result=PX4LITE_OK;
  Rpi(31090U,0,0,0,0,MAV_RESULT_ACCEPTED);Expect("000101");
}
/** @brief 百分比/精确脉宽必须等四路PWM成功，不在setter/ACK接受时高音。 */
static void TestRealManualOutputs(void)
{
  Fresh(1U);
  Rpi(31011U,20,30,40,50,MAV_RESULT_ACCEPTED);Expect("");
  TestAdvance(20U);Expect("1");
  assert(s_test_pwm[0]==1200U && s_test_pwm[3]==1500U);
  TestAdvance(80U);Expect("1");
  Rpi(31013U,1100,1234,1567,1800,MAV_RESULT_ACCEPTED);Expect("1");
  TestAdvance(20U);Expect("11");
  assert(s_test_pwm[1]==1234U && s_test_pwm[2]==1567U);
  Rpi(31011U,25,25,25,25,MAV_RESULT_ACCEPTED);s_test_write_fail=1U;
  TestAdvance(20U);Expect("110");assert(s_manual_locked);
  s_test_write_fail=0U;TestAdvance(100U);Expect("110");
}
/** @brief 预备期非零等候；锁/预备期合法全零在真实四路最小写入后成功。 */
static void TestPreparingAndZero(void)
{
  Fresh(0U);Rpi(31011U,0,0,0,0,MAV_RESULT_ACCEPTED);Expect("");
  TestAdvance(20U);Expect("1");TestMinimum();
  Fresh(0U);assert(Px4Lite_ControlSetMotorLocked(0U)==PX4LITE_OK);
  Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);TestAdvance(2000U);Expect("");
  TestAdvance(600U);Expect("1");
  Fresh(0U);assert(Px4Lite_ControlSetMotorLocked(0U)==PX4LITE_OK);
  Rpi(31013U,1000,1000,1000,1000,MAV_RESULT_ACCEPTED);TestAdvance(20U);Expect("1");
  Fresh(0U);s_control_initialized=0U;
  Rpi(31011U,0,0,0,0,MAV_RESULT_ACCEPTED);Expect("");
  assert(Px4Lite_ControlModuleInit()==PX4LITE_OK);Expect("0");
}
/** @brief 真实覆盖终结旧请求；被拒绝的新请求不取消原待执行目标。 */
static void TestSupersessionAndDenial(void)
{
  Fresh(1U);Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);
  assert(Px4Lite_ControlSetMotorThrottlePercent(0U,35U)==PX4LITE_OK);Expect("0");
  TestAdvance(20U);Expect("0");
  Fresh(1U);Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);
  Rpi(31011U,-1,0,0,0,MAV_RESULT_DENIED);Expect("0");
  TestAdvance(20U);Expect("01");
  Fresh(1U);Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);
  Rpi(31013U,1300,1300,1300,1300,MAV_RESULT_ACCEPTED);Expect("0");
  TestAdvance(20U);Expect("01");
  Fresh(1U);Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);
  assert(Px4Lite_ControlSetMotorLocked(1U)==PX4LITE_OK);Expect("0");
  TestAdvance(20U);Expect("0");
}
/** @brief 配置失效与赋值同一锁；恢复调度后插入的新RPi观察不能被旧配置误清掉。 */
static void TestConfigurationTransactions(void)
{
  Fresh(1U);Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);
  s_resume_expected_mode = PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST;
  s_resume_expected_roll = s_target_roll_deg100;s_resume_expected_pitch = s_target_pitch_deg100;s_resume_expected_yaw = s_target_yaw_deg100;
  s_require_feedback_lock = 1U;s_resume_probe = 1U;
  assert(Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST)==PX4LITE_OK);
  s_require_feedback_lock = 0U;assert(!s_resume_probe && !s_test_scheduler_depth);
  Expect("0");TestAdvance(20U);Expect("01");
  Fresh(1U);Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);
  s_resume_expected_mode = (uint8_t)s_control_mode;
  s_resume_expected_roll = 100;s_resume_expected_pitch = -100;s_resume_expected_yaw = 200;
  s_require_feedback_lock = 1U;s_resume_probe = 1U;
  assert(Px4Lite_ControlSetAttitudeTarget(100,-100,200)==PX4LITE_OK);
  s_require_feedback_lock = 0U;assert(!s_resume_probe && !s_test_scheduler_depth);
  Expect("0");TestAdvance(20U);Expect("01");
  Fresh(1U);Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);
  s_resume_expected_mode = PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST;
  s_resume_expected_roll = 100;s_resume_expected_pitch = -100;s_resume_expected_yaw = 200;
  s_resume_probe = 1U;
  RpiV01(31012U,1,1,-1,2,MAV_RESULT_ACCEPTED);
  assert(!s_resume_probe && !s_test_scheduler_depth);
  Expect("01");TestAdvance(20U);Expect("011");
}

/** @brief 已受理的 CNS 降落在 UART6 或本地设置变更时继续降到四路怠速。 */
static void TestCnsLandingConfiguration(void)
{
  const uint16_t pulse[PX4LITE_MOTOR_COUNT] = {1600U, 1600U, 1600U, 1600U};
  uint32_t token;
  unsigned channel;

  Fresh(1U);
  assert(Px4Lite_ControlSetMotorPulses(pulse) == PX4LITE_OK);
  TestAdvance(20U);
  token = RpiV01(31092U, 0, 0, 0, 0, MAV_RESULT_ACCEPTED);
  assert(token != 0U && s_cns.phase == CONTROL_CNS_WAIT_ACK);
  RpiV01(31012U, 1, 2, -3, 4, MAV_RESULT_TEMPORARILY_REJECTED);
  assert(s_control_mode == PX4LITE_CONTROL_MODE_DIRECT &&
         s_target_roll_deg100 == 0 && s_target_pitch_deg100 == 0 && s_target_yaw_deg100 == 0);
  RpiV01(31012U, 0, 0, 0, 0, MAV_RESULT_ACCEPTED);
  assert(Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_DIRECT) == PX4LITE_OK);
  assert(Px4Lite_ControlSetAttitudeTarget(0, 0, 0) == PX4LITE_OK);
  assert(s_cns.token == token && s_cns.phase == CONTROL_CNS_WAIT_ACK);

  Px4Lite_ControlCnsAckSent(token);
  TestAdvance(2020U);
  assert(s_cns.phase == CONTROL_CNS_RAMP && s_test_pwm[0] < 1600U &&
         s_test_pwm[0] > PX4LITE_CNS_IDLE_PULSE_US);
  assert(Px4Lite_ControlSetMode(PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST) == PX4LITE_BUSY);
  assert(Px4Lite_ControlSetAttitudeTarget(100, 0, 0) == PX4LITE_BUSY);
  RpiV01(31012U, 1, 0, 0, 0, MAV_RESULT_TEMPORARILY_REJECTED);
  RpiV01(31012U, 0, 2, 0, 0, MAV_RESULT_TEMPORARILY_REJECTED);
  assert(s_control_mode == PX4LITE_CONTROL_MODE_DIRECT &&
         s_target_roll_deg100 == 0 && s_target_pitch_deg100 == 0 && s_target_yaw_deg100 == 0);
  RpiV01(31012U, 0, 0, 0, 0, MAV_RESULT_ACCEPTED);
  TestAdvance(6000U);
  for (channel = 0U; channel < PX4LITE_MOTOR_COUNT; ++channel) {
    assert(s_test_pwm[channel] == PX4LITE_CNS_IDLE_PULSE_US);
  }
  assert(s_cns.phase == CONTROL_CNS_NONE && Px4Lite_ControlCnsArmed());
  RpiV01(31012U, 1, 2, -3, 4, MAV_RESULT_ACCEPTED);
  assert(s_control_mode == PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST &&
         s_target_roll_deg100 == 200 && s_target_pitch_deg100 == -300 && s_target_yaw_deg100 == 400);
}

/** @brief UART6 油门、本地油门及起降入口均不能取消等待 ACK 的降落。 */
static void TestPendingCnsLandingOutput(void)
{
  const uint16_t pulse[PX4LITE_MOTOR_COUNT] = {1600U, 1600U, 1600U, 1600U};
  uint32_t token;
  unsigned channel;

  Fresh(1U);
  assert(Px4Lite_ControlSetMotorPulses(pulse) == PX4LITE_OK);
  TestAdvance(20U);
  token = RpiV01(31092U, 0, 0, 0, 0, MAV_RESULT_ACCEPTED);
  assert(token != 0U && s_cns.phase == CONTROL_CNS_WAIT_ACK);
  RpiV01(31011U, 55, 55, 55, 55, MAV_RESULT_TEMPORARILY_REJECTED);
  RpiV01(31013U, 1750, 1750, 1750, 1750, MAV_RESULT_TEMPORARILY_REJECTED);
  RpiV01(31011U, 0, 0, 0, 0, MAV_RESULT_ACCEPTED);
  RpiV01(31013U, 1000, 1000, 1000, 1000, MAV_RESULT_ACCEPTED);
  Expect("0000");
  assert(App_SetMotorThrottlePercent(0U, 55U) == PX4LITE_NOT_READY);
  assert(App_SetMotorThrottlePercent(0U, 0U) == PX4LITE_OK);
  assert(Px4Lite_ControlSetMotorPulseUs(0U, 1750U) == PX4LITE_NOT_READY);
  assert(Px4Lite_ControlSetMotorPulseUs(0U, 1000U) == PX4LITE_OK);
  assert(App_StartMotorAutoTakeoff() == PX4LITE_BUSY);
  assert(App_StartMotorAutoLanding() == PX4LITE_BUSY);
  TestAdvance(20U);
  assert(s_cns.token == token && s_cns.phase == CONTROL_CNS_WAIT_ACK);
  for (channel = 0U; channel < PX4LITE_MOTOR_COUNT; ++channel) {
    assert(s_test_pwm[channel] == 1600U);
  }

  Px4Lite_ControlCnsAckSent(token);
  TestAdvance(8100U);
  assert(s_cns.phase == CONTROL_CNS_NONE);
  for (channel = 0U; channel < PX4LITE_MOTOR_COUNT; ++channel) {
    assert(s_test_pwm[channel] == PX4LITE_CNS_IDLE_PULSE_US);
  }
  Expect("0000");
}

/** @brief 急停及上锁仍可在 CNS 降落期间立即写入停车脉宽。 */
static void TestCnsLandingStop(void)
{
  const uint16_t pulse[PX4LITE_MOTOR_COUNT] = {1600U, 1600U, 1600U, 1600U};
  const uint16_t stops[] = {31090U, 31094U};
  uint32_t token;
  unsigned i;
  for (i = 0U; i < sizeof(stops) / sizeof(stops[0]); ++i) {
    Fresh(1U);
    assert(Px4Lite_ControlSetMotorPulses(pulse) == PX4LITE_OK);
    TestAdvance(20U);
    token = RpiV01(31092U, 0, 0, 0, 0, MAV_RESULT_ACCEPTED);
    Px4Lite_ControlCnsAckSent(token);
    TestAdvance(2020U);
    assert(s_cns.phase == CONTROL_CNS_RAMP && s_test_pwm[0] > PX4LITE_CNS_IDLE_PULSE_US);
    assert(RpiV01(stops[i], 0, 0, 0, 0, MAV_RESULT_ACCEPTED) != 0U);
    assert(TestLocked() && !Px4Lite_ControlCnsArmed());
    TestMinimum();
  }
}

/** @brief 自动起飞和降落只在各自真实终点四路写入成功后高音。 */
static void TestAutoCompletionAndNoop(void)
{
  uint32_t start;
  Fresh(1U);Rpi(31091U,0,0,0,0,MAV_RESULT_ACCEPTED);Expect("");
  TestAdvance(5980U);Expect("");TestAdvance(20U);Expect("1");
  assert(s_auto_profile==PX4LITE_CONTROL_AUTO_NONE && s_test_pwm[0]==1600U);
  TestAdvance(100U);Expect("1");
  Rpi(31092U,0,0,0,0,MAV_RESULT_ACCEPTED);Expect("1");
  start=s_auto_start_ms;
  Rpi(31011U,0,0,0,0,MAV_RESULT_ACCEPTED);Expect("10");
  assert(s_auto_profile==PX4LITE_CONTROL_AUTO_LANDING && s_auto_start_ms==start);
  TestAdvance(7980U);Expect("10");TestAdvance(20U);Expect("101");TestMinimum();
  Fresh(0U);Rpi(31092U,0,0,0,0,MAV_RESULT_ACCEPTED);Expect("1");
}
/** @brief 保护降落、取消、失败与复位都结束旧过程低音，内部撤收完成不能替其成功。 */
static void TestAutoFailureAndCancellation(void)
{
  Fresh(1U);Rpi(31091U,0,0,0,0,MAV_RESULT_ACCEPTED);TestAdvance(200U);Expect("");
  s_test_imu_state=PX4LITE_STATE_OFFLINE;TestAdvance(20U);Expect("0");
  TestAdvance(8100U);Expect("0");
  Fresh(1U);Rpi(31091U,0,0,0,0,MAV_RESULT_ACCEPTED);
  Rpi(31091U,0,0,0,0,MAV_RESULT_TEMPORARILY_REJECTED);Expect("0");
  TestAdvance(6000U);Expect("01");
  Fresh(1U);Rpi(31091U,0,0,0,0,MAV_RESULT_ACCEPTED);
  s_test_write_fail=1U;TestAdvance(20U);Expect("0");
  s_test_write_fail=0U;TestAdvance(100U);Expect("0");
  Fresh(1U);Rpi(31091U,0,0,0,0,MAV_RESULT_ACCEPTED);
  assert(Px4Lite_ControlSetMotorThrottlePercent(0U,30U)==PX4LITE_OK);Expect("0");
  TestAdvance(40U);Expect("0");
  Fresh(1U);Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);
  assert(Px4Lite_ControlRecover()==PX4LITE_OK);Expect("0");
  Fresh(1U);Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);
  s_test_now+=PX4LITE_CONTROL_FAILSAFE_TIMEOUT_MS+1U;Px4Lite_ControlRun(s_test_now);Expect("0");
#if PX4LITE_CONTROL_POWER_GATE_ENABLE
  Fresh(1U);Rpi(31011U,20,20,20,20,MAV_RESULT_ACCEPTED);
  s_test_battery_mv=3000U;TestAdvance(20U);Expect("0");
#endif
}
/** @brief 沿用真实控制计时检验跨毫秒回绕的自动过程终结。 */
static void TestWrapAndLocalSilence(void)
{
  Fresh(1U);s_test_now=UINT32_MAX-100U;s_last_run_ms=s_test_now;
  Rpi(31091U,0,0,0,0,MAV_RESULT_ACCEPTED);TestAdvance(6000U);Expect("1");
  Fresh(1U);TestCommand(31091U,0,0,0,0,MAV_RESULT_ACCEPTED);
  TestAdvance(6000U);Expect("");
  Fresh(1U);assert(Px4Lite_ControlStartAutoTakeoff()==PX4LITE_OK);
  TestAdvance(6000U);Expect("");
}
int main(void)
{
  TestClassification();TestRealManualOutputs();TestPreparingAndZero();
  TestCnsLandingConfiguration();TestPendingCnsLandingOutput();TestCnsLandingStop();
  TestSupersessionAndDenial();TestConfigurationTransactions();TestAutoCompletionAndNoop();TestAutoFailureAndCancellation();
  TestWrapAndLocalSilence();
  printf("RPi execution feedback: real Command/Control target-vs-PWM / preparing+zero / atomic config+preempt / CNS landing config interlock+safe stops / actual auto completion / abort+protection / failed+superseded / noop landing / local+LoRa silence / wrap PASS feedback=%u gate=%u\n",
    (unsigned)PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK,(unsigned)PX4LITE_CONTROL_POWER_GATE_ENABLE);
  return 0;
}
