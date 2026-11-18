/**
 * @file test_motor_music_integration.c
 * @brief 真实Control/PWM快照到真实sensor背景播放器的门控、暂停续播和提示抢占验证。
 * @details 复用原人工锁平台夹具，不替换Control或播放器决策；仅注入四相位原创测试音，
 *          不包含正式歌曲曲谱，不把平台PWM成功当作电机转速或实物声光验收。
 */
#define TEST_RPI_RESULT_NOTIFY 1
#define Px4Lite_GetModuleStatus MotorMusicLegacyGetModuleStatus
#define main MotorMusicLegacyManualLockMain
#include "test_motor_manual_lock.c"
#undef main
#undef Px4Lite_GetModuleStatus

/** @brief 补充原夹具未消费的Control状态出口，数值仍来自真实Control的状态发布。 */
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id,Px4Lite_ModuleStatus_t *out)
{
  if(id==PX4LITE_MODULE_CONTROL) {
    memset(out,0,sizeof(*out));out->module_id=id;out->state=s_test_control_state;
    out->fault_code=s_test_control_fault;return PX4LITE_OK;
  }
  return MotorMusicLegacyGetModuleStatus(id,out);
}

/* 仅用于验证引擎的原创短音，不复制正式歌曲；生产播放器按约定允许host注入。 */
#if PX4LITE_ENABLE_MOTOR_MUSIC
#define PX4LITE_MOTOR_MUSIC_SCORE_INJECTED 1
/** @brief 原创测试单音/静音相位，频率Hz、持续时间ms。 */
typedef struct {
  uint16_t frequency_hz; /**< 请求音高，单位Hz，0表示静音相位。 */
  uint16_t duration_ms; /**< 当前相位最短持续时间，单位真实ms。 */
} MotorMusicNote;
static const MotorMusicNote s_motor_music_score[]={
  {523U,100U},{0U,40U},{659U,120U},{0U,60U}
};
#define MOTOR_MUSIC_SCORE_COUNT (sizeof(s_motor_music_score)/sizeof(s_motor_music_score[0]))
#endif
#include "../../Framework/Src/px4lite_startup_tone.c"

static uint8_t s_music_sensor_owner;
static uint16_t s_music_frequency;
static uint8_t s_music_rgb[3];
static unsigned s_music_audio_calls;

/** @brief 任何蜂鸣器/RGB输出必须来自显式sensor步且不处于快照锁/调度锁中。 */
static void MusicIo(void)
{ assert(s_music_sensor_owner && !s_test_critical_depth && !s_test_scheduler_depth);s_music_audio_calls++; }
/** @brief 只记录初始化事实，不访问真实定时器。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void)
{ MusicIo();s_music_frequency=0U;return PX4LITE_OK; }
/** @brief 真实播放器通过唯一平台频率出口输出，不能改变控制电机PWM。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t frequency_hz)
{ MusicIo();s_music_frequency=frequency_hz;return PX4LITE_OK; }
/** @brief RGB初始化同样只能在sensor上下文。 */
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void) { MusicIo();return PX4LITE_OK; }
/** @brief 故障撤收不允许来自Comm或Control。 */
void Px4Lite_PlatformRgbDeInit(void) { MusicIo(); }
/** @brief 保存实际被平台接受的颜色，不直接操纵DMA缓冲。 */
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t r,uint8_t g,uint8_t b)
{ MusicIo();s_music_rgb[0]=r;s_music_rgb[1]=g;s_music_rgb[2]=b;return PX4LITE_OK; }
/** @brief RGB服务沿用真实毫秒时刻，不生成新的硬件所有者。 */
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t now_ms)
{ MusicIo();assert(now_ms==s_test_now);return PX4LITE_OK; }

/** @brief 一次真实sensor推进，同时证明背景播放不改变电机输出、topic或ACK。 */
static Px4Lite_StartupToneStatus_t MusicSensor(void)
{
  Px4Lite_StartupToneStatus_t status;
  Px4Lite_MotorOutputs_t before=s_test_motor;
  uint16_t pwm[PX4LITE_MOTOR_COUNT];uint8_t ack=s_test_ack;
  memcpy(pwm,s_test_pwm,sizeof(pwm));
  s_music_sensor_owner=1U;Px4Lite_StartupToneRun(s_test_now*1000U);s_music_sensor_owner=0U;
  assert(Px4Lite_StartupToneCopyStatus(&status)==PX4LITE_OK);
  assert(!memcmp(pwm,s_test_pwm,sizeof(pwm)) && !memcmp(&before,&s_test_motor,sizeof(before)) && ack==s_test_ack);
  assert(!s_test_scheduler_depth && !s_test_critical_depth);
  return status;
}
/** @brief 按20ms真实Control周期交错推进sensor，避免无意混入控制失步保护。 */
static Px4Lite_StartupToneStatus_t MusicAdvance(uint32_t duration_ms)
{
  Px4Lite_StartupToneStatus_t status;uint32_t elapsed;
  assert(duration_ms && duration_ms%PX4LITE_CONTROL_PERIOD_MS==0U);
  for(elapsed=0U;elapsed<duration_ms;elapsed+=PX4LITE_CONTROL_PERIOD_MS) {
    TestAdvance(PX4LITE_CONTROL_PERIOD_MS);status=MusicSensor();
  }
  return status;
}
/** @brief 复位场景平台再初始化真实控制及播放器，模拟新启动；不把前场景声卡频率当上电默认。 */
static void MusicFresh(uint8_t unlock)
{
  Px4Lite_StartupToneStatus_t status;
  s_music_frequency=0U;memset(s_music_rgb,0,sizeof(s_music_rgb));
  TestReset();s_music_sensor_owner=1U;Px4Lite_StartupToneInit(s_test_now*1000U);s_music_sensor_owner=0U;
#if PX4LITE_ENABLE_STARTUP_TONE
  status=MusicAdvance(2700U);assert(status.state==PX4LITE_TONE_DONE);
#else
  status=MusicSensor();
#endif
  assert(!status.feedback_active && !status.feedback_pending);
  assert(!status.music_gate && !status.music_running && !s_music_frequency);
  if(unlock) { TestUnlockReady();status=MusicSensor();assert(!status.music_gate && !status.music_running); }
}
/** @brief 四路停止目标只提交意图，真正最小输出由下一次真实ControlRun确认。 */
static void MusicStopTargets(void)
{
  static const uint16_t minimum[PX4LITE_MOTOR_COUNT]={1000U,1000U,1000U,1000U};
  assert(Px4Lite_ControlSetMotorPulses(minimum)==PX4LITE_OK);
}

/** @brief 1000us预备不唱，任意单路1001us即使百分比为0仍触发，停止后保持进度。 */
static void TestActualPulseAndPauseResume(void)
{
  Px4Lite_StartupToneStatus_t status,paused;uint32_t before_calls;
  MusicFresh(0U);assert(TestLocked()==1U);before_calls=s_music_audio_calls;
  assert(Px4Lite_ControlSetMotorPulseUs(0U,1200U)==PX4LITE_NOT_READY);
  status=MusicSensor();assert(!status.music_gate && !s_music_frequency && s_music_audio_calls>=before_calls);
  TestUnlockReady();status=MusicSensor();assert(!status.music_gate && s_test_motor.run_state==0U);
  assert(Px4Lite_ControlSetMotorPulseUs(0U,1001U)==PX4LITE_OK);
  status=MusicSensor();assert(!status.music_gate); /* 尚未写PWM，不能接受即唱。 */
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);assert(s_test_pwm[0]==1001U);
  assert(s_test_motor.duty_percent[0]==0U && s_test_motor.speed_level==0U);
  status=MusicSensor();
#if PX4LITE_ENABLE_MOTOR_MUSIC
  assert(status.music_ready && status.music_gate && status.music_running && s_music_frequency==523U);
  status=MusicAdvance(180U);assert(status.music_phase_index==2U && s_music_frequency==659U);
#else
  assert(!status.music_gate && !status.music_running && !s_music_frequency);
#endif
  MusicStopTargets();status=MusicSensor();
#if PX4LITE_ENABLE_MOTOR_MUSIC
  assert(status.music_gate); /* 硬件还是旧PWM，不能仅按停止目标提前暂停。 */
#endif
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);TestMinimum();paused=MusicSensor();
  assert(!paused.music_gate && !paused.music_running && !s_music_frequency);
  status=MusicAdvance(300U);
  assert(status.music_phase_index==paused.music_phase_index && status.music_remaining_us==paused.music_remaining_us);
  assert(Px4Lite_ControlSetMotorPulseUs(3U,1300U)==PX4LITE_OK);
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);assert(s_test_pwm[3]==1300U && s_test_pwm[0]==1000U);status=MusicSensor();
#if PX4LITE_ENABLE_MOTOR_MUSIC
  assert(status.music_gate && status.music_running && status.music_phase_index==paused.music_phase_index);
  assert(status.music_remaining_us<=paused.music_remaining_us && status.music_remaining_us>0U);
#endif
}

/** @brief 急停不立即刷新Motor topic时，当前人工锁和Control状态仍能立即否决旧高脉宽。 */
static void TestEmergencyStopOldSnapshot(void)
{
  Px4Lite_StartupToneStatus_t status;uint32_t sequence;
  MusicFresh(1U);assert(Px4Lite_ControlSetMotorPulseUs(2U,1400U)==PX4LITE_OK);
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);status=MusicSensor();
#if PX4LITE_ENABLE_MOTOR_MUSIC
  assert(status.music_gate && s_music_frequency);
#endif
  sequence=s_test_motor.header.sequence;
  assert(Px4Lite_ControlEmergencyStop(s_test_now)==PX4LITE_OK);TestMinimum();
  assert(s_test_motor.header.sequence==sequence && s_test_motor.pulse_us[2]==1400U && s_test_motor.run_state);
  status=MusicSensor();assert(!status.music_gate && !status.music_running && !s_music_frequency);
}
/** @brief 写PWM和撤收都失败时旧topic保持高值，故障/锁必须使背景音乐暂停。 */
static void TestFailedOutputRetainsOldSnapshot(void)
{
  Px4Lite_StartupToneStatus_t status;uint32_t sequence;
  MusicFresh(1U);assert(Px4Lite_ControlSetMotorPulseUs(0U,1500U)==PX4LITE_OK);
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);status=MusicSensor();sequence=s_test_motor.header.sequence;
  s_test_write_fail=1U;s_test_disarm_result=PX4LITE_IO_ERROR;
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);
  assert(s_test_motor.header.sequence==sequence && s_test_motor.pulse_us[0]==1500U);
  assert(TestLocked() && s_test_control_state==PX4LITE_STATE_DEGRADED);
  status=MusicSensor();assert(!status.music_gate && !status.music_running && !s_music_frequency);
  s_test_write_fail=0U;s_test_disarm_result=PX4LITE_OK;
}
/** @brief Control停更超过200ms不能继续借用旧运行快照；人工锁成功也立即暂停。 */
static void TestFreshnessAndManualLock(void)
{
  Px4Lite_StartupToneStatus_t status;
  MusicFresh(1U);TestCommand(31011U,25,0,0,0,MAV_RESULT_ACCEPTED);
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);assert(s_test_pwm[0]==1250U && s_test_pwm[3]==1000U);
  status=MusicSensor();
#if PX4LITE_ENABLE_MOTOR_MUSIC
  assert(status.music_gate);
#endif
  s_test_now+=PX4LITE_MOTOR_MUSIC_FRESH_MS+1U;status=MusicSensor();
  assert(!status.music_gate && !status.music_running && !s_music_frequency);
  MusicFresh(1U);assert(Px4Lite_ControlSetMotorPulseUs(1U,1350U)==PX4LITE_OK);
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);(void)MusicSensor();
  assert(App_SetMotorLocked(1U)==PX4LITE_OK);TestMinimum();status=MusicSensor();
  assert(!status.music_gate && !status.music_running && !s_music_frequency);
}
/** @brief 有限插拔/命令提示抢占背景曲，曲谱位置冻结，提示完再续播；停机不吞掉提示。 */
static void TestFeedbackPriority(void)
{
  Px4Lite_StartupToneStatus_t before,status;unsigned guard=0U;
  MusicFresh(1U);assert(Px4Lite_ControlSetMotorPulseUs(0U,1200U)==PX4LITE_OK);
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);(void)MusicSensor();before=MusicAdvance(180U);
#if PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK
  Px4Lite_StartupToneNotifyModuleConnection(1U);status=MusicSensor();
  assert(status.feedback_active && status.feedback_kind==PX4LITE_FEEDBACK_MODULE_CONNECTED && s_music_frequency==1047U);
  assert(!status.music_running && status.music_phase_index==before.music_phase_index);
  while(status.feedback_active || status.feedback_pending) { assert(++guard<100U);status=MusicAdvance(20U); }
  if(!status.music_running) { status=MusicAdvance(20U); }
#if PX4LITE_ENABLE_MOTOR_MUSIC
  assert(status.music_gate && status.music_running && status.music_phase_index==before.music_phase_index);
#endif
#endif
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  Px4Lite_StartupToneNotifyRpiResult(0U);status=MusicSensor();
  assert(status.feedback_active && s_music_frequency==880U && !status.music_running);
  MusicStopTargets();TestAdvance(PX4LITE_CONTROL_PERIOD_MS);status=MusicSensor();
  assert(!status.music_gate && status.feedback_active && s_music_frequency==880U);
  guard=0U;while(status.feedback_active || status.feedback_pending) { assert(++guard<100U);status=MusicAdvance(20U); }
  assert(!status.music_running && !s_music_frequency);
#endif
  (void)before;(void)status;(void)guard;
}

/** @brief 各功能组合均运行真实控制，不触发实体电机或蜂鸣器。 */
int main(void)
{
  TestActualPulseAndPauseResume();TestEmergencyStopOldSnapshot();TestFailedOutputRetainsOldSnapshot();
  TestFreshnessAndManualLock();TestFeedbackPriority();
  assert(!s_test_critical_depth && !s_test_scheduler_depth && !s_music_sensor_owner);
  printf("Motor music integration: real Control/PWM/topic/player / one1001us / preparing minimum silent / pause-resume / lock-estop / stale-high failure / age / cue priority / ACK+motor unchanged PASS music=%u\n",(unsigned)PX4LITE_ENABLE_MOTOR_MUSIC);
  return 0;
}
