/**
 * @file test_pi_result_notify_integration.c
 * @brief 真实Command/Control、波特率事务与声光播放器的Pi结果通知防重复集成验证。
 * @details UART6 CRC和消息解码由独立RX专项验证；这里直接调用其校验边界，
 *          不替换控制决策、配置应用或播放器。硬件仅为受控平台记录，不证明物理执行。
 */
#define TEST_RPI_RESULT_NOTIFY 1
#define Px4Lite_CopyMotor PiNotifyLegacyCopyMotor
#define main PiNotifyLegacyManualLockMain
#include "test_motor_manual_lock.c"
#undef main
#undef Px4Lite_CopyMotor

/** @brief 本夹具聚焦Pi结果通知；不发布背景音乐使用的电机topic，真实音乐由独立集成覆盖。 */
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out) { (void)out;return PX4LITE_NOT_READY; }
#include "../../Framework/Src/px4lite_rpi_baud_switch.c"
#include "../../Framework/Src/px4lite_startup_tone.c"

static uint8_t s_audio_owner;
static uint16_t s_audio_frequency;
static unsigned s_audio_io;
static uint32_t s_applied_baud=115200U;

/** @brief 任一声光硬件操作只能发生在显式模拟的sensor上下文。 */
static void AudioIo(void)
{ assert(s_audio_owner==1U && !s_test_critical_depth && !s_test_scheduler_depth);s_audio_io++; }
/** @brief 初始化仅登记输出事实，不调用真实定时器。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void) { AudioIo();s_audio_frequency=0U;return PX4LITE_OK; }
/** @brief 记录真实播放器请求的频率，禁止Comm/Control直接发声。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t hz) { AudioIo();s_audio_frequency=hz;return PX4LITE_OK; }
/** @brief RGB输出均保留真实播放器调用链，仅替换外设。 */
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void) { AudioIo();return PX4LITE_OK; }
/** @brief 撤收同样不得在接收或控制线程执行。 */
void Px4Lite_PlatformRgbDeInit(void) { AudioIo(); }
/** @brief 颜色限制由原播放器专项验证，本单元核验唯一硬件owner。 */
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t red,uint8_t green,uint8_t blue)
{ (void)red;(void)green;(void)blue;AudioIo();return PX4LITE_OK; }
/** @brief 服务使用同一真实毫秒时钟，不能因通知额外创建硬件所有者。 */
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t now_ms)
{ AudioIo();assert(now_ms==s_test_now);return PX4LITE_OK; }
/** @brief 无H1待发响应；原波特率参数校验和应用过程仍使用生产实现。 */
Px4Lite_Result_t Px4Lite_RpiH1EchoPeekResponse(Px4Lite_RpiH1EchoResponseView_t *out)
{ (void)out;return PX4LITE_NOT_READY; }
/** @brief 只有真实配置应用API可以改变本夹具的已应用速率。 */
Px4Lite_Result_t Px4Lite_RpiMavlinkSetBaudRate(uint32_t baud_bps)
{ assert(baud_bps==57600U);s_applied_baud=baud_bps;return PX4LITE_OK; }
/** @brief 配置结果读回反映已应用值，不能把入队当作成功。 */
Px4Lite_Result_t Px4Lite_RpiMavlinkGetBaudRate(uint32_t *baud_bps)
{ assert(baud_bps);*baud_bps=s_applied_baud;return PX4LITE_OK; }

/** @brief 真实sensor单步推进声光并发布状态，其余任务只登记结果。 */
static Px4Lite_StartupToneStatus_t Sensor(void)
{
  Px4Lite_StartupToneStatus_t status;
  s_audio_owner=1U;Px4Lite_StartupToneRun(s_test_now*1000U);s_audio_owner=0U;
  assert(Px4Lite_StartupToneCopyStatus(&status)==PX4LITE_OK);
  assert(!s_test_critical_depth && !s_test_scheduler_depth);
  return status;
}
/** @brief 实际原控制周期与sensor交错推进，避免测试无意触发旧失步保护。 */
static Px4Lite_StartupToneStatus_t Advance(uint32_t duration_ms)
{
  Px4Lite_StartupToneStatus_t status;
  uint32_t elapsed;
  for(elapsed=0U;elapsed<duration_ms;elapsed+=PX4LITE_CONTROL_PERIOD_MS) {
    TestAdvance(PX4LITE_CONTROL_PERIOD_MS);status=Sensor();
  }
  assert(duration_ms);return status;
}
/** @brief 完成一次既有旋律，核验类型和总登记次数，禁止同一结果再自动追加一声。 */
static void Play(uint8_t kind,uint16_t first_hz,unsigned received,unsigned remaining)
{
  Px4Lite_StartupToneStatus_t status=Sensor();unsigned guard=0U;
  assert(status.feedback_received==received && status.feedback_active && status.feedback_kind==kind);
  assert(s_audio_frequency==first_hz);
  while(status.feedback_active) { assert(++guard<100U);status=Advance(PX4LITE_CONTROL_PERIOD_MS); }
  assert(status.feedback_pending==remaining && status.feedback_received==received && s_audio_frequency==0U);
}
/** @brief 新结果通知不能修改控制输出、COMMAND_ACK或从当前上下文驱动声光。 */
static void Notify(uint8_t succeeded)
{
  uint8_t payload[128]={1U,0U};Px4Lite_Result_t result;unsigned before=s_audio_io;
  uint16_t pwm[PX4LITE_MOTOR_COUNT];
  memcpy(pwm,s_test_pwm,sizeof(pwm));payload[1]=succeeded;s_test_ack=0xA5U;
  result=Px4Lite_CommandHandleRpiResultTunnel(1U,191U,1U,193U,0x8040U,payload,2U,
    (uint8_t)(succeeded?2U:1U),PX4LITE_MAVLINK_LINK_RPI);
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK && PX4LITE_RPI_RESULT_SOURCE_PI
  assert(result==PX4LITE_OK);
#else
  assert(result==PX4LITE_IDLE || result==PX4LITE_OK);
#endif
  assert(s_test_ack==0xA5U && !memcmp(pwm,s_test_pwm,sizeof(pwm)) && before==s_audio_io);
}
/** @brief 通过真实命令入口提交RPi控制，保留业务ACK和实际PWM写入。 */
static void Command(float value,uint8_t expected_ack)
{
  unsigned before=s_audio_io;
  assert(Px4Lite_CommandHandleMavlinkLong(31011U,1U,191U,1U,193U,value,value,value,value,
    PX4LITE_MAVLINK_LINK_RPI,s_test_now)==PX4LITE_OK);
  assert(s_test_ack==expected_ack && before==s_audio_io);
}
/** @brief 默认Pi模式不自动重复；旧模式自动提示而0x8040静默，主开关关闭仍保模块。 */
int main(void)
{
  Px4Lite_StartupToneStatus_t status;unsigned received=0U;
  static const uint8_t request[14]={1U,3U,1U,2U,3U,4U,5U,6U,7U,8U,0U,0xE1U,0U,0U};
  TestReset();TestUnlockReady();
  s_audio_owner=1U;Px4Lite_StartupToneInit(s_test_now*1000U);s_audio_owner=0U;
  (void)Advance(2700U);status=Sensor();assert(!status.feedback_active && !status.feedback_received);

  Command(25.0f,MAV_RESULT_ACCEPTED);
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);assert(s_test_pwm[0]==1250U && s_test_pwm[3]==1250U);
#if PX4LITE_RPI_LOCAL_RESULT_FEEDBACK
  Notify(1U);Play(PX4LITE_FEEDBACK_COMMAND_SUCCEEDED,1568U,++received,0U);
#elif PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK && PX4LITE_RPI_RESULT_SOURCE_PI
  status=Sensor();assert(!status.feedback_received && !status.feedback_active);
  Notify(1U);Play(PX4LITE_FEEDBACK_COMMAND_SUCCEEDED,1568U,++received,0U);
#else
  Notify(1U);status=Sensor();assert(!status.feedback_received && !status.feedback_active);
#endif

  Command(-1.0f,MAV_RESULT_DENIED);
#if PX4LITE_RPI_LOCAL_RESULT_FEEDBACK || (PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK && PX4LITE_RPI_RESULT_SOURCE_PI)
  Notify(0U);Play(PX4LITE_FEEDBACK_COMMAND_FAILED,880U,++received,0U);
#else
  Notify(0U);status=Sensor();assert(status.feedback_received==received && !status.feedback_active);
#endif

  assert(Px4Lite_RpiBaudSwitchInit()==PX4LITE_OK);
  assert(Px4Lite_RpiBaudSwitchHandleTunnel(1U,191U,1U,193U,1U,0x8003U,request,14U,12U)==PX4LITE_OK);
  status=Sensor();assert(!status.feedback_active && status.feedback_received==received);
  Px4Lite_RpiBaudSwitchResponseFailed(PX4LITE_IO_ERROR);
#if PX4LITE_RPI_LOCAL_RESULT_FEEDBACK
  Play(PX4LITE_FEEDBACK_COMMAND_FAILED,880U,++received,0U);
#else
  status=Sensor();assert(!status.feedback_active && status.feedback_received==received);
#endif
  assert(Px4Lite_RpiBaudSwitchIsPending() && s_applied_baud==115200U);
  assert(Px4Lite_RpiBaudSwitchResponseSent()==PX4LITE_OK && s_applied_baud==57600U);
#if PX4LITE_RPI_LOCAL_RESULT_FEEDBACK || (PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK && PX4LITE_RPI_RESULT_SOURCE_PI)
  Notify(1U);Play(PX4LITE_FEEDBACK_COMMAND_SUCCEEDED,1568U,++received,0U);
#else
  Notify(1U);status=Sensor();assert(!status.feedback_active && status.feedback_received==received);
#endif

  /* 两条同值可以属于不同请求：F407不能按结果值或时间自行去重。 */
  Notify(1U);Notify(1U);
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK && PX4LITE_RPI_RESULT_SOURCE_PI
  received+=2U;Play(PX4LITE_FEEDBACK_COMMAND_SUCCEEDED,1568U,received,1U);
  TestAdvance(PX4LITE_CONTROL_PERIOD_MS);Play(PX4LITE_FEEDBACK_COMMAND_SUCCEEDED,1568U,received,0U);
#else
  status=Sensor();assert(!status.feedback_active && status.feedback_received==received);
#endif

  /* 原模块提示不依赖命令结果来源；此处直接注入已有Health的通知边界。 */
  Px4Lite_StartupToneNotifyModuleConnection(1U);Play(PX4LITE_FEEDBACK_MODULE_CONNECTED,1047U,++received,0U);
  Px4Lite_StartupToneNotifyModuleConnection(0U);TestAdvance(PX4LITE_CONTROL_PERIOD_MS);
  Play(PX4LITE_FEEDBACK_MODULE_DISCONNECTED,2093U,++received,0U);
  assert(!s_test_scheduler_depth && !s_test_critical_depth);
  printf("Pi result integration: real Command+Control+Baud+player / execution+ACK retained / one source only / same-value notifications independent / module cues preserved PASS source_pi=%u feedback=%u\n",
    (unsigned)PX4LITE_RPI_RESULT_SOURCE_PI,(unsigned)PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK);
  return 0;
}
