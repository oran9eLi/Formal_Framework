/**
 * @file test_cns_tone_v01.c
 * @brief CNS V0.1真实声光owner的节拍、抢占、取消、token及故障隔离回归。
 * @details 只观察平台输出和公开结果；所有I/O必须来自sensor推进且临界区外，不控制实机。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static unsigned s_critical,s_sensor;
#define taskENTER_CRITICAL() (s_critical++)
#define taskEXIT_CRITICAL() do { assert(s_critical);s_critical--; } while(0)
#include "px4lite_startup_tone.h"
#include "px4lite_config.h"
#include "px4lite_platform.h"
/** @brief 平台替身仅记录真正接受的频率/灯色，允许注入真实失败或DMA忙。 */
static struct {
  uint32_t ms,replace_token;
  uint16_t hz;
  uint8_t rgb[3];
  unsigned buzzer_init,tones,colors,rgb_init,services;
  Px4Lite_Result_t init_rc,tone_rc,color_rc,service_rc;
} m;
static uint32_t s_base_us,s_base_ms;

uint32_t Px4Lite_PlatformGetMs(void) { return m.ms; }
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void)
{ assert(s_sensor && !s_critical);m.buzzer_init++;return m.init_rc; }
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t hz)
{
  assert(s_sensor && !s_critical);m.tones++;
  if(hz && m.tone_rc!=PX4LITE_OK) { return m.tone_rc; }
  m.hz=hz;
  if(!hz && m.replace_token) {
    uint32_t token=m.replace_token;m.replace_token=0U;
    Px4Lite_StartupToneCnsCancel(1U);Px4Lite_StartupToneCnsStart(PX4LITE_CNS_TONE_ESTOP,token);
  }
  return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void)
{ assert(s_sensor && !s_critical);m.rgb_init++;return PX4LITE_OK; }
void Px4Lite_PlatformRgbDeInit(void) { assert(s_sensor && !s_critical); }
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t r,uint8_t g,uint8_t b)
{
  assert(s_sensor && !s_critical);m.colors++;
  if(m.color_rc!=PX4LITE_OK) { return m.color_rc; }
  m.rgb[0]=r;m.rgb[1]=g;m.rgb[2]=b;return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t now_ms)
{ assert(s_sensor && !s_critical && now_ms==m.ms);m.services++;return m.service_rc; }
/** @brief 背景曲没有有效电机事实，不得抢占专项动作。 */
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out)
{ (void)out;return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_ControlGetMotorLocked(uint8_t *out)
{ *out=1U;return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id,Px4Lite_ModuleStatus_t *out)
{ (void)id;(void)out;return PX4LITE_NOT_READY; }
#include "../../Framework/Src/px4lite_startup_tone.c"

/** @brief sensor显式推进独立微秒/毫秒时钟，API调用本身不得做硬件I/O。 */
static void At(uint32_t ms)
{
  m.ms=s_base_ms+ms;s_sensor=1U;
  Px4Lite_StartupToneRun(s_base_us+ms*1000U);
  s_sensor=0U;assert(!s_critical);
}
/** @brief 清理前场景后重建owner；生产只初始化一次，夹具重复初始化不冒充热恢复。 */
static void Begin(uint8_t finish_startup)
{
  Px4Lite_StartupToneCnsCancel(1U);At(0U);
  memset(&m,0,sizeof(m));s_base_us=0U;s_base_ms=0U;
  s_sensor=1U;Px4Lite_StartupToneInit(0U);s_sensor=0U;
#if PX4LITE_ENABLE_STARTUP_TONE
  if(finish_startup) { At(2625U);s_base_us=2625000U;s_base_ms=2625U; }
#else
  (void)finish_startup;
#endif
}

#if PX4LITE_ENABLE_CNS_CONTROL
/** @brief 公开诊断快照只用于独立累计事实，不读取播放器私有相位。 */
static Px4Lite_StartupToneStatus_t Status(void)
{ Px4Lite_StartupToneStatus_t s;assert(Px4Lite_StartupToneCopyStatus(&s)==PX4LITE_OK);return s; }
/** @brief 登记和查询在调用任务执行，不允许同步蜂鸣器或灯I/O。 */
static void Start(uint8_t action,uint32_t token)
{
  unsigned tones=m.tones,colors=m.colors,init=m.buzzer_init;
  Px4Lite_StartupToneCnsStart(action,token);
  assert(Px4Lite_StartupToneCnsResult(token)==PX4LITE_BUSY);
  assert(m.tones==tones && m.colors==colors && m.buzzer_init==init && !s_critical);
}
/** @brief 解锁/上锁完整1秒，token匹配且到期前不得报告完成。 */
static void TestLongTones(void)
{
  Begin(1U);Start(PX4LITE_CNS_TONE_ARM,101U);At(0U);assert(m.hz==880U);
  At(999U);assert(m.hz==880U && Px4Lite_StartupToneCnsResult(101U)==PX4LITE_BUSY);
  At(1000U);assert(!m.hz && Px4Lite_StartupToneCnsResult(101U)==PX4LITE_OK);
  assert(Px4Lite_StartupToneCnsResult(100U)==PX4LITE_IDLE);
  Start(PX4LITE_CNS_TONE_DISARM,102U);At(1010U);assert(m.hz==3136U);
  At(2009U);assert(Px4Lite_StartupToneCnsResult(102U)==PX4LITE_BUSY);
  At(2010U);assert(!m.hz && Px4Lite_StartupToneCnsResult(102U)==PX4LITE_OK);
}
/** @brief 起飞三声/急停两声，每100ms音/间隙；延迟服务不会跳过音符。 */
static void TestQuickTones(void)
{
  unsigned i;Begin(1U);Start(PX4LITE_CNS_TONE_TAKEOFF,201U);
  for(i=0U;i<6U;i++) {
    At(i*100U);assert(m.hz==((i&1U)?0U:3136U));
    assert(Px4Lite_StartupToneCnsResult(201U)==PX4LITE_BUSY);
  }
  At(600U);assert(Px4Lite_StartupToneCnsResult(201U)==PX4LITE_OK && !m.hz);
  Start(PX4LITE_CNS_TONE_ESTOP,202U);
  for(i=0U;i<4U;i++) { At(610U+i*1000U);assert(m.hz==((i&1U)?0U:3136U)); }
  At(4610U);assert(Px4Lite_StartupToneCnsResult(202U)==PX4LITE_OK && !m.hz);
}
/** @brief 降落间歇循环永不自动完成；取消立即撤销token，下一sensor轮静音。 */
static void TestLandAndTokens(void)
{
  unsigned i,tones;Begin(1U);Start(PX4LITE_CNS_TONE_LAND,301U);
  for(i=0U;i<8U;i++) {
    At(i*500U);assert(m.hz==((i&1U)?0U:3136U));
    assert(Px4Lite_StartupToneCnsResult(301U)==PX4LITE_BUSY);
  }
  tones=m.tones;Px4Lite_StartupToneCnsCancel(0U);
  assert(Px4Lite_StartupToneCnsResult(301U)==PX4LITE_IDLE && m.tones==tones);
  At(3510U);assert(!m.hz);
  Start(PX4LITE_CNS_TONE_ARM,302U);At(3520U);assert(m.hz==880U);
  Start(PX4LITE_CNS_TONE_DISARM,303U);assert(Px4Lite_StartupToneCnsResult(302U)==PX4LITE_IDLE);
  At(3530U);assert(m.hz==3136U);
  Px4Lite_StartupToneCnsCancel(0U);At(3540U);assert(!m.hz);
}
/** @brief 调速复用现有成功四音与绿青闪灯，尾静音结束才完成。 */
static void TestSuccessSequence(void)
{
  static const uint16_t hz[]={1568U,0U,1976U,0U,2349U,0U,3136U,0U};
  static const uint16_t at[]={0U,60U,80U,140U,160U,240U,260U,420U};
  unsigned i;Begin(1U);Start(PX4LITE_CNS_TONE_SET_MOTOR_PWM,401U);
  for(i=0U;i<8U;i++) {
    At(at[i]);assert(m.hz==hz[i] && Px4Lite_StartupToneCnsResult(401U)==PX4LITE_BUSY);
#if PX4LITE_ENABLE_STARTUP_RGB
    if(hz[i]) { assert(m.rgb[1]>=32U && m.rgb[0]<=8U && m.rgb[2]<=48U); }
    else { assert(!(m.rgb[0]|m.rgb[1]|m.rgb[2])); }
#endif
  }
  At(519U);assert(Px4Lite_StartupToneCnsResult(401U)==PX4LITE_BUSY);
  At(520U);assert(Px4Lite_StartupToneCnsResult(401U)==PX4LITE_OK && !m.hz);
}
/** @brief 急停登记连续Cancel+Start不能丢失clear_all，旧启动/队列与旧token均不得复活。 */
static void TestEmergencyPreemption(void)
{
  Begin(0U);At(1100U);
  Px4Lite_StartupToneNotifyRpiResult(0U);Px4Lite_StartupToneNotifyModuleConnection(1U);
  Start(PX4LITE_CNS_TONE_ARM,501U);At(1110U);assert(m.hz==880U);
  Px4Lite_StartupToneNotifyRpiResult(0U);
  Px4Lite_StartupToneCnsCancel(1U);Start(PX4LITE_CNS_TONE_ESTOP,502U);
  assert(Px4Lite_StartupToneCnsResult(501U)==PX4LITE_IDLE);
  At(1120U);assert(m.hz==3136U && !Status().feedback_pending && !Status().feedback_active);
  At(1220U);assert(!m.hz);At(1320U);assert(m.hz==3136U);At(1420U);assert(!m.hz);
  At(1520U);assert(Px4Lite_StartupToneCnsResult(502U)==PX4LITE_OK);
  At(5000U);assert(!m.hz && !Status().feedback_pending && !Status().feedback_active);
#if PX4LITE_ENABLE_STARTUP_TONE
  assert(Status().state==PX4LITE_TONE_DONE);
#endif
}
/** @brief 输出失败返回IO_ERROR；失败请求不能被后来的时间流逝改写成成功。 */
static void TestFailuresAndSupersession(void)
{
  Begin(1U);m.tone_rc=PX4LITE_IO_ERROR;Start(PX4LITE_CNS_TONE_ARM,601U);At(0U);
  assert(Px4Lite_StartupToneCnsResult(601U)==PX4LITE_IO_ERROR && Status().cns_error_count==1U);
  At(1000U);assert(Px4Lite_StartupToneCnsResult(601U)==PX4LITE_IO_ERROR);
  m.tone_rc=PX4LITE_OK;m.init_rc=PX4LITE_IO_ERROR;Start(PX4LITE_CNS_TONE_ARM,605U);At(1010U);
  assert(Px4Lite_StartupToneCnsResult(605U)==PX4LITE_BUSY && Status().cns_error_count==1U);
  At(2000U);assert(Px4Lite_StartupToneCnsResult(605U)==PX4LITE_IO_ERROR && Status().cns_error_count==2U);
  Begin(1U);Start(PX4LITE_CNS_TONE_ARM,602U);At(0U);m.replace_token=603U;At(1000U);
  assert(Px4Lite_StartupToneCnsResult(602U)==PX4LITE_IDLE);
  assert(Px4Lite_StartupToneCnsResult(603U)==PX4LITE_BUSY);
  At(1010U);assert(m.hz==3136U && Px4Lite_StartupToneCnsResult(603U)==PX4LITE_BUSY);
#if PX4LITE_ENABLE_STARTUP_RGB
  Begin(1U);m.color_rc=PX4LITE_IO_ERROR;Start(PX4LITE_CNS_TONE_SET_MOTOR_PWM,604U);At(0U);
  assert(Px4Lite_StartupToneCnsResult(604U)==PX4LITE_IO_ERROR && Status().cns_error_count==1U);
#endif
}
/** @brief 普通CNS只暂停旧generic音符时间轴，完成后从剩余时间续播。 */
static void TestGenericPause(void)
{
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  Begin(1U);Px4Lite_StartupToneNotifyRpiResult(1U);At(0U);assert(m.hz==1568U);
  At(20U);Start(PX4LITE_CNS_TONE_ARM,701U);At(20U);assert(m.hz==880U);
  At(1020U);assert(Px4Lite_StartupToneCnsResult(701U)==PX4LITE_OK);
  At(1030U);assert(m.hz==1568U);At(1069U);assert(m.hz==1568U);At(1070U);assert(!m.hz);
#endif
}
/** @brief 微秒和毫秒分别自然回绕时，1秒最低时长不变且不提前提交完成。 */
static void TestClockWrap(void)
{
  Begin(1U);s_base_us=UINT32_MAX-500000U;s_base_ms=UINT32_MAX-500U;
  Start(PX4LITE_CNS_TONE_ARM,801U);At(0U);assert(m.hz==880U);
  At(999U);assert(Px4Lite_StartupToneCnsResult(801U)==PX4LITE_BUSY && m.hz==880U);
  At(1000U);assert(Px4Lite_StartupToneCnsResult(801U)==PX4LITE_OK && !m.hz);
}
#endif

int main(void)
{
#if PX4LITE_ENABLE_CNS_CONTROL
  TestLongTones();TestQuickTones();TestLandAndTokens();TestSuccessSequence();
  TestEmergencyPreemption();TestFailuresAndSupersession();TestGenericPause();TestClockWrap();
#else
  unsigned tones,colors;Begin(1U);tones=m.tones;colors=m.colors;
  Px4Lite_StartupToneCnsStart(PX4LITE_CNS_TONE_ARM,1U);Px4Lite_StartupToneCnsCancel(1U);
  assert(Px4Lite_StartupToneCnsResult(1U)==PX4LITE_IDLE && m.tones==tones && m.colors==colors);
#endif
  puts("CNS tone V0.1: PASS");return 0;
}
