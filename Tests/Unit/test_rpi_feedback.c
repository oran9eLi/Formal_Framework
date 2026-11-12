/**
 * @file test_rpi_feedback.c
 * @brief 真实声光owner的双鸣相位、排队/启动争用、回绕和故障隔离回归。
 * @details 不启动RTOS；IO替身拒绝临界区内硬件操作，不能用host代替实板声光验收。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static unsigned s_critical;
#define taskENTER_CRITICAL() (s_critical++)
#define taskEXIT_CRITICAL() do { assert(s_critical);s_critical--; } while(0)
#include "px4lite_startup_tone.h"
#include "px4lite_config.h"
#include "px4lite_platform.h"

/** @brief 平台只存实际接受的输出，错误/BUSY不会偷偷更新颜色或频率。 */
static struct {
  uint32_t ms; /**< 独立真实毫秒时钟。 */
  uint16_t hz; /**< 最近接受频率，Hz。 */
  uint8_t rgb[3]; /**< 最近接受RGB，每通道0～255。 */
  unsigned bi,tones,ri,colors,services,deinit; /**< 初始化、提交和服务累计调用次数。 */
  Px4Lite_Result_t bi_result,tone_result,ri_result,color_result,service_result; /**< 可注入硬件结果。 */
} m;
uint32_t Px4Lite_PlatformGetMs(void) { return m.ms; }
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void)
{ assert(!s_critical);m.bi++;return m.bi_result; }
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t hz)
{ assert(!s_critical);m.tones++;if(hz && m.tone_result!=PX4LITE_OK) { return m.tone_result; }m.hz=hz;return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void)
{ assert(!s_critical);m.ri++;return m.ri_result; }
void Px4Lite_PlatformRgbDeInit(void) { assert(!s_critical);m.deinit++; }
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t r,uint8_t g,uint8_t b)
{ assert(!s_critical);m.colors++;if(m.color_result==PX4LITE_OK) { m.rgb[0]=r;m.rgb[1]=g;m.rgb[2]=b; }return m.color_result; }
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t now)
{ assert(!s_critical && now==m.ms);m.services++;return m.service_result; }
#include "../../Framework/Src/px4lite_startup_tone.c"

/** @brief 公共状态使用生产Copy接口读取，测试不修改模块私有状态。 */
static Px4Lite_StartupToneStatus_t Status(void)
{ Px4Lite_StartupToneStatus_t out;assert(Px4Lite_StartupToneCopyStatus(&out)==PX4LITE_OK);return out; }
/** @brief 两时钟可分别回绕，不由微秒整除伪造毫秒。 */
typedef struct { uint32_t us,ms; } Clock;
static void At(Clock start,uint32_t elapsed_ms)
{ m.ms=start.ms+elapsed_ms;Px4Lite_StartupToneRun(start.us+elapsed_ms*1000U);assert(!s_critical); }
/** @brief 启动一次新owner事务，旧测试场景已经排空所有提示。 */
static Clock Begin(uint32_t us,uint32_t ms)
{
  Clock out={us,ms};memset(&m,0,sizeof(m));m.ms=ms;Px4Lite_StartupToneInit(us);
#if PX4LITE_ENABLE_STARTUP_TONE
  out.us+=2625000U;out.ms+=2625U;At(out,0U);assert(Status().state==PX4LITE_TONE_DONE);
#endif
  return out;
}
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
/** @brief 纯软件Notify不允许碰硬件，随后恰好两次非零上升/下降及两枚白闪。 */
static void TestTwoPulses(void)
{
  Clock c=Begin(0U,0U);unsigned tones=m.tones,colors=m.colors;Px4Lite_StartupToneStatus_t before=Status();
  Px4Lite_StartupToneNotifyRpiCommand();assert(m.tones==tones && m.colors==colors);
  At(c,10U);assert(m.hz==2400U && Status().feedback_active);
#if PX4LITE_ENABLE_STARTUP_RGB
  assert(m.rgb[0]==48U && m.rgb[1]==48U && m.rgb[2]==48U);
#endif
  At(c,109U);assert(m.hz==2400U);At(c,110U);assert(m.hz==0U);
#if PX4LITE_ENABLE_STARTUP_RGB
  assert(!(m.rgb[0]|m.rgb[1]|m.rgb[2]));
#endif
  At(c,210U);assert(m.hz==2400U);
#if PX4LITE_ENABLE_STARTUP_RGB
  assert(m.rgb[0]==48U && m.rgb[1]==48U && m.rgb[2]==48U);
#endif
  At(c,310U);assert(m.hz==0U);At(c,410U);assert(!Status().feedback_active && !Status().feedback_pending);
  assert(m.tones==tones+4U && Status().feedback_started==before.feedback_started+1U && Status().feedback_finished==before.feedback_finished+1U);
#if PX4LITE_ENABLE_STARTUP_RGB && PX4LITE_ENABLE_STARTUP_TONE
  assert((m.rgb[0]|m.rgb[1]|m.rgb[2]) && !(m.rgb[0]==48U && m.rgb[1]==48U && m.rgb[2]==48U));
#endif
}
/** @brief 启动前通知不会被Init抹掉；启动旋律优先，禁用启动音时首请求才初始化。 */
static void TestBeforeInitAndStartupPriority(void)
{
  Clock c={10000000U,10000U};Px4Lite_StartupToneStatus_t before=Status();
  memset(&m,0,sizeof(m));m.ms=c.ms;Px4Lite_StartupToneNotifyRpiCommand();assert(!m.bi && !m.ri);
  Px4Lite_StartupToneInit(c.us);
#if PX4LITE_ENABLE_STARTUP_TONE
  At(c,999U);assert(Status().feedback_pending==1U && !Status().feedback_active && m.hz==0U);
  At(c,1000U);assert(Status().state==PX4LITE_TONE_PLAYING && !Status().feedback_active && m.hz==1760U);
  c.us+=2625000U;c.ms+=2625U;
#else
  assert(!m.bi && !m.ri && !m.tones && !m.colors);
#endif
  At(c,0U);assert(Status().feedback_active && m.hz==2400U && Status().feedback_received==before.feedback_received+1U);
  At(c,100U);At(c,200U);At(c,300U);At(c,400U);assert(!Status().feedback_active && !Status().feedback_pending);
}
/** @brief 长时间未调度不会跳过第二声，每个间隙仍至少100ms，不用补播循环。 */
static void TestDelayedServiceAndWrap(void)
{
  Clock c=Begin(UINT32_MAX-2675000U,UINT32_MAX-2675U);unsigned tones=m.tones;
  Px4Lite_StartupToneNotifyRpiCommand();At(c,0U);assert(m.hz==2400U);
  At(c,500U);assert(m.hz==0U && Status().feedback_active);
  At(c,510U);assert(m.hz==0U);
  At(c,1000U);assert(m.hz==2400U);
  At(c,1500U);assert(m.hz==0U);
  At(c,2000U);assert(!Status().feedback_active && m.tones==tones+4U);
}
/** @brief 8枚有界pending不截断当前两声，队满计数与指令执行链相互独立。 */
static void TestBoundedQueue(void)
{
  Clock c=Begin(30000000U,30000U);unsigned i;Px4Lite_StartupToneStatus_t before=Status();
  for(i=0U;i<10U;i++) { Px4Lite_StartupToneNotifyRpiCommand(); }
  for(i=0U;i<8U;i++) {
    uint32_t start=i*410U;
    At(c,start);assert(m.hz==2400U && Status().feedback_pending==7U-i);
    At(c,start+100U);assert(m.hz==0U);At(c,start+200U);assert(m.hz==2400U);
    At(c,start+300U);assert(m.hz==0U);At(c,start+400U);assert(!Status().feedback_active);
  }
  assert(Status().feedback_received==before.feedback_received+10U && Status().feedback_dropped==before.feedback_dropped+2U);
  assert(Status().feedback_finished==before.feedback_finished+8U && Status().feedback_pending==0U);
}
/** @brief 正在播放时新通知只排队，不能把已进入间隙的第一声重启。 */
static void TestQueueDuringActive(void)
{
  Clock c=Begin(40000000U,40000U);Px4Lite_StartupToneStatus_t before=Status();
  Px4Lite_StartupToneNotifyRpiCommand();At(c,0U);At(c,100U);
  Px4Lite_StartupToneNotifyRpiCommand();At(c,110U);assert(m.hz==0U && Status().feedback_pending==1U);
  At(c,200U);assert(m.hz==2400U);At(c,300U);At(c,400U);assert(Status().feedback_pending==1U);
  At(c,410U);assert(m.hz==2400U && Status().feedback_pending==0U);
  At(c,510U);At(c,610U);At(c,710U);At(c,810U);assert(Status().feedback_finished==before.feedback_finished+2U);
}
/** @brief 蜂鸣器失败只静音，RGB仍可双闪；后续请求允许2秒恢复。 */
static void TestBuzzerFailureIsolation(void)
{
  Clock c=Begin(50000000U,50000U);Px4Lite_StartupToneStatus_t before=Status();
  m.tone_result=PX4LITE_IO_ERROR;Px4Lite_StartupToneNotifyRpiCommand();At(c,0U);
  assert(m.hz==0U && Status().feedback_buzzer_errors==before.feedback_buzzer_errors+1U);
#if PX4LITE_ENABLE_STARTUP_RGB
  assert(m.rgb[0]==48U && m.rgb[1]==48U && m.rgb[2]==48U);
#endif
  At(c,100U);At(c,200U);At(c,300U);At(c,400U);assert(!Status().feedback_active);
  m.tone_result=PX4LITE_OK;Px4Lite_StartupToneNotifyRpiCommand();At(c,2100U);assert(m.hz==2400U);
  At(c,2200U);At(c,2300U);At(c,2400U);At(c,2500U);
}
#if PX4LITE_ENABLE_STARTUP_RGB
/** @brief RGB BUSY不覆盖颜色，恢复只取当前相位；RGB错误不会切断蜂鸣器第二声。 */
static void TestRgbBusyAndFailure(void)
{
  Clock c=Begin(60000000U,60000U);unsigned colors=m.colors,ri;uint32_t errors=Status().rgb_error_count;
  m.service_result=PX4LITE_BUSY;Px4Lite_StartupToneNotifyRpiCommand();At(c,0U);
  assert(m.hz==2400U && m.colors==colors);
  m.service_result=PX4LITE_OK;At(c,20U);assert(m.rgb[0]==48U && m.rgb[1]==48U && m.rgb[2]==48U);
  At(c,100U);At(c,200U);At(c,300U);At(c,400U);
  c.us+=1000000U;c.ms+=1000U;m.service_result=PX4LITE_IO_ERROR;
  Px4Lite_StartupToneNotifyRpiCommand();At(c,0U);ri=m.ri;
  assert(m.hz==2400U && Status().rgb_error_count==errors+1U);
  m.service_result=PX4LITE_OK;At(c,100U);At(c,200U);assert(m.hz==2400U && m.ri==ri);
  At(c,300U);At(c,400U);At(c,2000U);assert(m.ri==ri+1U && Status().rgb_ready);
}
#if PX4LITE_ENABLE_STARTUP_TONE
/** @brief 与未覆盖的渐变对照相位一致，提示结束不重播旧颜色/启动音。 */
static void TestAmbientContinuity(void)
{
  Clock c;uint8_t expected[3];
  c=Begin(1000000U,1000U);At(c,400U);memcpy(expected,m.rgb,3U);
  c=Begin(1000000U,1000U);Px4Lite_StartupToneNotifyRpiCommand();At(c,0U);
  At(c,100U);At(c,200U);At(c,300U);At(c,400U);assert(memcmp(expected,m.rgb,3U)==0);
  assert(Status().state==PX4LITE_TONE_DONE);
}
#endif
#endif
/** @brief 初始化故障也可由提示独立恢复，不把反馈成功改写为启动旋律成功。 */
static void TestStartupBuzzerInitFailure(void)
{
  Clock c={70000000U,70000U};unsigned attempts;uint32_t before=Status().feedback_buzzer_errors;
  memset(&m,0,sizeof(m));m.ms=c.ms;m.bi_result=PX4LITE_IO_ERROR;
  Px4Lite_StartupToneInit(c.us);Px4Lite_StartupToneNotifyRpiCommand();At(c,0U);
  assert(Status().feedback_buzzer_result!=PX4LITE_OK);attempts=m.bi;
  At(c,100U);At(c,200U);At(c,300U);At(c,400U);assert(m.bi==attempts);
  m.bi_result=PX4LITE_OK;Px4Lite_StartupToneNotifyRpiCommand();At(c,2100U);assert(m.hz==2400U);
  At(c,2200U);At(c,2300U);At(c,2400U);At(c,2500U);
  assert(Status().feedback_buzzer_errors>=before);
}
#endif
int main(void)
{
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  TestTwoPulses();TestBeforeInitAndStartupPriority();TestDelayedServiceAndWrap();TestBoundedQueue();TestQueueDuringActive();TestBuzzerFailureIsolation();
#if PX4LITE_ENABLE_STARTUP_RGB
  TestRgbBusyAndFailure();
#if PX4LITE_ENABLE_STARTUP_TONE
  TestAmbientContinuity();
#endif
#endif
  TestStartupBuzzerInitFailure();
  puts("RPi feedback owner: two pulses / white double flash / bounded queue / pre-init+startup / delayed phases / wrap / independent failure / ambient PASS");
#else
  Clock c=Begin(0U,0U);unsigned tones=m.tones;Px4Lite_StartupToneNotifyRpiCommand();At(c,1U);
  assert(!Status().feedback_received && !Status().feedback_pending && !Status().feedback_active && m.tones==tones);
  puts("RPi feedback disabled: notification no-op, original startup behavior retained PASS");
#endif
  assert(!s_critical);return 0;
}
