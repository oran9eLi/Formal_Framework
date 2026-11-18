/**
 * @file test_rpi_feedback.c
 * @brief 真实声光owner的命令/模块四类事件旋律、混合FIFO和故障隔离回归。
 * @details 不启动RTOS；IO替身禁止临界区硬件操作；host不代替实际声光验收。
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
/** @brief 仅保存平台实际接受的输出；BUSY和错误不偷偷提交颜色或频率。 */
static struct {
  uint32_t ms; /**< 独立真实毫秒时钟。 */
  uint16_t hz; /**< 最近实际接受频率，Hz。 */
  uint8_t rgb[3]; /**< 最近实际接受RGB，0～255。 */
  unsigned bi,tones,ri,colors,services,deinit; /**< IO调用累计次数。 */
  Px4Lite_Result_t bi_result,tone_result,ri_result,color_result,service_result; /**< 注入返回结果。 */
} m;
/** @brief 事件播放器专项不运行电机，背景音乐不得制造额外输出。 */
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out) { (void)out;return PX4LITE_NOT_READY; }
/** @brief 原事件夹具新增只读门控保持上锁，原旋律/灯断言不变。 */
Px4Lite_Result_t Px4Lite_ControlGetMotorLocked(uint8_t *locked) { *locked=1U;return PX4LITE_OK; }
/** @brief 该夹具不建立控制模块运行事实。 */
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id,Px4Lite_ModuleStatus_t *out)
{ (void)id;(void)out;return PX4LITE_NOT_READY; }
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
/** @brief 用公开快照读取，不改写真实播放器私有状态。 */
static Px4Lite_StartupToneStatus_t Status(void)
{ Px4Lite_StartupToneStatus_t out;assert(Px4Lite_StartupToneCopyStatus(&out)==PX4LITE_OK);return out; }
/** @brief 独立微秒与毫秒时钟，允许分别自然回绕。 */
typedef struct { uint32_t us,ms; } Clock;
static void At(Clock start,uint32_t elapsed_ms)
{ m.ms=start.ms+elapsed_ms;Px4Lite_StartupToneRun(start.us+elapsed_ms*1000U);assert(!s_critical); }
/** @brief 前一场景排空后初始化owner，不偷改结果FIFO。 */
static Clock Begin(uint32_t us,uint32_t ms)
{
  Clock out={us,ms};memset(&m,0,sizeof(m));m.ms=ms;Px4Lite_StartupToneInit(us);
#if PX4LITE_ENABLE_STARTUP_TONE
  out.us+=2625000U;out.ms+=2625U;At(out,0U);assert(Status().state==PX4LITE_TONE_DONE);
#endif
  return out;
}
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
/** @brief 用户确认的外部音符/间隙时间轴，尾静音另由生产配置定义。 */
static const uint16_t success_hz[]={1568U,0U,1976U,0U,2349U,0U,3136U,0U};
static const uint16_t success_at[]={0U,60U,80U,140U,160U,240U,260U,420U};
static const uint16_t failure_hz[]={880U,0U,659U,0U,440U,0U};
static const uint16_t failure_at[]={0U,120U,160U,300U,340U,560U};
static uint32_t Duration(uint8_t succeeded)
{ return (succeeded?420U:560U)+PX4LITE_RPI_FEEDBACK_TAIL_MS; }
/** @brief 明确验证成功亮绿/青与失败低亮暗红，静音相位为黑色。 */
static void CheckColor(uint8_t succeeded,uint16_t hz)
{
#if PX4LITE_ENABLE_STARTUP_RGB
  if(!hz) { assert(!(m.rgb[0]|m.rgb[1]|m.rgb[2])); }
  else if(succeeded) { assert(m.rgb[1]>=32U && m.rgb[0]<=8U && m.rgb[0]<=48U && m.rgb[1]<=48U && m.rgb[2]<=48U); }
  else { assert(m.rgb[0]==12U && m.rgb[1]==1U && m.rgb[2]==0U); }
#else
  (void)succeeded;(void)hz;
#endif
}
/** @brief 从真实平台输出验证每个音符和间隙的边界，不能靠私有phase自证正确。 */
static void Play(Clock c,uint32_t start,uint8_t succeeded)
{
  const uint16_t *hz=succeeded?success_hz:failure_hz;
  const uint16_t *at=succeeded?success_at:failure_at;
  unsigned i,count=succeeded?8U:6U;
  for(i=0U;i<count;i++) {
    if(i) { At(c,start+at[i]-1U);assert(m.hz==hz[i-1U]); }
    At(c,start+at[i]);assert(m.hz==hz[i] && Status().feedback_active && Status().feedback_succeeded==succeeded);
    CheckColor(succeeded,hz[i]);
  }
  At(c,start+Duration(succeeded)-1U);assert(Status().feedback_active && !m.hz);
  At(c,start+Duration(succeeded));assert(!Status().feedback_active && !m.hz);
}
/** @brief 成功与失败的精确音高/节奏及旧入口不可误报成功。 */
static void TestResultPatterns(void)
{
  Clock c=Begin(0U,0U);Px4Lite_StartupToneStatus_t before=Status();unsigned tones=m.tones,colors=m.colors;
  Px4Lite_StartupToneNotifyRpiResult(1U);assert(m.tones==tones && m.colors==colors);Play(c,10U,1U);
  assert(Status().feedback_started==before.feedback_started+1U && Status().feedback_finished==before.feedback_finished+1U);
  assert(!Status().feedback_pending);
  c=Begin(10000000U,10000U);Px4Lite_StartupToneNotifyRpiResult(0U);Play(c,0U,0U);
  c=Begin(20000000U,20000U);Px4Lite_StartupToneNotifyRpiCommand();Play(c,0U,0U);
  c=Begin(30000000U,30000U);Px4Lite_StartupToneNotifyRpiResult(2U);Play(c,0U,0U);
}
/** @brief 初始化前结果类型和顺序保留，启动旋律不被结果抢占；Tone0首结果才IO。 */
static void TestBeforeInitAndStartupPriority(void)
{
  Clock c={40000000U,40000U};Px4Lite_StartupToneStatus_t before=Status();
  memset(&m,0,sizeof(m));m.ms=c.ms;
  Px4Lite_StartupToneNotifyRpiResult(1U);Px4Lite_StartupToneNotifyRpiResult(0U);assert(!m.bi && !m.ri);
  Px4Lite_StartupToneInit(c.us);
#if PX4LITE_ENABLE_STARTUP_TONE
  At(c,999U);assert(Status().feedback_pending==2U && !Status().feedback_active && !m.hz);
  At(c,1000U);assert(Status().state==PX4LITE_TONE_PLAYING && !Status().feedback_active && m.hz==1760U);
  c.us+=2625000U;c.ms+=2625U;
#else
  assert(!m.bi && !m.ri && !m.tones && !m.colors);
#endif
  Play(c,0U,1U);Play(c,Duration(1U)+10U,0U);
  assert(!Status().feedback_pending && Status().feedback_received==before.feedback_received+2U);
}
/** @brief 延迟调度与双时钟回绕不能跳过任何音符或间隙；每Run只推进一相位。 */
static void TestDelayedServiceAndWrap(void)
{
  Clock c=Begin(UINT32_MAX-2700000U,UINT32_MAX-2700U);unsigned i,tones=m.tones;
  Px4Lite_StartupToneNotifyRpiResult(1U);
  for(i=0U;i<8U;i++) { At(c,i*500U);assert(m.hz==success_hz[i] && Status().feedback_active); }
  At(c,4000U);assert(!Status().feedback_active && m.tones==tones+8U);
}
/** @brief 混合结果按FIFO顺序播出，8项上限和索引回绕不截断当前旋律。 */
static void TestBoundedMixedQueue(void)
{
  Clock c=Begin(50000000U,50000U);unsigned i;uint32_t start=0U;Px4Lite_StartupToneStatus_t before=Status();
  for(i=0U;i<10U;i++) { Px4Lite_StartupToneNotifyRpiResult((uint8_t)((i%2U)==0U)); }
  for(i=0U;i<8U;i++) {
    uint8_t succeeded=(uint8_t)((i%2U)==0U);Play(c,start,succeeded);
    assert(Status().feedback_pending==7U-i);start+=Duration(succeeded)+10U;
  }
  assert(Status().feedback_received==before.feedback_received+10U && Status().feedback_dropped==before.feedback_dropped+2U);
  assert(Status().feedback_finished==before.feedback_finished+8U && !Status().feedback_pending);
}
/** @brief 正在播放失败旋律时新成功只排队，不会重启或混淆当前结果。 */
static void TestQueueDuringActive(void)
{
  Clock c=Begin(60000000U,60000U);
  Px4Lite_StartupToneNotifyRpiResult(0U);At(c,0U);At(c,120U);assert(!m.hz);
  Px4Lite_StartupToneNotifyRpiResult(1U);At(c,130U);assert(!m.hz && !Status().feedback_succeeded && Status().feedback_pending==1U);
  At(c,160U);assert(m.hz==659U);At(c,300U);At(c,340U);assert(m.hz==440U);
  At(c,560U);At(c,Duration(0U));assert(Status().feedback_pending==1U);
  Play(c,Duration(0U)+10U,1U);assert(!Status().feedback_pending);
}
/** @brief 蜂鸣器故障不抑制结果灯效，2秒后新结果允许独立初始化恢复。 */
static void TestBuzzerFailureIsolation(void)
{
  Clock c=Begin(70000000U,70000U);uint32_t errors=Status().feedback_buzzer_errors;unsigned i;
  m.tone_result=PX4LITE_IO_ERROR;Px4Lite_StartupToneNotifyRpiResult(1U);At(c,0U);
  assert(!m.hz && Status().feedback_buzzer_errors==errors+1U);CheckColor(1U,1568U);
  for(i=1U;i<8U;i++) { At(c,success_at[i]); }
  At(c,Duration(1U));assert(!Status().feedback_active);
  m.tone_result=PX4LITE_OK;Px4Lite_StartupToneNotifyRpiResult(0U);Play(c,2100U,0U);
}
#if PX4LITE_ENABLE_STARTUP_RGB
/** @brief RGB忙/错误不覆盖DMA也不中止旋律，恢复只发送当前灯色，不补旧闪光。 */
static void TestRgbBusyAndFailure(void)
{
  Clock c=Begin(80000000U,80000U);unsigned colors=m.colors,ri,i;uint32_t errors=Status().rgb_error_count;
  m.service_result=PX4LITE_BUSY;Px4Lite_StartupToneNotifyRpiResult(1U);At(c,0U);assert(m.hz==1568U && m.colors==colors);
  m.service_result=PX4LITE_OK;At(c,20U);CheckColor(1U,1568U);
  for(i=1U;i<8U;i++) { At(c,success_at[i]);assert(m.hz==success_hz[i]); }
  At(c,Duration(1U));c.us+=1000000U;c.ms+=1000U;
  m.service_result=PX4LITE_IO_ERROR;Px4Lite_StartupToneNotifyRpiResult(0U);At(c,0U);ri=m.ri;
  assert(m.hz==880U && Status().rgb_error_count==errors+1U);
  m.service_result=PX4LITE_OK;
  for(i=1U;i<6U;i++) { At(c,failure_at[i]);assert(m.hz==failure_hz[i] && m.ri==ri); }
  At(c,Duration(0U));At(c,2000U);assert(m.ri==ri+1U && Status().rgb_ready);
}
#if PX4LITE_ENABLE_STARTUP_TONE
/** @brief 提示覆盖不冻结环境渐变相位，完成后与未覆盖同一时刻颜色相同。 */
static void TestAmbientContinuity(void)
{
  Clock c;uint8_t expected[3];
  c=Begin(1000000U,1000U);At(c,Duration(1U));memcpy(expected,m.rgb,3U);
  c=Begin(1000000U,1000U);Px4Lite_StartupToneNotifyRpiResult(1U);Play(c,0U,1U);
  assert(memcmp(expected,m.rgb,3U)==0 && Status().state==PX4LITE_TONE_DONE);
}
#endif
#endif
/** @brief 启动蜂鸣器初始化失败不虚构成功，后续结果可恢复且不重播启动旋律。 */
static void TestStartupBuzzerInitFailure(void)
{
  Clock c={90000000U,90000U};unsigned attempts,i;
  memset(&m,0,sizeof(m));m.ms=c.ms;m.bi_result=PX4LITE_IO_ERROR;
  Px4Lite_StartupToneInit(c.us);Px4Lite_StartupToneNotifyRpiResult(0U);At(c,0U);
  assert(Status().feedback_buzzer_result!=PX4LITE_OK);attempts=m.bi;
  for(i=1U;i<6U;i++) { At(c,failure_at[i]); }
  At(c,Duration(0U));assert(m.bi==attempts && !Status().feedback_active);
  m.bi_result=PX4LITE_OK;Px4Lite_StartupToneNotifyRpiResult(1U);Play(c,2100U,1U);
#if PX4LITE_ENABLE_STARTUP_TONE
  assert(Status().state==PX4LITE_TONE_ERROR);
#endif
}
#endif
#if PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK
/** @brief 用户已确认的插入/拔出两音时间轴，包含尾静音相位起点。 */
static const uint16_t connected_hz[]={1047U,0U,2093U,0U};
static const uint16_t connected_at[]={0U,80U,120U,300U};
static const uint16_t disconnected_hz[]={2093U,0U,1047U,0U};
static const uint16_t disconnected_at[]={0U,140U,200U,400U};
static uint32_t ModuleDuration(uint8_t connected)
{ return (connected?300U:400U)+PX4LITE_RPI_FEEDBACK_TAIL_MS; }
/** @brief 只依据实际平台频率/颜色和公开kind验证模块旋律，不把模块事件当命令结果。 */
static void PlayModule(Clock c,uint32_t start,uint8_t connected)
{
  const uint16_t *hz=connected?connected_hz:disconnected_hz;
  const uint16_t *at=connected?connected_at:disconnected_at;
  uint8_t kind=connected?PX4LITE_FEEDBACK_MODULE_CONNECTED:PX4LITE_FEEDBACK_MODULE_DISCONNECTED;
  unsigned i;
  for(i=0U;i<4U;i++) {
    if(i) { At(c,start+at[i]-1U);assert(m.hz==hz[i-1U]); }
    At(c,start+at[i]);assert(m.hz==hz[i] && Status().feedback_active && Status().feedback_kind==kind);
    assert(!Status().feedback_succeeded);
#if PX4LITE_ENABLE_STARTUP_RGB
    if(!hz[i]) { assert(!(m.rgb[0]|m.rgb[1]|m.rgb[2])); }
    else if(connected) { assert(m.rgb[0]==0U && m.rgb[1]>=16U && m.rgb[1]<=48U && m.rgb[2]>=32U && m.rgb[2]<=48U); }
    else { assert(m.rgb[0]==12U && m.rgb[1]==4U && m.rgb[2]==0U); }
#endif
  }
  At(c,start+ModuleDuration(connected)-1U);assert(Status().feedback_active && !m.hz);
  At(c,start+ModuleDuration(connected));assert(!Status().feedback_active && !m.hz && Status().feedback_kind==kind);
}
/** @brief 精确插拔旋律、通知无IO及未知bool值只能按中断处理。 */
static void TestModulePatterns(void)
{
  Clock c=Begin(100000000U,100000U);unsigned tones=m.tones,colors=m.colors;
  assert(Status().feedback_kind==PX4LITE_FEEDBACK_NONE);
  Px4Lite_StartupToneNotifyModuleConnection(1U);assert(m.tones==tones && m.colors==colors);PlayModule(c,0U,1U);
  c=Begin(110000000U,110000U);Px4Lite_StartupToneNotifyModuleConnection(0U);PlayModule(c,0U,0U);
  c=Begin(120000000U,120000U);Px4Lite_StartupToneNotifyModuleConnection(2U);PlayModule(c,0U,0U);
}
/** @brief 初始化前插拔事件保留，原启动旋律优先；Tone0首事件才访问输出。 */
static void TestModuleStartupAndLateWrap(void)
{
  Clock c={130000000U,130000U};unsigned i;
  memset(&m,0,sizeof(m));m.ms=c.ms;
  Px4Lite_StartupToneNotifyModuleConnection(1U);Px4Lite_StartupToneNotifyModuleConnection(0U);assert(!m.bi && !m.ri);
  Px4Lite_StartupToneInit(c.us);assert(Status().feedback_kind==PX4LITE_FEEDBACK_NONE);
#if PX4LITE_ENABLE_STARTUP_TONE
  At(c,999U);assert(Status().feedback_pending==2U && !Status().feedback_active);
  At(c,1000U);assert(m.hz==1760U && !Status().feedback_active);
  c.us+=2625000U;c.ms+=2625U;
#else
  assert(!m.bi && !m.ri && !m.tones && !m.colors);
#endif
  PlayModule(c,0U,1U);PlayModule(c,ModuleDuration(1U)+10U,0U);
  c=Begin(UINT32_MAX-2700000U,UINT32_MAX-2700U);Px4Lite_StartupToneNotifyModuleConnection(0U);
  for(i=0U;i<4U;i++) { At(c,i*700U);assert(m.hz==disconnected_hz[i] && Status().feedback_active); }
  At(c,2800U);assert(!Status().feedback_active);
}
/** @brief 插拔可独立填满8项，队满只丢提示；读写索引回绕保留顺序。 */
static void TestModuleQueueAndFailure(void)
{
  Clock c=Begin(140000000U,140000U);unsigned i;uint32_t start=0U;Px4Lite_StartupToneStatus_t before=Status();
  for(i=0U;i<10U;i++) { Px4Lite_StartupToneNotifyModuleConnection((uint8_t)((i%2U)==0U)); }
  for(i=0U;i<8U;i++) {
    uint8_t connected=(uint8_t)((i%2U)==0U);PlayModule(c,start,connected);start+=ModuleDuration(connected)+10U;
  }
  assert(Status().feedback_received==before.feedback_received+10U && Status().feedback_dropped==before.feedback_dropped+2U);
  c=Begin(150000000U,150000U);m.tone_result=PX4LITE_IO_ERROR;
  Px4Lite_StartupToneNotifyModuleConnection(1U);At(c,0U);assert(!m.hz && Status().feedback_buzzer_errors);
#if PX4LITE_ENABLE_STARTUP_RGB
  assert(m.rgb[2]==48U && m.rgb[1]==16U);
#endif
  for(i=1U;i<4U;i++) { At(c,connected_at[i]); }
  At(c,ModuleDuration(1U));m.tone_result=PX4LITE_OK;
  Px4Lite_StartupToneNotifyModuleConnection(0U);PlayModule(c,2100U,0U);
}
#if PX4LITE_ENABLE_STARTUP_RGB
/** @brief 单独模块功能下RGB BUSY不覆盖DMA，错误不挡声音，恢复仍只发送当前颜色。 */
static void TestModuleRgbFailureAndAmbient(void)
{
  Clock c=Begin(155000000U,155000U);unsigned i,colors=m.colors,ri;
  uint32_t errors=Status().rgb_error_count;
  m.service_result=PX4LITE_BUSY;Px4Lite_StartupToneNotifyModuleConnection(0U);At(c,0U);
  assert(m.hz==2093U && m.colors==colors && Status().feedback_kind==PX4LITE_FEEDBACK_MODULE_DISCONNECTED);
  m.service_result=PX4LITE_OK;At(c,20U);assert(m.rgb[0]==12U && m.rgb[1]==4U);
  for(i=1U;i<4U;i++) { At(c,disconnected_at[i]);assert(m.hz==disconnected_hz[i]); }
  At(c,ModuleDuration(0U));c.us+=1000000U;c.ms+=1000U;
  m.service_result=PX4LITE_IO_ERROR;Px4Lite_StartupToneNotifyModuleConnection(1U);At(c,0U);ri=m.ri;
  assert(m.hz==1047U && Status().rgb_error_count==errors+1U);
  m.service_result=PX4LITE_OK;
  for(i=1U;i<4U;i++) { At(c,connected_at[i]);assert(m.hz==connected_hz[i] && m.ri==ri); }
  At(c,ModuleDuration(1U));At(c,2000U);assert(m.ri==ri+1U && Status().rgb_ready);
#if PX4LITE_ENABLE_STARTUP_TONE
  {
    uint8_t expected[3];
    c=Begin(1000000U,1000U);At(c,ModuleDuration(0U));memcpy(expected,m.rgb,3U);
    c=Begin(1000000U,1000U);Px4Lite_StartupToneNotifyModuleConnection(0U);PlayModule(c,0U,0U);
    assert(memcmp(expected,m.rgb,3U)==0 && Status().state==PX4LITE_TONE_DONE);
  }
#endif
}
#endif
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
/** @brief 四类事件进入同一FIFO，模块不能继承命令成功位或改变原命令旋律。 */
static void TestFourKindsMixedQueue(void)
{
  Clock c=Begin(160000000U,160000U);uint32_t start=0U;unsigned i;
  Px4Lite_StartupToneNotifyRpiResult(0U);Px4Lite_StartupToneNotifyModuleConnection(1U);
  Px4Lite_StartupToneNotifyRpiResult(1U);Px4Lite_StartupToneNotifyModuleConnection(0U);
  for(i=0U;i<4U;i++) {
    if(i==0U) { Play(c,start,0U);assert(Status().feedback_kind==PX4LITE_FEEDBACK_COMMAND_FAILED);start+=Duration(0U)+10U; }
    if(i==1U) { PlayModule(c,start,1U);start+=ModuleDuration(1U)+10U; }
    if(i==2U) { Play(c,start,1U);assert(Status().feedback_kind==PX4LITE_FEEDBACK_COMMAND_SUCCEEDED);start+=Duration(1U)+10U; }
    if(i==3U) { PlayModule(c,start,0U); }
    assert(Status().feedback_pending==3U-i);
  }
}
#endif
#endif
int main(void)
{
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  TestResultPatterns();TestBeforeInitAndStartupPriority();TestDelayedServiceAndWrap();TestBoundedMixedQueue();TestQueueDuringActive();TestBuzzerFailureIsolation();
#if PX4LITE_ENABLE_STARTUP_RGB
  TestRgbBusyAndFailure();
#if PX4LITE_ENABLE_STARTUP_TONE
  TestAmbientContinuity();
#endif
#endif
  TestStartupBuzzerInitFailure();
  puts("RPi result feedback: selected success/failure notes+timing / bright-vs-dim LEDs / typed FIFO+overflow / pre-init+startup / delayed+wrap / independent failures / ambient PASS");
#else
  Clock c=Begin(0U,0U);unsigned tones=m.tones;
  Px4Lite_StartupToneNotifyRpiResult(1U);Px4Lite_StartupToneNotifyRpiResult(0U);Px4Lite_StartupToneNotifyRpiCommand();At(c,1U);
  assert(!Status().feedback_received && !Status().feedback_pending && !Status().feedback_active && m.tones==tones);
  puts("RPi result feedback disabled: result/legacy entry no-op; original startup retained PASS");
#endif
#if PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK
  TestModulePatterns();TestModuleStartupAndLateWrap();TestModuleQueueAndFailure();
#if PX4LITE_ENABLE_STARTUP_RGB
  TestModuleRgbFailureAndAmbient();
#endif
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  TestFourKindsMixedQueue();
#endif
  puts("Module feedback: selected insert/remove notes+timing / blue-cyan vs dim-orange / independent switch / FIFO+overflow / startup+wrap / command-kind separation PASS");
#else
  {
    Clock c=Begin(170000000U,170000U);unsigned tones=m.tones;uint32_t received=Status().feedback_received;
    Px4Lite_StartupToneNotifyModuleConnection(1U);Px4Lite_StartupToneNotifyModuleConnection(0U);At(c,1U);
    assert(m.tones==tones && Status().feedback_received==received && !Status().feedback_active);
  }
#endif
  assert(!s_critical);return 0;
}
