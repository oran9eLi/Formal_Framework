/**
 * @file test_motor_music.c
 * @brief 真实蜂鸣器owner的电机背景门控、暂停续播、抢占和功能关闭回归。
 * @details 默认注入原创短谱；可选直接引用用户曲谱验证263相位整圈，不复制第二份歌曲。
 *          平台替身记录实际请求，不代表实物验收。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static unsigned s_critical,s_scheduler;
#define taskENTER_CRITICAL() (s_critical++)
#define taskEXIT_CRITICAL() do { assert(s_critical);s_critical--; } while(0)
#define vTaskSuspendAll() (s_scheduler++)
#define xTaskResumeAll() (assert(s_scheduler),s_scheduler--,0)
#include "px4lite_startup_tone.h"
#include "px4lite_config.h"
#include "px4lite_platform.h"
#if PX4LITE_ENABLE_MOTOR_MUSIC && !defined(TEST_MOTOR_MUSIC_REAL_SCORE)
/** @brief 测试专用原创三音/休止相位，不与用户的歌曲数据相耦合。 */
typedef struct { uint16_t frequency_hz,duration_ms; } MotorMusicNote;
static const MotorMusicNote s_motor_music_score[]={{523U,100U},{0U,40U},{659U,120U},{0U,60U}};
#define MOTOR_MUSIC_SCORE_COUNT (sizeof(s_motor_music_score)/sizeof(s_motor_music_score[0]))
#define PX4LITE_MOTOR_MUSIC_SCORE_INJECTED 1
#endif
static struct {
  uint32_t ms;uint16_t hz;uint8_t rgb[3];unsigned tones,init,colors;
  Px4Lite_Result_t init_rc,tone_rc,motor_rc,lock_rc,status_rc;
  uint8_t locked,fresh;Px4Lite_State_t state;Px4Lite_MotorOutputs_t motor;
} m;
uint32_t Px4Lite_PlatformGetMs(void) { return m.ms; }
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out)
{ assert(s_scheduler && !s_critical);*out=m.motor;return m.motor_rc; }
Px4Lite_Result_t Px4Lite_ControlGetMotorLocked(uint8_t *out)
{ assert(s_scheduler && !s_critical);*out=m.locked;return m.lock_rc; }
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id,Px4Lite_ModuleStatus_t *out)
{ assert(s_scheduler && !s_critical && id==PX4LITE_MODULE_CONTROL);memset(out,0,sizeof(*out));out->state=m.state;return m.status_rc; }
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void)
{ assert(!s_scheduler && !s_critical);m.init++;return m.init_rc; }
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t hz)
{ assert(!s_scheduler && !s_critical);m.tones++;if(hz && m.tone_rc!=PX4LITE_OK) { return m.tone_rc; }m.hz=hz;return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void) { assert(!s_scheduler && !s_critical);return PX4LITE_OK; }
void Px4Lite_PlatformRgbDeInit(void) { assert(!s_scheduler && !s_critical); }
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t r,uint8_t g,uint8_t b)
{ assert(!s_scheduler && !s_critical);m.rgb[0]=r;m.rgb[1]=g;m.rgb[2]=b;m.colors++;return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t now)
{ assert(!s_scheduler && !s_critical && now==m.ms);return PX4LITE_OK; }
#include "../../Framework/Src/px4lite_startup_tone.c"
static uint32_t s_base_us,s_base_ms;
/** @brief 保持两类时钟独立推进，每次可自动发布新鲜的实际电机事实。 */
static void At(uint32_t elapsed_ms)
{
  m.ms=s_base_ms+elapsed_ms;
  if(m.fresh) { m.motor.header.sample_time_ms=m.ms;m.motor.header.publish_time_ms=m.ms; }
  Px4Lite_StartupToneRun(s_base_us+elapsed_ms*1000U);assert(!s_scheduler && !s_critical);
}
static Px4Lite_StartupToneStatus_t Status(void)
{ Px4Lite_StartupToneStatus_t out;assert(Px4Lite_StartupToneCopyStatus(&out)==PX4LITE_OK);return out; }
/** @brief 每个场景从停止状态开始，原启动音走完后才测试背景。 */
static void Begin(uint32_t us,uint32_t ms)
{
  memset(&m,0,sizeof(m));m.state=PX4LITE_STATE_ONLINE;m.fresh=1U;m.motor.header.valid=1U;
  m.motor.header.flags=PX4LITE_DATA_VALID;m.motor.run_state=1U;m.ms=ms;
  s_base_us=us;s_base_ms=ms;Px4Lite_StartupToneInit(us);
#if PX4LITE_ENABLE_STARTUP_TONE
  At(2625U);s_base_us+=2625000U;s_base_ms+=2625U;
#endif
}
#if PX4LITE_ENABLE_MOTOR_MUSIC && !defined(TEST_MOTOR_MUSIC_REAL_SCORE)
/** @brief 一路1001us且百分比为0照样启动；停机暂停相位内剩余，恢复和循环都准确。 */
static void TestPauseResumeAndLoop(void)
{
  Begin(0U,0U);At(0U);assert(!m.hz && !Status().music_gate);
  m.motor.pulse_us[2]=1001U;At(10U);assert(m.hz==523U && Status().music_running && !m.motor.duty_percent[2]);
  At(50U);assert(Status().music_remaining_us==60000U);
  m.motor.pulse_us[2]=1000U;At(70U);assert(!m.hz && !Status().music_running && Status().music_remaining_us==40000U);
  At(1000U);At(5000000U);assert(Status().music_remaining_us==40000U && !m.hz);
  m.motor.pulse_us[0]=1500U;At(5000010U);assert(m.hz==523U && Status().music_phase_index==0U && Status().music_remaining_us==40000U);
  At(5000050U);assert(!m.hz && Status().music_phase_index==1U);
  At(5000090U);assert(m.hz==659U && Status().music_phase_index==2U);
  At(5000210U);assert(!m.hz && Status().music_phase_index==3U);
  At(5000270U);assert(m.hz==523U && Status().music_phase_index==0U);
  m.motor.pulse_us[0]=1000U;At(5000280U);
}
/** @brief 缺topic、锁定、故障、无效或过龄快照都暂停，不能用旧PWM误唱。 */
static void TestGateFacts(void)
{
  unsigned mode;
  for(mode=0U;mode<9U;mode++) {
    Begin(2000000U,2000U);m.motor.pulse_us[3]=1400U;At(0U);assert(m.hz==523U);
    switch(mode) {
      case 0:m.locked=1U;break;
      case 1:m.state=PX4LITE_STATE_DEGRADED;break;
      case 2:m.motor.header.valid=0U;break;
      case 3:m.motor.header.flags=0U;break;
      case 4:m.motor.run_state=0U;break;
      case 5:m.motor_rc=PX4LITE_NOT_READY;break;
      case 6:m.lock_rc=PX4LITE_NOT_READY;break;
      case 7:m.status_rc=PX4LITE_NOT_READY;break;
      default:m.fresh=0U;break;
    }
    At(mode==8U?201U:10U);assert(!m.hz && !Status().music_gate && !Status().music_running);
  }
}
/** @brief 实际输出不断时仅时间轴继续，音乐不接管已有RGB渐变或事件诊断。 */
static void TestEventsAndRgb(void)
{
  uint32_t phase,remaining;
  Begin(1000000U,1000U);m.motor.pulse_us[0]=1300U;At(0U);At(30U);
  phase=Status().music_phase_index;remaining=Status().music_remaining_us;
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  Px4Lite_StartupToneNotifyRpiResult(1U);At(40U);remaining-=10000U;
  assert(m.hz==1568U && !Status().music_running && Status().music_phase_index==phase);
  At(100U);At(120U);At(180U);At(200U);At(280U);At(300U);At(460U);
  assert(Status().music_remaining_us==remaining && !m.hz);
  At(560U);assert(m.hz==523U && Status().music_phase_index==phase && Status().music_remaining_us==remaining);
#else
  (void)phase;(void)remaining;
#endif
  m.motor.pulse_us[0]=1000U;At(570U);assert(!m.hz);
}
/** @brief 失败不推进未听见的音符，共用2s退避但不污染事件错误计数。 */
static void TestFailureAndWrap(void)
{
  uint32_t errors;
  Begin(0xFFFF0000U,0xFFFFFF00U);m.motor.pulse_us[1]=1500U;
  errors=Status().feedback_buzzer_errors;m.tone_rc=PX4LITE_IO_ERROR;At(0U);
  assert(!Status().music_running && Status().music_error_count==1U && Status().music_remaining_us==100000U);
  assert(Status().feedback_buzzer_errors==errors);
  m.tone_rc=PX4LITE_OK;At(10U);assert(!m.hz && Status().music_remaining_us==100000U);
  At(2010U);assert(m.hz==523U && Status().music_running && Status().music_remaining_us==100000U);
  At(2050U);assert(Status().music_remaining_us==60000U);
  m.locked=1U;At(2060U);assert(!m.hz && Status().music_remaining_us==50000U);
}
#elif PX4LITE_ENABLE_MOTOR_MUSIC
/** @brief 直接引用正式用户曲谱，逐相位验证真实输出、跨255索引和60000ms完整循环。 */
static void TestRealScoreCycle(void)
{
  uint32_t i,total_ms=0U;
  Begin(0U,0U);m.motor.pulse_us[3]=1001U;At(0U);
  assert(Status().music_ready && Status().music_running && MOTOR_MUSIC_SCORE_COUNT>255U);
  for(i=0U;i<MOTOR_MUSIC_SCORE_COUNT;++i) {
    assert(Status().music_phase_index==i && m.hz==s_motor_music_score[i].frequency_hz);
    assert(Status().music_remaining_us==(uint32_t)s_motor_music_score[i].duration_ms*1000U);
    total_ms+=s_motor_music_score[i].duration_ms;
    At(total_ms);
    assert(Status().music_phase_index==(i+1U<MOTOR_MUSIC_SCORE_COUNT?i+1U:0U));
    if(i==255U) { assert(Status().music_phase_index==256U); }
  }
  assert(total_ms==MOTOR_MUSIC_SCORE_TOTAL_MS && total_ms==60000U);
  assert(Status().music_phase_index==0U && m.hz==s_motor_music_score[0].frequency_hz);
  assert(Status().music_remaining_us==(uint32_t)s_motor_music_score[0].duration_ms*1000U);
}
#endif
int main(void)
{
#if PX4LITE_ENABLE_MOTOR_MUSIC
#if defined(TEST_MOTOR_MUSIC_REAL_SCORE)
  TestRealScoreCycle();
  puts("Motor music user score: all263 real phases / index255->256 / actual frequencies and rests / nominal60000ms / loop to first phase PASS");
#else
  TestPauseResumeAndLoop();TestGateFacts();TestEventsAndRgb();TestFailureAndWrap();
  puts("Motor music: real player / any1001us / coherent gate / stale-fault-lock pause / remaining resume / cycle / long pause wrap / foreground priority / independent errors PASS");
#endif
#else
  Begin(0U,0U);m.motor.pulse_us[0]=1500U;At(0U);assert(!m.hz && !Status().music_running && !Status().music_ready);
  puts("Motor music disabled: unchanged startup/feedback paths, motor output does not play background PASS");
#endif
  return 0;
}
