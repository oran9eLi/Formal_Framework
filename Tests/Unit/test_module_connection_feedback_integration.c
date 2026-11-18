/**
 * @file test_module_connection_feedback_integration.c
 * @brief 真实Health连接边沿与真实sensor播放器的混合事件集成回归。
 * @details 仅注入已有LoRa状态事实；不替代状态判定或播放器，不证明物理拔插。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
static unsigned s_critical;
#define taskENTER_CRITICAL() (++s_critical)
#define taskEXIT_CRITICAL() do { assert(s_critical);--s_critical; } while(0)
#include "../../Framework/Src/px4lite_modules.c"
/* 两份真实实现的私有s_status同名；只在独立测试翻译单元重命名播放器副本。 */
#define s_status s_tone_status
#include "../../Framework/Src/px4lite_startup_tone.c"
#undef s_status

static uint32_t s_now;
static unsigned s_owner,s_io,s_health_publications,s_alarm_updates,s_recoveries;
static uint16_t s_frequency;
static uint8_t s_rgb[3];

/** @brief 本夹具只验证模块/命令提示，未接入电机输出topic时背景音乐必须保持暂停。 */
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out) { (void)out;return PX4LITE_NOT_READY; }
/** @brief 无电机源事实不伪造解锁，独立音乐集成专项覆盖真实运行门控。 */
Px4Lite_Result_t Px4Lite_ControlGetMotorLocked(uint8_t *locked)
{ if(locked) { *locked=1U; }return PX4LITE_NOT_READY; }

/** @brief 所有声光边界只允许由测试标记的sensor上下文调用。 */
static void Hardware(void) { assert(s_owner==1U && !s_critical);s_io++; }
/** @brief 平台时钟独立于被测消息，单位真实ms。 */
uint32_t Px4Lite_PlatformGetMs(void) { return s_now; }
/** @brief 本夹具时间小于微秒回绕周期，回绕由播放器专项覆盖。 */
uint32_t Px4Lite_PlatformGetUs(void) { return s_now*1000U; }
/** @brief 真实Health保留原授时调度，但不访问RTC。 */
void Px4Lite_TimeRun(uint32_t now_ms) { assert(now_ms==s_now); }
/** @brief 真实Health仍调用恢复监督；本夹具不创建注册表或执行硬件恢复。 */
void Px4Lite_RecoveryMonitorRun(uint32_t now_ms) { assert(now_ms==s_now);s_recoveries++; }
/** @brief 保留健康快照的FIFO统计读取，不依赖真实IMU。 */
void Px4Lite_GetImuStats(Px4Lite_FifoStats_t *out) { memset(out,0,sizeof(*out)); }
/** @brief 告警消费者只观察原状态快照，提示功能不能提前代替其运行。 */
void Px4Lite_AlarmUpdateFromStatuses(const Px4Lite_ModuleStatus_t *status,uint16_t count,uint32_t now_ms)
{ assert(status && count==PX4LITE_MODULE_COUNT && now_ms==s_now);s_alarm_updates++; }
/** @brief 健康发布保留，模块音频不能阻止或替代系统快照。 */
void Px4Lite_PublishHealth(const Px4Lite_SystemHealth_t *health)
{ assert(health && health->header.sample_time_ms==s_now);s_health_publications++; }
/** @brief 仅记录真实播放器初始化调用。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void) { Hardware();s_frequency=0U;return PX4LITE_OK; }
/** @brief 记录真实播放器请求频率，直接对照外部选定旋律。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t hz) { Hardware();s_frequency=hz;return PX4LITE_OK; }
/** @brief RGB始终可用，异常分支由独立播放器专项覆盖。 */
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void) { Hardware();return PX4LITE_OK; }
/** @brief 即使撤收也不能由Health直接操作。 */
void Px4Lite_PlatformRgbDeInit(void) { Hardware(); }
/** @brief 记录三通道发送请求，不模拟物理亮度或DMA波形。 */
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t red,uint8_t green,uint8_t blue)
{ Hardware();s_rgb[0]=red;s_rgb[1]=green;s_rgb[2]=blue;return PX4LITE_OK; }
/** @brief 只允许sensor推进输出服务。 */
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t now_ms) { Hardware();assert(now_ms==s_now);return PX4LITE_OK; }

/** @brief 真正执行Health，断言只通知而不在该任务直接发声/改灯。 */
static void Health(uint32_t now_ms)
{
  unsigned before=s_io;
  s_now=now_ms;s_owner=2U;Px4Lite_HealthRun(s_now);s_owner=0U;
  assert(s_io==before && !s_critical);
}
/** @brief 真正由sensor播放，非本上下文的硬件访问将立即失败。 */
static void Sensor(uint32_t now_ms,uint16_t expected_hz)
{
  s_now=now_ms;s_owner=1U;Px4Lite_StartupToneRun(s_now*1000U);s_owner=0U;
  assert(s_frequency==expected_hz && !s_critical);
}
/** @brief 注入Comm既有本机LoRa可用性结果，不使用远端报文冒充存在。 */
static void LoRa(uint8_t connected,uint32_t now_ms)
{
  Px4Lite_SetStatus(PX4LITE_MODULE_LORA,connected?PX4LITE_STATE_ONLINE:PX4LITE_STATE_OFFLINE,
    connected?PX4LITE_FAULT_NONE:PX4LITE_FAULT_COMM_OFFLINE,now_ms);
  assert(s_status[PX4LITE_MODULE_LORA].consecutive_valid==0U);
}

/** @brief Health插/拔事件与原成功/失败提示在同一有界队列中保持类型及顺序。 */
int main(void)
{
  unsigned before;
  s_now=0U;assert(Px4Lite_ModulesInit()==PX4LITE_OK && Px4Lite_HealthInit()==PX4LITE_OK);
  s_owner=1U;Px4Lite_StartupToneInit(0U);s_owner=0U;
  LoRa(0U,0U);Health(100U);Sensor(2625U,0U);
  Health(2900U);Health(3100U);
  LoRa(1U,3200U);Health(3200U);Health(3600U);
  before=s_io;Px4Lite_StartupToneNotifyRpiResult(1U);assert(s_io==before);
  /* 插入双音先播放，随后才可开始原命令成功四音。 */
  Sensor(3600U,1047U);Sensor(3680U,0U);Sensor(3720U,2093U);Sensor(3900U,0U);Sensor(4000U,0U);
  LoRa(0U,4000U);Health(4000U);
  Sensor(4010U,1568U);Sensor(4070U,0U);Sensor(4090U,1976U);Sensor(4150U,0U);
  Sensor(4170U,2349U);Sensor(4250U,0U);Sensor(4270U,3136U);
  /* Health检测拔出时，正在播放的成功高音不得被跨任务直接改写。 */
  Health(4300U);assert(s_frequency==3136U);
  Sensor(4430U,0U);before=s_io;Px4Lite_StartupToneNotifyRpiResult(0U);assert(s_io==before);
  Sensor(4530U,0U);
  /* 拔出双音必须独立于原命令失败三音。 */
  Sensor(4540U,2093U);Sensor(4680U,0U);Sensor(4740U,1047U);Sensor(4940U,0U);Sensor(5040U,0U);
  Sensor(5050U,880U);Sensor(5170U,0U);Sensor(5210U,659U);Sensor(5350U,0U);
  Sensor(5390U,440U);Sensor(5610U,0U);Sensor(5710U,0U);
  assert(s_health_publications==s_alarm_updates && s_health_publications==s_recoveries);
  assert(s_health_publications>=7U);
  puts("Module feedback integration: real Health enqueue-only / sensor owns sound+RGB / insert-success-remove-failure FIFO / original melodies preserved PASS");
  return 0;
}
