/**
 * @file test_module_connection_feedback.c
 * @brief 真实Health快照后的模块连接提示边沿、防抖、启动基线及状态不变性验证。
 * @details 仅注入既有Framework状态事实和时间；不模拟设备在位，不新增总线I/O。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
static unsigned s_critical;
static uint8_t s_force_snapshot_busy;
static void Enter(void);
static void Leave(void);
#define taskENTER_CRITICAL() Enter()
#define taskEXIT_CRITICAL() Leave()
#include "px4lite_config.h"
#if defined(TEST_CONNECTION_MODULES_DISABLED) && TEST_CONNECTION_MODULES_DISABLED
#undef PX4LITE_ENABLE_GNSS
#undef PX4LITE_ENABLE_IMU
#undef PX4LITE_ENABLE_BARO
#undef PX4LITE_ENABLE_LORA
#undef PX4LITE_ENABLE_REMOTE_ID
#undef PX4LITE_ENABLE_STORAGE
#define PX4LITE_ENABLE_GNSS 0U
#define PX4LITE_ENABLE_IMU 0U
#define PX4LITE_ENABLE_BARO 0U
#define PX4LITE_ENABLE_LORA 0U
#define PX4LITE_ENABLE_REMOTE_ID 0U
#define PX4LITE_ENABLE_STORAGE 0U
#endif
#if defined(__GNUC__) && defined(TEST_CONNECTION_MODULES_DISABLED) && TEST_CONNECTION_MODULES_DISABLED
/* 六模块全关闭时既有GnssQuality仍保留声明；仅隔离此夹具的未引用静态函数诊断。 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "../../Framework/Src/px4lite_modules.c"
#if defined(__GNUC__) && defined(TEST_CONNECTION_MODULES_DISABLED) && TEST_CONNECTION_MODULES_DISABLED
#pragma GCC diagnostic pop
#endif

static uint32_t s_now;
static uint8_t s_events[32];
static unsigned s_event_count,s_health_count,s_alarm_count,s_time_count,s_recovery_count;
static Px4Lite_SystemHealth_t s_last_health;
static const Px4Lite_ModuleId_t s_supported[]={PX4LITE_MODULE_GNSS,PX4LITE_MODULE_IMU,
  PX4LITE_MODULE_BARO,PX4LITE_MODULE_LORA,PX4LITE_MODULE_REMOTE_ID,PX4LITE_MODULE_STORAGE};

/** @brief 临界区替身只核验配对，不允许事件回调位于状态复制锁内。 */
static void Enter(void) { s_critical++; }
/** @brief 可注入持续版本变化，验证失败快照不生成提示。 */
static void Leave(void) { assert(s_critical);s_critical--;if(s_force_snapshot_busy) { s_status_version++; } }
/** @brief 确定性真实毫秒来源，允许0和uint32回绕。 */
uint32_t Px4Lite_PlatformGetMs(void) { return s_now; }
/** @brief 健康原有授时入口，不读取新硬件。 */
void Px4Lite_TimeRun(uint32_t now) { assert(now==s_now && !s_critical);s_time_count++; }
/** @brief 保留原恢复调度位置；具体驱动恢复由既有专项覆盖。 */
void Px4Lite_RecoveryMonitorRun(uint32_t now) { assert(now==s_now && !s_critical);s_recovery_count++; }
/** @brief 快照只读统计，无传感器采集。 */
void Px4Lite_GetImuStats(Px4Lite_FifoStats_t *out) { memset(out,0,sizeof(*out));out->peak=3U;out->overflow_count=2U; }
/** @brief 原告警入口收到完整一致快照，附加观察不得改变其状态。 */
void Px4Lite_AlarmUpdateFromStatuses(const Px4Lite_ModuleStatus_t *status,uint16_t count,uint32_t now)
{
  unsigned i;assert(!s_critical && count==PX4LITE_MODULE_COUNT && now==s_now);
  for(i=0U;i<count;i++) { assert(status[i].state==s_status[i].state && status[i].fault_code==s_status[i].fault_code); }
  s_alarm_count++;
}
/** @brief 保存真实健康快照，验证原时间、序号、FIFO诊断和状态位图语义。 */
void Px4Lite_PublishHealth(const Px4Lite_SystemHealth_t *out)
{
  unsigned i;assert(!s_critical && out->header.sample_time_ms==s_now && out->header.valid);
  assert(out->status_version==s_status_version && out->imu_fifo_peak==3U && out->imu_fifo_overflow==2U);
  for(i=0U;i<PX4LITE_MODULE_COUNT;i++) {
    assert(out->module_state[i]==s_status[i].state);
    if(s_status[i].state==PX4LITE_STATE_FAILED) { assert(out->blocking_fault_mask&(1UL<<i)); }
    if(s_status[i].state==PX4LITE_STATE_OFFLINE || s_status[i].state==PX4LITE_STATE_DEGRADED) { assert(out->warning_fault_mask&(1UL<<i)); }
  }
  s_last_health=*out;s_health_count++;
}
/** @brief Health只排队连接类别；任何硬件声光输出不属于本单元。 */
void Px4Lite_StartupToneNotifyModuleConnection(uint8_t connected)
{
  assert(!s_critical && connected<=1U && s_event_count<sizeof(s_events));
  s_events[s_event_count++]=connected;
}

/** @brief 仅通过实际状态发布函数注入已由设备owner确认的事实；不把时间0当无数据。 */
static void Fact(Px4Lite_ModuleId_t id,Px4Lite_State_t state,uint16_t fault)
{
  Px4Lite_SetStatus(id,state,fault,s_now);
  if((id==PX4LITE_MODULE_GNSS || id==PX4LITE_MODULE_IMU || id==PX4LITE_MODULE_BARO) &&
     (state==PX4LITE_STATE_ONLINE || state==PX4LITE_STATE_DEGRADED)) { s_status[id].last_rx_ms=s_now; }
}
/** @brief Health正常运行中提供持续存在的样本时间；关闭刷新时走真实超时分支。 */
static void Tick(uint32_t now,uint8_t fresh)
{
  unsigned i;s_now=now;
  if(fresh) {
    for(i=0U;i<3U;i++) {
      Px4Lite_ModuleId_t id=s_supported[i];
      if(s_status[id].state==PX4LITE_STATE_ONLINE || s_status[id].state==PX4LITE_STATE_DEGRADED) { s_status[id].last_rx_ms=s_now; }
    }
  }
  Px4Lite_HealthRun(now);assert(!s_critical);
}
/** @brief 一次冷初始化，六类均给出明确缺席事实；初次缺席不得报拔出。 */
static void Reset(uint32_t now)
{
  unsigned i;s_now=now;s_force_snapshot_busy=0U;s_critical=0U;
  assert(Px4Lite_ModulesInit()==PX4LITE_OK && Px4Lite_HealthInit()==PX4LITE_OK);
  for(i=0U;i<sizeof(s_supported)/sizeof(s_supported[0]);i++) {
    if(s_supported[i]==PX4LITE_MODULE_STORAGE) { Fact(s_supported[i],PX4LITE_STATE_DEGRADED,PX4LITE_FAULT_STORAGE_NOT_READY); }
    else { Fact(s_supported[i],PX4LITE_STATE_FAILED,PX4LITE_FAULT_SENSOR_INIT); }
  }
  s_event_count=s_health_count=s_alarm_count=s_time_count=s_recovery_count=0U;
  memset(s_events,0xFF,sizeof(s_events));
}
/** @brief 模块或功能关闭时仍跑原Health，只有附加事件应静默。 */
static void Expect(Px4Lite_ModuleId_t id,const char *pattern)
{
  unsigned i,length=(unsigned)strlen(pattern);
  if(!PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK || !Px4Lite_IsModuleEnabled(id)) { length=0U; }
  assert(s_event_count==length);
  for(i=0U;i<length;i++) { assert(s_events[i]==(uint8_t)(pattern[i]-'0')); }
}
/** @brief 启动期模块建立或一直缺席均不响；未定位/降级往返不等于拔插。 */
static void TestStartupAndNoFix(void)
{
  Reset(0U);Tick(0U,1U);
  s_now=1000U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_DEGRADED,PX4LITE_FAULT_SENSOR_NO_FIX);
  Tick(1000U,1U);Tick(3000U,1U);Expect(PX4LITE_MODULE_GNSS,"");
  s_now=3100U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_ONLINE,0U);Tick(3100U,1U);
  s_now=3500U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_DEGRADED,PX4LITE_FAULT_SENSOR_NO_FIX);Tick(3500U,1U);
  Tick(3900U,1U);Expect(PX4LITE_MODULE_GNSS,"");
  assert(s_time_count==s_recovery_count && s_alarm_count==s_health_count);
}
/** @brief 六种支持来源都严格300ms双边沿，持续同状态不会重复。 */
static void TestSixSources(void)
{
  unsigned i;
  for(i=0U;i<sizeof(s_supported)/sizeof(s_supported[0]);i++) {
    Px4Lite_ModuleId_t id=s_supported[i];
    Reset(0U);Tick(0U,1U);Tick(3000U,1U);
    s_now=3100U;Fact(id,PX4LITE_STATE_ONLINE,0U);Tick(3100U,1U);Tick(3399U,1U);Expect(id,"");
    Tick(3400U,1U);Expect(id,"1");Tick(3800U,1U);Expect(id,"1");
    s_now=3900U;
    Fact(id,id==PX4LITE_MODULE_STORAGE?PX4LITE_STATE_DEGRADED:PX4LITE_STATE_OFFLINE,
         id==PX4LITE_MODULE_STORAGE?PX4LITE_FAULT_STORAGE_NOT_READY:PX4LITE_FAULT_SENSOR_OFFLINE);
    Tick(3900U,1U);Tick(4199U,1U);Expect(id,"1");Tick(4200U,1U);Expect(id,"10");Tick(5000U,1U);Expect(id,"10");
  }
}
/** @brief 恢复STARTING不自行确认拔出，也不能永久清掉明确断开候选；插入须连续明确。 */
static void TestStartingAndDebounce(void)
{
  Reset(0U);Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_DEGRADED,PX4LITE_FAULT_SENSOR_NO_FIX);
  Tick(0U,1U);Tick(3000U,1U);
  s_now=3100U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_OFFLINE,PX4LITE_FAULT_SENSOR_OFFLINE);Tick(3100U,1U);
  s_now=3200U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_STARTING,0U);Tick(3200U,1U);Tick(3500U,1U);Expect(PX4LITE_MODULE_GNSS,"");
  s_now=3600U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_OFFLINE,PX4LITE_FAULT_SENSOR_OFFLINE);Tick(3600U,1U);Expect(PX4LITE_MODULE_GNSS,"0");
  s_now=4000U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_DEGRADED,PX4LITE_FAULT_SENSOR_NO_FIX);Tick(4000U,1U);
  s_now=4100U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_STARTING,0U);Tick(4100U,1U);
  s_now=4200U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_DEGRADED,PX4LITE_FAULT_SENSOR_NO_FIX);Tick(4200U,1U);Tick(4499U,1U);Expect(PX4LITE_MODULE_GNSS,"0");
  Tick(4500U,1U);Expect(PX4LITE_MODULE_GNSS,"01");
  s_now=4600U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_OFFLINE,0U);Tick(4600U,1U);
  s_now=4700U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_ONLINE,0U);Tick(4700U,1U);Tick(5200U,1U);Expect(PX4LITE_MODULE_GNSS,"01");
}
/** @brief 存储未就绪单独解释；网络、电压、显示和虚拟模块状态都不进入提示。 */
static void TestStorageAndExcludedSources(void)
{
  static const Px4Lite_ModuleId_t excluded[]={PX4LITE_MODULE_5G,PX4LITE_MODULE_BATTERY,
    PX4LITE_MODULE_DISPLAY,PX4LITE_MODULE_CONTROL,PX4LITE_MODULE_ALARM,PX4LITE_MODULE_SYSTEM,
    PX4LITE_MODULE_ESTIMATOR,PX4LITE_MODULE_BUSINESS};
  unsigned i;Reset(0U);Fact(PX4LITE_MODULE_STORAGE,PX4LITE_STATE_ONLINE,0U);Tick(0U,1U);Tick(3000U,1U);
  s_now=3100U;Fact(PX4LITE_MODULE_STORAGE,PX4LITE_STATE_DEGRADED,PX4LITE_FAULT_STORAGE_WRITE);Tick(3100U,1U);Tick(3500U,1U);Expect(PX4LITE_MODULE_STORAGE,"");
  for(i=0U;i<sizeof(excluded)/sizeof(excluded[0]);i++) {
    Fact(excluded[i],PX4LITE_STATE_FAILED,0U);Tick(s_now+400U,1U);
    Fact(excluded[i],PX4LITE_STATE_ONLINE,0U);Tick(s_now+400U,1U);
  }
  Expect(PX4LITE_MODULE_STORAGE,"");
}
/** @brief 软件禁用/未初始化重新建基线，不制造拔出；重复HealthInit不修改设备事实。 */
static void TestRebaseline(void)
{
  Px4Lite_ModuleStatus_t before[PX4LITE_MODULE_COUNT];
  Reset(0U);Fact(PX4LITE_MODULE_LORA,PX4LITE_STATE_ONLINE,0U);Tick(0U,1U);Tick(3000U,1U);
  s_now=3100U;Fact(PX4LITE_MODULE_LORA,PX4LITE_STATE_DISABLED,0U);Tick(3100U,1U);
  s_now=3500U;Fact(PX4LITE_MODULE_LORA,PX4LITE_STATE_OFFLINE,0U);Tick(3500U,1U);Tick(3900U,1U);Expect(PX4LITE_MODULE_LORA,"");
  s_now=4000U;Fact(PX4LITE_MODULE_LORA,PX4LITE_STATE_ONLINE,0U);Tick(4000U,1U);Tick(4300U,1U);Expect(PX4LITE_MODULE_LORA,"1");
  memcpy(before,s_status,sizeof(before));assert(Px4Lite_HealthInit()==PX4LITE_OK);assert(memcmp(before,s_status,sizeof(before))==0);Expect(PX4LITE_MODULE_LORA,"1");
  s_now=4400U;Fact(PX4LITE_MODULE_LORA,PX4LITE_STATE_OFFLINE,0U);Tick(4400U,1U);Tick(7400U,1U);Expect(PX4LITE_MODULE_LORA,"1");
  s_now=7500U;Fact(PX4LITE_MODULE_LORA,PX4LITE_STATE_ONLINE,0U);Tick(7500U,1U);Tick(7800U,1U);Expect(PX4LITE_MODULE_LORA,"11");
}
/** @brief 启动窗口和候选计时都跨uint32毫秒回绕，不依赖时间0哨兵。 */
static void TestWrap(void)
{
  uint32_t start=UINT32_MAX-4000U;
  Reset(start);Tick(start,1U);Tick(start+3000U,1U);
  s_now=UINT32_MAX-100U;Fact(PX4LITE_MODULE_LORA,PX4LITE_STATE_ONLINE,0U);Tick(s_now,1U);
  Tick(100U,1U);Expect(PX4LITE_MODULE_LORA,"");Tick(200U,1U);Expect(PX4LITE_MODULE_LORA,"1");
  Reset(UINT32_MAX-1000U);Tick(UINT32_MAX-1000U,1U);Tick(1999U,1U);
  s_now=2100U;Fact(PX4LITE_MODULE_REMOTE_ID,PX4LITE_STATE_ONLINE,0U);Tick(2100U,1U);Tick(2400U,1U);Expect(PX4LITE_MODULE_REMOTE_ID,"1");
  /* 已完成基线的长期运行越过整圈后，不能再次把真实插入吞作启动事实。 */
  Reset(0U);Tick(0U,1U);Tick(3000U,1U);Tick(UINT32_MAX-100U,1U);
  s_now=50U;Fact(PX4LITE_MODULE_LORA,PX4LITE_STATE_ONLINE,0U);Tick(50U,1U);Tick(350U,1U);Expect(PX4LITE_MODULE_LORA,"1");
}
#if PX4LITE_ENABLE_GNSS
/** @brief 真实原Health超时仍决定OFFLINE；附加观察仅在其后稳定去抖。 */
static void TestRealHealthTimeout(void)
{
  Reset(0U);Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_DEGRADED,PX4LITE_FAULT_SENSOR_NO_FIX);Tick(0U,1U);Tick(3000U,1U);
  Tick(6001U,0U);assert(s_status[PX4LITE_MODULE_GNSS].state==PX4LITE_STATE_OFFLINE && s_status[PX4LITE_MODULE_GNSS].last_rx_ms==3000U);
  Tick(6300U,0U);Expect(PX4LITE_MODULE_GNSS,"");Tick(6301U,0U);Expect(PX4LITE_MODULE_GNSS,"0");
  s_now=6400U;Fact(PX4LITE_MODULE_GNSS,PX4LITE_STATE_DEGRADED,PX4LITE_FAULT_SENSOR_NO_FIX);Tick(6400U,1U);Tick(6700U,1U);Expect(PX4LITE_MODULE_GNSS,"01");
}
#endif
/** @brief 一致副本失败不发布健康也不消费连接候选，后续成功副本重新开始去抖。 */
static void TestFailedSnapshot(void)
{
  unsigned published;
  Reset(0U);Tick(0U,1U);Tick(3000U,1U);published=s_health_count;
  s_now=3100U;Fact(PX4LITE_MODULE_LORA,PX4LITE_STATE_ONLINE,0U);s_force_snapshot_busy=1U;Tick(3100U,1U);
  assert(s_health_count==published);Expect(PX4LITE_MODULE_LORA,"");
  s_force_snapshot_busy=0U;Tick(3400U,1U);Tick(3699U,1U);Expect(PX4LITE_MODULE_LORA,"");Tick(3700U,1U);Expect(PX4LITE_MODULE_LORA,"1");
}
/** @brief 运行真实Health及观察器；硬件、声光播放和物理插拔由其它层专项/实板验收。 */
int main(void)
{
  TestStartupAndNoFix();TestSixSources();TestStartingAndDebounce();TestStorageAndExcludedSources();
  TestRebaseline();TestWrap();TestFailedSnapshot();
#if PX4LITE_ENABLE_GNSS
  TestRealHealthTimeout();
#endif
  printf("Module connection Health: startup / six sources / no-fix / stable edges / STARTING recovery / excluded network+ADC+display / reset / time0+wrap / snapshot failure / original Health+alarm retained PASS enabled=%u\n",(unsigned)PX4LITE_ENABLE_MODULE_CONNECTION_FEEDBACK);
  return 0;
}
