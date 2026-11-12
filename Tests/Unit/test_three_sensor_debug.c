/**
 * @file test_three_sensor_debug.c
 * @brief 真实Debug报告器的三模块状态、IMU双ID、Baro有效性与既有水位副本回归。
 * @details 仅注入已缓存的Framework/App数据，无BSP/Sensor/HAL访问或生产发布入口。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <setjmp.h>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
#define pdPASS 1
#define pdFAIL 0
#define tskIDLE_PRIORITY 0U
#define pdMS_TO_TICKS(ms) (ms)
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *last,TickType_t ticks);
BaseType_t xTaskCreate(void (*fn)(void *),const char *name,uint16_t words,void *arg,UBaseType_t priority,TaskHandle_t *out);
#include "debug_config.h"
/* 本用例测三模块诊断；GNSS原帧和电源报告由各自已有用例覆盖。 */
#undef DEBUG_GNSS_MONITOR_ENABLE
#define DEBUG_GNSS_MONITOR_ENABLE 0U
#undef DEBUG_POWER_MONITOR_ENABLE
#define DEBUG_POWER_MONITOR_ENABLE 0U
#if defined(TEST_THREE_SENSOR_DIAG_DISABLED) && TEST_THREE_SENSOR_DIAG_DISABLED
#undef DEBUG_STACK_MONITOR_ENABLE
#define DEBUG_STACK_MONITOR_ENABLE 0U
#undef DEBUG_IMU_MONITOR_ENABLE
#define DEBUG_IMU_MONITOR_ENABLE 0U
#undef DEBUG_BARO_MONITOR_ENABLE
#define DEBUG_BARO_MONITOR_ENABLE 0U
#endif
#include "../../Debug/Src/debug_service.c"
#include "../../Debug/Src/debug_task_monitor.c"
#if defined(BSP_I2C_H) || defined(SENSOR_BMI088_H) || defined(SENSOR_BME280_H) || defined(STM32F4xx_HAL_H)
#error Debug reporter must not expose sensor or hardware headers
#endif

static uint32_t s_clock,s_lines,s_max_line,s_status_copies,s_baro_copies,s_driver_copies,s_nav_copies;
static unsigned s_runtime_copies,s_task_row_copies,s_created,s_runtime_resets,s_registered;
static char s_output[128][256];
static Px4Lite_ModuleStatus_t s_state[PX4LITE_MODULE_COUNT];
static Px4Lite_ImuDiagnostics_t s_driver;
static App_NavigationSnapshot_t s_nav;
static Px4Lite_SensorBaro_t s_baro;
static Px4Lite_Result_t s_driver_rc=PX4LITE_OK,s_nav_rc=PX4LITE_NOT_READY,s_baro_rc=PX4LITE_NOT_READY;
static jmp_buf s_task_exit;

/** @brief 模块状态源只复制；测试末逐字核对它未被报告器修改。 */
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id,Px4Lite_ModuleStatus_t *out)
{ assert(id<PX4LITE_MODULE_COUNT);s_status_copies++;*out=s_state[id];return PX4LITE_OK; }
/** @brief 只在成功时提供诊断，检出失败时复用旧ID。 */
Px4Lite_Result_t Px4Lite_ImuCopyDiagnostics(Px4Lite_ImuDiagnostics_t *out)
{ s_driver_copies++;if(s_driver_rc==PX4LITE_OK) { *out=s_driver; }return s_driver_rc; }
/** @brief 导航副本只供观察，不触发估计或采集。 */
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out,uint32_t now)
{ assert(now==s_clock);s_nav_copies++;if(s_nav_rc==PX4LITE_OK) { *out=s_nav; }return s_nav_rc; }
/** @brief 独立模拟Baro尚无topic和成功但过期的已有样本。 */
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out)
{ s_baro_copies++;if(s_baro_rc==PX4LITE_OK) { *out=s_baro; }return s_baro_rc; }
uint32_t Px4Lite_PlatformGetMs(void) { return s_clock; }
/** @brief 栈报告仅有复制API；若生产调用扫描函数，本用例将因缺实现而失败。 */
Px4Lite_Result_t Px4Lite_RuntimeCopy(Px4Lite_RuntimeSnapshot_t *out)
{
  s_runtime_copies++;memset(out,0,sizeof(*out));out->measured_ms=s_clock;out->heap_total=40960U;
  out->heap_free=8500U;out->heap_minimum=8100U;out->sensor_service_max_us=104000U;out->task_count=3U;return PX4LITE_OK;
}
/** @brief 返回已测sensor/debug两行和未测一行；不测栈/不读硬件。 */
Px4Lite_Result_t Px4Lite_RuntimeCopyTask(uint8_t index,Px4Lite_TaskRuntime_t *out)
{
  assert(index<3U);s_task_row_copies++;memset(out,0,sizeof(*out));
  out->name=index==0U?"sensor":index==1U?"debug":"display";out->configured_words=index==1U?384U:512U;
  out->minimum_free_words=index==0U?80U:200U;out->warning=(uint8_t)(index==0U);out->measured=(uint8_t)(index<2U);out->measured_ms=4000U;
  return PX4LITE_OK;
}
void Px4Lite_RuntimeInit(void) { s_runtime_resets++; }
BaseType_t Px4Lite_RuntimeRegister(TaskHandle_t handle,const char *name,uint16_t words)
{ assert(handle==(void *)1 && !strcmp(name,"debug") && words==DEBUG_SERVICE_TASK_STACK_WORDS);s_registered++;return pdPASS; }
/** @brief 严格保留完整一行并包含前缀/CRLF检查256B串口格式上限。 */
void DebugConsole_Printf(const char *prefix,const char *format,...)
{
  va_list args;int n;uint32_t bytes;
  assert(!strcmp(prefix,"[DBG] ") && s_lines<128U);
  va_start(args,format);n=vsnprintf(s_output[s_lines],sizeof(s_output[0]),format,args);va_end(args);
  assert(n>=0 && n<256);bytes=(uint32_t)n+(uint32_t)strlen(prefix)+2U;assert(bytes<256U);
  if(bytes>s_max_line) { s_max_line=bytes; }s_lines++;
}
TickType_t xTaskGetTickCount(void) { return s_clock; }
/** @brief 推进真实100ms任务循环至5.3s，不改变任何采集周期。 */
void vTaskDelayUntil(TickType_t *last,TickType_t ticks)
{ assert(ticks==100U);*last+=ticks;s_clock+=ticks;if(s_clock>5200U) { longjmp(s_task_exit,1); } }
BaseType_t xTaskCreate(void (*fn)(void *),const char *name,uint16_t words,void *arg,UBaseType_t priority,TaskHandle_t *out)
{ (void)fn;(void)arg;assert(!strcmp(name,"debug") && words==384U && priority==1U);s_created++;*out=(void *)1;return pdPASS; }

#if DEBUG_IMU_MONITOR_ENABLE && DEBUG_BARO_MONITOR_ENABLE && DEBUG_STACK_MONITOR_ENABLE
/** @brief 独立场景仅清夹具数据；不对外提供清生产计数入口。 */
static void Reset(void)
{
  memset(s_output,0,sizeof(s_output));memset(s_state,0,sizeof(s_state));memset(&s_driver,0,sizeof(s_driver));
  memset(&s_nav,0,sizeof(s_nav));memset(&s_baro,0,sizeof(s_baro));s_lines=s_clock=0U;
  s_status_copies=s_baro_copies=s_driver_copies=s_nav_copies=0U;s_runtime_copies=s_task_row_copies=0U;
  s_created=s_registered=s_runtime_resets=0U;s_driver_rc=PX4LITE_OK;s_nav_rc=s_baro_rc=PX4LITE_NOT_READY;
  s_state[PX4LITE_MODULE_GNSS].state=PX4LITE_STATE_OFFLINE;s_state[PX4LITE_MODULE_GNSS].fault_code=0x0101U;
  s_state[PX4LITE_MODULE_IMU].state=PX4LITE_STATE_FAILED;s_state[PX4LITE_MODULE_IMU].fault_code=0x0102U;
  s_state[PX4LITE_MODULE_BARO].state=PX4LITE_STATE_STARTING;s_state[PX4LITE_MODULE_BARO].fault_code=0x0103U;
  s_driver.driver=1U;s_driver.bus=1U;s_driver.accel_chip_id=0x1EU;s_driver.gyro_chip_id=0x0FU;
  s_driver.accel_addr_7bit=0x18U;s_driver.gyro_addr_7bit=0x68U;s_driver.device_ready=1U;s_driver.last_stage=5U;
}
/** @brief 三模块真实状态不被ready/无测量覆盖；缺样本显示N/A而不是0Pa。 */
static void TestStatesAndUnknownBaro(void)
{
  Px4Lite_ModuleStatus_t before[PX4LITE_MODULE_COUNT];Reset();memcpy(before,s_state,sizeof(before));
  DebugService_ReportSensorStates();assert(s_lines==1U);
  assert(strstr(s_output[0],"GNSS=OFFLINE") && strstr(s_output[0],"IMU=FAILED") && strstr(s_output[0],"Baro=STARTING"));
  DebugService_ReportBaro(0U);assert(strstr(s_output[1],"ready=0 sample_valid=0") && strstr(s_output[1],"p_pa=N/A"));
  assert(!memcmp(before,s_state,sizeof(before)) && s_baro_copies==1U && !s_runtime_resets);
}
/** @brief IMU双ID/接口与失败结果均可见，失败getter不泄漏上次成功ID。 */
static void TestImuFacts(void)
{
  Reset();DebugService_ReportImu(0U);
  assert(strstr(s_output[0],"IMU-DRV:") && strstr(s_output[0],"id=0x1E/0x0F") && strstr(s_output[0],"bus=1"));
  assert(strstr(s_output[1],"no valid attitude"));
  s_driver_rc=PX4LITE_NOT_READY;DebugService_ReportImu(0U);assert(strstr(s_output[2],"id=0x00/0x00"));
}
/** @brief 合法0ms样本不当未初始化，旧样本保留source时标并明确fresh0。 */
static void TestBaroFreshnessAndInvalidHeader(void)
{
  Reset();s_clock=5000U;s_baro_rc=PX4LITE_OK;s_baro.header.valid=1U;s_baro.header.sample_time_ms=0U;s_baro.pressure_pa=100000.0f;
  DebugService_ReportBaro(s_clock);assert(strstr(s_output[0],"ready=1 sample_valid=1 fresh=0 sample=0 age=5000 p_pa=100000"));
  s_baro.header.sample_time_ms=4900U;DebugService_ReportBaro(s_clock);assert(strstr(s_output[1],"fresh=1 sample=4900 age=100"));
  s_baro.header.valid=0U;DebugService_ReportBaro(s_clock);assert(strstr(s_output[2],"sample_valid=0") && strstr(s_output[2],"p_pa=N/A"));
}
/** @brief 一次启动标签、1秒状态/IMU/Baro及5秒资源水位，均走既有副本入口。 */
static void TestTaskCadenceAndWatermarks(void)
{
  unsigned i;volatile unsigned tag=0U,states=0U,imu=0U,baro=0U,mem=0U,stack=0U;
  Reset();assert(DebugService_CreateTask()==pdPASS && s_created==1U && s_registered==1U);
  if(setjmp(s_task_exit)==0) { DebugService_Task(NULL); }
  for(i=0U;i<s_lines;i++) {
    if(strstr(s_output[i],"SENS-DIAG: sensor-stack-20261111")) { tag++; }
    if(!strncmp(s_output[i],"SENS: ",6U)) { states++; }
    if(!strncmp(s_output[i],"IMU-DRV: ",9U)) { imu++; }
    if(!strncmp(s_output[i],"BARO: ",6U)) { baro++; }
    if(!strncmp(s_output[i],"MEM: ",5U)) { mem++;assert(strstr(s_output[i],"sensor_max_us=104000")); }
    if(!strncmp(s_output[i],"STACK: ",7U)) { stack++; }
  }
  assert(tag==1U && states==5U && imu==5U && baro==5U && mem==1U && stack==2U);
  assert(s_runtime_copies==1U && s_task_row_copies==3U && !s_runtime_resets && s_clock==5300U);
}
/** @brief 极限计数也不能越过256B，未改生产状态或触发采样。 */
static void TestLineBounds(void)
{
  unsigned i;Reset();s_clock=0x7FFFFFFFU;
  for(i=0U;i<PX4LITE_MODULE_COUNT;i++) { s_state[i].state=PX4LITE_STATE_DEGRADED;s_state[i].fault_code=0xFFFFU;s_state[i].last_rx_ms=UINT32_MAX;s_state[i].error_count=UINT32_MAX; }
  s_driver.reinit_count=s_driver.fifo_drop_count=UINT32_MAX;
  DebugService_ReportSensorStates();DebugService_ReportImu(s_clock);DebugService_ReportBaro(s_clock);
}
#endif

/** @brief 默认与关闭构建分别核验，诊断不改变任务数或栈配置。 */
int main(void)
{
#if DEBUG_IMU_MONITOR_ENABLE && DEBUG_BARO_MONITOR_ENABLE && DEBUG_STACK_MONITOR_ENABLE
  TestStatesAndUnknownBaro();TestImuFacts();TestBaroFreshnessAndInvalidHeader();TestTaskCadenceAndWatermarks();TestLineBounds();
  printf("Three sensor Debug: states / BMI IDs / unknown+stale Baro / once tag / 1s+5s cadence / copy-only / maxline=%luB scratch=%luB runtime=%luB row=%luB PASS\n",
    (unsigned long)s_max_line,(unsigned long)sizeof(s_sensor_scratch),(unsigned long)sizeof(s_debug_runtime),(unsigned long)sizeof(s_debug_task_row));
#else
  assert(DebugService_CreateTask()==pdPASS && !s_created && !s_status_copies && !s_runtime_copies && !s_lines);
  puts("Three sensor Debug disabled: no task / no getters / no output PASS");
#endif
  return 0;
}
