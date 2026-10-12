/** @file test_sensor_debug_ownership.c
 * @brief 执行真实 sensor/Debug 任务循环，验证电源日志只读快照且不占用采集任务。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
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
void vTaskDelayUntil(TickType_t *last, TickType_t ticks);
void vTaskDelete(TaskHandle_t task);
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, uint16_t words, void *arg, UBaseType_t priority, TaskHandle_t *task);
size_t xPortGetFreeHeapSize(void);
#include "debug_config.h"
#undef DEBUG_GNSS_MONITOR_ENABLE
#undef DEBUG_IMU_MONITOR_ENABLE
#undef DEBUG_BARO_MONITOR_ENABLE
#undef DEBUG_POWER_MONITOR_ENABLE
#define DEBUG_GNSS_MONITOR_ENABLE 0U
#define DEBUG_IMU_MONITOR_ENABLE 0U
#define DEBUG_BARO_MONITOR_ENABLE 0U
#ifndef TEST_POWER_MONITOR
#define TEST_POWER_MONITOR 1U
#endif
#define DEBUG_POWER_MONITOR_ENABLE TEST_POWER_MONITOR
#include "../../Framework/Src/px4lite_app.c"
#include "../../Framework/Src/px4lite_work.c"
#include "../../Debug/Src/debug_service.c"

static jmp_buf task_exit;
static uint32_t now = 1000U;
static unsigned output_count, hardware_reads, sensor_runs, heartbeat_count;
static uint8_t battery_absent;
static char line[512];
uint32_t Px4Lite_PlatformGetMs(void) { return now; }
uint32_t Px4Lite_PlatformGetUs(void) { return now * 1000U; }
TickType_t xTaskGetTickCount(void) { return now; }
void vTaskDelayUntil(TickType_t *last, TickType_t ticks)
{ (void)last; assert(ticks > 0U); longjmp(task_exit, 1); }
void Px4Lite_PlatformHeartbeat(Px4Lite_HeartbeatId_t id, uint32_t ms)
{ (void)ms; assert(id == PX4LITE_HEARTBEAT_SENSOR); ++heartbeat_count; }
Px4Lite_Result_t Px4Lite_RegistryStart(Px4Lite_ModuleId_t id, uint32_t ms)
{ (void)id; (void)ms; return PX4LITE_OK; }
void Px4Lite_SensorWorkRun(uint32_t ms) { (void)ms; ++sensor_runs; }
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out)
{ if (battery_absent) { return PX4LITE_NOT_READY; } memset(out, 0, sizeof(*out)); out->voltage_mv = 12000U; out->current_ma = 321; out->header.sample_time_ms = 900U; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out)
{ if (battery_absent) { return PX4LITE_NOT_READY; } memset(out, 0, sizeof(*out)); out->voltage_mv = 11000U; out->current_ma = 123; out->header.sample_time_ms = 800U; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *out)
{ (void)id; memset(out, 0, sizeof(*out)); out->state = PX4LITE_STATE_ONLINE; return PX4LITE_OK; }
void DebugConsole_Printf(const char *prefix, const char *fmt, ...)
{ va_list ap; (void)prefix; ++output_count; va_start(ap, fmt); vsnprintf(line, sizeof(line), fmt, ap); va_end(ap); }
/* 旧实现会绕过快照进入该接口；修复后不应再有调用。 */
#include "bsp_adc_current.h"
void BSP_ADC_Current_GetDiag(BSP_ADC_CurrentChannel_t channel, uint32_t *mv, uint8_t *floating)
{ (void)channel; ++hardware_reads; *mv = 0U; *floating = 0U; }
int main(void)
{
  if (setjmp(task_exit) == 0) { Px4Lite_SensorTask(0); }
  assert(sensor_runs == 1U && heartbeat_count == 1U);
  assert(output_count == 0U && hardware_reads == 0U);
#if TEST_POWER_MONITOR
  if (setjmp(task_exit) == 0) { DebugService_Task(0); }
  assert(output_count == 1U && hardware_reads == 0U);
  assert(strstr(line, "12000") != 0 && strstr(line, "321") != 0);
  assert(strstr(line, "11000") != 0 && strstr(line, "123") != 0);
  assert(strstr(line, "age_ms=100") != 0 && strstr(line, "age_ms=200") != 0);
  battery_absent = 1U;
  if (setjmp(task_exit) == 0) { DebugService_Task(0); }
  assert(output_count == 2U && hardware_reads == 0U);
  assert(strstr(line, "B1 ready=0") != 0 && strstr(line, "B2 ready=0") != 0);
#else
  assert(DebugService_CreateTask() == pdPASS);
  assert(output_count == 0U && hardware_reads == 0U);
#endif
  puts("sensor/debug ownership tests passed");
  return 0;
}
