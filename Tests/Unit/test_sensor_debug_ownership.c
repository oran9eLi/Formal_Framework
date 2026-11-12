/** @file test_sensor_debug_ownership.c
 * @brief 执行真实sensor/Debug任务循环，验证只读取启用组且逐组报告，不占用采集任务。
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
#undef DEBUG_STACK_MONITOR_ENABLE
#define DEBUG_GNSS_MONITOR_ENABLE 0U
#define DEBUG_IMU_MONITOR_ENABLE 0U
#define DEBUG_BARO_MONITOR_ENABLE 0U
#define DEBUG_STACK_MONITOR_ENABLE 0U
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
static unsigned snapshot_reads[2], diagnostic_copies;
static uint8_t battery_absent;
static char lines[12][256];
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
/** @brief 启动音调由独立回归覆盖，本用例只隔离其调度依赖。 */
void Px4Lite_StartupToneInit(uint32_t now_us) { (void)now_us; }
void Px4Lite_StartupToneRun(uint32_t now_us) { (void)now_us; }
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out)
{ snapshot_reads[0]++; if (battery_absent) { return PX4LITE_NOT_READY; } memset(out, 0, sizeof(*out)); out->header.valid = 1U; out->voltage_mv = 12000U; out->current_ma = 321; out->current_valid = 1U; out->header.sample_time_ms = 900U; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out)
{ snapshot_reads[1]++; if (battery_absent) { return PX4LITE_NOT_READY; } memset(out, 0, sizeof(*out)); out->header.valid = 1U; out->voltage_mv = 11000U; out->current_ma = 123; out->current_valid = 1U; out->header.sample_time_ms = 800U; return PX4LITE_OK; }
/** @brief Debug只能复制owner已发布的诊断；本桩刻意不调用任何ADC接口。 */
Px4Lite_Result_t Px4Lite_Battery2CopyDiagnostics(Px4Lite_PowerDiagnostics_t *out)
{
  ++diagnostic_copies;
  if (battery_absent) { return PX4LITE_NOT_READY; }
  memset(out, 0, sizeof(*out));
  out->sequence = 77U; out->sample_time_ms = 800U;
  out->voltage_raw = 1464U; out->voltage_pin_mv = 1180U;
  out->voltage_bsp_mv = 12009U; out->voltage_mv = 12009U;
  out->vref_raw = 1500U; out->vref_cal_raw = 1500U;
  out->vdda_mv = 3300U; out->vdda_calculated_mv = 3300U;
  out->current_pin_mv = 52U; out->current_ma = 800;
  out->reference_status = PX4LITE_POWER_REF_MEASURED;
  out->current_valid = 1U; out->voltage_valid = 1U;
  return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *out)
{ (void)id; memset(out, 0, sizeof(*out)); out->state = PX4LITE_STATE_ONLINE; return PX4LITE_OK; }
void DebugConsole_Printf(const char *prefix, const char *fmt, ...)
{ va_list ap; (void)prefix; assert(output_count < sizeof(lines) / sizeof(lines[0])); va_start(ap, fmt); vsnprintf(lines[output_count], sizeof(lines[0]), fmt, ap); va_end(ap); output_count++; }
/* 旧实现会绕过快照进入该接口；修复后不应再有调用。 */
#include "bsp_adc_current.h"
void BSP_ADC_Current_GetDiag(BSP_ADC_CurrentChannel_t channel, uint32_t *mv, uint8_t *floating)
{ (void)channel; ++hardware_reads; *mv = 0U; *floating = 0U; }
/** @brief 查找一轮报告中指定输入的独立记录，不能以同一行拼接掩盖禁用组读取。 */
#if TEST_POWER_MONITOR
static const char *FindInputLine(unsigned first, unsigned count, const char *input)
{
  unsigned i;
  for (i = first; i < first + count; i++) { if (strstr(lines[i], input)) { return lines[i]; } }
  return 0;
}
#endif

/** @brief 同一夹具覆盖默认仅第二组、显式双组及关闭Debug三种编译配置。 */
int main(void)
{
  if (setjmp(task_exit) == 0) { Px4Lite_SensorTask(0); }
  assert(sensor_runs == 1U && heartbeat_count == 1U);
  assert(output_count == 0U && hardware_reads == 0U);
  assert(diagnostic_copies == 0U);
  assert(snapshot_reads[0] == 0U && snapshot_reads[1] == 0U);
#if TEST_POWER_MONITOR
  {
  const unsigned enabled = PX4LITE_ENABLE_BATTERY1 + PX4LITE_ENABLE_BATTERY2;
  const unsigned first_lines = enabled + 2U * PX4LITE_ENABLE_BATTERY2;
  const unsigned absent_lines = enabled + PX4LITE_ENABLE_BATTERY2;
  const char *line;
  if (setjmp(task_exit) == 0) { DebugService_Task(0); }
  assert(output_count == first_lines && hardware_reads == 0U);
  assert(diagnostic_copies == (unsigned)PX4LITE_ENABLE_BATTERY2);
  assert(snapshot_reads[0] == (unsigned)PX4LITE_ENABLE_BATTERY1);
  assert(snapshot_reads[1] == (unsigned)PX4LITE_ENABLE_BATTERY2);
  line = FindInputLine(0U, first_lines, "input=1 ");
#if PX4LITE_ENABLE_BATTERY1
  assert(line && strstr(line, "v_mv=12000") && strstr(line, "current_ma=321") && strstr(line, "age_ms=100"));
#else
  assert(line == 0);
#endif
  line = FindInputLine(0U, first_lines, "input=2 ");
#if PX4LITE_ENABLE_BATTERY2
  assert(line && strstr(line, "v_mv=11000") && strstr(line, "current_valid=1") && strstr(line, "current_ma=123") && strstr(line, "age_ms=200"));
  line = FindInputLine(0U, first_lines, "POWER-ADC:");
  assert(line && strstr(line, "seq=77") && strstr(line, "raw=1464") && strstr(line, "pin_mv=1180"));
  assert(strstr(line, "bsp_mv=12009") && strstr(line, "final_mv=12009") && strstr(line, "age_ms=200"));
  line = FindInputLine(0U, first_lines, "POWER-REF:");
  assert(line && strstr(line, "seq=77") && strstr(line, "ref_raw=1500") && strstr(line, "used_vdda_mv=3300"));
#else
  assert(line == 0);
#endif
  battery_absent = 1U;
  if (setjmp(task_exit) == 0) { DebugService_Task(0); }
  assert(output_count == first_lines + absent_lines && hardware_reads == 0U);
  assert(diagnostic_copies == (unsigned)PX4LITE_ENABLE_BATTERY2 * 2U);
  assert(snapshot_reads[0] == (unsigned)PX4LITE_ENABLE_BATTERY1 * 2U);
  assert(snapshot_reads[1] == (unsigned)PX4LITE_ENABLE_BATTERY2 * 2U);
#if PX4LITE_ENABLE_BATTERY1
  line = FindInputLine(first_lines, absent_lines, "input=1 "); assert(line && strstr(line, "ready=0"));
#endif
#if PX4LITE_ENABLE_BATTERY2
  line = FindInputLine(first_lines, absent_lines, "input=2 "); assert(line && strstr(line, "ready=0"));
  line = FindInputLine(first_lines, absent_lines, "POWER-ADC:"); assert(line && strstr(line, "no sample"));
#endif
  }
#else
  assert(DebugService_CreateTask() == pdPASS);
  assert(output_count == 0U && hardware_reads == 0U);
  assert(diagnostic_copies == 0U);
  assert(snapshot_reads[0] == 0U && snapshot_reads[1] == 0U);
#endif
  puts("sensor/debug ownership tests passed");
  return 0;
}
