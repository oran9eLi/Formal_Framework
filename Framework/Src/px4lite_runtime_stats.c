/**
 * @file px4lite_runtime_stats.c
 * @brief Framework 固定任务登记及低频资源测量。
 * @details 单写者biz_system扫描栈；comm/display只提交各自事实；临界区仅复制。
 */
#include "px4lite_runtime_stats.h"
#include "px4lite_topics.h"
#include "px4lite_modules.h"
#include "timers.h"
#include <string.h>
/** @brief 常驻任务槽，不保留栈内容。 */
typedef struct {
  TaskHandle_t handle; /**< 已创建任务句柄。 */
  Px4Lite_TaskRuntime_t sample; /**< 最近测量副本。 */
} RuntimeTask_t;
static RuntimeTask_t s_tasks[PX4LITE_RUNTIME_TASK_CAPACITY];
static Px4Lite_RuntimeSnapshot_t s_runtime;
static uint8_t s_cursor;
static uint8_t s_comm_seen;
static uint8_t s_kernel_registered;
static uint32_t s_comm_last_ms;
/** @brief 仅在启动阶段复位。 */
void Px4Lite_RuntimeInit(void)
{
  memset(s_tasks, 0, sizeof(s_tasks)); memset(&s_runtime, 0, sizeof(s_runtime));
  s_cursor = 0U; s_comm_seen = 0U; s_comm_last_ms = 0U;
  s_kernel_registered = 0U;
}
/** @brief 调度前登记，失败保留明确计数。 */
BaseType_t Px4Lite_RuntimeRegister(TaskHandle_t handle, const char *name, uint16_t words)
{
  uint8_t i;
  for (i = 0U; i < s_runtime.task_count; i++) {
    if (s_tasks[i].handle == handle) { s_runtime.registration_errors++; return pdFAIL; }
  }
  if (!handle || !name || !words || s_runtime.task_count >= PX4LITE_RUNTIME_TASK_CAPACITY) {
    s_runtime.registration_errors++; return pdFAIL;
  }
  i = s_runtime.task_count++;
  s_tasks[i].handle = handle; s_tasks[i].sample.name = name;
  s_tasks[i].sample.configured_words = words;
  return pdPASS;
}
/** @brief 每次最多一次栈扫描，避免低优先级任务长时间连续遍历。 */
void Px4Lite_RuntimeStep(uint32_t now_ms)
{
  Px4Lite_FifoStats_t fifo;
  Px4Lite_CommDebugInfo_t comm;
  uint32_t heap_free = (uint32_t)xPortGetFreeHeapSize();
  uint32_t heap_min = (uint32_t)xPortGetMinimumEverFreeHeapSize();
  if (!s_kernel_registered) {
    taskENTER_CRITICAL();
    (void)Px4Lite_RuntimeRegister(xTaskGetIdleTaskHandle(), "IDLE", configMINIMAL_STACK_SIZE);
#if configUSE_TIMERS
    (void)Px4Lite_RuntimeRegister(xTimerGetTimerDaemonTaskHandle(), "Tmr Svc", configTIMER_TASK_STACK_DEPTH);
#endif
    s_kernel_registered = 1U;
    taskEXIT_CRITICAL();
  }
  if (s_runtime.task_count) {
    Px4Lite_TaskRuntime_t row = s_tasks[s_cursor].sample;
    UBaseType_t free_words = uxTaskGetStackHighWaterMark(s_tasks[s_cursor].handle);
    row.minimum_free_words = (uint16_t)free_words; row.measured_ms = now_ms; row.measured = 1U;
    row.warning = (uint8_t)(free_words < 100U || free_words * 4U < row.configured_words);
    taskENTER_CRITICAL(); s_tasks[s_cursor].sample = row; taskEXIT_CRITICAL();
    s_cursor = (uint8_t)((s_cursor + 1U) % s_runtime.task_count);
  }
  Px4Lite_GetImuStats(&fifo); memset(&comm, 0, sizeof(comm)); Px4Lite_GetCommDebugInfo(&comm);
  taskENTER_CRITICAL();
  s_runtime.measured_ms = now_ms; s_runtime.heap_total = (uint32_t)configTOTAL_HEAP_SIZE;
  s_runtime.heap_free = heap_free; s_runtime.heap_minimum = heap_min;
  s_runtime.imu_count = fifo.count; s_runtime.imu_peak = fifo.peak; s_runtime.imu_overflows = fifo.overflow_count;
  s_runtime.rx_drops = comm.rx_drop_count; s_runtime.rx_overflows = comm.rx_overflow_count;
  taskEXIT_CRITICAL();
}
/** @brief comm为唯一写者，回绕使用无符号差值。 */
void Px4Lite_RuntimeReportComm(uint32_t now_ms)
{
  uint32_t period = now_ms - s_comm_last_ms;
  taskENTER_CRITICAL();
  if (s_comm_seen && period > s_runtime.comm_period_max_ms) { s_runtime.comm_period_max_ms = period; }
  s_comm_seen = 1U; s_comm_last_ms = now_ms;
  taskEXIT_CRITICAL();
}
/** @brief 显示测量只能由显示任务产生。 */
void Px4Lite_RuntimeReportDisplay(const Px4Lite_DisplayRuntime_t *sample)
{
  if (!sample) { return; }
  taskENTER_CRITICAL(); s_runtime.display = *sample; taskEXIT_CRITICAL();
}
/** @brief 复制摘要，不重新扫描。 */
Px4Lite_Result_t Px4Lite_RuntimeCopy(Px4Lite_RuntimeSnapshot_t *out)
{
  if (!out) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL(); *out = s_runtime; taskEXIT_CRITICAL(); return PX4LITE_OK;
}
/** @brief 复制一行，未测量通过measured字段表达。 */
Px4Lite_Result_t Px4Lite_RuntimeCopyTask(uint8_t index, Px4Lite_TaskRuntime_t *out)
{
  if (!out) { return PX4LITE_INVALID_PARAM; }
  if (index >= s_runtime.task_count) { return PX4LITE_NOT_READY; }
  taskENTER_CRITICAL(); *out = s_tasks[index].sample; taskEXIT_CRITICAL(); return PX4LITE_OK;
}
