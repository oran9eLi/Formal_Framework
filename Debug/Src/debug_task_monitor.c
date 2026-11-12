/**
 * @file debug_task_monitor.c
 * @brief 兼容任务登记入口；常驻资源表归Framework，串口打印为可选消费者。
 * @details 关闭debug打印仍登记任务；栈扫描由biz_system分次执行，报告只复制副本。
 */

#include "debug_task_monitor.h"

#include "debug_config.h"
#include "debug_console.h"
#include <string.h>

#include "px4lite_runtime_stats.h"

#if DEBUG_STACK_MONITOR_ENABLE
/** @brief DebugTask独占的资源摘要副本，不在384word任务栈上叠放大结构。 */
static Px4Lite_RuntimeSnapshot_t s_debug_runtime;
/** @brief 顺序复制的单个任务水位，不在此执行栈扫描或采样。 */
static Px4Lite_TaskRuntime_t s_debug_task_row;
#endif

/**
 * @brief 启动阶段复位唯一任务资源表。
 */
void DebugTaskMonitor_Init(void)
{
  Px4Lite_RuntimeInit();
}

/**
 * @brief 调度前登记任务及配置栈深度，单位word；失败返回pdFAIL。
 */
BaseType_t DebugTaskMonitor_Register(TaskHandle_t handle, const char *name, uint16_t stack_words)
{
  return Px4Lite_RuntimeRegister(handle, name, stack_words);
}

/**
 * @brief 按开关打印已测量的水位副本；不执行第二次全表扫描。
 */
void DebugTaskMonitor_Report(void)
{
#if DEBUG_STACK_MONITOR_ENABLE
  uint8_t index;
  if(Px4Lite_RuntimeCopy(&s_debug_runtime)!=PX4LITE_OK) { DBG_PRINT("MEM: no resource snapshot");return; }
  DBG_PRINT("MEM: cfg=%lu free=%lu min=%lu sensor_max_us=%lu tasks=%u", (unsigned long)s_debug_runtime.heap_total,
    (unsigned long)s_debug_runtime.heap_free, (unsigned long)s_debug_runtime.heap_minimum,
    (unsigned long)s_debug_runtime.sensor_service_max_us, (unsigned int)s_debug_runtime.task_count);
  for (index = 0U; index < s_debug_runtime.task_count && index<PX4LITE_RUNTIME_TASK_CAPACITY; ++index) {
    if (Px4Lite_RuntimeCopyTask(index, &s_debug_task_row) == PX4LITE_OK && s_debug_task_row.measured) {
      DBG_PRINT("STACK: %.16s cfg=%uW min=%uW sample_ms=%lu %s", s_debug_task_row.name?s_debug_task_row.name:"?",
        (unsigned int)s_debug_task_row.configured_words, (unsigned int)s_debug_task_row.minimum_free_words,
        (unsigned long)s_debug_task_row.measured_ms, s_debug_task_row.warning ? "WARN" : "OK");
    }
  }
#endif
}
