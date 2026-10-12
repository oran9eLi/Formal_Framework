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
  Px4Lite_RuntimeSnapshot_t runtime;
  Px4Lite_TaskRuntime_t row;
  uint8_t index;
  (void)Px4Lite_RuntimeCopy(&runtime);
  DBG_PRINT("MEM: cfg=%lu free=%lu min=%lu", (unsigned long)runtime.heap_total, (unsigned long)runtime.heap_free, (unsigned long)runtime.heap_minimum);
  for (index = 0U; index < runtime.task_count; ++index) {
    if (Px4Lite_RuntimeCopyTask(index, &row) == PX4LITE_OK && row.measured) {
      DBG_PRINT("STACK: %s cfg=%uW min=%uW %s", row.name, (unsigned int)row.configured_words,
        (unsigned int)row.minimum_free_words, row.warning ? "WARN" : "OK");
    }
  }
#endif
}
