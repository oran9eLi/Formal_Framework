/**
 * @file debug_task_monitor.c
 * @brief Implement optional serial reporting of task stack and heap usage.
 */

#include "debug_task_monitor.h"

#include "debug_config.h"
#include "debug_console.h"
#include <string.h>

#define DEBUG_TASK_MONITOR_CAPACITY 16U
#define DEBUG_STACK_WARN_MIN_WORDS  100U

typedef struct {
  TaskHandle_t handle;
  const char *name;
  uint16_t configured_words;
} DebugTaskMonitor_Record_t;

#if DEBUG_STACK_MONITOR_ENABLE
static DebugTaskMonitor_Record_t s_task_records[DEBUG_TASK_MONITOR_CAPACITY];
static uint16_t s_task_count;
#endif

/**
 * @brief Reset the debug-only task stack registry.
 */
void DebugTaskMonitor_Init(void)
{
#if DEBUG_STACK_MONITOR_ENABLE
  memset(s_task_records, 0, sizeof(s_task_records));
  s_task_count = 0U;
#endif
}

/**
 * @brief Register one application task and its configured stack depth in words.
 */
BaseType_t DebugTaskMonitor_Register(TaskHandle_t handle, const char *name, uint16_t stack_words)
{
#if DEBUG_STACK_MONITOR_ENABLE
  if ((handle == 0) || (name == 0) || (stack_words == 0U) || (s_task_count >= DEBUG_TASK_MONITOR_CAPACITY)) { return pdFAIL; }

  s_task_records[s_task_count].handle           = handle;
  s_task_records[s_task_count].name             = name;
  s_task_records[s_task_count].configured_words = stack_words;
  s_task_count++;
  return pdPASS;
#else
  (void)handle;
  (void)name;
  (void)stack_words;
  return pdPASS;
#endif
}

/**
 * @brief Print registered task stack high-water marks and heap statistics.
 */
void DebugTaskMonitor_Report(void)
{
#if DEBUG_STACK_MONITOR_ENABLE
  uint16_t index;
  size_t heap_free;
  size_t heap_min;

  heap_free = xPortGetFreeHeapSize();
  heap_min  = xPortGetMinimumEverFreeHeapSize();
  DBG_PRINT("MEM: heap_cfg=%lu heap_free=%lu heap_min=%lu heap_peak_used=%lu", (unsigned long)configTOTAL_HEAP_SIZE, (unsigned long)heap_free, (unsigned long)heap_min, (unsigned long)(configTOTAL_HEAP_SIZE - heap_min));

  for (index = 0U; index < s_task_count; ++index) {
    UBaseType_t free_words;
    uint32_t used_words;
    uint8_t warning;

    free_words = uxTaskGetStackHighWaterMark(s_task_records[index].handle);
    used_words = (free_words < s_task_records[index].configured_words) ? ((uint32_t)s_task_records[index].configured_words - (uint32_t)free_words) : 0U;
    warning    = ((free_words < DEBUG_STACK_WARN_MIN_WORDS) || (((uint32_t)free_words * 4U) < s_task_records[index].configured_words)) ? 1U : 0U;

    DBG_PRINT("STACK: task=%s cfg=%uW peak=%luW min_free=%luW/%luB %s", s_task_records[index].name, (unsigned int)s_task_records[index].configured_words, (unsigned long)used_words, (unsigned long)free_words, (unsigned long)((uint32_t)free_words * sizeof(StackType_t)), (warning != 0U) ? "WARN" : "OK");
  }
#endif
}
