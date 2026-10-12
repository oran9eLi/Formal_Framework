/**
 * @file freertos_hooks.c
 * @brief Core 层处理 FreeRTOS 栈溢出/分配失败；先撤收输出，再保存最小现场。
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "core_fatal.h"

#include <stdint.h>

volatile TaskHandle_t g_stack_overflow_task;
volatile uintptr_t g_stack_overflow_task_name_ptr;
volatile char g_stack_overflow_task_name[configMAX_TASK_NAME_LEN];
volatile uint32_t g_malloc_failed_count;

/**
 * @brief 保存发生栈溢出的任务名副本。
 *
 * @param[in] task_name FreeRTOS 提供的任务名指针，允许为 NULL。
 *
 * @note 栈溢出现场不做 printf，只复制固定长度字符，便于调试器事后查看。
 */
static void FreeRtosHooks_CopyTaskName(char *task_name)
{
  uint32_t i;

  g_stack_overflow_task_name_ptr = (uintptr_t)task_name;
  for (i = 0U; i < (uint32_t)(configMAX_TASK_NAME_LEN - 1); ++i) {
    if ((Core_FatalRamRangeValid((uintptr_t)task_name + i, 1U) == 0U) || (task_name[i] == '\0')) { break; }
    g_stack_overflow_task_name[i] = task_name[i];
  }
  g_stack_overflow_task_name[i] = '\0';
}

/**
 * @brief 栈溢出先撤收输出，再尽可能保存任务信息。
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
  Core_FatalQuiesce();
  g_stack_overflow_task = task;
  FreeRtosHooks_CopyTaskName(task_name);
  Error_Handler();
}

/**
 * @brief 分配失败先撤收输出，再记录计数并停机。
 */
void vApplicationMallocFailedHook(void)
{
  Core_FatalQuiesce();
  g_malloc_failed_count++;
  Error_Handler();
}
