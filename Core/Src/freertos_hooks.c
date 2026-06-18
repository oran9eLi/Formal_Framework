/**
 * @file freertos_hooks.c
 * @brief Handle FreeRTOS stack overflow and allocation failure events.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

volatile TaskHandle_t g_stack_overflow_task;
volatile uint32_t g_malloc_failed_count;

/**
 * @brief Capture the failed task and enter the fatal error path on stack overflow.
 */
void vApplicationStackOverflowHook(TaskHandle_t task,
                                   char *task_name)
{
    (void)task_name;
    g_stack_overflow_task = task;
    Error_Handler();
}

/**
 * @brief Count allocation failures and enter the fatal error path.
 */
void vApplicationMallocFailedHook(void)
{
    g_malloc_failed_count++;
    Error_Handler();
}
