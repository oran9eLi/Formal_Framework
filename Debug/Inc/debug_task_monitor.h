/**
 * @file debug_task_monitor.h
 * @brief Declare optional task stack and heap diagnostics.
 */

#ifndef DEBUG_TASK_MONITOR_H
#define DEBUG_TASK_MONITOR_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/**
 * @brief Reset the debug-only task stack registry.
 */
void DebugTaskMonitor_Init(void);

/**
 * @brief Register one application task and its configured stack depth in words.
 */
BaseType_t DebugTaskMonitor_Register(
    TaskHandle_t handle,
    const char *name,
    uint16_t stack_words);

/**
 * @brief Print registered task stack high-water marks and heap statistics.
 */
void DebugTaskMonitor_Report(void);

#endif
