/**
 * @file debug_console.h
 * @brief Declare thread-safe formatted debug console functions.
 */

#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include <stdint.h>
#include <stdio.h>
#include "debug_config.h"

#if DEBUG_CONSOLE_ENABLE

/**
 * @brief Initialize the debug UART and console synchronization resources.
 */
void DebugConsole_Init(void);
/**
 * @brief Print a byte buffer as hexadecimal diagnostic output.
 */
void DebugConsole_HexDump(const char *prefix, const uint8_t *data, uint16_t len);
/**
 * @brief Format and print a prefixed debug line.
 */
void DebugConsole_Printf(const char *prefix, const char *fmt, ...);
/**
 * @brief Format and print a task-scoped diagnostic line.
 */
void DebugConsole_TaskInfo(const char *name, const char *fmt, ...);

/**
 * @brief Format and print a prefixed debug line.
 */
#define DBG_PRINT(fmt, ...) DebugConsole_Printf("[DBG] ", fmt, ##__VA_ARGS__)
/**
 * @brief Format and print a task-scoped diagnostic line.
 */
#define DBG_TASK_INFO(name, fmt, ...) DebugConsole_TaskInfo((name), fmt, ##__VA_ARGS__)

#else

/**
 * @brief Initialize the debug UART and console synchronization resources.
 */
#define DebugConsole_Init()
/**
 * @brief Print a byte buffer as hexadecimal diagnostic output.
 */
#define DebugConsole_HexDump(p, d, l)
/**
 * @brief Format and print a prefixed debug line.
 */
#define DebugConsole_Printf(prefix, fmt, ...)
/**
 * @brief Format and print a task-scoped diagnostic line.
 */
#define DebugConsole_TaskInfo(name, fmt, ...)
#define DBG_PRINT(fmt, ...)
#define DBG_TASK_INFO(name, fmt, ...)

#endif

#if DEBUG_BOOT_LOG_ENABLE
#define DBG_BOOT_PRINT(fmt, ...) DBG_PRINT(fmt, ##__VA_ARGS__)
#else
#define DBG_BOOT_PRINT(fmt, ...)
#endif

#if DEBUG_BUSINESS_LOG_ENABLE
#define DBG_BUSINESS_PRINT(fmt, ...) DBG_PRINT(fmt, ##__VA_ARGS__)
#else
#define DBG_BUSINESS_PRINT(fmt, ...)
#endif

#endif
