/**
 * @file debug_service.h
 * @brief Declare the independent periodic debug reporting task.
 */

#ifndef DEBUG_SERVICE_H
#define DEBUG_SERVICE_H

#include "FreeRTOS.h"

/**
 * @brief Create the low-priority DebugTask when any periodic monitor is enabled.
 */
BaseType_t DebugService_CreateTask(void);

#endif
