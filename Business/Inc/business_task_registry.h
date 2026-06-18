/**
 * @file business_task_registry.h
 * @brief Declare creation of business-layer FreeRTOS tasks.
 */

#ifndef BUSINESS_TASK_REGISTRY_H
#define BUSINESS_TASK_REGISTRY_H

#include "FreeRTOS.h"

/**
 * @brief Initialize business services and create business-owned tasks.
 */
BaseType_t Business_AppInit(void);

/**
 * @brief Create the configured fixed set of business-layer tasks.
 */
BaseType_t Business_CreateTasks(void);

#endif
