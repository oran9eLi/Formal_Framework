/**
 * @file business_task_registry.h
 * @brief Business 层 FreeRTOS 任务创建入口。
 *
 * @details
 * 本文件只暴露 Business 层任务初始化和创建函数。任务周期、栈大小和优先级来自
 * `Business/Inc/business_template_config.h`，不得在调用处硬编码。
 */

#ifndef BUSINESS_TASK_REGISTRY_H
#define BUSINESS_TASK_REGISTRY_H

#include "FreeRTOS.h"

/**
 * @brief 初始化 Business 服务并创建 Business 拥有的任务。
 *
 * @return FreeRTOS 执行结果，`pdPASS` 表示成功。
 *
 * @note 应在调度器启动前由系统组合根调用。
 */
BaseType_t Business_AppInit(void);

/**
 * @brief 创建配置中启用的固定 Business 任务集合。
 *
 * @return FreeRTOS 执行结果，`pdPASS` 表示所有任务创建成功。
 */
BaseType_t Business_CreateTasks(void);

#endif
