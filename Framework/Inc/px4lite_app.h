/**
 * @file px4lite_app.h
 * @brief Framework 初始化和固定任务创建入口。
 *
 * @details
 * 本文件是 Framework 层被 `main.c` 调用的组合入口。它负责初始化 Framework
 * 数据服务、注册模块并创建固定任务集合；业务任务由 Business 层自己的入口创建。
 */

#ifndef PX4LITE_APP_H
#define PX4LITE_APP_H

#include "FreeRTOS.h"

/**
 * @brief 初始化 Framework 数据服务并创建固定任务集合。
 *
 * @return FreeRTOS 执行结果，`pdPASS` 表示初始化和任务创建成功。
 *
 * @note 应在 FreeRTOS 调度器启动前调用。
 */
BaseType_t Px4Lite_AppInit(void);

#endif
