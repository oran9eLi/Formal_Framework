/**
 * @file business_task_registry.c
 * @brief Business 层任务创建和服务初始化实现。
 *
 * @details`r`n * 本文件负责初始化 Business 拥有的服务并创建固定 Business 任务集合。Display`r`n * 模块描述符注册由 `business_platform_adapter.c` 统一收口。任务参数来自`r`n * `business_template_config.h`。
 */

#include "business_task_registry.h"

#include "business_event_bus.h"
#include "business_task_template.h"
#include "business_template_config.h"
#include "debug_task_monitor.h"
#include "task.h"

/**
 * @brief 初始化 Business 拥有的服务并创建 Business 任务。
 *
 * @return FreeRTOS 执行结果，`pdPASS` 表示成功。
 */
BaseType_t Business_AppInit(void)
{
  if (Business_EventBusInit() != pdPASS) { return pdFAIL; }

#if BUSINESS_ENABLE_DISPLAY
  if (Business_PlatformRegisterModules() == 0U) { return pdFAIL; }
#endif

  return Business_CreateTasks();
}

/**
 * @brief 创建配置中启用的固定 Business 任务集合。
 *
 * @return FreeRTOS 执行结果，`pdPASS` 表示所有任务创建成功。
 */
BaseType_t Business_CreateTasks(void)
{
  TaskHandle_t task_handle;

  task_handle = 0;
  if (xTaskCreate(Business_SystemTask, "biz_system", BUSINESS_SYSTEM_TASK_STACK_WORDS, 0, BUSINESS_PRIORITY_SYSTEM, &task_handle) != pdPASS) { return pdFAIL; }
  (void)DebugTaskMonitor_Register(task_handle, "biz_system", BUSINESS_SYSTEM_TASK_STACK_WORDS);

  task_handle = 0;
  if (xTaskCreate(Business_AcquisitionTask, "biz_acq", BUSINESS_ACQUISITION_TASK_STACK_WORDS, 0, BUSINESS_PRIORITY_ACQUISITION, &task_handle) != pdPASS) { return pdFAIL; }
  (void)DebugTaskMonitor_Register(task_handle, "biz_acq", BUSINESS_ACQUISITION_TASK_STACK_WORDS);

#if BUSINESS_ENABLE_DISPLAY
  task_handle = 0;
  if (xTaskCreate(Business_DisplayServiceTask, "biz_display", BUSINESS_DISPLAY_TASK_STACK_WORDS, 0, BUSINESS_PRIORITY_DISPLAY, &task_handle) != pdPASS) { return pdFAIL; }
  (void)DebugTaskMonitor_Register(task_handle, "biz_display", BUSINESS_DISPLAY_TASK_STACK_WORDS);
#endif

  return pdPASS;
}
