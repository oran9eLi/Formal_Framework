/**
 * @file business_task_registry.c
 * @brief Business 层任务创建和 Display 模块注册实现。
 *
 * @details
 * 本文件负责初始化 Business 拥有的服务、注册 Display 模块描述符并创建固定
 * Business 任务集合。任务参数来自 `business_template_config.h`。
 */

#include "business_task_registry.h"

#include "business_event_bus.h"
#include "business_task_template.h"
#include "business_template_config.h"
#include "debug_task_monitor.h"
#include "display.h"
#include "px4lite_config.h"
#include "px4lite_registry.h"
#include "task.h"

/**
 * @brief Display 模块注册表 init 回调。
 *
 * @return 初始化结果。
 */
static Px4Lite_Result_t Business_DisplayModuleInit(void)
{
  return (Display_Init() == DISPLAY_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

/**
 * @brief Display 模块注册表 self_check 回调。
 *
 * @return 自检结果。
 */
static Px4Lite_Result_t Business_DisplayModuleSelfCheck(void)
{
  uint16_t error_code;

  return (Display_SelfCheck(&error_code) == DISPLAY_OK) ? PX4LITE_OK : PX4LITE_NOT_READY;
}

/**
 * @brief Display 模块注册表 recover 回调。
 *
 * @return 恢复请求结果。
 *
 * @note 本函数只请求显示模块恢复，实际恢复由显示服务上下文完成。
 */
static Px4Lite_Result_t Business_DisplayModuleRecover(void)
{
  Display_RequestRecover();
  return PX4LITE_OK;
}

static const Px4Lite_ModuleDescriptor_t s_display_descriptor = {PX4LITE_MODULE_DISPLAY, "display", PX4LITE_ENABLE_DISPLAY, 0U, Business_DisplayModuleInit, Business_DisplayModuleSelfCheck, 0, 0, Business_DisplayModuleRecover};

/**
 * @brief 初始化 Business 拥有的服务并创建 Business 任务。
 *
 * @return FreeRTOS 执行结果，`pdPASS` 表示成功。
 */
BaseType_t Business_AppInit(void)
{
  if (Business_EventBusInit() != pdPASS) { return pdFAIL; }

#if BUSINESS_ENABLE_DISPLAY
  if (Px4Lite_RegistryRegister(&s_display_descriptor) != PX4LITE_OK) { return pdFAIL; }
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
