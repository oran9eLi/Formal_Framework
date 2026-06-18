/**
 * @file business_task_registry.c
 * @brief Create the configured business task set.
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

static Px4Lite_Result_t Business_DisplayModuleInit(void)
{
    return (Display_Init() == DISPLAY_OK)
               ? PX4LITE_OK
               : PX4LITE_IO_ERROR;
}

static Px4Lite_Result_t Business_DisplayModuleSelfCheck(void)
{
    uint16_t error_code;

    return (Display_SelfCheck(&error_code) == DISPLAY_OK)
               ? PX4LITE_OK
               : PX4LITE_NOT_READY;
}

static Px4Lite_Result_t Business_DisplayModuleRecover(void)
{
    Display_RequestRecover();
    return PX4LITE_OK;
}

static const Px4Lite_ModuleDescriptor_t s_display_descriptor = {
    PX4LITE_MODULE_DISPLAY,
    "display",
    PX4LITE_ENABLE_DISPLAY,
    0U,
    Business_DisplayModuleInit,
    Business_DisplayModuleSelfCheck,
    0,
    0,
    Business_DisplayModuleRecover
};

/**
 * @brief Initialize business-owned services and create business tasks.
 */
BaseType_t Business_AppInit(void)
{
    if (Business_EventBusInit() != pdPASS)
    {
        return pdFAIL;
    }

#if BUSINESS_ENABLE_DISPLAY
    if (Px4Lite_RegistryRegister(&s_display_descriptor) != PX4LITE_OK)
    {
        return pdFAIL;
    }
#endif

    return Business_CreateTasks();
}

/**
 * @brief Create the configured fixed set of business-layer tasks.
 */
BaseType_t Business_CreateTasks(void)
{
    TaskHandle_t task_handle;

    task_handle = 0;
    if (xTaskCreate(Business_SystemTask,
                    "biz_system",
                    BUSINESS_SYSTEM_TASK_STACK_WORDS,
                    0,
                    BUSINESS_PRIORITY_SYSTEM,
                    &task_handle) != pdPASS)
    {
        return pdFAIL;
    }
    (void)DebugTaskMonitor_Register(
        task_handle,
        "biz_system",
        BUSINESS_SYSTEM_TASK_STACK_WORDS);

    task_handle = 0;
    if (xTaskCreate(Business_AcquisitionTask,
                    "biz_acq",
                    BUSINESS_ACQUISITION_TASK_STACK_WORDS,
                    0,
                    BUSINESS_PRIORITY_ACQUISITION,
                    &task_handle) != pdPASS)
    {
        return pdFAIL;
    }
    (void)DebugTaskMonitor_Register(
        task_handle,
        "biz_acq",
        BUSINESS_ACQUISITION_TASK_STACK_WORDS);

#if BUSINESS_ENABLE_DISPLAY
    task_handle = 0;
    if (xTaskCreate(Business_DisplayServiceTask,
                    "biz_display",
                    BUSINESS_DISPLAY_TASK_STACK_WORDS,
                    0,
                    BUSINESS_PRIORITY_DISPLAY,
                    &task_handle) != pdPASS)
    {
        return pdFAIL;
    }
    (void)DebugTaskMonitor_Register(
        task_handle,
        "biz_display",
        BUSINESS_DISPLAY_TASK_STACK_WORDS);
#endif

    return pdPASS;
}
