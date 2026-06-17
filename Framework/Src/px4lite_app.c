/**
 * @file px4lite_app.c
 * @brief Framework 固定任务集合创建和模块注册实现。
 *
 * @details
 * 本文件负责注册 Framework 核心模块并创建 sensor、estimator、health、comm、
 * storage 等固定任务。任务周期、栈和优先级来自 `px4lite_config.h`，调用处不得
 * 硬编码这些参数。
 */

#include "px4lite_app.h"
#include "px4lite_config.h"
#include "px4lite_modules.h"
#include "px4lite_platform.h"
#include "px4lite_registry.h"
#include "px4lite_topics.h"
#include "px4lite_work.h"
#include "px4lite_storage.h"
#include "debug_console.h"
#include "debug_service.h"
#include "debug_task_monitor.h"
#include "task.h"

static void Px4Lite_SensorTask(void *argument);
static void Px4Lite_EstimatorTask(void *argument);
static void Px4Lite_HealthTask(void *argument);
static void Px4Lite_CommTask(void *argument);
static void Px4Lite_StorageTask(void *argument);
static BaseType_t Px4Lite_RegisterCoreModules(void);

static const Px4Lite_ModuleDescriptor_t s_gnss_descriptor = {
    PX4LITE_MODULE_GNSS,
    "gnss",
    PX4LITE_ENABLE_GNSS,
    1U,
    Px4Lite_GnssModuleInit,
    0,
    0,
    0,
    Px4Lite_GnssRecover
};

#if PX4LITE_ENABLE_IMU
static const Px4Lite_ModuleDescriptor_t s_imu_descriptor = {
    PX4LITE_MODULE_IMU,
    "imu",
    PX4LITE_ENABLE_IMU,
    0U,
    Px4Lite_ImuModuleInit,
    0,
    0,
    0,
    Px4Lite_ImuRecover
};
#endif

#if PX4LITE_ENABLE_BARO
static const Px4Lite_ModuleDescriptor_t s_baro_descriptor = {
    PX4LITE_MODULE_BARO,
    "baro",
    PX4LITE_ENABLE_BARO,
    0U,
    Px4Lite_BaroModuleInit,
    0,
    0,
    0,
    Px4Lite_BaroRecover
};
#endif

#if PX4LITE_ENABLE_BATTERY
static const Px4Lite_ModuleDescriptor_t s_battery_descriptor = {
    PX4LITE_MODULE_BATTERY,
    "battery",
    PX4LITE_ENABLE_BATTERY,
    0U,
    Px4Lite_BatteryModuleInit,
    0,
    0,
    0,
    Px4Lite_BatteryRecover
};
#endif

static const Px4Lite_ModuleDescriptor_t s_estimator_descriptor = {
    PX4LITE_MODULE_ESTIMATOR,
    "estimator",
    1U,
    1U,
    Px4Lite_EstimatorInit,
    0,
    0,
    0,
    Px4Lite_EstimatorInit
};

#if PX4LITE_ENABLE_ALARM
static const Px4Lite_ModuleDescriptor_t s_alarm_descriptor = {
    PX4LITE_MODULE_ALARM,
    "alarm",
    PX4LITE_ENABLE_ALARM,
    0U,
    Px4Lite_AlarmModuleInit,
    0,
    0,
    0,
    0
};
#endif

#if PX4LITE_ENABLE_LORA
static const Px4Lite_ModuleDescriptor_t s_lora_descriptor = {
    PX4LITE_MODULE_LORA,
    "lora",
    PX4LITE_ENABLE_LORA,
    0U,
    Px4Lite_CommModulesInit,
    0,
    0,
    0,
    Px4Lite_LoraRecover
};
#endif

#if PX4LITE_ENABLE_STORAGE
static const Px4Lite_ModuleDescriptor_t s_storage_descriptor = {
    PX4LITE_MODULE_STORAGE,
    "storage",
    PX4LITE_ENABLE_STORAGE,
    0U,
    Px4Lite_StorageModuleInit,
    0,
    0,
    0,
    Px4Lite_StorageRecover
};
#endif

/**
 * @brief 注册当前 Framework 模块及其生命周期回调。
 */
static BaseType_t Px4Lite_RegisterCoreModules(void)
{
    if ((Px4Lite_RegistryRegister(&s_gnss_descriptor) != PX4LITE_OK) ||
        (Px4Lite_RegistryRegister(&s_estimator_descriptor) !=
         PX4LITE_OK))
    {
        return pdFAIL;
    }
#if PX4LITE_ENABLE_IMU
    if (Px4Lite_RegistryRegister(&s_imu_descriptor) != PX4LITE_OK)
    {
        return pdFAIL;
    }
#endif
#if PX4LITE_ENABLE_BARO
    if (Px4Lite_RegistryRegister(&s_baro_descriptor) != PX4LITE_OK)
    {
        return pdFAIL;
    }
#endif
#if PX4LITE_ENABLE_BATTERY
    if (Px4Lite_RegistryRegister(&s_battery_descriptor) != PX4LITE_OK)
    {
        return pdFAIL;
    }
#endif
#if PX4LITE_ENABLE_LORA
    if (Px4Lite_RegistryRegister(&s_lora_descriptor) != PX4LITE_OK)
    {
        return pdFAIL;
    }
#endif
#if PX4LITE_ENABLE_ALARM
    if (Px4Lite_RegistryRegister(&s_alarm_descriptor) != PX4LITE_OK)
    {
        return pdFAIL;
    }
#endif
#if PX4LITE_ENABLE_STORAGE
    if (Px4Lite_RegistryRegister(&s_storage_descriptor) != PX4LITE_OK)
    {
        return pdFAIL;
    }
#endif
    return pdPASS;
}

/**
 * @brief 初始化 Framework 数据服务并创建固定任务集合。
 */
BaseType_t Px4Lite_AppInit(void)
{
    TaskHandle_t task_handle;

    DebugTaskMonitor_Init();
    if ((Px4Lite_TopicsInit() != PX4LITE_OK) ||
        (Px4Lite_ModulesInit() != PX4LITE_OK) ||
        (Px4Lite_RegistryInit() != PX4LITE_OK) ||
        (Px4Lite_RegisterCoreModules() != pdPASS))
    {
        return pdFAIL;
    }

    task_handle = 0;
    if (xTaskCreate(Px4Lite_SensorTask, "sensor",
                    PX4LITE_STACK_SENSOR, 0,
                    PX4LITE_PRIORITY_SENSOR,
                    &task_handle) != pdPASS)
    {
        return pdFAIL;
    }
    (void)DebugTaskMonitor_Register(
        task_handle, "sensor", PX4LITE_STACK_SENSOR);

    task_handle = 0;
    if (xTaskCreate(Px4Lite_EstimatorTask, "estimator",
                    PX4LITE_STACK_ESTIMATOR, 0,
                    PX4LITE_PRIORITY_ESTIMATOR,
                    &task_handle) != pdPASS)
    {
        return pdFAIL;
    }
    (void)DebugTaskMonitor_Register(
        task_handle, "estimator", PX4LITE_STACK_ESTIMATOR);

    task_handle = 0;
    if (xTaskCreate(Px4Lite_HealthTask, "health",
                    PX4LITE_STACK_HEALTH, 0,
                    PX4LITE_PRIORITY_HEALTH,
                    &task_handle) != pdPASS)
    {
        return pdFAIL;
    }
    (void)DebugTaskMonitor_Register(
        task_handle, "health", PX4LITE_STACK_HEALTH);

#if PX4LITE_ENABLE_LORA
    task_handle = 0;
    if (xTaskCreate(Px4Lite_CommTask, "comm",
                    PX4LITE_STACK_COMM, 0,
                    PX4LITE_PRIORITY_COMM,
                    &task_handle) != pdPASS)
    {
        return pdFAIL;
    }
    (void)DebugTaskMonitor_Register(
        task_handle, "comm", PX4LITE_STACK_COMM);
#endif

#if PX4LITE_ENABLE_STORAGE
    task_handle = 0;
    if (xTaskCreate(Px4Lite_StorageTask, "storage",
                    PX4LITE_STACK_STORAGE, 0,
                    PX4LITE_PRIORITY_STORAGE,
                    &task_handle) != pdPASS)
    {
        return pdFAIL;
    }
    (void)DebugTaskMonitor_Register(
        task_handle, "storage", PX4LITE_STACK_STORAGE);
#endif

    if (DebugService_CreateTask() != pdPASS)
    {
        return pdFAIL;
    }

    DBG_BOOT_PRINT("Framework tasks ready, heap_free=%lu bytes",
                   (unsigned long)xPortGetFreeHeapSize());

    return pdPASS;
}

/**
 * @brief 按配置周期服务 LoRa 接收和 MAVLink 发送。
 */
static void Px4Lite_CommTask(void *argument)
{
#if PX4LITE_ENABLE_LORA
    TickType_t last_wake;
    Px4Lite_WorkItem_t work;
    uint32_t now_ms;

    (void)argument;
    now_ms = Px4Lite_PlatformGetMs();
    (void)Px4Lite_RegistryStart(PX4LITE_MODULE_LORA, now_ms);
    (void)Px4Lite_WorkInit(
        &work,
        "comm_work",
        PX4LITE_COMM_PERIOD_MS,
        now_ms,
        Px4Lite_CommWorkRun);
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        now_ms = Px4Lite_PlatformGetMs();
        (void)Px4Lite_WorkRunDue(&work, now_ms);
        Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_COMM,
                                  now_ms);
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(PX4LITE_COMM_PERIOD_MS));
    }
#else
    (void)argument;
    vTaskDelete(0);
#endif
}

/**
 * @brief 按配置周期驱动 SD CSV 存储工作项。
 */
static void Px4Lite_StorageTask(void *argument)
{
#if PX4LITE_ENABLE_STORAGE
    TickType_t last_wake;
    Px4Lite_WorkItem_t work;
    uint32_t now_ms;

    (void)argument;
    now_ms = Px4Lite_PlatformGetMs();
    (void)Px4Lite_RegistryStart(PX4LITE_MODULE_STORAGE, now_ms);
    (void)Px4Lite_WorkInit(
        &work,
        "storage_work",
        PX4LITE_STORAGE_PERIOD_MS,
        now_ms,
        Px4Lite_StorageWorkRun);
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        now_ms = Px4Lite_PlatformGetMs();
        (void)Px4Lite_WorkRunDue(&work, now_ms);
        /* Storage 阻塞 I/O 优先级最低，故意不纳入 watchdog heartbeat 集合。 */
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(PX4LITE_STORAGE_PERIOD_MS));
    }
#else
    (void)argument;
    vTaskDelete(0);
#endif
}

/**
 * @brief 按配置采集周期服务所有启用的传感器驱动。
 */
static void Px4Lite_SensorTask(void *argument)
{
    TickType_t last_wake;
    Px4Lite_WorkItem_t work;
    uint32_t now_ms;

    (void)argument;
    now_ms = Px4Lite_PlatformGetMs();
    (void)Px4Lite_RegistryStart(PX4LITE_MODULE_GNSS, now_ms);
#if PX4LITE_ENABLE_IMU
    (void)Px4Lite_RegistryStart(PX4LITE_MODULE_IMU, now_ms);
#endif
#if PX4LITE_ENABLE_BARO
    (void)Px4Lite_RegistryStart(PX4LITE_MODULE_BARO, now_ms);
#endif
#if PX4LITE_ENABLE_BATTERY
    (void)Px4Lite_RegistryStart(PX4LITE_MODULE_BATTERY, now_ms);
#endif
    (void)Px4Lite_WorkInit(
        &work,
        "sensor_work",
        PX4LITE_SENSOR_WORK_PERIOD_MS,
        now_ms,
        Px4Lite_SensorWorkRun);
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        now_ms = Px4Lite_PlatformGetMs();
        (void)Px4Lite_WorkRunDue(&work, now_ms);
        Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_SENSOR,
                                  now_ms);
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(PX4LITE_SENSOR_WORK_PERIOD_MS));
    }
}

/**
 * @brief 按固定周期执行领域转换和后续融合工作。
 */
static void Px4Lite_EstimatorTask(void *argument)
{
    TickType_t last_wake;
    Px4Lite_WorkItem_t work;
    uint32_t now_ms;

    (void)argument;
    now_ms = Px4Lite_PlatformGetMs();
    (void)Px4Lite_RegistryStart(PX4LITE_MODULE_ESTIMATOR, now_ms);
    (void)Px4Lite_WorkInit(
        &work,
        "estimator_work",
        PX4LITE_ESTIMATOR_PERIOD_MS,
        now_ms,
        Px4Lite_EstimatorRun);
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        now_ms = Px4Lite_PlatformGetMs();
        (void)Px4Lite_WorkRunDue(&work, now_ms);
        Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_ESTIMATOR,
                                  now_ms);
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(PX4LITE_ESTIMATOR_PERIOD_MS));
    }
}

/**
 * @brief 执行健康评估和 watchdog 门控。
 */
static void Px4Lite_HealthTask(void *argument)
{
    TickType_t last_wake;
    Px4Lite_WorkItem_t work;
    uint32_t now_ms;

    (void)argument;
    now_ms = Px4Lite_PlatformGetMs();
#if PX4LITE_ENABLE_ALARM
    (void)Px4Lite_RegistryStart(PX4LITE_MODULE_ALARM, now_ms);
#endif
    (void)Px4Lite_HealthInit();
    (void)Px4Lite_WorkInit(
        &work,
        "health_work",
        PX4LITE_HEALTH_PERIOD_MS,
        now_ms,
        Px4Lite_HealthRun);
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        now_ms = Px4Lite_PlatformGetMs();

        Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_HEALTH,
                                  now_ms);
        (void)Px4Lite_WorkRunDue(&work, now_ms);

        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(PX4LITE_HEALTH_PERIOD_MS));
    }
}
