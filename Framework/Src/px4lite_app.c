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
#include "px4lite_control.h"
#include "px4lite_modules.h"
#include "px4lite_platform.h"
#include "px4lite_registry.h"
#include "px4lite_topics.h"
#include "px4lite_time.h"
#include "px4lite_work.h"
#include "px4lite_storage.h"

#include <stddef.h>
#include "debug_console.h"
#include "debug_config.h"
#include "debug_service.h"
#include "debug_task_monitor.h"
#include "bsp_adc_current.h" /* 临时诊断：独立电流串口打印用 */
#include "task.h"
#include <stddef.h>

static void Px4Lite_SensorTask(void *argument);
static void Px4Lite_EstimatorTask(void *argument);
static void Px4Lite_HealthTask(void *argument);
static void Px4Lite_CommTask(void *argument);
static void Px4Lite_ControlTask(void *argument);
static void Px4Lite_StorageTask(void *argument);
static BaseType_t Px4Lite_RegisterCoreModules(void);

static const Px4Lite_ModuleDescriptor_t s_gnss_descriptor = {PX4LITE_MODULE_GNSS, "gnss", PX4LITE_ENABLE_GNSS, 1U, Px4Lite_GnssModuleInit, NULL, NULL, NULL, Px4Lite_GnssRecover};

#if PX4LITE_ENABLE_IMU
static const Px4Lite_ModuleDescriptor_t s_imu_descriptor = {PX4LITE_MODULE_IMU, "imu", PX4LITE_ENABLE_IMU, 0U, Px4Lite_ImuModuleInit, NULL, NULL, NULL, Px4Lite_ImuRecover};
#endif

#if PX4LITE_ENABLE_BARO
static const Px4Lite_ModuleDescriptor_t s_baro_descriptor = {PX4LITE_MODULE_BARO, "baro", PX4LITE_ENABLE_BARO, 0U, Px4Lite_BaroModuleInit, NULL, NULL, NULL, Px4Lite_BaroRecover};
#endif

#if PX4LITE_ENABLE_BATTERY
static const Px4Lite_ModuleDescriptor_t s_battery_descriptor = {PX4LITE_MODULE_BATTERY, "battery", PX4LITE_ENABLE_BATTERY, 0U, Px4Lite_BatteryModuleInit, NULL, NULL, NULL, Px4Lite_BatteryRecover};
#endif

static const Px4Lite_ModuleDescriptor_t s_estimator_descriptor = {PX4LITE_MODULE_ESTIMATOR, "estimator", 1U, 1U, Px4Lite_EstimatorInit, NULL, NULL, NULL, Px4Lite_EstimatorInit};

#if PX4LITE_ENABLE_ALARM
static const Px4Lite_ModuleDescriptor_t s_alarm_descriptor = {PX4LITE_MODULE_ALARM, "alarm", PX4LITE_ENABLE_ALARM, 0U, Px4Lite_AlarmModuleInit, NULL, NULL, NULL, NULL};
#endif

#if PX4LITE_ENABLE_LORA
static const Px4Lite_ModuleDescriptor_t s_lora_descriptor = {PX4LITE_MODULE_LORA, "lora", PX4LITE_ENABLE_LORA, 0U, Px4Lite_CommModulesInit, NULL, NULL, NULL, Px4Lite_LoraRecover};
#endif

#if PX4LITE_ENABLE_REMOTE_ID
static const Px4Lite_ModuleDescriptor_t s_remoteid_descriptor = {PX4LITE_MODULE_REMOTE_ID, "remoteid", PX4LITE_ENABLE_REMOTE_ID, 0U, Px4Lite_RemoteIdModuleInit, NULL, NULL, NULL, Px4Lite_RemoteIdRecover};
#endif

#if PX4LITE_ENABLE_STORAGE
static const Px4Lite_ModuleDescriptor_t s_storage_descriptor = {PX4LITE_MODULE_STORAGE, "storage", PX4LITE_ENABLE_STORAGE, 0U, Px4Lite_StorageModuleInit, NULL, NULL, NULL, Px4Lite_StorageRecover};
#endif

#if PX4LITE_ENABLE_CONTROL
static const Px4Lite_ModuleDescriptor_t s_control_descriptor = {PX4LITE_MODULE_CONTROL, "control", PX4LITE_ENABLE_CONTROL, 0U, Px4Lite_ControlModuleInit, NULL, NULL, NULL, Px4Lite_ControlRecover};
#endif

/**
 * @brief 注册当前 Framework 模块及其生命周期回调。
 */
static BaseType_t Px4Lite_RegisterCoreModules(void)
{
  if ((Px4Lite_RegistryRegister(&s_gnss_descriptor) != PX4LITE_OK) || (Px4Lite_RegistryRegister(&s_estimator_descriptor) != PX4LITE_OK)) { return pdFAIL; }
#if PX4LITE_ENABLE_IMU
  if (Px4Lite_RegistryRegister(&s_imu_descriptor) != PX4LITE_OK) { return pdFAIL; }
#endif
#if PX4LITE_ENABLE_BARO
  if (Px4Lite_RegistryRegister(&s_baro_descriptor) != PX4LITE_OK) { return pdFAIL; }
#endif
#if PX4LITE_ENABLE_BATTERY
  if (Px4Lite_RegistryRegister(&s_battery_descriptor) != PX4LITE_OK) { return pdFAIL; }
#endif
#if PX4LITE_ENABLE_LORA
  if (Px4Lite_RegistryRegister(&s_lora_descriptor) != PX4LITE_OK) { return pdFAIL; }
#endif
#if PX4LITE_ENABLE_REMOTE_ID
  if (Px4Lite_RegistryRegister(&s_remoteid_descriptor) != PX4LITE_OK) { return pdFAIL; }
#endif
#if PX4LITE_ENABLE_ALARM
  if (Px4Lite_RegistryRegister(&s_alarm_descriptor) != PX4LITE_OK) { return pdFAIL; }
#endif
#if PX4LITE_ENABLE_STORAGE
  if (Px4Lite_RegistryRegister(&s_storage_descriptor) != PX4LITE_OK) { return pdFAIL; }
#endif
#if PX4LITE_ENABLE_CONTROL
  if (Px4Lite_RegistryRegister(&s_control_descriptor) != PX4LITE_OK) { return pdFAIL; }
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
  if (Px4Lite_TopicsInit() != PX4LITE_OK) {
    DBG_BOOT_PRINT("framework init failed: topics");
    return pdFAIL;
  }
  if (Px4Lite_ModulesInit() != PX4LITE_OK) {
    DBG_BOOT_PRINT("framework init failed: modules");
    return pdFAIL;
  }
  if (Px4Lite_TimeInit() != PX4LITE_OK) {
    DBG_BOOT_PRINT("framework init failed: time");
    return pdFAIL;
  }
  if (Px4Lite_RegistryInit() != PX4LITE_OK) {
    DBG_BOOT_PRINT("framework init failed: registry");
    return pdFAIL;
  }
  if (Px4Lite_RegisterCoreModules() != pdPASS) {
    DBG_BOOT_PRINT("framework init failed: module register");
    return pdFAIL;
  }

  task_handle = 0;
  if (xTaskCreate(Px4Lite_SensorTask, "sensor", PX4LITE_STACK_SENSOR, 0, PX4LITE_PRIORITY_SENSOR, &task_handle) != pdPASS) { return pdFAIL; }
  (void)DebugTaskMonitor_Register(task_handle, "sensor", PX4LITE_STACK_SENSOR);

  task_handle = 0;
  if (xTaskCreate(Px4Lite_EstimatorTask, "estimator", PX4LITE_STACK_ESTIMATOR, 0, PX4LITE_PRIORITY_ESTIMATOR, &task_handle) != pdPASS) { return pdFAIL; }
  (void)DebugTaskMonitor_Register(task_handle, "estimator", PX4LITE_STACK_ESTIMATOR);

  task_handle = 0;
  if (xTaskCreate(Px4Lite_HealthTask, "health", PX4LITE_STACK_HEALTH, 0, PX4LITE_PRIORITY_HEALTH, &task_handle) != pdPASS) { return pdFAIL; }
  (void)DebugTaskMonitor_Register(task_handle, "health", PX4LITE_STACK_HEALTH);

#if PX4LITE_ENABLE_LORA || PX4LITE_ENABLE_REMOTE_ID
  task_handle = 0;
  if (xTaskCreate(Px4Lite_CommTask, "comm", PX4LITE_STACK_COMM, 0, PX4LITE_PRIORITY_COMM, &task_handle) != pdPASS) { return pdFAIL; }
  (void)DebugTaskMonitor_Register(task_handle, "comm", PX4LITE_STACK_COMM);
#endif

#if PX4LITE_ENABLE_STORAGE
  task_handle = 0;
  if (xTaskCreate(Px4Lite_StorageTask, "storage", PX4LITE_STACK_STORAGE, 0, PX4LITE_PRIORITY_STORAGE, &task_handle) != pdPASS) { return pdFAIL; }
  (void)DebugTaskMonitor_Register(task_handle, "storage", PX4LITE_STACK_STORAGE);
#endif

#if PX4LITE_ENABLE_CONTROL
  task_handle = 0;
  if (xTaskCreate(Px4Lite_ControlTask, "control", PX4LITE_STACK_CONTROL, 0, PX4LITE_PRIORITY_CONTROL, &task_handle) != pdPASS) { return pdFAIL; }
  (void)DebugTaskMonitor_Register(task_handle, "control", PX4LITE_STACK_CONTROL);
#endif

  if (DebugService_CreateTask() != pdPASS) { return pdFAIL; }

  if (xPortGetFreeHeapSize() < (size_t)PX4LITE_HEAP_MIN_FREE_BYTES) {
    DBG_BOOT_PRINT("boot failed: heap_free=%lu bytes < %lu bytes", (unsigned long)xPortGetFreeHeapSize(), (unsigned long)PX4LITE_HEAP_MIN_FREE_BYTES);
    return pdFAIL;
  }

  DBG_BOOT_PRINT("Framework tasks ready, heap_free=%lu bytes", (unsigned long)xPortGetFreeHeapSize());

  return pdPASS;
}

/**
 * @brief 按固定周期下发电机控制输出。
 */
static void Px4Lite_ControlTask(void *argument)
{
#if PX4LITE_ENABLE_CONTROL
  TickType_t last_wake;
  uint32_t now_ms;

  (void)argument;
  now_ms = Px4Lite_PlatformGetMs();
  (void)Px4Lite_RegistryStart(PX4LITE_MODULE_CONTROL, now_ms);
  last_wake = xTaskGetTickCount();

  for (;;) {
    now_ms = Px4Lite_PlatformGetMs();
    Px4Lite_ControlRun(now_ms);
    Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_CONTROL, now_ms);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PX4LITE_CONTROL_PERIOD_MS));
  }
#else
  (void)argument;
  vTaskDelete(0);
#endif
}

/**
 * @brief 按配置周期服务 LoRa 接收和 MAVLink 发送。
 */
static void Px4Lite_CommTask(void *argument)
{
#if PX4LITE_ENABLE_LORA || PX4LITE_ENABLE_REMOTE_ID
  TickType_t last_wake;
  Px4Lite_WorkItem_t work;
  uint32_t now_ms;

  (void)argument;
  now_ms = Px4Lite_PlatformGetMs();
#if PX4LITE_ENABLE_LORA
  (void)Px4Lite_RegistryStart(PX4LITE_MODULE_LORA, now_ms);
#endif
#if PX4LITE_ENABLE_REMOTE_ID
  (void)Px4Lite_RegistryStart(PX4LITE_MODULE_REMOTE_ID, now_ms);
#endif
  (void)Px4Lite_WorkInit(&work, "comm_work", PX4LITE_COMM_PERIOD_MS, now_ms, Px4Lite_CommWorkRun);
  last_wake = xTaskGetTickCount();

  for (;;) {
    now_ms = Px4Lite_PlatformGetMs();
    (void)Px4Lite_WorkRunDue(&work, now_ms);
    Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_COMM, now_ms);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PX4LITE_COMM_PERIOD_MS));
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
  (void)Px4Lite_WorkInit(&work, "storage_work", PX4LITE_STORAGE_PERIOD_MS, now_ms, Px4Lite_StorageWorkRun);
  last_wake = xTaskGetTickCount();

  for (;;) {
    now_ms = Px4Lite_PlatformGetMs();
    (void)Px4Lite_WorkRunDue(&work, now_ms);
    /* Storage 阻塞 I/O 优先级最低，故意不纳入 watchdog heartbeat 集合。 */
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PX4LITE_STORAGE_PERIOD_MS));
  }
#else
  (void)argument;
  vTaskDelete(0);
#endif
}

/**
 * @brief 按配置采集周期服务所有启用的传感器驱动。
 */
#if DEBUG_CONSOLE_ENABLE
/*
 * 临时诊断：独立电流串口打印。挂在高优先级 sensor 任务(必然执行)，走互斥保护的
 * DebugConsole，与 debug 服务任务无关。n 为心跳计数——只要看到 [CUR] 行且 n 递增，
 * 就说明串口链路通、sensor 任务在跑；连 [CUR] 都没有 = 硬件/接线/固件问题。
 * mv=减零点前的原始引脚电压(mV)，f=浮空标志，ma=换算电流。
 * 定位电流问题后删除本函数及其调用即可。
 */
static void Px4Lite_SensorReportCurrent(uint32_t now_ms)
{
  static uint32_t s_last_ms;
  static uint32_t s_seq;
  Px4Lite_BatteryStatus_t b1;
  Px4Lite_BatteryStatus_t b2;
  uint32_t mv1 = 0U;
  uint32_t mv2 = 0U;
  uint8_t  f1  = 0U;
  uint8_t  f2  = 0U;
  long ma1;
  long ma2;

  if ((s_last_ms != 0U) && ((uint32_t)(now_ms - s_last_ms) < 1000U)) { return; }
  s_last_ms = now_ms;
  s_seq++;

  BSP_ADC_Current_GetDiag(BSP_ADC_CURRENT_BATTERY1, &mv1, &f1);
  BSP_ADC_Current_GetDiag(BSP_ADC_CURRENT_BATTERY2, &mv2, &f2);
  {
    Px4Lite_Result_t rc1 = Px4Lite_CopyBattery(&b1);
    Px4Lite_Result_t rc2 = Px4Lite_CopyBattery2(&b2);
    unsigned long v1_mv = (rc1 == PX4LITE_OK) ? (unsigned long)b1.voltage_mv : 0UL;
    unsigned long v2_mv = (rc2 == PX4LITE_OK) ? (unsigned long)b2.voltage_mv : 0UL;
    ma1 = (rc1 == PX4LITE_OK) ? (long)b1.current_ma : 0L;
    ma2 = (rc2 == PX4LITE_OK) ? (long)b2.current_ma : 0L;

    /* V1(PA5)/V2(PA4) 为电池电压，用于一眼看清电压/电流是否接反：
       V2 稳定 ~11000mV = VOLT 正确接在 PA4；V2 随电机油门乱跳 = CURR 错接在 PA4。 */
    DebugConsole_Printf("[CUR] ", "n=%lu I1(PC0) mv=%lu f=%u ma=%ld V1(PA5)=%lu | I2(PC1) mv=%lu f=%u ma=%ld V2(PA4)=%lu",
                        (unsigned long)s_seq, (unsigned long)mv1, (unsigned int)f1, ma1, v1_mv,
                        (unsigned long)mv2, (unsigned int)f2, ma2, v2_mv);
  }
}
#endif

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
  (void)Px4Lite_WorkInit(&work, "sensor_work", PX4LITE_SENSOR_WORK_PERIOD_MS, now_ms, Px4Lite_SensorWorkRun);
  last_wake = xTaskGetTickCount();

  for (;;) {
    now_ms = Px4Lite_PlatformGetMs();
    (void)Px4Lite_WorkRunDue(&work, now_ms);
#if DEBUG_CONSOLE_ENABLE
    Px4Lite_SensorReportCurrent(now_ms); /* 临时诊断电流打印，定位后删除 */
#endif
    Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_SENSOR, now_ms);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PX4LITE_SENSOR_WORK_PERIOD_MS));
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
  (void)Px4Lite_WorkInit(&work, "estimator_work", PX4LITE_ESTIMATOR_PERIOD_MS, now_ms, Px4Lite_EstimatorRun);
  last_wake = xTaskGetTickCount();

  for (;;) {
    now_ms = Px4Lite_PlatformGetMs();
    (void)Px4Lite_WorkRunDue(&work, now_ms);
    Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_ESTIMATOR, now_ms);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PX4LITE_ESTIMATOR_PERIOD_MS));
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
  (void)Px4Lite_WorkInit(&work, "health_work", PX4LITE_HEALTH_PERIOD_MS, now_ms, Px4Lite_HealthRun);
  last_wake = xTaskGetTickCount();

  for (;;) {
    now_ms = Px4Lite_PlatformGetMs();

    Px4Lite_PlatformHeartbeat(PX4LITE_HEARTBEAT_HEALTH, now_ms);
    (void)Px4Lite_WorkRunDue(&work, now_ms);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PX4LITE_HEALTH_PERIOD_MS));
  }
}
