/**
 * @file business_task_template.c
 * @brief Business 层固定周期系统、采集和显示任务实现。
 *
 * @details
 * Business 任务只通过应用数据接口消费 Framework 快照，不直接访问 BSP、Sensor
 * 或驱动私有变量。周期任务使用 `vTaskDelayUntil()` 保持固定周期。
 */

#include "business_task_template.h"

#include "business_template_config.h"
#include "FreeRTOS.h"
#include "px4lite_registry.h"
#include "task.h"

#define BUSINESS_LOG_LEVEL_INFO 1U

#if BUSINESS_ENABLE_DISPLAY
/**
 * @brief 执行支持回绕的毫秒 deadline 判断。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 * @param[in] deadline_ms 目标 deadline，单位：ms。
 *
 * @return 1 表示 deadline 已到达，0 表示尚未到达。
 */
static uint8_t Business_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (((int32_t)(now_ms - deadline_ms)) >= 0) ? 1U : 0U;
}
#endif

/**
 * @brief Business 系统任务入口。
 *
 * @param[in] argument FreeRTOS 任务参数，当前未使用。
 *
 * @note 周期为 `BUSINESS_SYSTEM_PERIOD_MS`，负责启动日志、应用注册表轮询和系统心跳。
 */
void Business_SystemTask(void *argument)
{
  Business_LogRecord_t startup_record;
  TickType_t last_wake;

  (void)argument;
  startup_record.timestamp_ms = Business_PlatformGetMs();
  startup_record.source_id    = (uint16_t)BUSINESS_COMPONENT_SYSTEM;
  startup_record.level        = BUSINESS_LOG_LEVEL_INFO;
  startup_record.reserved     = 0U;
  startup_record.field        = "system.startup";
  startup_record.value        = "ok";

  /* 启动日志是边沿事件，只写一次。 */
  (void)Business_LogWrite(&startup_record);
  last_wake = xTaskGetTickCount();

  for (;;) {
    uint32_t now_ms = Business_PlatformGetMs();

    Business_RegistryPoll(now_ms);
    Business_StatusHeartbeat(BUSINESS_COMPONENT_SYSTEM);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BUSINESS_SYSTEM_PERIOD_MS));
  }
}

/**
 * @brief Business 采集任务入口。
 *
 * @param[in] argument FreeRTOS 任务参数，当前未使用。
 *
 * @note 周期为 `BUSINESS_ACQUISITION_PERIOD_MS`，负责复制 Navigation 快照并发布业务事件。
 */
void Business_AcquisitionTask(void *argument)
{
  TickType_t last_wake;

  (void)argument;
  last_wake = xTaskGetTickCount();

  for (;;) {
    uint32_t now_ms = Business_PlatformGetMs();

    (void)Business_AcquisitionRunOnce(now_ms);
    Business_StatusHeartbeat(BUSINESS_COMPONENT_ACQUISITION);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BUSINESS_ACQUISITION_PERIOD_MS));
  }
}

/**
 * @brief Business 显示服务任务入口。
 *
 * @param[in] argument FreeRTOS 任务参数，当前未使用。
 *
 * @note 触摸轮询优先于页面刷新；页面刷新通过预算化 step 防止阻塞传感器采集。
 */
void Business_DisplayServiceTask(void *argument)
{
#if BUSINESS_ENABLE_DISPLAY
  TickType_t last_wake;
  uint32_t next_refresh_ms;
  uint8_t refresh_pending = 0U;

  (void)argument;
  last_wake       = xTaskGetTickCount();
  next_refresh_ms = Business_PlatformGetMs();
  (void)Px4Lite_RegistryStart(PX4LITE_MODULE_DISPLAY, next_refresh_ms);

  for (;;) {
    Business_ServiceResult_t result;
    uint32_t now_ms = Business_PlatformGetMs();

    /*
     * Input is always serviced first. Display refresh cannot suppress
     * touch polling merely because one refresh step failed.
     */
    result = Business_DisplayPollTouch(now_ms);
    if ((result != BUSINESS_SERVICE_OK) && (result != BUSINESS_SERVICE_IDLE)) { Business_DisplayReportResult(result, now_ms); }

    /* KEY0 弹出隐藏页：按下边沿即切页，整页重绘由下方立即刷新路径处理 */
    result = Business_DisplayPollKey(now_ms);
    if ((result != BUSINESS_SERVICE_OK) && (result != BUSINESS_SERVICE_IDLE)) { Business_DisplayReportResult(result, now_ms); }

    if (Business_TimeReached(now_ms, next_refresh_ms) != 0U) {
      result = Business_DisplayPrepareSnapshot(now_ms);
      if (result == BUSINESS_SERVICE_OK) {
        refresh_pending = 1U;
      } else {
        Business_DisplayReportResult(result, now_ms);
      }
      next_refresh_ms = now_ms + BUSINESS_DISPLAY_REFRESH_PERIOD_MS;
    }

    /* 切页等事件需要整页重绘时，不等 200ms 常规节拍，下一个 10ms tick 立即开始刷新。
       静态骨架用现有缓存值绘制，数据仍由常规节拍的快照更新，故无需在此重取快照。 */
    if (Business_DisplayNeedsImmediateRefresh() != 0U) {
      refresh_pending = 1U;
    }

    if (refresh_pending != 0U) {
      result = Business_DisplayRefreshStep(now_ms, BUSINESS_DISPLAY_REFRESH_BUDGET_US);
      if (result == BUSINESS_SERVICE_OK) {
        refresh_pending = 0U;
        Business_DisplayReportResult(result, now_ms);
      } else if (result != BUSINESS_SERVICE_BUSY) {
        refresh_pending = 0U;
        Business_DisplayReportResult(result, now_ms);
      }
    }

    Business_StatusHeartbeat(BUSINESS_COMPONENT_DISPLAY);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BUSINESS_DISPLAY_SERVICE_PERIOD_MS));
  }
#else
  (void)argument;
  vTaskDelete(0);
#endif
}
