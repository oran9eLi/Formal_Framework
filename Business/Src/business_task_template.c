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
  uint8_t report_due = 0U;

  (void)argument;

  /* Let the independently powered LCD controller complete its cold-start ramp. */
  vTaskDelay(pdMS_TO_TICKS(BUSINESS_DISPLAY_STARTUP_DELAY_MS));

  for (;;) {
    uint32_t init_ms = Business_PlatformGetMs();

    if (Px4Lite_RegistryStart(PX4LITE_MODULE_DISPLAY, init_ms) == PX4LITE_OK) { break; }
    Business_StatusHeartbeat(BUSINESS_COMPONENT_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(BUSINESS_DISPLAY_INIT_RETRY_MS));
  }

  last_wake       = xTaskGetTickCount();
  next_refresh_ms = Business_PlatformGetMs();

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

    /* 数据快照仍按 200ms 常规节拍准备(重取 Framework 数据、更新 HMI 值)。 */
    if (Business_TimeReached(now_ms, next_refresh_ms) != 0U) {
      result = Business_DisplayPrepareSnapshot(now_ms);
      if (result != BUSINESS_SERVICE_OK) {
        Business_DisplayReportResult(result, now_ms);
      }
      next_refresh_ms = now_ms + BUSINESS_DISPLAY_REFRESH_PERIOD_MS;
      report_due      = 1U;
    }

    /* lv_timer_handler(触摸读取 + 脏区增量渲染)必须每个 10ms 节拍都推进：否则触摸只在
       200ms 数据节拍时被采样，滑块/按键严重不跟手(约 5Hz)。静止无脏区时开销极小，切页整页
       重绘与预算化多步渲染同样逐拍完成;刷新结果保持原 200ms 节拍上报，出错即时上报。 */
    result = Business_DisplayRefreshStep(now_ms, BUSINESS_DISPLAY_REFRESH_BUDGET_US);
    if (result != BUSINESS_SERVICE_BUSY) {
      if ((report_due != 0U) || (result != BUSINESS_SERVICE_OK)) {
        Business_DisplayReportResult(result, now_ms);
        report_due = 0U;
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
