/**
 * @file business_task_template.c
 * @brief Implement fixed-period system, acquisition, and display tasks.
 */

#include "business_task_template.h"

#include "business_template_config.h"
#include "FreeRTOS.h"
#include "px4lite_registry.h"
#include "task.h"

#define BUSINESS_LOG_LEVEL_INFO      1U

#if BUSINESS_ENABLE_DISPLAY
/**
 * @brief Perform a wrap-safe millisecond deadline comparison.
 */
static uint8_t Business_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (((int32_t)(now_ms - deadline_ms)) >= 0) ? 1U : 0U;
}
#endif

/**
 * @brief Run periodic application registry service and system heartbeat updates.
 */
void Business_SystemTask(void *argument)
{
    Business_LogRecord_t startup_record;
    TickType_t last_wake;

    (void)argument;
    startup_record.timestamp_ms = Business_PlatformGetMs();
    startup_record.source_id =
        (uint16_t)BUSINESS_COMPONENT_SYSTEM;
    startup_record.level = BUSINESS_LOG_LEVEL_INFO;
    startup_record.reserved = 0U;
    startup_record.field = "system.startup";
    startup_record.value = "ok";

    /* Startup is an edge event and is written exactly once. */
    (void)Business_LogWrite(&startup_record);
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        uint32_t now_ms = Business_PlatformGetMs();

        Business_RegistryPoll(now_ms);
        Business_StatusHeartbeat(BUSINESS_COMPONENT_SYSTEM);
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(BUSINESS_SYSTEM_PERIOD_MS));
    }
}

/**
 * @brief Distribute newly published navigation snapshots at a fixed period.
 */
void Business_AcquisitionTask(void *argument)
{
    TickType_t last_wake;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        uint32_t now_ms = Business_PlatformGetMs();

        (void)Business_AcquisitionRunOnce(now_ms);
        Business_StatusHeartbeat(
            BUSINESS_COMPONENT_ACQUISITION);
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(BUSINESS_ACQUISITION_PERIOD_MS));
    }
}

/**
 * @brief Poll input and advance the non-blocking display refresh state machine.
 */
void Business_DisplayServiceTask(void *argument)
{
#if BUSINESS_ENABLE_DISPLAY
    TickType_t last_wake;
    uint32_t next_refresh_ms;
    uint8_t refresh_pending = 0U;

    (void)argument;
    last_wake = xTaskGetTickCount();
    next_refresh_ms = Business_PlatformGetMs();
    (void)Px4Lite_RegistryStart(
        PX4LITE_MODULE_DISPLAY,
        next_refresh_ms);

    for (;;)
    {
        Business_ServiceResult_t result;
        uint32_t now_ms = Business_PlatformGetMs();

        /*
         * Input is always serviced first. Display refresh cannot suppress
         * touch polling merely because one refresh step failed.
         */
        result = Business_DisplayPollTouch(now_ms);
        if ((result != BUSINESS_SERVICE_OK) &&
            (result != BUSINESS_SERVICE_IDLE))
        {
            Business_DisplayReportResult(result, now_ms);
        }

        if (Business_TimeReached(now_ms, next_refresh_ms) != 0U)
        {
            result = Business_DisplayPrepareSnapshot(now_ms);
            if (result == BUSINESS_SERVICE_OK)
            {
                refresh_pending = 1U;
            }
            else
            {
                Business_DisplayReportResult(result, now_ms);
            }
            next_refresh_ms = now_ms +
                              BUSINESS_DISPLAY_REFRESH_PERIOD_MS;
        }

        if (refresh_pending != 0U)
        {
            result = Business_DisplayRefreshStep(
                now_ms,
                BUSINESS_DISPLAY_REFRESH_BUDGET_US);
            if (result == BUSINESS_SERVICE_OK)
            {
                refresh_pending = 0U;
                Business_DisplayReportResult(result, now_ms);
            }
            else if (result != BUSINESS_SERVICE_BUSY)
            {
                refresh_pending = 0U;
                Business_DisplayReportResult(result, now_ms);
            }
        }

        Business_StatusHeartbeat(BUSINESS_COMPONENT_DISPLAY);
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(
                            BUSINESS_DISPLAY_SERVICE_PERIOD_MS));
    }
#else
    (void)argument;
    vTaskDelete(0);
#endif
}
