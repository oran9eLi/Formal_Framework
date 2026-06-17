/**
 * @file business_task_template.h
 * @brief Declare business task services and platform hooks.
 */

#ifndef BUSINESS_TASK_TEMPLATE_H
#define BUSINESS_TASK_TEMPLATE_H

#include <stdint.h>

typedef enum
{
    BUSINESS_SERVICE_OK = 0,
    BUSINESS_SERVICE_BUSY,
    BUSINESS_SERVICE_IDLE,
    BUSINESS_SERVICE_NOT_READY,
    BUSINESS_SERVICE_ERROR
} Business_ServiceResult_t;

typedef enum
{
    BUSINESS_COMPONENT_SYSTEM = 0,
    BUSINESS_COMPONENT_ACQUISITION,
    BUSINESS_COMPONENT_DISPLAY,
    BUSINESS_COMPONENT_COUNT
} Business_ComponentId_t;

typedef struct
{
    uint32_t timestamp_ms;
    uint16_t source_id;
    uint8_t level;
    uint8_t reserved;
    const char *field;
    const char *value;
} Business_LogRecord_t;

/*
 * Platform/application hooks. Implement these in the real application layer;
 * sensor drivers and BSP code must not call these hooks directly.
 */
uint32_t Business_PlatformGetMs(void);
/**
 * @brief Run one non-blocking application registry service cycle.
 */
void Business_RegistryPoll(uint32_t now_ms);
/**
 * @brief Update the heartbeat and online status of a business module.
 */
void Business_StatusHeartbeat(Business_ComponentId_t component_id);
/**
 * @brief Convert a business log record into an event bus message.
 */
Business_ServiceResult_t Business_LogWrite(
    const Business_LogRecord_t *record);
/**
 * @brief Copy one new navigation snapshot and publish an acquisition event.
 */
Business_ServiceResult_t Business_AcquisitionRunOnce(uint32_t now_ms);

/*
 * Reserved display hooks. RefreshStep must return within budget_us. Return
 * BUSINESS_SERVICE_BUSY when more work remains for the next service period.
 */
Business_ServiceResult_t Business_DisplayPollTouch(uint32_t now_ms);
/**
 * @brief Placeholder hook for preparing one coherent display snapshot.
 */
Business_ServiceResult_t Business_DisplayPrepareSnapshot(uint32_t now_ms);
/**
 * @brief Placeholder hook for one budget-limited display refresh step.
 */
Business_ServiceResult_t Business_DisplayRefreshStep(uint32_t now_ms,
                                                     uint32_t budget_us);
/**
 * @brief Map a display service result to the framework module state.
 */
void Business_DisplayReportResult(Business_ServiceResult_t result,
                                  uint32_t now_ms);

/**
 * @brief Run periodic application registry service and system heartbeat updates.
 */
void Business_SystemTask(void *argument);
/**
 * @brief Distribute newly published navigation snapshots at a fixed period.
 */
void Business_AcquisitionTask(void *argument);
/**
 * @brief Poll input and advance the non-blocking display refresh state machine.
 */
void Business_DisplayServiceTask(void *argument);

#endif
