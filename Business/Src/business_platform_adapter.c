/**
 * @file business_platform_adapter.c
 * @brief Bridge business hooks to framework topics, status, and time services.
 */

#include "business_task_template.h"

#include <string.h>
#include "app_data_api.h"
#include "business_event_bus.h"
#include "debug_console.h"
#include "px4lite_modules.h"
#include "px4lite_faults.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"

/**
 * @brief Return the platform monotonic time in milliseconds to business code.
 */
uint32_t Business_PlatformGetMs(void)
{
    return Px4Lite_PlatformGetMs();
}

/**
 * @brief Run one non-blocking application registry service cycle.
 */
void Business_RegistryPoll(uint32_t now_ms)
{
    (void)now_ms;
}

/**
 * @brief Update the heartbeat and online status of a business module.
 */
void Business_StatusHeartbeat(Business_ComponentId_t component_id)
{
    uint32_t now_ms = Px4Lite_PlatformGetMs();

    switch (component_id)
    {
        case BUSINESS_COMPONENT_SYSTEM:
            Px4Lite_PlatformHeartbeat(
                PX4LITE_HEARTBEAT_SYSTEM, now_ms);
            Px4Lite_SetExternalModuleState(
                PX4LITE_MODULE_SYSTEM,
                PX4LITE_STATE_ONLINE,
                PX4LITE_FAULT_NONE,
                now_ms);
            break;

        case BUSINESS_COMPONENT_ACQUISITION:
            Px4Lite_PlatformHeartbeat(
                PX4LITE_HEARTBEAT_BUSINESS, now_ms);
            Px4Lite_SetExternalModuleState(
                PX4LITE_MODULE_BUSINESS,
                PX4LITE_STATE_ONLINE,
                PX4LITE_FAULT_NONE,
                now_ms);
            break;

        case BUSINESS_COMPONENT_DISPLAY:
            Px4Lite_PlatformHeartbeat(
                PX4LITE_HEARTBEAT_DISPLAY, now_ms);
            break;

        default:
            break;
    }
}

/**
 * @brief Convert a business log record into an event bus message.
 */
Business_ServiceResult_t Business_LogWrite(
    const Business_LogRecord_t *record)
{
    Business_Event_t event;
    Business_PublishResult_t result;
    uint16_t offset = 0U;
    uint16_t length;

    if ((record == 0) || (record->field == 0) || (record->value == 0))
    {
        return BUSINESS_SERVICE_ERROR;
    }

    memset(&event, 0, sizeof(event));
    event.topic = BUSINESS_TOPIC_LOG_RECORD;
    event.source_id = record->source_id;
    event.payload_type = BUSINESS_PAYLOAD_BYTES;
    event.timestamp_ms = record->timestamp_ms;

    length = (uint16_t)strlen(record->field);
    if (length > (BUSINESS_EVENT_PAYLOAD_MAX - 2U))
    {
        length = BUSINESS_EVENT_PAYLOAD_MAX - 2U;
    }
    memcpy(&event.payload[offset], record->field, length);
    offset = (uint16_t)(offset + length);
    event.payload[offset++] = '=';

    length = (uint16_t)strlen(record->value);
    if (length > (uint16_t)(BUSINESS_EVENT_PAYLOAD_MAX - offset))
    {
        length = (uint16_t)(BUSINESS_EVENT_PAYLOAD_MAX - offset);
    }
    memcpy(&event.payload[offset], record->value, length);
    offset = (uint16_t)(offset + length);
    event.data_len = offset;

    result = Business_EventBusPublish(&event, 0);
    DBG_BUSINESS_PRINT("BUSINESS LOG: %s=%s",
                       record->field,
                       record->value);

    return ((result == BUSINESS_PUBLISH_OK) ||
            (result == BUSINESS_PUBLISH_NO_SUBSCRIBER))
               ? BUSINESS_SERVICE_OK
               : BUSINESS_SERVICE_ERROR;
}

/**
 * @brief Copy one new navigation snapshot and publish an acquisition event.
 */
Business_ServiceResult_t Business_AcquisitionRunOnce(uint32_t now_ms)
{
    static uint32_t last_navigation_sequence;
    App_NavigationSnapshot_t navigation;
    Business_Event_t event;
    Business_PublishResult_t result;

    if (App_CopyNavigation(&navigation, now_ms) != PX4LITE_OK)
    {
        return BUSINESS_SERVICE_IDLE;
    }

    if (navigation.header.sequence == last_navigation_sequence)
    {
        return BUSINESS_SERVICE_IDLE;
    }

    if (sizeof(navigation) > BUSINESS_EVENT_PAYLOAD_MAX)
    {
        return BUSINESS_SERVICE_ERROR;
    }

    memset(&event, 0, sizeof(event));
    event.topic = BUSINESS_TOPIC_ACQUISITION_DONE;
    event.source_id =
        (uint16_t)BUSINESS_COMPONENT_ACQUISITION;
    event.payload_type = BUSINESS_PAYLOAD_BYTES;
    event.timestamp_ms = now_ms;
    event.instance_id = navigation.header.device_id;
    event.data_index = (uint16_t)(navigation.header.sequence & 0xFFFFU);
    event.data_len = (uint16_t)sizeof(navigation);
    memcpy(event.payload, &navigation, sizeof(navigation));

    result = Business_EventBusPublish(&event, 0);
    last_navigation_sequence = navigation.header.sequence;

    return ((result == BUSINESS_PUBLISH_OK) ||
            (result == BUSINESS_PUBLISH_NO_SUBSCRIBER))
               ? BUSINESS_SERVICE_OK
               : BUSINESS_SERVICE_ERROR;
}

/**
 * @brief Map a display service result to the framework module state.
 */
void Business_DisplayReportResult(Business_ServiceResult_t result,
                                  uint32_t now_ms)
{
    if ((result == BUSINESS_SERVICE_OK) ||
        (result == BUSINESS_SERVICE_IDLE))
    {
        Px4Lite_SetExternalModuleState(PX4LITE_MODULE_DISPLAY,
                                       PX4LITE_STATE_ONLINE,
                                       0U,
                                       now_ms);
    }
    else if (result == BUSINESS_SERVICE_NOT_READY)
    {
        Px4Lite_SetExternalModuleState(PX4LITE_MODULE_DISPLAY,
                                       PX4LITE_STATE_DEGRADED,
                                       PX4LITE_FAULT_DISPLAY_OFFLINE,
                                       now_ms);
    }
    else
    {
        Px4Lite_SetExternalModuleState(PX4LITE_MODULE_DISPLAY,
                                       PX4LITE_STATE_DEGRADED,
                                       PX4LITE_FAULT_DISPLAY_REFRESH,
                                       now_ms);
    }
}
