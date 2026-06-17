/**
 * @file business_event_bus.h
 * @brief Declare bounded non-blocking business event fan-out.
 */

#ifndef BUSINESS_EVENT_BUS_H
#define BUSINESS_EVENT_BUS_H

#include <stdint.h>
#include "FreeRTOS.h"

#define BUSINESS_EVENT_PAYLOAD_MAX 96U

typedef enum
{
    BUSINESS_TOPIC_EXCEPTION_REPORT = 0,
    BUSINESS_TOPIC_ALARM_STATE_CHANGED,
    BUSINESS_TOPIC_MODULE_STATUS_CHANGED,
    BUSINESS_TOPIC_ACQUISITION_DONE,
    BUSINESS_TOPIC_BUFFER_READY,
    BUSINESS_TOPIC_COMMAND_RECEIVED,
    BUSINESS_TOPIC_COMMAND_RESULT,
    BUSINESS_TOPIC_LOG_RECORD,
    BUSINESS_TOPIC_COMM_TX_FRAME,
    BUSINESS_TOPIC_COUNT
} Business_Topic_t;

typedef enum
{
    BUSINESS_SUBSCRIBER_ALARM = 0,
    BUSINESS_SUBSCRIBER_LOGGER,
    BUSINESS_SUBSCRIBER_STATE,
    BUSINESS_SUBSCRIBER_PROTOCOL,
    BUSINESS_SUBSCRIBER_COMM,
    BUSINESS_SUBSCRIBER_COUNT
} Business_Subscriber_t;

typedef enum
{
    BUSINESS_PAYLOAD_NONE = 0,
    BUSINESS_PAYLOAD_BYTES,
    BUSINESS_PAYLOAD_INDEX
} Business_PayloadType_t;

typedef enum
{
    BUSINESS_PUBLISH_OK = 0,
    BUSINESS_PUBLISH_PARTIAL,
    BUSINESS_PUBLISH_NO_SUBSCRIBER,
    BUSINESS_PUBLISH_INVALID,
    BUSINESS_PUBLISH_NOT_READY
} Business_PublishResult_t;

typedef struct
{
    Business_Topic_t topic;
    uint16_t source_id;
    Business_PayloadType_t payload_type;
    uint32_t timestamp_ms;
    uint16_t instance_id;
    uint16_t data_index;
    uint16_t data_len;
    uint32_t flags;
    uint8_t payload[BUSINESS_EVENT_PAYLOAD_MAX];
} Business_Event_t;

typedef struct
{
    uint32_t subscribed_mask;
    uint32_t delivered_mask;
    uint32_t failed_mask;
    uint8_t delivered_count;
    uint8_t failed_count;
    uint16_t reserved;
} Business_PublishReport_t;

typedef struct
{
    uint32_t attempt_count;
    uint32_t complete_count;
    uint32_t partial_count;
    uint32_t no_subscriber_count;
} Business_TopicStats_t;

typedef struct
{
    uint32_t delivered_count;
    uint32_t drop_count;
} Business_SubscriberStats_t;

/**
 * @brief Create bounded queues and reset business event bus state.
 */
BaseType_t Business_EventBusInit(void);
/**
 * @brief Subscribe one consumer queue to a business topic.
 */
BaseType_t Business_EventBusSubscribe(Business_Topic_t topic,
                                      Business_Subscriber_t subscriber);
/**
 * @brief Fan out an event to all subscribed task-context consumers without blocking.
 */
Business_PublishResult_t Business_EventBusPublish(
    const Business_Event_t *event,
    Business_PublishReport_t *report);
/**
 * @brief Fan out an event from interrupt context without blocking.
 */
Business_PublishResult_t Business_EventBusPublishFromISR(
    const Business_Event_t *event,
    Business_PublishReport_t *report,
    BaseType_t *higher_priority_task_woken);
/**
 * @brief Receive the next event owned by one business subscriber.
 */
BaseType_t Business_EventBusReceive(Business_Subscriber_t subscriber,
                                    Business_Event_t *event,
                                    TickType_t timeout_ticks);
/**
 * @brief Copy aggregate publication statistics for one business topic.
 */
BaseType_t Business_EventBusGetTopicStats(Business_Topic_t topic,
                                          Business_TopicStats_t *stats);
/**
 * @brief Copy delivery and drop statistics for one topic subscriber pair.
 */
BaseType_t Business_EventBusGetSubscriberStats(
    Business_Topic_t topic,
    Business_Subscriber_t subscriber,
    Business_SubscriberStats_t *stats);

#endif
