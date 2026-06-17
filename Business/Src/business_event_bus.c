/**
 * @file business_event_bus.c
 * @brief 实现订阅者隔离的 Business 事件分发和统计。
 */

#include "business_event_bus.h"

#include <string.h>
#include "business_template_config.h"
#include "queue.h"
#include "task.h"

static QueueHandle_t s_queues[BUSINESS_SUBSCRIBER_COUNT];
static uint8_t s_subscriptions[BUSINESS_TOPIC_COUNT]
                              [BUSINESS_SUBSCRIBER_COUNT];
static Business_TopicStats_t s_topic_stats[BUSINESS_TOPIC_COUNT];
static Business_SubscriberStats_t
    s_subscriber_stats[BUSINESS_TOPIC_COUNT][BUSINESS_SUBSCRIBER_COUNT];
static uint8_t s_initialized;

/**
 * @brief 检查业务主题编号是否在有效范围内。
 */
static uint8_t Business_IsValidTopic(Business_Topic_t topic)
{
    return ((uint32_t)topic < (uint32_t)BUSINESS_TOPIC_COUNT) ? 1U : 0U;
}

/**
 * @brief 检查业务订阅者编号是否在有效范围内。
 */
static uint8_t Business_IsValidSubscriber(Business_Subscriber_t subscriber)
{
    return ((uint32_t)subscriber <
            (uint32_t)BUSINESS_SUBSCRIBER_COUNT) ? 1U : 0U;
}

/**
 * @brief 校验业务事件头、负载类型和负载长度。
 */
static uint8_t Business_IsValidEvent(const Business_Event_t *event)
{
    if ((event == 0) ||
        (Business_IsValidTopic(event->topic) == 0U) ||
        (event->data_len > BUSINESS_EVENT_PAYLOAD_MAX))
    {
        return 0U;
    }

    if ((event->payload_type == BUSINESS_PAYLOAD_NONE) &&
        (event->data_len != 0U))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 清空可选的事件发布报告。
 */
static void Business_ClearReport(Business_PublishReport_t *report)
{
    if (report != 0)
    {
        (void)memset(report, 0, sizeof(*report));
    }
}

/**
 * @brief 在发布报告中标记一个订阅者被本次发布命中。
 */
static void Business_ReportSubscribed(Business_PublishReport_t *report,
                                      uint32_t subscriber)
{
    if (report != 0)
    {
        report->subscribed_mask |= (1UL << subscriber);
    }
}

/**
 * @brief 在发布报告中记录一个订阅者投递成功。
 */
static void Business_ReportDelivered(Business_PublishReport_t *report,
                                     uint32_t subscriber)
{
    if (report != 0)
    {
        report->delivered_mask |= (1UL << subscriber);
        report->delivered_count++;
    }
}

/**
 * @brief 在发布报告中记录一个订阅者投递失败。
 */
static void Business_ReportFailed(Business_PublishReport_t *report,
                                  uint32_t subscriber)
{
    if (report != 0)
    {
        report->failed_mask |= (1UL << subscriber);
        report->failed_count++;
    }
}

/**
 * @brief 汇总任务上下文发布统计并返回发布结果。
 */
static Business_PublishResult_t Business_FinishPublish(
    Business_Topic_t topic,
    uint32_t subscriber_count,
    uint32_t failed_count)
{
    Business_PublishResult_t result;

    taskENTER_CRITICAL();
    s_topic_stats[topic].attempt_count++;
    if (subscriber_count == 0U)
    {
        s_topic_stats[topic].no_subscriber_count++;
        result = BUSINESS_PUBLISH_NO_SUBSCRIBER;
    }
    else if (failed_count != 0U)
    {
        s_topic_stats[topic].partial_count++;
        result = BUSINESS_PUBLISH_PARTIAL;
    }
    else
    {
        s_topic_stats[topic].complete_count++;
        result = BUSINESS_PUBLISH_OK;
    }
    taskEXIT_CRITICAL();

    return result;
}

/**
 * @brief 汇总中断上下文发布统计并返回发布结果。
 */
static Business_PublishResult_t Business_FinishPublishFromISR(
    Business_Topic_t topic,
    uint32_t subscriber_count,
    uint32_t failed_count)
{
    Business_PublishResult_t result;
    UBaseType_t interrupt_mask;

    interrupt_mask = portSET_INTERRUPT_MASK_FROM_ISR();
    s_topic_stats[topic].attempt_count++;
    if (subscriber_count == 0U)
    {
        s_topic_stats[topic].no_subscriber_count++;
        result = BUSINESS_PUBLISH_NO_SUBSCRIBER;
    }
    else if (failed_count != 0U)
    {
        s_topic_stats[topic].partial_count++;
        result = BUSINESS_PUBLISH_PARTIAL;
    }
    else
    {
        s_topic_stats[topic].complete_count++;
        result = BUSINESS_PUBLISH_OK;
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(interrupt_mask);

    return result;
}

/**
 * @brief 创建有界订阅者队列并复位 EventBus 状态。
 */
BaseType_t Business_EventBusInit(void)
{
    uint32_t i;

    if (s_initialized != 0U)
    {
        return pdPASS;
    }

    (void)memset(s_queues, 0, sizeof(s_queues));
    (void)memset(s_subscriptions, 0, sizeof(s_subscriptions));
    (void)memset(s_topic_stats, 0, sizeof(s_topic_stats));
    (void)memset(s_subscriber_stats, 0, sizeof(s_subscriber_stats));

    for (i = 0U; i < (uint32_t)BUSINESS_SUBSCRIBER_COUNT; i++)
    {
        s_queues[i] = xQueueCreate(BUSINESS_EVENT_QUEUE_LENGTH,
                                   sizeof(Business_Event_t));
        if (s_queues[i] == 0)
        {
            while (i > 0U)
            {
                i--;
                vQueueDelete(s_queues[i]);
                s_queues[i] = 0;
            }
            return pdFAIL;
        }
    }

    s_initialized = 1U;
    return pdPASS;
}

/**
 * @brief 将一个消费者订阅到指定业务主题。
 */
BaseType_t Business_EventBusSubscribe(Business_Topic_t topic,
                                      Business_Subscriber_t subscriber)
{
    if ((Business_IsValidTopic(topic) == 0U) ||
        (Business_IsValidSubscriber(subscriber) == 0U) ||
        (s_initialized == 0U) ||
        (s_queues[subscriber] == 0))
    {
        return pdFAIL;
    }

    taskENTER_CRITICAL();
    s_subscriptions[topic][subscriber] = 1U;
    taskEXIT_CRITICAL();
    return pdPASS;
}

/**
 * @brief 在任务上下文中非阻塞分发事件给所有订阅者。
 */
Business_PublishResult_t Business_EventBusPublish(
    const Business_Event_t *event,
    Business_PublishReport_t *report)
{
    uint32_t i;
    uint32_t subscriber_count = 0U;
    uint32_t failed_count = 0U;

    Business_ClearReport(report);
    if (s_initialized == 0U)
    {
        return BUSINESS_PUBLISH_NOT_READY;
    }
    if (Business_IsValidEvent(event) == 0U)
    {
        return BUSINESS_PUBLISH_INVALID;
    }

    for (i = 0U; i < (uint32_t)BUSINESS_SUBSCRIBER_COUNT; i++)
    {
        if (s_subscriptions[event->topic][i] == 0U)
        {
            continue;
        }

        subscriber_count++;
        Business_ReportSubscribed(report, i);

        /* 每个订阅者只尝试一次；某个队列满不会阻塞后续订阅者。 */
        if ((s_queues[i] != 0) &&
            (xQueueSend(s_queues[i], event, 0U) == pdPASS))
        {
            taskENTER_CRITICAL();
            s_subscriber_stats[event->topic][i].delivered_count++;
            taskEXIT_CRITICAL();
            Business_ReportDelivered(report, i);
        }
        else
        {
            taskENTER_CRITICAL();
            s_subscriber_stats[event->topic][i].drop_count++;
            taskEXIT_CRITICAL();
            failed_count++;
            Business_ReportFailed(report, i);
        }
    }

    return Business_FinishPublish(event->topic,
                                  subscriber_count,
                                  failed_count);
}

/**
 * @brief 在中断上下文中非阻塞分发事件给所有订阅者。
 */
Business_PublishResult_t Business_EventBusPublishFromISR(
    const Business_Event_t *event,
    Business_PublishReport_t *report,
    BaseType_t *higher_priority_task_woken)
{
    BaseType_t any_task_woken = pdFALSE;
    uint32_t i;
    uint32_t subscriber_count = 0U;
    uint32_t failed_count = 0U;

    Business_ClearReport(report);
    if (s_initialized == 0U)
    {
        return BUSINESS_PUBLISH_NOT_READY;
    }
    if (Business_IsValidEvent(event) == 0U)
    {
        return BUSINESS_PUBLISH_INVALID;
    }

    for (i = 0U; i < (uint32_t)BUSINESS_SUBSCRIBER_COUNT; i++)
    {
        BaseType_t task_woken = pdFALSE;
        UBaseType_t interrupt_mask;

        if (s_subscriptions[event->topic][i] == 0U)
        {
            continue;
        }

        subscriber_count++;
        Business_ReportSubscribed(report, i);

        if ((s_queues[i] != 0) &&
            (xQueueSendFromISR(s_queues[i], event, &task_woken) == pdPASS))
        {
            interrupt_mask = portSET_INTERRUPT_MASK_FROM_ISR();
            s_subscriber_stats[event->topic][i].delivered_count++;
            portCLEAR_INTERRUPT_MASK_FROM_ISR(interrupt_mask);
            Business_ReportDelivered(report, i);
            if (task_woken != pdFALSE)
            {
                any_task_woken = pdTRUE;
            }
        }
        else
        {
            interrupt_mask = portSET_INTERRUPT_MASK_FROM_ISR();
            s_subscriber_stats[event->topic][i].drop_count++;
            portCLEAR_INTERRUPT_MASK_FROM_ISR(interrupt_mask);
            failed_count++;
            Business_ReportFailed(report, i);
        }
    }

    if ((higher_priority_task_woken != 0) &&
        (any_task_woken != pdFALSE))
    {
        *higher_priority_task_woken = pdTRUE;
    }

    return Business_FinishPublishFromISR(event->topic,
                                         subscriber_count,
                                         failed_count);
}

/**
 * @brief 接收指定订阅者队列中的下一条事件。
 */
BaseType_t Business_EventBusReceive(Business_Subscriber_t subscriber,
                                    Business_Event_t *event,
                                    TickType_t timeout_ticks)
{
    if ((Business_IsValidSubscriber(subscriber) == 0U) ||
        (event == 0) ||
        (s_queues[subscriber] == 0))
    {
        return pdFAIL;
    }

    return xQueueReceive(s_queues[subscriber], event, timeout_ticks);
}

/**
 * @brief 复制指定业务主题的聚合发布统计。
 */
BaseType_t Business_EventBusGetTopicStats(Business_Topic_t topic,
                                          Business_TopicStats_t *stats)
{
    if ((Business_IsValidTopic(topic) == 0U) || (stats == 0))
    {
        return pdFAIL;
    }

    taskENTER_CRITICAL();
    *stats = s_topic_stats[topic];
    taskEXIT_CRITICAL();
    return pdPASS;
}

/**
 * @brief 复制指定主题和订阅者组合的投递/丢弃统计。
 */
BaseType_t Business_EventBusGetSubscriberStats(
    Business_Topic_t topic,
    Business_Subscriber_t subscriber,
    Business_SubscriberStats_t *stats)
{
    if ((Business_IsValidTopic(topic) == 0U) ||
        (Business_IsValidSubscriber(subscriber) == 0U) ||
        (stats == 0))
    {
        return pdFAIL;
    }

    taskENTER_CRITICAL();
    *stats = s_subscriber_stats[topic][subscriber];
    taskEXIT_CRITICAL();
    return pdPASS;
}
