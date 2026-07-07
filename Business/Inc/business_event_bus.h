/**
 * @file business_event_bus.h
 * @brief 声明有界、非阻塞的 Business 事件分发接口。
 *
 * @details
 * EventBus 为每个订阅者维护独立队列。发布者不会因某个订阅者队列满而阻塞，
 * 同一事件会尽力分发给所有已订阅消费者，并通过 report/statistics 暴露投递结果。
 */

#ifndef BUSINESS_EVENT_BUS_H
#define BUSINESS_EVENT_BUS_H

#include <stdint.h>
#include "FreeRTOS.h"

/**
 * @brief 单条事件负载最大长度，单位 byte。
 */
#define BUSINESS_EVENT_PAYLOAD_MAX 96U

/**
 * @brief Business 层事件主题。
 */
typedef enum {
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

/**
 * @brief Business 层事件订阅者。
 */
typedef enum {
  BUSINESS_SUBSCRIBER_ALARM = 0,
  BUSINESS_SUBSCRIBER_LOGGER,
  BUSINESS_SUBSCRIBER_STATE,
  BUSINESS_SUBSCRIBER_PROTOCOL,
  BUSINESS_SUBSCRIBER_COMM,
  BUSINESS_SUBSCRIBER_COUNT
} Business_Subscriber_t;

/**
 * @brief 事件负载类型。
 */
typedef enum {
  BUSINESS_PAYLOAD_NONE = 0,
  BUSINESS_PAYLOAD_BYTES,
  BUSINESS_PAYLOAD_INDEX
} Business_PayloadType_t;

/**
 * @brief 事件发布结果。
 */
typedef enum {
  BUSINESS_PUBLISH_OK = 0,
  BUSINESS_PUBLISH_PARTIAL,
  BUSINESS_PUBLISH_NO_SUBSCRIBER,
  BUSINESS_PUBLISH_INVALID,
  BUSINESS_PUBLISH_NOT_READY
} Business_PublishResult_t;

/**
 * @brief Business 事件记录。
 */
typedef struct {
  Business_Topic_t topic;                      /**< 事件主题。 */
  uint16_t source_id;                          /**< 事件来源组件或模块编号。 */
  Business_PayloadType_t payload_type;         /**< 负载编码类型。 */
  uint32_t timestamp_ms;                       /**< 事件产生时间，单位 ms。 */
  uint16_t instance_id;                        /**< 事件实例或设备编号。 */
  uint16_t data_index;                         /**< 负载索引或低 16 bit 序列号。 */
  uint16_t data_len;                           /**< payload 有效长度，单位 byte。 */
  uint32_t flags;                              /**< 事件标志位。 */
  uint8_t payload[BUSINESS_EVENT_PAYLOAD_MAX]; /**< 小块内联事件负载。 */
} Business_Event_t;

/**
 * @brief 单次事件发布报告。
 */
typedef struct {
  uint32_t subscribed_mask; /**< 本次发布命中的订阅者位图。 */
  uint32_t delivered_mask;  /**< 本次发布成功投递的订阅者位图。 */
  uint32_t failed_mask;     /**< 本次发布投递失败的订阅者位图。 */
  uint8_t delivered_count;  /**< 成功投递数量。 */
  uint8_t failed_count;     /**< 投递失败数量。 */
  uint16_t reserved;        /**< 对齐预留。 */
} Business_PublishReport_t;

/**
 * @brief 单个事件主题的发布统计。
 */
typedef struct {
  uint32_t attempt_count;       /**< 发布尝试次数。 */
  uint32_t complete_count;      /**< 全部订阅者投递成功次数。 */
  uint32_t partial_count;       /**< 部分订阅者投递失败次数。 */
  uint32_t no_subscriber_count; /**< 发布时无订阅者次数。 */
} Business_TopicStats_t;

/**
 * @brief 单个订阅者在单个主题上的投递统计。
 */
typedef struct {
  uint32_t delivered_count; /**< 成功投递次数。 */
  uint32_t drop_count;      /**< 因队列满或未就绪丢弃次数。 */
} Business_SubscriberStats_t;

/**
 * @brief 创建有界订阅者队列并复位 EventBus 状态。
 */
BaseType_t Business_EventBusInit(void);
/**
 * @brief 将一个消费者订阅到指定业务主题。
 */
BaseType_t Business_EventBusSubscribe(Business_Topic_t topic, Business_Subscriber_t subscriber);
/**
 * @brief 在任务上下文中非阻塞分发事件给所有订阅者。
 */
Business_PublishResult_t Business_EventBusPublish(const Business_Event_t *event, Business_PublishReport_t *report);
/**
 * @brief 在中断上下文中非阻塞分发事件给所有订阅者。
 */
Business_PublishResult_t Business_EventBusPublishFromISR(const Business_Event_t *event, Business_PublishReport_t *report, BaseType_t *higher_priority_task_woken);
/**
 * @brief 接收指定订阅者队列中的下一条事件。
 */
BaseType_t Business_EventBusReceive(Business_Subscriber_t subscriber, Business_Event_t *event, TickType_t timeout_ticks);
/**
 * @brief 复制指定业务主题的聚合发布统计。
 */
BaseType_t Business_EventBusGetTopicStats(Business_Topic_t topic, Business_TopicStats_t *stats);
/**
 * @brief 复制指定主题和订阅者组合的投递/丢弃统计。
 */
BaseType_t Business_EventBusGetSubscriberStats(Business_Topic_t topic, Business_Subscriber_t subscriber, Business_SubscriberStats_t *stats);

#endif
