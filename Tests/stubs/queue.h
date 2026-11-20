/** @file queue.h
 * @brief 主机采样 topic 测试只使用静态快照；编译同一实现时声明未调用的队列入口。
 */
#ifndef HOST_QUEUE_H
#define HOST_QUEUE_H
#include <stddef.h>
#include <stdint.h>
typedef void *QueueHandle_t;
#define pdPASS 1
QueueHandle_t xQueueCreate(size_t count, size_t item_size);
int xQueueSend(QueueHandle_t queue, const void *item, uint32_t ticks);
int xQueueReceive(QueueHandle_t queue, void *item, uint32_t ticks);
#endif
