/**
 * @file storage_drop_event.h
 * @brief 跟踪 Storage 队列丢弃计数变化。
 */

#ifndef STORAGE_DROP_EVENT_H
#define STORAGE_DROP_EVENT_H

#include <stdint.h>
#include "px4lite_types.h"

void StorageDropEvent_Init(void);
Px4Lite_Result_t StorageDropEvent_Update(uint32_t drop_count, uint32_t *event_count);

#endif
