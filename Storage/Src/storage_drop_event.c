/**
 * @file storage_drop_event.c
 * @brief 检测 Storage 队列丢弃计数是否增加。
 */

#include "storage_drop_event.h"

static uint32_t s_last_drop_count;

void StorageDropEvent_Init(void)
{
  s_last_drop_count = 0U;
}

Px4Lite_Result_t StorageDropEvent_Update(uint32_t drop_count, uint32_t *event_count)
{
  if (event_count == 0) { return PX4LITE_INVALID_PARAM; }
  *event_count = 0U;

  if (drop_count < s_last_drop_count) {
    s_last_drop_count = drop_count;
    return PX4LITE_IDLE;
  }

  if (drop_count > s_last_drop_count) {
    s_last_drop_count = drop_count;
    *event_count = drop_count;
    return PX4LITE_OK;
  }

  return PX4LITE_IDLE;
}
