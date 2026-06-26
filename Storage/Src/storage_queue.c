/**
 * @file storage_queue.c
 * @brief Implement a bounded non-blocking Storage record queue.
 */

#include "storage_queue.h"

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

static Storage_Record_t s_records[STORAGE_QUEUE_LENGTH];
static uint16_t s_head;
static uint16_t s_tail;
static uint16_t s_count;
static uint32_t s_drop_count;

void StorageQueue_Init(void)
{
  taskENTER_CRITICAL();
  memset(s_records, 0, sizeof(s_records));
  s_head       = 0U;
  s_tail       = 0U;
  s_count      = 0U;
  s_drop_count = 0U;
  taskEXIT_CRITICAL();
}

Px4Lite_Result_t Storage_RecordSetLine(Storage_Record_t *record, const char *line)
{
  size_t len;

  if ((record == 0) || (line == 0)) { return PX4LITE_INVALID_PARAM; }

  len = strlen(line);
  if (len >= STORAGE_CSV_LINE_MAX) { len = STORAGE_CSV_LINE_MAX - 1U; }
  memcpy(record->line, line, len);
  record->line[len] = '\0';
  return (line[len] == '\0') ? PX4LITE_OK : PX4LITE_OVERFLOW;
}

Px4Lite_Result_t StorageQueue_Push(const Storage_Record_t *record)
{
  if (record == 0) { return PX4LITE_INVALID_PARAM; }

  taskENTER_CRITICAL();
  if (s_count >= STORAGE_QUEUE_LENGTH) {
    s_drop_count++;
    taskEXIT_CRITICAL();
    return PX4LITE_OVERFLOW;
  }

  s_records[s_head] = *record;
  s_head            = (uint16_t)((s_head + 1U) % STORAGE_QUEUE_LENGTH);
  s_count++;
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}

Px4Lite_Result_t StorageQueue_Pop(Storage_Record_t *record)
{
  if (record == 0) { return PX4LITE_INVALID_PARAM; }

  taskENTER_CRITICAL();
  if (s_count == 0U) {
    taskEXIT_CRITICAL();
    return PX4LITE_NOT_READY;
  }

  *record = s_records[s_tail];
  s_tail  = (uint16_t)((s_tail + 1U) % STORAGE_QUEUE_LENGTH);
  s_count--;
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}

uint16_t StorageQueue_Count(void)
{
  uint16_t count;

  taskENTER_CRITICAL();
  count = s_count;
  taskEXIT_CRITICAL();
  return count;
}

uint32_t StorageQueue_DropCount(void)
{
  uint32_t drop_count;

  taskENTER_CRITICAL();
  drop_count = s_drop_count;
  taskEXIT_CRITICAL();
  return drop_count;
}
