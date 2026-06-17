/**
 * @file storage_queue.h
 * @brief Declare the fixed-size Storage record queue.
 */

#ifndef STORAGE_QUEUE_H
#define STORAGE_QUEUE_H

#include <stdint.h>
#include "px4lite_types.h"
#include "storage_config.h"

typedef enum
{
    STORAGE_RECORD_DATA = 0,
    STORAGE_RECORD_ERROR = 1
} Storage_RecordType_t;

typedef struct
{
    Storage_RecordType_t type;
    uint32_t enqueue_time_ms;
    char line[STORAGE_CSV_LINE_MAX];
} Storage_Record_t;

void StorageQueue_Init(void);
Px4Lite_Result_t StorageQueue_Push(const Storage_Record_t *record);
Px4Lite_Result_t StorageQueue_Pop(Storage_Record_t *record);
uint16_t StorageQueue_Count(void);
uint32_t StorageQueue_DropCount(void);
Px4Lite_Result_t Storage_RecordSetLine(Storage_Record_t *record,
                                       const char *line);

#endif
