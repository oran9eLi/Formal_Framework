/**
 * @file storage_event.h
 * @brief Track persistent event state so EVENT logs only record changes.
 */

#ifndef STORAGE_EVENT_H
#define STORAGE_EVENT_H

#include <stdint.h>

typedef enum {
  STORAGE_EVENT_ACTION_NONE = 0,
  STORAGE_EVENT_ACTION_ACTIVE,
  STORAGE_EVENT_ACTION_CLEAR
} Storage_EventAction_t;

void StorageEventTracker_Init(void);
Storage_EventAction_t StorageEventTracker_Update(const char *source, uint32_t fault, const char *message, uint8_t active, uint32_t *count_out);

#endif
