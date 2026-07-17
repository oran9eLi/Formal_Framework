/**
 * @file storage_event.c
 * @brief Implement fixed-size event deduplication for Storage EVENT logs.
 */

#include "storage_event.h"

#include <string.h>

#define STORAGE_EVENT_TRACK_MAX 16U

typedef struct {
  const char *source;
  const char *message;
  uint32_t fault;
  uint32_t count;
  uint8_t used;
  uint8_t active;
} Storage_EventTrack_t;

static Storage_EventTrack_t s_tracks[STORAGE_EVENT_TRACK_MAX];

void StorageEventTracker_Init(void)
{
  memset(s_tracks, 0, sizeof(s_tracks));
}

static uint8_t StorageEventTracker_KeyEquals(const Storage_EventTrack_t *track, const char *source, uint32_t fault, const char *message)
{
  if ((track == 0) || (track->used == 0U) || (source == 0) || (message == 0)) { return 0U; }
  return ((track->fault == fault) && (strcmp(track->source, source) == 0) && (strcmp(track->message, message) == 0)) ? 1U : 0U;
}

static Storage_EventTrack_t *StorageEventTracker_FindOrAlloc(const char *source, uint32_t fault, const char *message)
{
  uint8_t i;
  Storage_EventTrack_t *free_track = 0;

  for (i = 0U; i < STORAGE_EVENT_TRACK_MAX; i++) {
    if (StorageEventTracker_KeyEquals(&s_tracks[i], source, fault, message) != 0U) { return &s_tracks[i]; }
    if ((free_track == 0) && (s_tracks[i].used == 0U)) { free_track = &s_tracks[i]; }
  }

  if (free_track != 0) {
    free_track->source  = source;
    free_track->message = message;
    free_track->fault   = fault;
    free_track->count   = 0U;
    free_track->active  = 0U;
    free_track->used    = 1U;
  }

  return free_track;
}

Storage_EventAction_t StorageEventTracker_Update(const char *source, uint32_t fault, const char *message, uint8_t active, uint32_t *count_out)
{
  Storage_EventTrack_t *track;

  if (count_out != 0) { *count_out = 0U; }
  if ((source == 0) || (message == 0)) { return STORAGE_EVENT_ACTION_NONE; }

  track = StorageEventTracker_FindOrAlloc(source, fault, message);
  if (track == 0) { return STORAGE_EVENT_ACTION_NONE; }

  if (active != 0U) {
    if (track->count < 0xFFFFFFFFUL) { track->count++; }
    if (count_out != 0) { *count_out = track->count; }
    if (track->active == 0U) {
      track->active = 1U;
      return STORAGE_EVENT_ACTION_ACTIVE;
    }
    return STORAGE_EVENT_ACTION_NONE;
  }

  if (count_out != 0) { *count_out = track->count; }
  if (track->active != 0U) {
    track->active = 0U;
    return STORAGE_EVENT_ACTION_CLEAR;
  }

  return STORAGE_EVENT_ACTION_NONE;
}
