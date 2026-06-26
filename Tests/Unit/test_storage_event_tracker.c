#include <stdint.h>
#include <stdio.h>

#include "storage_event.h"

static int ExpectUint32(const char *label, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("%s: expected %lu, got %lu\n", label, (unsigned long)expected, (unsigned long)actual);
    return 0;
  }
  return 1;
}

int main(void)
{
  Storage_EventAction_t action;
  uint32_t count = 0U;
  int ok = 1;

  StorageEventTracker_Init();

  action = StorageEventTracker_Update("BATTERY", 0x2301U, "low_voltage", 1U, &count);
  ok &= ExpectUint32("first active action", (uint32_t)action, (uint32_t)STORAGE_EVENT_ACTION_ACTIVE);
  ok &= ExpectUint32("first active count", count, 1U);

  action = StorageEventTracker_Update("BATTERY", 0x2301U, "low_voltage", 1U, &count);
  ok &= ExpectUint32("repeat active action", (uint32_t)action, (uint32_t)STORAGE_EVENT_ACTION_NONE);
  ok &= ExpectUint32("repeat active count", count, 2U);

  action = StorageEventTracker_Update("BATTERY", 0x2301U, "low_voltage", 0U, &count);
  ok &= ExpectUint32("clear action", (uint32_t)action, (uint32_t)STORAGE_EVENT_ACTION_CLEAR);
  ok &= ExpectUint32("clear count", count, 2U);

  action = StorageEventTracker_Update("BATTERY", 0x2301U, "low_voltage", 0U, &count);
  ok &= ExpectUint32("repeat clear action", (uint32_t)action, (uint32_t)STORAGE_EVENT_ACTION_NONE);

  if (ok != 0) {
    printf("storage event tracker tests passed\n");
    return 0;
  }

  return 1;
}
