#include <stdint.h>
#include <stdio.h>

#include "storage_drop_event.h"

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
  uint32_t event_count = 0U;
  int ok = 1;

  StorageDropEvent_Init();

  ok &= ExpectUint32("zero has no event", (uint32_t)StorageDropEvent_Update(0U, &event_count), (uint32_t)PX4LITE_IDLE);

  ok &= ExpectUint32("first drop emits event", (uint32_t)StorageDropEvent_Update(1U, &event_count), (uint32_t)PX4LITE_OK);
  ok &= ExpectUint32("first drop count", event_count, 1U);

  ok &= ExpectUint32("same drop count has no event", (uint32_t)StorageDropEvent_Update(1U, &event_count), (uint32_t)PX4LITE_IDLE);

  ok &= ExpectUint32("larger drop count emits event", (uint32_t)StorageDropEvent_Update(3U, &event_count), (uint32_t)PX4LITE_OK);
  ok &= ExpectUint32("larger drop count", event_count, 3U);

  ok &= ExpectUint32("counter reset has no event", (uint32_t)StorageDropEvent_Update(0U, &event_count), (uint32_t)PX4LITE_IDLE);
  ok &= ExpectUint32("drop after reset emits event", (uint32_t)StorageDropEvent_Update(1U, &event_count), (uint32_t)PX4LITE_OK);
  ok &= ExpectUint32("drop after reset count", event_count, 1U);

  if (ok != 0) {
    printf("storage drop event tests passed\n");
    return 0;
  }

  return 1;
}
