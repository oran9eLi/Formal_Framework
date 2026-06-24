#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "storage_file.h"
#include "storage_queue.h"
#include "storage_time.h"

static int ExpectUint32(const char *label, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("%s: expected %lu, got %lu\n", label, (unsigned long)expected, (unsigned long)actual);
    return 0;
  }
  return 1;
}

static int ExpectString(const char *label, const char *actual, const char *expected)
{
  if (strcmp(actual, expected) != 0) {
    printf("%s: expected %s, got %s\n", label, expected, actual);
    return 0;
  }
  return 1;
}

static int TestFatFsTimeUsesRtcCache(void)
{
  uint32_t expected = ((uint32_t)(2026U - 1980U) << 25) |
                      ((uint32_t)6U << 21) |
                      ((uint32_t)22U << 16) |
                      ((uint32_t)15U << 11) |
                      ((uint32_t)30U << 5) |
                      ((uint32_t)(45U / 2U));

  Storage_TimeInit();
  Storage_TimeUpdate(20260622U, 153045U, 1U);
  return ExpectUint32("fatfs timestamp", Storage_TimeGetFatFsTime(), expected);
}

static int TestFatFsTimeFallsBackWhenInvalid(void)
{
  uint32_t expected = ((uint32_t)(2026U - 1980U) << 25) |
                      ((uint32_t)1U << 21) |
                      ((uint32_t)1U << 16);

  Storage_TimeInit();
  Storage_TimeUpdate(0U, 0U, 0U);
  return ExpectUint32("fatfs fallback timestamp", Storage_TimeGetFatFsTime(), expected);
}

static int TestLogFileNamesUseEightDotThreeFormat(void)
{
  char path[24];
  int ok = 1;

  ok &= (StorageFile_FormatPath(STORAGE_RECORD_DATA, 20260622U, path, sizeof(path)) == PX4LITE_OK);
  ok &= ExpectString("data path", path, "0:/260622_D.CSV");
  ok &= (StorageFile_FormatPath(STORAGE_RECORD_EVENT, 20260622U, path, sizeof(path)) == PX4LITE_OK);
  ok &= ExpectString("event path", path, "0:/260622_E.CSV");
  ok &= (StorageFile_FormatPath(STORAGE_RECORD_DATA, 0U, path, sizeof(path)) == PX4LITE_OK);
  ok &= ExpectString("unsynced data path", path, "0:/UNSYNC_D.CSV");
  ok &= (StorageFile_FormatPath(STORAGE_RECORD_EVENT, 0U, path, sizeof(path)) == PX4LITE_OK);
  ok &= ExpectString("unsynced event path", path, "0:/UNSYNC_E.CSV");
  return ok;
}

static int TestQueuePreservesRecordDate(void)
{
  Storage_Record_t in;
  Storage_Record_t out;
  int ok = 1;

  StorageQueue_Init();
  memset(&in, 0, sizeof(in));
  in.type                = STORAGE_RECORD_EVENT;
  in.enqueue_time_ms     = 100U;
  in.target_date_ymd     = 20260622U;
  in.target_time_hhmmss  = 153045U;
  (void)Storage_RecordSetLine(&in, "event line\r\n");

  ok &= (StorageQueue_Push(&in) == PX4LITE_OK);
  ok &= (StorageQueue_Pop(&out) == PX4LITE_OK);
  ok &= ExpectUint32("record date", out.target_date_ymd, 20260622U);
  ok &= ExpectUint32("record time", out.target_time_hhmmss, 153045U);
  ok &= ExpectUint32("record type", (uint32_t)out.type, (uint32_t)STORAGE_RECORD_EVENT);
  ok &= ExpectString("record line", out.line, "event line\r\n");
  return ok;
}

int main(void)
{
  int ok = 1;

  ok &= TestFatFsTimeUsesRtcCache();
  ok &= TestFatFsTimeFallsBackWhenInvalid();
  ok &= TestLogFileNamesUseEightDotThreeFormat();
  ok &= TestQueuePreservesRecordDate();

  if (ok != 0) {
    printf("storage time and file tests passed\n");
    return 0;
  }

  return 1;
}
