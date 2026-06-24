/**
 * @file storage_time.c
 * @brief 缓存 Framework 本地时间，避免 FatFs diskio 反向依赖 Framework。
 */

#include "storage_time.h"

#define STORAGE_FALLBACK_DATE_YMD 20260101U
#define STORAGE_FALLBACK_TIME     0U

static uint32_t s_local_date_ymd;
static uint32_t s_local_time_hhmmss;
static uint8_t s_time_valid;

static uint8_t Storage_TimeDecodeDate(uint32_t ymd, uint16_t *year, uint8_t *month, uint8_t *day)
{
  uint32_t y = ymd / 10000U;
  uint32_t m = (ymd / 100U) % 100U;
  uint32_t d = ymd % 100U;

  if ((year == 0) || (month == 0) || (day == 0)) { return 0U; }
  if ((y < 2000U) || (y > 2099U) || (m < 1U) || (m > 12U) || (d < 1U) || (d > 31U)) { return 0U; }

  *year  = (uint16_t)y;
  *month = (uint8_t)m;
  *day   = (uint8_t)d;
  return 1U;
}

static uint8_t Storage_TimeDecodeTime(uint32_t hhmmss, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
  uint32_t h = hhmmss / 10000U;
  uint32_t m = (hhmmss / 100U) % 100U;
  uint32_t s = hhmmss % 100U;

  if ((hour == 0) || (minute == 0) || (second == 0)) { return 0U; }
  if ((h > 23U) || (m > 59U) || (s > 59U)) { return 0U; }

  *hour   = (uint8_t)h;
  *minute = (uint8_t)m;
  *second = (uint8_t)s;
  return 1U;
}

void Storage_TimeInit(void)
{
  s_local_date_ymd    = 0U;
  s_local_time_hhmmss = 0U;
  s_time_valid        = 0U;
}

void Storage_TimeUpdate(uint32_t local_date_ymd, uint32_t local_time_hhmmss, uint8_t valid)
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  if ((valid == 0U) ||
      (Storage_TimeDecodeDate(local_date_ymd, &year, &month, &day) == 0U) ||
      (Storage_TimeDecodeTime(local_time_hhmmss, &hour, &minute, &second) == 0U)) {
    s_time_valid = 0U;
    return;
  }

  s_local_date_ymd    = local_date_ymd;
  s_local_time_hhmmss = local_time_hhmmss;
  s_time_valid        = 1U;
}

uint8_t Storage_TimeIsValid(void)
{
  return s_time_valid;
}

uint32_t Storage_TimeDateYmd(void)
{
  return s_local_date_ymd;
}

uint32_t Storage_TimeTimeHhmmss(void)
{
  return s_local_time_hhmmss;
}

uint32_t Storage_TimeGetFatFsTime(void)
{
  uint32_t date = (s_time_valid != 0U) ? s_local_date_ymd : STORAGE_FALLBACK_DATE_YMD;
  uint32_t time = (s_time_valid != 0U) ? s_local_time_hhmmss : STORAGE_FALLBACK_TIME;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  (void)Storage_TimeDecodeDate(date, &year, &month, &day);
  (void)Storage_TimeDecodeTime(time, &hour, &minute, &second);

  return (((uint32_t)(year - 1980U)) << 25) |
         (((uint32_t)month) << 21) |
         (((uint32_t)day) << 16) |
         (((uint32_t)hour) << 11) |
         (((uint32_t)minute) << 5) |
         ((uint32_t)second / 2U);
}
