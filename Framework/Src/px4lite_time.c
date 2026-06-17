/**
 * @file px4lite_time.c
 * @brief 实现 RTC 持久时间和 GNSS 自动校时策略。
 */

#include "px4lite_time.h"

#include "px4lite_config.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static Px4Lite_TimeSnapshot_t s_time_snapshot;
static uint32_t s_time_sequence;
static uint32_t s_last_gnss_sync_ms;
static uint8_t s_time_ready;

/**
 * @brief 判断完整年份是否为闰年。
 */
static uint8_t Px4Lite_TimeIsLeapYear(uint16_t year)
{
    if ((year % 400U) == 0U)
    {
        return 1U;
    }
    if ((year % 100U) == 0U)
    {
        return 0U;
    }
    return ((year % 4U) == 0U) ? 1U : 0U;
}

/**
 * @brief 返回指定年月的天数。
 */
static uint8_t Px4Lite_TimeDaysInMonth(uint16_t year,
                                       uint8_t month)
{
    static const uint8_t days[12] =
        {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

    if ((month < 1U) || (month > 12U))
    {
        return 31U;
    }
    if (month == 2U)
    {
        return (Px4Lite_TimeIsLeapYear(year) != 0U) ? 29U : 28U;
    }
    return days[month - 1U];
}

/**
 * @brief 校验 UTC 日期时间字段范围。
 */
static uint8_t Px4Lite_TimeIsDateTimeValid(
    const Px4Lite_UtcDateTime_t *date_time)
{
    if (date_time == 0)
    {
        return 0U;
    }
    if ((date_time->year < 2000U) || (date_time->year > 2099U))
    {
        return 0U;
    }
    if ((date_time->month < 1U) || (date_time->month > 12U))
    {
        return 0U;
    }
    if ((date_time->day < 1U) ||
        (date_time->day >
         Px4Lite_TimeDaysInMonth(date_time->year, date_time->month)))
    {
        return 0U;
    }
    if ((date_time->hours > 23U) ||
        (date_time->minutes > 59U) ||
        (date_time->seconds > 59U))
    {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 计算从 2000-01-01 起经过的天数。
 */
static uint32_t Px4Lite_TimeDaysSince2000(
    const Px4Lite_UtcDateTime_t *date_time)
{
    uint32_t days = 0U;
    uint16_t year;
    uint8_t month;

    for (year = 2000U; year < date_time->year; year++)
    {
        days += (Px4Lite_TimeIsLeapYear(year) != 0U) ? 366U : 365U;
    }
    for (month = 1U; month < date_time->month; month++)
    {
        days += Px4Lite_TimeDaysInMonth(date_time->year, month);
    }
    days += (uint32_t)date_time->day - 1U;
    return days;
}

/**
 * @brief 将 UTC 日期时间转换为 2000-01-01 起的秒数。
 */
static uint32_t Px4Lite_TimeToEpochSeconds(
    const Px4Lite_UtcDateTime_t *date_time)
{
    uint32_t days;

    days = Px4Lite_TimeDaysSince2000(date_time);
    return (days * 86400UL) +
           ((uint32_t)date_time->hours * 3600UL) +
           ((uint32_t)date_time->minutes * 60UL) +
           date_time->seconds;
}

/**
 * @brief 计算两个 UTC 日期时间的绝对秒差。
 */
static uint32_t Px4Lite_TimeAbsDeltaSeconds(
    const Px4Lite_UtcDateTime_t *a,
    const Px4Lite_UtcDateTime_t *b)
{
    uint32_t ea = Px4Lite_TimeToEpochSeconds(a);
    uint32_t eb = Px4Lite_TimeToEpochSeconds(b);

    return (ea >= eb) ? (ea - eb) : (eb - ea);
}

/**
 * @brief 日期向后推进一天。
 */
static void Px4Lite_TimeIncrementDay(Px4Lite_UtcDateTime_t *date_time)
{
    date_time->day++;
    if (date_time->day >
        Px4Lite_TimeDaysInMonth(date_time->year, date_time->month))
    {
        date_time->day = 1U;
        date_time->month++;
        if (date_time->month > 12U)
        {
            date_time->month = 1U;
            if (date_time->year < 2099U)
            {
                date_time->year++;
            }
        }
    }
}

/**
 * @brief 给日期时间增加秒数，当前用于 UTC+8 本地显示转换。
 */
static void Px4Lite_TimeAddSeconds(Px4Lite_UtcDateTime_t *date_time,
                                   uint32_t add_seconds)
{
    uint32_t seconds;

    seconds = ((uint32_t)date_time->hours * 3600UL) +
              ((uint32_t)date_time->minutes * 60UL) +
              date_time->seconds +
              add_seconds;

    while (seconds >= 86400UL)
    {
        seconds -= 86400UL;
        Px4Lite_TimeIncrementDay(date_time);
    }

    date_time->hours = (uint8_t)(seconds / 3600UL);
    date_time->minutes = (uint8_t)((seconds / 60UL) % 60UL);
    date_time->seconds = (uint8_t)(seconds % 60UL);
}

/**
 * @brief 将日期编码为 YYYYMMDD。
 */
static uint32_t Px4Lite_TimeEncodeDateYmd(
    const Px4Lite_UtcDateTime_t *date_time)
{
    return ((uint32_t)date_time->year * 10000UL) +
           ((uint32_t)date_time->month * 100UL) +
           date_time->day;
}

/**
 * @brief 将时间编码为 HHMMSS。
 */
static uint32_t Px4Lite_TimeEncodeTimeHhmmss(
    const Px4Lite_UtcDateTime_t *date_time)
{
    return ((uint32_t)date_time->hours * 10000UL) +
           ((uint32_t)date_time->minutes * 100UL) +
           date_time->seconds;
}

/**
 * @brief 将 GNSS yymmdd 和当日秒数转换为 UTC 日期时间。
 */
static uint8_t Px4Lite_TimeFromGnss(
    uint32_t utc_date,
    uint32_t utc_sec,
    Px4Lite_UtcDateTime_t *out)
{
    uint32_t yy;
    uint32_t mm;
    uint32_t dd;

    if ((out == 0) || (utc_date == 0U) || (utc_sec >= 86400UL))
    {
        return 0U;
    }

    yy = utc_date / 10000UL;
    mm = (utc_date / 100UL) % 100UL;
    dd = utc_date % 100UL;

    out->year = (uint16_t)(2000U + yy);
    out->month = (uint8_t)mm;
    out->day = (uint8_t)dd;
    out->hours = (uint8_t)(utc_sec / 3600UL);
    out->minutes = (uint8_t)((utc_sec / 60UL) % 60UL);
    out->seconds = (uint8_t)(utc_sec % 60UL);

    return Px4Lite_TimeIsDateTimeValid(out);
}

/**
 * @brief 从 UTC 日期时间生成统一时间快照。
 */
static void Px4Lite_TimeFillSnapshot(
    Px4Lite_TimeSnapshot_t *snapshot,
    const Px4Lite_UtcDateTime_t *utc,
    Px4Lite_TimeSource_t source,
    Px4Lite_TimeSyncState_t sync_state,
    uint32_t now_ms)
{
    Px4Lite_UtcDateTime_t local;

    memset(snapshot, 0, sizeof(*snapshot));
    local = *utc;
    Px4Lite_TimeAddSeconds(&local, PX4LITE_TIME_LOCAL_OFFSET_S);

    snapshot->header.sample_time_ms = now_ms;
    snapshot->header.publish_time_ms = now_ms;
    snapshot->header.sequence = ++s_time_sequence;
    snapshot->header.device_id = 0x0102U;
    snapshot->header.valid = 1U;
    snapshot->header.flags = PX4LITE_DATA_VALID;
    snapshot->header.quality =
        (source == PX4LITE_TIME_SOURCE_GNSS) ? 100U : 70U;
    snapshot->utc_date_ymd = Px4Lite_TimeEncodeDateYmd(utc);
    snapshot->utc_time_hhmmss = Px4Lite_TimeEncodeTimeHhmmss(utc);
    snapshot->local_date_ymd = Px4Lite_TimeEncodeDateYmd(&local);
    snapshot->local_time_hhmmss = Px4Lite_TimeEncodeTimeHhmmss(&local);
    snapshot->last_sync_ms = s_last_gnss_sync_ms;
    snapshot->sync_age_s =
        (s_last_gnss_sync_ms != 0U)
            ? (Px4Lite_ElapsedMs(now_ms, s_last_gnss_sync_ms) / 1000UL)
            : 0U;
    snapshot->source = source;
    snapshot->sync_state = sync_state;
}

/**
 * @brief 发布新的统一时间快照。
 */
static void Px4Lite_TimePublish(
    const Px4Lite_UtcDateTime_t *utc,
    Px4Lite_TimeSource_t source,
    Px4Lite_TimeSyncState_t sync_state,
    uint32_t now_ms)
{
    Px4Lite_TimeSnapshot_t next;

    Px4Lite_TimeFillSnapshot(&next, utc, source, sync_state, now_ms);
    taskENTER_CRITICAL();
    s_time_snapshot = next;
    s_time_ready = 1U;
    taskEXIT_CRITICAL();
}

/**
 * @brief 判断当前 GNSS 快照是否可用于校准 RTC。
 */
static uint8_t Px4Lite_TimeCopyValidGnssTime(
    uint32_t now_ms,
    Px4Lite_UtcDateTime_t *out)
{
    Px4Lite_SensorGnss_t gnss;

    if (Px4Lite_CopyGnss(&gnss) != PX4LITE_OK)
    {
        return 0U;
    }
    if ((Px4Lite_IsFresh(&gnss.header,
                         now_ms,
                         PX4LITE_GNSS_MAX_AGE_MS) == 0U) ||
        (gnss.fix_type == 0U))
    {
        return 0U;
    }

    return Px4Lite_TimeFromGnss(gnss.utc_date, gnss.utc_sec, out);
}

Px4Lite_Result_t Px4Lite_TimeInit(void)
{
    Px4Lite_UtcDateTime_t rtc_time;

    memset(&s_time_snapshot, 0, sizeof(s_time_snapshot));
    s_time_sequence = 0U;
    s_last_gnss_sync_ms = 0U;
    s_time_ready = 0U;

    if ((Px4Lite_PlatformRtcRead(&rtc_time) == PX4LITE_OK) &&
        (Px4Lite_TimeIsDateTimeValid(&rtc_time) != 0U))
    {
        Px4Lite_TimeFillSnapshot(&s_time_snapshot,
                                 &rtc_time,
                                 PX4LITE_TIME_SOURCE_RTC,
                                 PX4LITE_TIME_SYNC_RTC_VALID,
                                 Px4Lite_PlatformGetMs());
        s_time_ready = 1U;
    }
    return PX4LITE_OK;
}

void Px4Lite_TimeRun(uint32_t now_ms)
{
    Px4Lite_UtcDateTime_t rtc_time;
    Px4Lite_UtcDateTime_t gnss_time;
    Px4Lite_TimeSyncState_t rtc_state;
    uint8_t rtc_valid;
    uint8_t gnss_valid;
    uint8_t should_try_sync;

    rtc_valid =
        ((Px4Lite_PlatformRtcRead(&rtc_time) == PX4LITE_OK) &&
         (Px4Lite_TimeIsDateTimeValid(&rtc_time) != 0U))
            ? 1U
            : 0U;
    gnss_valid = Px4Lite_TimeCopyValidGnssTime(now_ms, &gnss_time);

    should_try_sync =
        ((s_last_gnss_sync_ms == 0U) ||
         (Px4Lite_ElapsedMs(now_ms, s_last_gnss_sync_ms) >=
          PX4LITE_TIME_GNSS_SYNC_INTERVAL_MS))
            ? 1U
            : 0U;

    if ((gnss_valid != 0U) && (should_try_sync != 0U))
    {
        uint8_t need_write =
            ((rtc_valid == 0U) ||
             (Px4Lite_TimeAbsDeltaSeconds(&rtc_time, &gnss_time) >
              PX4LITE_TIME_SYNC_DELTA_S))
                ? 1U
                : 0U;

        if ((need_write == 0U) ||
            (Px4Lite_PlatformRtcWrite(&gnss_time) == PX4LITE_OK))
        {
            s_last_gnss_sync_ms = now_ms;
            Px4Lite_TimePublish(&gnss_time,
                                PX4LITE_TIME_SOURCE_GNSS,
                                PX4LITE_TIME_SYNC_GNSS_SYNCED,
                                now_ms);
            return;
        }
    }

    if (rtc_valid == 0U)
    {
        return;
    }

    rtc_state =
        ((s_last_gnss_sync_ms != 0U) &&
         (Px4Lite_ElapsedMs(now_ms, s_last_gnss_sync_ms) >
          PX4LITE_TIME_STALE_MS))
            ? PX4LITE_TIME_SYNC_STALE
            : PX4LITE_TIME_SYNC_RTC_VALID;
    Px4Lite_TimePublish(&rtc_time,
                        PX4LITE_TIME_SOURCE_RTC,
                        rtc_state,
                        now_ms);
}

Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out)
{
    uint8_t ready;

    if (out == 0)
    {
        return PX4LITE_INVALID_PARAM;
    }

    taskENTER_CRITICAL();
    ready = s_time_ready;
    if (ready != 0U)
    {
        *out = s_time_snapshot;
    }
    taskEXIT_CRITICAL();

    return (ready != 0U) ? PX4LITE_OK : PX4LITE_NOT_READY;
}
