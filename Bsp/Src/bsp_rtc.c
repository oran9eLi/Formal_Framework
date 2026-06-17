/**
 * @file bsp_rtc.c
 * @brief 实现 STM32F407 RTC 初始化、备份域标记和 UTC 日期时间读写。
 */

#include "bsp_rtc.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rtc.h"
#include "stm32f4xx_hal_rtc_ex.h"
#include <string.h>

#define BSP_RTC_MAGIC_VALUE            0x4657U
#define BSP_RTC_BACKUP_MAGIC_REGISTER  RTC_BKP_DR0
#define BSP_RTC_CLOCK_STARTUP_TIMEOUT_MS 5000U

static RTC_HandleTypeDef s_rtc;
static uint32_t s_rtc_clock_source;
static uint8_t s_rtc_ready;

/**
 * @brief 判断完整年份是否为闰年。
 */
static uint8_t BSP_RTC_IsLeapYear(uint16_t year)
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
static uint8_t BSP_RTC_DaysInMonth(uint16_t year, uint8_t month)
{
    static const uint8_t days[12] =
        {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

    if ((month < 1U) || (month > 12U))
    {
        return 31U;
    }
    if (month == 2U)
    {
        return (BSP_RTC_IsLeapYear(year) != 0U) ? 29U : 28U;
    }
    return days[month - 1U];
}

/**
 * @brief 校验 RTC 日期时间字段范围。
 */
static uint8_t BSP_RTC_IsDateTimeValid(
    const BSP_RTC_DateTime_t *date_time)
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
         BSP_RTC_DaysInMonth(date_time->year, date_time->month)))
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
static uint32_t BSP_RTC_DaysSince2000(uint16_t year,
                                      uint8_t month,
                                      uint8_t day)
{
    uint32_t days = 0U;
    uint16_t y;
    uint8_t m;

    for (y = 2000U; y < year; y++)
    {
        days += (BSP_RTC_IsLeapYear(y) != 0U) ? 366U : 365U;
    }
    for (m = 1U; m < month; m++)
    {
        days += BSP_RTC_DaysInMonth(year, m);
    }
    days += (uint32_t)day - 1U;
    return days;
}

/**
 * @brief 根据日期计算 RTC 星期值。
 */
static uint8_t BSP_RTC_ComputeWeekday(uint16_t year,
                                      uint8_t month,
                                      uint8_t day)
{
    uint32_t days;

    days = BSP_RTC_DaysSince2000(year, month, day);
    return (uint8_t)(((days + 5U) % 7U) + 1U);
}

/**
 * @brief 等待 RCC 标志位达到期望状态。
 */
static BSP_Status_t BSP_RTC_WaitClockFlag(uint32_t flag,
                                          FlagStatus expected)
{
    uint32_t start_ms = HAL_GetTick();

    while (__HAL_RCC_GET_FLAG(flag) != expected)
    {
        if ((uint32_t)(HAL_GetTick() - start_ms) >
            BSP_RTC_CLOCK_STARTUP_TIMEOUT_MS)
        {
            return BSP_STATUS_TIMEOUT;
        }
    }
    return BSP_STATUS_OK;
}

/**
 * @brief 尝试启用 LSE 并作为 RTC 时钟。
 */
static BSP_Status_t BSP_RTC_SelectLseClock(void)
{
    __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);
    if (BSP_RTC_WaitClockFlag(RCC_FLAG_LSERDY, SET) != BSP_STATUS_OK)
    {
        __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);
        return BSP_STATUS_TIMEOUT;
    }

    __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
    s_rtc_clock_source = RCC_RTCCLKSOURCE_LSE;
    return BSP_STATUS_OK;
}

/**
 * @brief 启用 LSI 并作为 RTC 时钟。
 */
static BSP_Status_t BSP_RTC_SelectLsiClock(void)
{
    __HAL_RCC_LSI_ENABLE();
    if (BSP_RTC_WaitClockFlag(RCC_FLAG_LSIRDY, SET) != BSP_STATUS_OK)
    {
        return BSP_STATUS_TIMEOUT;
    }

    __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI);
    s_rtc_clock_source = RCC_RTCCLKSOURCE_LSI;
    return BSP_STATUS_OK;
}

/**
 * @brief 初始化 RTC 时钟源，优先保留已有备份域配置。
 */
static BSP_Status_t BSP_RTC_ConfigClock(void)
{
    uint32_t current_source;

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    current_source = __HAL_RCC_GET_RTC_SOURCE();
    if (current_source == RCC_RTCCLKSOURCE_LSE)
    {
        s_rtc_clock_source = RCC_RTCCLKSOURCE_LSE;
    }
    else if (current_source == RCC_RTCCLKSOURCE_LSI)
    {
        s_rtc_clock_source = RCC_RTCCLKSOURCE_LSI;
    }
    else
    {
        if (BSP_RTC_SelectLseClock() != BSP_STATUS_OK)
        {
            if (BSP_RTC_SelectLsiClock() != BSP_STATUS_OK)
            {
                return BSP_STATUS_ERROR;
            }
        }
    }

    __HAL_RCC_RTC_ENABLE();
    return BSP_STATUS_OK;
}

/**
 * @brief 初始化 RTC 句柄参数。
 */
static void BSP_RTC_FillHandle(void)
{
    memset(&s_rtc, 0, sizeof(s_rtc));
    s_rtc.Instance = RTC;
    s_rtc.Init.HourFormat = RTC_HOURFORMAT_24;
    s_rtc.Init.AsynchPrediv = 127U;
    s_rtc.Init.SynchPrediv =
        (s_rtc_clock_source == RCC_RTCCLKSOURCE_LSI) ? 249U : 255U;
    s_rtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    s_rtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    s_rtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
}

BSP_Status_t BSP_RTC_Init(void)
{
    if (BSP_RTC_ConfigClock() != BSP_STATUS_OK)
    {
        s_rtc_ready = 0U;
        return BSP_STATUS_ERROR;
    }

    BSP_RTC_FillHandle();
    if (HAL_RTC_Init(&s_rtc) != HAL_OK)
    {
        s_rtc_ready = 0U;
        return BSP_STATUS_ERROR;
    }

    s_rtc_ready = 1U;
    return BSP_STATUS_OK;
}

uint8_t BSP_RTC_IsTimeValid(void)
{
    if (s_rtc_ready == 0U)
    {
        return 0U;
    }

    return (HAL_RTCEx_BKUPRead(&s_rtc,
                               BSP_RTC_BACKUP_MAGIC_REGISTER) ==
            BSP_RTC_MAGIC_VALUE)
               ? 1U
               : 0U;
}

BSP_Status_t BSP_RTC_ReadDateTime(BSP_RTC_DateTime_t *out)
{
    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;

    if ((out == 0) || (s_rtc_ready == 0U))
    {
        return BSP_STATUS_ERROR;
    }
    if (BSP_RTC_IsTimeValid() == 0U)
    {
        return BSP_STATUS_ERROR;
    }

    memset(&date, 0, sizeof(date));
    memset(&time, 0, sizeof(time));
    if (HAL_RTC_GetTime(&s_rtc, &time, RTC_FORMAT_BIN) != HAL_OK)
    {
        return BSP_STATUS_ERROR;
    }
    if (HAL_RTC_GetDate(&s_rtc, &date, RTC_FORMAT_BIN) != HAL_OK)
    {
        return BSP_STATUS_ERROR;
    }

    out->year = (uint16_t)(2000U + date.Year);
    out->month = date.Month;
    out->day = date.Date;
    out->hours = time.Hours;
    out->minutes = time.Minutes;
    out->seconds = time.Seconds;

    return (BSP_RTC_IsDateTimeValid(out) != 0U)
               ? BSP_STATUS_OK
               : BSP_STATUS_ERROR;
}

BSP_Status_t BSP_RTC_WriteDateTime(
    const BSP_RTC_DateTime_t *date_time)
{
    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;

    if ((s_rtc_ready == 0U) ||
        (BSP_RTC_IsDateTimeValid(date_time) == 0U))
    {
        return BSP_STATUS_ERROR;
    }

    memset(&date, 0, sizeof(date));
    memset(&time, 0, sizeof(time));
    time.Hours = date_time->hours;
    time.Minutes = date_time->minutes;
    time.Seconds = date_time->seconds;
    time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    time.StoreOperation = RTC_STOREOPERATION_RESET;

    date.Year = (uint8_t)(date_time->year - 2000U);
    date.Month = date_time->month;
    date.Date = date_time->day;
    date.WeekDay = BSP_RTC_ComputeWeekday(
        date_time->year,
        date_time->month,
        date_time->day);

    if (HAL_RTC_SetTime(&s_rtc, &time, RTC_FORMAT_BIN) != HAL_OK)
    {
        return BSP_STATUS_ERROR;
    }
    if (HAL_RTC_SetDate(&s_rtc, &date, RTC_FORMAT_BIN) != HAL_OK)
    {
        return BSP_STATUS_ERROR;
    }

    HAL_RTCEx_BKUPWrite(&s_rtc,
                        BSP_RTC_BACKUP_MAGIC_REGISTER,
                        BSP_RTC_MAGIC_VALUE);
    return BSP_STATUS_OK;
}
