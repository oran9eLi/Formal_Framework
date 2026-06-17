/**
 * @file px4lite_work.c
 * @brief 实现固定周期工作项调度和执行耗时统计。
 */

#include "px4lite_work.h"

#include "px4lite_platform.h"
#include <string.h>

/**
 * @brief 使用回绕安全的有符号差值判断毫秒截止时间是否到期。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 * @param[in] deadline_ms 截止时间，单位 ms。
 *
 * @return 1 表示已到期，0 表示尚未到期。
 */
static uint8_t Px4Lite_WorkTimeReached(
    uint32_t now_ms,
    uint32_t deadline_ms)
{
    return (((int32_t)(now_ms - deadline_ms)) >= 0) ? 1U : 0U;
}

/**
 * @brief 初始化一个固定周期工作项。
 */
Px4Lite_Result_t Px4Lite_WorkInit(
    Px4Lite_WorkItem_t *item,
    const char *name,
    uint32_t period_ms,
    uint32_t start_ms,
    Px4Lite_WorkFn_t run)
{
    if ((item == 0) || (name == 0) ||
        (period_ms == 0U) || (run == 0))
    {
        return PX4LITE_INVALID_PARAM;
    }

    memset(item, 0, sizeof(*item));
    item->name = name;
    item->run = run;
    item->period_ms = period_ms;
    item->next_run_ms = start_ms;
    item->initialized = 1U;
    return PX4LITE_OK;
}

/**
 * @brief 在到期时运行一个工作项并更新执行统计。
 */
Px4Lite_Result_t Px4Lite_WorkRunDue(
    Px4Lite_WorkItem_t *item,
    uint32_t now_ms)
{
    uint32_t start_us;
    uint32_t elapsed_us;

    if ((item == 0) || (item->initialized == 0U) ||
        (item->run == 0))
    {
        return PX4LITE_INVALID_PARAM;
    }

    if (Px4Lite_WorkTimeReached(now_ms, item->next_run_ms) == 0U)
    {
        return PX4LITE_IDLE;
    }

    while (Px4Lite_WorkTimeReached(
               now_ms, item->next_run_ms + item->period_ms) != 0U)
    {
        item->next_run_ms += item->period_ms;
        item->deadline_miss_count++;
    }

    start_us = Px4Lite_PlatformGetUs();
    item->run(now_ms);
    elapsed_us = Px4Lite_PlatformGetUs() - start_us;

    item->next_run_ms += item->period_ms;
    item->run_count++;
    if (elapsed_us > item->max_execution_us)
    {
        item->max_execution_us = elapsed_us;
    }
    return PX4LITE_OK;
}
