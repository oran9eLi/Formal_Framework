/**
 * @file px4lite_work.c
 * @brief Implement fixed-period work dispatch and timing statistics.
 */

#include "px4lite_work.h"

#include "px4lite_platform.h"
#include <string.h>

/**
 * @brief Check a millisecond deadline using wrap-safe signed subtraction.
 */
static uint8_t Px4Lite_WorkTimeReached(
    uint32_t now_ms,
    uint32_t deadline_ms)
{
    return (((int32_t)(now_ms - deadline_ms)) >= 0) ? 1U : 0U;
}

/**
 * @brief Initialize one bounded periodic work item.
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
 * @brief Run one work item when its wrap-safe deadline is due.
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
