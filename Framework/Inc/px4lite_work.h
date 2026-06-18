/**
 * @file px4lite_work.h
 * @brief Declare fixed-period work items and execution statistics.
 */

#ifndef PX4LITE_WORK_H
#define PX4LITE_WORK_H

#include "px4lite_types.h"

typedef void (*Px4Lite_WorkFn_t)(uint32_t now_ms);

typedef struct
{
    const char *name;
    Px4Lite_WorkFn_t run;
    uint32_t period_ms;
    uint32_t next_run_ms;
    uint32_t run_count;
    uint32_t deadline_miss_count;
    uint32_t max_execution_us;
    uint8_t initialized;
} Px4Lite_WorkItem_t;

/**
 * @brief Initialize one bounded periodic work item.
 */
Px4Lite_Result_t Px4Lite_WorkInit(
    Px4Lite_WorkItem_t *item,
    const char *name,
    uint32_t period_ms,
    uint32_t start_ms,
    Px4Lite_WorkFn_t run);

/**
 * @brief Run one work item when its wrap-safe deadline is due.
 */
Px4Lite_Result_t Px4Lite_WorkRunDue(
    Px4Lite_WorkItem_t *item,
    uint32_t now_ms);

#endif
