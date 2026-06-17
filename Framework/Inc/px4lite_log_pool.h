/**
 * @file px4lite_log_pool.h
 * @brief Declare the future fixed-size logging memory pool interface.
 */

#ifndef PX4LITE_LOG_POOL_H
#define PX4LITE_LOG_POOL_H

#include "px4lite_config.h"
#include "px4lite_types.h"

/*
 * Reserved contract.
 * Do not call these APIs while PX4LITE_ENABLE_LOG_POOL == 0U.
 * No task, queue, pool block, or heap memory is allocated while disabled.
 */

typedef struct
{
    uint16_t free_count;
    uint16_t free_min;
    uint16_t committed_count;
    uint32_t append_count;
    uint32_t drop_count;
} Px4Lite_LogPoolStats_t;

/**
 * @brief Initialize the reserved fixed-size logging memory pool.
 */
Px4Lite_Result_t Px4Lite_LogPoolInit(void);
/**
 * @brief Append one complete log record to the reserved logging pool.
 */
Px4Lite_Result_t Px4Lite_LogPoolAppend(
    const uint8_t *record,
    uint16_t length);
/**
 * @brief Request pending log records to be made available for storage.
 */
Px4Lite_Result_t Px4Lite_LogPoolFlush(void);
/**
 * @brief Take ownership of one committed log block for writing.
 */
Px4Lite_Result_t Px4Lite_LogPoolTake(
    uint16_t *index,
    const uint8_t **data,
    uint16_t *length);
/**
 * @brief Release a previously taken log block back to the pool.
 */
void Px4Lite_LogPoolRelease(uint16_t index);
/**
 * @brief Copy logging pool capacity and drop statistics.
 */
void Px4Lite_LogPoolGetStats(Px4Lite_LogPoolStats_t *stats);

#endif

