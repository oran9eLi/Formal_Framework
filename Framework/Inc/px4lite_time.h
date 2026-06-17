/**
 * @file px4lite_time.h
 * @brief 声明 Framework 统一时间服务接口。
 *
 * @details
 * 时间服务以 RTC UTC 时间作为本地连续时间源，以 GNSS UTC 日期时间作为校准源。
 * Display 和 Business 只能读取本服务发布的时间快照，不直接解析 GNSS 或访问 RTC。
 */

#ifndef PX4LITE_TIME_H
#define PX4LITE_TIME_H

#include "px4lite_types.h"

/**
 * @brief 当前时间快照来源。
 */
typedef enum
{
    PX4LITE_TIME_SOURCE_NONE = 0, /**< 尚无可用时间源。 */
    PX4LITE_TIME_SOURCE_RTC,      /**< 当前快照来自 RTC。 */
    PX4LITE_TIME_SOURCE_GNSS      /**< 当前快照来自 GNSS 校准结果。 */
} Px4Lite_TimeSource_t;

/**
 * @brief 时间同步状态。
 */
typedef enum
{
    PX4LITE_TIME_SYNC_INVALID = 0, /**< 尚无有效日期时间。 */
    PX4LITE_TIME_SYNC_RTC_VALID,   /**< RTC 时间有效但本次启动尚未 GNSS 校准。 */
    PX4LITE_TIME_SYNC_GNSS_SYNCED, /**< 本次启动已经通过 GNSS 校准。 */
    PX4LITE_TIME_SYNC_STALE        /**< 曾经校准过，但距离最近校准已经较久。 */
} Px4Lite_TimeSyncState_t;

/**
 * @brief Framework 统一时间快照。
 */
typedef struct
{
    Px4Lite_TopicHeader_t header;      /**< 快照头，包含更新时间和序号。 */
    uint32_t utc_date_ymd;             /**< UTC 日期，编码 YYYYMMDD。 */
    uint32_t utc_time_hhmmss;          /**< UTC 时间，编码 HHMMSS。 */
    uint32_t local_date_ymd;           /**< 本地显示日期，编码 YYYYMMDD。 */
    uint32_t local_time_hhmmss;        /**< 本地显示时间，编码 HHMMSS。 */
    uint32_t last_sync_ms;             /**< 最近一次 GNSS 校准的系统时间，未知时为 0。 */
    uint32_t sync_age_s;               /**< 最近一次 GNSS 校准距今秒数，未知时为 0。 */
    Px4Lite_TimeSource_t source;       /**< 当前快照来源。 */
    Px4Lite_TimeSyncState_t sync_state;/**< 当前同步状态。 */
} Px4Lite_TimeSnapshot_t;

/**
 * @brief 初始化时间服务并尝试读取 RTC。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_TimeInit(void);

/**
 * @brief 执行一次时间服务维护，处理 RTC 读取和 GNSS 周期校准。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 */
void Px4Lite_TimeRun(uint32_t now_ms);

/**
 * @brief 复制最新统一时间快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out);

#endif
