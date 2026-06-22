/**
 * @file px4lite_log_pool.h
 * @brief 固定大小日志记录池的预留接口。
 *
 * @details
 * 日志池用于未来保存不能丢样的结构化记录。当前当 `PX4LITE_ENABLE_LOG_POOL`
 * 为 0 时，不分配任务、队列、池块或堆内存，调用方不得调用这些接口。
 */

#ifndef PX4LITE_LOG_POOL_H
#define PX4LITE_LOG_POOL_H

#include "px4lite_config.h"
#include "px4lite_types.h"

/**
 * @brief 日志池容量和丢弃统计。
 */
typedef struct {
  uint16_t free_count;      /**< 当前空闲块数量。 */
  uint16_t free_min;        /**< 历史最小空闲块数量。 */
  uint16_t committed_count; /**< 已提交、等待写入的记录数量。 */
  uint32_t append_count;    /**< 累计追加记录次数。 */
  uint32_t drop_count;      /**< 因容量不足或参数非法丢弃记录次数。 */
} Px4Lite_LogPoolStats_t;

/**
 * @brief 初始化固定大小日志池。
 *
 * @return 初始化结果。
 *
 * @warning 当 `PX4LITE_ENABLE_LOG_POOL == 0U` 时不得调用。
 */
Px4Lite_Result_t Px4Lite_LogPoolInit(void);

/**
 * @brief 追加一条完整日志记录到固定池。
 *
 * @param[in] record 记录字节缓冲区，不能为 NULL。
 * @param[in] length 记录长度，单位：byte。
 *
 * @return 追加结果。
 *
 * @warning 调用方必须一次提交完整记录，不得提交半条记录。
 */
Px4Lite_Result_t Px4Lite_LogPoolAppend(const uint8_t *record, uint16_t length);

/**
 * @brief 请求将待处理日志记录转为可写入状态。
 *
 * @return 刷新结果。
 */
Px4Lite_Result_t Px4Lite_LogPoolFlush(void);

/**
 * @brief 取出一条已提交日志记录的所有权。
 *
 * @param[out] index 池块索引输出，不能为 NULL。
 * @param[out] data 记录数据指针输出，不能为 NULL。
 * @param[out] length 记录长度输出，单位：byte，不能为 NULL。
 *
 * @return 读取结果。
 *
 * @note 调用方写入完成后必须调用 `Px4Lite_LogPoolRelease()` 释放池块。
 */
Px4Lite_Result_t Px4Lite_LogPoolTake(uint16_t *index, const uint8_t **data, uint16_t *length);

/**
 * @brief 释放已取出的日志池块。
 *
 * @param[in] index 由 `Px4Lite_LogPoolTake()` 返回的池块索引。
 */
void Px4Lite_LogPoolRelease(uint16_t index);

/**
 * @brief 复制日志池容量和丢弃统计。
 *
 * @param[out] stats 输出缓冲区，不能为 NULL。
 */
void Px4Lite_LogPoolGetStats(Px4Lite_LogPoolStats_t *stats);

#endif
