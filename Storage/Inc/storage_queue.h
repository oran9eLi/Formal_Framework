/**
 * @file storage_queue.h
 * @brief Storage 模块固定容量记录队列接口。
 *
 * @details
 * 生产者把完整 CSV 行封装成 `Storage_Record_t` 后入队，Storage task 独占
 * FatFs/SD 写入。队列满时按实现策略统计丢弃，不允许生产者直接执行 SD 写入。
 */

#ifndef STORAGE_QUEUE_H
#define STORAGE_QUEUE_H

#include <stdint.h>
#include "px4lite_types.h"
#include "storage_config.h"

/**
 * @brief 存储记录类型。
 */
typedef enum {
  STORAGE_RECORD_DATA  = 0, /**< 常规传感器/业务数据记录。 */
  STORAGE_RECORD_EVENT = 1, /**< 告警、故障或状态变化事件记录。 */
  STORAGE_RECORD_ERROR = STORAGE_RECORD_EVENT
} Storage_RecordType_t;

/**
 * @brief 一条待写入 SD 的 CSV 记录。
 */
typedef struct {
  Storage_RecordType_t type;       /**< 记录类型，决定写入哪个 CSV 文件。 */
  uint32_t enqueue_time_ms;        /**< 入队时间，单位：ms。 */
  uint32_t target_date_ymd;        /**< 目标日志文件日期，编码：YYYYMMDD。 */
  uint32_t target_time_hhmmss;     /**< 记录产生时的本地时间，编码：HHMMSS。 */
  char line[STORAGE_CSV_LINE_MAX]; /**< 完整 CSV 行，包含行尾换行或结束符由实现约定。 */
} Storage_Record_t;

/**
 * @brief 初始化存储记录队列和统计。
 */
void StorageQueue_Init(void);

/**
 * @brief 入队一条完整存储记录。
 *
 * @param[in] record 待入队记录，不能为 NULL。
 *
 * @return 入队结果。
 * @retval PX4LITE_OK 入队成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_OVERFLOW 队列已满，记录被丢弃。
 */
Px4Lite_Result_t StorageQueue_Push(const Storage_Record_t *record);

/**
 * @brief 非阻塞取出一条待写入记录。
 *
 * @param[out] record 输出缓冲区，不能为 NULL。
 *
 * @return 出队结果。
 * @retval PX4LITE_OK 出队成功。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 队列为空。
 */
Px4Lite_Result_t StorageQueue_Pop(Storage_Record_t *record);

/**
 * @brief 获取当前队列记录数量。
 *
 * @return 当前队列深度。
 */
uint16_t StorageQueue_Count(void);

/**
 * @brief 获取队列累计丢弃记录数量。
 *
 * @return 累计丢弃次数。
 */
uint32_t StorageQueue_DropCount(void);

/**
 * @brief 将文本安全复制到存储记录行缓冲区。
 *
 * @param[in,out] record 目标记录，不能为 NULL。
 * @param[in] line 源 CSV 行字符串，不能为 NULL。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t Storage_RecordSetLine(Storage_Record_t *record, const char *line);

#endif
