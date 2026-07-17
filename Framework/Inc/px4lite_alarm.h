/**
 * @file px4lite_alarm.h
 * @brief 声明由 Framework 模块状态派生的活动告警表接口。
 */

#ifndef PX4LITE_ALARM_H
#define PX4LITE_ALARM_H

#include "px4lite_types.h"

/**
 * @brief 复位活动告警表和告警序列号。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @retval PX4LITE_OK 初始化成功。
 */
Px4Lite_Result_t Px4Lite_AlarmInit(uint32_t now_ms);

/**
 * @brief 使用一份一致的模块状态快照刷新活动告警表。
 *
 * @param[in] status 模块状态数组，不能为 NULL。
 * @param[in] count 数组元素数量，不能大于 PX4LITE_MODULE_COUNT。
 * @param[in] now_ms 当前系统毫秒时间。
 */
void Px4Lite_AlarmUpdateFromStatuses(const Px4Lite_ModuleStatus_t *status, uint16_t count, uint32_t now_ms);

/**
 * @brief 复制最新活动告警快照。
 *
 * @param[out] out 告警快照输出缓冲区，不能为 NULL。
 *
 * @retval PX4LITE_OK 快照有效。
 * @retval PX4LITE_NOT_READY 快照尚未有效。
 * @retval PX4LITE_INVALID_PARAM 参数非法。
 */
Px4Lite_Result_t Px4Lite_CopyAlarmSnapshot(Px4Lite_AlarmSnapshot_t *out);

/**
 * @brief 复制告警表轻量摘要，避免周期任务在栈上放完整告警表。
 *
 * @param[out] publish_time_ms 告警摘要发布时间，单位：ms，允许为 NULL。
 * @param[out] sequence 告警表序号，允许为 NULL。
 * @param[out] active_count 当前活动告警数量，允许为 NULL。
 * @param[out] highest_fault_code 当前最高严重度故障码，允许为 NULL。
 * @param[out] highest_source_id 当前最高严重度来源 ID，允许为 NULL。
 * @param[out] highest_severity 当前最高告警严重度，允许为 NULL。
 *
 * @retval PX4LITE_OK 摘要有效。
 * @retval PX4LITE_NOT_READY 告警表尚未有效。
 */
Px4Lite_Result_t Px4Lite_CopyAlarmSummary(uint32_t *publish_time_ms, uint32_t *sequence, uint16_t *active_count, uint16_t *highest_fault_code, uint16_t *highest_source_id, Px4Lite_AlarmSeverity_t *highest_severity);

/**
 * @brief 复制指定索引的单条告警记录。
 *
 * @param[in] index 告警记录索引，范围 0 到 `PX4LITE_MODULE_COUNT - 1`。
 * @param[out] out 输出记录，不能为 NULL。
 *
 * @retval PX4LITE_OK 复制成功。
 * @retval PX4LITE_INVALID_PARAM 参数非法。
 * @retval PX4LITE_NOT_READY 告警表尚未有效。
 */
Px4Lite_Result_t Px4Lite_CopyAlarmRecord(uint16_t index, Px4Lite_AlarmRecord_t *out);

#endif
