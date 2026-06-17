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
void Px4Lite_AlarmUpdateFromStatuses(
    const Px4Lite_ModuleStatus_t *status,
    uint16_t count,
    uint32_t now_ms);

/**
 * @brief 复制最新活动告警快照。
 *
 * @param[out] out 告警快照输出缓冲区，不能为 NULL。
 *
 * @retval PX4LITE_OK 快照有效。
 * @retval PX4LITE_NOT_READY 快照尚未有效。
 * @retval PX4LITE_INVALID_PARAM 参数非法。
 */
Px4Lite_Result_t Px4Lite_CopyAlarmSnapshot(
    Px4Lite_AlarmSnapshot_t *out);

#endif
