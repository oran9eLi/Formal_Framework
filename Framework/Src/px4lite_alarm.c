/**
 * @file px4lite_alarm.c
 * @brief 实现供应用层和显示层读取的活动告警表。
 */

#include "px4lite_alarm.h"

#include <string.h>
#include "FreeRTOS.h"
#include "px4lite_config.h"
#include "px4lite_faults.h"
#include "px4lite_topics.h"
#include "task.h"

static Px4Lite_AlarmSnapshot_t s_alarm_snapshot;
static uint32_t s_alarm_sequence;

/**
 * @brief 判断一个模块状态是否应产生活动告警。
 *
 * @param[in] status 模块状态，允许为 NULL。
 *
 * @return 1 表示应生成告警，0 表示无需告警。
 */
static uint8_t Alarm_IsActiveStatus(
    const Px4Lite_ModuleStatus_t *status)
{
    if (status == 0)
    {
        return 0U;
    }

    if ((status->fault_code == PX4LITE_FAULT_NONE) ||
        (status->state == PX4LITE_STATE_ONLINE) ||
        (status->state == PX4LITE_STATE_DISABLED))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 判断候选告警是否应成为当前最高优先级告警。
 *
 * @param[in] candidate 候选告警记录。
 * @param[in] snapshot 正在构造的告警快照。
 *
 * @return 1 表示候选告警优先级更高，0 表示不替换。
 */
static uint8_t Alarm_IsHigherSeverity(
    const Px4Lite_AlarmRecord_t *candidate,
    const Px4Lite_AlarmSnapshot_t *snapshot)
{
    if (snapshot->highest_fault_code == PX4LITE_FAULT_NONE)
    {
        return 1U;
    }

    if ((uint8_t)candidate->severity >
        (uint8_t)snapshot->highest_severity)
    {
        return 1U;
    }

    if (((uint8_t)candidate->severity ==
         (uint8_t)snapshot->highest_severity) &&
        (candidate->updated_ms <
         snapshot->records[snapshot->highest_source_id].updated_ms))
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 将一个模块状态转换为告警记录并写入快照。
 *
 * @param[in,out] snapshot 正在构造的告警快照。
 * @param[in] status 模块状态。
 * @param[in] old_record 上一次同源告警记录，用于保留 raised_ms。
 * @param[in] now_ms 当前系统毫秒时间。
 */
static void Alarm_AddRecord(Px4Lite_AlarmSnapshot_t *snapshot,
                            const Px4Lite_ModuleStatus_t *status,
                            const Px4Lite_AlarmRecord_t *old_record,
                            uint32_t now_ms)
{
    Px4Lite_AlarmRecord_t *record;
    uint16_t source_id;

    source_id = (uint16_t)status->module_id;
    if ((snapshot == 0) ||
        ((uint32_t)source_id >= (uint32_t)PX4LITE_MODULE_COUNT))
    {
        return;
    }

    record = &snapshot->records[source_id];
    memset(record, 0, sizeof(*record));
    record->source_id = source_id;
    record->fault_code = status->fault_code;
    record->severity = (Px4Lite_AlarmSeverity_t)status->severity;
    record->active = 1U;
    record->raised_ms =
        ((old_record != 0) &&
         (old_record->active != 0U) &&
         (old_record->fault_code == status->fault_code))
            ? old_record->raised_ms
            : now_ms;
    record->updated_ms = now_ms;
    record->detail = (uint32_t)status->state;

    snapshot->active_count++;
    if (Alarm_IsHigherSeverity(record, snapshot) != 0U)
    {
        snapshot->highest_fault_code = record->fault_code;
        snapshot->highest_source_id = record->source_id;
        snapshot->highest_severity = record->severity;
    }
}

/**
 * @brief 根据模块状态数组填充一份新的告警快照。
 *
 * @param[out] snapshot 新快照输出。
 * @param[in] old_snapshot 上一次告警快照。
 * @param[in] status 模块状态数组。
 * @param[in] count 模块状态数量。
 * @param[in] now_ms 当前系统毫秒时间。
 */
static void Alarm_FillSnapshot(
    Px4Lite_AlarmSnapshot_t *snapshot,
    const Px4Lite_AlarmSnapshot_t *old_snapshot,
    const Px4Lite_ModuleStatus_t *status,
    uint16_t count,
    uint32_t now_ms)
{
    uint16_t i;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->header.sample_time_ms = now_ms;
    snapshot->header.publish_time_ms = now_ms;
    snapshot->header.sequence = ++s_alarm_sequence;
    snapshot->header.device_id = (uint16_t)PX4LITE_MODULE_ALARM;
    snapshot->header.valid = 1U;
    snapshot->header.flags = PX4LITE_DATA_VALID;
    snapshot->highest_fault_code = PX4LITE_FAULT_NONE;
    snapshot->highest_source_id = (uint16_t)PX4LITE_MODULE_COUNT;
    snapshot->highest_severity = PX4LITE_ALARM_INFO;

    for (i = 0U; i < count; ++i)
    {
        const Px4Lite_ModuleStatus_t *module = &status[i];
        const Px4Lite_AlarmRecord_t *old_record = 0;

        if ((uint32_t)module->module_id >=
            (uint32_t)PX4LITE_MODULE_COUNT)
        {
            continue;
        }

        old_record =
            &old_snapshot->records[(uint16_t)module->module_id];
        if (Alarm_IsActiveStatus(module) != 0U)
        {
            Alarm_AddRecord(snapshot, module, old_record, now_ms);
        }
    }
}

/**
 * @brief 判断同源告警记录的激活状态或严重度是否变化。
 *
 * @param[in] old_record 旧记录。
 * @param[in] new_record 新记录。
 *
 * @return 1 表示发生变化，0 表示未变化。
 */
static uint8_t Alarm_RecordChanged(
    const Px4Lite_AlarmRecord_t *old_record,
    const Px4Lite_AlarmRecord_t *new_record)
{
    return ((old_record->active != new_record->active) ||
            (old_record->fault_code != new_record->fault_code) ||
            (old_record->severity != new_record->severity))
               ? 1U
               : 0U;
}

/**
 * @brief 将一条告警记录发布为告警事件。
 *
 * @param[in] record 告警记录。
 * @param[in] active 1 表示告警激活，0 表示告警清除。
 * @param[in] now_ms 当前系统毫秒时间。
 */
static void Alarm_PublishEvent(const Px4Lite_AlarmRecord_t *record,
                               uint8_t active,
                               uint32_t now_ms)
{
    Px4Lite_AlarmEvent_t event;

    memset(&event, 0, sizeof(event));
    event.timestamp_ms = now_ms;
    event.sequence = s_alarm_sequence;
    event.source_id = record->source_id;
    event.fault_code = record->fault_code;
    event.severity = record->severity;
    event.active = active;
    event.detail = record->detail;
    (void)Px4Lite_PublishAlarm(&event);
}

/**
 * @brief 比较新旧告警快照并发布增量告警事件。
 *
 * @param[in] old_snapshot 旧告警快照。
 * @param[in] new_snapshot 新告警快照。
 * @param[in] now_ms 当前系统毫秒时间。
 */
static void Alarm_PublishChanges(
    const Px4Lite_AlarmSnapshot_t *old_snapshot,
    const Px4Lite_AlarmSnapshot_t *new_snapshot,
    uint32_t now_ms)
{
    uint16_t i;

    for (i = 0U; i < (uint16_t)PX4LITE_MODULE_COUNT; ++i)
    {
        const Px4Lite_AlarmRecord_t *old_record =
            &old_snapshot->records[i];
        const Px4Lite_AlarmRecord_t *new_record =
            &new_snapshot->records[i];

        if (Alarm_RecordChanged(old_record, new_record) == 0U)
        {
            continue;
        }

        if (new_record->active != 0U)
        {
            Alarm_PublishEvent(new_record, 1U, now_ms);
        }
        else if (old_record->active != 0U)
        {
            Alarm_PublishEvent(old_record, 0U, now_ms);
        }
    }
}

Px4Lite_Result_t Px4Lite_AlarmInit(uint32_t now_ms)
{
    memset(&s_alarm_snapshot, 0, sizeof(s_alarm_snapshot));
    s_alarm_sequence = 0U;
    s_alarm_snapshot.header.sample_time_ms = now_ms;
    s_alarm_snapshot.header.publish_time_ms = now_ms;
    s_alarm_snapshot.header.device_id = (uint16_t)PX4LITE_MODULE_ALARM;
    s_alarm_snapshot.header.valid = 1U;
    s_alarm_snapshot.header.flags = PX4LITE_DATA_VALID;
    s_alarm_snapshot.highest_fault_code = PX4LITE_FAULT_NONE;
    s_alarm_snapshot.highest_source_id = (uint16_t)PX4LITE_MODULE_COUNT;
    s_alarm_snapshot.highest_severity = PX4LITE_ALARM_INFO;
    return PX4LITE_OK;
}

void Px4Lite_AlarmUpdateFromStatuses(
    const Px4Lite_ModuleStatus_t *status,
    uint16_t count,
    uint32_t now_ms)
{
#if PX4LITE_ENABLE_ALARM
    Px4Lite_AlarmSnapshot_t old_snapshot;
    Px4Lite_AlarmSnapshot_t new_snapshot;

    if ((status == 0) ||
        (count > (uint16_t)PX4LITE_MODULE_COUNT))
    {
        return;
    }

    taskENTER_CRITICAL();
    old_snapshot = s_alarm_snapshot;
    taskEXIT_CRITICAL();

    Alarm_FillSnapshot(&new_snapshot,
                       &old_snapshot,
                       status,
                       count,
                       now_ms);

    taskENTER_CRITICAL();
    s_alarm_snapshot = new_snapshot;
    taskEXIT_CRITICAL();

    Alarm_PublishChanges(&old_snapshot, &new_snapshot, now_ms);
#else
    (void)status;
    (void)count;
    (void)now_ms;
#endif
}

Px4Lite_Result_t Px4Lite_CopyAlarmSnapshot(
    Px4Lite_AlarmSnapshot_t *out)
{
    if (out == 0)
    {
        return PX4LITE_INVALID_PARAM;
    }

    taskENTER_CRITICAL();
    *out = s_alarm_snapshot;
    taskEXIT_CRITICAL();

    return (out->header.valid != 0U) ? PX4LITE_OK : PX4LITE_NOT_READY;
}
