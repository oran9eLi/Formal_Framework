/**
 * @file px4lite_alarm.h
 * @brief Maintain the active alarm table derived from framework module status.
 */

#ifndef PX4LITE_ALARM_H
#define PX4LITE_ALARM_H

#include "px4lite_types.h"

/**
 * @brief Reset the active alarm table and sequence counter.
 */
Px4Lite_Result_t Px4Lite_AlarmInit(uint32_t now_ms);

/**
 * @brief Refresh the active alarm table from one coherent module status copy.
 */
void Px4Lite_AlarmUpdateFromStatuses(
    const Px4Lite_ModuleStatus_t *status,
    uint16_t count,
    uint32_t now_ms);

/**
 * @brief Copy the latest active alarm snapshot.
 */
Px4Lite_Result_t Px4Lite_CopyAlarmSnapshot(
    Px4Lite_AlarmSnapshot_t *out);

#endif
