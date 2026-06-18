/**
 * @file px4lite_mavlink_tx.h
 * @brief Encode framework snapshots and schedule MAVLink telemetry.
 */

#ifndef PX4LITE_MAVLINK_TX_H
#define PX4LITE_MAVLINK_TX_H

#include <stdint.h>
#include "px4lite_types.h"

typedef struct
{
    uint32_t heartbeat_count;
    uint32_t gps_raw_count;
    uint32_t gnss_detail_count;
    uint32_t attitude_count;
    uint32_t position_count;
    uint32_t sys_status_count;
    uint32_t battery_status_count;
    uint32_t scaled_pressure_count;
    uint32_t statustext_count;
    uint32_t no_data_count;
    uint32_t stale_count;
    uint32_t busy_count;
    uint32_t error_count;
    uint32_t last_gps_sequence;
    uint32_t last_detail_sequence;
    uint32_t last_attitude_sequence;
    uint32_t last_battery_sequence;
    uint32_t last_pressure_sequence;
    uint32_t last_alarm_sequence;
    uint32_t last_message_id;
} Px4Lite_MavlinkTxStats_t;

/**
 * @brief Reset MAVLink telemetry deadlines, counters, and sequence state.
 */
Px4Lite_Result_t Px4Lite_MavlinkTxInit(uint32_t now_ms);

/**
 * @brief Run one telemetry scheduling cycle and submit at most one frame.
 */
Px4Lite_Result_t Px4Lite_MavlinkTxRun(uint32_t now_ms);

/**
 * @brief Copy current MAVLink transmission statistics.
 */
void Px4Lite_MavlinkTxGetStats(Px4Lite_MavlinkTxStats_t *out);

#endif
