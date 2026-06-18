/**
 * @file px4lite_topics.h
 * @brief Declare typed snapshot, FIFO, alarm, and command topics.
 */

#ifndef PX4LITE_TOPICS_H
#define PX4LITE_TOPICS_H

#include "px4lite_types.h"

typedef struct
{
    uint16_t count;
    uint16_t peak;
    uint32_t push_count;
    uint32_t pop_count;
    uint32_t overflow_count;
} Px4Lite_FifoStats_t;

/**
 * @brief Reset topic storage and create enabled optional queues.
 */
Px4Lite_Result_t Px4Lite_TopicsInit(void);

/**
 * @brief Atomically publish one complete GNSS measurement snapshot.
 */
void Px4Lite_PublishGnss(const Px4Lite_SensorGnss_t *data);
/**
 * @brief Copy the latest complete GNSS measurement snapshot.
 */
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *data);

/**
 * @brief Push one IMU sample into the bounded FIFO, dropping the oldest on overflow.
 */
Px4Lite_Result_t Px4Lite_PushImu(const Px4Lite_SensorImu_t *data);
/**
 * @brief Pop the oldest available IMU sample from the FIFO.
 */
Px4Lite_Result_t Px4Lite_PopImu(Px4Lite_SensorImu_t *data);
/**
 * @brief Copy current IMU FIFO usage and overflow statistics.
 */
void Px4Lite_GetImuStats(Px4Lite_FifoStats_t *stats);

/**
 * @brief Atomically publish one complete barometer measurement snapshot.
 */
void Px4Lite_PublishBaro(const Px4Lite_SensorBaro_t *data);
/**
 * @brief Copy the latest complete barometer measurement snapshot.
 */
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *data);

/**
 * @brief Atomically publish one complete battery status snapshot.
 */
void Px4Lite_PublishBattery(const Px4Lite_BatteryStatus_t *data);
/**
 * @brief Copy the latest complete battery status snapshot.
 */
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *data);

/**
 * @brief Atomically publish one complete navigation domain snapshot.
 */
void Px4Lite_PublishNavigation(
    const Px4Lite_VehicleNavigation_t *data);
/**
 * @brief Copy the latest complete navigation domain snapshot.
 */
Px4Lite_Result_t Px4Lite_CopyNavigation(
    Px4Lite_VehicleNavigation_t *data);

/**
 * @brief Atomically publish one complete system health snapshot.
 */
void Px4Lite_PublishHealth(const Px4Lite_SystemHealth_t *data);
/**
 * @brief Copy the latest complete system health snapshot.
 */
Px4Lite_Result_t Px4Lite_CopyHealth(Px4Lite_SystemHealth_t *data);

/**
 * @brief Publish one alarm event to the bounded alarm queue.
 */
Px4Lite_Result_t Px4Lite_PublishAlarm(
    const Px4Lite_AlarmEvent_t *event);
/**
 * @brief Take one pending alarm event without blocking.
 */
Px4Lite_Result_t Px4Lite_TakeAlarm(Px4Lite_AlarmEvent_t *event);

/**
 * @brief Publish one application command to the bounded command queue.
 */
Px4Lite_Result_t Px4Lite_PublishCommand(
    const Px4Lite_Command_t *command);
/**
 * @brief Take one pending application command without blocking.
 */
Px4Lite_Result_t Px4Lite_TakeCommand(Px4Lite_Command_t *command);

/**
 * @brief Publish one command acknowledgement to its bounded queue.
 */
Px4Lite_Result_t Px4Lite_PublishCommandAck(
    const Px4Lite_CommandAck_t *ack);
/**
 * @brief Take one pending command acknowledgement without blocking.
 */
Px4Lite_Result_t Px4Lite_TakeCommandAck(Px4Lite_CommandAck_t *ack);

#endif

