/**
 * @file px4lite_attitude.h
 * @brief Declare the lightweight IMU attitude estimator.
 */

#ifndef PX4LITE_ATTITUDE_H
#define PX4LITE_ATTITUDE_H

#include <stdint.h>
#include "px4lite_types.h"

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32_t last_sample_time_ms;
    uint8_t valid;
    uint8_t reserved[3];
} Px4Lite_AttitudeState_t;

/**
 * @brief Reset the attitude estimator state.
 */
void Px4Lite_AttitudeInit(Px4Lite_AttitudeState_t *state);

/**
 * @brief Update roll and pitch from one IMU sample.
 */
Px4Lite_Result_t Px4Lite_AttitudeUpdate(
    Px4Lite_AttitudeState_t *state,
    const Px4Lite_SensorImu_t *imu,
    int32_t *roll_deg100,
    int32_t *pitch_deg100,
    int32_t *yaw_deg100);

#endif
