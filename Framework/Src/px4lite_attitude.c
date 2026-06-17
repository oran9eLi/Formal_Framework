/**
 * @file px4lite_attitude.c
 * @brief Implement a first-order complementary roll/pitch estimator.
 */

#include "px4lite_attitude.h"

#include <math.h>
#include <string.h>

#define PX4LITE_ATTITUDE_ALPHA        0.98f
#define PX4LITE_ATTITUDE_RAD_TO_DEG   57.2957795f
#define PX4LITE_ATTITUDE_DEFAULT_DT_S 0.02f
#define PX4LITE_ATTITUDE_MAX_DT_S     0.2f
#define PX4LITE_ATTITUDE_YAW_DEADBAND_DPS 0.75f

static int32_t Px4Lite_AttitudeRoundDeg100(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * 100.0f) + 0.5f);
    }
    return (int32_t)((value * 100.0f) - 0.5f);
}

static float Px4Lite_AttitudeWrapDeg(float value)
{
    while (value >= 180.0f)
    {
        value -= 360.0f;
    }
    while (value < -180.0f)
    {
        value += 360.0f;
    }
    return value;
}

static float Px4Lite_AttitudeApplyYawDeadband(float gyro_z_dps)
{
    if ((gyro_z_dps > -PX4LITE_ATTITUDE_YAW_DEADBAND_DPS) &&
        (gyro_z_dps < PX4LITE_ATTITUDE_YAW_DEADBAND_DPS))
    {
        return 0.0f;
    }
    return gyro_z_dps;
}

static float Px4Lite_AttitudeDtSeconds(
    Px4Lite_AttitudeState_t *state,
    const Px4Lite_SensorImu_t *imu)
{
    float dt_s = PX4LITE_ATTITUDE_DEFAULT_DT_S;

    if (imu->sample_period_us != 0U)
    {
        dt_s = ((float)imu->sample_period_us) / 1000000.0f;
    }
    else if ((state->last_sample_time_ms != 0U) &&
             (imu->header.sample_time_ms != 0U))
    {
        dt_s = ((float)(uint32_t)(imu->header.sample_time_ms -
                                  state->last_sample_time_ms)) /
               1000.0f;
    }

    if ((dt_s <= 0.0f) || (dt_s > PX4LITE_ATTITUDE_MAX_DT_S))
    {
        dt_s = PX4LITE_ATTITUDE_DEFAULT_DT_S;
    }
    return dt_s;
}

void Px4Lite_AttitudeInit(Px4Lite_AttitudeState_t *state)
{
    if (state != 0)
    {
        memset(state, 0, sizeof(*state));
    }
}

Px4Lite_Result_t Px4Lite_AttitudeUpdate(
    Px4Lite_AttitudeState_t *state,
    const Px4Lite_SensorImu_t *imu,
    int32_t *roll_deg100,
    int32_t *pitch_deg100,
    int32_t *yaw_deg100)
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_acc;
    float pitch_acc;
    float roll_gyro;
    float pitch_gyro;
    float dt_s;

    if ((state == 0) || (imu == 0) ||
        (roll_deg100 == 0) || (pitch_deg100 == 0) ||
        (yaw_deg100 == 0))
    {
        return PX4LITE_INVALID_PARAM;
    }

    accel_x_g = ((float)imu->accel_mg[0]) / 1000.0f;
    accel_y_g = ((float)imu->accel_mg[1]) / 1000.0f;
    accel_z_g = ((float)imu->accel_mg[2]) / 1000.0f;
    gyro_x_dps = ((float)imu->gyro_mdps[0]) / 1000.0f;
    gyro_y_dps = ((float)imu->gyro_mdps[1]) / 1000.0f;
    gyro_z_dps = Px4Lite_AttitudeApplyYawDeadband(
        ((float)imu->gyro_mdps[2]) / 1000.0f);

    roll_acc = atan2f(accel_y_g, accel_z_g) *
               PX4LITE_ATTITUDE_RAD_TO_DEG;
    pitch_acc = atan2f(-accel_x_g,
                       sqrtf((accel_y_g * accel_y_g) +
                             (accel_z_g * accel_z_g))) *
                PX4LITE_ATTITUDE_RAD_TO_DEG;

    if (state->valid == 0U)
    {
        state->roll_deg = roll_acc;
        state->pitch_deg = pitch_acc;
        state->yaw_deg = 0.0f;
        state->valid = 1U;
    }
    else
    {
        dt_s = Px4Lite_AttitudeDtSeconds(state, imu);
        roll_gyro = state->roll_deg + (gyro_x_dps * dt_s);
        pitch_gyro = state->pitch_deg + (gyro_y_dps * dt_s);
        state->roll_deg =
            (PX4LITE_ATTITUDE_ALPHA * roll_gyro) +
            ((1.0f - PX4LITE_ATTITUDE_ALPHA) * roll_acc);
        state->pitch_deg =
            (PX4LITE_ATTITUDE_ALPHA * pitch_gyro) +
            ((1.0f - PX4LITE_ATTITUDE_ALPHA) * pitch_acc);
        state->yaw_deg =
            Px4Lite_AttitudeWrapDeg(
                state->yaw_deg + (gyro_z_dps * dt_s));
    }

    state->last_sample_time_ms = imu->header.sample_time_ms;
    *roll_deg100 = Px4Lite_AttitudeRoundDeg100(state->roll_deg);
    *pitch_deg100 = Px4Lite_AttitudeRoundDeg100(state->pitch_deg);
    *yaw_deg100 = Px4Lite_AttitudeRoundDeg100(state->yaw_deg);
    return PX4LITE_OK;
}
