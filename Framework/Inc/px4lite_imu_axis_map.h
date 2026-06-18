/**
 * @file px4lite_imu_axis_map.h
 * @brief Map raw IMU axes to the configured airframe body axes.
 */

#ifndef PX4LITE_IMU_AXIS_MAP_H
#define PX4LITE_IMU_AXIS_MAP_H

#include <stdint.h>

static __inline void Px4Lite_MapImuAxes(
    int32_t accel_mg[3],
    int32_t gyro_mdps[3])
{
    if ((accel_mg == 0) || (gyro_mdps == 0))
    {
        return;
    }

    accel_mg[1] = -accel_mg[1];
    gyro_mdps[0] = -gyro_mdps[0];
    gyro_mdps[2] = -gyro_mdps[2];
}

#endif
