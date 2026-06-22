/**
 * @file px4lite_imu_axis_map.h
 * @brief 定义 MPU6050 原始轴到机体系轴向的映射规则。
 *
 * @details
 * 本文件属于 Framework 平台适配边界，只处理驱动坐标到机体系坐标的符号映射。
 * Sensor Driver 继续保持芯片原始轴定义，不包含机体安装方向假设。
 */

#ifndef PX4LITE_IMU_AXIS_MAP_H
#define PX4LITE_IMU_AXIS_MAP_H

#include <stdint.h>

/**
 * @brief 将 IMU 三轴数据从驱动轴向映射到当前机体系轴向。
 *
 * @param[in,out] accel_mg 加速度数组，单位：mg，顺序为 X/Y/Z。
 * @param[in,out] gyro_mdps 角速度数组，单位：mdps，顺序为 X/Y/Z。
 *
 * @note 本函数只做符号翻转，不做量纲转换、滤波或有效性判定。
 */
static __inline void Px4Lite_MapImuAxes(int32_t accel_mg[3], int32_t gyro_mdps[3])
{
  if ((accel_mg == 0) || (gyro_mdps == 0)) { return; }

  accel_mg[1]  = -accel_mg[1];
  gyro_mdps[0] = -gyro_mdps[0];
  gyro_mdps[2] = -gyro_mdps[2];
}

#endif
