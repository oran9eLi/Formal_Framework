/**
 * @file px4lite_imu_axis_map.h
 * @brief 定义MPU6050既有安装映射及BMI088可配置的机体系轴向映射。
 *
 * @details
 * 本文件属于 Framework 平台适配边界，只处理驱动坐标到机体系坐标的符号映射。
 * Sensor Driver 继续保持芯片原始轴定义，不包含机体安装方向假设。
 */

#ifndef PX4LITE_IMU_AXIS_MAP_H
#define PX4LITE_IMU_AXIS_MAP_H

#include <stdint.h>
#include "px4lite_config.h"

#define PX4LITE_AXIS_ABS(v) (((v) < 0) ? -(v) : (v))
#if (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_X) < 1) || (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_X) > 3) || \
    (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Y) < 1) || (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Y) > 3) || \
    (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Z) < 1) || (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Z) > 3) || \
    (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_X) == PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Y)) || \
    (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_X) == PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Z)) || \
    (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Y) == PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Z))
#error "BMI088 axis map must be a signed permutation of X/Y/Z"
#endif
#if (PX4LITE_BMI088_AXIS_X * PX4LITE_BMI088_AXIS_Y * PX4LITE_BMI088_AXIS_Z * \
     (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_X) - PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Y)) * \
     (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Y) - PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Z)) * \
     (PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_Z) - PX4LITE_AXIS_ABS(PX4LITE_BMI088_AXIS_X))) <= 0
#error "BMI088 axis map must preserve a right-handed coordinate system"
#endif

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

/**
 * @brief 同时旋转BMI088加速度和角速度轴，默认保留芯片原轴，不沿用MPU模块安装假设。
 * @param[in,out] accel_mg 加速度，单位mg；三个值必须与物理量程一致。
 * @param[in,out] gyro_mdps 角速度，单位mdps。
 * @note 安装映射必须为右手坐标系旋转，实物需六面及三个转轴验收。
 */
static __inline void Px4Lite_MapBmi088Axes(int32_t accel_mg[3], int32_t gyro_mdps[3])
{
  const int8_t axes[3] = {PX4LITE_BMI088_AXIS_X, PX4LITE_BMI088_AXIS_Y, PX4LITE_BMI088_AXIS_Z};
  int32_t accel[3], gyro[3];
  uint8_t i;
  if ((accel_mg == 0) || (gyro_mdps == 0)) { return; }
  for (i = 0U; i < 3U; ++i) { accel[i] = accel_mg[i]; gyro[i] = gyro_mdps[i]; }
  for (i = 0U; i < 3U; ++i) {
    uint8_t index = (uint8_t)(PX4LITE_AXIS_ABS(axes[i]) - 1);
    accel_mg[i] = (axes[i] < 0) ? -accel[index] : accel[index];
    gyro_mdps[i] = (axes[i] < 0) ? -gyro[index] : gyro[index];
  }
}

#undef PX4LITE_AXIS_ABS

#endif
