/**
 * @file test_bmi088_axis_mapping.c
 * @brief 验证BMI088默认原轴和绕Z旋转安装映射，MPU旧映射单独保留。
 * @details 主机逻辑验证，不代替模块六面静置及实际安装检查。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "px4lite_imu_axis_map.h"

/** @brief 确认两类物理向量采用同一安装旋转，避免误用MPU固定翻轴。 */
int main(void)
{
  int32_t accel[3] = {100, -200, 1000};
  int32_t gyro[3] = {-4000, 5000, 6000};
  Px4Lite_MapBmi088Axes(accel, gyro);
#if defined(TEST_BMI088_ROTATED)
  assert(accel[0] == -200 && accel[1] == -100 && accel[2] == 1000);
  assert(gyro[0] == 5000 && gyro[1] == 4000 && gyro[2] == 6000);
#else
  assert(accel[0] == 100 && accel[1] == -200 && accel[2] == 1000);
  assert(gyro[0] == -4000 && gyro[1] == 5000 && gyro[2] == 6000);
#endif
  Px4Lite_MapBmi088Axes(0, gyro);
  Px4Lite_MapBmi088Axes(accel, 0);
  puts("BMI088 installation axis mapping passed");
  return 0;
}
