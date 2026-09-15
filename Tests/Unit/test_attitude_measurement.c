/** @file test_attitude_measurement.c
 * @brief 真实姿态算法回归：匀速积分、倾斜上电、低速运动不被掩盖。
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "px4lite_attitude.h"
static Px4Lite_AttitudeState_t state;
static Px4Lite_SensorImu_t imu;
static int32_t roll, pitch, yaw;
static void Step(void)
{
  imu.header.sample_time_ms += 20U;
  assert(Px4Lite_AttitudeUpdate(&state, &imu, &roll, &pitch, &yaw) == PX4LITE_OK);
}
static void Init(void)
{
  Px4Lite_AttitudeInit(&state);
  memset(&imu, 0, sizeof(imu));
  imu.header.valid = 1U;
  imu.sample_period_us = 20000U;
  imu.accel_mg[2] = 1000;
}
int main(void)
{
  unsigned i;
  Init();
  for (i = 0; i < 150U; i++) { Step(); }
  imu.gyro_mdps[2] = 30000;
  for (i = 0; i < 100U; i++) { Step(); }
  assert(yaw >= 5990 && yaw <= 6010); /* 30 deg/s * 2s，容差0.1deg仅覆盖数值舍入。 */
  Init();
  imu.accel_mg[1] = 500; imu.accel_mg[2] = 866;
  for (i = 0; i < 160U; i++) { Step(); }
  assert(roll >= 2990 && roll <= 3010);
  assert(state.gyro_bias_valid == 0U); /* 上电不自动采纳零偏。 */
  Init();
  imu.gyro_mdps[2] = 30000;
  for (i = 0; i < 160U; i++) { Step(); }
  assert(state.gyro_bias_valid == 0U);
  assert(yaw >= 9530 && yaw <= 9550); /* 首帧建基准，159次积分。 */
  Init(); Step();
  imu.gyro_mdps[2] = 500;
  for (i = 0; i < 100U; i++) { Step(); }
  assert(yaw >= 99 && yaw <= 101);
  puts("attitude measurement tests passed");
  return 0;
}
