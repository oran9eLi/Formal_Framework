/**
 * @file test_attitude_filter.c
 * @brief 验证 MPU6050 姿态估计的零偏抑制和动态加速度门控。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "px4lite_attitude.h"

static int ExpectAbsLe(const char *label, int32_t actual, int32_t limit)
{
  int32_t abs_value = (actual < 0) ? -actual : actual;

  if (abs_value > limit) {
    printf("%s: |%ld| > %ld\n", label, (long)actual, (long)limit);
    return 1;
  }
  return 0;
}

static void FillLevelImu(Px4Lite_SensorImu_t *imu, uint32_t time_ms)
{
  memset(imu, 0, sizeof(*imu));
  imu->header.valid          = 1U;
  imu->header.sample_time_ms = time_ms;
  imu->sample_period_us      = 10000U;
  imu->accel_mg[0]           = 0;
  imu->accel_mg[1]           = 0;
  imu->accel_mg[2]           = 1000;
}

static int TestStaticGyroBiasDoesNotAccumulateYawDrift(void)
{
  Px4Lite_AttitudeState_t state;
  Px4Lite_SensorImu_t imu;
  int32_t roll = 0;
  int32_t pitch = 0;
  int32_t yaw = 0;
  uint32_t i;

  Px4Lite_AttitudeInit(&state);

  for (i = 0U; i < 260U; i++) {
    FillLevelImu(&imu, 1000U + (i * 10U));
    imu.gyro_mdps[2] = 1200;
    (void)Px4Lite_AttitudeUpdate(&state, &imu, &roll, &pitch, &yaw);
  }

  return ExpectAbsLe("yaw drift after static gyro bias", yaw, 30);
}

static int TestUntrustedAccelerationDoesNotPullRollTowardLinearAcceleration(void)
{
  Px4Lite_AttitudeState_t state;
  Px4Lite_SensorImu_t imu;
  int32_t roll = 0;
  int32_t pitch = 0;
  int32_t yaw = 0;
  uint32_t i;

  Px4Lite_AttitudeInit(&state);

  for (i = 0U; i < 20U; i++) {
    FillLevelImu(&imu, 2000U + (i * 10U));
    (void)Px4Lite_AttitudeUpdate(&state, &imu, &roll, &pitch, &yaw);
  }

  for (i = 0U; i < 50U; i++) {
    FillLevelImu(&imu, 3000U + (i * 10U));
    imu.accel_mg[1] = 1000;
    imu.accel_mg[2] = 1000;
    (void)Px4Lite_AttitudeUpdate(&state, &imu, &roll, &pitch, &yaw);
  }

  return ExpectAbsLe("roll during untrusted 1.41g acceleration", roll, 500);
}

int main(void)
{
  int failures = 0;

  failures += TestStaticGyroBiasDoesNotAccumulateYawDrift();
  failures += TestUntrustedAccelerationDoesNotPullRollTowardLinearAcceleration();

  if (failures != 0) {
    printf("attitude filter tests failed: %d\n", failures);
    return 1;
  }

  printf("attitude filter tests passed\n");
  return 0;
}
