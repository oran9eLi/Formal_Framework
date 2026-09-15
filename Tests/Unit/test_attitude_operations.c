/** @file test_attitude_operations.c
 * @brief 校准状态、失败保持旧零偏、相对归零与测量隔离回归。
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "px4lite_attitude.h"
static Px4Lite_AttitudeState_t state;
static Px4Lite_SensorImu_t imu;
static int32_t r, p, y;
static void Step(void)
{
  imu.header.sample_time_ms += 20U;
  assert(Px4Lite_AttitudeUpdate(&state, &imu, &r, &p, &y) == PX4LITE_OK);
  Px4Lite_AttitudeService(&state, imu.header.sample_time_ms);
}
int main(void)
{
  unsigned i;
  Px4Lite_AttitudeInit(&state);
  assert(Px4Lite_AttitudeRequest(&state, PX4LITE_ATTITUDE_ACTION_ZERO, 0U) == PX4LITE_NOT_READY);
  assert(state.operation.phase == PX4LITE_ATTITUDE_FAILED);
  memset(&imu, 0, sizeof(imu)); imu.header.valid = 1U; imu.sample_period_us = 20000U;
  imu.accel_mg[1] = 500; imu.accel_mg[2] = 866; imu.gyro_mdps[2] = 200;
  Step();
  assert(Px4Lite_AttitudeRequest(&state, PX4LITE_ATTITUDE_ACTION_BIAS, 20U) == PX4LITE_OK);
  assert(Px4Lite_AttitudeRequest(&state, PX4LITE_ATTITUDE_ACTION_ZERO, 20U) == PX4LITE_BUSY);
  for (i = 0U; i < 60U; i++) { Step(); }
  assert(state.operation.phase == PX4LITE_ATTITUDE_COLLECTING);
  assert(state.operation.progress > 0U && state.operation.progress < 100U);
  /* 重复帧不能冒充连续静止样本。 */
  assert(Px4Lite_AttitudeUpdate(&state, &imu, &r, &p, &y) == PX4LITE_IDLE);
  for (i = 0U; i < 70U; i++) { Step(); }
  assert(state.operation.phase == PX4LITE_ATTITUDE_SUCCEEDED);
  assert(state.operation.progress == 100U && state.gyro_bias_valid == 1U);
  assert(state.gyro_bias_dps[2] > 0.199f && state.gyro_bias_dps[2] < 0.201f);
  assert(r >= 2990 && r <= 3010);
  assert(Px4Lite_AttitudeRequest(&state, PX4LITE_ATTITUDE_ACTION_ZERO, imu.header.sample_time_ms) == PX4LITE_OK);
  assert(state.operation.reference_active == 1U);
  assert(state.operation.offset_deg100[0] >= 2990);
  Step(); assert(r >= 2990); /* 原测量不归零。 */
  assert(Px4Lite_AttitudeRequest(&state, PX4LITE_ATTITUDE_ACTION_RESTORE, imu.header.sample_time_ms) == PX4LITE_OK);
  assert(state.operation.reference_active == 0U && state.operation.offset_deg100[0] == 0);
  assert(Px4Lite_AttitudeRequest(&state, PX4LITE_ATTITUDE_ACTION_BIAS, imu.header.sample_time_ms) == PX4LITE_OK);
  imu.gyro_mdps[2] = 30000;
  for (i = 0U; i < 510U; i++) { Step(); }
  assert(state.operation.phase == PX4LITE_ATTITUDE_FAILED);
  assert(state.operation.reason == PX4LITE_ATTITUDE_REASON_NOT_STILL);
  assert(state.gyro_bias_dps[2] > 0.199f && state.gyro_bias_dps[2] < 0.201f);
  imu.gyro_mdps[2] = 200; Step();
  assert(Px4Lite_AttitudeRequest(&state, PX4LITE_ATTITUDE_ACTION_BIAS, imu.header.sample_time_ms) == PX4LITE_OK);
  Px4Lite_AttitudeService(&state, imu.header.sample_time_ms + 2000U);
  assert(state.operation.phase == PX4LITE_ATTITUDE_FAILED);
  assert(state.operation.reason == PX4LITE_ATTITUDE_REASON_NO_DATA);
  assert(state.operation.measurement_valid == 0U);
  puts("attitude operation tests passed"); return 0;
}
