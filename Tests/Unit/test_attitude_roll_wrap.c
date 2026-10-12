/** @file test_attitude_roll_wrap.c
 * @brief 验证真实姿态滤波在横滚分支切口附近沿最短角差修正。
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "px4lite_attitude.h"

static Px4Lite_AttitudeState_t state;
static Px4Lite_SensorImu_t imu;
static int32_t roll, pitch, yaw;

/** @brief 设置独立物理姿态输入，每步间隔 20 ms。 */
static void Step(float angle, int32_t rate)
{
  const float rad = angle * 0.0174532925199f;
  imu.accel_mg[1] = (int32_t)lroundf(1000.0f * sinf(rad));
  imu.accel_mg[2] = (int32_t)lroundf(1000.0f * cosf(rad));
  imu.gyro_mdps[0] = rate;
  imu.header.sample_time_ms += 20U;
  assert(Px4Lite_AttitudeUpdate(&state, &imu, &roll, &pitch, &yaw) == PX4LITE_OK);
}

/** @brief 角差仅用于比较物理方向，允许输出自身在 ±180 度处换表示。 */
static float Error(float actual, float expected)
{
  return fabsf(remainderf(actual - expected, 360.0f));
}

/** @brief 每种轨迹从全新滤波器和有效传感器帧开始。 */
static void Reset(void)
{
  Px4Lite_AttitudeInit(&state);
  memset(&imu, 0, sizeof(imu));
  imu.header.valid = 1U;
  imu.sample_period_us = 20000U;
}

int main(void)
{
  int direction;
  unsigned i;
  for (direction = -1; direction <= 1; direction += 2) {
    Reset();
    Step((float)direction * 179.0f, 0);
    Step((float)-direction * 179.0f, 0);
    assert(Error((float)roll / 100.0f, (float)direction * 179.0f) < 0.2f);
    for (i = 0; i < 1000U; ++i) {
      Step((i & 1U) ? 179.8f : -179.8f, 0);
      assert(Error((float)roll / 100.0f, 180.0f) < 1.1f);
    }
    Reset();
    Step(170.0f * (float)direction, 0);
    for (i = 1U; i <= 4000U; ++i) {
      float expected = (170.0f + 0.2f * (float)i) * (float)direction;
      Step(expected, 10000 * direction);
      assert(Error((float)roll / 100.0f, expected) < 0.2f);
      assert(roll >= -18000 && roll <= 18000); /* 百分度取整可把 179.999 度表示为 18000。 */
    }
  }
  puts("attitude roll wrap tests passed");
  return 0;
}
