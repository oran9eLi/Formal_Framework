#include <stdint.h>
#include <stdio.h>

#include "px4lite_imu_axis_map.h"

static int ExpectI32(const char *label, int32_t actual, int32_t expected)
{
  if (actual != expected) {
    printf("%s: expected %ld, got %ld\n", label, (long)expected, (long)actual);
    return 0;
  }
  return 1;
}

int main(void)
{
  int32_t accel_mg[3]  = {100, 200, 300};
  int32_t gyro_mdps[3] = {400, 500, 600};
  int ok               = 1;

  Px4Lite_MapImuAxes(accel_mg, gyro_mdps);

  ok &= ExpectI32("accel x unchanged", accel_mg[0], 100);
  ok &= ExpectI32("accel y inverted", accel_mg[1], -200);
  ok &= ExpectI32("accel z unchanged", accel_mg[2], 300);
  ok &= ExpectI32("gyro x inverted", gyro_mdps[0], -400);
  ok &= ExpectI32("gyro y unchanged", gyro_mdps[1], 500);
  ok &= ExpectI32("gyro z inverted", gyro_mdps[2], -600);

  if (ok != 0) {
    printf("platform imu axis mapping tests passed\n");
    return 0;
  }

  return 1;
}
