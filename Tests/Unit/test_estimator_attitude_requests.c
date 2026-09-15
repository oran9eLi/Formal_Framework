/** @file test_estimator_attitude_requests.c
 * @brief 真实估计任务请求/快照/发布联动；IMU FIFO 用受控输入替代硬件采样。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Framework/Src/px4lite_modules.c"
#include "../../Framework/Src/px4lite_attitude.c"
static uint32_t now;
static uint8_t available;
static Px4Lite_SensorImu_t input;
static Px4Lite_VehicleNavigation_t published;
uint32_t Px4Lite_PlatformGetMs(void) { return now; }
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_PopImu(Px4Lite_SensorImu_t *out)
{ if (available == 0U) { return PX4LITE_NOT_READY; } *out = input; available = 0U; return PX4LITE_OK; }
void Px4Lite_PublishNavigation(const Px4Lite_VehicleNavigation_t *out) { published = *out; }
static void Step(void)
{
  now += 20U; input.header.sample_time_ms = now; available = 1U;
  Px4Lite_EstimatorRun(now);
}
int main(void)
{
  Px4Lite_AttitudeStatus_t status;
  unsigned i;
  assert(Px4Lite_EstimatorInit() == PX4LITE_OK);
  assert(Px4Lite_RequestAttitudeAction(PX4LITE_ATTITUDE_ACTION_BIAS) == PX4LITE_NOT_READY);
  memset(&input, 0, sizeof(input)); input.header.valid = 1U; input.sample_period_us = 20000U;
  input.accel_mg[1] = 500; input.accel_mg[2] = 866;
  Step();
  assert(Px4Lite_RequestAttitudeLevelCalibration() == PX4LITE_OK);
  assert(Px4Lite_CopyAttitudeStatus(&status) == PX4LITE_OK && status.phase == PX4LITE_ATTITUDE_REQUESTED);
  assert(Px4Lite_RequestAttitudeLevelCalibration() == PX4LITE_BUSY);
  Step();
  assert(Px4Lite_CopyAttitudeStatus(&status) == PX4LITE_OK && status.phase == PX4LITE_ATTITUDE_SUCCEEDED);
  assert(status.reference_active == 1U && status.offset_deg100[0] >= 2990);
  assert(published.roll_deg100 >= 2990); /* 控制/日志真值未被屏幕归零污染。 */
  assert(Px4Lite_RequestAttitudeAction(PX4LITE_ATTITUDE_ACTION_BIAS) == PX4LITE_OK);
  for (i = 0; i < 140U; i++) { Step(); }
  assert(Px4Lite_CopyAttitudeStatus(&status) == PX4LITE_OK && status.bias_valid == 1U);
  assert(status.phase == PX4LITE_ATTITUDE_SUCCEEDED && status.reference_active == 1U);
  assert(Px4Lite_RequestAttitudeAction(PX4LITE_ATTITUDE_ACTION_BIAS) == PX4LITE_OK);
  Step(); now += 2000U; Px4Lite_EstimatorRun(now);
  assert(Px4Lite_CopyAttitudeStatus(&status) == PX4LITE_OK);
  assert(status.phase == PX4LITE_ATTITUDE_FAILED && status.reason == PX4LITE_ATTITUDE_REASON_NO_DATA);
  assert(Px4Lite_RequestAttitudeLevelCalibration() == PX4LITE_NOT_READY);
  puts("estimator attitude request tests passed"); return 0;
}
