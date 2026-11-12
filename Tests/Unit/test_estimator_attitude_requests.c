/** @file test_estimator_attitude_requests.c
 * @brief 真实估计任务请求/快照/发布联动；IMU FIFO 用受控输入替代硬件采样。
 */
#include <assert.h>
#include <stdio.h>
/** @brief 单线程主机夹具所需FreeRTOS公开类型，不创建调度器任务。 */
typedef int BaseType_t;
typedef void *TaskHandle_t;

#include "px4lite_log_message_ids.h"
#include "px4lite_local_msglog.h"
/** @brief 日志边界替身；本用例仅验证姿态数学与状态，不启动日志服务。 */
uint8_t Px4Lite_LogMessageSeverity(uint16_t message_id)
{ (void)message_id; return 0U; }
/** @brief 接收姿态事件，保持生产接口完整，不修改待测姿态状态。 */
void Px4Lite_LocalMsgLogEvent(uint16_t message_id, uint32_t now_ms, uint16_t fault_code,
                            uint8_t source_id, uint8_t active)
{ (void)message_id; (void)now_ms; (void)fault_code; (void)source_id; (void)active; }

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
