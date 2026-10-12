/** @file test_estimator_gnss_freshness.c
 * @brief 真实估计任务在无新帧时撤销过期子域，并保留短输入间隙内的新鲜姿态。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Framework/Src/px4lite_modules.c"
#include "../../Framework/Src/px4lite_attitude.c"
static Px4Lite_SensorGnss_t gnss_input;
static Px4Lite_SensorImu_t imu_input;
static Px4Lite_VehicleNavigation_t published;
static uint32_t now_ms, publishes;
static uint8_t have_gnss=1U, have_imu;
uint32_t Px4Lite_PlatformGetMs(void) { return now_ms; }
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out)
{ *out=gnss_input; return have_gnss ? PX4LITE_OK : PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_PopImu(Px4Lite_SensorImu_t *out)
{ if(!have_imu) { return PX4LITE_NOT_READY; } *out=imu_input; have_imu=0U; return PX4LITE_OK; }
void Px4Lite_PublishNavigation(const Px4Lite_VehicleNavigation_t *out) { published=*out; publishes++; }
static void Step(uint32_t now) { now_ms=now; Px4Lite_EstimatorRun(now); }
int main(void)
{
  uint32_t before;
  assert(Px4Lite_EstimatorInit()==PX4LITE_OK);
  gnss_input.header.valid=1U; gnss_input.header.sequence=1U;
  gnss_input.fix_type=1U; gnss_input.fix_dimension=3U; gnss_input.latitude_e7=123;
  gnss_input.valid_mask=PX4LITE_GNSS_VALID_POSITION|PX4LITE_GNSS_VALID_VELOCITY|PX4LITE_GNSS_VALID_TIME|PX4LITE_GNSS_VALID_ALTITUDE;
  Step(0U); assert(published.header.sample_time_ms==0U);
  gnss_input.time_update_ms=1000U; gnss_input.header.sample_time_ms=1000U; gnss_input.header.sequence++;
  Step(1000U); before=publishes;
  Step(PX4LITE_GNSS_MAX_AGE_MS+1U);
  assert(publishes==before+1U);
  assert((published.valid_mask&PX4LITE_NAV_VALID_POSITION)==0U && published.latitude_e7==0);
  assert((published.valid_mask&PX4LITE_NAV_VALID_VELOCITY)==0U);
  assert((published.valid_mask&PX4LITE_NAV_VALID_ALTITUDE)==0U);
  assert((published.valid_mask&PX4LITE_NAV_VALID_TIME)!=0U);
  have_gnss=0U; have_imu=1U; imu_input.header.valid=1U; imu_input.header.sample_time_ms=5000U;
  imu_input.sample_period_us=20000U; imu_input.accel_mg[2]=1000;
  Step(5000U); assert((published.valid_mask&PX4LITE_NAV_VALID_ATTITUDE)!=0U);
  Step(5010U); assert((published.valid_mask&PX4LITE_NAV_VALID_ATTITUDE)!=0U);
  Step(5000U+PX4LITE_IMU_MAX_AGE_MS+1U);
  assert((published.valid_mask&PX4LITE_NAV_VALID_ATTITUDE)==0U);
  puts("estimator GNSS freshness tests passed"); return 0;
}
