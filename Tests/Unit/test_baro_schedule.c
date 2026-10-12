/** @file test_baro_schedule.c
 * @brief 执行真实 sensor 工作函数，验证 Baro 触发周期与状态推进分离。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Framework/Src/px4lite_modules.c"
static uint32_t now, start_ms;
static unsigned services, triggers, publishes;
static uint8_t waiting, fail_trigger;
static Px4Lite_SensorBaro_t latest;
uint32_t Px4Lite_PlatformGetMs(void) { return now; }
Px4Lite_Result_t Px4Lite_GnssRead(Px4Lite_SensorGnss_t *out) { (void)out; return PX4LITE_IDLE; }
Px4Lite_Result_t Px4Lite_ImuRead(Px4Lite_SensorImu_t *out) { (void)out; return PX4LITE_IDLE; }
Px4Lite_Result_t Px4Lite_BatteryRead(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_IDLE; }
Px4Lite_Result_t Px4Lite_Battery2Read(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_IDLE; }
/** @brief 受控设备在触发后的下一拍完成；策略在真实工作函数中执行。 */
Px4Lite_Result_t Px4Lite_BaroRead(Px4Lite_SensorBaro_t *out, uint8_t trigger)
{
  ++services;
  if (waiting) {
    waiting = 0U;
    out->header.sample_time_ms = start_ms;
    out->pressure_pa = 101325.0f;
    return PX4LITE_OK;
  }
  if (trigger) {
    ++triggers;
    if (fail_trigger) { return PX4LITE_IO_ERROR; }
    waiting = 1U; start_ms = now;
  }
  return PX4LITE_IDLE;
}
void Px4Lite_PublishBaro(const Px4Lite_SensorBaro_t *out) { ++publishes; latest = *out; }
int main(void)
{
  assert(Px4Lite_ModulesInit() == PX4LITE_OK);
  Px4Lite_SensorWorkRun(now);
  assert(services == 1U && triggers == 1U && publishes == 0U);
  now = 10U; Px4Lite_SensorWorkRun(now);
  assert(services == 2U && triggers == 1U && publishes == 1U);
  assert(latest.header.sample_time_ms == 0U && latest.header.publish_time_ms == 10U);
  for (now = 20U; now < 1000U; now += 10U) { Px4Lite_SensorWorkRun(now); }
  assert(services == 100U && triggers == 1U && publishes == 1U);
  Px4Lite_SensorWorkRun(now);
  assert(triggers == 2U);
  now += 10U; Px4Lite_SensorWorkRun(now);
  assert(publishes == 2U && latest.header.sample_time_ms == 1000U);
  fail_trigger = 1U;
  now = 2000U; Px4Lite_SensorWorkRun(now);
  assert(triggers == 3U && publishes == 2U);
  for (now = 2010U; now < 3000U; now += 10U) { Px4Lite_SensorWorkRun(now); }
  assert(triggers == 3U && publishes == 2U);
  fail_trigger = 0U;
  Px4Lite_SensorWorkRun(now);
  now += 10U; Px4Lite_SensorWorkRun(now);
  assert(triggers == 4U && publishes == 3U);
  now = 0xfffffff0U;
  assert(Px4Lite_ModulesInit() == PX4LITE_OK);
  Px4Lite_SensorWorkRun(now);
  now += 10U; Px4Lite_SensorWorkRun(now);
  assert(latest.header.sample_time_ms == 0xfffffff0U);
  now += 10U; Px4Lite_SensorWorkRun(now);
  assert(triggers == 5U);
  now = 984U; Px4Lite_SensorWorkRun(now);
  assert(triggers == 6U);
  puts("Baro scheduling tests passed");
  return 0;
}
