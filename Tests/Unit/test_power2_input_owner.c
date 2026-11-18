/**
 * @file test_power2_input_owner.c
 * @brief 实际Framework采样所有者的BUSY快轮询、1Hz锚点及无输入立即失效到App验证。
 * @details 平台注入结果，其余采集/健康/恢复及环境有效位使用生产实现。
 */
#define main Power2LegacySupplyServiceMain
#include "test_single_supply_service.c"
#undef main
#include "../../Business/Src/app_data_api.c"

/** @brief 本夹具无Baro发布，不能靠无关topic掩盖电源全部失效。 */
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out) { (void)out;return PX4LITE_NOT_READY; }
/** @brief 只复制生产PowerWork通过正常topic出口发布的第一组副本。 */
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out)
{ if(!s_publishes[0]) { return PX4LITE_NOT_READY; }*out=s_latest[0];return PX4LITE_OK; }
/** @brief 失效快照仍是完整topic发布，读者必须按header有效位解释。 */
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out)
{ if(!s_publishes[1]) { return PX4LITE_NOT_READY; }*out=s_latest[1];return PX4LITE_OK; }

/** @brief 下次sensor拍继续等待，仅完成/无输入后恢复原一秒节流，不续期旧测量。 */
int main(void)
{
#if PX4LITE_ENABLE_BATTERY2
  App_EnvironmentSnapshot_t env;unsigned reads,pubs;uint32_t version;
  s_now=0U;assert(Px4Lite_ModulesInit()==PX4LITE_OK);s_read_result[1]=PX4LITE_BUSY;
  TestSensorAt(0U);assert(s_reads[1]==1U && !s_publishes[1] && !TestPowerStatus().error_count);
  TestSensorAt(10U);assert(s_reads[1]==2U && !s_publishes[1]);
  s_read_result[1]=PX4LITE_OK;TestSensorAt(100U);assert(s_reads[1]==3U && s_publishes[1]==1U);
  assert(App_CopyEnvironment(&env,100U)==PX4LITE_OK && (env.valid_mask&APP_ENV_VALID_VOLTAGE2));
  reads=s_reads[1];TestSensorAt(999U);assert(s_reads[1]==reads);
  s_read_result[1]=PX4LITE_BUSY;TestSensorAt(1000U);assert(s_reads[1]==reads+1U);
  TestSensorAt(1010U);assert(s_reads[1]==reads+2U && s_publishes[1]==1U);
  s_read_result[1]=PX4LITE_NOT_READY;TestSensorAt(1100U);assert(s_publishes[1]==2U);
  assert(s_latest[1].header.valid==0U && s_latest[1].header.flags==0U && !s_latest[1].current_valid);
  assert(s_latest[1].header.publish_time_ms==1100U && s_latest[1].header.sample_time_ms==0U);
  assert(!TestPowerStatus().error_count && TestPowerStatus().last_valid_ms==(PX4LITE_ENABLE_BATTERY1?1000U:100U));
  (void)App_CopyEnvironment(&env,1100U);assert(!(env.valid_mask&(APP_ENV_VALID_VOLTAGE2|APP_ENV_VALID_CURRENT2)));
  if(PX4LITE_ENABLE_BATTERY1) { assert(env.valid_mask&APP_ENV_VALID_VOLTAGE1); }
  pubs=s_publishes[1];TestSensorAt(2000U);assert(s_publishes[1]==pubs);
  s_read_result[1]=PX4LITE_BUSY;TestSensorAt(3000U);
  s_read_result[1]=PX4LITE_OK;TestSensorAt(3100U);assert(s_publishes[1]==pubs+1U && s_latest[1].header.sequence==3U);
  assert(App_CopyEnvironment(&env,3100U)==PX4LITE_OK && (env.valid_mask&APP_ENV_VALID_CURRENT2));
  reads=s_reads[1];TestSensorAt(3999U);assert(s_reads[1]==reads);
  s_read_result[1]=PX4LITE_IO_ERROR;version=TestPowerStatus().error_count;TestSensorAt(4000U);
  assert(s_reads[1]==reads+1U && s_publishes[1]==pubs+1U && TestPowerStatus().error_count==version+1U);
  assert(App_CopyEnvironment(&env,4000U)==PX4LITE_OK && (env.valid_mask&APP_ENV_VALID_VOLTAGE2));
  /* 超量程须立即撤销之前有效topic，不能沿普通瞬时I/O故障续用旧电压。 */
  pubs=s_publishes[1];version=TestPowerStatus().error_count;s_read_result[1]=PX4LITE_OVERFLOW;
  TestSensorAt(5000U);assert(s_publishes[1]==pubs+1U && TestPowerStatus().error_count==version+1U);
  assert(!s_latest[1].header.valid && !s_latest[1].header.flags && !s_latest[1].current_valid);
  assert(s_latest[1].header.publish_time_ms==5000U && s_latest[1].header.sample_time_ms==0U);
  assert(s_power_facts[1].io_failed && !s_power_facts[1].low_voltage);
  (void)App_CopyEnvironment(&env,5000U);assert(!(env.valid_mask&(APP_ENV_VALID_VOLTAGE2|APP_ENV_VALID_CURRENT2)));
  if(PX4LITE_ENABLE_BATTERY1) { assert(env.valid_mask&APP_ENV_VALID_VOLTAGE1); }
  pubs=s_publishes[1];version=TestPowerStatus().error_count;TestSensorAt(6000U);
  assert(s_publishes[1]==pubs && TestPowerStatus().error_count==version+1U);
  s_read_result[1]=PX4LITE_OK;TestSensorAt(7000U);assert(s_publishes[1]==pubs+1U && s_latest[1].header.valid);
  assert(App_CopyEnvironment(&env,7000U)==PX4LITE_OK && (env.valid_mask&APP_ENV_VALID_CURRENT2));
  /* 启动首轮已超量程也必须发布无效，而不是等待旧topic超时。 */
  s_now=8000U;assert(Px4Lite_ModulesInit()==PX4LITE_OK);s_read_result[1]=PX4LITE_OVERFLOW;
  TestSensorAt(8000U);assert(s_latest[1].header.sequence==1U && !s_latest[1].header.valid && !s_latest[1].current_valid);
  assert(TestPowerStatus().error_count==1U && s_power_facts[1].io_failed);
  /* pending跨毫秒回绕后，正常下一轮仍锚定第一次启动，不延迟到完成后再等一秒。 */
  s_now=UINT32_MAX-49U;assert(Px4Lite_ModulesInit()==PX4LITE_OK);s_read_result[1]=PX4LITE_BUSY;
  TestSensorAt(s_now);reads=s_reads[1];TestSensorAt(10U);assert(s_reads[1]==reads+1U);
  s_read_result[1]=PX4LITE_OK;TestSensorAt(50U);reads=s_reads[1];
  TestSensorAt(949U);assert(s_reads[1]==reads);TestSensorAt(950U);assert(s_reads[1]==reads+1U);
  puts("Power2 owner/App: startup+running overflow immediate invalid/App N-A bits and I/O count, ordinary IO retained, recovery, prior timing/dual/wrap PASS");
#else
  return Power2LegacySupplyServiceMain();
#endif
  return 0;
}
