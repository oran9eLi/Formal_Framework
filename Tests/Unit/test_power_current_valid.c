/**
 * @file test_power_current_valid.c
 * @brief 通过真实第二组电流驱动和App接口验证单路输入的零值、失败和新鲜度。
 * @details 第一组复制替身记录误读；当前通用输入不授权任何SOC有效位。
 */
#include <assert.h>
#include <stdio.h>
/* 资源只读接口需要任务类型；本用例不创建或扫描任务。 */
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Sensor/Src/sensor_power2.c"
#include "../../Business/Src/app_data_api.c"

static uint8_t fail_current, fail_voltage, source_ready = 1U;
static uint32_t clock_ms;
static unsigned primary_copies, secondary_copies, current_reads;

BSP_Status_t BSP_ADC2_ReadVoltageMv(uint32_t *out)
{ if (fail_voltage) { return BSP_STATUS_ERROR; } *out = 12000U; return BSP_STATUS_OK; }
BSP_Status_t BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CurrentChannel_t channel, uint32_t *out)
{
  assert(channel == BSP_ADC_CURRENT_BATTERY2); current_reads++;
  if (fail_current) { return BSP_STATUS_ERROR; }
  *out = POWER2_CURRENT_ZERO_MV;
  return BSP_STATUS_OK;
}
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out)
{ (void)out; return PX4LITE_NOT_READY; }
/** @brief 禁用组即使有旧快照也不能被App读取或映射为总输入。 */
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out)
{ (void)out; primary_copies++; return PX4LITE_NOT_READY; }
/** @brief 按真实Driver公开快照生成第二组topic替身，保留源采样时刻。 */
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out)
{
  Power_Snapshot_t sample;
  secondary_copies++;
  if (!source_ready || Sensor_Power2_CopySnapshot(&sample) != POWER_RESULT_OK) { return PX4LITE_NOT_READY; }
  memset(out, 0, sizeof(*out)); out->header.valid = 1U;
  out->header.sample_time_ms = sample.sample_time_ms;
  out->header.sequence = sample.rx_sequence;
  out->voltage_mv = (uint32_t)(sample.voltage_v * 1000.0f + 0.5f);
  out->current_ma = sample.current_ma; out->current_valid = sample.current_valid;
  out->percent = sample.percent;
  return PX4LITE_OK;
}
/** @brief 第一组字段不伪造采样来源，SOC内部估算不得成为有效对外数据。 */
static void AssertInactiveFields(const App_EnvironmentSnapshot_t *env)
{
  assert(primary_copies == 0U);
  assert((env->valid_mask & APP_ENV_VALID_BATTERY1) == 0U);
  assert((env->valid_mask & (APP_ENV_VALID_PERCENT1 | APP_ENV_VALID_PERCENT2)) == 0U);
  assert(env->voltage_mv == 0U && env->current_ma == 0 && env->power1_measurement_ms == 0U);
}
/** @brief 合法0mA与ADC失败不能混淆，电流故障不使仍有效的电压失效。 */
int main(void)
{
  App_EnvironmentSnapshot_t env;
  Power_Status_t status;
  unsigned reads_before;
  assert(!PX4LITE_ENABLE_BATTERY1 && PX4LITE_ENABLE_BATTERY2);
  assert(Sensor_Power2_Init() == POWER_RESULT_OK);
  clock_ms = 1000U;
  assert(Sensor_Power2_Service(clock_ms) == POWER_RESULT_OK);
  assert(Sensor_Power2_GetStatus(&status) == POWER_RESULT_OK && status.current_valid == 1U && status.current_ma == 0);
  assert(App_CopyEnvironment(&env, clock_ms) == PX4LITE_OK);
  assert(env.valid_mask == (APP_ENV_VALID_VOLTAGE2 | APP_ENV_VALID_CURRENT2));
  assert(env.voltage2_mv == 12000U && env.current2_ma == 0 && env.power2_measurement_ms == 1000U);
  AssertInactiveFields(&env);

  fail_current = 1U; clock_ms = 2000U;
  assert(Sensor_Power2_Service(clock_ms) == POWER_RESULT_OK);
  assert(Sensor_Power2_GetStatus(&status) == POWER_RESULT_OK && status.current_valid == 0U);
  assert(App_CopyEnvironment(&env, clock_ms) == PX4LITE_OK);
  assert(env.valid_mask == APP_ENV_VALID_VOLTAGE2 && env.voltage2_mv == 12000U);
  assert(env.current2_ma == 0 && env.power2_measurement_ms == 2000U);
  AssertInactiveFields(&env);

  fail_current = 0U; clock_ms = 3000U;
  assert(Sensor_Power2_Service(clock_ms) == POWER_RESULT_OK);
  assert(App_CopyEnvironment(&env, clock_ms) == PX4LITE_OK);
  assert(env.valid_mask == (APP_ENV_VALID_VOLTAGE2 | APP_ENV_VALID_CURRENT2));
  assert(env.current2_ma == 0 && env.power2_measurement_ms == 3000U);
  AssertInactiveFields(&env);
  fail_voltage = 1U; reads_before = current_reads; clock_ms = 4000U;
  assert(Sensor_Power2_Service(clock_ms) == POWER_RESULT_IO_ERROR && current_reads == reads_before);
  assert(App_CopyEnvironment(&env, 3000U + APP_ENVIRONMENT_MAX_AGE_MS) == PX4LITE_OK);
  assert(env.power2_measurement_ms == 3000U); /* 失败不能续期旧测量。 */
  assert(App_CopyEnvironment(&env, 3001U + APP_ENVIRONMENT_MAX_AGE_MS) == PX4LITE_NOT_READY);
  assert(env.valid_mask == 0U && env.voltage2_mv == 0U);
  AssertInactiveFields(&env);
  source_ready = 0U;
  assert(App_CopyEnvironment(&env, clock_ms) == PX4LITE_NOT_READY && env.valid_mask == 0U);
  AssertInactiveFields(&env);
  assert(secondary_copies == 6U && current_reads == 3U);
  puts("power current validity: PASS (input 2, valid zero, current failure independent of voltage, disabled input 1, no SOC, freshness)");
  return 0;
}
