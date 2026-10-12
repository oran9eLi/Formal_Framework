/** @file test_power_current_valid.c
 * @brief 两路真实电流驱动与 App 快照区分有效 0mA 和 ADC 失败，电压继续有效。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Sensor/Src/sensor_power.c"
#include "../../Sensor/Src/sensor_power2.c"
#include "../../Business/Src/app_data_api.c"
static uint8_t fail_current[2];
static uint32_t clock_ms;
BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *out) { *out=12000U; return BSP_STATUS_OK; }
BSP_Status_t BSP_ADC2_ReadVoltageMv(uint32_t *out) { *out=15000U; return BSP_STATUS_OK; }
BSP_Status_t BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CurrentChannel_t channel,uint32_t *out)
{
  if(fail_current[channel]) { return BSP_STATUS_ERROR; }
  *out=(channel==BSP_ADC_CURRENT_BATTERY1)?POWER_CURRENT_ZERO_MV:POWER2_CURRENT_ZERO_MV;
  return BSP_STATUS_OK;
}
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out) { (void)out; return PX4LITE_NOT_READY; }
static void Battery(Px4Lite_BatteryStatus_t *out,const Power_Snapshot_t *in)
{
  memset(out,0,sizeof(*out)); out->header.valid=1U; out->header.sample_time_ms=clock_ms;
  out->voltage_mv=(uint32_t)(in->voltage_v*1000.0f); out->current_ma=in->current_ma;
  out->current_valid=in->current_valid; out->percent=in->percent;
}
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out) { Battery(out,&s_snapshot); return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out) { Battery(out,&s_snapshot2); return PX4LITE_OK; }
int main(void)
{
  App_EnvironmentSnapshot_t env;
  Power_Status_t status;
  assert(Sensor_Power_Init()==POWER_RESULT_OK && Sensor_Power2_Init()==POWER_RESULT_OK);
  clock_ms=1000U;
  assert(Sensor_Power_Service(clock_ms)==POWER_RESULT_OK && Sensor_Power2_Service(clock_ms)==POWER_RESULT_OK);
  assert(s_snapshot.current_valid==1U && s_snapshot2.current_valid==1U);
  assert(s_snapshot.current_ma==0 && s_snapshot2.current_ma==0);
  assert(App_CopyEnvironment(&env,clock_ms)==PX4LITE_OK);
  assert((env.valid_mask&(APP_ENV_VALID_CURRENT1|APP_ENV_VALID_CURRENT2))==(APP_ENV_VALID_CURRENT1|APP_ENV_VALID_CURRENT2));
  fail_current[0]=1U; clock_ms=2000U;
  assert(Sensor_Power_Service(clock_ms)==POWER_RESULT_OK && Sensor_Power2_Service(clock_ms)==POWER_RESULT_OK);
  assert(Sensor_Power_GetStatus(&status)==POWER_RESULT_OK && status.current_valid==0U);
  assert(App_CopyEnvironment(&env,clock_ms)==PX4LITE_OK);
  assert((env.valid_mask&APP_ENV_VALID_CURRENT1)==0U && (env.valid_mask&APP_ENV_VALID_CURRENT2)!=0U);
  assert((env.valid_mask&(APP_ENV_VALID_VOLTAGE1|APP_ENV_VALID_VOLTAGE2))==(APP_ENV_VALID_VOLTAGE1|APP_ENV_VALID_VOLTAGE2));
  fail_current[0]=0U; fail_current[1]=1U; clock_ms=3000U;
  assert(Sensor_Power_Service(clock_ms)==POWER_RESULT_OK && Sensor_Power2_Service(clock_ms)==POWER_RESULT_OK);
  assert(Sensor_Power2_GetStatus(&status)==POWER_RESULT_OK && status.current_valid==0U);
  assert(App_CopyEnvironment(&env,clock_ms)==PX4LITE_OK);
  assert((env.valid_mask&APP_ENV_VALID_CURRENT1)!=0U && (env.valid_mask&APP_ENV_VALID_CURRENT2)==0U);
  puts("power current validity tests passed"); return 0;
}
