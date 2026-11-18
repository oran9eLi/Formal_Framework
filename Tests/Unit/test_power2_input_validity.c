/**
 * @file test_power2_input_validity.c
 * @brief 真实第二组Driver的有效输入回差、无电失效、零电流及重接首帧验证。
 * @details 只注入BSP已完成的正常电压或合作BUSY，不假装硬件开路检测已验证。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Sensor/Src/sensor_power2.c"

static BSP_Status_t s_voltage_result=BSP_STATUS_OK,s_current_result=BSP_STATUS_OK;
static uint32_t s_voltage=12000U,s_current_pin;
static unsigned s_voltage_reads,s_current_reads;
static BSP_Status_t s_diag_result=BSP_STATUS_OK;
static uint8_t s_discharge_enabled=1U,s_discharge_checked=1U,s_sample_valid=1U;
static uint32_t s_discharge_pin_mv=200U;
static uint32_t s_raw_average=1500U,s_raw_max=1500U;
/** @brief 只返回本轮正常测量或恢复等待，不在等待时写输出。 */
BSP_Status_t BSP_ADC2_ReadVoltageMv(uint32_t *out)
{ s_voltage_reads++;if(s_voltage_result==BSP_STATUS_OK) { *out=s_voltage; }return s_voltage_result; }
/** @brief 已完成同轮电压诊断，无ADC访问；可独立注入弱源与正常恢复后的电压差异。 */
BSP_Status_t BSP_ADC2_GetDiag(BSP_ADC2_Diag_t *out)
{
  if(s_diag_result!=BSP_STATUS_OK) { return s_diag_result; }
  memset(out,0,sizeof(*out));out->ready=1U;out->sample_valid=s_sample_valid;
  out->raw_average=s_raw_average;out->raw_max=s_raw_max;
  out->discharge_enabled=s_discharge_enabled;out->discharge_checked=s_discharge_checked;
  out->discharge_pin_mv=s_discharge_pin_mv;out->discharge_status=(uint8_t)BSP_STATUS_OK;
  return BSP_STATUS_OK;
}
/** @brief 未供电时禁止采PC1；有电时0mV允许成为真实0A。 */
BSP_Status_t BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CurrentChannel_t channel,uint32_t *out)
{ assert(channel==BSP_ADC_CURRENT_BATTERY2);s_current_reads++;if(s_current_result==BSP_STATUS_OK) { *out=s_current_pin; }return s_current_result; }
/** @brief 不通过私有快照自证，始终读取公开Driver输出。 */
static Power_Snapshot_t Snapshot(Power_Result_t expected)
{ Power_Snapshot_t value;assert(Sensor_Power2_CopySnapshot(&value)==expected);return value; }
/** @brief 上轨精确复现既有错显，拒绝启动与运行中饱和，恢复首帧不混旧滤波。 */
static void TestSaturationAndCurrentRecovery(void)
{
  Power_Snapshot_t value;Power_Status_t status;unsigned current_before;uint32_t seq,errors;
  const uint32_t rail_voltage_mv=(3300U*101779U)/10000U;
  const uint32_t rail_current_ma=(3300U*1000U)/65U;
  assert(rail_voltage_mv==33587U && rail_current_ma==50769U);
  assert(((uint64_t)rail_voltage_mv*rail_current_ma)/1000U==1705178U);
  s_sample_valid=1U;s_discharge_enabled=1U;s_discharge_checked=1U;
  s_diag_result=BSP_STATUS_OK;s_voltage_result=BSP_STATUS_OK;s_current_result=BSP_STATUS_OK;
  s_voltage=rail_voltage_mv;s_discharge_pin_mv=3300U;s_raw_average=4095U;s_raw_max=4095U;
  assert(Sensor_Power2_Init()==POWER_RESULT_OK);current_before=s_current_reads;
  assert(Sensor_Power2_Service(2000U)==POWER_RESULT_INVALID_DATA);value=Snapshot(POWER_RESULT_INVALID_DATA);
  assert(!value.rx_sequence && !value.current_valid && value.voltage_v==0.0f && !value.power_mw);
  assert(value.error_count==1U && s_current_reads==current_before);
  assert(Sensor_Power2_GetStatus(&status)==POWER_RESULT_INVALID_DATA && !status.current_valid && status.error_count==1U);
  s_raw_average=1500U;s_raw_max=1502U;s_voltage=12000U;s_discharge_pin_mv=200U;s_current_pin=650U;
  assert(Sensor_Power2_Service(2100U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.rx_sequence==1U && value.current_valid && value.current_ma==10000 && value.voltage_v==12.0f);
  seq=value.rx_sequence;errors=value.error_count;current_before=s_current_reads;
  /* 即使平均值尚未到上轨，任何正式样本4095都不应被平均后当作精确电压。 */
  s_raw_average=4000U;s_raw_max=4095U;s_voltage=32800U;s_discharge_pin_mv=3300U;
  assert(Sensor_Power2_Service(2200U)==POWER_RESULT_INVALID_DATA);value=Snapshot(POWER_RESULT_INVALID_DATA);
  assert(value.rx_sequence==seq && value.sample_time_ms==2100U && value.error_count==errors+1U);
  assert(value.voltage_v==0.0f && !value.current_valid && !value.current_ma && !value.power_mw && s_current_reads==current_before);
  s_voltage_result=BSP_STATUS_BUSY;assert(Sensor_Power2_Service(2210U)==POWER_RESULT_PENDING);
  value=Snapshot(POWER_RESULT_INVALID_DATA);assert(value.error_count==errors+1U && value.rx_sequence==seq);
  s_voltage_result=BSP_STATUS_ERROR;assert(Sensor_Power2_Service(2220U)==POWER_RESULT_IO_ERROR);
  value=Snapshot(POWER_RESULT_INVALID_DATA);assert(value.error_count==errors+2U);
  s_voltage_result=BSP_STATUS_OK;s_raw_average=1200U;s_raw_max=1202U;s_voltage=9000U;s_discharge_pin_mv=200U;s_current_pin=POWER2_CURRENT_ZERO_MV;
  assert(Sensor_Power2_Service(2300U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.rx_sequence==seq+1U && value.current_valid && !value.current_ma && value.voltage_v==9.0f && !value.power_mw);
  /* 电流单独失败保持电压有效，但不能让恢复的0A继续带着此前10A的IIR尾值。 */
  assert(Sensor_Power2_Init()==POWER_RESULT_OK);s_voltage=12000U;s_current_pin=650U;
  assert(Sensor_Power2_Service(2400U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);assert(value.current_ma==10000);
  s_current_result=BSP_STATUS_ERROR;assert(Sensor_Power2_Service(2500U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.voltage_v==12.0f && !value.current_valid && !value.power_mw);
  s_current_result=BSP_STATUS_OK;s_current_pin=POWER2_CURRENT_ZERO_MV;
  assert(Sensor_Power2_Service(2600U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.current_valid && value.current_ma==0 && !value.power_mw && value.rx_sequence==3U);
}
/** @brief 无输入不产生当前测量、保持累计序号和错误；恢复等待不伪造错误。 */
int main(void)
{
  Power_Snapshot_t value;Power_Status_t status;unsigned current_before;uint32_t seq,errors;
  assert(POWER_RESULT_OK==0 && POWER_RESULT_NO_DATA==1 && POWER_RESULT_IO_ERROR==2 && POWER_RESULT_INVALID_PARAM==3);
  assert(Sensor_Power2_Init()==POWER_RESULT_OK);Snapshot(POWER_RESULT_NO_DATA);
  s_voltage_result=BSP_STATUS_BUSY;assert(Sensor_Power2_Service(0U)==POWER_RESULT_PENDING);
  assert(!s_current_reads);value=Snapshot(POWER_RESULT_NO_DATA);assert(!value.error_count && !value.rx_sequence);
  s_voltage_result=BSP_STATUS_OK;s_voltage=0U;assert(Sensor_Power2_Service(100U)==POWER_RESULT_NOT_CONNECTED);
  value=Snapshot(POWER_RESULT_NOT_CONNECTED);assert(!value.current_valid && !value.rx_sequence && !value.error_count && !s_current_reads);
  assert(Sensor_Power2_GetStatus(&status)==POWER_RESULT_NOT_CONNECTED && !status.current_valid);
  s_voltage=POWER2_INPUT_ON_MV-1U;assert(Sensor_Power2_Service(200U)==POWER_RESULT_NOT_CONNECTED && !s_current_reads);
  s_voltage=POWER2_INPUT_ON_MV;s_current_pin=POWER2_CURRENT_ZERO_MV;
  assert(Sensor_Power2_Service(300U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.rx_sequence==1U && value.current_valid && value.current_ma==0 && value.sample_time_ms==300U);
  s_voltage=(POWER2_INPUT_ON_MV+POWER2_INPUT_OFF_MV)/2U;
  assert(Sensor_Power2_Service(400U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);seq=value.rx_sequence;
  s_voltage=POWER2_INPUT_OFF_MV;current_before=s_current_reads;
  assert(Sensor_Power2_Service(500U)==POWER_RESULT_NOT_CONNECTED);
  value=Snapshot(POWER_RESULT_NOT_CONNECTED);assert(value.rx_sequence==seq && !value.current_valid && !value.power_mw && s_current_reads==current_before);
  s_voltage=(POWER2_INPUT_ON_MV+POWER2_INPUT_OFF_MV)/2U;
  assert(Sensor_Power2_Service(600U)==POWER_RESULT_NOT_CONNECTED && s_current_reads==current_before);
  s_voltage=12000U;s_current_pin=650U;
  assert(Sensor_Power2_Service(700U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.rx_sequence==seq+1U && value.voltage_v==12.0f && value.current_ma==10000);
  seq=value.rx_sequence;s_voltage=0U;assert(Sensor_Power2_Service(800U)==POWER_RESULT_NOT_CONNECTED);
  s_voltage=9000U;s_current_pin=65U;assert(Sensor_Power2_Service(900U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.rx_sequence==seq+1U && value.voltage_v==9.0f && value.current_ma==1000);seq=value.rx_sequence;
  s_voltage_result=BSP_STATUS_ERROR;current_before=s_current_reads;
  assert(Sensor_Power2_Service(1000U)==POWER_RESULT_IO_ERROR);value=Snapshot(POWER_RESULT_OK);
  assert(value.rx_sequence==seq && value.sample_time_ms==900U && value.current_ma==1000 && s_current_reads==current_before && value.error_count==1U);
  s_voltage_result=BSP_STATUS_OK;s_voltage=9500U;s_current_result=BSP_STATUS_ERROR;
  assert(Sensor_Power2_Service(1100U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.current_valid==0U && value.voltage_v==9.5f && value.error_count==2U);seq=value.rx_sequence;errors=value.error_count;
  s_voltage_result=BSP_STATUS_BUSY;current_before=s_current_reads;
  assert(Sensor_Power2_Service(1110U)==POWER_RESULT_PENDING);value=Snapshot(POWER_RESULT_OK);
  assert(value.rx_sequence==seq && value.error_count==errors && s_current_reads==current_before);
  Sensor_Power2_RequestReinit();assert(Sensor_Power2_Service(1120U)==POWER_RESULT_PENDING);
  value=Snapshot(POWER_RESULT_NO_DATA);assert(value.rx_sequence==seq && value.error_count==errors);
  s_voltage_result=BSP_STATUS_OK;s_current_result=BSP_STATUS_OK;s_voltage=12000U;s_current_pin=POWER2_CURRENT_ZERO_MV;
  assert(Sensor_Power2_Service(1200U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.rx_sequence==seq+1U && value.current_valid && value.current_ma==0 && value.error_count==errors);
  /* 恢复Analog后开路又抬高到8V，弱负载读数仍低，必须拒绝而不能当真实供电。 */
  seq=value.rx_sequence;current_before=s_current_reads;s_voltage=8000U;s_discharge_pin_mv=0U;
  assert(Sensor_Power2_Service(1300U)==POWER_RESULT_NOT_CONNECTED);value=Snapshot(POWER_RESULT_NOT_CONNECTED);
  assert(value.rx_sequence==seq && !value.current_valid && s_current_reads==current_before);
  s_discharge_pin_mv=POWER2_INPUT_SOURCE_MIN_PIN_MV;
  assert(Sensor_Power2_Service(1400U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.current_valid && value.current_ma==0 && value.rx_sequence==seq+1U);seq=value.rx_sequence;
  s_discharge_checked=0U;current_before=s_current_reads;
  assert(Sensor_Power2_Service(1500U)==POWER_RESULT_IO_ERROR && s_current_reads==current_before);
  value=Snapshot(POWER_RESULT_OK);assert(value.rx_sequence==seq);
  s_diag_result=BSP_STATUS_ERROR;
  assert(Sensor_Power2_Service(1600U)==POWER_RESULT_IO_ERROR && s_current_reads==current_before);
  s_diag_result=BSP_STATUS_OK;s_discharge_enabled=0U;s_discharge_pin_mv=0U;s_voltage=12000U;
  assert(Sensor_Power2_Service(1700U)==POWER_RESULT_OK);value=Snapshot(POWER_RESULT_OK);
  assert(value.current_valid && value.current_ma==0 && value.rx_sequence==seq+1U);
  s_sample_valid=0U;assert(Sensor_Power2_Service(1800U)==POWER_RESULT_IO_ERROR);
  TestSaturationAndCurrentRecovery();
  puts("Power2 input Driver: fullscale33587mV/50769mA/1705178mW rejected, startup+running saturation, recovery fresh zero, current-only fault resets IIR, prior input validity PASS");
  return 0;
}
