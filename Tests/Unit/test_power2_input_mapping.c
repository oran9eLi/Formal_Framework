/**
 * @file test_power2_input_mapping.c
 * @brief 真实F407平台对第二组未供电与合作等待的转换、诊断撤销和重接序号验证。
 * @details 复用原ADC诊断边界夹具，只注入Driver事实，不伪造物理接头在位。
 */
#define main Power2LegacyDiagnosticsMain
#include "test_power_diagnostics_mapping.c"
#undef main

/** @brief 正常结果、等待、无输入、重接均走实际平台转换，主机不执行ADC。 */
int main(void)
{
#if PX4LITE_ENABLE_BATTERY2
  Px4Lite_BatteryStatus_t value,untouched;Px4Lite_PowerDiagnostics_t diag;unsigned copies,errors;
  SetSample(101U,100U);s_source_diag.discharge_pin_mv=180U;
  s_source_diag.discharge_enabled=1U;s_source_diag.discharge_checked=1U;
  assert(Px4Lite_Battery2Read(&value)==PX4LITE_OK);
  diag=CopyWithoutIO();assert(diag.voltage_valid && diag.current_valid);
  assert(diag.voltage_discharge_pin_mv==180U && diag.voltage_discharge_enabled && diag.voltage_discharge_checked);
  copies=s_snapshot_copies;errors=s_voltage_diag_copies;
  s_now=110U;s_service_result=POWER_RESULT_PENDING;memset(&untouched,0xA5,sizeof(untouched));value=untouched;
  assert(Px4Lite_Battery2Read(&value)==PX4LITE_BUSY && !memcmp(&value,&untouched,sizeof(value)));
  diag=CopyWithoutIO();assert(!diag.voltage_valid && !diag.current_valid && diag.sample_time_ms==100U);
  assert(diag.voltage_discharge_pin_mv==180U && diag.voltage_discharge_enabled && diag.voltage_discharge_checked);
  assert(s_snapshot_copies==copies && s_voltage_diag_copies==errors);
  s_now=120U;s_service_result=POWER_RESULT_NOT_CONNECTED;
  assert(Px4Lite_Battery2Read(&value)==PX4LITE_NOT_READY);
  assert(!value.header.valid && !value.current_valid && value.voltage_mv==0U && value.current_ma==0);
  diag=CopyWithoutIO();assert(!diag.voltage_valid && !diag.current_valid && s_snapshot_copies==copies);
  /* BSP缓存序号没有变化也必须保持无效，不能让原诊断valid在去重早退后复活。 */
  assert(Px4Lite_Battery2Read(&value)==PX4LITE_NOT_READY);diag=CopyWithoutIO();assert(!diag.voltage_valid);
  /* formal仍高但同轮弱拉源低的诊断不可被清valid顺带抹掉，现场需据此复核误拒。 */
  s_now=130U;s_source_diag.sequence=102U;s_source_diag.discharge_pin_mv=8U;
  assert(Px4Lite_Battery2Read(&value)==PX4LITE_NOT_READY);diag=CopyWithoutIO();
  assert(!diag.voltage_valid && diag.voltage_discharge_pin_mv==8U && diag.voltage_discharge_enabled && diag.voltage_discharge_checked);
  SetSample(103U,200U);s_source_sample.current_ma=0;
  assert(Px4Lite_Battery2Read(&value)==PX4LITE_OK && value.current_valid && value.current_ma==0);
  diag=CopyWithoutIO();assert(diag.voltage_valid && diag.current_valid && diag.sequence==103U);
  s_service_result=POWER_RESULT_NO_DATA;assert(Px4Lite_Battery2Read(&value)==PX4LITE_IDLE);
  s_service_result=POWER_RESULT_IO_ERROR;s_source_diag.sequence++;
  assert(Px4Lite_Battery2Read(&value)==PX4LITE_IO_ERROR);
  SetSample(105U,300U);assert(Px4Lite_Battery2Read(&value)==PX4LITE_OK);
  s_now=400U;s_service_result=POWER_RESULT_INVALID_DATA;s_source_diag.sequence=106U;
  s_source_diag.raw_average=4095U;s_source_diag.raw_max=4095U;s_source_diag.pin_mv=3300U;
  s_source_diag.input_mv=33587U;s_source_diag.discharge_pin_mv=3300U;
  s_source_diag.discharge_enabled=1U;s_source_diag.discharge_checked=1U;
  assert(Px4Lite_Battery2Read(&value)==PX4LITE_OVERFLOW);
  assert(!value.header.valid && !value.current_valid && !value.voltage_mv && !value.current_ma);
  diag=CopyWithoutIO();assert(!diag.voltage_valid && !diag.current_valid);
  assert(diag.voltage_raw==4095U && diag.voltage_pin_mv==3300U && diag.voltage_bsp_mv==33587U);
  assert(diag.voltage_discharge_pin_mv==3300U && !diag.voltage_mv);
  SetSample(107U,500U);s_source_sample.current_ma=0;
  assert(Px4Lite_Battery2Read(&value)==PX4LITE_OK && value.current_valid && !value.current_ma);
  diag=CopyWithoutIO();assert(diag.voltage_valid && diag.current_valid && diag.sequence==107U);
  puts("Power2 platform: rail->OVERFLOW immediate invalid with raw diagnostic retained, next-sequence true-zero recovery, prior input mapping PASS");
#else
  Px4Lite_BatteryStatus_t value;assert(Px4Lite_Battery2Read(&value)==PX4LITE_NOT_READY);
  assert(!s_services && !s_snapshot_copies);puts("Power2 platform disabled: no driver service PASS");
#endif
  return 0;
}
