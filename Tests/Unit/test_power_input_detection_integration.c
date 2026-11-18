/**
 * @file test_power_input_detection_integration.c
 * @brief 真实ADC2弱泄放到真实Power2驱动的无电消隐、有效零电流与恢复验证。
 * @details 复用确定性GPIO/ADC模型，不证明实物阻抗、拔插检测率或标定准确度。
 */
#define s_initialized2 s_bsp_integration_initialized2
#define main PowerDischargeLegacyMain
#include "test_power_voltage_discharge.c"
#undef main
#undef s_initialized2

#include "../../Sensor/Src/sensor_power2.c"

static unsigned s_current_reads;
static uint32_t s_current_pin_mv;
static BSP_Status_t s_current_result=BSP_STATUS_OK;

/** @brief 电流仅在真实电压源与工作窗口通过后允许被调用；不访问GPIO或真实设备。 */
BSP_Status_t BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CurrentChannel_t channel,uint32_t *voltage_mv)
{
  assert(channel==BSP_ADC_CURRENT_BATTERY2 && voltage_mv);
  assert(s_mode==GPIO_MODE_ANALOG && s_pull==GPIO_NOPULL && !s_critical);
  s_current_reads++;
  if(s_current_result==BSP_STATUS_OK) { *voltage_mv=s_current_pin_mv; }
  return s_current_result;
}

/** @brief 场景从真实BSP释放/Driver初始化开始，不直接改写生产私有状态。 */
static void Fresh(void)
{
  Reset();s_current_reads=0U;s_current_pin_mv=0U;s_current_result=BSP_STATUS_OK;
  assert(Sensor_Power2_Init()==POWER_RESULT_OK);
}

/** @brief 两阶段服务只用独立真毫秒推进，等待期引脚已恢复且不应读取电流。 */
static Power_Result_t Measure(uint32_t start_ms)
{
  unsigned currents=s_current_reads,normal;
  s_ms=start_ms;
  assert(Sensor_Power2_Service(s_ms)==POWER_RESULT_PENDING);
  assert(s_mode==GPIO_MODE_ANALOG && s_pull==GPIO_NOPULL && s_current_reads==currents);
  normal=s_normal;
  s_ms=start_ms+BSP_ADC2_DISCHARGE_RECOVERY_MS-1U;
  assert(Sensor_Power2_Service(s_ms)==POWER_RESULT_PENDING);
  assert(s_normal==normal && s_current_reads==currents);
  s_ms=start_ms+BSP_ADC2_DISCHARGE_RECOVERY_MS;
  return Sensor_Power2_Service(s_ms);
}

/** @brief 12V源通过恢复后才采电流，有效0A保留；失电时清掉历史V/A并停止电流读取。 */
static void TestPoweredZeroCurrentOffAndReconnect(void)
{
  Power_Snapshot_t sample;unsigned before;uint32_t sequence;
  Fresh();
  assert(Measure(0U)==POWER_RESULT_OK && s_current_reads==1U);
  assert(Sensor_Power2_CopySnapshot(&sample)==POWER_RESULT_OK);
  assert(sample.sample_time_ms==BSP_ADC2_DISCHARGE_RECOVERY_MS && sample.current_valid);
  assert(sample.current_ma==0 && sample.power_mw==0U);
  assert(sample.voltage_v>12.008f && sample.voltage_v<12.011f);
  sequence=sample.rx_sequence;assert(sequence==1U);

  s_current_pin_mv=65U;
  assert(Measure(1000U)==POWER_RESULT_OK);
  assert(Sensor_Power2_CopySnapshot(&sample)==POWER_RESULT_OK);
  assert(sample.current_valid && sample.current_ma>0);before=s_current_reads;
  s_powered=0U;s_ghost_raw=1464U;
  assert(Measure(2000U)==POWER_RESULT_NOT_CONNECTED && s_current_reads==before);
  assert(Sensor_Power2_CopySnapshot(&sample)==POWER_RESULT_NOT_CONNECTED);
  assert(!sample.current_valid && !sample.current_ma && !sample.power_mw && sample.voltage_v==0.0f);
  assert(Measure(3000U)==POWER_RESULT_NOT_CONNECTED && s_current_reads==before);

  s_powered=1U;s_normal_raw=1220U;s_pulled_raw=1100U;s_current_pin_mv=0U;
  assert(Measure(4000U)==POWER_RESULT_OK && s_current_reads==before+1U);
  assert(Sensor_Power2_CopySnapshot(&sample)==POWER_RESULT_OK);
  assert(sample.voltage_v>10.0f && sample.voltage_v<10.02f);
  assert(sample.current_valid && sample.current_ma==0); /* 无旧滤波电流混入重连首帧。 */
  assert(sample.rx_sequence>sequence && sample.sample_time_ms==4000U+BSP_ADC2_DISCHARGE_RECOVERY_MS);
}

/** @brief 正式读数仍高但弱源检查近0时，不能继续显示或读取电流；该模型不等同真实模块。 */
static void TestGhostRecoveredHighStillRejected(void)
{
  Power_Snapshot_t sample;
  Fresh();s_powered=1U;s_normal_raw=1464U;s_pulled_raw=0U;
  assert(Measure(0U)==POWER_RESULT_NOT_CONNECTED && !s_current_reads);
  assert(Diag().pin_mv==1180U && Diag().discharge_pin_mv==0U && Diag().sample_valid);
  assert(Sensor_Power2_CopySnapshot(&sample)==POWER_RESULT_NOT_CONNECTED && !sample.current_valid);
  s_pulled_raw=1400U;
  assert(Measure(1000U)==POWER_RESULT_OK && s_current_reads==1U);
  assert(Sensor_Power2_CopySnapshot(&sample)==POWER_RESULT_OK && sample.current_valid && sample.current_ma==0);
}

/** @brief 主回路窗口用还原后的mV，1.5V回差保持状态而非把阈值错用在所选电压脚。 */
static void TestRestoredVoltageWindow(void)
{
  Power_Snapshot_t sample;unsigned before;
  Fresh();s_normal_raw=181U;s_pulled_raw=100U; /* 正式约1.49V主回路，弱源约81mV引脚。 */
  assert(Measure(0U)==POWER_RESULT_NOT_CONNECTED && !s_current_reads);
  s_normal_raw=249U;
  assert(Measure(1000U)==POWER_RESULT_OK && s_current_reads==1U);
  assert(Sensor_Power2_CopySnapshot(&sample)==POWER_RESULT_OK && sample.voltage_v>2.0f);
  s_normal_raw=181U;
  assert(Measure(2000U)==POWER_RESULT_OK && s_current_reads==2U);
  s_normal_raw=122U;s_pulled_raw=94U;before=s_current_reads;
  assert(Measure(3000U)==POWER_RESULT_NOT_CONNECTED && s_current_reads==before);
}

/** @brief 电压转换失败不假装正常等待，恢复/重连仍可返回有效零电流。 */
static void TestFailureThenRecovery(void)
{
  Power_Snapshot_t sample;
  Fresh();s_start_rc=HAL_ERROR;
  assert(Sensor_Power2_Service(0U)==POWER_RESULT_IO_ERROR && !s_current_reads);
  assert(s_mode==GPIO_MODE_ANALOG && s_pull==GPIO_NOPULL);
  s_start_rc=HAL_OK;
  assert(Measure(1000U)==POWER_RESULT_OK && s_current_reads==1U);
  assert(Sensor_Power2_CopySnapshot(&sample)==POWER_RESULT_OK && sample.error_count==1U);
  Sensor_Power2_RequestReinit();
  assert(Measure(2000U)==POWER_RESULT_OK);
  assert(Sensor_Power2_CopySnapshot(&sample)==POWER_RESULT_OK && sample.rx_sequence==2U);
  assert(sample.error_count==1U && sample.current_valid && !sample.current_ma);
}

/** @brief 真实BSP/Driver组合不替代硬件源恢复与标定实测。 */
int main(void)
{
  CheckPhysicalRoute();
  TestPoweredZeroCurrentOffAndReconnect();TestGhostRecoveredHighStillRejected();
  TestRestoredVoltageWindow();TestFailureThenRecovery();
  puts("Power input integration: selected physical route / real ADC2+Power2 / restored Analog / pending / source qualifier / off skips current / zero A valid / first reconnect+filter reset / voltage hysteresis / failure+warm recovery PASS");
  return 0;
}
