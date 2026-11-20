/** @file test_module_experiment.c
 * @brief 使用真实模块调度和 topic 验证主动停采、恢复与通信出口隔离。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
static unsigned s_critical;
#define taskENTER_CRITICAL() (++s_critical)
#define taskEXIT_CRITICAL() do { assert(s_critical); --s_critical; } while (0)
#include "../../Framework/Src/px4lite_topics.c"
#include "../../Framework/Src/px4lite_modules.c"

static uint32_t clock_ms;
static unsigned imu_reads, imu_inits, baro_reads, baro_inits, bmi_invalidated;
static unsigned lora_runs, lora_pauses, lora_inits, lora_rx_resets, lora_tx_enabled;
static unsigned rid_runs, rid_aborts, rid_inits, rpi_identity_runs, rpi_runs, cues;
static unsigned gnss_opens, gnss_stops, gnss_uplink_discards, gnss_raw_discards;
static uint8_t motor_blocks, lora_enabled_at_rpi_rx;
static Px4Lite_Result_t battery2_result = PX4LITE_IDLE;
static uint8_t battery2_current_valid = 1U;

/** @brief 仅提供独立外设和输出边界，检查调度上下文与暂停后的调用计数。 */
uint32_t Px4Lite_PlatformGetMs(void) { return clock_ms; }
uint32_t Px4Lite_PlatformGetUs(void) { return clock_ms * 1000U; }
void Px4Lite_RuntimeReportComm(uint32_t now) { assert(now == clock_ms); }
void Px4Lite_RuntimeReportCommWork(uint32_t us) { (void)us; }
void Px4Lite_RuntimeReportSensorWork(uint32_t us) { (void)us; }
void Px4Lite_CaptureWorkRun(void) { }
Px4Lite_Result_t Px4Lite_GnssRead(Px4Lite_SensorGnss_t *out) { (void)out; return PX4LITE_IDLE; }
Px4Lite_Result_t Px4Lite_GnssSetExperimentEnabled(uint8_t enabled)
{ assert(!s_critical); if (enabled) { gnss_opens++; } else { gnss_stops++; } return PX4LITE_OK; }
void Px4Lite_GnssRawUplinkDiscard(void) { assert(!s_critical); gnss_raw_discards++; }
void Px4Lite_GnssUplinkDiscard(uint32_t now)
{ assert(!s_critical && now == clock_ms); gnss_uplink_discards++; }
Px4Lite_Result_t Px4Lite_BatteryRead(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_IDLE; }
Px4Lite_Result_t Px4Lite_Battery2Read(Px4Lite_BatteryStatus_t *out)
{
  out->header.sample_time_ms = clock_ms;
  out->voltage_mv = 12000U;
  out->current_valid = battery2_current_valid;
  return battery2_result;
}
Px4Lite_Result_t Px4Lite_ImuInit(void) { assert(!s_critical); imu_inits++; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_BaroInit(void) { assert(!s_critical); baro_inits++; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_ImuRead(Px4Lite_SensorImu_t *out)
{ assert(!s_critical); out->gyro_mdps[0] = (int32_t)++imu_reads; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_BaroRead(Px4Lite_SensorBaro_t *out, uint8_t permit)
{ assert(!s_critical && permit); out->pressure_pa = 100000U + ++baro_reads; return PX4LITE_OK; }
uint8_t Px4Lite_ControlModulePauseAllowed(Px4Lite_ModuleId_t id)
{ assert(s_critical && (id == PX4LITE_MODULE_IMU || id == PX4LITE_MODULE_BARO)); return (uint8_t)!motor_blocks; }
void Px4Lite_Bmi088UplinkInvalidate(void) { assert(!s_critical); bmi_invalidated++; }
void Px4Lite_StartupToneNotifyRpiResult(uint8_t success) { assert(success && !s_critical); cues++; }
Px4Lite_Result_t Px4Lite_RpiMavlinkService(uint32_t now) { assert(now == clock_ms); return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_MavlinkRxRunRpi(uint32_t now)
{
  assert(now == clock_ms);
  lora_enabled_at_rpi_rx = Px4Lite_ModuleExperimentEnabled(PX4LITE_MODULE_LORA);
  return PX4LITE_IDLE;
}
Px4Lite_Result_t Px4Lite_MavlinkTxRunRpi(uint32_t now) { assert(now == clock_ms); rpi_runs++; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_LoRaInit(void) { assert(!s_critical); lora_inits++; return PX4LITE_OK; }
void Px4Lite_LoRaRequestReinit(void) { assert(0); }
Px4Lite_Result_t Px4Lite_LoRaService(uint32_t now) { assert(now == clock_ms); lora_runs++; return PX4LITE_OK; }
void Px4Lite_LoRaPause(void) { assert(!s_critical); lora_pauses++; }
void Px4Lite_LoraLabRun(uint32_t now) { assert(now == clock_ms); }
Px4Lite_Result_t Px4Lite_LoraLabRequest(const Px4Lite_LabRequest_t *request)
{ assert(request->operation == PX4LITE_LAB_STOP); return PX4LITE_OK; }
void Px4Lite_RemoteDiscoveryRun(uint32_t now) { assert(now == clock_ms); }
void Px4Lite_RemoteTelemetryResetLink(uint32_t now) { assert(now == clock_ms); }
Px4Lite_Result_t Px4Lite_MavlinkRxRun(uint32_t now) { assert(now == clock_ms); return PX4LITE_IDLE; }
void Px4Lite_MavlinkRxResetLoRa(void) { lora_rx_resets++; }
uint32_t Px4Lite_MavlinkRxLastPeerMs(void) { return 0U; }
Px4Lite_Result_t Px4Lite_MavlinkTxRun(uint32_t now) { assert(now == clock_ms); return PX4LITE_OK; }
void Px4Lite_MavlinkSetTxEnabled(uint8_t enabled) { lora_tx_enabled = enabled; }
void Px4Lite_LoRaGetDebugInfo(Px4Lite_CommDebugInfo_t *out) { memset(out, 0, sizeof(*out)); }
Px4Lite_State_t Px4Lite_LoRaGetState(uint32_t now) { assert(now == clock_ms); return PX4LITE_STATE_ONLINE; }
uint8_t Px4Lite_LoRaIsPresent(void) { return 1U; }
Px4Lite_Result_t Px4Lite_RemoteIdInit(void) { assert(!s_critical); rid_inits++; return PX4LITE_OK; }
uint8_t Px4Lite_RemoteIdIsReady(void) { return 1U; }
uint8_t Px4Lite_RemoteIdIsPresent(void) { return 1U; }
void Px4Lite_RemoteIdAbortTx(void) { assert(!s_critical); rid_aborts++; }
Px4Lite_Result_t Px4Lite_RemoteIdTxInit(uint32_t now) { assert(now == clock_ms); return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RemoteIdTxRun(uint32_t now) { assert(now == clock_ms); rid_runs++; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RemoteIdTxRunRpiIdentity(uint32_t now)
{ assert(now == clock_ms); rpi_identity_runs++; return PX4LITE_OK; }
void Px4Lite_RemoteIdTxGetStats(Px4Lite_RemoteIdTxStats_t *out)
{ memset(out, 0, sizeof(*out)); out->basic_id_count = 3U; out->location_count = 5U; }

/** @brief 互锁拒绝保留现用数据；受理后主题立即失效，由 sensor 恢复新样本。 */
static void TestSensors(void)
{
  Px4Lite_SensorImu_t imu;
  Px4Lite_SensorBaro_t baro;
  Px4Lite_VehicleNavigation_t navigation = {0};
  Px4Lite_ModuleStatus_t status;
  clock_ms = 100U;
  Px4Lite_SensorWorkRun(clock_ms);
  assert(Px4Lite_CopyImuLatest(&imu) == PX4LITE_OK && imu.gyro_mdps[0] == 1);
  assert(Px4Lite_CopyBaro(&baro) == PX4LITE_OK && baro.pressure_pa == 100001U);
  navigation.valid_mask = PX4LITE_NAV_VALID_ATTITUDE | PX4LITE_NAV_VALID_YAW_REL | PX4LITE_NAV_VALID_POSITION;
  Px4Lite_PublishNavigation(&navigation);
  motor_blocks = 1U;
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_IMU, 0U, clock_ms) == PX4LITE_BUSY);
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_BARO, 0U, clock_ms) == PX4LITE_BUSY);
  assert(Px4Lite_CopyImuLatest(&imu) == PX4LITE_OK && Px4Lite_CopyBaro(&baro) == PX4LITE_OK);
  motor_blocks = 0U;
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_IMU, 0U, clock_ms) == PX4LITE_OK);
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_BARO, 0U, clock_ms) == PX4LITE_OK);
  assert(Px4Lite_CopyImuLatest(&imu) == PX4LITE_NOT_READY && Px4Lite_PopImu(&imu) == PX4LITE_NOT_READY);
  assert(Px4Lite_CopyBaro(&baro) == PX4LITE_NOT_READY);
  assert(Px4Lite_CopyNavigation(&navigation) == PX4LITE_OK && navigation.valid_mask == PX4LITE_NAV_VALID_POSITION);
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_IMU, &status) == PX4LITE_OK && status.state == PX4LITE_STATE_DISABLED);
  clock_ms = 300U; Px4Lite_SensorWorkRun(clock_ms);
  assert(imu_reads == 1U && baro_reads == 1U && bmi_invalidated == 1U);
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_IMU, 1U, clock_ms) == PX4LITE_OK);
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_BARO, 1U, clock_ms) == PX4LITE_OK);
  assert(Px4Lite_CopyImuLatest(&imu) == PX4LITE_NOT_READY && Px4Lite_CopyBaro(&baro) == PX4LITE_NOT_READY);
  clock_ms = 400U; Px4Lite_SensorWorkRun(clock_ms);
  assert(imu_inits == 1U && baro_inits == 1U);
  assert(Px4Lite_CopyImuLatest(&imu) == PX4LITE_OK && imu.gyro_mdps[0] == 2);
  assert(Px4Lite_CopyBaro(&baro) == PX4LITE_OK && baro.pressure_pa == 100002U);
}

/** @brief LoRa 和 UART4 停收发时 UART6 周期业务和身份继续，OFF 回执不改选页。 */
static void TestComm(void)
{
  Px4Lite_ModuleExperimentDisplay_t display;
  assert(Px4Lite_RemoteIdModuleInit() == PX4LITE_OK);
  Px4Lite_CommWorkRun(clock_ms);
  Px4Lite_ModuleExperimentCopyDisplay(&display);
  assert(display.remote_id_submitted_count == 8U && lora_runs == 1U && rid_runs == 1U);
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_LORA, 0U, clock_ms) == PX4LITE_OK);
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_REMOTE_ID, 0U, clock_ms) == PX4LITE_OK);
  clock_ms += 10U; Px4Lite_CommWorkRun(clock_ms);
  Px4Lite_ModuleExperimentAck(PX4LITE_MODULE_REMOTE_ID, 0U);
  Px4Lite_ModuleExperimentCopyDisplay(&display);
  assert(display.remote_id_submitted_count == 0U && display.page_sequence == 0U && cues == 1U);
  assert(lora_pauses == 1U && lora_rx_resets == 1U && lora_tx_enabled == 0U && rid_aborts == 1U);
  assert(lora_runs == 1U && rid_runs == 1U && rpi_runs == 2U && rpi_identity_runs == 2U);
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_LORA, 1U, clock_ms) == PX4LITE_OK);
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_REMOTE_ID, 1U, clock_ms) == PX4LITE_OK);
  clock_ms += 10U; Px4Lite_CommWorkRun(clock_ms);
  Px4Lite_ModuleExperimentAck(PX4LITE_MODULE_REMOTE_ID, 1U);
  Px4Lite_ModuleExperimentCopyDisplay(&display);
  assert(display.remote_id_submitted_count == 8U && display.page_sequence == 1U && cues == 2U);
  assert(display.page_module == PX4LITE_MODULE_REMOTE_ID && lora_inits == 1U && lora_tx_enabled == 1U && rid_inits == 2U);
  assert(lora_runs == 2U && rid_runs == 2U && rpi_runs == 3U && rpi_identity_runs == 3U);
}

/** @brief 本地开启等待旧关闭动作完成，复用对应所有者且不产生远端选页或声光。 */
static void TestLocalOpen(void)
{
  static const Px4Lite_ModuleId_t ids[] = {
    PX4LITE_MODULE_GNSS, PX4LITE_MODULE_IMU, PX4LITE_MODULE_BARO,
    PX4LITE_MODULE_LORA, PX4LITE_MODULE_REMOTE_ID
  };
  Px4Lite_ModuleExperimentDisplay_t before, after;
  const unsigned imu_before = imu_inits, baro_before = baro_inits;
  const unsigned lora_before = lora_inits, rid_before = rid_inits, cues_before = cues;
  unsigned i;

  Px4Lite_ModuleExperimentCopyDisplay(&before);
  assert(Px4Lite_ModuleExperimentRequestLocalOpen(PX4LITE_MODULE_STORAGE) == PX4LITE_INVALID_PARAM);
  assert(Px4Lite_ModuleExperimentRequestLocalOpen(PX4LITE_MODULE_COUNT) == PX4LITE_INVALID_PARAM);
  assert(Px4Lite_ModuleExperimentRequestLocalOpen(PX4LITE_MODULE_GNSS) == PX4LITE_OK);
  for (i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
    assert(Px4Lite_ModuleExperimentRequest(ids[i], 0U, clock_ms) == PX4LITE_OK);
    assert(Px4Lite_ModuleExperimentRequestLocalOpen(ids[i]) == PX4LITE_OK);
    assert(Px4Lite_ModuleExperimentEnabled(ids[i]) == 0U);
  }
  assert(Px4Lite_ModuleExperimentRequestLocalOpen(PX4LITE_MODULE_IMU) == PX4LITE_OK);

  clock_ms += 10U; Px4Lite_CommWorkRun(clock_ms);
  Px4Lite_ModuleExperimentCopyDisplay(&after);
  assert(after.disabled_mask == MODULE_EXPERIMENT_MASK);
  assert(lora_pauses == 2U && rid_aborts == 2U && gnss_opens == 0U);

  Px4Lite_SensorWorkRun(clock_ms);
  assert(gnss_stops == 1U && gnss_opens == 0U);
  clock_ms += 10U; Px4Lite_CommWorkRun(clock_ms);
  Px4Lite_ModuleExperimentCopyDisplay(&after);
  assert(lora_enabled_at_rpi_rx == 0U && after.disabled_mask == 0U);
  assert(lora_inits == lora_before + 1U && rid_inits == rid_before + 1U && lora_tx_enabled == 1U);
  assert(after.page_sequence == before.page_sequence && after.page_module == before.page_module);
  assert(cues == cues_before && imu_inits == imu_before && baro_inits == baro_before);

  Px4Lite_SensorWorkRun(clock_ms);
  assert(gnss_opens == 1U && imu_inits == imu_before + 1U && baro_inits == baro_before + 1U);
  assert(gnss_raw_discards == 1U && gnss_uplink_discards == 1U);

  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_LORA, 0U, clock_ms) == PX4LITE_OK);
  clock_ms += 10U; Px4Lite_CommWorkRun(clock_ms);
  clock_ms += 10U; Px4Lite_CommWorkRun(clock_ms);
  assert(Px4Lite_ModuleExperimentEnabled(PX4LITE_MODULE_LORA) == 0U);
}

/** @brief 通过采集所有者及真实 topic 检查电压故障与历史读数的独立性。 */
static void TestBattery2VoltageFault(void)
{
  Px4Lite_BatteryStatus_t good, updated;
  clock_ms += PX4LITE_BATTERY_WORK_PERIOD_MS;
  battery2_result = PX4LITE_OK;
  Px4Lite_PowerWorkRun(clock_ms);
  assert(Px4Lite_CopyBattery2(&good) == PX4LITE_OK && good.header.valid && good.current_valid);

  clock_ms += PX4LITE_BATTERY_WORK_PERIOD_MS;
  battery2_result = PX4LITE_IO_ERROR;
  Px4Lite_PowerWorkRun(clock_ms);
  assert(Px4Lite_CopyBattery2(&updated) == PX4LITE_OK);
  assert((updated.header.flags & PX4LITE_DATA_BATTERY2_VOLTAGE_IO_FAULT) != 0U);
  assert(updated.header.valid && updated.voltage_mv == good.voltage_mv);
  assert(updated.header.sample_time_ms == good.header.sample_time_ms);
  assert(updated.header.publish_time_ms == good.header.publish_time_ms);
  assert(updated.header.sequence == good.header.sequence);

  clock_ms += PX4LITE_BATTERY_WORK_PERIOD_MS;
  battery2_result = PX4LITE_BUSY;
  Px4Lite_PowerWorkRun(clock_ms);
  assert(Px4Lite_CopyBattery2(&updated) == PX4LITE_OK);
  assert((updated.header.flags & PX4LITE_DATA_BATTERY2_VOLTAGE_IO_FAULT) != 0U);

  clock_ms += PX4LITE_SENSOR_WORK_PERIOD_MS;
  battery2_result = PX4LITE_OK;
  battery2_current_valid = 0U;
  Px4Lite_PowerWorkRun(clock_ms);
  assert(Px4Lite_CopyBattery2(&updated) == PX4LITE_OK);
  assert(updated.header.valid && !updated.current_valid && updated.header.sample_time_ms == clock_ms);
  assert((updated.header.flags & PX4LITE_DATA_BATTERY2_VOLTAGE_IO_FAULT) == 0U);
  assert((updated.header.flags & PX4LITE_DATA_DEGRADED) != 0U);
}

int main(void)
{
  assert(Px4Lite_ModulesInit() == PX4LITE_OK);
  TestSensors(); TestComm(); TestLocalOpen();
  TestBattery2VoltageFault();
  assert(!s_critical);
  puts("Module experiment: owners / invalidation / interlock / UART6 / ACK / reopen / Battery2 voltage fault and recovery PASS");
  return 0;
}
