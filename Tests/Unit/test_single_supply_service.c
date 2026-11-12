/**
 * @file test_single_supply_service.c
 * @brief 使用真实 Framework 采集、健康与恢复服务验证电源通道开关。
 * @details 主机测试仅替代平台 I/O 和 topic 出口，不访问硬件，不代替实板验收。
 */
#include <assert.h>
#include <stdio.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Framework/Src/px4lite_modules.c"
#include "../../Framework/Src/px4lite_recovery.c"

static uint32_t s_now;
static unsigned s_reads[2], s_publishes[2], s_reinit_requests;
static Px4Lite_Result_t s_read_result[2] = {PX4LITE_OK, PX4LITE_OK};
static uint8_t s_current_valid[2] = {1U, 1U};
static uint8_t s_reference_degraded[2];
static Px4Lite_BatteryStatus_t s_latest[2];
static Px4Lite_SystemHealth_t s_latest_health;

uint32_t Px4Lite_PlatformGetMs(void) { return s_now; }
uint32_t Px4Lite_PlatformGetUs(void) { return s_now * 1000U; }
void Px4Lite_RuntimeReportSensorWork(uint32_t elapsed_us) { (void)elapsed_us; }
Px4Lite_Result_t Px4Lite_GnssRead(Px4Lite_SensorGnss_t *out) { (void)out; return PX4LITE_IDLE; }
Px4Lite_Result_t Px4Lite_ImuRead(Px4Lite_SensorImu_t *out) { (void)out; return PX4LITE_IDLE; }
Px4Lite_Result_t Px4Lite_BaroRead(Px4Lite_SensorBaro_t *out, uint8_t start) { (void)out; (void)start; return PX4LITE_IDLE; }

/** @brief 产生受控平台事实，保留 0 ms 合法采样时间和独立通道身份。 */
static Px4Lite_Result_t TestRead(unsigned channel, Px4Lite_BatteryStatus_t *out)
{
  ++s_reads[channel];
  if (s_read_result[channel] != PX4LITE_OK) { return s_read_result[channel]; }
  memset(out, 0, sizeof(*out));
  out->header.sample_time_ms = s_now;
  out->voltage_mv = 12000U + channel * 500U;
  out->current_ma = 100 + (int32_t)channel * 300;
  out->current_valid = s_current_valid[channel];
  if (s_reference_degraded[channel]) { out->header.flags |= PX4LITE_DATA_DEGRADED; }
  out->percent = 77U;
  return PX4LITE_OK;
}
Px4Lite_Result_t Px4Lite_BatteryRead(Px4Lite_BatteryStatus_t *out) { return TestRead(0U, out); }
Px4Lite_Result_t Px4Lite_Battery2Read(Px4Lite_BatteryStatus_t *out) { return TestRead(1U, out); }
void Px4Lite_PublishBattery(const Px4Lite_BatteryStatus_t *out) { ++s_publishes[0]; s_latest[0] = *out; }
void Px4Lite_PublishBattery2(const Px4Lite_BatteryStatus_t *out) { ++s_publishes[1]; s_latest[1] = *out; }
void Px4Lite_TimeRun(uint32_t now_ms) { (void)now_ms; }
void Px4Lite_GetImuStats(Px4Lite_FifoStats_t *out) { memset(out, 0, sizeof(*out)); }
void Px4Lite_AlarmUpdateFromStatuses(const Px4Lite_ModuleStatus_t *items, uint16_t count, uint32_t now_ms)
{ (void)items; (void)count; (void)now_ms; }
void Px4Lite_PublishHealth(const Px4Lite_SystemHealth_t *out) { s_latest_health = *out; }
void Px4Lite_BatteryRequestReinit(void) { ++s_reinit_requests; }

/** @brief 仅注册本测试的电源模块，使真实恢复监督调用真实轻量 recover。 */
Px4Lite_Result_t Px4Lite_RegistryGet(Px4Lite_ModuleId_t id, Px4Lite_ModuleDescriptor_t *descriptor,
                                    Px4Lite_ModuleRuntime_t *runtime)
{
  (void)runtime;
  if (id != PX4LITE_MODULE_BATTERY) { return PX4LITE_NOT_READY; }
  memset(descriptor, 0, sizeof(*descriptor));
  descriptor->module_id = id;
  descriptor->enabled = PX4LITE_ENABLE_BATTERY;
  descriptor->recover = Px4Lite_BatteryRecover;
  return PX4LITE_OK;
}

/** @brief 执行一拍真实采集，并确认禁用组从未被读或发布。 */
static void TestSensorAt(uint32_t now_ms)
{
  s_now = now_ms;
  Px4Lite_SensorWorkRun(s_now);
  if (!PX4LITE_ENABLE_BATTERY1) { assert(s_reads[0] == 0U && s_publishes[0] == 0U); }
  if (!PX4LITE_ENABLE_BATTERY2) { assert(s_reads[1] == 0U && s_publishes[1] == 0U); }
}

/** @brief 以公开接口读取电源共享模块状态，避免只检查内部事实。 */
static Px4Lite_ModuleStatus_t TestPowerStatus(void)
{
  Px4Lite_ModuleStatus_t status;
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_BATTERY, &status) == PX4LITE_OK);
  return status;
}

int main(void)
{
  Px4Lite_ModuleStatus_t status;
  unsigned target = PX4LITE_ENABLE_BATTERY2 ? 1U : 0U;
  unsigned target_publishes, total_reads;
  uint8_t recoveries;
  assert(Px4Lite_ModulesInit() == PX4LITE_OK);
  TestSensorAt(0U);
  TestSensorAt(10U);
  TestSensorAt(999U);
  assert(s_reads[0] == (PX4LITE_ENABLE_BATTERY1 ? 1U : 0U));
  assert(s_reads[1] == (PX4LITE_ENABLE_BATTERY2 ? 1U : 0U));

  if (!PX4LITE_ENABLE_BATTERY) {
    TestSensorAt(1000U);
    s_now = 6000U; Px4Lite_HealthRun(s_now);
    status = TestPowerStatus();
    assert(status.state == PX4LITE_STATE_DISABLED && status.fault_code == PX4LITE_FAULT_NONE);
    assert(s_reinit_requests == 0U);
    assert((s_latest_health.warning_fault_mask & (1UL << PX4LITE_MODULE_BATTERY)) == 0U);
    puts("single supply service: all disabled passed");
    return 0;
  }

  status = TestPowerStatus();
  assert(status.state == PX4LITE_STATE_ONLINE && status.last_valid_ms == 0U);
  assert(s_latest[target].header.sample_time_ms == 0U && s_latest[target].header.sequence == 1U);
  assert(s_latest[target].header.valid == 1U);
  TestSensorAt(1000U);
  assert(s_reads[target] == 2U && s_publishes[target] == 2U);
  status = TestPowerStatus();
  assert(status.state == PX4LITE_STATE_ONLINE && status.last_valid_ms == 1000U);

  s_read_result[target] = PX4LITE_IO_ERROR;
  target_publishes = s_publishes[target];
  TestSensorAt(2000U);
  status = TestPowerStatus();
  assert(s_publishes[target] == target_publishes && status.error_count == 1U);
  assert(status.state != PX4LITE_STATE_OFFLINE && status.state != PX4LITE_STATE_FAILED);
  if (PX4LITE_ENABLE_BATTERY1 && PX4LITE_ENABLE_BATTERY2) { assert(status.state == PX4LITE_STATE_DEGRADED); }

  s_read_result[target] = PX4LITE_OK;
  s_current_valid[target] = 0U;
  TestSensorAt(3000U);
  status = TestPowerStatus();
  assert(s_publishes[target] == target_publishes + 1U);
  assert(s_latest[target].header.valid == 1U && s_latest[target].voltage_mv == 12000U + target * 500U);
  assert(s_latest[target].current_valid == 0U && s_latest[target].header.quality == 50U);
  assert((s_latest[target].header.flags & PX4LITE_DATA_DEGRADED) != 0U);
  assert(status.state == PX4LITE_STATE_DEGRADED && status.last_valid_ms == 3000U && status.error_count == 2U);

  s_current_valid[target] = 1U;
  TestSensorAt(4000U);
  status = TestPowerStatus();
  assert(status.state == PX4LITE_STATE_ONLINE && status.consecutive_errors == 0U);

  /* 参考回退有电压且电流采集成功，仍应保留降级事实，不能冒充已实测参考。 */
  s_reference_degraded[target] = 1U;
  TestSensorAt(5000U);
  status = TestPowerStatus();
  assert(s_latest[target].header.valid == 1U && s_latest[target].current_valid == 1U);
  assert(s_latest[target].header.quality == 50U && (s_latest[target].header.flags & PX4LITE_DATA_DEGRADED));
  assert(status.state == PX4LITE_STATE_DEGRADED && status.error_count == 3U);
  assert(status.last_valid_ms == 5000U && status.consecutive_errors == 1U);
  s_reference_degraded[target] = 0U;
  TestSensorAt(6000U);
  status = TestPowerStatus();
  assert(status.state == PX4LITE_STATE_ONLINE && status.consecutive_errors == 0U);
  assert(s_latest[target].header.quality == 100U && !(s_latest[target].header.flags & PX4LITE_DATA_DEGRADED));
  recoveries = status.recovery_count;
  total_reads = s_reads[0] + s_reads[1];
  s_now = 6000U + PX4LITE_BATTERY_OFFLINE_MS;
  Px4Lite_HealthRun(s_now);
  assert(TestPowerStatus().state == PX4LITE_STATE_ONLINE);
  ++s_now; Px4Lite_HealthRun(s_now);
  status = TestPowerStatus();
  assert(status.state == PX4LITE_STATE_OFFLINE && status.fault_code == PX4LITE_FAULT_SENSOR_OFFLINE);
  assert(s_reinit_requests == 0U && s_reads[0] + s_reads[1] == total_reads);
  s_now += PX4LITE_RECOVERY_DELAY_MS - 1U; Px4Lite_HealthRun(s_now);
  assert(s_reinit_requests == 1U && s_reads[0] + s_reads[1] == total_reads);
  s_now += 1000U; Px4Lite_HealthRun(s_now);
  assert(s_reinit_requests == 1U && s_reads[0] + s_reads[1] == total_reads);
  TestSensorAt(s_now);
  status = TestPowerStatus();
  assert(status.state == PX4LITE_STATE_ONLINE && status.last_valid_ms == s_now);
  assert(status.recovery_count == (uint8_t)(recoveries + 1U));
  Px4Lite_HealthRun(s_now);
  assert(s_latest_health.module_state[PX4LITE_MODULE_BATTERY] == PX4LITE_STATE_ONLINE);
  assert(s_reinit_requests == 1U);

  /* 计时回绕后仍按完整 1000 ms 节流，不把首次零时间误判成未采样。 */
  s_now = UINT32_MAX - 499U;
  assert(Px4Lite_ModulesInit() == PX4LITE_OK);
  TestSensorAt(s_now);
  total_reads = s_reads[target];
  TestSensorAt(499U);
  assert(s_reads[target] == total_reads);
  TestSensorAt(500U);
  assert(s_reads[target] == total_reads + 1U);

  /* 启动瞬间已有有效样本时，0 ms 也是有效心跳，不能额外套用无样本启动宽限。 */
  s_now = 0U;
  assert(Px4Lite_ModulesInit() == PX4LITE_OK);
  TestSensorAt(0U);
  s_now = PX4LITE_BATTERY_OFFLINE_MS;
  Px4Lite_HealthRun(s_now);
  assert(TestPowerStatus().state == PX4LITE_STATE_ONLINE);
  ++s_now; Px4Lite_HealthRun(s_now);
  assert(TestPowerStatus().state == PX4LITE_STATE_OFFLINE);

  /* 真正从未取得电压样本时继续给初始化宽限，不能把首帧 I/O 失败误作曾在线。 */
  s_now = 0U;
  assert(Px4Lite_ModulesInit() == PX4LITE_OK);
  s_read_result[0] = PX4LITE_IO_ERROR;
  s_read_result[1] = PX4LITE_IO_ERROR;
  TestSensorAt(0U);
  s_now = PX4LITE_BATTERY_OFFLINE_MS + 1U;
  Px4Lite_HealthRun(s_now);
  assert(TestPowerStatus().state == PX4LITE_STATE_UNINITIALIZED);
  s_now = PX4LITE_BATTERY_STARTUP_GRACE_MS + 1U;
  Px4Lite_HealthRun(s_now);
  assert(TestPowerStatus().state == PX4LITE_STATE_OFFLINE);
  printf("single supply service: enable1=%u enable2=%u passed\n",
         (unsigned)PX4LITE_ENABLE_BATTERY1, (unsigned)PX4LITE_ENABLE_BATTERY2);
  return 0;
}
