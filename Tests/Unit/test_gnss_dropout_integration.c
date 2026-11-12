/**
 * @file test_gnss_dropout_integration.c
 * @brief 真实Adapter、Framework采集和Health的GNSS异常在线事实回归。
 * @details Driver快照与I/O结果为独立故障注入；不复制健康策略，不改生产配置。
 * 本测试仅禁用无关硬件采集，保留真实GNSS两秒离线规则和App字段时效。
 */
#include "px4lite_config.h"
#undef PX4LITE_ENABLE_IMU
#undef PX4LITE_ENABLE_BARO
#undef PX4LITE_ENABLE_BATTERY
#undef PX4LITE_ENABLE_LORA
#undef PX4LITE_ENABLE_5G
#undef PX4LITE_ENABLE_DISPLAY
#undef PX4LITE_ENABLE_REMOTE_ID
#undef PX4LITE_ENABLE_CONTROL
#define PX4LITE_ENABLE_IMU 0U
#define PX4LITE_ENABLE_BARO 0U
#define PX4LITE_ENABLE_BATTERY 0U
#define PX4LITE_ENABLE_LORA 0U
#define PX4LITE_ENABLE_5G 0U
#define PX4LITE_ENABLE_DISPLAY 0U
#define PX4LITE_ENABLE_REMOTE_ID 0U
#define PX4LITE_ENABLE_CONTROL 0U

#define main ExistingStatusDomainMain
#include "test_gnss_status_domain.c"
#undef main
#include "../../Framework/Src/px4lite_modules.c"

static uint32_t s_published_gnss,s_health_runs;
static Px4Lite_SystemHealth_t s_last_health;

/** @brief 只保存真实采集服务的topic出口，不修改测量或发布时刻。 */
void Px4Lite_PublishGnss(const Px4Lite_SensorGnss_t *value)
{ s_topic_source=*value;s_published_gnss++; }
/** @brief 计时统计替身不影响采集或健康判断。 */
void Px4Lite_RuntimeReportSensorWork(uint32_t elapsed_us) { (void)elapsed_us; }
/** @brief 授时不是本次被测模块，禁止读取任何新硬件。 */
void Px4Lite_TimeRun(uint32_t now_ms) { assert(now_ms==s_clock); }
/** @brief 隔离通用恢复的外部副作用，使断流判断使用真实原last_rx而非夹具复位。 */
void Px4Lite_RecoveryMonitorRun(uint32_t now_ms) { assert(now_ms==s_clock); }
/** @brief 没有启动IMU任务，仅为真实Health出口提供空FIFO事实。 */
void Px4Lite_GetImuStats(Px4Lite_FifoStats_t *out) { memset(out,0,sizeof(*out)); }
/** @brief 告警出口只核对所有者参数，不回写模块状态。 */
void Px4Lite_AlarmUpdateFromStatuses(const Px4Lite_ModuleStatus_t *items,uint16_t count,uint32_t now_ms)
{ assert(items && count==PX4LITE_MODULE_COUNT && now_ms==s_clock); }
/** @brief 保存真实Health结果，检出OFFLINE是否同时进入健康快照。 */
void Px4Lite_PublishHealth(const Px4Lite_SystemHealth_t *value)
{ s_last_health=*value;s_health_runs++; }

/** @brief 每场景新建独立Framework事实，0ms仍为合法时钟。 */
static void DropoutReset(void)
{
  Reset();s_published_gnss=0U;s_health_runs=0U;memset(&s_last_health,0,sizeof(s_last_health));
  assert(Px4Lite_ModulesInit()==PX4LITE_OK);
}
/** @brief 推进真实sensor和health入口，不在测试内重写它们的判断。 */
static void DropoutStep(uint32_t now_ms)
{
  s_clock=now_ms;s_host_timer.CNT=0U;
  Px4Lite_SensorWorkRun(now_ms);Px4Lite_HealthRun(now_ms);
}
/** @brief 快照与服务返回独立，可表示已解析新帧而同轮TX失败。 */
static void NewNoFix(uint32_t now_ms)
{
  s_driver_source.rx_sequence++;
  if(!s_driver_source.rx_sequence) { s_driver_source.rx_sequence=1U; }
  s_driver_source.last_rx_ms=now_ms;s_driver_source.status_update_ms=now_ms;
  s_driver_source.valid_mask=GNSS_VALID_NAV_STATUS;s_driver_source.fix_quality=0U;
  s_driver_source.satellites=0U;s_snapshot_result=GNSS_RESULT_NO_FIX;
}
/** @brief 连续有效接收不能被TX故障伪装成红色离线，无定位仍必须是DEGRADED。 */
static void TestNewRxSurvivesServiceError(void)
{
  uint32_t now_ms;
  Px4Lite_ModuleStatus_t status;
  DropoutReset();s_service_result=GNSS_RESULT_IO_ERROR;
  for(now_ms=100U;now_ms<=6100U;now_ms+=1000U) {
    NewNoFix(now_ms);DropoutStep(now_ms);
    assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_GNSS,&status)==PX4LITE_OK);
    assert(status.state==PX4LITE_STATE_DEGRADED && status.last_rx_ms==now_ms);
    assert(s_last_health.module_state[PX4LITE_MODULE_GNSS]==PX4LITE_STATE_DEGRADED);
    assert(s_topic_source.last_rx_ms==now_ms && s_topic_source.fix_type==0U);
    assert(s_topic_source.valid_mask==PX4LITE_GNSS_VALID_NAV_STATUS);
    assert(s_topic_source.header.sample_time_ms==now_ms);
  }
  assert(s_published_gnss==7U && s_health_runs==7U);
}
/** @brief 没有新rx_sequence的旧快照即使反复IOERROR也不能续命，原2s门限仍生效。 */
static void TestRealOfflineWithoutNewRx(void)
{
  uint32_t before;
  Px4Lite_ModuleStatus_t status;
  DropoutReset();NewNoFix(100U);DropoutStep(100U);before=s_published_gnss;
  s_service_result=GNSS_RESULT_IO_ERROR;
  DropoutStep(2100U);assert(s_published_gnss==before);
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_GNSS,&status)==PX4LITE_OK);
  assert(status.last_rx_ms==100U && status.state==PX4LITE_STATE_DEGRADED);
  DropoutStep(2101U);assert(s_published_gnss==before);
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_GNSS,&status)==PX4LITE_OK);
  assert(status.last_rx_ms==100U && status.state==PX4LITE_STATE_OFFLINE);
  assert(s_last_health.module_state[PX4LITE_MODULE_GNSS]==PX4LITE_STATE_OFFLINE);
  NewNoFix(2200U);DropoutStep(2200U);
  assert(Px4Lite_GetModuleStatus(PX4LITE_MODULE_GNSS,&status)==PX4LITE_OK);
  assert(status.state==PX4LITE_STATE_DEGRADED && status.last_rx_ms==2200U && s_published_gnss==before+1U);
}
/** @brief 只有ACK等新链路事实不能续旧坐标/UTC/定位状态，App仍按独立时刻撤权。 */
static void TestOldMeasurementsNotResurrected(void)
{
  App_GnssDetails_t details;
  DropoutReset();s_clock=6000U;s_service_result=GNSS_RESULT_IO_ERROR;
  s_driver_source.rx_sequence=7U;s_driver_source.last_rx_ms=6000U;
  s_driver_source.valid_mask=GNSS_VALID_POSITION|GNSS_VALID_TIME|GNSS_VALID_NAV_STATUS;
  s_driver_source.position_update_ms=100U;s_driver_source.time_update_ms=100U;s_driver_source.status_update_ms=100U;
  s_driver_source.latitude_deg_e7=123456;s_driver_source.fix_valid=1U;s_driver_source.fix_quality=1U;
  DropoutStep(6000U);assert(s_published_gnss==1U && s_topic_source.fix_type==0U);
  assert(App_CopyGnssDetails(&details,6000U)==PX4LITE_OK);
  assert(details.valid_mask==0U && details.last_rx_ms==6000U);
  assert(details.position_update_ms==100U && details.time_update_ms==100U && details.status_update_ms==100U);
}
/** @brief 新序号也不能把明确失败的快照当有效接收事实；错误和断流仍保持可见。 */
static void TestSnapshotErrorsDoNotPublish(void)
{
  DropoutReset();NewNoFix(100U);s_snapshot_result=GNSS_RESULT_INVALID_PARAM;
  s_service_result=GNSS_RESULT_IO_ERROR;DropoutStep(100U);assert(!s_published_gnss);
  s_snapshot_result=GNSS_RESULT_NO_DATA;DropoutStep(200U);assert(!s_published_gnss);
}

/** @brief 执行跨层异常/时效回归，不能用通过结果代替实板接线与长稳验证。 */
int main(void)
{
  TestNewRxSurvivesServiceError();TestRealOfflineWithoutNewRx();
  TestOldMeasurementsNotResurrected();TestSnapshotErrorsDoNotPublish();
  puts("GNSS dropout integration: real Adapter/SensorWork/Health new RX+IO error / real 2s offline / stale fields / invalid snapshot PASS");
  return 0;
}
