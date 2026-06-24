/**
 * @file px4lite_modules.c
 * @brief Framework 测量发布、导航转换、通信调度和健康状态实现。
 *
 * @details
 * 本文件承载 sensor、estimator、comm 和 health 周期任务的核心服务。普通传感器
 * 统一由 sensor 任务采集；Health task 独占 OFFLINE/FAILED 判定；状态复制通过
 * 版本号保持一致性。
 */

#include "px4lite_modules.h"
#include "px4lite_alarm.h"
#include "px4lite_attitude.h"
#include "px4lite_config.h"
#include "px4lite_faults.h"
#include "px4lite_platform.h"
#include "px4lite_recovery.h"
#include "px4lite_time.h"
#include "px4lite_topics.h"
#include "px4lite_mavlink_tx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static Px4Lite_ModuleStatus_t s_status[PX4LITE_MODULE_COUNT];
static uint32_t s_status_version;
static uint32_t s_gnss_sequence;
static uint32_t s_imu_sequence;
static uint32_t s_baro_sequence;
static uint32_t s_battery_sequence;
static uint32_t s_navigation_sequence;
static uint32_t s_health_sequence;
static uint32_t s_start_ms;
static uint32_t s_last_imu_work_ms;
static uint32_t s_last_baro_work_ms;
static uint32_t s_last_battery_work_ms;
static Px4Lite_AttitudeState_t s_attitude_state;

/**
 * @brief 根据定位类型、卫星数和 HDOP 计算有限范围 GNSS 质量分。
 *
 * @param[in] gnss GNSS 测量快照，不能为 NULL。
 *
 * @return 质量分，范围 0 到 100。
 */
static uint8_t Px4Lite_GnssQuality(const Px4Lite_SensorGnss_t *gnss)
{
  uint8_t quality;

  if (gnss->fix_type == 0U) { return ((gnss->gps_visible + gnss->bds_visible) != 0U) ? 20U : 5U; }

  quality = 100U;
  if (gnss->satellites_used < 8U) { quality = (uint8_t)(quality - 20U); }
  if (gnss->hdop_x100 > 200U) {
    quality = (uint8_t)(quality - 30U);
  } else if (gnss->hdop_x100 > 100U) {
    quality = (uint8_t)(quality - 10U);
  }
  return quality;
}

/**
 * @brief 将 GGA/GSA 定位事实映射为显示侧 GNSS fix 状态。
 *
 * @param[in] gnss GNSS 测量快照，不能为 NULL。
 *
 * @return 显示侧 fix 状态值。
 */
static uint8_t Px4Lite_GnssDisplayFixState(const Px4Lite_SensorGnss_t *gnss)
{
  if ((gnss->fix_type == 0U) || (gnss->fix_dimension == 1U)) { return 0U; }

  if (gnss->fix_type == 2U) { return 4U; }

  if (gnss->fix_dimension == 2U) { return 2U; }

  if (gnss->fix_dimension == 3U) { return 3U; }

  return (gnss->satellites_used >= 4U) ? 3U : 2U;
}

/**
 * @brief 查询 Framework 模块是否被配置启用。
 *
 * @param[in] id 模块编号。
 *
 * @return 1 表示启用，0 表示关闭或模块编号非法。
 */
static uint8_t Px4Lite_IsModuleEnabled(Px4Lite_ModuleId_t id)
{
  switch (id) {
    case PX4LITE_MODULE_GNSS:
      return PX4LITE_ENABLE_GNSS;
    case PX4LITE_MODULE_IMU:
      return PX4LITE_ENABLE_IMU;
    case PX4LITE_MODULE_BARO:
      return PX4LITE_ENABLE_BARO;
    case PX4LITE_MODULE_BATTERY:
      return PX4LITE_ENABLE_BATTERY;
    case PX4LITE_MODULE_LORA:
      return PX4LITE_ENABLE_LORA;
    case PX4LITE_MODULE_5G:
      return PX4LITE_ENABLE_5G;
    case PX4LITE_MODULE_STORAGE:
      return PX4LITE_ENABLE_STORAGE;
    case PX4LITE_MODULE_REMOTE_ID:
      return PX4LITE_ENABLE_REMOTE_ID;
    case PX4LITE_MODULE_DISPLAY:
      return PX4LITE_ENABLE_DISPLAY;
    case PX4LITE_MODULE_CONTROL:
      return PX4LITE_ENABLE_CONTROL;
    case PX4LITE_MODULE_ALARM:
      return PX4LITE_ENABLE_ALARM;
    case PX4LITE_MODULE_SYSTEM:
    case PX4LITE_MODULE_ESTIMATOR:
    case PX4LITE_MODULE_BUSINESS:
      return 1U;
    default:
      return 0U;
  }
}

/**
 * @brief 将模块公开状态映射为默认告警严重度。
 *
 * @param[in] state 模块公开状态。
 *
 * @return 默认告警严重度。
 */
static uint8_t Px4Lite_StateSeverity(Px4Lite_State_t state)
{
  if (state == PX4LITE_STATE_DEGRADED) { return (uint8_t)PX4LITE_SEVERITY_WARNING; }
  if (state == PX4LITE_STATE_OFFLINE) { return (uint8_t)PX4LITE_SEVERITY_ERROR; }
  if (state == PX4LITE_STATE_FAILED) { return (uint8_t)PX4LITE_SEVERITY_CRITICAL; }
  return (uint8_t)PX4LITE_SEVERITY_INFO;
}

/**
 * @brief 更新单个模块状态，并在发生变化时推进共享状态版本号。
 *
 * @param[in] id 模块编号。
 * @param[in] state 新的公开状态。
 * @param[in] fault 故障码，0 表示无故障。
 * @param[in] now_ms 当前系统毫秒时间。
 */
static void Px4Lite_SetStatus(Px4Lite_ModuleId_t id, Px4Lite_State_t state, uint16_t fault, uint32_t now_ms)
{
  Px4Lite_ModuleStatus_t *item;
  uint8_t changed = 0U;

  if ((uint32_t)id >= (uint32_t)PX4LITE_MODULE_COUNT) { return; }

  taskENTER_CRITICAL();
  item = &s_status[id];
  if (item->state != state) {
    if (((item->state == PX4LITE_STATE_DEGRADED) || (item->state == PX4LITE_STATE_OFFLINE) || (item->state == PX4LITE_STATE_FAILED)) && (state == PX4LITE_STATE_ONLINE)) {
      if (item->recovery_count < 255U) { item->recovery_count++; }
    }
    item->state          = state;
    item->state_since_ms = now_ms;
    changed              = 1U;
  }
  if (item->fault_code != fault) {
    item->fault_code = fault;
    changed          = 1U;
  }
  if (item->severity != Px4Lite_StateSeverity(state)) {
    item->severity = Px4Lite_StateSeverity(state);
    changed        = 1U;
  }
  if (changed != 0U) { s_status_version++; }
  taskEXIT_CRITICAL();
}

/**
 * @brief 记录传感器 I/O 错误事实，但不直接写 OFFLINE/FAILED 状态。
 *
 * @param[in] id 模块编号。
 * @param[in] now_ms 当前系统毫秒时间，当前仅用于保持接口语义。
 */
static void Px4Lite_RecordSensorIoError(Px4Lite_ModuleId_t id, uint32_t now_ms)
{
  if ((uint32_t)id >= (uint32_t)PX4LITE_MODULE_COUNT) { return; }

  /*
     * 这里只记录硬件事实（错误计数）。公开 ONLINE/OFFLINE 状态的单写者是 Health，
     * 它根据模块超时判断 OFFLINE，避免一次瞬时 I/O 抖动导致显示反复清空。
     */
  (void)now_ms;

  taskENTER_CRITICAL();
  s_status[id].error_count++;
  s_status[id].consecutive_valid = 0U;
  if (s_status[id].consecutive_errors < 65535U) { s_status[id].consecutive_errors++; }
  s_status_version++;
  taskEXIT_CRITICAL();
}

/**
 * @brief 复位 Framework 模块状态、统计计数和启动时间。
 */
Px4Lite_Result_t Px4Lite_ModulesInit(void)
{
  uint32_t i;

  memset(s_status, 0, sizeof(s_status));
  for (i = 0U; i < (uint32_t)PX4LITE_MODULE_COUNT; ++i) {
    s_status[i].module_id = (Px4Lite_ModuleId_t)i;
    s_status[i].state     = (Px4Lite_IsModuleEnabled((Px4Lite_ModuleId_t)i) != 0U) ? PX4LITE_STATE_UNINITIALIZED : PX4LITE_STATE_DISABLED;
  }

  s_status_version       = 1U;
  s_gnss_sequence        = 0U;
  s_imu_sequence         = 0U;
  s_baro_sequence        = 0U;
  s_battery_sequence     = 0U;
  s_navigation_sequence  = 0U;
  s_health_sequence      = 0U;
  s_last_imu_work_ms     = 0U;
  s_last_baro_work_ms    = 0U;
  s_last_battery_work_ms = 0U;
  s_start_ms             = Px4Lite_PlatformGetMs();
  return PX4LITE_OK;
}

/**
 * @brief 根据初始化结果设置一个传感器模块的初始 Framework 状态。
 */
static void Px4Lite_PublishInitState(Px4Lite_ModuleId_t id, Px4Lite_Result_t init_result, uint32_t now_ms)
{
  Px4Lite_SetStatus(id, (init_result == PX4LITE_OK) ? PX4LITE_STATE_STARTING : PX4LITE_STATE_FAILED, (init_result == PX4LITE_OK) ? 0U : PX4LITE_FAULT_SENSOR_INIT, now_ms);
}

/**
 * @brief 注册表 init 回调：启动 GNSS 设备并发布初始状态。
 */
Px4Lite_Result_t Px4Lite_GnssModuleInit(void)
{
#if PX4LITE_ENABLE_GNSS
  Px4Lite_PublishInitState(PX4LITE_MODULE_GNSS, Px4Lite_GnssInit(), Px4Lite_PlatformGetMs());
#endif
  return PX4LITE_OK;
}

/**
 * @brief 注册表 init 回调：启动 IMU 设备并发布初始状态。
 */
Px4Lite_Result_t Px4Lite_ImuModuleInit(void)
{
#if PX4LITE_ENABLE_IMU
  Px4Lite_PublishInitState(PX4LITE_MODULE_IMU, Px4Lite_ImuInit(), Px4Lite_PlatformGetMs());
#endif
  return PX4LITE_OK;
}

/**
 * @brief 注册表 init 回调：启动气压计设备并发布初始状态。
 */
Px4Lite_Result_t Px4Lite_BaroModuleInit(void)
{
#if PX4LITE_ENABLE_BARO
  Px4Lite_PublishInitState(PX4LITE_MODULE_BARO, Px4Lite_BaroInit(), Px4Lite_PlatformGetMs());
#endif
  return PX4LITE_OK;
}

/**
 * @brief 注册表 init 回调：启动电源/ADC 设备并发布初始状态。
 */
Px4Lite_Result_t Px4Lite_BatteryModuleInit(void)
{
#if PX4LITE_ENABLE_BATTERY
  Px4Lite_PublishInitState(PX4LITE_MODULE_BATTERY, Px4Lite_BatteryInit(), Px4Lite_PlatformGetMs());
#endif
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_AlarmModuleInit(void)
{
#if PX4LITE_ENABLE_ALARM
  uint32_t now_ms         = Px4Lite_PlatformGetMs();
  Px4Lite_Result_t result = Px4Lite_AlarmInit(now_ms);

  Px4Lite_SetStatus(PX4LITE_MODULE_ALARM, (result == PX4LITE_OK) ? PX4LITE_STATE_ONLINE : PX4LITE_STATE_FAILED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SYSTEM_SELF_CHECK, now_ms);
  return result;
#else
  return PX4LITE_OK;
#endif
}

/*
 * recover 回调由 Health 侧恢复监视器调用。它们必须保持轻量，只请求所属模块的
 * Service 在本任务中执行 re-init，不能在 Health 任务里访问共享总线。
 */
Px4Lite_Result_t Px4Lite_GnssRecover(void)
{
#if PX4LITE_ENABLE_GNSS
  Px4Lite_GnssRequestReinit();
#endif
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_ImuRecover(void)
{
#if PX4LITE_ENABLE_IMU
  Px4Lite_ImuRequestReinit();
#endif
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_BaroRecover(void)
{
#if PX4LITE_ENABLE_BARO
  Px4Lite_BaroRequestReinit();
#endif
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_BatteryRecover(void)
{
#if PX4LITE_ENABLE_BATTERY
  Px4Lite_BatteryRequestReinit();
#endif
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_LoraRecover(void)
{
#if PX4LITE_ENABLE_LORA
  Px4Lite_LoRaRequestReinit();
#endif
  return PX4LITE_OK;
}

/**
 * @brief 执行一次非阻塞传感器采集和发布周期。
 */
void Px4Lite_SensorWorkRun(uint32_t now_ms)
{
#if PX4LITE_ENABLE_GNSS
  Px4Lite_SensorGnss_t gnss;
  Px4Lite_Result_t result;

  memset(&gnss, 0, sizeof(gnss));
  result = Px4Lite_GnssRead(&gnss);

  if (result == PX4LITE_OK) {
    gnss.header.sample_time_ms  = (gnss.header.sample_time_ms != 0U) ? gnss.header.sample_time_ms : now_ms;
    gnss.header.publish_time_ms = now_ms;
    gnss.header.sequence        = ++s_gnss_sequence;
    gnss.header.device_id       = (uint16_t)PX4LITE_MODULE_GNSS;
    gnss.header.valid           = 1U;
    gnss.header.quality         = Px4Lite_GnssQuality(&gnss);
    gnss.header.flags           = PX4LITE_DATA_VALID;
    if (gnss.fix_type == 0U) { gnss.header.flags |= PX4LITE_DATA_DEGRADED; }
    Px4Lite_PublishGnss(&gnss);

    taskENTER_CRITICAL();
    s_status[PX4LITE_MODULE_GNSS].last_rx_ms         = gnss.header.sample_time_ms;
    s_status[PX4LITE_MODULE_GNSS].consecutive_errors = 0U;
    if (s_status[PX4LITE_MODULE_GNSS].consecutive_valid < 65535U) { s_status[PX4LITE_MODULE_GNSS].consecutive_valid++; }
    if (gnss.fix_type != 0U) { s_status[PX4LITE_MODULE_GNSS].last_valid_ms = gnss.header.sample_time_ms; }
    s_status_version++;
    taskEXIT_CRITICAL();

    Px4Lite_SetStatus(PX4LITE_MODULE_GNSS, (gnss.fix_type != 0U) ? PX4LITE_STATE_ONLINE : PX4LITE_STATE_DEGRADED, (gnss.fix_type != 0U) ? 0U : PX4LITE_FAULT_SENSOR_NO_FIX, now_ms);
  } else if (result == PX4LITE_IO_ERROR) {
    Px4Lite_RecordSensorIoError(PX4LITE_MODULE_GNSS, now_ms);
  }
#endif

#if PX4LITE_ENABLE_IMU
  if ((s_last_imu_work_ms == 0U) || ((uint32_t)(now_ms - s_last_imu_work_ms) >= PX4LITE_IMU_WORK_PERIOD_MS)) {
    Px4Lite_SensorImu_t imu;
    Px4Lite_Result_t result;

    s_last_imu_work_ms = now_ms;
    memset(&imu, 0, sizeof(imu));
    result = Px4Lite_ImuRead(&imu);
    if (result == PX4LITE_OK) {
      imu.header.sample_time_ms  = (imu.header.sample_time_ms != 0U) ? imu.header.sample_time_ms : now_ms;
      imu.header.publish_time_ms = now_ms;
      imu.header.sequence        = ++s_imu_sequence;
      imu.header.device_id       = (uint16_t)PX4LITE_MODULE_IMU;
      imu.header.valid           = 1U;
      imu.header.quality         = 100U;
      imu.header.flags           = PX4LITE_DATA_VALID;
      (void)Px4Lite_PushImu(&imu);

      taskENTER_CRITICAL();
      s_status[PX4LITE_MODULE_IMU].last_rx_ms         = imu.header.sample_time_ms;
      s_status[PX4LITE_MODULE_IMU].last_valid_ms      = imu.header.sample_time_ms;
      s_status[PX4LITE_MODULE_IMU].consecutive_errors = 0U;
      if (s_status[PX4LITE_MODULE_IMU].consecutive_valid < 65535U) { s_status[PX4LITE_MODULE_IMU].consecutive_valid++; }
      s_status_version++;
      taskEXIT_CRITICAL();

      Px4Lite_SetStatus(PX4LITE_MODULE_IMU, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, now_ms);
    } else if (result == PX4LITE_IO_ERROR) {
      Px4Lite_RecordSensorIoError(PX4LITE_MODULE_IMU, now_ms);
    }
  }
#endif

#if PX4LITE_ENABLE_BARO
  if ((s_last_baro_work_ms == 0U) || ((uint32_t)(now_ms - s_last_baro_work_ms) >= PX4LITE_BARO_WORK_PERIOD_MS)) {
    Px4Lite_SensorBaro_t baro;
    Px4Lite_Result_t result;

    s_last_baro_work_ms = now_ms;
    memset(&baro, 0, sizeof(baro));
    result = Px4Lite_BaroRead(&baro);
    if (result == PX4LITE_OK) {
      baro.header.sample_time_ms  = (baro.header.sample_time_ms != 0U) ? baro.header.sample_time_ms : now_ms;
      baro.header.publish_time_ms = now_ms;
      baro.header.sequence        = ++s_baro_sequence;
      baro.header.device_id       = (uint16_t)PX4LITE_MODULE_BARO;
      baro.header.valid           = 1U;
      baro.header.quality         = 100U;
      baro.header.flags           = PX4LITE_DATA_VALID;
      Px4Lite_PublishBaro(&baro);

      taskENTER_CRITICAL();
      s_status[PX4LITE_MODULE_BARO].last_rx_ms         = baro.header.sample_time_ms;
      s_status[PX4LITE_MODULE_BARO].last_valid_ms      = baro.header.sample_time_ms;
      s_status[PX4LITE_MODULE_BARO].consecutive_errors = 0U;
      if (s_status[PX4LITE_MODULE_BARO].consecutive_valid < 65535U) { s_status[PX4LITE_MODULE_BARO].consecutive_valid++; }
      s_status_version++;
      taskEXIT_CRITICAL();

      Px4Lite_SetStatus(PX4LITE_MODULE_BARO, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, now_ms);
    } else if (result == PX4LITE_IO_ERROR) {
      Px4Lite_RecordSensorIoError(PX4LITE_MODULE_BARO, now_ms);
    }
  }
#endif

#if PX4LITE_ENABLE_BATTERY
  if ((s_last_battery_work_ms == 0U) || ((uint32_t)(now_ms - s_last_battery_work_ms) >= PX4LITE_BATTERY_WORK_PERIOD_MS)) {
    Px4Lite_BatteryStatus_t battery;
    Px4Lite_Result_t result;

    s_last_battery_work_ms = now_ms;
    memset(&battery, 0, sizeof(battery));
    result = Px4Lite_BatteryRead(&battery);
    if (result == PX4LITE_OK) {
      battery.header.sample_time_ms  = (battery.header.sample_time_ms != 0U) ? battery.header.sample_time_ms : now_ms;
      battery.header.publish_time_ms = now_ms;
      battery.header.sequence        = ++s_battery_sequence;
      battery.header.device_id       = (uint16_t)PX4LITE_MODULE_BATTERY;
      battery.header.valid           = 1U;
      battery.header.quality         = (battery.low_voltage != 0U) ? 50U : 100U;
      battery.header.flags           = PX4LITE_DATA_VALID;
      if (battery.low_voltage != 0U) { battery.header.flags |= PX4LITE_DATA_DEGRADED; }
      Px4Lite_PublishBattery(&battery);

      taskENTER_CRITICAL();
      s_status[PX4LITE_MODULE_BATTERY].last_rx_ms         = battery.header.sample_time_ms;
      s_status[PX4LITE_MODULE_BATTERY].last_valid_ms      = battery.header.sample_time_ms;
      s_status[PX4LITE_MODULE_BATTERY].consecutive_errors = 0U;
      if (s_status[PX4LITE_MODULE_BATTERY].consecutive_valid < 65535U) { s_status[PX4LITE_MODULE_BATTERY].consecutive_valid++; }
      s_status_version++;
      taskEXIT_CRITICAL();

      Px4Lite_SetStatus(PX4LITE_MODULE_BATTERY, (battery.low_voltage != 0U) ? PX4LITE_STATE_DEGRADED : PX4LITE_STATE_ONLINE, (battery.low_voltage != 0U) ? PX4LITE_FAULT_SENSOR_INVALID : PX4LITE_FAULT_NONE, now_ms);
    } else if (result == PX4LITE_IO_ERROR) {
      Px4Lite_RecordSensorIoError(PX4LITE_MODULE_BATTERY, now_ms);
    }
  }
#endif

#if !PX4LITE_ENABLE_GNSS && !PX4LITE_ENABLE_IMU && !PX4LITE_ENABLE_BARO && !PX4LITE_ENABLE_BATTERY
  (void)now_ms;
#endif
}

/**
 * @brief 初始化估计器模块状态。
 */
Px4Lite_Result_t Px4Lite_EstimatorInit(void)
{
  Px4Lite_AttitudeInit(&s_attitude_state);
  Px4Lite_SetStatus(PX4LITE_MODULE_ESTIMATOR, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, Px4Lite_PlatformGetMs());
  return PX4LITE_OK;
}

/**
 * @brief 将新鲜 GNSS 测量转换为 Navigation 快照。
 */
void Px4Lite_EstimatorRun(uint32_t now_ms)
{
  static uint32_t last_gnss_sequence;
  Px4Lite_SensorGnss_t gnss;
  Px4Lite_SensorImu_t imu;
  Px4Lite_VehicleNavigation_t navigation;
  uint8_t publish_navigation = 0U;
  uint8_t attitude_updated   = 0U;
  uint8_t imu_pop_count      = 0U;
  int32_t roll_deg100        = 0;
  int32_t pitch_deg100       = 0;
  int32_t yaw_deg100         = 0;
  int32_t roll_rate_dps100   = 0;
  int32_t pitch_rate_dps100  = 0;
  int32_t yaw_rate_dps100    = 0;

  memset(&navigation, 0, sizeof(navigation));

  if ((Px4Lite_CopyGnss(&gnss) == PX4LITE_OK) && (Px4Lite_IsFresh(&gnss.header, now_ms, PX4LITE_GNSS_MAX_AGE_MS) != 0U)) {
    navigation.latitude_e7        = gnss.latitude_e7;
    navigation.longitude_e7       = gnss.longitude_e7;
    navigation.fused_altitude_mm  = gnss.altitude_mm;
    navigation.gnss_utc_sec       = gnss.utc_sec;
    navigation.gnss_utc_date      = gnss.utc_date;
    navigation.hdop_x100          = gnss.hdop_x100;
    navigation.satellites_used    = gnss.satellites_used;
    navigation.gnss_fix_type      = Px4Lite_GnssDisplayFixState(&gnss);
    navigation.navigation_quality = gnss.header.quality;

    if (gnss.fix_type != 0U) {
      navigation.valid_mask = PX4LITE_NAV_VALID_POSITION | PX4LITE_NAV_VALID_ALTITUDE;
    } else {
      navigation.header.flags |= PX4LITE_DATA_DEGRADED;
    }

    navigation.header.sample_time_ms = gnss.header.sample_time_ms;
    if (gnss.header.sequence != last_gnss_sequence) {
      last_gnss_sequence = gnss.header.sequence;
      publish_navigation = 1U;
    }
  }

  while ((imu_pop_count < 8U) && (Px4Lite_PopImu(&imu) == PX4LITE_OK)) {
    imu_pop_count++;
    if ((imu.header.valid != 0U) && (Px4Lite_IsFresh(&imu.header, now_ms, PX4LITE_IMU_MAX_AGE_MS) != 0U) && (Px4Lite_AttitudeUpdate(&s_attitude_state, &imu, &roll_deg100, &pitch_deg100, &yaw_deg100) == PX4LITE_OK)) {
      attitude_updated                 = 1U;
      roll_rate_dps100                 = imu.gyro_mdps[0] / 10;
      pitch_rate_dps100                = imu.gyro_mdps[1] / 10;
      yaw_rate_dps100                  = imu.gyro_mdps[2] / 10;
      navigation.header.sample_time_ms = imu.header.sample_time_ms;
    }
  }

  if (attitude_updated != 0U) {
    navigation.roll_deg100       = roll_deg100;
    navigation.pitch_deg100      = pitch_deg100;
    navigation.yaw_deg100        = yaw_deg100;
    navigation.roll_rate_dps100  = roll_rate_dps100;
    navigation.pitch_rate_dps100 = pitch_rate_dps100;
    navigation.yaw_rate_dps100   = yaw_rate_dps100;
    navigation.valid_mask |= PX4LITE_NAV_VALID_ATTITUDE;
    navigation.valid_mask |= PX4LITE_NAV_VALID_YAW_REL;
    navigation.header.flags |= PX4LITE_DATA_FILTERED;
    publish_navigation = 1U;
  }

  if (publish_navigation == 0U) { return; }

  if (navigation.header.sample_time_ms == 0U) { navigation.header.sample_time_ms = now_ms; }
  navigation.header.publish_time_ms = now_ms;
  navigation.header.sequence        = ++s_navigation_sequence;
  navigation.header.device_id       = 0x0100U;
  navigation.header.valid           = 1U;
  navigation.header.flags |= PX4LITE_DATA_VALID;

  Px4Lite_PublishNavigation(&navigation);
}

/**
 * @brief 初始化健康监控服务。
 */
Px4Lite_Result_t Px4Lite_HealthInit(void)
{
  return PX4LITE_OK;
}

/**
 * @brief 评估模块超时并发布一致的系统健康快照。
 */
void Px4Lite_HealthRun(uint32_t now_ms)
{
  Px4Lite_SystemHealth_t health;
  Px4Lite_ModuleStatus_t status[PX4LITE_MODULE_COUNT];
  Px4Lite_FifoStats_t imu_stats;
  Px4Lite_State_t gnss_state;
  Px4Lite_State_t imu_state;
  Px4Lite_State_t baro_state;
  Px4Lite_State_t battery_state;
  Px4Lite_State_t lora_state;
  Px4Lite_State_t display_state;
  uint32_t i;
  uint32_t last_rx_ms;
  uint32_t imu_last_rx_ms;
  uint32_t baro_last_rx_ms;
  uint32_t battery_last_rx_ms;
  uint32_t display_last_valid_ms;
  uint32_t status_version;

  taskENTER_CRITICAL();
  last_rx_ms            = s_status[PX4LITE_MODULE_GNSS].last_rx_ms;
  gnss_state            = s_status[PX4LITE_MODULE_GNSS].state;
  imu_last_rx_ms        = s_status[PX4LITE_MODULE_IMU].last_rx_ms;
  imu_state             = s_status[PX4LITE_MODULE_IMU].state;
  baro_last_rx_ms       = s_status[PX4LITE_MODULE_BARO].last_rx_ms;
  baro_state            = s_status[PX4LITE_MODULE_BARO].state;
  battery_last_rx_ms    = s_status[PX4LITE_MODULE_BATTERY].last_rx_ms;
  battery_state         = s_status[PX4LITE_MODULE_BATTERY].state;
  lora_state            = s_status[PX4LITE_MODULE_LORA].state;
  display_last_valid_ms = s_status[PX4LITE_MODULE_DISPLAY].last_valid_ms;
  display_state         = s_status[PX4LITE_MODULE_DISPLAY].state;
  taskEXIT_CRITICAL();

#if PX4LITE_ENABLE_GNSS
  if ((gnss_state != PX4LITE_STATE_FAILED) && (Px4Lite_ElapsedMs(now_ms, last_rx_ms) > PX4LITE_GNSS_OFFLINE_MS)) {
    if ((last_rx_ms != 0U) || (Px4Lite_ElapsedMs(now_ms, s_start_ms) > PX4LITE_GNSS_STARTUP_GRACE_MS)) { Px4Lite_SetStatus(PX4LITE_MODULE_GNSS, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_SENSOR_OFFLINE, now_ms); }
  }
#else
  (void)gnss_state;
  (void)last_rx_ms;
#endif

#if PX4LITE_ENABLE_IMU
  if ((imu_state != PX4LITE_STATE_FAILED) && (Px4Lite_ElapsedMs(now_ms, imu_last_rx_ms) > PX4LITE_IMU_OFFLINE_MS)) {
    if ((imu_last_rx_ms != 0U) || (Px4Lite_ElapsedMs(now_ms, s_start_ms) > PX4LITE_IMU_STARTUP_GRACE_MS)) { Px4Lite_SetStatus(PX4LITE_MODULE_IMU, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_SENSOR_OFFLINE, now_ms); }
  }
#else
  (void)imu_state;
  (void)imu_last_rx_ms;
#endif

#if PX4LITE_ENABLE_BARO
  if ((baro_state != PX4LITE_STATE_FAILED) && (Px4Lite_ElapsedMs(now_ms, baro_last_rx_ms) > PX4LITE_BARO_OFFLINE_MS)) {
    if ((baro_last_rx_ms != 0U) || (Px4Lite_ElapsedMs(now_ms, s_start_ms) > PX4LITE_BARO_STARTUP_GRACE_MS)) { Px4Lite_SetStatus(PX4LITE_MODULE_BARO, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_SENSOR_OFFLINE, now_ms); }
  }
#else
  (void)baro_state;
  (void)baro_last_rx_ms;
#endif

#if PX4LITE_ENABLE_BATTERY
  if ((battery_state != PX4LITE_STATE_FAILED) && (Px4Lite_ElapsedMs(now_ms, battery_last_rx_ms) > PX4LITE_BATTERY_OFFLINE_MS)) {
    if ((battery_last_rx_ms != 0U) || (Px4Lite_ElapsedMs(now_ms, s_start_ms) > PX4LITE_BATTERY_STARTUP_GRACE_MS)) { Px4Lite_SetStatus(PX4LITE_MODULE_BATTERY, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_SENSOR_OFFLINE, now_ms); }
  }
#else
  (void)battery_state;
  (void)battery_last_rx_ms;
#endif

#if PX4LITE_ENABLE_LORA
  /*
   * LoRa has two separate facts: local module availability and peer RX
   * freshness. Peer timeout is a degraded link, not a local OFFLINE module.
   * CommWorkRun owns the peer freshness state; TX does not prove peer online.
   */
  (void)lora_state;
#else
  (void)lora_state;
#endif

#if PX4LITE_ENABLE_DISPLAY
  if ((display_state != PX4LITE_STATE_FAILED) && (Px4Lite_ElapsedMs(now_ms, display_last_valid_ms) > PX4LITE_DISPLAY_OFFLINE_MS)) {
    if ((display_last_valid_ms != 0U) || (Px4Lite_ElapsedMs(now_ms, s_start_ms) > PX4LITE_DISPLAY_STARTUP_GRACE_MS)) { Px4Lite_SetStatus(PX4LITE_MODULE_DISPLAY, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_DISPLAY_OFFLINE, now_ms); }
  }
#else
  (void)display_state;
  (void)display_last_valid_ms;
#endif

  Px4Lite_TimeRun(now_ms);
  Px4Lite_RecoveryMonitorRun(now_ms);

  memset(&health, 0, sizeof(health));
  health.header.sample_time_ms  = now_ms;
  health.header.publish_time_ms = now_ms;
  health.header.sequence        = ++s_health_sequence;
  health.header.device_id       = 0x0101U;
  health.header.valid           = 1U;
  health.header.flags           = PX4LITE_DATA_VALID;

  if (Px4Lite_CopyModuleStatuses(status, PX4LITE_MODULE_COUNT, &status_version) != PX4LITE_OK) { return; }

  for (i = 0U; i < (uint32_t)PX4LITE_MODULE_COUNT; ++i) {
    health.module_state[i] = status[i].state;
    if (status[i].state == PX4LITE_STATE_FAILED) {
      health.blocking_fault_mask |= (1UL << i);
    } else if ((status[i].state == PX4LITE_STATE_DEGRADED) || (status[i].state == PX4LITE_STATE_OFFLINE)) {
      health.warning_fault_mask |= (1UL << i);
    } else if ((status[i].state == PX4LITE_STATE_UNINITIALIZED) || (status[i].state == PX4LITE_STATE_STARTING)) {
      health.not_ready_mask |= (1UL << i);
    }
  }
  health.status_version = status_version;
  Px4Lite_GetImuStats(&imu_stats);
  health.imu_fifo_peak     = imu_stats.peak;
  health.imu_fifo_overflow = imu_stats.overflow_count;

  Px4Lite_AlarmUpdateFromStatuses(status, (uint16_t)PX4LITE_MODULE_COUNT, now_ms);

  Px4Lite_PublishHealth(&health);
  Px4Lite_PlatformWatchdogFeed(now_ms);
}

/**
 * @brief 在 Framework 同步保护下复制单个模块状态。
 */
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleStatus_t *status)
{
  if ((status == 0) || ((uint32_t)module_id >= (uint32_t)PX4LITE_MODULE_COUNT)) { return PX4LITE_INVALID_PARAM; }

  taskENTER_CRITICAL();
  *status = s_status[module_id];
  taskEXIT_CRITICAL();
  return PX4LITE_OK;
}

/**
 * @brief 复制版本一致的模块状态数组。
 */
Px4Lite_Result_t Px4Lite_CopyModuleStatuses(Px4Lite_ModuleStatus_t *status, uint16_t count, uint32_t *version)
{
  uint32_t version_before;
  uint32_t version_after;
  uint32_t attempt;
  uint16_t i;

  if ((status == 0) || (count < (uint16_t)PX4LITE_MODULE_COUNT)) { return PX4LITE_INVALID_PARAM; }

  for (attempt = 0U; attempt < 3U; ++attempt) {
    version_before = Px4Lite_GetStatusVersion();
    for (i = 0U; i < (uint16_t)PX4LITE_MODULE_COUNT; ++i) {
      taskENTER_CRITICAL();
      status[i] = s_status[i];
      taskEXIT_CRITICAL();
    }
    version_after = Px4Lite_GetStatusVersion();
    if (version_before == version_after) {
      if (version != 0) { *version = version_after; }
      return PX4LITE_OK;
    }
  }

  return PX4LITE_BUSY;
}

/**
 * @brief 返回当前模块状态版本号。
 */
uint32_t Px4Lite_GetStatusVersion(void)
{
  uint32_t version;

  taskENTER_CRITICAL();
  version = s_status_version;
  taskEXIT_CRITICAL();
  return version;
}

/**
 * @brief 允许外部服务适配器更新对应 Framework 模块状态。
 */
void Px4Lite_SetExternalModuleState(Px4Lite_ModuleId_t module_id, Px4Lite_State_t state, uint16_t fault_code, uint32_t now_ms)
{
  if ((uint32_t)module_id >= (uint32_t)PX4LITE_MODULE_COUNT) { return; }

  if (state == PX4LITE_STATE_ONLINE) {
    taskENTER_CRITICAL();
    if (s_status[module_id].last_valid_ms != now_ms) {
      s_status[module_id].last_valid_ms = now_ms;
      s_status_version++;
    }
    taskEXIT_CRITICAL();
  }

  Px4Lite_SetStatus(module_id, state, fault_code, now_ms);
}

Px4Lite_Result_t Px4Lite_CommModulesInit(void)
{
#if PX4LITE_ENABLE_LORA
  uint32_t now_ms         = Px4Lite_PlatformGetMs();
  Px4Lite_Result_t result = Px4Lite_LoRaInit();

  if (result == PX4LITE_OK) { result = Px4Lite_MavlinkTxInit(now_ms); }

  Px4Lite_SetStatus(PX4LITE_MODULE_LORA, (result == PX4LITE_OK) ? PX4LITE_STATE_STARTING : PX4LITE_STATE_FAILED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_COMM_OFFLINE, now_ms);
  return result;
#else
  return PX4LITE_OK;
#endif
}

void Px4Lite_CommWorkRun(uint32_t now_ms)
{
#if PX4LITE_ENABLE_LORA
  Px4Lite_CommDebugInfo_t info;
  Px4Lite_State_t state;
  Px4Lite_Result_t result;
  Px4Lite_Result_t tx_result;

  result = Px4Lite_LoRaService(now_ms);
  if (result == PX4LITE_OK) {
    tx_result = Px4Lite_MavlinkTxRun(now_ms);
    if ((tx_result != PX4LITE_OK) && (tx_result != PX4LITE_IDLE) && (tx_result != PX4LITE_NOT_READY) && (tx_result != PX4LITE_STALE) && (tx_result != PX4LITE_BUSY)) { result = tx_result; }
  }

  memset(&info, 0, sizeof(info));
  Px4Lite_LoRaGetDebugInfo(&info);
  state = Px4Lite_LoRaGetState(now_ms);

  taskENTER_CRITICAL();
  s_status[PX4LITE_MODULE_LORA].last_rx_ms    = info.last_rx_ms;
  s_status[PX4LITE_MODULE_LORA].last_valid_ms = info.last_rx_ms;
  s_status[PX4LITE_MODULE_LORA].error_count   = info.parse_error_count + info.send_error_count;
  s_status[PX4LITE_MODULE_LORA].drop_count    = info.rx_drop_count + info.rx_overflow_count;
  taskEXIT_CRITICAL();

  /*
   * RX freshness is the only ONLINE evidence. Local TX completion stays in
   * debug stats for later half-duplex scheduling and link-budget analysis.
   */
  if (result != PX4LITE_OK) {
    Px4Lite_SetStatus(PX4LITE_MODULE_LORA, PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_COMM_TIMEOUT, now_ms);
  } else if (state == PX4LITE_STATE_ONLINE) {
    Px4Lite_SetStatus(PX4LITE_MODULE_LORA, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, now_ms);
  } else if ((state == PX4LITE_STATE_DEGRADED) || ((state == PX4LITE_STATE_STARTING) && (Px4Lite_ElapsedMs(now_ms, s_start_ms) > PX4LITE_LORA_STARTUP_GRACE_MS))) {
    Px4Lite_SetStatus(PX4LITE_MODULE_LORA, PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_COMM_TIMEOUT, now_ms);
  }
#else
  (void)now_ms;
#endif
}

void Px4Lite_GetCommDebugInfo(Px4Lite_CommDebugInfo_t *out)
{
  Px4Lite_MavlinkTxStats_t stats;

  if (out == 0) { return; }

  Px4Lite_LoRaGetDebugInfo(out);
  memset(&stats, 0, sizeof(stats));
  Px4Lite_MavlinkTxGetStats(&stats);
  out->mav_heartbeat_count       = stats.heartbeat_count;
  out->mav_gps_raw_count         = stats.gps_raw_count;
  out->mav_gnss_detail_count     = stats.gnss_detail_count;
  out->mav_attitude_count        = stats.attitude_count;
  out->mav_position_count        = stats.position_count;
  out->mav_sys_status_count      = stats.sys_status_count;
  out->mav_module_state_count    = stats.module_state_count;
  out->mav_battery_status_count  = stats.battery_status_count;
  out->mav_scaled_pressure_count = stats.scaled_pressure_count;
  out->mav_statustext_count      = stats.statustext_count;
  out->mav_no_data_count         = stats.no_data_count;
  out->mav_stale_count           = stats.stale_count;
  out->mav_error_count           = stats.error_count;
  out->mav_last_tx_msg_id        = stats.last_message_id;
}

Px4Lite_Result_t Px4Lite_CopyCommRxFrame(Px4Lite_CommRxFrame_t *out)
{
  return Px4Lite_LoRaCopyRxFrame(out);
}
