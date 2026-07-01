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
#include "px4lite_remote_telemetry.h"
#include "px4lite_recovery.h"
#include "px4lite_time.h"
#include "px4lite_topics.h"
#include "px4lite_mavlink_rx.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_remoteid_tx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define PX4LITE_ESTIMATOR_DEG100_TO_RAD 0.000174532925f
#define PX4LITE_IMU_ACCEL_NORM_TARGET_MG 1000
#define PX4LITE_IMU_ACCEL_NORM_WARN_MG   150
#define PX4LITE_IMU_ACCEL_NORM_BAD_MG    350
#define PX4LITE_IMU_GYRO_WARN_MDPS       100000
#define PX4LITE_IMU_GYRO_BAD_MDPS        240000

typedef struct {
  int64_t accel_sum_mg[3];   /**< 静止标定期间累计加速度，单位 mg。 */
  int64_t gyro_sum_mdps[3];  /**< 静止标定期间累计角速度，单位 mdps。 */
  int32_t accel_bias_mg[3];  /**< 启动静止零偏，单位 mg。 */
  int32_t gyro_bias_mdps[3]; /**< 启动静止零偏，单位 mdps。 */
  uint16_t sample_count;     /**< 已累计静止样本数量。 */
  uint8_t calibrated;        /**< 1 表示当前启动周期零偏标定完成。 */
  uint8_t reserved;          /**< 对齐预留。 */
} Px4Lite_ImuCalibration_t;

typedef struct {
  int32_t baro_to_gnss_offset_mm; /**< 气压高度对齐 GNSS 高度的偏移，单位 mm。 */
  uint8_t offset_valid;           /**< 1 表示偏移已经由一次有效 GNSS 高度校准。 */
  uint8_t reserved[3];            /**< 对齐预留。 */
} Px4Lite_AltitudeFusion_t;

static Px4Lite_ModuleStatus_t s_status[PX4LITE_MODULE_COUNT];
static uint32_t s_status_version;
static uint32_t s_gnss_sequence;
static uint32_t s_imu_sequence;
static uint32_t s_baro_sequence;
static uint32_t s_battery_sequence;
static uint32_t s_battery2_sequence;
static uint32_t s_navigation_sequence;
static uint32_t s_health_sequence;
static uint32_t s_start_ms;
static uint32_t s_remoteid_busy_since_ms;
static volatile uint8_t s_remoteid_reinit_requested;
static uint32_t s_last_imu_work_ms;
static uint32_t s_last_baro_work_ms;
static uint32_t s_last_battery_work_ms;
static Px4Lite_AttitudeState_t s_attitude_state;
static Px4Lite_ImuCalibration_t s_imu_calibration;
static Px4Lite_AltitudeFusion_t s_altitude_fusion;
static Px4Lite_VehicleNavigation_t s_estimator_navigation;
static Px4Lite_SensorGnss_t s_estimator_gnss;
static Px4Lite_SensorImu_t s_estimator_imu;
static Px4Lite_SensorImu_t s_estimator_calibrated_imu;
static Px4Lite_SensorBaro_t s_estimator_baro;
static uint32_t s_estimator_last_gnss_sequence;
static uint32_t s_estimator_last_baro_sequence;

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
 * @brief 返回 int32 绝对值，避免在质量评分中重复展开。
 */
static int32_t Px4Lite_AbsI32(int32_t value)
{
  return (value >= 0) ? value : -value;
}

/**
 * @brief 计算两个毫秒时间戳的绝对差，要求差值小于 int32 可表达范围。
 */
static uint32_t Px4Lite_AbsTimeDeltaMs(uint32_t a_ms, uint32_t b_ms)
{
  return (a_ms >= b_ms) ? (a_ms - b_ms) : (b_ms - a_ms);
}

/**
 * @brief 将浮点值四舍五入为 int32。
 */
static int32_t Px4Lite_ModulesRoundFloatToI32(float value)
{
  if (value >= 0.0f) { return (int32_t)(value + 0.5f); }
  return (int32_t)(value - 0.5f);
}

/**
 * @brief 计算三轴加速度模长，单位 mg。
 */
static uint32_t Px4Lite_ImuAccelNormMg(const Px4Lite_SensorImu_t *imu)
{
  float x;
  float y;
  float z;

  x = (float)imu->accel_mg[0];
  y = (float)imu->accel_mg[1];
  z = (float)imu->accel_mg[2];
  return (uint32_t)Px4Lite_ModulesRoundFloatToI32(sqrtf((x * x) + (y * y) + (z * z)));
}

/**
 * @brief 判断一帧 IMU 是否适合参与启动静止零偏标定。
 */
static uint8_t Px4Lite_ImuSampleStationary(const Px4Lite_SensorImu_t *imu)
{
  uint32_t accel_norm_mg;
  uint8_t i;

  accel_norm_mg = Px4Lite_ImuAccelNormMg(imu);
  if ((accel_norm_mg < PX4LITE_IMU_CALIB_ACCEL_MIN_MG) || (accel_norm_mg > PX4LITE_IMU_CALIB_ACCEL_MAX_MG)) { return 0U; }

  for (i = 0U; i < 3U; ++i) {
    if (Px4Lite_AbsI32(imu->gyro_mdps[i]) > (int32_t)PX4LITE_IMU_CALIB_GYRO_MAX_MDPS) { return 0U; }
  }
  return 1U;
}

/**
 * @brief 复位启动 IMU 零偏标定状态。
 */
static void Px4Lite_ImuCalibrationReset(void)
{
  memset(&s_imu_calibration, 0, sizeof(s_imu_calibration));
}

/**
 * @brief 累计静止 IMU 样本并在达到样本数后生成零偏。
 */
static void Px4Lite_ImuCalibrationUpdate(const Px4Lite_SensorImu_t *imu)
{
  uint8_t i;

  if ((s_imu_calibration.calibrated != 0U) || (imu == 0)) { return; }

  if (Px4Lite_ImuSampleStationary(imu) == 0U) {
    s_imu_calibration.sample_count = 0U;
    memset(s_imu_calibration.accel_sum_mg, 0, sizeof(s_imu_calibration.accel_sum_mg));
    memset(s_imu_calibration.gyro_sum_mdps, 0, sizeof(s_imu_calibration.gyro_sum_mdps));
    return;
  }

  for (i = 0U; i < 3U; ++i) {
    s_imu_calibration.accel_sum_mg[i] += imu->accel_mg[i];
    s_imu_calibration.gyro_sum_mdps[i] += imu->gyro_mdps[i];
  }

  if (s_imu_calibration.sample_count < 65535U) { s_imu_calibration.sample_count++; }
  if (s_imu_calibration.sample_count >= PX4LITE_IMU_CALIBRATION_SAMPLES) {
    int32_t avg_accel_mg[3];
    int32_t expected_z_mg;

    for (i = 0U; i < 3U; ++i) {
      avg_accel_mg[i] = (int32_t)(s_imu_calibration.accel_sum_mg[i] / (int64_t)s_imu_calibration.sample_count);
      s_imu_calibration.gyro_bias_mdps[i] = (int32_t)(s_imu_calibration.gyro_sum_mdps[i] / (int64_t)s_imu_calibration.sample_count);
    }
    expected_z_mg = (avg_accel_mg[2] >= 0) ? PX4LITE_IMU_ACCEL_NORM_TARGET_MG : -PX4LITE_IMU_ACCEL_NORM_TARGET_MG;
    s_imu_calibration.accel_bias_mg[0] = avg_accel_mg[0];
    s_imu_calibration.accel_bias_mg[1] = avg_accel_mg[1];
    s_imu_calibration.accel_bias_mg[2] = avg_accel_mg[2] - expected_z_mg;
    s_imu_calibration.calibrated       = 1U;
  }
}

/**
 * @brief 对 IMU 样本应用启动零偏标定结果。
 */
static void Px4Lite_ImuCalibrationApply(const Px4Lite_SensorImu_t *in, Px4Lite_SensorImu_t *out)
{
  uint8_t i;

  *out = *in;
  if (s_imu_calibration.calibrated == 0U) { return; }

  for (i = 0U; i < 3U; ++i) {
    out->accel_mg[i] -= s_imu_calibration.accel_bias_mg[i];
    out->gyro_mdps[i] -= s_imu_calibration.gyro_bias_mdps[i];
  }
  out->header.flags |= PX4LITE_DATA_CALIBRATED;
}

/**
 * @brief 基于新鲜度、零偏、加速度模长和角速度范围计算姿态质量。
 */
static uint8_t Px4Lite_AttitudeQuality(const Px4Lite_SensorImu_t *imu)
{
  uint32_t accel_norm_mg;
  int32_t accel_error_mg;
  int32_t max_gyro_mdps = 0;
  int32_t quality       = 100;
  uint8_t i;

  if (imu == 0) { return 0U; }
  if (s_imu_calibration.calibrated == 0U) { quality -= 50; }

  accel_norm_mg = Px4Lite_ImuAccelNormMg(imu);
  accel_error_mg = (int32_t)accel_norm_mg - PX4LITE_IMU_ACCEL_NORM_TARGET_MG;
  accel_error_mg = Px4Lite_AbsI32(accel_error_mg);
  if (accel_error_mg > PX4LITE_IMU_ACCEL_NORM_BAD_MG) {
    quality -= 50;
  } else if (accel_error_mg > PX4LITE_IMU_ACCEL_NORM_WARN_MG) {
    quality -= 20;
  }

  for (i = 0U; i < 3U; ++i) {
    int32_t gyro_abs = Px4Lite_AbsI32(imu->gyro_mdps[i]);
    if (gyro_abs > max_gyro_mdps) { max_gyro_mdps = gyro_abs; }
  }
  if (max_gyro_mdps > PX4LITE_IMU_GYRO_BAD_MDPS) {
    quality -= 40;
  } else if (max_gyro_mdps > PX4LITE_IMU_GYRO_WARN_MDPS) {
    quality -= 15;
  }

  if (quality < 0) { quality = 0; }
  return (uint8_t)quality;
}

/**
 * @brief 根据压力范围、垂直速度和跳变结果计算气压计质量。
 */
static uint8_t Px4Lite_BaroQuality(const Px4Lite_SensorBaro_t *baro)
{
  int32_t quality = 100;

  if (baro == 0) { return 0U; }
  if ((baro->pressure_pa < PX4LITE_BARO_PRESSURE_MIN_PA) || (baro->pressure_pa > PX4LITE_BARO_PRESSURE_MAX_PA)) { return 0U; }
  if (Px4Lite_AbsI32(baro->vertical_speed_cms) > 1000) { quality -= 25; }
  if ((baro->pressure_altitude_mm == 0) && (baro->pressure_pa < 100000.0f)) { quality -= 20; }
  return (quality > 0) ? (uint8_t)quality : 0U;
}

/**
 * @brief 将 GNSS 地速和航向转换为 N/E 速度。
 */
static void Px4Lite_FillGnssHorizontalVelocity(Px4Lite_VehicleNavigation_t *navigation, const Px4Lite_SensorGnss_t *gnss)
{
  float heading_rad;

  if ((navigation == 0) || (gnss == 0) || (gnss->heading_deg100 > 36000U)) { return; }

  heading_rad = ((float)gnss->heading_deg100) * PX4LITE_ESTIMATOR_DEG100_TO_RAD;
  navigation->velocity_north_cms = Px4Lite_ModulesRoundFloatToI32(((float)gnss->ground_speed_cms) * cosf(heading_rad));
  navigation->velocity_east_cms  = Px4Lite_ModulesRoundFloatToI32(((float)gnss->ground_speed_cms) * sinf(heading_rad));
  navigation->valid_mask |= PX4LITE_NAV_VALID_VELOCITY;
}

/**
 * @brief 使用 GNSS 高度校准气压高度偏移，并输出当前可用融合高度。
 */
static void Px4Lite_UpdateAltitudeFusion(Px4Lite_VehicleNavigation_t *navigation, const Px4Lite_SensorGnss_t *gnss, const Px4Lite_SensorBaro_t *baro, uint8_t gnss_alt_valid, uint8_t baro_valid)
{
  int32_t corrected_baro_altitude_mm;
  uint8_t aligned = 0U;

  if ((navigation == 0) || (baro_valid == 0U) || (baro == 0)) { return; }

  corrected_baro_altitude_mm = baro->pressure_altitude_mm;
  if ((gnss_alt_valid != 0U) && (gnss != 0)) {
    int32_t new_offset = gnss->altitude_mm - baro->pressure_altitude_mm;

    if (s_altitude_fusion.offset_valid == 0U) {
      s_altitude_fusion.baro_to_gnss_offset_mm = new_offset;
      s_altitude_fusion.offset_valid           = 1U;
    } else {
      s_altitude_fusion.baro_to_gnss_offset_mm = ((s_altitude_fusion.baro_to_gnss_offset_mm * 15) + new_offset) / 16;
    }

    aligned = (Px4Lite_AbsTimeDeltaMs(gnss->header.sample_time_ms, baro->header.sample_time_ms) <= PX4LITE_FUSION_TIME_ALIGN_MS) ? 1U : 0U;
  }

  if (s_altitude_fusion.offset_valid != 0U) { corrected_baro_altitude_mm += s_altitude_fusion.baro_to_gnss_offset_mm; }

  if ((gnss_alt_valid != 0U) && (gnss != 0) && (aligned != 0U)) {
    navigation->fused_altitude_mm = ((gnss->altitude_mm * 3) + corrected_baro_altitude_mm) / 4;
    navigation->header.flags |= PX4LITE_DATA_FUSED;
  } else if (navigation->valid_mask == 0U) {
    navigation->fused_altitude_mm = corrected_baro_altitude_mm;
  } else if ((navigation->valid_mask & PX4LITE_NAV_VALID_ALTITUDE) == 0U) {
    navigation->fused_altitude_mm = corrected_baro_altitude_mm;
  }

  navigation->vertical_speed_cms = baro->vertical_speed_cms;
  navigation->velocity_down_cms  = -baro->vertical_speed_cms;
  navigation->valid_mask |= PX4LITE_NAV_VALID_ALTITUDE;
}

/**
 * @brief 汇总当前有效源的最低质量作为导航置信度。
 */
static uint8_t Px4Lite_NavigationQuality(uint8_t gnss_quality, uint8_t baro_quality, uint8_t attitude_quality, uint32_t valid_mask)
{
  uint8_t quality = 100U;
  uint8_t any     = 0U;

  if ((valid_mask & (PX4LITE_NAV_VALID_POSITION | PX4LITE_NAV_VALID_VELOCITY)) != 0U) {
    quality = gnss_quality;
    any     = 1U;
  }
  if (((valid_mask & PX4LITE_NAV_VALID_ALTITUDE) != 0U) && (baro_quality != 0U)) {
    if ((any == 0U) || (baro_quality < quality)) { quality = baro_quality; }
    any = 1U;
  }
  if ((valid_mask & PX4LITE_NAV_VALID_ATTITUDE) != 0U) {
    if ((any == 0U) || (attitude_quality < quality)) { quality = attitude_quality; }
    any = 1U;
  }
  return (any != 0U) ? quality : 0U;
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
  s_battery2_sequence    = 0U;
  s_navigation_sequence  = 0U;
  s_health_sequence      = 0U;
  s_remoteid_busy_since_ms = 0U;
  s_remoteid_reinit_requested = 0U;
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
  Px4Lite_Result_t result = Px4Lite_GnssInit();

  Px4Lite_PublishInitState(PX4LITE_MODULE_GNSS, result, Px4Lite_PlatformGetMs());
  return result;
#else
  return PX4LITE_OK;
#endif
}

/**
 * @brief 注册表 init 回调：启动 IMU 设备并发布初始状态。
 */
Px4Lite_Result_t Px4Lite_ImuModuleInit(void)
{
#if PX4LITE_ENABLE_IMU
  Px4Lite_Result_t result = Px4Lite_ImuInit();

  Px4Lite_PublishInitState(PX4LITE_MODULE_IMU, result, Px4Lite_PlatformGetMs());
  return result;
#else
  return PX4LITE_OK;
#endif
}

/**
 * @brief 注册表 init 回调：启动气压计设备并发布初始状态。
 */
Px4Lite_Result_t Px4Lite_BaroModuleInit(void)
{
#if PX4LITE_ENABLE_BARO
  Px4Lite_Result_t result = Px4Lite_BaroInit();

  Px4Lite_PublishInitState(PX4LITE_MODULE_BARO, result, Px4Lite_PlatformGetMs());
  return result;
#else
  return PX4LITE_OK;
#endif
}

/**
 * @brief 注册表 init 回调：启动电源/ADC 设备并发布初始状态。
 */
Px4Lite_Result_t Px4Lite_BatteryModuleInit(void)
{
#if PX4LITE_ENABLE_BATTERY
  Px4Lite_Result_t result = Px4Lite_BatteryInit();

  Px4Lite_PublishInitState(PX4LITE_MODULE_BATTERY, result, Px4Lite_PlatformGetMs());
  return result;
#else
  return PX4LITE_OK;
#endif
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
#if PX4LITE_ENABLE_LORA && PX4LITE_LORA_RECOVERY_ENABLE
  Px4Lite_LoRaRequestReinit();
#endif
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_RemoteIdRecover(void)
{
#if PX4LITE_ENABLE_REMOTE_ID
  s_remoteid_reinit_requested = 1U;
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
      baro.header.quality         = Px4Lite_BaroQuality(&baro);
      baro.header.flags           = PX4LITE_DATA_VALID;
      if (baro.header.quality < 60U) { baro.header.flags |= PX4LITE_DATA_DEGRADED; }
      Px4Lite_PublishBaro(&baro);

      taskENTER_CRITICAL();
      s_status[PX4LITE_MODULE_BARO].last_rx_ms         = baro.header.sample_time_ms;
      if (baro.header.quality >= 50U) { s_status[PX4LITE_MODULE_BARO].last_valid_ms = baro.header.sample_time_ms; }
      s_status[PX4LITE_MODULE_BARO].consecutive_errors = 0U;
      if (s_status[PX4LITE_MODULE_BARO].consecutive_valid < 65535U) { s_status[PX4LITE_MODULE_BARO].consecutive_valid++; }
      s_status_version++;
      taskEXIT_CRITICAL();

      Px4Lite_SetStatus(PX4LITE_MODULE_BARO, (baro.header.quality >= 50U) ? PX4LITE_STATE_ONLINE : PX4LITE_STATE_DEGRADED, (baro.header.quality >= 50U) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_SENSOR_INVALID, now_ms);
    } else if (result == PX4LITE_IO_ERROR) {
      Px4Lite_RecordSensorIoError(PX4LITE_MODULE_BARO, now_ms);
    }
  }
#endif

#if PX4LITE_ENABLE_BATTERY
  if ((s_last_battery_work_ms == 0U) || ((uint32_t)(now_ms - s_last_battery_work_ms) >= PX4LITE_BATTERY_WORK_PERIOD_MS)) {
    Px4Lite_BatteryStatus_t battery;
    Px4Lite_BatteryStatus_t battery2;
    Px4Lite_Result_t result;
    Px4Lite_Result_t result2;

    s_last_battery_work_ms = now_ms;
    memset(&battery, 0, sizeof(battery));
    memset(&battery2, 0, sizeof(battery2));
    result = Px4Lite_BatteryRead(&battery);
    result2 = Px4Lite_Battery2Read(&battery2);
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

    if (result2 == PX4LITE_OK) {
      battery2.header.sample_time_ms  = (battery2.header.sample_time_ms != 0U) ? battery2.header.sample_time_ms : now_ms;
      battery2.header.publish_time_ms = now_ms;
      battery2.header.sequence        = ++s_battery2_sequence;
      battery2.header.device_id       = (uint16_t)(PX4LITE_MODULE_BATTERY + 0x0100U);
      battery2.header.valid           = 1U;
      battery2.header.quality         = (battery2.low_voltage != 0U) ? 50U : 100U;
      battery2.header.flags           = PX4LITE_DATA_VALID;
      if (battery2.low_voltage != 0U) { battery2.header.flags |= PX4LITE_DATA_DEGRADED; }
      Px4Lite_PublishBattery2(&battery2);

      taskENTER_CRITICAL();
      if (battery2.header.sample_time_ms > s_status[PX4LITE_MODULE_BATTERY].last_rx_ms) { s_status[PX4LITE_MODULE_BATTERY].last_rx_ms = battery2.header.sample_time_ms; }
      if (battery2.header.sample_time_ms > s_status[PX4LITE_MODULE_BATTERY].last_valid_ms) { s_status[PX4LITE_MODULE_BATTERY].last_valid_ms = battery2.header.sample_time_ms; }
      taskEXIT_CRITICAL();

      if (battery2.low_voltage != 0U) { Px4Lite_SetStatus(PX4LITE_MODULE_BATTERY, PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_SENSOR_INVALID, now_ms); }
    } else if (result2 == PX4LITE_IO_ERROR) {
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
  Px4Lite_ImuCalibrationReset();
  memset(&s_altitude_fusion, 0, sizeof(s_altitude_fusion));
  memset(&s_estimator_navigation, 0, sizeof(s_estimator_navigation));
  memset(&s_estimator_gnss, 0, sizeof(s_estimator_gnss));
  memset(&s_estimator_imu, 0, sizeof(s_estimator_imu));
  memset(&s_estimator_calibrated_imu, 0, sizeof(s_estimator_calibrated_imu));
  memset(&s_estimator_baro, 0, sizeof(s_estimator_baro));
  s_estimator_last_gnss_sequence = 0U;
  s_estimator_last_baro_sequence = 0U;
  Px4Lite_SetStatus(PX4LITE_MODULE_ESTIMATOR, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, Px4Lite_PlatformGetMs());
  return PX4LITE_OK;
}

/**
 * @brief 融合 GNSS、IMU 和气压计快照，发布带有效位和质量分的 Navigation。
 */
void Px4Lite_EstimatorRun(uint32_t now_ms)
{
  uint8_t publish_navigation = 0U;
  uint8_t attitude_updated   = 0U;
  uint8_t imu_pop_count      = 0U;
  uint8_t gnss_quality       = 0U;
  uint8_t baro_quality       = 0U;
  uint8_t attitude_quality   = 0U;
  uint8_t gnss_alt_valid     = 0U;
  uint8_t baro_valid         = 0U;
  int32_t roll_deg100        = 0;
  int32_t pitch_deg100       = 0;
  int32_t yaw_deg100         = 0;
  int32_t roll_rate_dps100   = 0;
  int32_t pitch_rate_dps100  = 0;
  int32_t yaw_rate_dps100    = 0;

  memset(&s_estimator_navigation, 0, sizeof(s_estimator_navigation));

  if ((Px4Lite_CopyGnss(&s_estimator_gnss) == PX4LITE_OK) && (Px4Lite_IsFresh(&s_estimator_gnss.header, now_ms, PX4LITE_GNSS_MAX_AGE_MS) != 0U)) {
    gnss_quality                                  = s_estimator_gnss.header.quality;
    s_estimator_navigation.latitude_e7           = s_estimator_gnss.latitude_e7;
    s_estimator_navigation.longitude_e7          = s_estimator_gnss.longitude_e7;
    s_estimator_navigation.fused_altitude_mm     = s_estimator_gnss.altitude_mm;
    s_estimator_navigation.gnss_utc_sec          = s_estimator_gnss.utc_sec;
    s_estimator_navigation.gnss_utc_date         = s_estimator_gnss.utc_date;
    s_estimator_navigation.hdop_x100             = s_estimator_gnss.hdop_x100;
    s_estimator_navigation.satellites_used       = s_estimator_gnss.satellites_used;
    s_estimator_navigation.gnss_fix_type         = Px4Lite_GnssDisplayFixState(&s_estimator_gnss);
    s_estimator_navigation.header.sample_time_ms = s_estimator_gnss.header.sample_time_ms;

    if (s_estimator_gnss.fix_type != 0U) {
      s_estimator_navigation.valid_mask = PX4LITE_NAV_VALID_POSITION | PX4LITE_NAV_VALID_ALTITUDE;
      gnss_alt_valid                    = 1U;
      Px4Lite_FillGnssHorizontalVelocity(&s_estimator_navigation, &s_estimator_gnss);
    } else {
      s_estimator_navigation.header.flags |= PX4LITE_DATA_DEGRADED;
    }

    if (s_estimator_gnss.header.sequence != s_estimator_last_gnss_sequence) {
      s_estimator_last_gnss_sequence = s_estimator_gnss.header.sequence;
      publish_navigation             = 1U;
    }
  }

  if ((Px4Lite_CopyBaro(&s_estimator_baro) == PX4LITE_OK) && (Px4Lite_IsFresh(&s_estimator_baro.header, now_ms, PX4LITE_BARO_MAX_AGE_MS) != 0U)) {
    baro_quality = s_estimator_baro.header.quality;
    baro_valid   = (baro_quality >= 50U) ? 1U : 0U;
    if (baro_valid != 0U) {
      Px4Lite_UpdateAltitudeFusion(&s_estimator_navigation, &s_estimator_gnss, &s_estimator_baro, gnss_alt_valid, baro_valid);
      if (s_estimator_navigation.header.sample_time_ms == 0U) { s_estimator_navigation.header.sample_time_ms = s_estimator_baro.header.sample_time_ms; }
      if (s_estimator_baro.header.sequence != s_estimator_last_baro_sequence) {
        s_estimator_last_baro_sequence = s_estimator_baro.header.sequence;
        publish_navigation             = 1U;
      }
    }
  }

  while ((imu_pop_count < 8U) && (Px4Lite_PopImu(&s_estimator_imu) == PX4LITE_OK)) {
    uint8_t current_attitude_quality = 0U;

    imu_pop_count++;
    if ((s_estimator_imu.header.valid != 0U) && (Px4Lite_IsFresh(&s_estimator_imu.header, now_ms, PX4LITE_IMU_MAX_AGE_MS) != 0U)) {
      Px4Lite_ImuCalibrationUpdate(&s_estimator_imu);
      Px4Lite_ImuCalibrationApply(&s_estimator_imu, &s_estimator_calibrated_imu);
      current_attitude_quality = Px4Lite_AttitudeQuality(&s_estimator_calibrated_imu);
    }

    if ((current_attitude_quality >= PX4LITE_IMU_ATTITUDE_MIN_QUALITY) && (Px4Lite_AttitudeUpdate(&s_attitude_state, &s_estimator_calibrated_imu, &roll_deg100, &pitch_deg100, &yaw_deg100) == PX4LITE_OK)) {
      attitude_updated                 = 1U;
      attitude_quality                 = current_attitude_quality;
      roll_rate_dps100                 = s_estimator_calibrated_imu.gyro_mdps[0] / 10;
      pitch_rate_dps100                = s_estimator_calibrated_imu.gyro_mdps[1] / 10;
      yaw_rate_dps100                  = s_estimator_calibrated_imu.gyro_mdps[2] / 10;
      s_estimator_navigation.header.sample_time_ms = s_estimator_calibrated_imu.header.sample_time_ms;
    }
  }

  if (attitude_updated != 0U) {
    s_estimator_navigation.roll_deg100       = roll_deg100;
    s_estimator_navigation.pitch_deg100      = pitch_deg100;
    s_estimator_navigation.yaw_deg100        = yaw_deg100;
    s_estimator_navigation.roll_rate_dps100  = roll_rate_dps100;
    s_estimator_navigation.pitch_rate_dps100 = pitch_rate_dps100;
    s_estimator_navigation.yaw_rate_dps100   = yaw_rate_dps100;
    s_estimator_navigation.valid_mask |= PX4LITE_NAV_VALID_ATTITUDE;
    s_estimator_navigation.valid_mask |= PX4LITE_NAV_VALID_YAW_REL;
    s_estimator_navigation.header.flags |= PX4LITE_DATA_FILTERED;
    if (s_imu_calibration.calibrated != 0U) { s_estimator_navigation.header.flags |= PX4LITE_DATA_CALIBRATED; }
    publish_navigation = 1U;
  }

  if (publish_navigation == 0U) { return; }

  if ((s_estimator_navigation.valid_mask & PX4LITE_NAV_VALID_VELOCITY) == 0U) {
    s_estimator_navigation.velocity_north_cms = 0;
    s_estimator_navigation.velocity_east_cms  = 0;
  }
  s_estimator_navigation.navigation_quality = Px4Lite_NavigationQuality(gnss_quality, baro_quality, attitude_quality, s_estimator_navigation.valid_mask);
  if (s_estimator_navigation.navigation_quality < 60U) { s_estimator_navigation.header.flags |= PX4LITE_DATA_DEGRADED; }
  if (s_estimator_navigation.header.sample_time_ms == 0U) { s_estimator_navigation.header.sample_time_ms = now_ms; }
  s_estimator_navigation.header.publish_time_ms = now_ms;
  s_estimator_navigation.header.sequence        = ++s_navigation_sequence;
  s_estimator_navigation.header.device_id       = 0x0100U;
  s_estimator_navigation.header.valid           = (s_estimator_navigation.valid_mask != 0U) ? 1U : 0U;
  s_estimator_navigation.header.quality         = s_estimator_navigation.navigation_quality;
  if (s_estimator_navigation.header.valid != 0U) {
    s_estimator_navigation.header.flags |= PX4LITE_DATA_VALID;
  } else {
    s_estimator_navigation.header.flags |= PX4LITE_DATA_DEGRADED;
  }

  Px4Lite_PublishNavigation(&s_estimator_navigation);
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

  Px4Lite_RemoteTelemetryInit(now_ms);
  if (result == PX4LITE_OK) { result = Px4Lite_MavlinkTxInit(now_ms); }

  Px4Lite_SetStatus(PX4LITE_MODULE_LORA, (result == PX4LITE_OK) ? PX4LITE_STATE_STARTING : PX4LITE_STATE_FAILED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_COMM_OFFLINE, now_ms);
  return result;
#else
  return PX4LITE_OK;
#endif
}

Px4Lite_Result_t Px4Lite_RemoteIdModuleInit(void)
{
#if PX4LITE_ENABLE_REMOTE_ID
  uint32_t now_ms         = Px4Lite_PlatformGetMs();
  Px4Lite_Result_t result = Px4Lite_RemoteIdInit();

  if (result == PX4LITE_OK) { result = Px4Lite_RemoteIdTxInit(now_ms); }
  s_remoteid_busy_since_ms = 0U;
  s_remoteid_reinit_requested = 0U;

  Px4Lite_SetStatus(PX4LITE_MODULE_REMOTE_ID, (result == PX4LITE_OK) ? PX4LITE_STATE_STARTING : PX4LITE_STATE_FAILED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_COMM_OFFLINE, now_ms);
  return result;
#else
  return PX4LITE_OK;
#endif
}

static Px4Lite_Result_t Px4Lite_RemoteIdRunReinitIfRequested(uint32_t now_ms)
{
#if PX4LITE_ENABLE_REMOTE_ID
  Px4Lite_Result_t result;

  if (s_remoteid_reinit_requested == 0U) { return PX4LITE_OK; }

  s_remoteid_reinit_requested = 0U;
  Px4Lite_RemoteIdAbortTx();
  result = Px4Lite_RemoteIdInit();
  if (result == PX4LITE_OK) { result = Px4Lite_RemoteIdTxInit(now_ms); }

  s_remoteid_busy_since_ms = 0U;
  Px4Lite_SetStatus(PX4LITE_MODULE_REMOTE_ID, (result == PX4LITE_OK) ? PX4LITE_STATE_STARTING : PX4LITE_STATE_FAILED, (result == PX4LITE_OK) ? PX4LITE_FAULT_NONE : PX4LITE_FAULT_COMM_OFFLINE, now_ms);
  return result;
#else
  (void)now_ms;
  return PX4LITE_OK;
#endif
}

void Px4Lite_CommWorkRun(uint32_t now_ms)
{
#if PX4LITE_ENABLE_LORA
  Px4Lite_CommDebugInfo_t info;
  Px4Lite_State_t state;
  Px4Lite_Result_t result;
  Px4Lite_Result_t rx_result;
  Px4Lite_Result_t tx_result;

  result = Px4Lite_LoRaService(now_ms);
  if (result == PX4LITE_OK) {
    rx_result = Px4Lite_MavlinkRxRun(now_ms);
    if ((rx_result != PX4LITE_OK) && (rx_result != PX4LITE_IDLE) && (rx_result != PX4LITE_NOT_READY)) { result = rx_result; }
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
  s_status[PX4LITE_MODULE_LORA].drop_count    = info.rx_drop_count + info.rx_overflow_count + info.rx_sequence_lost_count;
  taskEXIT_CRITICAL();

  /*
   * RX freshness is the only ONLINE evidence. Local TX completion stays in
   * debug stats for later half-duplex scheduling and link-budget analysis.
   */
  if (state == PX4LITE_STATE_FAILED) {
    Px4Lite_SetStatus(PX4LITE_MODULE_LORA, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_COMM_OFFLINE, now_ms);
  } else if (result != PX4LITE_OK) {
    Px4Lite_SetStatus(PX4LITE_MODULE_LORA, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_COMM_OFFLINE, now_ms);
  } else if (state == PX4LITE_STATE_ONLINE) {
    Px4Lite_SetStatus(PX4LITE_MODULE_LORA, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, now_ms);
  } else if ((state == PX4LITE_STATE_DEGRADED) || ((state == PX4LITE_STATE_STARTING) && (Px4Lite_ElapsedMs(now_ms, s_start_ms) > PX4LITE_LORA_STARTUP_GRACE_MS))) {
    Px4Lite_SetStatus(PX4LITE_MODULE_LORA, PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_COMM_TIMEOUT, now_ms);
  }
#endif

#if PX4LITE_ENABLE_REMOTE_ID
  {
    Px4Lite_RemoteIdTxStats_t remoteid_stats;
    Px4Lite_Result_t remoteid_result;

    if (Px4Lite_RemoteIdRunReinitIfRequested(now_ms) != PX4LITE_OK) { return; }
    if (Px4Lite_RemoteIdIsReady() == 0U) {
      Px4Lite_RecordSensorIoError(PX4LITE_MODULE_REMOTE_ID, now_ms);
      Px4Lite_SetStatus(PX4LITE_MODULE_REMOTE_ID, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_COMM_OFFLINE, now_ms);
      return;
    }

    remoteid_result = Px4Lite_RemoteIdTxRun(now_ms);
    memset(&remoteid_stats, 0, sizeof(remoteid_stats));
    Px4Lite_RemoteIdTxGetStats(&remoteid_stats);

    if (remoteid_result == PX4LITE_OK) {
      taskENTER_CRITICAL();
      s_status[PX4LITE_MODULE_REMOTE_ID].last_rx_ms = remoteid_stats.last_success_ms;
      s_status[PX4LITE_MODULE_REMOTE_ID].last_valid_ms = remoteid_stats.last_success_ms;
      s_status[PX4LITE_MODULE_REMOTE_ID].error_count = remoteid_stats.error_count;
      s_status[PX4LITE_MODULE_REMOTE_ID].drop_count = remoteid_stats.busy_count;
      s_status[PX4LITE_MODULE_REMOTE_ID].consecutive_errors = 0U;
      if (s_status[PX4LITE_MODULE_REMOTE_ID].consecutive_valid < 65535U) { s_status[PX4LITE_MODULE_REMOTE_ID].consecutive_valid++; }
      taskEXIT_CRITICAL();
      s_remoteid_busy_since_ms = 0U;
      Px4Lite_SetStatus(PX4LITE_MODULE_REMOTE_ID, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, now_ms);
    } else if (remoteid_result == PX4LITE_BUSY) {
      if (s_remoteid_busy_since_ms == 0U) { s_remoteid_busy_since_ms = now_ms; }
      if (Px4Lite_ElapsedMs(now_ms, s_remoteid_busy_since_ms) > PX4LITE_REMOTEID_TX_BUSY_TIMEOUT_MS) {
        Px4Lite_RemoteIdAbortTx();
        s_remoteid_reinit_requested = 1U;
        Px4Lite_RecordSensorIoError(PX4LITE_MODULE_REMOTE_ID, now_ms);
        Px4Lite_SetStatus(PX4LITE_MODULE_REMOTE_ID, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_COMM_TIMEOUT, now_ms);
      }
    } else if (remoteid_result == PX4LITE_IO_ERROR) {
      s_remoteid_busy_since_ms = 0U;
      s_remoteid_reinit_requested = 1U;
      Px4Lite_RecordSensorIoError(PX4LITE_MODULE_REMOTE_ID, now_ms);
      Px4Lite_SetStatus(PX4LITE_MODULE_REMOTE_ID, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_COMM_OFFLINE, now_ms);
    } else {
      s_remoteid_busy_since_ms = 0U;
      if ((remoteid_stats.last_success_ms == 0U) && (Px4Lite_ElapsedMs(now_ms, s_start_ms) > PX4LITE_REMOTEID_STARTUP_GRACE_MS)) {
        Px4Lite_SetStatus(PX4LITE_MODULE_REMOTE_ID, PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_COMM_TIMEOUT, now_ms);
      } else if ((remoteid_stats.last_success_ms != 0U) && (Px4Lite_ElapsedMs(now_ms, remoteid_stats.last_success_ms) > PX4LITE_REMOTEID_OFFLINE_MS)) {
        Px4Lite_RecordSensorIoError(PX4LITE_MODULE_REMOTE_ID, now_ms);
        Px4Lite_SetStatus(PX4LITE_MODULE_REMOTE_ID, PX4LITE_STATE_OFFLINE, PX4LITE_FAULT_COMM_TIMEOUT, now_ms);
      }
    }
  }
#endif

#if !PX4LITE_ENABLE_LORA && !PX4LITE_ENABLE_REMOTE_ID
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
  out->mav_command_count         = stats.command_count;
  out->mav_command_ack_tx_count  = stats.command_ack_tx_count;
  out->mav_command_ack_rx_count  = stats.command_ack_rx_count;
  out->mav_no_data_count         = stats.no_data_count;
  out->mav_stale_count           = stats.stale_count;
  out->mav_error_count           = stats.error_count;
  out->mav_last_tx_msg_id        = stats.last_message_id;
}

Px4Lite_Result_t Px4Lite_CopyCommRxFrame(Px4Lite_CommRxFrame_t *out)
{
  return Px4Lite_LoRaCopyRxFrame(out);
}
