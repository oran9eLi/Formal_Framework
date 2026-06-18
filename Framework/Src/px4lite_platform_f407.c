/**
 * @file px4lite_platform_f407.c
 * @brief STM32F407 平台适配实现，将 BSP/Driver 数据转换为 Framework 接口。
 *
 * @details
 * 本文件是 Framework 层唯一允许包含 BSP 头文件的位置。它负责调用 BSP 和
 * Sensor Driver，并把驱动私有类型转换为 `px4lite_types.h` 中定义的 Framework
 * 强类型数据。其他 Framework 文件不得绕过本文件直接包含 BSP 或 HAL 头文件。
 */

#include "px4lite_platform.h"
#include "px4lite_config.h"
#include "px4lite_imu_axis_map.h"
#include "bsp_gnss.h"
#include "bsp_lora.h"
#include "bsp_rtc.h"
#include "lora_e22.h"
#include "bsp_time.h"
#include "sensor_bme280.h"
#include "sensor_gnss.h"
#include "sensor_mpu6050.h"
#include "sensor_power.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static uint32_t s_heartbeat_ms[PX4LITE_HEARTBEAT_COUNT];
static uint32_t s_heartbeat_seen_mask;

/**
 * @brief 将 float 按四舍五入方式转换为 int32。
 *
 * @param[in] value 输入浮点值。
 *
 * @return 四舍五入后的整数。
 */
static int32_t Px4Lite_RoundFloatToI32(float value)
{
  if (value >= 0.0f) { return (int32_t)(value + 0.5f); }
  return (int32_t)(value - 0.5f);
}

#if PX4LITE_ENABLE_HARDWARE_WATCHDOG
extern void BSP_WatchdogRefresh(void);
#endif

/**
 * @brief 复位平台心跳状态并初始化适配层拥有的服务。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_PlatformInit(void)
{
  memset(s_heartbeat_ms, 0, sizeof(s_heartbeat_ms));
  s_heartbeat_seen_mask = 0U;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_PlatformRtcRead(Px4Lite_UtcDateTime_t *out)
{
  BSP_RTC_DateTime_t rtc_time;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  if (BSP_RTC_ReadDateTime(&rtc_time) != BSP_STATUS_OK) { return PX4LITE_NOT_READY; }

  out->year    = rtc_time.year;
  out->month   = rtc_time.month;
  out->day     = rtc_time.day;
  out->hours   = rtc_time.hours;
  out->minutes = rtc_time.minutes;
  out->seconds = rtc_time.seconds;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_PlatformRtcWrite(const Px4Lite_UtcDateTime_t *date_time)
{
  BSP_RTC_DateTime_t rtc_time;

  if (date_time == 0) { return PX4LITE_INVALID_PARAM; }

  rtc_time.year    = date_time->year;
  rtc_time.month   = date_time->month;
  rtc_time.day     = date_time->day;
  rtc_time.hours   = date_time->hours;
  rtc_time.minutes = date_time->minutes;
  rtc_time.seconds = date_time->seconds;

  return (BSP_RTC_WriteDateTime(&rtc_time) == BSP_STATUS_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

/**
 * @brief 获取平台单调毫秒时间。
 *
 * @return 当前系统毫秒时间，单位：ms。
 */
uint32_t Px4Lite_PlatformGetMs(void)
{
  /* BSP_Time_GetTickMs 是板级唯一毫秒时间源，底层基于 TIM6。 */
  return BSP_Time_GetTickMs();
}

/**
 * @brief 获取由 HAL tick 和 TIM6 组合出的相干微秒时间戳。
 *
 * @return 当前平台时间，单位：us。
 */
uint32_t Px4Lite_PlatformGetUs(void)
{
  uint32_t primask;
  uint32_t milliseconds;
  uint32_t timer_us;
  uint32_t update_pending;

  /*
   * TIM6 is the HAL 1 ms time base and runs at 1 MHz with ARR=999.
   * Save interrupt state while taking a coherent tick/counter snapshot.
   * If an update is pending, account for the millisecond whose ISR has
   * not executed yet.
   */
  primask = __get_PRIMASK();
  __disable_irq();
  milliseconds   = HAL_GetTick();
  timer_us       = TIM6->CNT;
  update_pending = TIM6->SR & TIM_SR_UIF;
  if (update_pending != 0U) {
    timer_us = TIM6->CNT;
    milliseconds++;
  }
  if (primask == 0U) { __enable_irq(); }

  return (milliseconds * 1000U) + timer_us;
}

/**
 * @brief 记录一个必需任务最近一次成功执行时间。
 *
 * @param[in] id 心跳编号。
 * @param[in] now_ms 当前系统毫秒时间。
 */
void Px4Lite_PlatformHeartbeat(Px4Lite_HeartbeatId_t id, uint32_t now_ms)
{
  if ((uint32_t)id >= (uint32_t)PX4LITE_HEARTBEAT_COUNT) { return; }

  taskENTER_CRITICAL();
  s_heartbeat_ms[id] = now_ms;
  s_heartbeat_seen_mask |= (1UL << (uint32_t)id);
  taskEXIT_CRITICAL();
}

/**
 * @brief 检查所有必需任务心跳是否存在且未超时。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 1 表示心跳健康，0 表示至少一个必需任务未上报或已超时。
 */
uint8_t Px4Lite_PlatformHeartbeatsHealthy(uint32_t now_ms)
{
  uint32_t heartbeat[PX4LITE_HEARTBEAT_COUNT];
  uint32_t seen_mask;
  uint32_t required_mask;
  uint32_t i;

  taskENTER_CRITICAL();
  memcpy(heartbeat, s_heartbeat_ms, sizeof(heartbeat));
  seen_mask = s_heartbeat_seen_mask;
  taskEXIT_CRITICAL();

  required_mask = (1UL << (uint32_t)PX4LITE_HEARTBEAT_SENSOR) | (1UL << (uint32_t)PX4LITE_HEARTBEAT_ESTIMATOR) | (1UL << (uint32_t)PX4LITE_HEARTBEAT_HEALTH) | (1UL << (uint32_t)PX4LITE_HEARTBEAT_SYSTEM) | (1UL << (uint32_t)PX4LITE_HEARTBEAT_BUSINESS);
#if PX4LITE_ENABLE_DISPLAY
  required_mask |= (1UL << (uint32_t)PX4LITE_HEARTBEAT_DISPLAY);
#endif
  if ((seen_mask & required_mask) != required_mask) { return 0U; }

  for (i = 0U; i < (uint32_t)PX4LITE_HEARTBEAT_COUNT; ++i) {
    if ((uint32_t)(now_ms - heartbeat[i]) > PX4LITE_TASK_HEARTBEAT_TIMEOUT_MS) { return 0U; }
  }

  return 1U;
}

/**
 * @brief 在所有必需任务心跳健康时刷新硬件看门狗。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 */
void Px4Lite_PlatformWatchdogFeed(uint32_t now_ms)
{
  if (Px4Lite_PlatformHeartbeatsHealthy(now_ms) == 0U) { return; }

#if PX4LITE_ENABLE_HARDWARE_WATCHDOG
  BSP_WatchdogRefresh();
#else
  (void)now_ms;
#endif
}

/**
 * @brief 初始化 GNSS BSP 和强类型 GNSS 解析器。
 */
Px4Lite_Result_t Px4Lite_GnssInit(void)
{
  /* GNSS UART/DMA 由 BSP_Init() 集中初始化；此处只初始化解析器状态。 */
  return (Sensor_GNSS_Init() == GNSS_RESULT_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

void Px4Lite_GnssRequestReinit(void)
{
  Sensor_GNSS_RequestReinit();
}

/**
 * @brief 将一个最新 GNSS 驱动快照转换为 Framework 测量格式。
 */
Px4Lite_Result_t Px4Lite_GnssRead(Px4Lite_SensorGnss_t *measurement)
{
  static uint32_t last_rx_sequence;
  Gnss_Snapshot_t snapshot;
  Gnss_Result_t result;

  if (measurement == 0) { return PX4LITE_INVALID_PARAM; }

  (void)Sensor_GNSS_Service(Px4Lite_PlatformGetMs());
  result = Sensor_GNSS_CopySnapshot(&snapshot);
  if (result == GNSS_RESULT_INVALID_PARAM) { return PX4LITE_IO_ERROR; }

  if ((snapshot.rx_sequence == 0U) || (snapshot.rx_sequence == last_rx_sequence)) { return PX4LITE_IDLE; }

  memset(measurement, 0, sizeof(*measurement));
  measurement->header.sample_time_ms = snapshot.last_rx_ms;
  measurement->latitude_e7           = snapshot.latitude_deg_e7;
  measurement->longitude_e7          = snapshot.longitude_deg_e7;
  measurement->altitude_mm           = snapshot.altitude_mm;
  measurement->ground_speed_cms      = snapshot.speed_cms;
  measurement->utc_sec               = snapshot.utc_sec;
  measurement->utc_date              = snapshot.utc_date;
  measurement->heading_deg100        = (uint16_t)snapshot.heading_deg100;
  measurement->hdop_x100             = snapshot.hdop_cm;
  measurement->fix_type              = (snapshot.fix_valid != 0U) ? snapshot.fix_quality : 0U;
  measurement->fix_dimension         = snapshot.fix_dimension;
  measurement->satellites_used       = snapshot.satellites;
  measurement->gps_visible           = snapshot.gps_visible_sats;
  measurement->bds_visible           = snapshot.bds_visible_sats;
  measurement->gps_used              = snapshot.gps_used_sats;
  measurement->bds_used              = snapshot.bds_used_sats;

  last_rx_sequence = snapshot.rx_sequence;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_ImuInit(void)
{
  return (Sensor_MPU6050_Init() == MPU6050_RESULT_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

void Px4Lite_ImuRequestReinit(void)
{
  Sensor_MPU6050_RequestReinit();
}

Px4Lite_Result_t Px4Lite_ImuRead(Px4Lite_SensorImu_t *measurement)
{
  static uint32_t last_rx_sequence;
  static uint32_t last_reinit_count;
  static uint8_t stable_valid_count;
  Mpu6050_Snapshot_t snapshot;
  Mpu6050_Status_t status;
  Mpu6050_Result_t result;
  uint32_t now_ms;
  uint8_t i;

  if (measurement == 0) { return PX4LITE_INVALID_PARAM; }

  now_ms = Px4Lite_PlatformGetMs();
  result = Sensor_MPU6050_Service(now_ms);
  if (result != MPU6050_RESULT_OK) {
    stable_valid_count = 0U;
    return (result == MPU6050_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR;
  }

  result = Sensor_MPU6050_CopySnapshot(&snapshot);
  if (result != MPU6050_RESULT_OK) { return (result == MPU6050_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }

  if ((snapshot.rx_sequence == 0U) || (snapshot.rx_sequence == last_rx_sequence)) { return PX4LITE_IDLE; }

  memset(&status, 0, sizeof(status));
  (void)Sensor_MPU6050_GetStatus(&status);
  if (status.reinit_count != last_reinit_count) {
    last_reinit_count  = status.reinit_count;
    stable_valid_count = 0U;
  }
  if (stable_valid_count < PX4LITE_IMU_STABLE_VALID_MIN) {
    stable_valid_count++;
    last_rx_sequence = snapshot.rx_sequence;
    return PX4LITE_IDLE;
  }

  memset(measurement, 0, sizeof(*measurement));
  measurement->header.sample_time_ms = snapshot.sample_time_ms;
  for (i = 0U; i < 3U; ++i) {
    measurement->accel_mg[i]  = Px4Lite_RoundFloatToI32(snapshot.accel_g[i] * 1000.0f);
    measurement->gyro_mdps[i] = Px4Lite_RoundFloatToI32(snapshot.gyro_dps[i] * 1000.0f);
  }
  Px4Lite_MapImuAxes(measurement->accel_mg, measurement->gyro_mdps);
  measurement->temperature_cdeg = (int16_t)Px4Lite_RoundFloatToI32(snapshot.temperature_c * 100.0f);
  measurement->sample_period_us = (uint16_t)(PX4LITE_IMU_WORK_PERIOD_MS * 1000U);

  last_rx_sequence = snapshot.rx_sequence;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_BaroInit(void)
{
  return (Sensor_BME280_Init() == BME280_RESULT_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

void Px4Lite_BaroRequestReinit(void)
{
  Sensor_BME280_RequestReinit();
}

Px4Lite_Result_t Px4Lite_BaroRead(Px4Lite_SensorBaro_t *measurement)
{
  static uint32_t last_rx_sequence;
  Bme280_Snapshot_t snapshot;
  Bme280_Result_t result;
  uint32_t now_ms;

  if (measurement == 0) { return PX4LITE_INVALID_PARAM; }

  now_ms = Px4Lite_PlatformGetMs();
  result = Sensor_BME280_Service(now_ms);
  if (result != BME280_RESULT_OK) { return (result == BME280_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }

  result = Sensor_BME280_CopySnapshot(&snapshot);
  if (result != BME280_RESULT_OK) { return (result == BME280_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }

  if ((snapshot.rx_sequence == 0U) || (snapshot.rx_sequence == last_rx_sequence)) { return PX4LITE_IDLE; }

  memset(measurement, 0, sizeof(*measurement));
  measurement->header.sample_time_ms = snapshot.sample_time_ms;
  measurement->pressure_pa           = snapshot.pressure_pa;
  measurement->temperature_c         = snapshot.temperature_c;
  measurement->relative_humidity_pct = snapshot.relative_humidity_pct;

  last_rx_sequence = snapshot.rx_sequence;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_BatteryInit(void)
{
  return (Sensor_Power_Init() == POWER_RESULT_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

void Px4Lite_BatteryRequestReinit(void)
{
  Sensor_Power_RequestReinit();
}

Px4Lite_Result_t Px4Lite_BatteryRead(Px4Lite_BatteryStatus_t *measurement)
{
  static uint32_t last_rx_sequence;
  Power_Snapshot_t snapshot;
  Power_Result_t result;
  uint32_t now_ms;

  if (measurement == 0) { return PX4LITE_INVALID_PARAM; }

  now_ms = Px4Lite_PlatformGetMs();
  result = Sensor_Power_Service(now_ms);
  if (result != POWER_RESULT_OK) { return (result == POWER_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }

  result = Sensor_Power_CopySnapshot(&snapshot);
  if (result != POWER_RESULT_OK) { return (result == POWER_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }

  if ((snapshot.rx_sequence == 0U) || (snapshot.rx_sequence == last_rx_sequence)) { return PX4LITE_IDLE; }

  memset(measurement, 0, sizeof(*measurement));
  measurement->header.sample_time_ms = snapshot.sample_time_ms;
  measurement->voltage_mv            = (uint32_t)((snapshot.voltage_v * 1000.0f) + 0.5f);
  measurement->percent               = snapshot.percent;
  measurement->low_voltage           = snapshot.low_voltage;

  last_rx_sequence = snapshot.rx_sequence;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_LoRaInit(void)
{
  if (BSP_LoRa_Init() != 0) { return PX4LITE_IO_ERROR; }
  return (Lora_E22_Init() == LORA_RESULT_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

void Px4Lite_LoRaRequestReinit(void)
{
  Lora_E22_RequestReinit();
}

Px4Lite_Result_t Px4Lite_LoRaService(uint32_t now_ms)
{
  return (Lora_E22_Service(now_ms) == LORA_RESULT_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

Px4Lite_Result_t Px4Lite_LoRaSend(const uint8_t *data, uint16_t len)
{
  Lora_Result_t result = Lora_E22_Send(data, len);
  if (result == LORA_RESULT_OK) return PX4LITE_OK;
  if (result == LORA_RESULT_BUSY) return PX4LITE_BUSY;
  return PX4LITE_IO_ERROR;
}

Px4Lite_State_t Px4Lite_LoRaGetState(uint32_t now_ms)
{
  switch (Lora_E22_GetState(now_ms, PX4LITE_LORA_OFFLINE_MS)) {
    case LORA_STATE_ONLINE:
      return PX4LITE_STATE_ONLINE;
    case LORA_STATE_OFFLINE:
      return PX4LITE_STATE_OFFLINE;
    case LORA_STATE_FAILED:
      return PX4LITE_STATE_FAILED;
    default:
      return PX4LITE_STATE_STARTING;
  }
}

void Px4Lite_LoRaGetDebugInfo(Px4Lite_CommDebugInfo_t *out)
{
  Lora_DebugInfo_t info;

  if (out == 0) { return; }
  memset(&info, 0, sizeof(info));
  Lora_E22_GetDebugInfo(&info);
  out->rx_frame_count    = info.rx_frame_count;
  out->tx_frame_count    = info.tx_frame_count;
  out->tx_busy_count     = info.tx_busy_count;
  out->crc_error_count   = info.crc_error_count;
  out->send_error_count  = info.send_error_count;
  out->parse_error_count = info.parse_error_count;
  out->rx_byte_count     = info.rx_byte_count;
  out->rx_overflow_count = info.rx_overflow_count;
  out->rx_drop_count     = info.rx_drop_count;
  out->last_rx_ms        = info.last_rx_ms;
  out->last_tx_ms        = info.last_tx_ms;
  out->last_msg_id       = info.last_msg_id;
}
