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
#include "bsp_config_store.h"
#include "lora_e22_config.h"
#include "px4lite_heartbeat_policy.h"
#include "px4lite_config.h"
#include "px4lite_diagnostic_capture.h"
#include "px4lite_imu_axis_map.h"
#include "bsp_gnss.h"
#include "bsp_lora.h"
#include "bsp_remoteid.h"
#include "bsp_button.h"
#include "bsp_pwm.h"
#include "bsp_rtc.h"
#include "bsp_uart.h"
#include "bsp_watchdog.h"
#include "lora_e22.h"
#include "bsp_time.h"
#include "sensor_bme280.h"
#include "sensor_gnss.h"
#include "sensor_mpu6050.h"
#include "sensor_power.h"
#include "sensor_power2.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#if defined(__CC_ARM)
#define MAVLINK_ALIGNED_FIELDS   0
#define MAVLINK_COMM_NUM_BUFFERS 1
#pragma diag_suppress 66
#endif

#include "common/mavlink.h"
#if defined(__CC_ARM)
#pragma diag_default 66
#endif

#include <math.h>
#include <string.h>

static volatile uint8_t s_gnss_reinit_pending; /**< Health 只置位，由 sensor 消费。 */
static uint32_t s_gnss_last_rx_sequence; /**< 适配器已消费的驱动链路序号，重初始化后复位。 */

static uint32_t s_heartbeat_ms[PX4LITE_HEARTBEAT_COUNT];
static uint32_t s_heartbeat_seen_mask;
static uint32_t s_heartbeat_timeout_ms[PX4LITE_HEARTBEAT_COUNT]; /**< 创建者提交的期限，单位 ms。 */
static uint32_t s_heartbeat_required_mask;
static Px4Lite_WatchdogPhase_t s_watchdog_phase;
static uint32_t s_watchdog_start_ms;
static uint32_t s_watchdog_last_check_ms; /**< 上次策略检查时刻，单位 ms；监视毫秒时基进展。 */
static uint8_t s_heartbeat_config_sealed; /**< 调度器开始后不允许修改必需集合。 */
static uint32_t s_baro_last_sample_ms;
static int32_t s_baro_last_altitude_mm;
static int32_t s_baro_vertical_speed_cms;
static uint8_t s_baro_altitude_valid;

#if PX4LITE_ENABLE_RPI_MAVLINK
#define PX4LITE_RPI_RX_SERVICE_BYTE_BUDGET 128U
#define PX4LITE_RPI_RX_IDLE_RESET_MS 200U /**< 半帧最大字节间隔，单位 ms。 */

static mavlink_status_t s_rpi_parse_status;
static mavlink_message_t s_rpi_parse_msg;
static uint32_t s_rpi_pending_rx_ms;
static uint32_t s_rpi_last_byte_ms; /**< 最近消费字节时刻；解析状态单独表示半帧存在。 */
static uint8_t s_rpi_pending_frame;
static uint8_t s_rpi_diagnostic_raw[PX4LITE_CAPTURE_FRAME_BYTES];
static uint16_t s_rpi_diagnostic_length;
#endif

/** @brief Driver原字节事实经Adapter转换，捕获不接触生产队列。 */
static void PlatformCaptureLoRaRx(const uint8_t *bytes, uint16_t length, uint32_t now, uint8_t valid)
{ if (valid) { Px4Lite_CaptureRx(PX4LITE_CAPTURE_LORA_RX, bytes, length, now); } else { Px4Lite_CaptureReject(PX4LITE_CAPTURE_LORA_RX); } }
/** @brief Driver阶段与Framework阶段明确逐项转换，无ISR解析。 */
static void PlatformCaptureLoRaTx(uint8_t stage)
{
  switch (stage) {
    case 1U: Px4Lite_CaptureTxStage(PX4LITE_CAPTURE_QUEUED); break;
    case 2U: Px4Lite_CaptureTxStage(PX4LITE_CAPTURE_UART_DONE); break;
    case 3U: Px4Lite_CaptureTxStage(PX4LITE_CAPTURE_MODULE_IDLE); break;
    default: Px4Lite_CaptureTxStage(PX4LITE_CAPTURE_ERROR); break;
  }
}

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

/**
 * @brief 将整数限制到闭区间内。
 */
static int32_t Px4Lite_ClampI32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value) { return min_value; }
  if (value > max_value) { return max_value; }
  return value;
}

/**
 * @brief 将标准大气模型下的气压转换为海拔高度。
 */
static int32_t Px4Lite_BaroPressureToAltitudeMm(float pressure_pa)
{
  float ratio;
  float altitude_m;

  if (pressure_pa <= 0.0f) { return 0; }

  ratio      = pressure_pa / PX4LITE_BARO_SEA_LEVEL_PA;
  altitude_m = 44330.0f * (1.0f - powf(ratio, 0.19029495f));
  return Px4Lite_RoundFloatToI32(altitude_m * 1000.0f);
}

/**
 * @brief 更新气压高度差分垂直速度，使用限幅和一阶滤波抑制跳变。
 */
static int32_t Px4Lite_BaroUpdateVerticalSpeed(int32_t altitude_mm, uint32_t sample_time_ms)
{
  int32_t velocity_cms = 0;
  uint32_t dt_ms;

  if ((s_baro_altitude_valid == 0U) || (s_baro_last_sample_ms == 0U) || (sample_time_ms <= s_baro_last_sample_ms)) {
    s_baro_altitude_valid     = 1U;
    s_baro_last_sample_ms     = sample_time_ms;
    s_baro_last_altitude_mm   = altitude_mm;
    s_baro_vertical_speed_cms = 0;
    return 0;
  }

  dt_ms = sample_time_ms - s_baro_last_sample_ms;
  if (dt_ms <= PX4LITE_BARO_MAX_AGE_MS) {
    int32_t delta_mm = altitude_mm - s_baro_last_altitude_mm;

    if (delta_mm > PX4LITE_BARO_MAX_JUMP_MM) { delta_mm = PX4LITE_BARO_MAX_JUMP_MM; }
    if (delta_mm < -PX4LITE_BARO_MAX_JUMP_MM) { delta_mm = -PX4LITE_BARO_MAX_JUMP_MM; }

    velocity_cms = (int32_t)(((int64_t)delta_mm * 100) / (int64_t)dt_ms);
    velocity_cms = Px4Lite_ClampI32(velocity_cms, -PX4LITE_BARO_MAX_VERTICAL_CMS, PX4LITE_BARO_MAX_VERTICAL_CMS);
    s_baro_vertical_speed_cms += (velocity_cms - s_baro_vertical_speed_cms) / 4;
  } else {
    s_baro_vertical_speed_cms = 0;
  }

  s_baro_last_sample_ms   = sample_time_ms;
  s_baro_last_altitude_mm = altitude_mm;
  return s_baro_vertical_speed_cms;
}


/**
 * @brief 复位平台心跳状态并初始化适配层拥有的服务。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_PlatformInit(void)
{
  Px4Lite_CaptureInit();
  Lora_E22_SetDiagnosticCallbacks(PlatformCaptureLoRaRx, PlatformCaptureLoRaTx);
  memset(s_heartbeat_ms, 0, sizeof(s_heartbeat_ms));
  s_heartbeat_seen_mask = 0U;
  s_heartbeat_required_mask = 0U;
  s_heartbeat_config_sealed = 0U;
  s_watchdog_phase = PX4LITE_WATCHDOG_OFF;
  s_watchdog_start_ms = 0U;
  s_watchdog_last_check_ms = 0U;
  memset(s_heartbeat_timeout_ms, 0, sizeof(s_heartbeat_timeout_ms));
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
 * @brief 读取 STM32F407 96-bit 硬件唯一 ID。
 *
 * @param[out] uid_words 输出 UID word0/word1/word2。
 * @param[in] word_capacity 输出缓冲区 word 容量。
 *
 * @return 读取结果。
 */
Px4Lite_Result_t Px4Lite_PlatformGetHardwareUid(uint32_t *uid_words, uint8_t word_capacity)
{
  if ((uid_words == 0) || (word_capacity < 3U)) { return PX4LITE_INVALID_PARAM; }

  uid_words[0] = HAL_GetUIDw0();
  uid_words[1] = HAL_GetUIDw1();
  uid_words[2] = HAL_GetUIDw2();
  return PX4LITE_OK;
}

/**
 * @brief 调度前配置实际创建任务的固定心跳槽位；禁止重复、越界及运行时改配置。
 * @param[in] id 固定心跳编号。
 * @param[in] period_ms 正常工作周期，单位 ms。
 * @param[in] min_timeout_ms 初始化/重试等路径要求的最小期限，单位 ms。
 * @return 参数非法或已封存时拒绝，成功后该槽位成为必需项。
 */
Px4Lite_Result_t Px4Lite_PlatformHeartbeatConfigure(Px4Lite_HeartbeatId_t id, uint32_t period_ms, uint32_t min_timeout_ms)
{
  uint32_t timeout = Px4Lite_HeartbeatTimeoutMs(period_ms, min_timeout_ms);
  if (((uint32_t)id >= PX4LITE_HEARTBEAT_COUNT) || (timeout == 0U)) { return PX4LITE_INVALID_PARAM; }
  if ((s_heartbeat_config_sealed != 0U) || ((s_heartbeat_required_mask & (1UL << id)) != 0U)) { return PX4LITE_NOT_READY; }
  s_heartbeat_timeout_ms[id] = timeout;
  s_heartbeat_required_mask |= 1UL << id;
  return PX4LITE_OK;
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
  s_heartbeat_config_sealed = 1U;
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
  taskENTER_CRITICAL();
  memcpy(heartbeat, s_heartbeat_ms, sizeof(heartbeat));
  seen_mask = s_heartbeat_seen_mask;
  taskEXIT_CRITICAL();
  /* 期限与必需位图已在调度前固定，运行期无需重复复制。 */
  return Px4Lite_HeartbeatsCheck(heartbeat, s_heartbeat_timeout_ms, PX4LITE_HEARTBEAT_COUNT,
                               seen_mask, s_heartbeat_required_mask, now_ms);
}

/**
 * @brief 调度器启动前封存任务集合并开始首轮期限，不能重复交接。
 * @param[in] now_ms 当前真实毫秒时间。
 * @return 空集合/重复交接/硬件未启动时失败；失败不再续期。
 */
Px4Lite_Result_t Px4Lite_PlatformWatchdogBeginStartup(uint32_t now_ms)
{
  if ((s_heartbeat_config_sealed != 0U) || (s_heartbeat_required_mask == 0U)) {
    s_watchdog_phase = PX4LITE_WATCHDOG_FAILED;
    BSP_Watchdog_SealBootProgress();
    return PX4LITE_NOT_READY;
  }
#if PX4LITE_ENABLE_HARDWARE_WATCHDOG
  if (BSP_Watchdog_IsStarted() == 0U) {
    s_watchdog_phase = PX4LITE_WATCHDOG_FAILED;
    BSP_Watchdog_SealBootProgress();
    return PX4LITE_NOT_READY;
  }
  BSP_Watchdog_BootProgress();
#endif
  BSP_Watchdog_SealBootProgress();
  s_heartbeat_config_sealed = 1U;
  s_watchdog_start_ms = now_ms;
  s_watchdog_last_check_ms = now_ms;
  s_watchdog_phase = PX4LITE_WATCHDOG_WAITING;
  return PX4LITE_OK;
}

/** @brief HealthTask 完成工作并更新心跳后执行；时基停走不续期，首轮超时/硬件切换失败锁存。 */
void Px4Lite_PlatformWatchdogFeed(uint32_t now_ms)
{
  Px4Lite_WatchdogAction_t action = Px4Lite_WatchdogEvaluate(&s_watchdog_phase, &s_watchdog_last_check_ms, s_watchdog_start_ms,
                                                          now_ms, Px4Lite_PlatformHeartbeatsHealthy(now_ms));
  if (action == PX4LITE_WATCHDOG_ENTER_RUNTIME) {
#if PX4LITE_ENABLE_HARDWARE_WATCHDOG
    if (BSP_Watchdog_EnterRuntime() != BSP_STATUS_OK) {
      s_watchdog_phase = PX4LITE_WATCHDOG_FAILED;
      return;
    }
#endif
    s_watchdog_phase = PX4LITE_WATCHDOG_RUNTIME;
  } else if (action == PX4LITE_WATCHDOG_FEED) {
#if PX4LITE_ENABLE_HARDWARE_WATCHDOG
    BSP_WatchdogRefresh();
#endif
  }
}

/**
 * @brief 初始化 GNSS BSP 和强类型 GNSS 解析器。
 */

Px4Lite_Result_t Px4Lite_GnssInit(void)
{
  s_gnss_last_rx_sequence = 0U;
  s_gnss_reinit_pending = 0U;
  /* GNSS UART/DMA 由 BSP_Init() 集中初始化；此处只初始化解析器状态。 */
  return (Sensor_GNSS_Init() == GNSS_RESULT_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

void Px4Lite_GnssRequestReinit(void)
{
  s_gnss_reinit_pending = 1U;
}

/**
 * @brief 将一个最新 GNSS 驱动快照转换为 Framework 测量格式。
 */
Px4Lite_Result_t Px4Lite_GnssRead(Px4Lite_SensorGnss_t *measurement)
{
  uint32_t now_ms;
  uint8_t reinit;
  uint32_t best_age = UINT32_MAX;
  uint32_t age;
  uint32_t times[4];
  uint32_t i;
  static const uint32_t bits[4] = {GNSS_VALID_POSITION, GNSS_VALID_VELOCITY, GNSS_VALID_TIME, GNSS_VALID_ALTITUDE};
  Gnss_Snapshot_t snapshot;
  Gnss_Result_t result;

  if (measurement == 0) { return PX4LITE_INVALID_PARAM; }

  taskENTER_CRITICAL();
  reinit = s_gnss_reinit_pending;
  s_gnss_reinit_pending = 0U;
  taskEXIT_CRITICAL();
  if (reinit != 0U) {
    s_gnss_last_rx_sequence = 0U;
    Sensor_GNSS_RequestReinit();
  }
  now_ms = Px4Lite_PlatformGetMs();
  if (Sensor_GNSS_Service(now_ms) == GNSS_RESULT_IO_ERROR) { return PX4LITE_IO_ERROR; }
  result = Sensor_GNSS_CopySnapshot(&snapshot);
  if (result == GNSS_RESULT_INVALID_PARAM) { return PX4LITE_IO_ERROR; }

  if ((snapshot.rx_sequence == 0U) || (snapshot.rx_sequence == s_gnss_last_rx_sequence)) { return PX4LITE_IDLE; }

  memset(measurement, 0, sizeof(*measurement));
  measurement->last_rx_ms = snapshot.last_rx_ms;
  measurement->position_update_ms = snapshot.position_update_ms;
  measurement->velocity_update_ms = snapshot.velocity_update_ms;
  measurement->time_update_ms = snapshot.time_update_ms;
  measurement->altitude_update_ms = snapshot.altitude_update_ms;
  if ((snapshot.valid_mask & GNSS_VALID_POSITION) != 0U) { measurement->valid_mask |= PX4LITE_GNSS_VALID_POSITION; }
  if ((snapshot.valid_mask & GNSS_VALID_VELOCITY) != 0U) { measurement->valid_mask |= PX4LITE_GNSS_VALID_VELOCITY; }
  if ((snapshot.valid_mask & GNSS_VALID_TIME) != 0U) { measurement->valid_mask |= PX4LITE_GNSS_VALID_TIME; }
  if ((snapshot.valid_mask & GNSS_VALID_ALTITUDE) != 0U) { measurement->valid_mask |= PX4LITE_GNSS_VALID_ALTITUDE; }
  times[0] = snapshot.position_update_ms; times[1] = snapshot.velocity_update_ms;
  times[2] = snapshot.time_update_ms; times[3] = snapshot.altitude_update_ms;
  for (i = 0U; i < 4U; ++i) {
    age = Px4Lite_ElapsedMs(now_ms, times[i]);
    if (((snapshot.valid_mask & bits[i]) != 0U) && (age < best_age)) { best_age = age; measurement->header.sample_time_ms = times[i]; }
  }
  measurement->latitude_e7           = snapshot.latitude_deg_e7;
  measurement->longitude_e7          = snapshot.longitude_deg_e7;
  measurement->altitude_mm           = snapshot.altitude_mm;
  measurement->ground_speed_cms      = snapshot.speed_cms;
  measurement->utc_sec               = snapshot.utc_sec;
  measurement->utc_date              = snapshot.utc_date;
  measurement->heading_deg100        = (uint16_t)snapshot.heading_deg100;
  measurement->hdop_x100             = snapshot.hdop_cm;
  measurement->fix_type              = ((snapshot.fix_valid != 0U) && (Px4Lite_ElapsedMs(now_ms, snapshot.position_update_ms) <= PX4LITE_GNSS_MAX_AGE_MS)) ? snapshot.fix_quality : 0U;
  measurement->fix_dimension         = snapshot.fix_dimension;
  measurement->satellites_used       = snapshot.satellites;
  measurement->gps_visible           = snapshot.gps_visible_sats;
  measurement->bds_visible           = snapshot.bds_visible_sats;
  measurement->gps_used              = snapshot.gps_used_sats;
  measurement->bds_used              = snapshot.bds_used_sats;

  s_gnss_last_rx_sequence = snapshot.rx_sequence;
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
    /* 单帧采样失败只代表"本轮没有新样本"，不重置稳定计数：稳定计数的唯一用途是
       在重初始化之后等待若干帧再发布(见下方 reinit_count 判定)。若在这里一并清零，
       任何一次瞬时拒绝都会让计数在 0..MIN-1 之间反复横跳而永远到不了门限，
       last_rx_ms 随之冻结，Health 在 PX4LITE_IMU_OFFLINE_MS 后误判 IMU 离线。 */
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
  Px4Lite_Result_t result = (Sensor_BME280_Init() == BME280_RESULT_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;

  s_baro_last_sample_ms     = 0U;
  s_baro_last_altitude_mm   = 0;
  s_baro_vertical_speed_cms = 0;
  s_baro_altitude_valid     = 0U;
  return result;
}

void Px4Lite_BaroRequestReinit(void)
{
  Sensor_BME280_RequestReinit();
}

/** @brief 每轮推进 BME280，只在触发许可时开启新测量；发布时保留驱动采样时间。 */
Px4Lite_Result_t Px4Lite_BaroRead(Px4Lite_SensorBaro_t *measurement, uint8_t start_measurement)
{
  static uint32_t last_rx_sequence;
  Bme280_Snapshot_t snapshot;
  Bme280_Result_t result;
  uint32_t now_ms;

  if (measurement == 0) { return PX4LITE_INVALID_PARAM; }

  now_ms = Px4Lite_PlatformGetMs();
  result = Sensor_BME280_Service(now_ms, start_measurement);
  if (result != BME280_RESULT_OK) { return (result == BME280_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }

  result = Sensor_BME280_CopySnapshot(&snapshot);
  if (result != BME280_RESULT_OK) { return (result == BME280_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }

  if ((snapshot.rx_sequence == 0U) || (snapshot.rx_sequence == last_rx_sequence)) { return PX4LITE_IDLE; }

  memset(measurement, 0, sizeof(*measurement));
  measurement->header.sample_time_ms = snapshot.sample_time_ms;
  measurement->pressure_pa           = snapshot.pressure_pa;
  measurement->temperature_c         = snapshot.temperature_c;
  measurement->relative_humidity_pct = snapshot.relative_humidity_pct;
  if ((snapshot.pressure_pa >= PX4LITE_BARO_PRESSURE_MIN_PA) && (snapshot.pressure_pa <= PX4LITE_BARO_PRESSURE_MAX_PA)) {
    measurement->pressure_altitude_mm = Px4Lite_BaroPressureToAltitudeMm(snapshot.pressure_pa);
    measurement->vertical_speed_cms   = Px4Lite_BaroUpdateVerticalSpeed(measurement->pressure_altitude_mm, measurement->header.sample_time_ms);
  }

  last_rx_sequence = snapshot.rx_sequence;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_BatteryInit(void)
{
  /* 第二块电池(ADC2/PA4)与电池 1 同周期采样，独立缓存电压与电量；缺装时不阻塞电池 1。 */
  (void)Sensor_Power2_Init();
  return (Sensor_Power_Init() == POWER_RESULT_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

void Px4Lite_BatteryRequestReinit(void)
{
  Sensor_Power_RequestReinit();
  Sensor_Power2_RequestReinit();
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
  measurement->current_ma            = snapshot.current_ma;
  measurement->current_valid         = snapshot.current_valid;
  measurement->percent               = snapshot.percent;
  measurement->low_voltage           = snapshot.low_voltage;

  last_rx_sequence = snapshot.rx_sequence;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_Battery2Read(Px4Lite_BatteryStatus_t *measurement)
{
  static uint32_t last_rx_sequence;
  Power_Snapshot_t snapshot;
  Power_Result_t result;
  uint32_t now_ms;

  if (measurement == 0) { return PX4LITE_INVALID_PARAM; }

  now_ms = Px4Lite_PlatformGetMs();
  result = Sensor_Power2_Service(now_ms);
  if (result != POWER_RESULT_OK) { return (result == POWER_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }

  result = Sensor_Power2_CopySnapshot(&snapshot);
  if (result != POWER_RESULT_OK) { return (result == POWER_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }

  if ((snapshot.rx_sequence == 0U) || (snapshot.rx_sequence == last_rx_sequence)) { return PX4LITE_IDLE; }

  memset(measurement, 0, sizeof(*measurement));
  measurement->header.sample_time_ms = snapshot.sample_time_ms;
  measurement->voltage_mv            = (uint32_t)((snapshot.voltage_v * 1000.0f) + 0.5f);
  measurement->current_ma            = snapshot.current_ma;
  measurement->current_valid         = snapshot.current_valid;
  measurement->percent               = snapshot.percent;
  measurement->low_voltage           = snapshot.low_voltage;

  last_rx_sequence = snapshot.rx_sequence;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_MotorInit(void)
{
  if (BSP_PWM_Init() != BSP_STATUS_OK) { return PX4LITE_IO_ERROR; }
  return Px4Lite_MotorDisarmAll();
}

Px4Lite_Result_t Px4Lite_MotorWritePulseUs(uint8_t channel, uint16_t pulse_us)
{
  if (channel >= PX4LITE_MOTOR_COUNT) { return PX4LITE_INVALID_PARAM; }
  return (BSP_PWM_SetPulseUs((BSP_PwmChannel_t)channel, pulse_us) == BSP_STATUS_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

Px4Lite_Result_t Px4Lite_MotorDisarmAll(void)
{
  uint8_t i;
  Px4Lite_Result_t result = PX4LITE_OK;

  for (i = 0U; i < PX4LITE_MOTOR_COUNT; i++) {
    if (Px4Lite_MotorWritePulseUs(i, PX4LITE_CONTROL_ESC_MIN_PULSE_US) != PX4LITE_OK) { result = PX4LITE_IO_ERROR; }
  }
  return result;
}

uint8_t Px4Lite_ButtonPressed(Px4Lite_ButtonId_t button)
{
  if (button >= PX4LITE_BUTTON_COUNT) { return 0U; }
  return BSP_Button_IsPressed((BSP_ButtonId_t)button);
}

Px4Lite_Result_t Px4Lite_LoRaInit(void)
{
  Lora_Result_t result;

  if (BSP_LoRa_Init() != 0) { return PX4LITE_IO_ERROR; }
  result = Lora_E22_Init();
  if (result == LORA_RESULT_OK) { return PX4LITE_OK; }
  if (result == LORA_RESULT_BUSY) { return PX4LITE_BUSY; }
  return PX4LITE_IO_ERROR;
}

void Px4Lite_LoRaRequestReinit(void)
{
  Lora_E22_RequestReinit();
}

Px4Lite_Result_t Px4Lite_LoRaService(uint32_t now_ms)
{
  Lora_Result_t result = Lora_E22_Service(now_ms);
  if (result == LORA_RESULT_OK) { return PX4LITE_OK; }
  if (result == LORA_RESULT_BUSY) { return PX4LITE_BUSY; }
  return PX4LITE_IO_ERROR;
}

Px4Lite_Result_t Px4Lite_LoRaSend(const uint8_t *data, uint16_t len)
{
  Lora_Result_t result;
  uint32_t token = 0U;
  Px4Lite_Result_t mapped;

  if ((data == 0) || (len == 0U)) { return PX4LITE_INVALID_PARAM; }

  if (Lora_E22_IsTransportIdle()) { token = Px4Lite_CaptureTxBegin(data, len, Px4Lite_PlatformGetMs()); }
  result = Lora_E22_Send(data, len);
  mapped = result == LORA_RESULT_OK ? PX4LITE_OK : (result == LORA_RESULT_BUSY ? PX4LITE_BUSY : (result == LORA_RESULT_INVALID_PARAM ? PX4LITE_INVALID_PARAM : PX4LITE_IO_ERROR));
  Px4Lite_CaptureTxSubmit(token, mapped);
  return mapped;
}

uint8_t Px4Lite_LoRaIsTxIdle(void)
{
  return Lora_E22_IsTxIdle();
}

Px4Lite_Result_t Px4Lite_RemoteIdInit(void)
{
  return (BSP_RemoteId_Init() == 0) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

uint8_t Px4Lite_RemoteIdIsReady(void)
{
  return BSP_RemoteId_IsReady();
}

void Px4Lite_RemoteIdAbortTx(void)
{
  BSP_RemoteId_AbortTx();
}

Px4Lite_Result_t Px4Lite_RemoteIdSend(const uint8_t *data, uint16_t len)
{
  int32_t result;

  if ((data == 0) || (len == 0U)) { return PX4LITE_INVALID_PARAM; }

  result = BSP_RemoteId_StartSend(data, len);
  if (result == 0) { return PX4LITE_OK; }
  if (result == 1) { return PX4LITE_BUSY; }
  return PX4LITE_IO_ERROR;
}

#if PX4LITE_ENABLE_RPI_MAVLINK
/** @brief 初始化或切换串口后清理真实 MAVLink 接收状态，不能只清输出副本。 */
static void Px4Lite_RpiMavlinkResetRxState(void)
{
  memset(mavlink_get_channel_status(MAVLINK_COMM_0), 0, sizeof(mavlink_status_t));
  memset(mavlink_get_channel_buffer(MAVLINK_COMM_0), 0, sizeof(mavlink_message_t));
  memset(&s_rpi_parse_status, 0, sizeof(s_rpi_parse_status));
  memset(&s_rpi_parse_msg, 0, sizeof(s_rpi_parse_msg));
  s_rpi_pending_rx_ms = 0U;
  s_rpi_pending_frame = 0U;
  s_rpi_last_byte_ms = 0U;
  s_rpi_diagnostic_length = 0U;
}
#endif

Px4Lite_Result_t Px4Lite_RpiMavlinkInit(void)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  /* 树莓派链路走独立的 USART6(PC6/PC7)，与调试口 USART1 物理分离。 */
  Px4Lite_RpiMavlinkResetRxState();
  return (BSP_RpiUART_Init() == BSP_STATUS_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
#else
  return PX4LITE_OK;
#endif
}

Px4Lite_Result_t Px4Lite_RpiMavlinkSend(const uint8_t *data, uint16_t len)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  BSP_Status_t status;

  if ((data == 0) || (len == 0U)) { return PX4LITE_INVALID_PARAM; }

  status = BSP_RpiUART_Send(data, len, PX4LITE_RPI_MAVLINK_UART_TIMEOUT_MS);
  Px4Lite_CaptureRpiTx(data, len, Px4Lite_PlatformGetMs(), status == BSP_STATUS_OK ? PX4LITE_OK : PX4LITE_IO_ERROR);
  if (status == BSP_STATUS_OK) { return PX4LITE_OK; }
  if (status == BSP_STATUS_BUSY) { return PX4LITE_BUSY; }
  return PX4LITE_IO_ERROR;
#else
  (void)data;
  (void)len;
  return PX4LITE_OK;
#endif
}

/**
 * @brief 通过 Platform Adapter 查询 USART6 实际波特率。
 * @param[out] baud_bps 输出波特率，单位 bit/s。
 * @return Framework 通用返回码。
 */
Px4Lite_Result_t Px4Lite_RpiMavlinkGetBaudRate(uint32_t *baud_bps)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  BSP_Status_t status;

  if (baud_bps == 0) { return PX4LITE_INVALID_PARAM; }
  status = BSP_RpiUART_GetBaudRate(baud_bps);
  if (status == BSP_STATUS_OK) { return PX4LITE_OK; }
  if (status == BSP_STATUS_BUSY) { return PX4LITE_BUSY; }
  return PX4LITE_IO_ERROR;
#else
  (void)baud_bps;
  return PX4LITE_NOT_READY;
#endif
}

/**
 * @brief 通过 Platform Adapter 切换 USART6 波特率并丢弃旧半帧。
 * @param[in] baud_bps 目标波特率，单位 bit/s。
 * @return Framework 通用返回码。
 */
Px4Lite_Result_t Px4Lite_RpiMavlinkSetBaudRate(uint32_t baud_bps)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  BSP_Status_t status;

  if (baud_bps == 0U) { return PX4LITE_INVALID_PARAM; }
  status = BSP_RpiUART_SetBaudRate(baud_bps);
  /* 切换及失败回滚都会截断当前字节流，不能保留旧速率下的 MAVLink 半帧。 */
  Px4Lite_RpiMavlinkResetRxState();
  if (status == BSP_STATUS_OK) { return PX4LITE_OK; }
  if (status == BSP_STATUS_BUSY) { return PX4LITE_BUSY; }
  return PX4LITE_IO_ERROR;
#else
  (void)baud_bps;
  return PX4LITE_NOT_READY;
#endif
}

Px4Lite_Result_t Px4Lite_RpiMavlinkService(uint32_t now_ms)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  uint8_t byte;
  uint16_t i;

  if (s_rpi_pending_frame != 0U) { return PX4LITE_BUSY; }

  for (i = 0U; i < PX4LITE_RPI_RX_SERVICE_BYTE_BUDGET; ++i) {
    uint8_t framing;
    if (BSP_RpiUART_Read(&byte, 1U) == 0U) { return (i == 0U) ? PX4LITE_IDLE : PX4LITE_OK; }

    if ((mavlink_get_channel_status(MAVLINK_COMM_0)->parse_state != MAVLINK_PARSE_STATE_IDLE) &&
        ((uint32_t)(now_ms - s_rpi_last_byte_ms) > PX4LITE_RPI_RX_IDLE_RESET_MS)) {
      Px4Lite_RpiMavlinkResetRxState();
    }
    s_rpi_last_byte_ms = now_ms;
    if (mavlink_get_channel_status(MAVLINK_COMM_0)->parse_state <= MAVLINK_PARSE_STATE_IDLE) {
      s_rpi_diagnostic_length = 0U;
      if (byte == 0xFD || byte == 0xFE) { s_rpi_diagnostic_raw[s_rpi_diagnostic_length++] = byte; }
    } else if (s_rpi_diagnostic_length && s_rpi_diagnostic_length < sizeof(s_rpi_diagnostic_raw)) {
      s_rpi_diagnostic_raw[s_rpi_diagnostic_length++] = byte;
    }
    framing = mavlink_frame_char(MAVLINK_COMM_0, byte, &s_rpi_parse_msg, &s_rpi_parse_status);
    if (framing == MAVLINK_FRAMING_BAD_CRC || framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
      mavlink_status_t *parser = mavlink_get_channel_status(MAVLINK_COMM_0);
      mavlink_message_t *message = mavlink_get_channel_buffer(MAVLINK_COMM_0);
      Px4Lite_CaptureReject(PX4LITE_CAPTURE_RPI_RX);
      s_rpi_diagnostic_length = 0U; parser->msg_received = MAVLINK_FRAMING_INCOMPLETE; parser->parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (byte == MAVLINK_STX) { parser->parse_state = MAVLINK_PARSE_STATE_GOT_STX; message->len = 0U; mavlink_start_checksum(message); s_rpi_diagnostic_raw[0] = byte; s_rpi_diagnostic_length = 1U; }
    }
    if (framing == MAVLINK_FRAMING_OK) {
      Px4Lite_CaptureRx(PX4LITE_CAPTURE_RPI_RX, s_rpi_diagnostic_raw, s_rpi_diagnostic_length, now_ms);
      s_rpi_diagnostic_length = 0U;
      s_rpi_pending_rx_ms = now_ms;
      s_rpi_pending_frame = 1U;
      return PX4LITE_OK;
    }
  }

  return PX4LITE_OK;
#else
  (void)now_ms;
  return PX4LITE_IDLE;
#endif
}

Px4Lite_Result_t Px4Lite_RpiMavlinkCopyRxFrame(Px4Lite_CommRxFrame_t *out)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  if (s_rpi_pending_frame == 0U) { return PX4LITE_NOT_READY; }

  memset(out, 0, sizeof(*out));
  out->rx_time_ms   = s_rpi_pending_rx_ms;
  out->msg_id       = s_rpi_parse_msg.msgid;
  out->frame_len    = mavlink_msg_get_send_buffer_length(&s_rpi_parse_msg);
  out->system_id    = s_rpi_parse_msg.sysid;
  out->component_id = s_rpi_parse_msg.compid;
  out->sequence     = s_rpi_parse_msg.seq;
  out->payload_len  = (s_rpi_parse_msg.len > PX4LITE_COMM_RX_PAYLOAD_MAX) ? PX4LITE_COMM_RX_PAYLOAD_MAX : s_rpi_parse_msg.len;
  if (out->payload_len > 0U) { memcpy(out->payload, _MAV_PAYLOAD(&s_rpi_parse_msg), out->payload_len); }
  s_rpi_pending_frame = 0U;
  return PX4LITE_OK;
#else
  (void)out;
  return PX4LITE_NOT_READY;
#endif
}

Px4Lite_State_t Px4Lite_LoRaGetState(uint32_t now_ms)
{
  switch (Lora_E22_GetState(now_ms, PX4LITE_LORA_OFFLINE_MS)) {
    case LORA_STATE_ONLINE:
      return PX4LITE_STATE_ONLINE;
    case LORA_STATE_OFFLINE:
      return PX4LITE_STATE_DEGRADED;
    case LORA_STATE_FAILED:
      return PX4LITE_STATE_FAILED;
    default:
      return PX4LITE_STATE_STARTING;
  }
}

uint8_t Px4Lite_LoRaIsPresent(void)
{
  return Lora_E22_IsPresent();
}

uint8_t Px4Lite_RemoteIdIsPresent(void)
{
#if PX4LITE_ENABLE_REMOTE_ID
  return BSP_RemoteId_IsPresent();
#else
  return 0U;
#endif
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
  out->rx_sequence_expected_count = info.rx_sequence_expected_count;
  out->rx_sequence_lost_count     = info.rx_sequence_lost_count;
  out->last_rx_ms        = info.last_rx_ms;
  out->last_tx_ms        = info.last_tx_ms;
  out->last_msg_id       = info.last_msg_id;
  out->rx_loss_rate_x10  = info.rx_loss_rate_x10;
}

Px4Lite_Result_t Px4Lite_LoRaCopyRxFrame(Px4Lite_CommRxFrame_t *out)
{
  Lora_RxFrame_t frame;
  Lora_Result_t result;

  if (out == 0) { return PX4LITE_INVALID_PARAM; }

  memset(&frame, 0, sizeof(frame));
  result = Lora_E22_CopyRxFrame(&frame);
  if (result == LORA_RESULT_NO_DATA) { return PX4LITE_NOT_READY; }
  if (result == LORA_RESULT_INVALID_PARAM) { return PX4LITE_INVALID_PARAM; }
  if (result != LORA_RESULT_OK) { return PX4LITE_IO_ERROR; }

  memset(out, 0, sizeof(*out));
  out->rx_time_ms   = frame.rx_time_ms;
  out->msg_id       = frame.msg_id;
  out->frame_len    = frame.frame_len;
  out->system_id    = frame.system_id;
  out->component_id = frame.component_id;
  out->sequence     = frame.sequence;
  out->payload_len  = (frame.payload_len > PX4LITE_COMM_RX_PAYLOAD_MAX) ? PX4LITE_COMM_RX_PAYLOAD_MAX : frame.payload_len;
  if (out->payload_len > 0U) { memcpy(out->payload, frame.data, out->payload_len); }

  return PX4LITE_OK;
}

/** @brief 将 BSP 原始编程结果转换为平台结果。 */
static Px4Lite_Result_t ConfigStore_Result(int32_t result)
{ return (result==0)?PX4LITE_OK:((result==1)?PX4LITE_BUSY:PX4LITE_IO_ERROR); }
/** @brief 配置服务的原始读取适配，不解析记录。 */
Px4Lite_Result_t Px4Lite_ConfigStoreRead(uint32_t offset,uint8_t *out,uint16_t length)
{ return ConfigStore_Result(BSP_ConfigStore_Read(offset,out,length)); }
/** @brief 配置服务的单字追加适配，不擦除。 */
Px4Lite_Result_t Px4Lite_ConfigStoreProgramWord(uint32_t offset,uint32_t value)
{ return ConfigStore_Result(BSP_ConfigStore_ProgramWord(offset,value)); }
/** @brief 收取已启动操作，维护锁仍由 Framework 持有。 */
Px4Lite_Result_t Px4Lite_ConfigStoreFinishWord(void)
{ return ConfigStore_Result(BSP_ConfigStore_FinishWord()); }

/** @brief 逐字段转换驱动事实，不把 Driver 类型暴露给 Business。 */
void Px4Lite_LoRaCopyConfig(Px4Lite_LoRaConfig_t *out)
{
  Lora_ConfigSnapshot_t value;
  if(!out) { return; } Lora_E22_CopyConfig(&value);
  out->generation=value.generation; out->busy=value.busy; out->ready=value.ready;
  out->request_id=value.request_id; out->completed_request_id=value.completed_request_id;
  out->completed_result=(uint8_t)value.completed_result; out->completed_write=value.completed_write;
  out->completed_channel=value.completed_channel;
  out->channel_known=value.channel_known; out->channel=value.channel; out->result=(uint8_t)value.result;
  Lora_E22_CopyTxFacts(&out->uart_done,&out->module_idle);
}
/** @brief 仅 CommTask 调用，配置请求与正常传输在驱动内串行。 */
Px4Lite_Result_t Px4Lite_LoRaApplyChannel(uint8_t channel,uint32_t now_ms)
{
  int32_t result=Lora_E22_ConfigRequest(channel,now_ms);
  return (result==0)?PX4LITE_OK:((result==1)?PX4LITE_BUSY:PX4LITE_NOT_READY);
}
