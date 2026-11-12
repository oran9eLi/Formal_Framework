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
#include "px4lite_gnss_raw_uplink.h"
#include "px4lite_bmi088_uplink.h"
#include "px4lite_imu_axis_map.h"
#include "bsp_gnss.h"
#include "bsp_lora.h"
#include "bsp_remoteid.h"
#include "bsp_button.h"
#include "bsp_pwm.h"
#include "bsp_buzzer.h"
#include "bsp_rgb.h"
#include "bsp_rtc.h"
#include "bsp_uart.h"
#include "bsp_watchdog.h"
#include "lora_e22.h"
#include "bsp_time.h"
#include "bsp_adc2.h"
#include "bsp_adc_current.h"
#include "sensor_bme280.h"
#include "sensor_gnss.h"
#include "sensor_mpu6050.h"
#include "sensor_bmi088.h"
#include "sensor_power.h"
#include "sensor_power2.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#if GNSS_RAW_FORWARD_MAX_BYTES != PX4LITE_GNSS_RAW_UPLINK_FRAME_MAX
#error "Driver and Framework RAW forwarding capacities must match"
#endif

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

/** @brief IMU状态仅由sensor写入，Debug/自检只读缓存，不接触设备总线。 */
static Px4Lite_ImuDiagnostics_t s_imu_diagnostics;
static uint8_t s_imu_diagnostics_ready;
static uint32_t s_imu_last_rx_sequence; /**< Adapter已消费的完整样本序号。 */
static uint32_t s_imu_last_reinit_count; /**< 用于重新初始化后的稳定帧门控。 */
static uint32_t s_imu_last_sample_ms; /**< 上一发布样本时间，单位ms。 */
static uint8_t s_imu_stable_valid_count; /**< 本次初始化后的连续有效样本数。 */

#if PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088
typedef Bmi088_Snapshot_t PlatformImuSnapshot_t;
typedef Bmi088_Status_t PlatformImuStatus_t;
typedef Bmi088_Result_t PlatformImuResult_t;
#define PLATFORM_IMU_OK BMI088_RESULT_OK
#define PLATFORM_IMU_NO_DATA BMI088_RESULT_NO_DATA
#define PlatformImu_Service Sensor_BMI088_Service
#define PlatformImu_CopySnapshot Sensor_BMI088_CopySnapshot
#define PlatformImu_GetStatus Sensor_BMI088_GetStatus
#else
typedef Mpu6050_Snapshot_t PlatformImuSnapshot_t;
typedef Mpu6050_Status_t PlatformImuStatus_t;
typedef Mpu6050_Result_t PlatformImuResult_t;
#define PLATFORM_IMU_OK MPU6050_RESULT_OK
#define PLATFORM_IMU_NO_DATA MPU6050_RESULT_NO_DATA
#define PlatformImu_Service Sensor_MPU6050_Service
#define PlatformImu_CopySnapshot Sensor_MPU6050_CopySnapshot
#define PlatformImu_GetStatus Sensor_MPU6050_GetStatus
#endif

#if PX4LITE_ENABLE_BATTERY2
/** @brief sensor单写、Debug只读的主回路采样诊断；临界区内只复制。 */
static Px4Lite_PowerDiagnostics_t s_power2_diagnostics;
static uint8_t s_power2_diagnostics_ready;
#endif

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
/** @brief Comm 独占的协议收发累计事实；不随半帧重置或波特率切换清零。 */
static struct {
  uint32_t rx_valid_frames;  /**< MAVLink 解析器返回 FRAMING_OK 的帧数。 */
  uint32_t rx_frame_errors;  /**< BAD_CRC/BAD_SIGNATURE 次数。 */
  uint32_t tx_bytes;         /**< 完整发送成功的字节数。 */
  uint32_t tx_frames;        /**< 完整发送成功的帧数。 */
  uint32_t tx_failures;      /**< 非 OK 发送结果次数。 */
  uint32_t last_tx_ms;       /**< 最近完整发送时间，ms。 */
  uint8_t tx_seen;           /**< 是否有过完整发送。 */
} s_rpi_monitor;

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
    case 5U: Px4Lite_CaptureTxStage(PX4LITE_CAPTURE_TIMEOUT); break;
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


/** @brief 可选声输出的初始化适配，BSP错误仅返回给试听服务。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void)
{
  return (BSP_Buzzer_Init() == BSP_STATUS_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

/** @brief 原始PWM适配，频率和旋律策略由调用者负责。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t frequency_hz)
{
  return (BSP_Buzzer_SetFrequency(frequency_hz) == BSP_STATUS_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

/** @brief 原始输出结果转换，不将DMA提交成功解释为灯珠在位或已发光。 */
static Px4Lite_Result_t PlatformRgb_Result(BSP_Status_t result)
{
  if (result == BSP_STATUS_OK) { return PX4LITE_OK; }
  if (result == BSP_STATUS_BUSY) { return PX4LITE_BUSY; }
  return PX4LITE_IO_ERROR;
}

/** @brief RGB是可选输出，初始化结果仅由声光服务诊断和恢复。 */
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void)
{
  return PlatformRgb_Result(BSP_RGB_Init());
}

/** @brief 仅释放RGB资源；正常熄灯需先发送完整黑帧。 */
void Px4Lite_PlatformRgbDeInit(void)
{
  (void)BSP_RGB_DeInit();
}

/** @brief 在适配层编码GRB高位优先，BSP仅copy并发送原始比较值。 */
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t red, uint8_t green, uint8_t blue)
{
  uint16_t duty[124];
  uint16_t period, zero, one;
  uint32_t grb = ((uint32_t)green << 16U) | ((uint32_t)red << 8U) | blue;
  uint16_t index;
  if (BSP_RGB_GetTiming(&period, &zero, &one) != BSP_STATUS_OK) { return PX4LITE_NOT_READY; }
  if ((zero == 0U) || (one >= period)) { return PX4LITE_IO_ERROR; }
  for (index = 0U; index < 24U; ++index) {
    duty[index] = ((grb & (1UL << (23U - index))) != 0U) ? one : zero;
  }
  /* 100个零占空比槽约125us；DMA完成前已包含至少120us连续低电平。 */
  for (; index < 124U; ++index) { duty[index] = 0U; }
  return PlatformRgb_Result(BSP_RGB_Send(duty, 124U));
}

/** @brief 每10ms调用，包含旋律结束之后的黑帧完成、超时和恢复观察。 */
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t now_ms)
{
  return PlatformRgb_Result(BSP_RGB_Service(now_ms));
}

/** @brief 保留屏幕NMEA捕获，并将完整原句独立复制给RAW上行；NULL超长通知不重复入Raw统计。 */
static void PlatformCaptureGnssNmea(const uint8_t *bytes, uint16_t length, uint8_t type, uint8_t checksum_valid, uint32_t now)
{
  Px4Lite_CaptureNmea(bytes, length, type, checksum_valid, now);
  if (bytes != NULL && length != 0U) { Px4Lite_GnssRawEnqueue(bytes, length, 1U, type, 0U, 0U, checksum_valid, now); }
}

/**
 * @brief 屏幕与独立RAW出口共用一次完整UBX借用视图，不读取三槽采样代替原帧事件流。
 * @details 屏幕/Debug仍为392B，较长帧可记录屏幕drop；独立RAW接收完整1024B，不截断。
 */
static void PlatformCaptureGnssUbx(const uint8_t *bytes, uint16_t length, uint8_t cls, uint8_t id, uint8_t checksum_valid, uint32_t now)
{
  Px4Lite_CaptureUbx(bytes, length, cls, id, checksum_valid, now);
  if (bytes != NULL && length != 0U) { Px4Lite_GnssRawEnqueue(bytes, length, 2U, 0U, cls, id, checksum_valid, now); }
}

/** @brief 独立记录来源拒绝软件事件，计数不冒充物理空口丢帧。 */
static void PlatformRejectGnssRaw(uint8_t protocol, uint32_t length, uint8_t reason, uint32_t now)
{ Px4Lite_GnssRawSourceReject(protocol, length, reason, now); }

/**
 * @brief 复位平台心跳状态并初始化适配层拥有的服务。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_PlatformInit(void)
{
  Px4Lite_CaptureInit();
  Lora_E22_SetDiagnosticCallbacks(PlatformCaptureLoRaRx, PlatformCaptureLoRaTx);
  Gnss_SetDiagnosticCallback(PlatformCaptureGnssNmea);
  Gnss_SetUbxDiagnosticCallback(PlatformCaptureGnssUbx);
  Gnss_SetRawRejectCallback(PlatformRejectGnssRaw);
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

/** @brief 将Driver来源编码显式转换为Framework独立编码，未知值不得冒充已识别协议来源。 */
static uint8_t PlatformGnssSource(uint8_t source)
{
  switch (source) {
    case GNSS_SOURCE_UBX_PVT: return PX4LITE_GNSS_SOURCE_UBX_PVT;
    case GNSS_SOURCE_NMEA_GGA: return PX4LITE_GNSS_SOURCE_NMEA_GGA;
    case GNSS_SOURCE_NMEA_RMC: return PX4LITE_GNSS_SOURCE_NMEA_RMC;
    case GNSS_SOURCE_NMEA_GSA: return PX4LITE_GNSS_SOURCE_NMEA_GSA;
    case GNSS_SOURCE_NMEA_GSV: return PX4LITE_GNSS_SOURCE_NMEA_GSV;
    case GNSS_SOURCE_UBX_DOP: return PX4LITE_GNSS_SOURCE_UBX_DOP;
    default: return PX4LITE_GNSS_SOURCE_NONE;
  }
}

/**
 * @brief 逐字段转换GNSS五个基础域及独立DOP/计数事实，不跨层发送Driver结构体内存。
 * @details 基础聚合时间保持五域语义；追加事实只由各自present/source/时间授权，不续期位置。
 *          Service的TX/恢复错误不能遮住本次尚未发布的有效RX；无新RX时仍返回错误，
 *          不允许重复使用旧快照给在线心跳续期。
 *          只由sensor采集链调用，不可重入；Driver快照使用owner静态暂存，
 *          每轮清零并逐字段复制到调用者输出，不将暂存地址交给其他任务。
 */
Px4Lite_Result_t Px4Lite_GnssRead(Px4Lite_SensorGnss_t *measurement)
{
  uint32_t now_ms;
  uint8_t reinit;
  uint32_t best_age = UINT32_MAX;
  uint32_t age;
  uint32_t times[5];
  uint32_t i;
  static const uint32_t bits[5] = {GNSS_VALID_POSITION, GNSS_VALID_VELOCITY, GNSS_VALID_TIME, GNSS_VALID_ALTITUDE, GNSS_VALID_NAV_STATUS};
  /** @brief sensor所有者独占的Driver快照，避免在嵌套解析路径上保留200B栈对象。 */
  static Gnss_Snapshot_t snapshot;
  Gnss_Result_t result;
  Gnss_Result_t service_result;

  if (measurement == 0) { return PX4LITE_INVALID_PARAM; }
  memset(&snapshot,0,sizeof(snapshot));

  taskENTER_CRITICAL();
  reinit = s_gnss_reinit_pending;
  s_gnss_reinit_pending = 0U;
  taskEXIT_CRITICAL();
  if (reinit != 0U) {
    s_gnss_last_rx_sequence = 0U;
    Sensor_GNSS_RequestReinit();
  }
  now_ms = Px4Lite_PlatformGetMs();
  service_result = Sensor_GNSS_Service(now_ms);
  result = Sensor_GNSS_CopySnapshot(&snapshot);
  if (result == GNSS_RESULT_NO_DATA) {
    return service_result == GNSS_RESULT_IO_ERROR ? PX4LITE_IO_ERROR : PX4LITE_IDLE;
  }
  if (result != GNSS_RESULT_OK && result != GNSS_RESULT_NO_FIX) { return PX4LITE_IO_ERROR; }

  if ((snapshot.rx_sequence == 0U) || (snapshot.rx_sequence == s_gnss_last_rx_sequence)) {
    return service_result == GNSS_RESULT_IO_ERROR ? PX4LITE_IO_ERROR : PX4LITE_IDLE;
  }

  memset(measurement, 0, sizeof(*measurement));
  measurement->last_rx_ms = snapshot.last_rx_ms;
  measurement->position_update_ms = snapshot.position_update_ms;
  measurement->velocity_update_ms = snapshot.velocity_update_ms;
  measurement->time_update_ms = snapshot.time_update_ms;
  measurement->altitude_update_ms = snapshot.altitude_update_ms;
  measurement->status_update_ms = snapshot.status_update_ms;
  if ((snapshot.valid_mask & GNSS_VALID_POSITION) != 0U) { measurement->valid_mask |= PX4LITE_GNSS_VALID_POSITION; }
  if ((snapshot.valid_mask & GNSS_VALID_VELOCITY) != 0U) { measurement->valid_mask |= PX4LITE_GNSS_VALID_VELOCITY; }
  if ((snapshot.valid_mask & GNSS_VALID_TIME) != 0U) { measurement->valid_mask |= PX4LITE_GNSS_VALID_TIME; }
  if ((snapshot.valid_mask & GNSS_VALID_ALTITUDE) != 0U) { measurement->valid_mask |= PX4LITE_GNSS_VALID_ALTITUDE; }
  if ((snapshot.valid_mask & GNSS_VALID_NAV_STATUS) != 0U) { measurement->valid_mask |= PX4LITE_GNSS_VALID_NAV_STATUS; }
  times[0] = snapshot.position_update_ms; times[1] = snapshot.velocity_update_ms;
  times[2] = snapshot.time_update_ms; times[3] = snapshot.altitude_update_ms;
  times[4] = snapshot.status_update_ms;
  for (i = 0U; i < 5U; ++i) {
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
  if ((snapshot.valid_mask & GNSS_VALID_NAV_STATUS) != 0U) {
    measurement->fix_type = (Px4Lite_ElapsedMs(now_ms, snapshot.status_update_ms) <= PX4LITE_GNSS_MAX_AGE_MS) ? snapshot.fix_quality : 0U;
  } else {
    /* 未提供新状态域的旧来源仍保持原位置有效性语义，不凭ACK构造新定位状态。 */
    measurement->fix_type = ((snapshot.fix_valid != 0U) && (Px4Lite_ElapsedMs(now_ms, snapshot.position_update_ms) <= PX4LITE_GNSS_MAX_AGE_MS)) ? snapshot.fix_quality : 0U;
  }
  measurement->fix_dimension         = snapshot.fix_dimension;
  measurement->satellites_used       = snapshot.satellites;
  measurement->gps_visible           = snapshot.gps_visible_sats;
  measurement->bds_visible           = snapshot.bds_visible_sats;
  measurement->gps_used              = snapshot.gps_used_sats;
  measurement->bds_used              = snapshot.bds_used_sats;
  measurement->position_source = PlatformGnssSource(snapshot.position_source);
  measurement->velocity_source = PlatformGnssSource(snapshot.velocity_source);
  measurement->time_source = PlatformGnssSource(snapshot.time_source);
  measurement->altitude_source = PlatformGnssSource(snapshot.altitude_source);
  measurement->status_source = PlatformGnssSource(snapshot.status_source);
  measurement->raw_status_type = snapshot.raw_status_type;
  measurement->dimension_source = PlatformGnssSource(snapshot.dimension_source);
  measurement->dimension_update_ms = snapshot.dimension_update_ms;
  measurement->dop_present_mask = snapshot.dop_present_mask & 0x7FU;
  measurement->dop_itow_known_mask = snapshot.dop_itow_known_mask & measurement->dop_present_mask;
  measurement->dop_solution_mask = snapshot.dop_solution_mask & measurement->dop_present_mask;
  for (i = 0U; i < PX4LITE_GNSS_DOP_COUNT; ++i) {
    measurement->dop_source[i] = PlatformGnssSource(snapshot.dop_source[i]);
    measurement->dop_x100[i] = snapshot.dop_x100[i];
    measurement->dop_update_ms[i] = snapshot.dop_update_ms[i];
    measurement->dop_itow_ms[i] = snapshot.dop_itow_ms[i];
  }
  measurement->counts_present_mask = snapshot.counts_present_mask & 0x0FU;
  for (i = 0U; i < PX4LITE_GNSS_COUNTS_COUNT; ++i) {
    measurement->counts_source[i] = PlatformGnssSource(snapshot.counts_source[i]);
    measurement->counts_update_ms[i] = snapshot.counts_update_ms[i];
  }

  s_gnss_last_rx_sequence = snapshot.rx_sequence;
  return PX4LITE_OK;
}

/**
 * @brief 复制完整线帧到调用者缓冲区，仅用小元数据局部变量，避免大栈副本。
 */
Px4Lite_Result_t Px4Lite_GnssCopyRawFrame(uint8_t slot, Px4Lite_GnssRawFrame_t *out)
{
  if ((out == NULL) || (slot >= PX4LITE_GNSS_RAW_SLOT_COUNT)) { return PX4LITE_INVALID_PARAM; }
  memset(out, 0, sizeof(*out));
#if PX4LITE_ENABLE_GNSS
  {
    Gnss_RawFrameInfo_t info;
    Gnss_Result_t result;
    memset(&info, 0, sizeof(info));
    result = Sensor_GNSS_CopyRawFrame(slot, out->bytes, sizeof(out->bytes), &info);
    if (result != GNSS_RESULT_OK) { return PX4LITE_NOT_READY; }
    if ((info.length == 0U) || (info.length > sizeof(out->bytes))) { return PX4LITE_NOT_READY; }
    out->sequence = info.sequence;
    out->received_ms = info.received_ms;
    out->length = info.length;
    out->protocol = info.protocol;
    out->class_id = info.class_id;
    out->message_id = info.message_id;
    out->checksum_valid = info.checksum_valid;
  }
  return PX4LITE_OK;
#else
  return PX4LITE_NOT_READY;
#endif
}

/**
 * @brief 将Driver固定5星页逐字段转换；不复制整张64星表，不触发接收服务。
 * @details 局部副本80B，适配后由App独立判断逐星消息年龄，不借定位帧续期。
 */
Px4Lite_Result_t Px4Lite_GnssCopySatellitePage(uint8_t page, Px4Lite_GnssSatellitePage_t *out)
{
  if (out == NULL) { return PX4LITE_INVALID_PARAM; }
  memset(out, 0, sizeof(*out));
#if PX4LITE_ENABLE_GNSS
  {
    Gnss_SatellitePage_t source;
    Gnss_Result_t result;
    uint8_t i;
#if GNSS_SAT_PAGE_CAPACITY != PX4LITE_GNSS_SAT_PAGE_CAPACITY
#error "Driver and Framework satellite page capacities must match"
#endif
    result = Sensor_GNSS_CopySatellitePage(page, &source);
    if (result == GNSS_RESULT_INVALID_PARAM) { return PX4LITE_INVALID_PARAM; }
    if (result != GNSS_RESULT_OK) { return result == GNSS_RESULT_NO_DATA ? PX4LITE_NOT_READY : PX4LITE_IO_ERROR; }
    if (!source.available) { return PX4LITE_NOT_READY; }
    if (source.count > PX4LITE_GNSS_SAT_PAGE_CAPACITY || source.stored > GNSS_SAT_CAPACITY ||
        source.stored > source.total || source.page != page) { return PX4LITE_IO_ERROR; }
    out->sequence = source.sequence; out->received_ms = source.received_ms; out->itow_ms = source.itow_ms;
    out->total = source.total; out->stored = source.stored; out->page = source.page; out->count = source.count;
    out->truncated = source.truncated; out->available = source.available; out->used_count = source.used_count;
    for (i = 0U; i < source.count; ++i) {
      const Gnss_Satellite_t *s = &source.entries[i];
      Px4Lite_GnssSatellite_t *d = &out->entries[i];
      d->gnss_id = s->gnss_id; d->sv_id = s->sv_id; d->cno_dbhz = s->cno_dbhz;
      d->quality = s->quality; d->used = s->used;
      d->elevation_deg = s->elevation_deg; d->azimuth_deg = s->azimuth_deg;
      if (s->valid_mask & GNSS_SAT_VALID_ELEVATION) { d->valid_mask |= PX4LITE_GNSS_SAT_VALID_ELEVATION; }
      if (s->valid_mask & GNSS_SAT_VALID_AZIMUTH) { d->valid_mask |= PX4LITE_GNSS_SAT_VALID_AZIMUTH; }
    }
    return PX4LITE_OK;
  }
#else
  (void)page;
  return PX4LITE_NOT_READY;
#endif
}

/**
 * @brief 逐字段转换Driver和BSP诊断，不消费接收队列、不启动发送或修复外设。
 * @details 软件副本只使用sensor服务时间；传输读取失败不抹掉Driver事实。
 *          两侧分别复制而非同一原子时刻，因此不能把字节差额直接宣称为丢失。
 */
Px4Lite_Result_t Px4Lite_GnssCopyDiagnostics(Px4Lite_GnssDiagnostics_t *out)
{
  if (out == NULL) { return PX4LITE_INVALID_PARAM; }
  memset(out, 0, sizeof(*out));
#if PX4LITE_ENABLE_GNSS
  {
    Gnss_Diagnostics_t source;
    BSP_GNSS_TransportInfo_t transport;
    Gnss_Result_t result;
    memset(&source, 0, sizeof(source));
    memset(&transport, 0, sizeof(transport));
    result = Sensor_GNSS_CopyDiagnostics(&source);
    if (BSP_GNSS_CopyTransportInfo(&transport) == BSP_STATUS_OK) {
      out->transport_baud_bps = transport.baud_bps;
      out->transport_rx_bytes = transport.rx_bytes;
      out->rx_drop_bytes = transport.rx_drop_bytes;
      out->uart_errors = transport.uart_errors;
      out->dma_errors = transport.dma_errors;
      out->tx_packets = transport.tx_packets;
      out->tx_failures = transport.tx_failures;
      out->recoveries = transport.recoveries;
      out->ndtr = transport.ndtr;
      out->queued_bytes = transport.queued_bytes;
      out->initialized = (uint8_t)(transport.initialized != 0U);
      out->tx_busy = (uint8_t)(transport.tx_busy != 0U);
      out->transport_available = 1U;
    }
    if (result != GNSS_RESULT_OK) {
      return result == GNSS_RESULT_NO_DATA ? PX4LITE_NOT_READY : PX4LITE_IO_ERROR;
    }
    out->measured_ms = source.measured_ms;
    out->service_count = source.service_count;
    out->rx_bytes = source.rx_bytes;
    out->valid_nmea = source.valid_nmea;
    out->valid_ubx = source.valid_ubx;
    out->checksum_errors = source.checksum_errors;
    out->framing_resets = source.framing_resets;
    out->tx_commands = source.tx_commands;
    out->ack_count = source.ack_count;
    out->nak_count = source.nak_count;
    out->timeout_count = source.timeout_count;
    out->baud_changes = source.baud_changes;
    out->last_rx_ms = source.last_rx_ms;
    out->last_frame_ms = source.last_frame_ms;
    out->baud_bps = source.baud_bps;
    out->protocol_version_x100 = source.protocol_version_x100;
    out->rx_seen = (uint8_t)(source.rx_seen != 0U);
    out->frame_seen = (uint8_t)(source.frame_seen != 0U);
    switch (source.protocol) {
      case GNSS_PROTOCOL_NMEA: out->protocol = PX4LITE_GNSS_PROTOCOL_NMEA; break;
      case GNSS_PROTOCOL_UBX: out->protocol = PX4LITE_GNSS_PROTOCOL_UBX; break;
      default: out->protocol = PX4LITE_GNSS_PROTOCOL_UNKNOWN; break;
    }
    switch (source.state) {
      case GNSS_LINK_SCAN: out->state = PX4LITE_GNSS_LINK_DETECTING; break;
      case GNSS_LINK_VERSION: out->state = PX4LITE_GNSS_LINK_VERSION; break;
      case GNSS_LINK_CONFIGURE: out->state = PX4LITE_GNSS_LINK_CONFIGURING; break;
      case GNSS_LINK_RECEIVING: out->state = PX4LITE_GNSS_LINK_RECEIVING; break;
      case GNSS_LINK_UNCONFIRMED: out->state = PX4LITE_GNSS_LINK_UNCONFIRMED; break;
      case GNSS_LINK_IO_ERROR: out->state = PX4LITE_GNSS_LINK_IO_ERROR; break;
      default: out->state = PX4LITE_GNSS_LINK_UNKNOWN; break;
    }
    out->config_step = source.config_step;
    out->config_total = source.config_total;
    out->config_confirmed = (uint8_t)(source.config_confirmed != 0U);
    out->version_seen = (uint8_t)(source.version_seen != 0U);
    out->available = 1U;
    return PX4LITE_OK;
  }
#else
  return PX4LITE_NOT_READY;
#endif
}

/** @brief 将驱动只读状态转换为公共诊断；计算在临界区外，临界区只复制。 */
static void Px4Lite_ImuPublishDiagnostics(const PlatformImuStatus_t *status)
{
  Px4Lite_ImuDiagnostics_t diag;
  memset(&diag, 0, sizeof(diag));
  diag.driver = (uint8_t)PX4LITE_IMU_DRIVER;
  diag.rx_sequence = status->rx_sequence;
  diag.sample_time_ms = status->sample_time_ms;
  diag.error_count = status->error_count;
  diag.reinit_count = status->reinit_count;
  diag.last_stage = (uint8_t)status->last_stage;
  diag.last_result = (uint8_t)status->last_result;
  diag.last_bsp_status = status->last_bsp_status;
#if PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088
  diag.bus = (uint8_t)status->bus;
  diag.accel_chip_id = status->accel_chip_id;
  diag.gyro_chip_id = status->gyro_chip_id;
  diag.accel_addr_7bit = status->accel_addr_7bit;
  diag.gyro_addr_7bit = status->gyro_addr_7bit;
  diag.fifo_drop_count = status->fifo_drop_count;
  diag.device_ready = status->initialized;
#else
  diag.bus = 1U;
  diag.accel_chip_id = status->last_chip_id;
  diag.gyro_chip_id = status->last_chip_id;
  diag.accel_addr_7bit = status->i2c_addr_7bit;
  diag.gyro_addr_7bit = status->i2c_addr_7bit;
  diag.device_ready = (status->last_result == MPU6050_RESULT_OK) ? 1U : 0U;
#endif
  taskENTER_CRITICAL();
  s_imu_diagnostics = diag;
  s_imu_diagnostics_ready = 1U;
  taskEXIT_CRITICAL();
}

/** @brief 启动选定IMU并清除Adapter门控；BMI初始化接受与设备验证分别记录。 */
Px4Lite_Result_t Px4Lite_ImuInit(void)
{
  PlatformImuResult_t result;
  PlatformImuStatus_t status;
  s_imu_last_rx_sequence = 0U;
  s_imu_last_reinit_count = 0U;
  s_imu_last_sample_ms = 0U;
  s_imu_stable_valid_count = 0U;
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
  Px4Lite_Bmi088UplinkInvalidate();
#endif
#if PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088
  result = Sensor_BMI088_Init((Bmi088_Bus_t)PX4LITE_IMU_BMI088_BUS);
#else
  result = Sensor_MPU6050_Init();
#endif
  memset(&status, 0, sizeof(status));
  (void)PlatformImu_GetStatus(&status);
  Px4Lite_ImuPublishDiagnostics(&status);
  return (result == PLATFORM_IMU_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
}

/** @brief Health只发送请求；总线操作由下一轮sensor服务执行。 */
void Px4Lite_ImuRequestReinit(void)
{
#if PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088
  Sensor_BMI088_RequestReinit();
#else
  Sensor_MPU6050_RequestReinit();
#endif
}

/** @brief 复制缓存诊断，不读时钟、不采样、不推进初始化或恢复。 */
Px4Lite_Result_t Px4Lite_ImuCopyDiagnostics(Px4Lite_ImuDiagnostics_t *out)
{
  uint8_t ready;
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  *out = s_imu_diagnostics;
  ready = s_imu_diagnostics_ready;
  taskEXIT_CRITICAL();
  return (ready != 0U) ? PX4LITE_OK : PX4LITE_NOT_READY;
}

/** @brief 在sensor推进驱动并发布统一单位；重初始化后第三个连续有效样本开始可用。 */
Px4Lite_Result_t Px4Lite_ImuRead(Px4Lite_SensorImu_t *measurement)
{
  PlatformImuSnapshot_t snapshot;
  PlatformImuStatus_t status;
  PlatformImuResult_t result;
  uint32_t now_ms;
  uint32_t delta_ms;
  uint8_t i;

  if (measurement == 0) { return PX4LITE_INVALID_PARAM; }

  now_ms = Px4Lite_PlatformGetMs();
  result = PlatformImu_Service(now_ms);
  memset(&status, 0, sizeof(status));
  (void)PlatformImu_GetStatus(&status);
  Px4Lite_ImuPublishDiagnostics(&status);
  if (status.reinit_count != s_imu_last_reinit_count) {
    s_imu_last_reinit_count = status.reinit_count;
    s_imu_stable_valid_count = 0U;
    s_imu_last_sample_ms = 0U;
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
    Px4Lite_Bmi088UplinkInvalidate();
#endif
  }
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK && (PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088)
  if (!status.initialized) { Px4Lite_Bmi088UplinkInvalidate(); }
#endif
  if (result != PLATFORM_IMU_OK) {
    /* 初始化门控要求连续有效帧，真实I/O错误仅在门控未完成时清零；
       NO_DATA不表示错误，已完成门控后单次瞬态也不重启整个等待窗口。 */
    if ((result != PLATFORM_IMU_NO_DATA) && (s_imu_stable_valid_count < PX4LITE_IMU_STABLE_VALID_MIN)) {
      s_imu_stable_valid_count = 0U;
    }
    return (result == PLATFORM_IMU_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR;
  }

  result = PlatformImu_CopySnapshot(&snapshot);
  if (result != PLATFORM_IMU_OK) { return (result == PLATFORM_IMU_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }

  if ((snapshot.rx_sequence == 0U) || (snapshot.rx_sequence == s_imu_last_rx_sequence)) { return PX4LITE_IDLE; }

  if (s_imu_stable_valid_count < PX4LITE_IMU_STABLE_VALID_MIN) {
    s_imu_stable_valid_count++;
    if (s_imu_stable_valid_count < PX4LITE_IMU_STABLE_VALID_MIN) {
      s_imu_last_rx_sequence = snapshot.rx_sequence;
      return PX4LITE_IDLE;
    }
  }

  memset(measurement, 0, sizeof(*measurement));
  measurement->header.sample_time_ms = snapshot.sample_time_ms;
  for (i = 0U; i < 3U; ++i) {
    measurement->accel_mg[i]  = Px4Lite_RoundFloatToI32(snapshot.accel_g[i] * 1000.0f);
    measurement->gyro_mdps[i] = Px4Lite_RoundFloatToI32(snapshot.gyro_dps[i] * 1000.0f);
  }
#if PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088
  Px4Lite_MapBmi088Axes(measurement->accel_mg, measurement->gyro_mdps);
#else
  Px4Lite_MapImuAxes(measurement->accel_mg, measurement->gyro_mdps);
#endif
  measurement->temperature_cdeg = (int16_t)Px4Lite_RoundFloatToI32(snapshot.temperature_c * 100.0f);
  measurement->sample_period_us = (uint16_t)(PX4LITE_IMU_WORK_PERIOD_MS * 1000U);
  delta_ms = snapshot.sample_time_ms - s_imu_last_sample_ms;
  if ((s_imu_last_sample_ms != 0U) && (delta_ms > 0U) && (delta_ms <= 65U)) {
    measurement->sample_period_us = (uint16_t)(delta_ms * 1000U);
  }

#if PX4LITE_ENABLE_RPI_BMI088_UPLINK && (PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088)
  {
    Px4Lite_Bmi088Sample_t pair;
    /* 只在完整新样本通过既有稳定门控后发布；原字节与解析值来自同一次Driver副本。 */
    memset(&pair,0,sizeof(pair));
    pair.sample_sequence=snapshot.rx_sequence;
    pair.sample_time_ms=snapshot.sample_time_ms;
    pair.reinit_count=snapshot.reinit_count;
    pair.driver_error_count=snapshot.error_count;
    pair.fifo_drop_count=snapshot.fifo_drop_count;
    memcpy(pair.accel_mg,measurement->accel_mg,sizeof(pair.accel_mg));
    memcpy(pair.gyro_mdps,measurement->gyro_mdps,sizeof(pair.gyro_mdps));
    pair.temperature_cdeg=measurement->temperature_cdeg;
    pair.sample_period_us=measurement->sample_period_us;
    pair.bus=snapshot.raw.bus;
    pair.accel_addr_7bit=snapshot.raw.accel_addr_7bit;
    pair.gyro_addr_7bit=snapshot.raw.gyro_addr_7bit;
    pair.accel_chip_id=snapshot.raw.accel_chip_id;
    pair.gyro_chip_id=snapshot.raw.gyro_chip_id;
    pair.accel_status=snapshot.raw.accel_status;
    pair.gyro_fifo_status=snapshot.raw.gyro_fifo_status;
    pair.gyro_frame_count=snapshot.raw.gyro_frame_count;
    memcpy(pair.raw_accel,snapshot.raw.accel,sizeof(pair.raw_accel));
    memcpy(pair.raw_temperature,snapshot.raw.temperature,sizeof(pair.raw_temperature));
    memcpy(pair.raw_gyro,snapshot.raw.gyro,sizeof(pair.raw_gyro));
    pair.axis_map[0]=PX4LITE_BMI088_AXIS_X;
    pair.axis_map[1]=PX4LITE_BMI088_AXIS_Y;
    pair.axis_map[2]=PX4LITE_BMI088_AXIS_Z;
    Px4Lite_Bmi088UplinkPublish(&pair);
  }
#endif

  s_imu_last_sample_ms = snapshot.sample_time_ms;
  s_imu_last_rx_sequence = snapshot.rx_sequence;
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

/** @brief 初始化已启用的电源驱动；第二组为当前整机主回路，禁用组不建立样本。 */
Px4Lite_Result_t Px4Lite_BatteryInit(void)
{
#if PX4LITE_ENABLE_BATTERY1
  if (Sensor_Power_Init() != POWER_RESULT_OK) { return PX4LITE_IO_ERROR; }
#endif
#if PX4LITE_ENABLE_BATTERY2
  if (Sensor_Power2_Init() != POWER_RESULT_OK) { return PX4LITE_IO_ERROR; }
#endif
  return PX4LITE_OK;
}

/** @brief Health 只请求已启用驱动恢复；实际重新初始化由 sensor 服务执行。 */
void Px4Lite_BatteryRequestReinit(void)
{
#if PX4LITE_ENABLE_BATTERY1
  Sensor_Power_RequestReinit();
#endif
#if PX4LITE_ENABLE_BATTERY2
  Sensor_Power2_RequestReinit();
#endif
}

/** @brief 读取第一组真实测量；停用时返回 NOT_READY，绝不借用第二组填充。 */
Px4Lite_Result_t Px4Lite_BatteryRead(Px4Lite_BatteryStatus_t *measurement)
{
#if PX4LITE_ENABLE_BATTERY1
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
#else
  (void)measurement;
  return PX4LITE_NOT_READY;
#endif
}

#if PX4LITE_ENABLE_BATTERY2
/** @brief 将BSP参考事实显式转换为Framework枚举，不向消费者暴露BSP类型。 */
static uint8_t PlatformPowerReferenceStatus(uint8_t status)
{
  switch (status) {
    case BSP_ADC_REF_MEASURED: return PX4LITE_POWER_REF_MEASURED;
    case BSP_ADC_REF_FALLBACK_CAL_ZERO:
    case BSP_ADC_REF_FALLBACK_CAL_ERASED:
    case BSP_ADC_REF_FALLBACK_CAL_RANGE: return PX4LITE_POWER_REF_CAL_INVALID;
    case BSP_ADC_REF_FALLBACK_READ: return PX4LITE_POWER_REF_READ_FAILED;
    case BSP_ADC_REF_FALLBACK_RAW: return PX4LITE_POWER_REF_RAW_INVALID;
    case BSP_ADC_REF_FALLBACK_VDDA_RANGE: return PX4LITE_POWER_REF_VDDA_INVALID;
    default: return PX4LITE_POWER_REF_UNAVAILABLE;
  }
}

/**
 * @brief sensor在Service之后发布同轮只读诊断，失败尝试明确标为无效。
 * @param[in] now_ms 本轮采样时刻，单位ms。
 * @param[in] snapshot 成功的Driver副本；采集失败时为NULL。
 * @details 仅复制ADC缓存，不启动转换；ADC2内嵌参考不受后续电流参考采样覆盖。
 */
static void PlatformPower2PublishDiagnostics(uint32_t now_ms, const Power_Snapshot_t *snapshot)
{
  BSP_ADC2_Diag_t source;
  Px4Lite_PowerDiagnostics_t converted;
  uint8_t floating;

  if (BSP_ADC2_GetDiag(&source) != BSP_STATUS_OK) { return; }
  if (s_power2_diagnostics_ready && (source.sequence == s_power2_diagnostics.sequence)) { return; }
  memset(&converted, 0, sizeof(converted));
  converted.sequence = source.sequence;
  converted.sample_time_ms = now_ms;
  converted.voltage_raw = source.raw_average;
  converted.voltage_pin_mv = source.pin_mv;
  converted.voltage_bsp_mv = source.input_mv;
  converted.vref_raw = source.reference.vref_raw;
  converted.vref_cal_raw = source.reference.calibration_raw;
  converted.vdda_mv = source.reference.vdda_mv;
  converted.vdda_calculated_mv = source.reference.calculated_vdda_mv;
  converted.reference_status = source.reference.ready ? PlatformPowerReferenceStatus(source.reference.status) : PX4LITE_POWER_REF_UNAVAILABLE;
  if ((snapshot != 0) && source.sample_valid) {
    converted.voltage_valid = 1U;
    converted.voltage_mv = (uint32_t)(snapshot->voltage_v * 1000.0f + 0.5f);
    converted.current_ma = snapshot->current_ma;
    converted.current_valid = snapshot->current_valid;
    if (snapshot->current_valid) {
      BSP_ADC_Current_GetDiag(BSP_ADC_CURRENT_BATTERY2, &converted.current_pin_mv, &floating);
    }
  }
  taskENTER_CRITICAL();
  s_power2_diagnostics = converted;
  s_power2_diagnostics_ready = 1U;
  taskEXIT_CRITICAL();
}
#endif

/** @brief 只复制sensor已发布的主回路诊断；关闭第二组时不读取任何底层缓存。 */
Px4Lite_Result_t Px4Lite_Battery2CopyDiagnostics(Px4Lite_PowerDiagnostics_t *out)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
#if PX4LITE_ENABLE_BATTERY2
  taskENTER_CRITICAL();
  if (s_power2_diagnostics_ready) {
    *out = s_power2_diagnostics;
    taskEXIT_CRITICAL();
    return PX4LITE_OK;
  }
  taskEXIT_CRITICAL();
#endif
  return PX4LITE_NOT_READY;
}

/** @brief 读取第二组主回路测量，保持原topic身份；参考回退作为降级事实交给Framework。 */
Px4Lite_Result_t Px4Lite_Battery2Read(Px4Lite_BatteryStatus_t *measurement)
{
#if PX4LITE_ENABLE_BATTERY2
  static uint32_t last_rx_sequence;
  Power_Snapshot_t snapshot;
  Power_Result_t result;
  uint32_t now_ms;

  if (measurement == 0) { return PX4LITE_INVALID_PARAM; }

  now_ms = Px4Lite_PlatformGetMs();
  result = Sensor_Power2_Service(now_ms);
  if (result != POWER_RESULT_OK) {
    if (result != POWER_RESULT_NO_DATA) { PlatformPower2PublishDiagnostics(now_ms, 0); }
    return (result == POWER_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR;
  }

  result = Sensor_Power2_CopySnapshot(&snapshot);
  if (result != POWER_RESULT_OK) { return (result == POWER_RESULT_NO_DATA) ? PX4LITE_IDLE : PX4LITE_IO_ERROR; }
  PlatformPower2PublishDiagnostics(now_ms, &snapshot);

  if ((snapshot.rx_sequence == 0U) || (snapshot.rx_sequence == last_rx_sequence)) { return PX4LITE_IDLE; }

  memset(measurement, 0, sizeof(*measurement));
  measurement->header.sample_time_ms = snapshot.sample_time_ms;
  measurement->voltage_mv            = (uint32_t)((snapshot.voltage_v * 1000.0f) + 0.5f);
  measurement->current_ma            = snapshot.current_ma;
  measurement->current_valid         = snapshot.current_valid;
  measurement->percent               = snapshot.percent;
  measurement->low_voltage           = snapshot.low_voltage;

  if (s_power2_diagnostics_ready &&
      (s_power2_diagnostics.reference_status != PX4LITE_POWER_REF_MEASURED)) {
    measurement->header.flags |= PX4LITE_DATA_DEGRADED;
  }

  last_rx_sequence = snapshot.rx_sequence;
  return PX4LITE_OK;
#else
  (void)measurement;
  return PX4LITE_NOT_READY;
#endif
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
  memset(&s_rpi_monitor, 0, sizeof(s_rpi_monitor));
  return (BSP_RpiUART_Init() == BSP_STATUS_OK) ? PX4LITE_OK : PX4LITE_IO_ERROR;
#else
  return PX4LITE_OK;
#endif
}

/**
 * @brief Comm 沿既有阻塞路径发送 UART6 原始帧，并独立累计本机完成/失败事实。
 * @details 返回码、捕获提交和超时保持既有语义；不推断对端是否收到，不改变实验事务。
 */
Px4Lite_Result_t Px4Lite_RpiMavlinkSend(const uint8_t *data, uint16_t len)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  BSP_Status_t status;
  uint32_t completed_ms;

  if ((data == 0) || (len == 0U)) { return PX4LITE_INVALID_PARAM; }

  status = BSP_RpiUART_Send(data, len, PX4LITE_RPI_MAVLINK_UART_TIMEOUT_MS);
  completed_ms = Px4Lite_PlatformGetMs();
  if (status == BSP_STATUS_OK) {
    s_rpi_monitor.tx_bytes += len;
    s_rpi_monitor.tx_frames++;
    s_rpi_monitor.last_tx_ms = completed_ms;
    s_rpi_monitor.tx_seen = 1U;
  } else {
    s_rpi_monitor.tx_failures++;
  }
  Px4Lite_CaptureRpiTx(data, len, completed_ms, status == BSP_STATUS_OK ? PX4LITE_CAPTURE_UART_DONE : (status == BSP_STATUS_TIMEOUT ? PX4LITE_CAPTURE_TIMEOUT : PX4LITE_CAPTURE_ERROR));
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
 * @brief Comm 复制通用 UART6 监视事实，并把 HAL 配置转换为与平台无关的值。
 * @details BSP 只复制原始事实；本函数不改变解析、发送、捕获或实验事务。
 */
Px4Lite_Result_t Px4Lite_RpiMavlinkCopyMonitor(Px4Lite_SerialMonitor_t *out)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  memset(out, 0, sizeof(*out));
#if PX4LITE_ENABLE_RPI_MAVLINK
  {
    BSP_RpiUART_Diagnostics_t raw;
    BSP_RpiUART_CopyDiagnostics(&raw);
    out->baud_bps = raw.baud_rate;
    out->data_bits = raw.word_length == UART_WORDLENGTH_8B ? 8U :
                     (raw.word_length == UART_WORDLENGTH_9B ? 9U : 0U);
    out->parity = raw.parity == UART_PARITY_NONE ? PX4LITE_SERIAL_PARITY_NONE :
                  (raw.parity == UART_PARITY_EVEN ? PX4LITE_SERIAL_PARITY_EVEN :
                  (raw.parity == UART_PARITY_ODD ? PX4LITE_SERIAL_PARITY_ODD : PX4LITE_SERIAL_PARITY_UNKNOWN));
    if (out->data_bits != 0U && out->parity != PX4LITE_SERIAL_PARITY_NONE &&
        out->parity != PX4LITE_SERIAL_PARITY_UNKNOWN) { out->data_bits--; }
    out->stop_bits_x2 = raw.stop_bits == UART_STOPBITS_1 ? 2U : (raw.stop_bits == UART_STOPBITS_2 ? 4U : 0U);
    out->flow_control = raw.hw_flow_ctl == UART_HWCONTROL_NONE ? PX4LITE_SERIAL_FLOW_NONE :
                        (raw.hw_flow_ctl == UART_HWCONTROL_RTS ? PX4LITE_SERIAL_FLOW_RTS :
                        (raw.hw_flow_ctl == UART_HWCONTROL_CTS ? PX4LITE_SERIAL_FLOW_CTS :
                        (raw.hw_flow_ctl == UART_HWCONTROL_RTS_CTS ? PX4LITE_SERIAL_FLOW_RTS_CTS : PX4LITE_SERIAL_FLOW_UNKNOWN)));
    out->rx_byte_count = raw.rx_byte_count;
    out->rx_uart_error_count = raw.rx_uart_error_count;
    out->rx_drop_count = raw.rx_drop_count;
    out->rx_rearm_failure_count = raw.rx_rearm_failure_count;
    out->last_rx_ms = raw.last_rx_ms;
    out->rx_seen = raw.rx_seen;
    out->config_valid = raw.config_valid;
    out->rx_valid_frame_count = s_rpi_monitor.rx_valid_frames;
    out->rx_frame_error_count = s_rpi_monitor.rx_frame_errors;
    out->tx_complete_byte_count = s_rpi_monitor.tx_bytes;
    out->tx_complete_frame_count = s_rpi_monitor.tx_frames;
    out->tx_failure_count = s_rpi_monitor.tx_failures;
    out->last_tx_ms = s_rpi_monitor.last_tx_ms;
    out->tx_seen = s_rpi_monitor.tx_seen;
    out->available = 1U;
    out->measured_ms = Px4Lite_PlatformGetMs();
    return PX4LITE_OK;
  }
#else
  return PX4LITE_NOT_READY;
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
      s_rpi_monitor.rx_frame_errors++;
      Px4Lite_CaptureReject(PX4LITE_CAPTURE_RPI_RX);
      s_rpi_diagnostic_length = 0U; parser->msg_received = MAVLINK_FRAMING_INCOMPLETE; parser->parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (byte == MAVLINK_STX) { parser->parse_state = MAVLINK_PARSE_STATE_GOT_STX; message->len = 0U; mavlink_start_checksum(message); s_rpi_diagnostic_raw[0] = byte; s_rpi_diagnostic_length = 1U; }
    }
    if (framing == MAVLINK_FRAMING_OK) {
      s_rpi_monitor.rx_valid_frames++;
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
