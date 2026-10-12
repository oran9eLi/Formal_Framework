/**
 * @file sensor_bme280.c
 * @brief BME280 强制测量、补偿及设备事实缓存。
 * @details 属于 Sensor Driver 层，通过 BSP 访问总线和时间；sensor 任务唯一推进
 *          测量与恢复。不得访问 HAL、发布 Framework topic 或判定 OFFLINE。
 */

#include "sensor_bme280.h"

#include "bsp_critical.h"
#include "bsp_i2c.h"
#include "bsp_time.h"

#include <stddef.h>
#include <string.h>

#define BME280_I2C_ADDR_PRIMARY     (0x76U << 1)
#define BME280_I2C_ADDR_SECONDARY   (0x77U << 1)

#define BME280_REG_CALIB_TP         0x88U
#define BME280_REG_CALIB_H1         0xA1U
#define BME280_REG_ID               0xD0U
#define BME280_REG_RESET            0xE0U
#define BME280_REG_CALIB_H2         0xE1U
#define BME280_REG_CTRL_HUM         0xF2U
#define BME280_REG_STATUS           0xF3U
#define BME280_REG_CTRL_MEAS        0xF4U
#define BME280_REG_CONFIG           0xF5U
#define BME280_REG_DATA             0xF7U

#define BME280_CHIP_ID              0x60U
#define BME280_RESET_CMD            0xB6U
#define BME280_STATUS_IM_UPDATE     0x01U
#define BME280_STATUS_MEASURING     0x08U

#define BME280_OSRS_X1              0x01U
#define BME280_MODE_FORCED          0x01U
#define BME280_CTRL_MEAS_SLEEP_X1   ((BME280_OSRS_X1 << 5) | (BME280_OSRS_X1 << 2))
#define BME280_CTRL_MEAS_FORCED_X1  ((BME280_OSRS_X1 << 5) | (BME280_OSRS_X1 << 2) | BME280_MODE_FORCED)
#define BME280_CTRL_HUM_X1          BME280_OSRS_X1
#define BME280_CONFIG_STANDBY_0_5MS 0x00U

#define BME280_I2C_TIMEOUT_MS       20U
#define BME280_PROBE_TIMEOUT_MS     4U
#define BME280_INIT_RETRY_MS        1000U
#define BME280_REINIT_FAIL_LIMIT    5U
#define BME280_RESET_DELAY_MS       2U
#define BME280_MAX_WAIT_POLLS       20U
#define BME280_MEASURE_DELAY_MS     10U
#define BME280_MEASURE_TIMEOUT_MS   30U

typedef enum {
  BME280_MEASURE_IDLE = 0,
  BME280_MEASURE_WAITING
} Bme280_MeasureState_t;

typedef struct {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;
} Bme280_Calib_t;

static Bme280_Calib_t s_calib;
static Bme280_Snapshot_t s_snapshot;
static uint16_t s_addr = BME280_I2C_ADDR_PRIMARY;
static int32_t s_t_fine;
static uint8_t s_initialized;
static uint32_t s_next_init_ms;
static uint8_t s_read_fail_count;
static volatile uint8_t s_reinit_request;
static Bme280_MeasureState_t s_measure_state;
static uint32_t s_measure_start_ms;

static uint16_t Bme280_ReadU16LE(const uint8_t *data, uint16_t offset)
{
  return (uint16_t)((uint16_t)data[offset] | ((uint16_t)data[offset + 1U] << 8));
}

static int16_t Bme280_ReadS16LE(const uint8_t *data, uint16_t offset)
{
  return (int16_t)Bme280_ReadU16LE(data, offset);
}

static int16_t Bme280_SignExtend12(uint16_t value)
{
  if ((value & 0x0800U) != 0U) { value |= 0xF000U; }
  return (int16_t)value;
}

static BSP_Status_t Bme280_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
  return BSP_I2C_MemRead(s_addr, reg, data, len, BME280_I2C_TIMEOUT_MS);
}

static BSP_Status_t Bme280_WriteReg(uint8_t reg, uint8_t value)
{
  return BSP_I2C_MemWrite(s_addr, reg, &value, 1U, BME280_I2C_TIMEOUT_MS);
}

static Bme280_Result_t Bme280_ProbeAddress(void)
{
  if (BSP_I2C_IsDeviceReady(BME280_I2C_ADDR_PRIMARY, BME280_PROBE_TIMEOUT_MS) == BSP_STATUS_OK) {
    s_addr = BME280_I2C_ADDR_PRIMARY;
    return BME280_RESULT_OK;
  }

  if (BSP_I2C_IsDeviceReady(BME280_I2C_ADDR_SECONDARY, BME280_PROBE_TIMEOUT_MS) == BSP_STATUS_OK) {
    s_addr = BME280_I2C_ADDR_SECONDARY;
    return BME280_RESULT_OK;
  }

  return BME280_RESULT_IO_ERROR;
}

static Bme280_Result_t Bme280_CheckChipId(void)
{
  uint8_t chip_id = 0U;

  if (Bme280_ReadReg(BME280_REG_ID, &chip_id, 1U) != BSP_STATUS_OK) { return BME280_RESULT_IO_ERROR; }

  return (chip_id == BME280_CHIP_ID) ? BME280_RESULT_OK : BME280_RESULT_BAD_ID;
}

static Bme280_Result_t Bme280_WaitStatusClear(uint8_t mask)
{
  uint8_t i;

  for (i = 0U; i < BME280_MAX_WAIT_POLLS; ++i) {
    uint8_t status = 0U;

    if (Bme280_ReadReg(BME280_REG_STATUS, &status, 1U) != BSP_STATUS_OK) { return BME280_RESULT_IO_ERROR; }
    if ((status & mask) == 0U) { return BME280_RESULT_OK; }
    BSP_Time_DelayMs(1U);
  }

  return BME280_RESULT_TIMEOUT;
}

static Bme280_Result_t Bme280_ReadCalibration(void)
{
  uint8_t tp[26];
  uint8_t h1;
  uint8_t h[7];

  if ((Bme280_ReadReg(BME280_REG_CALIB_TP, tp, sizeof(tp)) != BSP_STATUS_OK) || (Bme280_ReadReg(BME280_REG_CALIB_H1, &h1, 1U) != BSP_STATUS_OK) || (Bme280_ReadReg(BME280_REG_CALIB_H2, h, sizeof(h)) != BSP_STATUS_OK)) { return BME280_RESULT_IO_ERROR; }

  s_calib.dig_T1 = Bme280_ReadU16LE(tp, 0U);
  s_calib.dig_T2 = Bme280_ReadS16LE(tp, 2U);
  s_calib.dig_T3 = Bme280_ReadS16LE(tp, 4U);
  s_calib.dig_P1 = Bme280_ReadU16LE(tp, 6U);
  s_calib.dig_P2 = Bme280_ReadS16LE(tp, 8U);
  s_calib.dig_P3 = Bme280_ReadS16LE(tp, 10U);
  s_calib.dig_P4 = Bme280_ReadS16LE(tp, 12U);
  s_calib.dig_P5 = Bme280_ReadS16LE(tp, 14U);
  s_calib.dig_P6 = Bme280_ReadS16LE(tp, 16U);
  s_calib.dig_P7 = Bme280_ReadS16LE(tp, 18U);
  s_calib.dig_P8 = Bme280_ReadS16LE(tp, 20U);
  s_calib.dig_P9 = Bme280_ReadS16LE(tp, 22U);
  s_calib.dig_H1 = h1;
  s_calib.dig_H2 = Bme280_ReadS16LE(h, 0U);
  s_calib.dig_H3 = h[2];
  s_calib.dig_H4 = Bme280_SignExtend12((uint16_t)(((uint16_t)h[3] << 4) | (h[4] & 0x0FU)));
  s_calib.dig_H5 = Bme280_SignExtend12((uint16_t)(((uint16_t)h[5] << 4) | (h[4] >> 4)));
  s_calib.dig_H6 = (int8_t)h[6];

  return (s_calib.dig_P1 != 0U) ? BME280_RESULT_OK : BME280_RESULT_IO_ERROR;
}

static float Bme280_CompensateTemperature(int32_t adc_T)
{
  int32_t var1;
  int32_t var2;
  int32_t temperature_cdeg;

  var1             = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) * ((int32_t)s_calib.dig_T2)) >> 11;
  var2             = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) * ((int32_t)s_calib.dig_T3)) >> 14;
  s_t_fine         = var1 + var2;
  temperature_cdeg = (s_t_fine * 5 + 128) >> 8;
  return ((float)temperature_cdeg) / 100.0f;
}

static float Bme280_CompensatePressurePa(int32_t adc_P)
{
  int64_t var1;
  int64_t var2;
  int64_t p;

  var1 = ((int64_t)s_t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) + ((var1 * (int64_t)s_calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)s_calib.dig_P1)) >> 33;

  if (var1 == 0) { return 0.0f; }

  p    = 1048576 - adc_P;
  p    = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
  p    = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);

  if (p < 0) { return 0.0f; }
  return (float)((p + 128) >> 8);
}

static float Bme280_CompensateHumidity(int32_t adc_H)
{
  int32_t v_x1_u32r;
  uint32_t humidity_q10;
  float humidity_pct;

  v_x1_u32r = s_t_fine - 76800;
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)s_calib.dig_H4) << 20) - (((int32_t)s_calib.dig_H5) * v_x1_u32r)) + 16384) >> 15) * (((((((v_x1_u32r * ((int32_t)s_calib.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)s_calib.dig_H3)) >> 11) + 32768)) >> 10) + 2097152) * ((int32_t)s_calib.dig_H2) + 8192) >> 14));
  v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)s_calib.dig_H1)) >> 4);

  if (v_x1_u32r < 0) { v_x1_u32r = 0; }
  if (v_x1_u32r > 419430400) { v_x1_u32r = 419430400; }

  humidity_q10 = (uint32_t)(v_x1_u32r >> 12);
  humidity_pct = ((float)humidity_q10) / 1024.0f;
  return (humidity_pct > 100.0f) ? 100.0f : humidity_pct;
}

/** @brief 初始化过采样参数并保持休眠，测量由周期服务显式触发。 */
static Bme280_Result_t Bme280_ConfigureSleepMode(void)
{
  if ((Bme280_WriteReg(BME280_REG_CTRL_HUM, BME280_CTRL_HUM_X1) != BSP_STATUS_OK) || (Bme280_WriteReg(BME280_REG_CONFIG, BME280_CONFIG_STANDBY_0_5MS) != BSP_STATUS_OK) || (Bme280_WriteReg(BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS_SLEEP_X1) != BSP_STATUS_OK)) { return BME280_RESULT_IO_ERROR; }

  return BME280_RESULT_OK;
}

/** @brief 检查芯片、加载补偿参数并配置休眠；不隐式触发采样。 */
Bme280_Result_t Sensor_BME280_Init(void)
{
  Bme280_Result_t result;
  uint32_t primask;

  primask = BSP_Critical_Enter();
  memset(&s_snapshot, 0, sizeof(s_snapshot));
  BSP_Critical_Exit(primask);
  memset(&s_calib, 0, sizeof(s_calib));
  s_initialized       = 0U;
  s_addr              = BME280_I2C_ADDR_PRIMARY;
  s_measure_state     = BME280_MEASURE_IDLE;
  s_measure_start_ms  = 0U;

  result = Bme280_ProbeAddress();
  if (result != BME280_RESULT_OK) { return result; }

  result = Bme280_CheckChipId();
  if (result != BME280_RESULT_OK) { return result; }

  if (Bme280_WriteReg(BME280_REG_RESET, BME280_RESET_CMD) != BSP_STATUS_OK) { return BME280_RESULT_IO_ERROR; }

  BSP_Time_DelayMs(BME280_RESET_DELAY_MS);

  result = Bme280_WaitStatusClear(BME280_STATUS_IM_UPDATE);
  if (result != BME280_RESULT_OK) { return result; }

  result = Bme280_ReadCalibration();
  if (result != BME280_RESULT_OK) { return result; }

  result = Bme280_ConfigureSleepMode();
  if (result == BME280_RESULT_OK) {
    s_initialized       = 1U;
    s_next_init_ms      = 0U;
    s_read_fail_count   = 0U;
    s_reinit_request    = 0U;
    s_measure_state     = BME280_MEASURE_IDLE;
    s_measure_start_ms  = 0U;
  }
  return result;
}

/*
 * Record one read failure and, after several in a row, force a re-init so a
 * device that reset (and lost its forced-mode configuration) gets reconfigured
 * instead of being polled forever while asleep.
 */
static void Bme280_NoteReadFailure(void)
{
  uint32_t primask;

  primask = BSP_Critical_Enter();
  s_snapshot.error_count++;
  BSP_Critical_Exit(primask);
  s_measure_state = BME280_MEASURE_IDLE;
  if (++s_read_fail_count >= BME280_REINIT_FAIL_LIMIT) {
    s_initialized      = 0U;
    s_reinit_request   = 1U;
    s_next_init_ms     = 0U;
    s_read_fail_count  = 0U;
    s_measure_start_ms = 0U;
  }
}

void Sensor_BME280_RequestReinit(void)
{
  s_reinit_request = 1U;
  s_next_init_ms   = 0U;
}

/**
 * @brief 由 sensor 任务推进初始化和测量，仅在空闲且获得许可时启动新转换。
 * @param[in] now_ms 本次服务的真实毫秒时间，用于等待与退避判断。
 * @param[in] start_measurement 非 0 允许启动一次测量；忙时忽略，不排队补采。
 * @return OK 表示新快照仅产生一次，NO_DATA 表示等待或本周期无采样许可。
 */
Bme280_Result_t Sensor_BME280_Service(uint32_t now_ms, uint8_t start_measurement)
{
  uint8_t data[8];
  int32_t adc_P;
  int32_t adc_T;
  int32_t adc_H;
  Bme280_Result_t result;
  uint8_t status;
  uint32_t elapsed_ms;

  if (s_reinit_request != 0U) {
    s_reinit_request = 0U;
    s_initialized    = 0U; /* force the re-init path below */
    s_next_init_ms   = 0U; /* clear back-off so it re-inits now */
    s_measure_state  = BME280_MEASURE_IDLE;
  }

  if (s_initialized == 0U) {
    /*
     * Back off between failed init attempts. Without this a missing or
     * stuck device is probed over blocking I2C every service period,
     * which starves lower-priority tasks (for example the display).
     */
    if ((s_next_init_ms != 0U) && ((int32_t)(now_ms - s_next_init_ms) < 0)) { return BME280_RESULT_NO_DATA; }

    (void)BSP_I2C_Recover();
    result = Sensor_BME280_Init();
    if (result != BME280_RESULT_OK) {
      uint32_t primask;

      s_next_init_ms = BSP_Time_GetTickMs() + BME280_INIT_RETRY_MS;
      primask = BSP_Critical_Enter();
      s_snapshot.error_count++;
      BSP_Critical_Exit(primask);
      return result;
    }
  }

  if (s_measure_state == BME280_MEASURE_IDLE) {
    if (start_measurement == 0U) { return BME280_RESULT_NO_DATA; }
    if (Bme280_WriteReg(BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS_FORCED_X1) != BSP_STATUS_OK) {
      Bme280_NoteReadFailure();
      return BME280_RESULT_IO_ERROR;
    }

    /* 用成功触发写事务后的真实时间保守标记测量，不能用迟到的读取时间刷新年龄。 */
    s_measure_start_ms = BSP_Time_GetTickMs();
    s_measure_state    = BME280_MEASURE_WAITING;
    return BME280_RESULT_NO_DATA;
  }

  elapsed_ms = (uint32_t)(now_ms - s_measure_start_ms);
  if (elapsed_ms < BME280_MEASURE_DELAY_MS) { return BME280_RESULT_NO_DATA; }

  status = 0U;
  if (Bme280_ReadReg(BME280_REG_STATUS, &status, 1U) != BSP_STATUS_OK) {
    Bme280_NoteReadFailure();
    return BME280_RESULT_IO_ERROR;
  }

  if ((status & BME280_STATUS_MEASURING) != 0U) {
    if (elapsed_ms < BME280_MEASURE_TIMEOUT_MS) { return BME280_RESULT_NO_DATA; }
    Bme280_NoteReadFailure();
    return BME280_RESULT_TIMEOUT;
  }

  if (Bme280_ReadReg(BME280_REG_DATA, data, sizeof(data)) != BSP_STATUS_OK) {
    Bme280_NoteReadFailure();
    return BME280_RESULT_IO_ERROR;
  }

  adc_P = (int32_t)((((uint32_t)data[0]) << 12) | (((uint32_t)data[1]) << 4) | (((uint32_t)data[2]) >> 4));
  adc_T = (int32_t)((((uint32_t)data[3]) << 12) | (((uint32_t)data[4]) << 4) | (((uint32_t)data[5]) >> 4));
  adc_H = (int32_t)((((uint32_t)data[6]) << 8) | data[7]);

  {
    Bme280_Snapshot_t candidate;
    uint32_t primask;

    primask = BSP_Critical_Enter();
    candidate = s_snapshot;
    BSP_Critical_Exit(primask);

    candidate.rx_sequence++;
    if (candidate.rx_sequence == 0U) { candidate.rx_sequence = 1U; }
    candidate.sample_time_ms        = s_measure_start_ms;
    candidate.temperature_c         = Bme280_CompensateTemperature(adc_T);
    candidate.pressure_pa           = Bme280_CompensatePressurePa(adc_P);
    candidate.relative_humidity_pct = Bme280_CompensateHumidity(adc_H);

    primask = BSP_Critical_Enter();
    s_snapshot = candidate;
    BSP_Critical_Exit(primask);
  }

  s_read_fail_count = 0U;
  s_measure_state   = BME280_MEASURE_IDLE;
  return BME280_RESULT_OK;
}

Bme280_Result_t Sensor_BME280_CopySnapshot(Bme280_Snapshot_t *out)
{
  uint32_t primask;
  uint32_t rx_sequence;

  if (out == 0) { return BME280_RESULT_INVALID_PARAM; }

  primask = BSP_Critical_Enter();
  *out = s_snapshot;
  rx_sequence = out->rx_sequence;
  BSP_Critical_Exit(primask);

  return (rx_sequence != 0U) ? BME280_RESULT_OK : BME280_RESULT_NO_DATA;
}

Bme280_Result_t Sensor_BME280_GetStatus(Bme280_Status_t *out)
{
  uint32_t primask;
  uint32_t rx_sequence;

  if (out == 0) { return BME280_RESULT_INVALID_PARAM; }

  primask = BSP_Critical_Enter();
  out->rx_sequence    = s_snapshot.rx_sequence;
  out->sample_time_ms = s_snapshot.sample_time_ms;
  out->error_count    = s_snapshot.error_count;
  rx_sequence = out->rx_sequence;
  BSP_Critical_Exit(primask);

  return (rx_sequence != 0U) ? BME280_RESULT_OK : BME280_RESULT_NO_DATA;
}
