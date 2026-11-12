/**
 * @file sensor_bmi088.c
 * @brief BMI088双器件协议、分阶段启动、完整新样本与有界恢复。
 * @details 属于Sensor Driver层，依赖BSP I2C、原始SPI、时间和短临界区。
 * 依据Bosch BMI088数据手册Rev1.9；寄存器协议不下沉至BSP，不访问HAL。
 * sensor任务单写，初始化每轮仅推进一阶段；不阻塞等待复位或模式切换。
 * 加速度DRDY与陀螺FIFO保证新样本，不依赖仅维持约300us的陀螺DRDY脉冲。
 * 成功轮次的真实寄存器返回字节与解析量同次发布，不增加读取或从浮点反算原值。
 */
#include "sensor_bmi088.h"
#include "bsp_critical.h"
#include "bsp_i2c.h"
#include "bsp_imu_spi.h"
#include "bsp_time.h"
#include <string.h>

#define BMI088_ACCEL_DEVICE         0U
#define BMI088_GYRO_DEVICE          1U
#define BMI088_ACCEL_CHIP_ID        0x1EU
#define BMI088_GYRO_CHIP_ID         0x0FU
#define BMI088_IO_TIMEOUT_MS        4U
#define BMI088_IO_FAILURE_LIMIT     5U
#define BMI088_FAST_RETRY_LIMIT     5U
#define BMI088_FAST_RETRY_MS        100U
#define BMI088_SLOW_RETRY_MS        500U
#define BMI088_SPI_BUFFER_SIZE      26U
#define BMI088_GYRO_FRAME_BYTES     6U
#define BMI088_GYRO_DRAIN_MAX        4U
#define BMI088_GYRO_STREAM_VALUE    0x88U
#define BMI088_ACCEL_DATA_READY     0x80U

/** @brief sensor所有者内部初始化阶段，不作为公开健康状态。 */
typedef enum {
  BMI088_PHASE_IDLE = 0,     /**< 等待显式初始化。 */
  BMI088_PHASE_SWITCH,       /**< SPI加速度接口选择。 */
  BMI088_PHASE_ACCEL_ID,     /**< 加速度探测。 */
  BMI088_PHASE_GYRO_ID,      /**< 陀螺探测。 */
  BMI088_PHASE_ACCEL_RESET,  /**< 加速度软复位。 */
  BMI088_PHASE_GYRO_RESET,   /**< 陀螺软复位。 */
  BMI088_PHASE_CONFIG,       /**< 分次写入配置。 */
  BMI088_PHASE_VERIFY,       /**< 分次读回配置。 */
  BMI088_PHASE_ERROR_CHECK,  /**< 启动错误位检查。 */
  BMI088_PHASE_FIFO_CLEAR,   /**< 丢弃初始化期间累积样本。 */
  BMI088_PHASE_RUNNING,      /**< 周期读取。 */
  BMI088_PHASE_RETRY         /**< 等待限速重试。 */
} Bmi088_Phase_t;

/** @brief 固定量程与100Hz轮询配置；delay_ms表示写后最短等待。 */
typedef struct {
  uint8_t device;   /**< 0加速度，1陀螺。 */
  uint8_t reg;      /**< 寄存器地址。 */
  uint8_t value;    /**< 写入值。 */
  uint8_t mask;     /**< 读回校验位，忽略只读或保留位。 */
  uint16_t delay_ms;/**< 后续总线操作前等待，单位ms。 */
} Bmi088_RegisterConfig_t;

static const Bmi088_RegisterConfig_t s_config[] = {
  {BMI088_ACCEL_DEVICE, 0x7CU, 0x00U, 0x03U, 1U},
  {BMI088_ACCEL_DEVICE, 0x7DU, 0x04U, 0x04U, 50U},
  {BMI088_GYRO_DEVICE,  0x11U, 0x00U, 0xA0U, 30U},
  {BMI088_ACCEL_DEVICE, 0x40U, 0xA8U, 0xFFU, 1U},
  {BMI088_ACCEL_DEVICE, 0x41U, 0x02U, 0x03U, 1U},
  {BMI088_GYRO_DEVICE,  0x0FU, 0x01U, 0x07U, 1U},
  {BMI088_GYRO_DEVICE,  0x10U, 0x07U, 0x07U, 1U},
  {BMI088_GYRO_DEVICE,  0x3DU, 0x00U, 0xFFU, 1U},
  /* 保持FIFO_CONFIG_1保留位3的复位值1；STREAM、XYZ、无tag。 */
  {BMI088_GYRO_DEVICE,  0x3EU, BMI088_GYRO_STREAM_VALUE, 0xC3U, 1U}
};

static Bmi088_Phase_t s_phase;
static Bmi088_Status_t s_work;
static Bmi088_Status_t s_status;
static Bmi088_Snapshot_t s_snapshot;
static uint32_t s_due_ms;
static uint32_t s_sequence;
static uint32_t s_last_sensor_time;
static uint32_t s_last_poll_ms;
static uint8_t s_have_sensor_time;
static uint8_t s_snapshot_ready;
static uint8_t s_config_index;
static uint8_t s_reset_done;
static uint8_t s_read_failures;
static uint8_t s_init_failures;
static volatile uint8_t s_reinit_requested;

/** @brief 发布轻量诊断快照；临界区只执行复制。 */
static Bmi088_Result_t Bmi088_PublishResult(Bmi088_Result_t result)
{
  uint32_t primask;
  s_work.last_result = result;
  primask = BSP_Critical_Enter();
  s_status = s_work;
  BSP_Critical_Exit(primask);
  return result;
}

/** @brief 只撤销旧样本可用标志，保留累计序号与诊断计数。 */
static void Bmi088_InvalidateSnapshot(void)
{
  uint32_t primask = BSP_Critical_Enter();
  s_snapshot_ready = 0U;
  BSP_Critical_Exit(primask);
}

/** @brief 将BSP返回码转换为驱动结果；不累计错误。 */
static Bmi088_Result_t Bmi088_MapBsp(BSP_Status_t result)
{
  s_work.last_bsp_status = (uint8_t)result;
  if (result == BSP_STATUS_OK) { return BMI088_RESULT_OK; }
  if (result == BSP_STATUS_TIMEOUT) { return BMI088_RESULT_TIMEOUT; }
  return BMI088_RESULT_IO_ERROR;
}

/** @brief 读取寄存器；SPI命令、加速度dummy均由Driver处理。 */
static Bmi088_Result_t Bmi088_Read(uint8_t device, uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t tx[BMI088_SPI_BUFFER_SIZE];
  uint8_t rx[BMI088_SPI_BUFFER_SIZE];
  uint16_t prefix;
  uint16_t addr;
  Bmi088_Result_t result;
  if ((data == 0) || (len == 0U) || (device > BMI088_GYRO_DEVICE)) { return BMI088_RESULT_INVALID_PARAM; }
  if (s_work.bus == BMI088_BUS_I2C) {
    addr = (uint16_t)((device == BMI088_ACCEL_DEVICE) ? s_work.accel_addr_7bit : s_work.gyro_addr_7bit);
    return Bmi088_MapBsp(BSP_I2C_MemRead((uint16_t)(addr << 1), reg, data, len, BMI088_IO_TIMEOUT_MS));
  }
  prefix = (device == BMI088_ACCEL_DEVICE) ? 2U : 1U;
  if ((uint32_t)len + prefix > sizeof(tx)) { return BMI088_RESULT_INVALID_PARAM; }
  memset(tx, 0, sizeof(tx));
  tx[0] = (uint8_t)(reg | 0x80U);
  result = Bmi088_MapBsp(BSP_IMU_SPI_Transfer(device, tx, rx, (uint16_t)(len + prefix), BMI088_IO_TIMEOUT_MS));
  if (result == BMI088_RESULT_OK) { memcpy(data, &rx[prefix], len); }
  return result;
}

/** @brief 写入单字节寄存器；陀螺不使用突发写。 */
static Bmi088_Result_t Bmi088_Write(uint8_t device, uint8_t reg, uint8_t value)
{
  uint8_t tx[2];
  uint8_t rx[2];
  uint16_t addr;
  if (s_work.bus == BMI088_BUS_I2C) {
    addr = (uint16_t)((device == BMI088_ACCEL_DEVICE) ? s_work.accel_addr_7bit : s_work.gyro_addr_7bit);
    return Bmi088_MapBsp(BSP_I2C_MemWrite((uint16_t)(addr << 1), reg, &value, 1U, BMI088_IO_TIMEOUT_MS));
  }
  tx[0] = (uint8_t)(reg & 0x7FU);
  tx[1] = value;
  return Bmi088_MapBsp(BSP_IMU_SPI_Transfer(device, tx, rx, sizeof(tx), BMI088_IO_TIMEOUT_MS));
}

/** @brief 在当前阶段探测两个允许的I2C地址，SPI只读选中片选。 */
static Bmi088_Result_t Bmi088_Probe(uint8_t device)
{
  uint8_t i;
  uint8_t chip_id = 0U;
  uint8_t first_addr = (device == BMI088_ACCEL_DEVICE) ? 0x18U : 0x68U;
  uint8_t expected = (device == BMI088_ACCEL_DEVICE) ? BMI088_ACCEL_CHIP_ID : BMI088_GYRO_CHIP_ID;
  uint8_t attempts = (s_work.bus == BMI088_BUS_I2C) ? 2U : 1U;
  uint8_t read_ok = 0U;
  Bmi088_Result_t result = BMI088_RESULT_IO_ERROR;
  for (i = 0U; i < attempts; i++) {
    if (s_work.bus == BMI088_BUS_I2C) {
      if (device == BMI088_ACCEL_DEVICE) { s_work.accel_addr_7bit = (uint8_t)(first_addr + i); }
      else { s_work.gyro_addr_7bit = (uint8_t)(first_addr + i); }
    }
    result = Bmi088_Read(device, 0x00U, &chip_id, 1U);
    if (result == BMI088_RESULT_OK) {
      read_ok = 1U;
      if (device == BMI088_ACCEL_DEVICE) { s_work.accel_chip_id = chip_id; }
      else { s_work.gyro_chip_id = chip_id; }
      if (chip_id == expected) { return BMI088_RESULT_OK; }
    }
  }
  return (read_ok != 0U) ? BMI088_RESULT_BAD_ID : result;
}

/** @brief 计算持续初始化失败的退避，前5次100ms，之后500ms。 */
static uint32_t Bmi088_RetryDelay(void)
{
  return (s_init_failures <= BMI088_FAST_RETRY_LIMIT) ? BMI088_FAST_RETRY_MS : BMI088_SLOW_RETRY_MS;
}

/** @brief 初始化或周期IO失败的唯一记录入口；无数据不调用。 */
static Bmi088_Result_t Bmi088_Fail(Bmi088_Result_t result, uint32_t now_ms, uint8_t initializing)
{
  s_work.error_count++;
  if (initializing != 0U) {
    if (s_init_failures < 255U) { s_init_failures++; }
  } else if (s_read_failures < 255U) { s_read_failures++; }
  if ((initializing != 0U) || (s_read_failures >= BMI088_IO_FAILURE_LIMIT)) {
    s_work.initialized = 0U;
    Bmi088_InvalidateSnapshot();
    s_phase = BMI088_PHASE_RETRY;
    s_due_ms = now_ms + Bmi088_RetryDelay();
  }
  return Bmi088_PublishResult(result);
}

/** @brief 开始一次初始化尝试；只调用有界BSP入口并安排后续等待。 */
static Bmi088_Result_t Bmi088_Start(uint32_t now_ms, uint8_t recover)
{
  Bmi088_Result_t result;
  s_work.initialized = 0U;
  s_work.last_stage = BMI088_STAGE_BUS;
  s_work.reinit_count++;
  s_work.accel_chip_id = 0U;
  s_work.gyro_chip_id = 0U;
  s_reset_done = 0U;
  s_config_index = 0U;
  s_read_failures = 0U;
  s_have_sensor_time = 0U;
  Bmi088_InvalidateSnapshot();
  if (s_work.bus == BMI088_BUS_I2C) {
    result = Bmi088_MapBsp((recover != 0U) ? BSP_I2C_Recover() : BSP_I2C_Init());
  } else {
    result = Bmi088_MapBsp((recover != 0U) ? BSP_IMU_SPI_Recover() : BSP_IMU_SPI_Init());
  }
  if (result != BMI088_RESULT_OK) { return Bmi088_Fail(result, now_ms, 1U); }
  s_work.last_stage = BMI088_STAGE_POWER_WAIT;
  s_phase = BMI088_PHASE_SWITCH;
  s_due_ms = now_ms + 2U;
  return Bmi088_PublishResult(BMI088_RESULT_OK);
}

/** @brief 按小端解析带符号16bit测量，避免依赖char符号性。 */
static int16_t Bmi088_ReadS16(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/**
 * @brief 读取完整新鲜六轴；加速度DRDY与陀螺FIFO均有新数据才发布。
 * @details sensortime为自由运行时钟，仅作附加去重，不能替代ACC_STATUS。
 * 最多读取4个陀螺帧，使用其中最新一个；严重积压清FIFO等待下一轮。
 */
static Bmi088_Result_t Bmi088_ReadSample(uint32_t now_ms)
{
  uint8_t accel_status;
  uint8_t fifo_status;
  uint8_t accel_data[9];
  uint8_t gyro_data[BMI088_GYRO_FRAME_BYTES * BMI088_GYRO_DRAIN_MAX];
  uint8_t temp_data[2];
  uint8_t frames;
  uint8_t i;
  uint16_t last_offset;
  int16_t temperature_raw;
  uint32_t sensor_time;
  uint32_t primask;
  Bmi088_Result_t result;
  Bmi088_Snapshot_t candidate;

  s_work.last_stage = BMI088_STAGE_DATA_READ;
  result = Bmi088_Read(BMI088_ACCEL_DEVICE, 0x03U, &accel_status, 1U);
  if (result != BMI088_RESULT_OK) { return Bmi088_Fail(result, now_ms, 0U); }
  if ((accel_status & BMI088_ACCEL_DATA_READY) == 0U) { return Bmi088_PublishResult(BMI088_RESULT_NO_DATA); }
  result = Bmi088_Read(BMI088_GYRO_DEVICE, 0x0EU, &fifo_status, 1U);
  if (result != BMI088_RESULT_OK) { return Bmi088_Fail(result, now_ms, 0U); }
  frames = (uint8_t)(fifo_status & 0x7FU);
  if (((fifo_status & 0x80U) != 0U) || (frames > BMI088_GYRO_DRAIN_MAX)) {
    result = Bmi088_Write(BMI088_GYRO_DEVICE, 0x3EU, BMI088_GYRO_STREAM_VALUE);
    if (result != BMI088_RESULT_OK) { return Bmi088_Fail(result, now_ms, 0U); }
    s_work.fifo_drop_count++;
    return Bmi088_PublishResult(BMI088_RESULT_NO_DATA);
  }
  if (frames == 0U) { return Bmi088_PublishResult(BMI088_RESULT_NO_DATA); }
  result = Bmi088_Read(BMI088_ACCEL_DEVICE, 0x12U, accel_data, sizeof(accel_data));
  if (result != BMI088_RESULT_OK) { return Bmi088_Fail(result, now_ms, 0U); }
  sensor_time = (uint32_t)accel_data[6] | ((uint32_t)accel_data[7] << 8) | ((uint32_t)accel_data[8] << 16);
  if ((s_have_sensor_time != 0U) && (sensor_time == s_last_sensor_time)) { return Bmi088_PublishResult(BMI088_RESULT_NO_DATA); }
  result = Bmi088_Read(BMI088_GYRO_DEVICE, 0x3FU, gyro_data, (uint16_t)(frames * BMI088_GYRO_FRAME_BYTES));
  if (result != BMI088_RESULT_OK) { return Bmi088_Fail(result, now_ms, 0U); }
  result = Bmi088_Read(BMI088_ACCEL_DEVICE, 0x22U, temp_data, sizeof(temp_data));
  if (result != BMI088_RESULT_OK) { return Bmi088_Fail(result, now_ms, 0U); }
  /* 手册将TEMP_MSB=0x80定义为无效；等待温度首次有效，不伪造23℃。 */
  if (temp_data[0] == 0x80U) { return Bmi088_PublishResult(BMI088_RESULT_NO_DATA); }
  temperature_raw = (int16_t)(((uint16_t)temp_data[0] << 3) | ((uint16_t)temp_data[1] >> 5));
  if (temperature_raw > 1023) { temperature_raw = (int16_t)(temperature_raw - 2048); }
  memset(&candidate, 0, sizeof(candidate));
  last_offset = (uint16_t)((frames - 1U) * BMI088_GYRO_FRAME_BYTES);
  for (i = 0U; i < 3U; i++) {
    candidate.accel_g[i] = (float)Bmi088_ReadS16(&accel_data[(uint16_t)i * 2U]) * (12.0f / 32768.0f);
    candidate.gyro_dps[i] = (float)Bmi088_ReadS16(&gyro_data[last_offset + (uint16_t)i * 2U]) * (1000.0f / 32768.0f);
  }
  candidate.temperature_c = (float)temperature_raw * 0.125f + 23.0f;
  s_sequence++;
  if (s_sequence == 0U) { s_sequence = 1U; }
  candidate.rx_sequence = s_sequence;
  candidate.sample_time_ms = now_ms;
  candidate.error_count = s_work.error_count;
  candidate.reinit_count = s_work.reinit_count;
  candidate.fifo_drop_count = s_work.fifo_drop_count;
  /* 只使用本轮已经完整成功的读取结果；memset保证未使用的FIFO尾字节为0。 */
  memcpy(candidate.raw.accel, accel_data, sizeof(candidate.raw.accel));
  memcpy(candidate.raw.gyro, gyro_data, (uint16_t)(frames * BMI088_GYRO_FRAME_BYTES));
  memcpy(candidate.raw.temperature, temp_data, sizeof(candidate.raw.temperature));
  candidate.raw.accel_status = accel_status;
  candidate.raw.gyro_fifo_status = fifo_status;
  candidate.raw.gyro_frame_count = frames;
  candidate.raw.bus = (uint8_t)s_work.bus;
  candidate.raw.accel_addr_7bit = s_work.accel_addr_7bit;
  candidate.raw.gyro_addr_7bit = s_work.gyro_addr_7bit;
  candidate.raw.accel_chip_id = s_work.accel_chip_id;
  candidate.raw.gyro_chip_id = s_work.gyro_chip_id;
  primask = BSP_Critical_Enter();
  s_snapshot = candidate;
  s_snapshot_ready = 1U;
  BSP_Critical_Exit(primask);
  s_last_sensor_time = sensor_time;
  s_have_sensor_time = 1U;
  s_read_failures = 0U;
  s_work.rx_sequence = candidate.rx_sequence;
  s_work.sample_time_ms = now_ms;
  s_work.accel_sensor_time = sensor_time;
  s_work.last_stage = BMI088_STAGE_NONE;
  return Bmi088_PublishResult(BMI088_RESULT_OK);
}

/** @brief 接受初始化请求；OK不表示双器件自检已经完成。 */
Bmi088_Result_t Sensor_BMI088_Init(Bmi088_Bus_t bus)
{
  uint32_t primask;
  if ((bus != BMI088_BUS_I2C) && (bus != BMI088_BUS_SPI)) { return BMI088_RESULT_INVALID_PARAM; }
  s_work.bus = bus;
  s_work.accel_addr_7bit = (bus == BMI088_BUS_I2C) ? 0x18U : 0U;
  s_work.gyro_addr_7bit = (bus == BMI088_BUS_I2C) ? 0x68U : 0U;
  s_init_failures = 0U;
  primask = BSP_Critical_Enter();
  s_reinit_requested = 0U;
  BSP_Critical_Exit(primask);
  return Bmi088_Start(BSP_Time_GetTickMs(), 0U);
}

/** @brief 仅提交恢复标志，禁止在Health回调中操作总线。 */
void Sensor_BMI088_RequestReinit(void)
{
  uint32_t primask = BSP_Critical_Enter();
  s_reinit_requested = 1U;
  BSP_Critical_Exit(primask);
}

/** @brief 由sensor任务分阶段初始化或读取；每轮初始化最多一个配置事务。 */
Bmi088_Result_t Sensor_BMI088_Service(uint32_t now_ms)
{
  uint32_t primask;
  uint8_t reinit;
  uint8_t value = 0U;
  const Bmi088_RegisterConfig_t *config;
  Bmi088_Result_t result = BMI088_RESULT_NO_DATA;
  if (s_work.bus == 0) { return BMI088_RESULT_NO_DATA; }
  primask = BSP_Critical_Enter();
  reinit = s_reinit_requested;
  s_reinit_requested = 0U;
  BSP_Critical_Exit(primask);
  if ((reinit != 0U) && (s_phase != BMI088_PHASE_RETRY)) {
    s_phase = BMI088_PHASE_RETRY;
    s_due_ms = now_ms;
    s_work.initialized = 0U;
    Bmi088_InvalidateSnapshot();
  }
  if ((s_phase != BMI088_PHASE_RUNNING) && ((int32_t)(now_ms - s_due_ms) < 0)) { return Bmi088_PublishResult(BMI088_RESULT_NO_DATA); }
  if (s_phase == BMI088_PHASE_RETRY) {
    s_work.last_stage = BMI088_STAGE_RETRY_WAIT;
    result = Bmi088_Start(now_ms, 1U);
    return (result == BMI088_RESULT_OK) ? Bmi088_PublishResult(BMI088_RESULT_NO_DATA) : result;
  }
  switch (s_phase) {
    case BMI088_PHASE_SWITCH:
      s_work.last_stage = BMI088_STAGE_PROBE_ACCEL;
      if (s_work.bus == BMI088_BUS_SPI) { result = Bmi088_Read(BMI088_ACCEL_DEVICE, 0x00U, &value, 1U); }
      else { result = BMI088_RESULT_OK; }
      if (result == BMI088_RESULT_OK) { s_phase = BMI088_PHASE_ACCEL_ID; }
      break;
    case BMI088_PHASE_ACCEL_ID:
      s_work.last_stage = BMI088_STAGE_PROBE_ACCEL;
      result = Bmi088_Probe(BMI088_ACCEL_DEVICE);
      if (result == BMI088_RESULT_OK) { s_phase = BMI088_PHASE_GYRO_ID; }
      break;
    case BMI088_PHASE_GYRO_ID:
      s_work.last_stage = BMI088_STAGE_PROBE_GYRO;
      result = Bmi088_Probe(BMI088_GYRO_DEVICE);
      if (result == BMI088_RESULT_OK) { s_phase = (s_reset_done != 0U) ? BMI088_PHASE_CONFIG : BMI088_PHASE_ACCEL_RESET; }
      break;
    case BMI088_PHASE_ACCEL_RESET:
      s_work.last_stage = BMI088_STAGE_RESET;
      result = Bmi088_Write(BMI088_ACCEL_DEVICE, 0x7EU, 0xB6U);
      if (result == BMI088_RESULT_OK) { s_phase = BMI088_PHASE_GYRO_RESET; s_due_ms = now_ms + 2U; }
      break;
    case BMI088_PHASE_GYRO_RESET:
      s_work.last_stage = BMI088_STAGE_RESET;
      result = Bmi088_Write(BMI088_GYRO_DEVICE, 0x14U, 0xB6U);
      if (result == BMI088_RESULT_OK) { s_reset_done = 1U; s_phase = BMI088_PHASE_SWITCH; s_due_ms = now_ms + 30U; }
      break;
    case BMI088_PHASE_CONFIG:
      s_work.last_stage = BMI088_STAGE_CONFIG;
      config = &s_config[s_config_index];
      result = Bmi088_Write(config->device, config->reg, config->value);
      if (result == BMI088_RESULT_OK) {
        s_due_ms = now_ms + config->delay_ms;
        s_config_index++;
        if (s_config_index >= (sizeof(s_config) / sizeof(s_config[0]))) { s_config_index = 0U; s_phase = BMI088_PHASE_VERIFY; }
      }
      break;
    case BMI088_PHASE_VERIFY:
      s_work.last_stage = BMI088_STAGE_VERIFY;
      config = &s_config[s_config_index];
      result = Bmi088_Read(config->device, config->reg, &value, 1U);
      if ((result == BMI088_RESULT_OK) && ((value & config->mask) != (config->value & config->mask))) { result = BMI088_RESULT_CONFIG_ERROR; }
      if (result == BMI088_RESULT_OK) {
        s_config_index++;
        if (s_config_index >= (sizeof(s_config) / sizeof(s_config[0]))) { s_phase = BMI088_PHASE_ERROR_CHECK; }
      }
      break;
    case BMI088_PHASE_ERROR_CHECK:
      s_work.last_stage = BMI088_STAGE_VERIFY;
      result = Bmi088_Read(BMI088_ACCEL_DEVICE, 0x02U, &value, 1U);
      if ((result == BMI088_RESULT_OK) && ((value & 0x1DU) != 0U)) { result = BMI088_RESULT_CONFIG_ERROR; }
      if (result == BMI088_RESULT_OK) { s_phase = BMI088_PHASE_FIFO_CLEAR; }
      break;
    case BMI088_PHASE_FIFO_CLEAR:
      s_work.last_stage = BMI088_STAGE_CONFIG;
      result = Bmi088_Write(BMI088_GYRO_DEVICE, 0x3EU, BMI088_GYRO_STREAM_VALUE);
      if (result == BMI088_RESULT_OK) {
        s_work.initialized = 1U;
        s_init_failures = 0U;
        s_phase = BMI088_PHASE_RUNNING;
        s_last_poll_ms = now_ms;
      }
      break;
    case BMI088_PHASE_RUNNING:
      if ((uint32_t)(now_ms - s_last_poll_ms) < 10U) { return Bmi088_PublishResult(BMI088_RESULT_NO_DATA); }
      s_last_poll_ms = now_ms;
      return Bmi088_ReadSample(now_ms);
    default:
      return Bmi088_PublishResult(BMI088_RESULT_NO_DATA);
  }
  if (result != BMI088_RESULT_OK) { return Bmi088_Fail(result, now_ms, 1U); }
  return Bmi088_PublishResult(BMI088_RESULT_NO_DATA);
}

/** @brief 同一临界区复制解析量和原字节；初始化/恢复时不返回旧测量或旧原字节。 */
Bmi088_Result_t Sensor_BMI088_CopySnapshot(Bmi088_Snapshot_t *out)
{
  uint32_t primask;
  uint8_t ready;
  if (out == 0) { return BMI088_RESULT_INVALID_PARAM; }
  primask = BSP_Critical_Enter();
  ready = s_snapshot_ready;
  if (ready != 0U) { *out = s_snapshot; }
  BSP_Critical_Exit(primask);
  return (ready != 0U) ? BMI088_RESULT_OK : BMI088_RESULT_NO_DATA;
}

/** @brief 复制统一状态缓存，任何调用方均不借此访问硬件。 */
Bmi088_Result_t Sensor_BMI088_GetStatus(Bmi088_Status_t *out)
{
  uint32_t primask;
  if (out == 0) { return BMI088_RESULT_INVALID_PARAM; }
  primask = BSP_Critical_Enter();
  *out = s_status;
  BSP_Critical_Exit(primask);
  return BMI088_RESULT_OK;
}
