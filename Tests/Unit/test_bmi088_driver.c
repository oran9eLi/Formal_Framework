/**
 * @file test_bmi088_driver.c
 * @brief 真实BMI088驱动在独立寄存器/FIFO模型上的I2C、SPI与故障恢复回归。
 * @details 属于主机测试层，不代替实物接线、轴向、噪声或长稳验收。
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bsp_status.h"
#include "sensor_bmi088.h"

#define BSP_CRITICAL_H
static uint32_t model_critical;
/** @brief 记录临界区，以检出关中断期间硬件访问。 */
static uint32_t BSP_Critical_Enter(void) { return model_critical++; }
/** @brief 检查原中断状态恢复配对。 */
static void BSP_Critical_Exit(uint32_t saved) { assert(model_critical == saved + 1U); model_critical = saved; }

static uint8_t model_regs[2][128];
static uint8_t model_fifo[24];
static uint8_t model_accel_addr;
static uint8_t model_gyro_addr;
static uint8_t model_fifo_frames;
static uint8_t model_spi_selected;
static uint8_t model_fail;
static uint8_t model_ignore_write_reg;
static uint8_t model_fail_device;
static uint8_t model_fail_reg;
static uint32_t model_now;
static uint32_t model_io;
static uint32_t model_recover;
static uint32_t model_dummy_reads;
static uint32_t model_delay_calls;

/** @brief 独立模拟数据手册复位值，不调用生产配置表。 */
static void ModelResetDevice(uint8_t device)
{
  memset(model_regs[device], 0, sizeof(model_regs[device]));
  if (device == 0U) {
    model_regs[0][0] = 0x1EU;
    model_regs[0][0x40] = 0xA8U;
    model_regs[0][0x41] = 1U;
    model_regs[0][0x7C] = 3U;
    model_spi_selected = 0U;
  } else {
    model_regs[1][0] = 0x0FU;
    model_regs[1][0x10] = 0x80U;
    model_regs[1][0x3E] = 8U;
    model_fifo_frames = 0U;
  }
}

/** @brief 重建芯片模型；驱动累计计数通过公开状态取基线。 */
static void ModelReset(void)
{
  ModelResetDevice(0U);
  ModelResetDevice(1U);
  model_accel_addr = 0x18U;
  model_gyro_addr = 0x68U;
  model_fail = 0U;
  model_ignore_write_reg = 0xFFU;
  model_fail_device = 0xFFU;
  model_fail_reg = 0xFFU;
  model_now = 0U;
  model_io = 0U;
  model_recover = 0U;
  model_dummy_reads = 0U;
  model_delay_calls = 0U;
}

/** @brief 提供单次事务的寄存器/FIFO语义，清除数据已读标志。 */
static BSP_Status_t ModelRead(uint8_t device, uint8_t reg, uint8_t *data, uint16_t len)
{
  assert(model_critical == 0U);
  model_io++;
  if ((model_fail != 0U) || ((device == model_fail_device) && (reg == model_fail_reg))) { return BSP_STATUS_TIMEOUT; }
  if ((device == 1U) && (reg == 0x0EU)) {
    assert(len == 1U);
    data[0] = (uint8_t)(model_fifo_frames | (model_regs[1][0x0E] & 0x80U));
  } else if ((device == 1U) && (reg == 0x3FU)) {
    assert((len % 6U) == 0U);
    assert(len <= (uint16_t)model_fifo_frames * 6U);
    memcpy(data, model_fifo, len);
    model_fifo_frames = (uint8_t)(model_fifo_frames - len / 6U);
    memmove(model_fifo, &model_fifo[len], (size_t)model_fifo_frames * 6U);
  } else {
    assert((uint16_t)reg + len <= 128U);
    memcpy(data, &model_regs[device][reg], len);
    if ((device == 0U) && (reg == 0x12U)) { model_regs[0][3] &= 0x7FU; }
  }
  return BSP_STATUS_OK;
}

/** @brief 模拟单字节配置、复位和FIFO清除，支持写入未生效故障。 */
static BSP_Status_t ModelWrite(uint8_t device, uint8_t reg, uint8_t value)
{
  assert(model_critical == 0U);
  model_io++;
  if (model_fail != 0U) { return BSP_STATUS_TIMEOUT; }
  if (reg == model_ignore_write_reg) { return BSP_STATUS_OK; }
  if ((((device == 0U) && (reg == 0x7EU)) || ((device == 1U) && (reg == 0x14U))) && (value == 0xB6U)) {
    ModelResetDevice(device);
  } else {
    model_regs[device][reg] = value;
    if ((device == 1U) && (reg == 0x3EU)) { model_fifo_frames = 0U; model_regs[1][0x0E] = 0U; }
  }
  return BSP_STATUS_OK;
}

/** @brief 模拟共享I2C初始化，不主动操作寄存器。 */
BSP_Status_t BSP_I2C_Init(void) { assert(model_critical == 0U); return BSP_STATUS_OK; }
/** @brief 记录真正由sensor发起的恢复。 */
BSP_Status_t BSP_I2C_Recover(void) { assert(model_critical == 0U); model_recover++; return BSP_STATUS_OK; }
/** @brief 识别HAL风格左移地址，错误地址真实返回失败。 */
BSP_Status_t BSP_I2C_MemRead(uint16_t address, uint16_t reg, uint8_t *data, uint16_t len, uint32_t timeout)
{
  assert(timeout <= 4U);
  if (address == (uint16_t)(model_accel_addr << 1)) { return ModelRead(0U, (uint8_t)reg, data, len); }
  if (address == (uint16_t)(model_gyro_addr << 1)) { return ModelRead(1U, (uint8_t)reg, data, len); }
  model_io++;
  return BSP_STATUS_ERROR;
}
/** @brief 单字节I2C写模型，检查命令边界。 */
BSP_Status_t BSP_I2C_MemWrite(uint16_t address, uint16_t reg, const uint8_t *data, uint16_t len, uint32_t timeout)
{
  assert((len == 1U) && (timeout <= 4U));
  if (address == (uint16_t)(model_accel_addr << 1)) { return ModelWrite(0U, (uint8_t)reg, data[0]); }
  if (address == (uint16_t)(model_gyro_addr << 1)) { return ModelWrite(1U, (uint8_t)reg, data[0]); }
  model_io++;
  return BSP_STATUS_ERROR;
}
/** @brief SPI初始化模型，不模拟协议层。 */
BSP_Status_t BSP_IMU_SPI_Init(void) { assert(model_critical == 0U); return BSP_STATUS_OK; }
/** @brief SPI恢复模型，保持芯片数据等待驱动实际重新配置。 */
BSP_Status_t BSP_IMU_SPI_Recover(void) { assert(model_critical == 0U); model_recover++; return BSP_STATUS_OK; }
/** @brief 以完整CS事务校验读写位及accel专有dummy，不替驱动补协议。 */
BSP_Status_t BSP_IMU_SPI_Transfer(uint8_t device, const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout)
{
  uint16_t prefix;
  uint8_t reg;
  assert(model_critical == 0U);
  assert((device <= 1U) && (len <= 26U) && (timeout <= 4U));
  memset(rx, 0xDD, len);
  reg = (uint8_t)(tx[0] & 0x7FU);
  if ((tx[0] & 0x80U) == 0U) { assert(len == 2U); return ModelWrite(device, reg, tx[1]); }
  prefix = (device == 0U) ? 2U : 1U;
  assert(len > prefix);
  if (device == 0U) {
    model_dummy_reads++;
    if (model_spi_selected == 0U) { model_spi_selected = 1U; model_io++; return BSP_STATUS_OK; }
  }
  return ModelRead(device, reg, &rx[prefix], (uint16_t)(len - prefix));
}
/** @brief 返回驱动测试唯一的真实毫秒时基。 */
uint32_t BSP_Time_GetTickMs(void) { return model_now; }
/** @brief 分阶段驱动不得使用阻塞等待，调用即作为失败。 */
void BSP_Time_DelayMs(uint32_t delay) { (void)delay; model_delay_calls++; assert(0); }

#include "../../Sensor/Src/sensor_bmi088.c"

/** @brief 前进一步10ms，与实际sensor调度一致。 */
static Bmi088_Result_t Step(void)
{
  model_now += 10U;
  return Sensor_BMI088_Service(model_now);
}

/** @brief 用公开状态等待初始化完成，拒绝把Init返回OK当设备已就绪。 */
static void Start(Bmi088_Bus_t bus)
{
  Bmi088_Status_t status;
  Bmi088_Snapshot_t snapshot;
  unsigned i;
  assert(Sensor_BMI088_Init(bus) == BMI088_RESULT_OK);
  assert(Sensor_BMI088_CopySnapshot(&snapshot) == BMI088_RESULT_NO_DATA);
  for (i = 0U; i < 100U; i++) {
    assert(Sensor_BMI088_GetStatus(&status) == BMI088_RESULT_OK);
    if (status.initialized != 0U) { break; }
    assert(Step() == BMI088_RESULT_NO_DATA);
  }
  assert(i < 100U);
  assert(status.accel_chip_id == 0x1EU && status.gyro_chip_id == 0x0FU);
  assert(model_delay_calls == 0U);
  assert(model_regs[0][0x41] == 2U);
  assert(model_regs[1][0x0F] == 1U);
  assert((model_regs[1][0x10] & 7U) == 7U);
  assert(model_regs[1][0x3E] == 0x88U);
}

/** @brief 写入独立工程输入，端序及正负测试值不调用生产转换函数。 */
static void Put16(uint8_t *data, int16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)((uint16_t)value >> 8);
}

/** @brief 注入一个accel DRDY及1..4个gyro FIFO新帧，温度23.25℃。 */
static void Feed(uint32_t sensor_time, uint8_t frames)
{
  uint8_t i;
  model_regs[0][3] = 0x80U;
  Put16(&model_regs[0][0x12], 8192);
  Put16(&model_regs[0][0x14], -16384);
  Put16(&model_regs[0][0x16], 2731);
  model_regs[0][0x18] = (uint8_t)sensor_time;
  model_regs[0][0x19] = (uint8_t)(sensor_time >> 8);
  model_regs[0][0x1A] = (uint8_t)(sensor_time >> 16);
  model_regs[0][0x22] = 0U;
  model_regs[0][0x23] = 0x40U;
  model_fifo_frames = frames;
  for (i = 0U; i < frames && i < 4U; i++) {
    Put16(&model_fifo[(uint16_t)i * 6U], (int16_t)(16384 + i));
    Put16(&model_fifo[(uint16_t)i * 6U + 2U], -8192);
    Put16(&model_fifo[(uint16_t)i * 6U + 4U], 0);
  }
}

/** @brief 在模拟总线读取前保存真实期望字节，避免读取清标志/FIFO消费改变参照。 */
static void CaptureExpectedRaw(uint8_t accel[9], uint8_t gyro[24], uint8_t temperature[2])
{
  memcpy(accel, &model_regs[0][0x12], 9U);
  memcpy(gyro, model_fifo, 24U);
  memcpy(temperature, &model_regs[0][0x22], 2U);
}

/** @brief 原字节逐字节保真，帧外尾部为零，SPI命令/dummy不能混入有效数据。 */
static void AssertRaw(const Bmi088_Snapshot_t *sample, Bmi088_Bus_t bus,
                      const uint8_t accel[9], const uint8_t gyro[24],
                      const uint8_t temperature[2], uint8_t frames,
                      uint8_t accel_status, uint8_t fifo_status)
{
  unsigned i;
  assert(memcmp(sample->raw.accel, accel, 9U) == 0);
  assert(memcmp(sample->raw.gyro, gyro, (size_t)frames * 6U) == 0);
  assert(memcmp(sample->raw.temperature, temperature, 2U) == 0);
  for (i = (unsigned)frames * 6U; i < 24U; ++i) { assert(sample->raw.gyro[i] == 0U); }
  assert(sample->raw.accel_status == accel_status);
  assert(sample->raw.gyro_fifo_status == fifo_status);
  assert(sample->raw.gyro_frame_count == frames);
  assert(sample->raw.bus == (uint8_t)bus);
  assert(sample->raw.accel_addr_7bit == ((bus == BMI088_BUS_I2C) ? model_accel_addr : 0U));
  assert(sample->raw.gyro_addr_7bit == ((bus == BMI088_BUS_I2C) ? model_gyro_addr : 0U));
  assert(sample->raw.accel_chip_id == 0x1EU && sample->raw.gyro_chip_id == 0x0FU);
}

/** @brief 验证两种协议均输出同单位/符号，并依赖真实新样本而非数值变化。 */
static void CheckMeasurements(Bmi088_Bus_t bus)
{
  Bmi088_Snapshot_t sample;
  Bmi088_Snapshot_t kept;
  Bmi088_Status_t before, after;
  uint8_t accel[9], gyro[24], temperature[2];
  uint32_t sequence, calls;
  ModelReset();
  Start(bus);
  assert(Sensor_BMI088_GetStatus(&before) == BMI088_RESULT_OK);
  assert(Step() == BMI088_RESULT_NO_DATA);
  memset(model_fifo, 0xA6, sizeof(model_fifo));
  Feed(0xFFFFF0U, 1U);
  model_regs[0][3] = 0xA5U;
  model_regs[0][0x23] = 0x5BU; /* 11bit温度之外的低位也必须原样保存。 */
  CaptureExpectedRaw(accel, gyro, temperature);
  calls = model_io;
  assert(Step() == BMI088_RESULT_OK);
  assert(model_io == calls + 5U); /* 保存原字节不允许新增一次寄存器读取。 */
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_OK);
  AssertRaw(&sample, bus, accel, gyro, temperature, 1U, 0xA5U, 1U);
  assert(sample.reinit_count == before.reinit_count && sample.fifo_drop_count == before.fifo_drop_count);
  sequence = sample.rx_sequence;
  assert(fabsf(sample.accel_g[0] - 3.0f) < 0.00001f);
  assert(fabsf(sample.accel_g[1] + 6.0f) < 0.00001f);
  assert(fabsf(sample.gyro_dps[0] - 500.0f) < 0.00001f);
  assert(fabsf(sample.gyro_dps[1] + 250.0f) < 0.00001f);
  assert(fabsf(sample.temperature_c - 23.25f) < 0.00001f);
  assert(sample.sample_time_ms == model_now);
  assert(Step() == BMI088_RESULT_NO_DATA);
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_OK && sample.rx_sequence == sequence);
  Feed(0xFFFFF0U, 1U);
  assert(Step() == BMI088_RESULT_NO_DATA);
  Feed(0x10U, 4U);
  CaptureExpectedRaw(accel, gyro, temperature);
  assert(Step() == BMI088_RESULT_OK);
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_OK && sample.rx_sequence == sequence + 1U);
  AssertRaw(&sample, bus, accel, gyro, temperature, 4U, 0x80U, 4U);
  assert(fabsf(sample.gyro_dps[0] - (16387.0f * 1000.0f / 32768.0f)) < 0.00001f);
  /* FIFO所有4帧均保留，但解析仅采用最后一帧；不能将第一帧冒充解析来源。 */
  assert(sample.raw.gyro[0] == 0U && sample.raw.gyro[1] == 0x40U);
  assert(sample.raw.gyro[18] == 3U && sample.raw.gyro[19] == 0x40U);
  Feed(0x20U, 1U);
  model_regs[0][0x22] = 0xC1U;
  model_regs[0][0x23] = 0x1FU;
  CaptureExpectedRaw(accel, gyro, temperature);
  assert(Step() == BMI088_RESULT_OK);
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_OK && sample.temperature_c == -40.0f);
  AssertRaw(&sample, bus, accel, gyro, temperature, 1U, 0x80U, 1U);
  kept = sample;
  Feed(0x30U, 1U);
  model_regs[0][0x22] = 0x80U;
  assert(Step() == BMI088_RESULT_NO_DATA);
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_OK && memcmp(&sample, &kept, sizeof(sample)) == 0);
  Feed(0x40U, 5U);
  assert(Step() == BMI088_RESULT_NO_DATA && model_fifo_frames == 0U);
  assert(Sensor_BMI088_GetStatus(&after) == BMI088_RESULT_OK);
  assert(after.fifo_drop_count == before.fifo_drop_count + 1U);
  assert(after.error_count == before.error_count);
  Feed(0x50U, 2U);
  CaptureExpectedRaw(accel, gyro, temperature);
  assert(Step() == BMI088_RESULT_OK);
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_OK);
  AssertRaw(&sample, bus, accel, gyro, temperature, 2U, 0x80U, 2U);
  assert(sample.fifo_drop_count == before.fifo_drop_count + 1U);
  if (bus == BMI088_BUS_SPI) { assert(model_dummy_reads > 4U); }
}

/** @brief 地址探测需允许两个子器件独立选择高地址。 */
static void CheckAddresses(void)
{
  Bmi088_Status_t status;
  Bmi088_Snapshot_t sample;
  ModelReset(); model_accel_addr = 0x19U; model_gyro_addr = 0x69U;
  Start(BMI088_BUS_I2C);
  assert(Sensor_BMI088_GetStatus(&status) == BMI088_RESULT_OK);
  assert(status.accel_addr_7bit == 0x19U && status.gyro_addr_7bit == 0x69U);
  Feed(1234U, 1U); assert(Step() == BMI088_RESULT_OK);
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_OK);
  assert(sample.raw.accel_addr_7bit == 0x19U && sample.raw.gyro_addr_7bit == 0x69U);
}

/** @brief 真实IO五连错后撤销旧缓存，退避期间无总线访问并可无限恢复。 */
static void CheckRecovery(void)
{
  Bmi088_Status_t baseline, status;
  Bmi088_Snapshot_t sample;
  uint32_t calls, sequence;
  unsigned i;
  ModelReset(); Start(BMI088_BUS_I2C); Feed(100U, 1U); assert(Step() == BMI088_RESULT_OK);
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_OK); sequence = sample.rx_sequence;
  assert(Sensor_BMI088_GetStatus(&baseline) == BMI088_RESULT_OK);
  model_fail = 1U;
  for (i = 0U; i < 5U; i++) { assert(Step() == BMI088_RESULT_TIMEOUT); }
  assert(Sensor_BMI088_GetStatus(&status) == BMI088_RESULT_OK);
  assert(status.initialized == 0U && status.error_count == baseline.error_count + 5U);
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_NO_DATA);
  calls = model_io;
  for (i = 0U; i < 9U; i++) { assert(Step() == BMI088_RESULT_NO_DATA); }
  assert(model_io == calls);
  model_fail = 0U;
  for (i = 0U; i < 100U; i++) {
    assert(Step() == BMI088_RESULT_NO_DATA);
    assert(Sensor_BMI088_GetStatus(&status) == BMI088_RESULT_OK);
    if (status.initialized != 0U) { break; }
  }
  assert(i < 100U && model_recover == 1U);
  assert(status.reinit_count == baseline.reinit_count + 1U);
  Feed(200U, 1U); assert(Step() == BMI088_RESULT_OK);
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_OK && sample.rx_sequence == sequence + 1U);
  assert(sample.reinit_count == status.reinit_count && sample.error_count == baseline.error_count + 5U);
  calls = model_io; Sensor_BMI088_RequestReinit(); assert(model_io == calls);
  assert(Step() == BMI088_RESULT_NO_DATA);
  assert(model_recover == 2U);
  memset(&sample, 0x5A, sizeof(sample));
  assert(Sensor_BMI088_CopySnapshot(&sample) == BMI088_RESULT_NO_DATA);
  assert(sample.raw.accel[0] == 0x5AU && sample.raw.gyro[23] == 0x5AU);
}

/** @brief 芯片ID不符和配置未生效必须分别可见，不能产生测量。 */
static void CheckStartupFaults(void)
{
  Bmi088_Status_t status;
  unsigned i;
  uint32_t calls;
  ModelReset(); model_regs[0][0] = 0x55U;
  assert(Sensor_BMI088_Init(BMI088_BUS_I2C) == BMI088_RESULT_OK);
  assert(Step() == BMI088_RESULT_NO_DATA);
  assert(Step() == BMI088_RESULT_BAD_ID);
  assert(Sensor_BMI088_GetStatus(&status) == BMI088_RESULT_OK && status.initialized == 0U);
  calls = model_io;
  for (i = 0U; i < 9U; i++) { assert(Step() == BMI088_RESULT_NO_DATA); }
  assert(model_io == calls);
  ModelReset(); model_ignore_write_reg = 0x41U;
  assert(Sensor_BMI088_Init(BMI088_BUS_I2C) == BMI088_RESULT_OK);
  for (i = 0U; i < 100U; i++) { if (Step() == BMI088_RESULT_CONFIG_ERROR) { break; } }
  assert(i < 100U);
  assert(Sensor_BMI088_GetStatus(&status) == BMI088_RESULT_OK && status.initialized == 0U);
}

/** @brief 一部分已读而另一部分失败时，不得把半帧发布为新六轴测量。 */
static void CheckPartialAndClockWrap(void)
{
  Bmi088_Snapshot_t before, after;
  uint32_t calls;
  ModelReset(); Start(BMI088_BUS_SPI);
  Feed(100U, 1U); assert(Step() == BMI088_RESULT_OK);
  assert(Sensor_BMI088_CopySnapshot(&before) == BMI088_RESULT_OK);
  Feed(200U, 1U); model_fail_device = 1U; model_fail_reg = 0x3FU;
  assert(Step() == BMI088_RESULT_TIMEOUT);
  assert(Sensor_BMI088_CopySnapshot(&after) == BMI088_RESULT_OK);
  assert(memcmp(&after, &before, sizeof(after)) == 0);
  model_fail_device = 0xFFU; model_fail_reg = 0xFFU;
  /* FIFO已读成功而最后温度失败时，先前完整raw与parsed仍须原子保持。 */
  Feed(250U, 4U); model_fail_device = 0U; model_fail_reg = 0x22U;
  assert(Step() == BMI088_RESULT_TIMEOUT);
  assert(Sensor_BMI088_CopySnapshot(&after) == BMI088_RESULT_OK);
  assert(memcmp(&after, &before, sizeof(after)) == 0);
  model_fail_device = 0xFFU; model_fail_reg = 0xFFU;
  Feed(300U, 1U); assert(Step() == BMI088_RESULT_OK);
  assert(Sensor_BMI088_CopySnapshot(&after) == BMI088_RESULT_OK && after.rx_sequence == before.rx_sequence + 1U);
  calls = model_io;
  assert(Sensor_BMI088_Service(model_now + 1U) == BMI088_RESULT_NO_DATA && model_io == calls);
  model_now = 0xFFFFFFF8U;
  Feed(400U, 1U); assert(Sensor_BMI088_Service(model_now) == BMI088_RESULT_OK);
  Feed(500U, 1U); assert(Step() == BMI088_RESULT_OK);
  assert(model_now == 2U);
  assert(Sensor_BMI088_CopySnapshot(&after) == BMI088_RESULT_OK && after.sample_time_ms == 2U);
}

/** @brief 缺失设备可持续重试，前五次失败100ms，之后500ms且等待时零IO。 */
static void CheckSlowRetry(void)
{
  unsigned attempt, i;
  uint32_t calls, recover, wait_steps;
  ModelReset(); model_regs[0][0] = 0x55U;
  assert(Sensor_BMI088_Init(BMI088_BUS_I2C) == BMI088_RESULT_OK);
  for (attempt = 1U; attempt <= 6U; attempt++) {
    for (i = 0U; i < 20U; i++) { if (Step() == BMI088_RESULT_BAD_ID) { break; } }
    assert(i < 20U);
    calls = model_io; recover = model_recover;
    wait_steps = (attempt <= 5U) ? 10U : 50U;
    for (i = 1U; i < wait_steps; i++) { assert(Step() == BMI088_RESULT_NO_DATA); }
    assert(model_io == calls && model_recover == recover);
    assert(Step() == BMI088_RESULT_NO_DATA);
    assert(model_recover == recover + 1U);
  }
}

/** @brief 执行协议、量纲、新鲜度、地址、自检及恢复边界验证。 */
int main(void)
{
  assert(Sensor_BMI088_Init((Bmi088_Bus_t)0) == BMI088_RESULT_INVALID_PARAM);
  assert(Sensor_BMI088_CopySnapshot(0) == BMI088_RESULT_INVALID_PARAM);
  assert(Sensor_BMI088_GetStatus(0) == BMI088_RESULT_INVALID_PARAM);
  CheckMeasurements(BMI088_BUS_I2C);
  CheckMeasurements(BMI088_BUS_SPI);
  CheckAddresses();
  CheckRecovery();
  CheckStartupFaults();
  CheckPartialAndClockWrap();
  CheckSlowRetry();
  printf("BMI088 paired snapshot=%uB raw=%uB\n", (unsigned)sizeof(Bmi088_Snapshot_t), (unsigned)sizeof(Bmi088_RawSample_t));
  puts("BMI088 driver: I2C/SPI, raw byte fidelity, paired parsing, freshness, startup verification and recovery PASS");
  return 0;
}
