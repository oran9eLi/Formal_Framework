/** @file test_bme280_progress.c
 * @brief 验证真实 BME280 强制测量状态机的许可、推进、时间戳和错误退避。
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "runtime_stubs/bsp_critical.h"
#define BSP_CRITICAL_H
#include "../../Sensor/Src/sensor_bme280.c"
static uint32_t now;
static unsigned forced_writes, data_reads, probes;
static uint8_t busy, fail_write, absent;
uint32_t BSP_Time_GetTickMs(void) { return now; }
void BSP_Time_DelayMs(uint32_t ms) { now += ms; }
BSP_Status_t BSP_I2C_Recover(void) { return BSP_STATUS_OK; }
BSP_Status_t BSP_I2C_IsDeviceReady(uint16_t addr, uint32_t timeout)
{ (void)addr; (void)timeout; ++probes; return absent ? BSP_STATUS_ERROR : BSP_STATUS_OK; }
BSP_Status_t BSP_I2C_MemWrite(uint16_t addr, uint16_t reg, const uint8_t *data, uint16_t len, uint32_t timeout)
{
  (void)addr; (void)len; (void)timeout;
  if (reg == BME280_REG_CTRL_MEAS && ((*data & 3U) == 1U)) {
    ++forced_writes;
    if (fail_write) { return BSP_STATUS_ERROR; }
    now += 2U; /* 模拟触发写事务耗时，检查时间戳不是调用前旧时间。 */
  }
  return BSP_STATUS_OK;
}
BSP_Status_t BSP_I2C_MemRead(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len, uint32_t timeout)
{
  (void)addr; (void)timeout;
  memset(data, 0, len);
  if (reg == BME280_REG_ID) { *data = BME280_CHIP_ID; }
  if (reg == BME280_REG_CALIB_TP) { data[6] = 0x70U; data[7] = 0x8eU; }
  if (reg == BME280_REG_STATUS) { *data = busy ? BME280_STATUS_MEASURING : 0U; }
  if (reg == BME280_REG_DATA) { ++data_reads; }
  return BSP_STATUS_OK;
}
/** @brief 兼容旧签名仅用于 RED 复现，正式测试不定义该宏。 */
static Bme280_Result_t Service(uint8_t trigger)
{
#ifdef TEST_BME_LEGACY
  (void)trigger;
  return Sensor_BME280_Service(now);
#else
  return Sensor_BME280_Service(now, trigger);
#endif
}
int main(void)
{
  Bme280_Snapshot_t snapshot;
  uint32_t started;
  unsigned before, i;
  now = 100U;
  assert(Sensor_BME280_Init() == BME280_RESULT_OK);
  assert(forced_writes == 0U); /* 初始化只配置，不产生未经调度的测量。 */
  assert(Service(0U) == BME280_RESULT_NO_DATA && forced_writes == 0U);
  assert(Service(1U) == BME280_RESULT_NO_DATA && forced_writes == 1U);
  started = now;
  now += 9U;
  assert(Service(0U) == BME280_RESULT_NO_DATA && data_reads == 0U);
  now += 1U;
  assert(Service(0U) == BME280_RESULT_OK && data_reads == 1U);
  assert(Sensor_BME280_CopySnapshot(&snapshot) == BME280_RESULT_OK);
  assert(snapshot.sample_time_ms == started && snapshot.rx_sequence == 1U);
  now += 10U;
  assert(Service(0U) == BME280_RESULT_NO_DATA && data_reads == 1U && forced_writes == 1U);
  now += 980U;
  assert(Service(1U) == BME280_RESULT_NO_DATA);
  started = now;
  now += 1000U; /* 消费者迟到时仍保留真实触发时刻。 */
  assert(Service(0U) == BME280_RESULT_OK);
  assert(Sensor_BME280_CopySnapshot(&snapshot) == BME280_RESULT_OK);
  assert(snapshot.sample_time_ms == started && snapshot.rx_sequence == 2U);
  assert(Service(0U) == BME280_RESULT_NO_DATA);
  assert(Service(1U) == BME280_RESULT_NO_DATA);
  busy = 1U; now += 20U;
  assert(Service(0U) == BME280_RESULT_NO_DATA);
  now += 10U;
  assert(Service(0U) == BME280_RESULT_TIMEOUT);
  busy = 0U;
  before = forced_writes;
  assert(Service(0U) == BME280_RESULT_NO_DATA && forced_writes == before);
  fail_write = 1U;
  for (i = 0U; i < 4U; ++i) {
    assert(Service(1U) == BME280_RESULT_IO_ERROR);
    assert(Service(0U) == BME280_RESULT_NO_DATA);
    now += 1000U;
  }
  fail_write = 0U;
  absent = 1U; Sensor_BME280_RequestReinit();
  assert(Service(1U) == BME280_RESULT_IO_ERROR);
  before = probes;
  now += 10U;
  assert(Service(0U) == BME280_RESULT_NO_DATA && probes == before);
  now += 1000U; absent = 0U;
  assert(Service(1U) == BME280_RESULT_NO_DATA);
  now += 10U;
  assert(Service(0U) == BME280_RESULT_OK);
  now = 0xfffffff8U;
  assert(Service(1U) == BME280_RESULT_NO_DATA);
  started = now;
  now += 10U;
  assert(Service(0U) == BME280_RESULT_OK);
  assert(Sensor_BME280_CopySnapshot(&snapshot) == BME280_RESULT_OK && snapshot.sample_time_ms == started);
  puts("BME280 measurement progress tests passed");
  return 0;
}
