/**
 * @file test_bmi088_platform.c
 * @brief 通过真实F407 Adapter验证IMU选型、恢复门控、单位与只读诊断。
 * @details 驱动替身注入设备事实；测试不访问实际I2C/SPI，不代替实板验轴。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } DMA_HandleTypeDef;
typedef struct { uint32_t CNT, SR; } HostTimer;
#define UART_WORDLENGTH_8B 0U
#define UART_WORDLENGTH_9B 0x1000U
#define UART_PARITY_NONE 0U
#define UART_PARITY_EVEN 0x0400U
#define UART_PARITY_ODD 0x0600U
#define UART_STOPBITS_1 0U
#define UART_STOPBITS_2 0x2000U
#define UART_HWCONTROL_NONE 0U
#define UART_HWCONTROL_RTS 0x0100U
#define UART_HWCONTROL_CTS 0x0200U
#define UART_HWCONTROL_RTS_CTS 0x0300U
static HostTimer s_host_timer;
#define TIM6 (&s_host_timer)
#define TIM_SR_UIF 1U
static uint32_t __get_PRIMASK(void) { return 0U; }
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static uint32_t HAL_GetTick(void) { return 0U; }
static uint32_t HAL_GetUIDw0(void) { return 0U; }
static uint32_t HAL_GetUIDw1(void) { return 0U; }
static uint32_t HAL_GetUIDw2(void) { return 0U; }
static unsigned s_critical_depth;
/** @brief 驱动和时钟替身拒绝在复制临界区内运行。 */
static void TestEnterCritical(void) { assert(s_critical_depth == 0U); ++s_critical_depth; }
/** @brief 保证每次快照同步均对称结束。 */
static void TestExitCritical(void) { assert(s_critical_depth == 1U); --s_critical_depth; }
#define taskENTER_CRITICAL() TestEnterCritical()
#define taskEXIT_CRITICAL() TestExitCritical()

#include "../../Framework/Src/px4lite_platform_f407.c"

static uint32_t s_now;
static unsigned s_clock_calls, s_services, s_copies, s_status_reads, s_inits, s_requests;
static PlatformImuResult_t s_service_result = PLATFORM_IMU_OK;
static PlatformImuResult_t s_copy_result = PLATFORM_IMU_OK;
static PlatformImuSnapshot_t s_sample;
static PlatformImuStatus_t s_status;
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
static Px4Lite_Bmi088Sample_t s_uplink_sample;
static unsigned s_uplink_published, s_uplink_invalidated;
/** @brief 捕获sensor发布的同次帧；不编码或发送，临界区内禁止跨层调用。 */
void Px4Lite_Bmi088UplinkPublish(const Px4Lite_Bmi088Sample_t *sample)
{ assert(s_critical_depth == 0U && sample != 0); s_uplink_sample=*sample; ++s_uplink_published; }
/** @brief 只记录失效请求，不做总线或通信工作。 */
void Px4Lite_Bmi088UplinkInvalidate(void)
{ assert(s_critical_depth == 0U); ++s_uplink_invalidated; }
#endif

/** @brief 统计时基访问，验证诊断Getter不读取当前时间。 */
uint32_t BSP_Time_GetTickMs(void)
{ assert(s_critical_depth == 0U); ++s_clock_calls; return s_now; }

/** @brief 模拟sensor唯一硬件所有者，每轮可返回新样本、等待或真实错误。 */
PlatformImuResult_t PlatformImu_Service(uint32_t now_ms)
{
  assert(s_critical_depth == 0U && now_ms == s_now);
  ++s_services;
  s_status.last_result = s_service_result;
  if (s_service_result == PLATFORM_IMU_OK) {
    s_status.rx_sequence = s_sample.rx_sequence;
    s_status.sample_time_ms = s_sample.sample_time_ms;
  }
  return s_service_result;
}

/** @brief 复制确定性样本，不把Driver私有状态直接交给业务消费者。 */
PlatformImuResult_t PlatformImu_CopySnapshot(PlatformImuSnapshot_t *out)
{ assert(s_critical_depth == 0U); ++s_copies; *out = s_sample; return s_copy_result; }

/** @brief 复制驱动状态；调用计数可捕获Debug越界访问驱动的行为。 */
PlatformImuResult_t PlatformImu_GetStatus(PlatformImuStatus_t *out)
{ assert(s_critical_depth == 0U); ++s_status_reads; *out = s_status; return PLATFORM_IMU_OK; }

#if PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088
/** @brief 记录配置总线；不在测试中执行分阶段芯片初始化。 */
Bmi088_Result_t Sensor_BMI088_Init(Bmi088_Bus_t bus)
{
  assert(s_critical_depth == 0U && (unsigned)bus == PX4LITE_IMU_BMI088_BUS);
  ++s_inits;
  s_status.bus = bus;
  s_status.accel_chip_id = 0x1EU;
  s_status.gyro_chip_id = 0x0FU;
  s_status.accel_addr_7bit = (bus == BMI088_BUS_I2C) ? 0x19U : 0U;
  s_status.gyro_addr_7bit = (bus == BMI088_BUS_I2C) ? 0x69U : 0U;
  s_status.initialized = 1U;
  s_status.fifo_drop_count = 7U;
  ++s_status.reinit_count;
  return BMI088_RESULT_OK;
}
/** @brief 恢复请求只记标志，不做I/O或立即更新初始化次数。 */
void Sensor_BMI088_RequestReinit(void) { assert(s_critical_depth == 0U); ++s_requests; }
#define TEST_IO_ERROR BMI088_RESULT_IO_ERROR
#else
/** @brief 模拟既有MPU初始化，用同一测试核查保留分支。 */
Mpu6050_Result_t Sensor_MPU6050_Init(void)
{
  assert(s_critical_depth == 0U);
  ++s_inits;
  s_status.last_chip_id = 0x68U;
  s_status.i2c_addr_7bit = 0x69U;
  ++s_status.reinit_count;
  return MPU6050_RESULT_OK;
}
/** @brief MPU恢复入口也必须保持请求语义。 */
void Sensor_MPU6050_RequestReinit(void) { assert(s_critical_depth == 0U); ++s_requests; }
#define TEST_IO_ERROR MPU6050_RESULT_IO_ERROR
#endif

/** @brief 注入一帧，采样时间与读取时间故意不同以验证不伪造时戳。 */
static void SetSample(uint32_t sequence, uint32_t time_ms)
{
  memset(&s_sample, 0, sizeof(s_sample));
  s_sample.rx_sequence = sequence;
  s_sample.sample_time_ms = time_ms;
  s_sample.accel_g[0] = 0.1234f;
  s_sample.accel_g[1] = -1.2344f;
  s_sample.accel_g[2] = 0.9996f;
  s_sample.gyro_dps[0] = 1.2344f;
  s_sample.gyro_dps[1] = -2.3456f;
  s_sample.gyro_dps[2] = 3.4567f;
  s_sample.temperature_c = 25.126f;
#if PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088
  s_sample.reinit_count=s_status.reinit_count;
  s_sample.fifo_drop_count=7U;
  s_sample.raw.bus=(uint8_t)PX4LITE_IMU_BMI088_BUS;
  s_sample.raw.accel_addr_7bit=s_status.accel_addr_7bit;
  s_sample.raw.gyro_addr_7bit=s_status.gyro_addr_7bit;
  s_sample.raw.accel_chip_id=0x1EU;s_sample.raw.gyro_chip_id=0x0FU;
  s_sample.raw.accel_status=0x80U;s_sample.raw.gyro_fifo_status=2U;s_sample.raw.gyro_frame_count=2U;
  s_sample.raw.accel[0]=0x12U;s_sample.raw.accel[8]=0x34U;
  s_sample.raw.gyro[0]=0x56U;s_sample.raw.gyro[11]=0x78U;
  s_sample.raw.temperature[0]=0x09U;s_sample.raw.temperature[1]=0xBFU;
#endif
  s_now = time_ms + 3U;
  s_service_result = PLATFORM_IMU_OK;
  s_copy_result = PLATFORM_IMU_OK;
}

/** @brief 诊断访问只能复制sensor已发布的缓存，不访问Driver和时钟。 */
static Px4Lite_ImuDiagnostics_t CopyWithoutIO(void)
{
  Px4Lite_ImuDiagnostics_t first, again;
  unsigned i;
  const unsigned clocks = s_clock_calls, services = s_services;
  const unsigned copies = s_copies, reads = s_status_reads;
  assert(Px4Lite_ImuCopyDiagnostics(&first) == PX4LITE_OK);
  for (i = 0U; i < 4U; ++i) {
    memset(&again, 0xA5, sizeof(again));
    assert(Px4Lite_ImuCopyDiagnostics(&again) == PX4LITE_OK);
    assert(memcmp(&first, &again, sizeof(first)) == 0);
  }
  assert(s_clock_calls == clocks && s_services == services);
  assert(s_copies == copies && s_status_reads == reads && s_critical_depth == 0U);
  return first;
}

/** @brief 初始化后第三个有效新样本开始发布；重复序号不得推进门控。 */
static void TestUnitsAxesAndWarmup(void)
{
  Px4Lite_SensorImu_t out, untouched;
  Px4Lite_ImuDiagnostics_t diag;
  assert(Px4Lite_ImuInit() == PX4LITE_OK && s_inits == 1U);
  memset(&out, 0xA5, sizeof(out)); untouched = out;
  SetSample(1U, 100U);
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IDLE && memcmp(&out, &untouched, sizeof(out)) == 0);
  SetSample(2U, 110U);
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IDLE);
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IDLE && memcmp(&out, &untouched, sizeof(out)) == 0);
  SetSample(3U, 120U);
  assert(Px4Lite_ImuRead(&out) == PX4LITE_OK);
  assert(out.header.sample_time_ms == 120U && out.sample_period_us == 10000U);
  assert(out.accel_mg[0] == 123 && out.accel_mg[2] == 1000);
  assert(out.gyro_mdps[1] == -2346 && out.temperature_cdeg == 2513);
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
  assert(s_uplink_published==1U && s_uplink_invalidated>=1U);
  assert(s_uplink_sample.sample_sequence==3U && s_uplink_sample.sample_time_ms==120U);
  assert(s_uplink_sample.reinit_count==s_sample.reinit_count && s_uplink_sample.fifo_drop_count==7U);
  assert(memcmp(s_uplink_sample.accel_mg,out.accel_mg,sizeof(out.accel_mg))==0);
  assert(memcmp(s_uplink_sample.gyro_mdps,out.gyro_mdps,sizeof(out.gyro_mdps))==0);
  assert(s_uplink_sample.temperature_cdeg==out.temperature_cdeg && s_uplink_sample.sample_period_us==out.sample_period_us);
  assert(memcmp(s_uplink_sample.raw_accel,s_sample.raw.accel,9U)==0);
  assert(memcmp(s_uplink_sample.raw_gyro,s_sample.raw.gyro,24U)==0);
  assert(memcmp(s_uplink_sample.raw_temperature,s_sample.raw.temperature,2U)==0);
  assert(s_uplink_sample.gyro_frame_count==2U && s_uplink_sample.gyro_fifo_status==2U);
  assert(s_uplink_sample.axis_map[0]==PX4LITE_BMI088_AXIS_X && s_uplink_sample.axis_map[1]==PX4LITE_BMI088_AXIS_Y && s_uplink_sample.axis_map[2]==PX4LITE_BMI088_AXIS_Z);
#endif
#if PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088
  assert(out.accel_mg[1] == -1234 && out.gyro_mdps[0] == 1234 && out.gyro_mdps[2] == 3457);
#else
  assert(out.accel_mg[1] == 1234 && out.gyro_mdps[0] == -1234 && out.gyro_mdps[2] == -3457);
#endif
  SetSample(4U, 132U);
  assert(Px4Lite_ImuRead(&out) == PX4LITE_OK && out.sample_period_us == 12000U);
  diag = CopyWithoutIO();
  assert(diag.driver == PX4LITE_IMU_DRIVER && diag.rx_sequence == 4U && diag.sample_time_ms == 132U);
  assert(diag.reinit_count == 1U && diag.device_ready == 1U);
#if PX4LITE_IMU_DRIVER == PX4LITE_IMU_DRIVER_BMI088
  assert(diag.bus == PX4LITE_IMU_BMI088_BUS);
  assert(diag.accel_chip_id == 0x1EU && diag.gyro_chip_id == 0x0FU && diag.fifo_drop_count == 7U);
  assert(diag.accel_addr_7bit == ((diag.bus == 1U) ? 0x19U : 0U));
  assert(diag.gyro_addr_7bit == ((diag.bus == 1U) ? 0x69U : 0U));
#else
  assert(diag.bus == 1U && diag.accel_chip_id == 0x68U && diag.gyro_chip_id == 0x68U);
  assert(diag.accel_addr_7bit == 0x69U && diag.gyro_addr_7bit == 0x69U && diag.fifo_drop_count == 0U);
#endif
}

/** @brief 等待、I/O失败及Snapshot失败均不得伪造新数据；在线瞬态不重新暖机。 */
static void TestFailureAndRecoveryGate(void)
{
  Px4Lite_SensorImu_t out, untouched;
  Px4Lite_ImuDiagnostics_t diag;
  unsigned copies;
  memset(&out, 0xA5, sizeof(out)); untouched = out;
  s_service_result = PLATFORM_IMU_NO_DATA; copies = s_copies;
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IDLE && s_copies == copies);
  assert(memcmp(&out, &untouched, sizeof(out)) == 0);
  s_service_result = TEST_IO_ERROR;
  s_status.error_count = 5U; s_status.last_bsp_status = 3U;
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IO_ERROR && s_copies == copies);
  assert(memcmp(&out, &untouched, sizeof(out)) == 0);
  diag = CopyWithoutIO();
  assert(diag.error_count == 5U && diag.last_result == TEST_IO_ERROR && diag.last_bsp_status == 3U);
  SetSample(5U, 160U); s_copy_result = PLATFORM_IMU_NO_DATA;
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IDLE && memcmp(&out, &untouched, sizeof(out)) == 0);
  s_copy_result = TEST_IO_ERROR;
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IO_ERROR && memcmp(&out, &untouched, sizeof(out)) == 0);
  s_copy_result = PLATFORM_IMU_OK;
  assert(Px4Lite_ImuRead(&out) == PX4LITE_OK);
  untouched = out;
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IDLE && memcmp(&out, &untouched, sizeof(out)) == 0);

  copies = s_services;
  Px4Lite_ImuRequestReinit();
  assert(s_requests == 1U && s_inits == 1U && s_services == copies);
  ++s_status.reinit_count;
  s_service_result = PLATFORM_IMU_NO_DATA;
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IDLE);
  SetSample(6U, 200U);
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IDLE);
  s_service_result = TEST_IO_ERROR;
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IO_ERROR);
  SetSample(7U, 220U);
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IDLE);
  SetSample(8U, 230U);
  assert(Px4Lite_ImuRead(&out) == PX4LITE_IDLE);
  SetSample(9U, 240U);
  assert(Px4Lite_ImuRead(&out) == PX4LITE_OK && out.header.sample_time_ms == 240U);
  diag = CopyWithoutIO();
  assert(diag.reinit_count == 2U && diag.rx_sequence == 9U);
}

/** @brief 同一用例支持BMI I2C、BMI SPI和原MPU编译选择。 */
int main(void)
{
  Px4Lite_ImuDiagnostics_t diag;
  assert(Px4Lite_ImuCopyDiagnostics(0) == PX4LITE_INVALID_PARAM);
  assert(Px4Lite_ImuCopyDiagnostics(&diag) == PX4LITE_NOT_READY);
  assert(s_services == 0U && s_status_reads == 0U && s_clock_calls == 0U);
  assert(Px4Lite_ImuRead(0) == PX4LITE_INVALID_PARAM && s_clock_calls == 0U);
  TestUnitsAxesAndWarmup();
  TestFailureAndRecoveryGate();
  assert(s_critical_depth == 0U);
  puts("BMI088/MPU platform tests passed");
  return 0;
}
