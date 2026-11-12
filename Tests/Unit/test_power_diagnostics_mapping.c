/**
 * @file test_power_diagnostics_mapping.c
 * @brief 通过真实F407 Adapter验证主回路诊断的任务所有权、原子发布与失效语义。
 * @details Driver/BSP仅注入确定性事实；测试不访问真实ADC，不替代现场标定。
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

static unsigned s_critical_depth, s_critical_entries;
/** @brief 在测试中确认所有同步区仅用于复制，I/O替身会拒绝在同步区运行。 */
static void TestEnterCritical(void) { assert(s_critical_depth == 0U); ++s_critical_depth; ++s_critical_entries; }
/** @brief 每次发布或读取结束必须恢复临界区深度。 */
static void TestExitCritical(void) { assert(s_critical_depth == 1U); --s_critical_depth; }
#define taskENTER_CRITICAL() TestEnterCritical()
#define taskEXIT_CRITICAL() TestExitCritical()

#include "px4lite_config.h"
#if defined(TEST_BATTERY2_DISABLED) && TEST_BATTERY2_DISABLED
#undef PX4LITE_ENABLE_BATTERY2
#define PX4LITE_ENABLE_BATTERY2 0U
#endif
#include "../../Framework/Src/px4lite_platform_f407.c"

static uint32_t s_now;
static unsigned s_clock_reads, s_services, s_snapshot_copies, s_voltage_diag_copies, s_current_diag_copies;
static Power_Result_t s_service_result = POWER_RESULT_OK;
static Power_Snapshot_t s_source_sample;
static BSP_ADC2_Diag_t s_source_diag;
static BSP_Status_t s_diag_result = BSP_STATUS_OK;

/** @brief 唯一时基替身；Copy接口若偷偷读取时基，将由调用计数捕获。 */
uint32_t BSP_Time_GetTickMs(void) { assert(s_critical_depth == 0U); ++s_clock_reads; return s_now; }
/** @brief 普通sensor所有者调用的Driver服务替身，临界区内禁止运行。 */
Power_Result_t Sensor_Power2_Service(uint32_t now_ms)
{ assert(s_critical_depth == 0U && now_ms == s_now); ++s_services; return s_service_result; }
/** @brief 驱动复制替身保留传入的原始序号与采样时间。 */
Power_Result_t Sensor_Power2_CopySnapshot(Power_Snapshot_t *out)
{ assert(s_critical_depth == 0U); ++s_snapshot_copies; *out = s_source_sample; return POWER_RESULT_OK; }

/** @brief 初始化只复位Driver事实，不执行ADC转换。 */
Power_Result_t Sensor_Power2_Init(void) { assert(s_critical_depth == 0U); return POWER_RESULT_OK; }
/** @brief 仅在owner执行Service后复制BSP既有缓存，不能从Debug调用。 */
BSP_Status_t BSP_ADC2_GetDiag(BSP_ADC2_Diag_t *out)
{ assert(s_critical_depth == 0U); ++s_voltage_diag_copies; *out = s_source_diag; return s_diag_result; }
/** @brief 电流原码同样来自owner已采样的缓存，I/O替身拒绝临界区内访问。 */
void BSP_ADC_Current_GetDiag(BSP_ADC_CurrentChannel_t channel, uint32_t *mv, uint8_t *floating)
{ assert(s_critical_depth == 0U && channel == BSP_ADC_CURRENT_BATTERY2); ++s_current_diag_copies; *mv = 52U; *floating = 0U; }

#if PX4LITE_ENABLE_BATTERY2
/** @brief 每轮输入设置不同的Driver与BSP字段，检查所有边界映射而非重新实现ADC算法。 */
static void SetSample(uint32_t sequence, uint32_t now_ms)
{
  s_now = now_ms;
  s_service_result = POWER_RESULT_OK;
  s_diag_result = BSP_STATUS_OK;
  memset(&s_source_sample, 0, sizeof(s_source_sample));
  memset(&s_source_diag, 0, sizeof(s_source_diag));
  s_source_sample.rx_sequence = sequence;
  s_source_sample.sample_time_ms = now_ms;
  s_source_sample.voltage_v = 12.009f;
  s_source_sample.current_ma = 800;
  s_source_sample.current_valid = 1U;
  s_source_diag.sequence = sequence;
  s_source_diag.raw_average = 1464U;
  s_source_diag.pin_mv = 1180U;
  s_source_diag.input_mv = 12009U;
  s_source_diag.sample_valid = 1U;
  s_source_diag.reference.vref_raw = 1499U;
  s_source_diag.reference.calibration_raw = 1501U;
  s_source_diag.reference.vdda_mv = 3304U;
  s_source_diag.reference.calculated_vdda_mv = 3304U;
  s_source_diag.reference.status = BSP_ADC_REF_MEASURED;
  s_source_diag.reference.ready = 1U;
}

/** @brief Getter必须只复制：任意重复读取不触发服务、BSP缓存读取或时基访问。 */
static Px4Lite_PowerDiagnostics_t CopyWithoutIO(void)
{
  Px4Lite_PowerDiagnostics_t first, again;
  const unsigned before_clock = s_clock_reads, before_service = s_services;
  const unsigned before_snapshot = s_snapshot_copies, before_diag = s_voltage_diag_copies;
  const unsigned before_current = s_current_diag_copies;
  unsigned i;
  assert(Px4Lite_Battery2CopyDiagnostics(&first) == PX4LITE_OK);
  for (i = 0U; i < 4U; ++i) {
    assert(Px4Lite_Battery2CopyDiagnostics(&again) == PX4LITE_OK);
    assert(memcmp(&first, &again, sizeof(first)) == 0);
  }
  assert(s_clock_reads == before_clock && s_services == before_service);
  assert(s_snapshot_copies == before_snapshot && s_voltage_diag_copies == before_diag);
  assert(s_current_diag_copies == before_current && s_critical_depth == 0U);
  return first;
}

/** @brief 0ms为合法采样时刻，所有单位与状态按同轮事实映射。 */
static void TestMappingAndReadOwnership(void)
{
  Px4Lite_BatteryStatus_t sample;
  Px4Lite_PowerDiagnostics_t out, copied;
  SetSample(1U, 0U);
  assert(Px4Lite_Battery2Read(&sample) == PX4LITE_OK);
  out = CopyWithoutIO();
  assert(out.sequence == 1U && out.sample_time_ms == 0U);
  assert(out.voltage_raw == 1464U && out.voltage_pin_mv == 1180U);
  assert(out.voltage_bsp_mv == 12009U && out.voltage_mv == 12009U);
  assert(out.vref_raw == 1499U && out.vref_cal_raw == 1501U);
  assert(out.vdda_mv == 3304U && out.vdda_calculated_mv == 3304U);
  assert(out.current_pin_mv == 52U && out.current_ma == 800);
  assert(out.voltage_valid && out.current_valid && out.reference_status == PX4LITE_POWER_REF_MEASURED);
  assert((sample.header.flags & PX4LITE_DATA_DEGRADED) == 0U && sample.voltage_mv == 12009U);
  assert(s_services == 1U && s_snapshot_copies == 1U && s_voltage_diag_copies == 1U && s_current_diag_copies == 1U);
  copied = out; copied.voltage_mv = 999U;
  assert(Px4Lite_Battery2CopyDiagnostics(&copied) == PX4LITE_OK && copied.voltage_mv == 12009U);
  s_now = 9000U;
  out = CopyWithoutIO();
  assert(out.sample_time_ms == 0U && out.sequence == 1U);
}

/** @brief 电流失败不授权旧电流针脚；电压失败新尝试明确无效，恢复后重新发布。 */
static void TestFailuresAndNoAttempt(void)
{
  Px4Lite_BatteryStatus_t sample;
  Px4Lite_PowerDiagnostics_t out;
  unsigned current_before, diag_before;
  SetSample(2U, 1000U);
  s_source_sample.current_valid = 0U; s_source_sample.current_ma = 0;
  current_before = s_current_diag_copies;
  assert(Px4Lite_Battery2Read(&sample) == PX4LITE_OK);
  out = CopyWithoutIO();
  assert(out.sequence == 2U && out.voltage_valid && !out.current_valid);
  assert(out.current_pin_mv == 0U && out.current_ma == 0 && s_current_diag_copies == current_before);

  SetSample(3U, 2000U); s_source_diag.sample_valid = 0U; s_service_result = POWER_RESULT_IO_ERROR;
  assert(Px4Lite_Battery2Read(&sample) == PX4LITE_IO_ERROR);
  out = CopyWithoutIO();
  assert(out.sequence == 3U && out.sample_time_ms == 2000U && !out.voltage_valid && !out.current_valid);
  assert(out.voltage_mv == 0U && out.current_ma == 0 && out.current_pin_mv == 0U);

  s_now = 3000U; s_service_result = POWER_RESULT_NO_DATA;
  diag_before = s_voltage_diag_copies;
  assert(Px4Lite_Battery2Read(&sample) == PX4LITE_IDLE);
  out = CopyWithoutIO();
  assert(out.sequence == 3U && out.sample_time_ms == 2000U && s_voltage_diag_copies == diag_before);
  s_service_result = POWER_RESULT_IO_ERROR;
  assert(Px4Lite_Battery2Read(&sample) == PX4LITE_IO_ERROR);
  out = CopyWithoutIO();
  assert(out.sequence == 3U && out.sample_time_ms == 2000U);

  SetSample(4U, 4000U);
  assert(Px4Lite_Battery2Read(&sample) == PX4LITE_OK);
  out = CopyWithoutIO();
  assert(out.sequence == 4U && out.sample_time_ms == 4000U && out.voltage_valid && out.current_valid);
}

/** @brief 参考回退必须显式降级，并保留估算与实际使用的参考电压。 */
static void TestReferenceMapping(void)
{
  static const uint8_t source_status[] = {
    BSP_ADC_REF_FALLBACK_CAL_ZERO, BSP_ADC_REF_FALLBACK_CAL_ERASED, BSP_ADC_REF_FALLBACK_CAL_RANGE,
    BSP_ADC_REF_FALLBACK_READ, BSP_ADC_REF_FALLBACK_RAW, BSP_ADC_REF_FALLBACK_VDDA_RANGE, 255U
  };
  static const uint8_t expected_status[] = {
    PX4LITE_POWER_REF_CAL_INVALID, PX4LITE_POWER_REF_CAL_INVALID, PX4LITE_POWER_REF_CAL_INVALID,
    PX4LITE_POWER_REF_READ_FAILED, PX4LITE_POWER_REF_RAW_INVALID, PX4LITE_POWER_REF_VDDA_INVALID,
    PX4LITE_POWER_REF_UNAVAILABLE
  };
  Px4Lite_BatteryStatus_t sample;
  Px4Lite_PowerDiagnostics_t out;
  unsigned i;
  for (i = 0U; i < sizeof(source_status) / sizeof(source_status[0]); ++i) {
    SetSample(10U + i, 5000U + i * 1000U);
    s_source_diag.reference.status = source_status[i];
    s_source_diag.reference.vdda_mv = 3300U;
    s_source_diag.reference.calculated_vdda_mv = 1200U + i;
    assert(Px4Lite_Battery2Read(&sample) == PX4LITE_OK);
    out = CopyWithoutIO();
    assert(out.reference_status == expected_status[i]);
    assert(out.vdda_mv == 3300U && out.vdda_calculated_mv == 1200U + i);
    assert(sample.header.flags & PX4LITE_DATA_DEGRADED);
  }
  SetSample(20U, 13000U); s_source_diag.reference.ready = 0U;
  assert(Px4Lite_Battery2Read(&sample) == PX4LITE_OK);
  out = CopyWithoutIO();
  assert(out.reference_status == PX4LITE_POWER_REF_UNAVAILABLE && (sample.header.flags & PX4LITE_DATA_DEGRADED));
  SetSample(21U, 14000U);
  assert(Px4Lite_Battery2Read(&sample) == PX4LITE_OK);
  assert((sample.header.flags & PX4LITE_DATA_DEGRADED) == 0U);
}
#endif

/** @brief 默认与第二组关闭配置均验证禁止隐式采样、参数错误和未就绪状态。 */
int main(void)
{
  Px4Lite_PowerDiagnostics_t untouched, out;
  Px4Lite_BatteryStatus_t sample;
  memset(&untouched, 0xA5, sizeof(untouched)); out = untouched;
  assert(Px4Lite_Battery2CopyDiagnostics(NULL) == PX4LITE_INVALID_PARAM);
  assert(Px4Lite_Battery2CopyDiagnostics(&out) == PX4LITE_NOT_READY);
  assert(memcmp(&out, &untouched, sizeof(out)) == 0);
  assert(s_services == 0U && s_snapshot_copies == 0U && s_voltage_diag_copies == 0U && s_current_diag_copies == 0U && s_clock_reads == 0U);
#if PX4LITE_ENABLE_BATTERY2
  assert(Px4Lite_Battery2Read(NULL) == PX4LITE_INVALID_PARAM && s_services == 0U && s_clock_reads == 0U);
  TestMappingAndReadOwnership(); TestFailuresAndNoAttempt(); TestReferenceMapping();
  assert(s_critical_entries != 0U && s_critical_depth == 0U);
#else
  assert(Px4Lite_Battery2Read(&sample) == PX4LITE_NOT_READY);
  assert(s_services == 0U && s_clock_reads == 0U && s_voltage_diag_copies == 0U && s_current_diag_copies == 0U);
#endif
  (void)sample;
  puts("power diagnostics mapping: PASS (owner-only publish; cached reads; invalid attempts; explicit fallback)");
  return 0;
}
