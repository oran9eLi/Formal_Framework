/**
 * @file test_gnss_diagnostics_mapping.c
 * @brief 通过真实F407 Adapter验证GNSS跨层只读映射、独立可用性及禁用配置。
 * @details Driver/BSP仅提供确定性副本；不读寄存器、不执行Service、UART发送或DMA消费。
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

#include "px4lite_config.h"
#if defined(TEST_GNSS_DISABLED) && TEST_GNSS_DISABLED
/* 仅在当前测试编译单元切换分支，不修改生产配置或要求生产宏可覆盖。 */
#undef PX4LITE_ENABLE_GNSS
#define PX4LITE_ENABLE_GNSS 0U
#endif
#include "../../Framework/Src/px4lite_platform_f407.c"

static Gnss_Diagnostics_t s_driver;
static BSP_GNSS_TransportInfo_t s_transport;
static Gnss_Result_t s_driver_result = GNSS_RESULT_OK;
static BSP_Status_t s_transport_result = BSP_STATUS_OK;
static unsigned s_driver_copies, s_transport_copies;

/** @brief 仅复制服务已发布的软件事实，不执行sensor服务。 */
Gnss_Result_t Sensor_GNSS_CopyDiagnostics(Gnss_Diagnostics_t *out)
{ s_driver_copies++; *out = s_driver; return s_driver_result; }
/** @brief 仅复制硬件事实；刻意与Driver填入不同计数和速率。 */
BSP_Status_t BSP_GNSS_CopyTransportInfo(BSP_GNSS_TransportInfo_t *out)
{ s_transport_copies++; *out = s_transport; return s_transport_result; }

#if PX4LITE_ENABLE_GNSS
/** @brief 用不同哨兵数值区分软件消费、硬件搬运和各个错误/应答字段。 */
static void FillDistinctFacts(void)
{
  memset(&s_driver, 0, sizeof(s_driver)); memset(&s_transport, 0, sizeof(s_transport));
  s_driver.measured_ms = 1U; s_driver.service_count = 2U; s_driver.rx_bytes = 3U;
  s_driver.valid_nmea = 4U; s_driver.valid_ubx = 5U; s_driver.checksum_errors = 6U;
  s_driver.framing_resets = 7U; s_driver.tx_commands = 8U; s_driver.ack_count = 9U;
  s_driver.nak_count = 10U; s_driver.timeout_count = 11U; s_driver.baud_changes = 12U;
  s_driver.last_rx_ms = 13U; s_driver.last_frame_ms = 14U; s_driver.baud_bps = 9600U;
  s_driver.protocol_version_x100 = 2712U; s_driver.protocol = GNSS_PROTOCOL_UBX;
  s_driver.state = GNSS_LINK_UNCONFIRMED; s_driver.config_step = 2U; s_driver.config_total = 5U;
  s_driver.rx_seen = s_driver.frame_seen = s_driver.version_seen = 1U;
  s_transport.baud_bps = 115200U; s_transport.rx_bytes = 31U; s_transport.rx_drop_bytes = 32U;
  s_transport.uart_errors = 33U; s_transport.dma_errors = 34U; s_transport.tx_packets = 35U;
  s_transport.tx_failures = 36U; s_transport.recoveries = 37U;
  s_transport.ndtr = 38U; s_transport.queued_bytes = 39U;
  s_transport.initialized = s_transport.tx_busy = 1U;
}

/** @brief 验证所有跨层字段与语义对应，不能用transport副本替代Driver时刻和确认状态。 */
static void TestFieldMapping(void)
{
  Px4Lite_GnssDiagnostics_t out;
  unsigned before = s_driver_copies;
  assert(Px4Lite_GnssCopyDiagnostics(NULL) == PX4LITE_INVALID_PARAM && s_driver_copies == before);
  FillDistinctFacts();
  assert(Px4Lite_GnssCopyDiagnostics(&out) == PX4LITE_OK);
#define SAME_DRIVER(field) assert(out.field == s_driver.field)
  SAME_DRIVER(measured_ms); SAME_DRIVER(service_count); SAME_DRIVER(rx_bytes);
  SAME_DRIVER(valid_nmea); SAME_DRIVER(valid_ubx); SAME_DRIVER(checksum_errors);
  SAME_DRIVER(framing_resets); SAME_DRIVER(tx_commands); SAME_DRIVER(ack_count);
  SAME_DRIVER(nak_count); SAME_DRIVER(timeout_count); SAME_DRIVER(baud_changes);
  SAME_DRIVER(last_rx_ms); SAME_DRIVER(last_frame_ms); SAME_DRIVER(baud_bps);
  SAME_DRIVER(protocol_version_x100); SAME_DRIVER(rx_seen); SAME_DRIVER(frame_seen);
  SAME_DRIVER(config_step); SAME_DRIVER(config_total); SAME_DRIVER(config_confirmed); SAME_DRIVER(version_seen);
#undef SAME_DRIVER
  assert(out.protocol == PX4LITE_GNSS_PROTOCOL_UBX && out.state == PX4LITE_GNSS_LINK_UNCONFIRMED);
  assert(out.transport_baud_bps == 115200U && out.transport_rx_bytes == 31U);
  assert(out.rx_drop_bytes == 32U && out.uart_errors == 33U && out.dma_errors == 34U);
  assert(out.tx_packets == 35U && out.tx_failures == 36U && out.recoveries == 37U);
  assert(out.ndtr == 38U && out.queued_bytes == 39U && out.initialized && out.tx_busy);
  assert(out.available && out.transport_available && !out.config_confirmed);
  assert(s_driver_copies == before + 1U && s_transport_copies == before + 1U);
}

/** @brief 首次Service前仍可独立观察传输；反之BSP不可用不能丢弃Driver诊断。 */
static void TestIndependentReadiness(void)
{
  Px4Lite_GnssDiagnostics_t out;
  s_transport_result = BSP_STATUS_ERROR;
  assert(Px4Lite_GnssCopyDiagnostics(&out) == PX4LITE_OK);
  assert(out.available && !out.transport_available && out.rx_bytes == 3U && out.measured_ms == 1U);
  assert(out.transport_rx_bytes == 0U && out.transport_baud_bps == 0U);
  s_transport_result = BSP_STATUS_OK; s_driver_result = GNSS_RESULT_NO_DATA;
  assert(Px4Lite_GnssCopyDiagnostics(&out) == PX4LITE_NOT_READY);
  assert(!out.available && out.transport_available && out.transport_rx_bytes == 31U);
  assert(out.rx_bytes == 0U && out.service_count == 0U && out.config_confirmed == 0U);
  s_driver_result = GNSS_RESULT_IO_ERROR;
  assert(Px4Lite_GnssCopyDiagnostics(&out) == PX4LITE_IO_ERROR && !out.available);
  s_driver_result = GNSS_RESULT_OK;
  s_driver.measured_ms = s_driver.service_count = 0U;
  assert(Px4Lite_GnssCopyDiagnostics(&out) == PX4LITE_OK && out.available && out.measured_ms == 0U);
}

/** @brief 所有合同状态逐项映射；未知状态/协议不得冒充成功或探测阶段。 */
static void TestEnums(void)
{
  Px4Lite_GnssDiagnostics_t out;
  uint8_t value;
  for (value = 0U; value < 6U; ++value) {
    s_driver.state = value;
    assert(Px4Lite_GnssCopyDiagnostics(&out) == PX4LITE_OK && out.state == value);
  }
  for (value = 0U; value < 3U; ++value) {
    s_driver.protocol = value;
    assert(Px4Lite_GnssCopyDiagnostics(&out) == PX4LITE_OK && out.protocol == value);
  }
  s_driver.state = s_driver.protocol = 255U;
  assert(Px4Lite_GnssCopyDiagnostics(&out) == PX4LITE_OK);
  assert(out.state == PX4LITE_GNSS_LINK_UNKNOWN && out.protocol == PX4LITE_GNSS_PROTOCOL_UNKNOWN);
}
#endif

/** @brief 同一用例可额外用GNSS=0编译，验证禁用时不访问任一底层接口。 */
int main(void)
{
#if PX4LITE_ENABLE_GNSS
  TestFieldMapping(); TestIndependentReadiness(); TestEnums();
#else
  Px4Lite_GnssDiagnostics_t out;
  memset(&out, 0xA5, sizeof(out));
  assert(Px4Lite_GnssCopyDiagnostics(&out) == PX4LITE_NOT_READY);
  assert(!out.available && !out.transport_available && out.rx_bytes == 0U);
  assert(s_driver_copies == 0U && s_transport_copies == 0U);
#endif
  printf("GNSS diagnostics mapping: PASS enabled=%u (read-only/independent sides/zero epoch/enums)\n", (unsigned)PX4LITE_ENABLE_GNSS);
  return 0;
}
