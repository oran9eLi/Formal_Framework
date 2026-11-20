/**
 * @file test_gnss_recovery_liveness.c
 * @brief 真实BSP、Driver、Adapter及Framework恢复链的GNSS在线事实集成回归。
 * @details 仅替换HAL寄存器/平台时间及topic出口；不以伪定位快照代替串口接收。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "px4lite_config.h"
#undef PX4LITE_ENABLE_IMU
#undef PX4LITE_ENABLE_BARO
#undef PX4LITE_ENABLE_BATTERY
#undef PX4LITE_ENABLE_LORA
#undef PX4LITE_ENABLE_5G
#undef PX4LITE_ENABLE_DISPLAY
#undef PX4LITE_ENABLE_REMOTE_ID
#undef PX4LITE_ENABLE_CONTROL
#define PX4LITE_ENABLE_IMU 0U
#define PX4LITE_ENABLE_BARO 0U
#define PX4LITE_ENABLE_BATTERY 0U
#define PX4LITE_ENABLE_LORA 0U
#define PX4LITE_ENABLE_5G 0U
#define PX4LITE_ENABLE_DISPLAY 0U
#define PX4LITE_ENABLE_REMOTE_ID 0U
#define PX4LITE_ENABLE_CONTROL 0U

#define main ExistingHardwareReconnectMain
#include "test_gnss_hardware_reconnect.c"
#undef main

/** @brief 防止被测函数的同名局部变量遮蔽寄存器模型的PRIMASK。 */
static uint32_t RecoveryGetPrimask(void) { return primask; }
static void RecoveryDisableIrq(void) { primask = 1U; }
#undef __get_PRIMASK
#undef __disable_irq
#define __get_PRIMASK() RecoveryGetPrimask()
#define __disable_irq() RecoveryDisableIrq()

typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { uint32_t CNT, SR; } RecoveryHostTimer;
static RecoveryHostTimer s_test_timer;
static uint32_t s_test_clock;
#define TIM6 (&s_test_timer)
#define TIM_SR_UIF 1U
#define BSP_ENABLE_RPI_UART 1U
#define UART_WORDLENGTH_8B 0U
#define UART_WORDLENGTH_9B 0x1000U
#define UART_PARITY_NONE 0U
#define UART_PARITY_EVEN 0x0400U
#define UART_PARITY_ODD 0x0600U
#define UART_STOPBITS_2 0x2000U
#define UART_STOPBITS_1 0U
#define UART_HWCONTROL_NONE 0U
#define UART_HWCONTROL_RTS 0x0100U
#define UART_HWCONTROL_CTS 0x0200U
#define UART_HWCONTROL_RTS_CTS 0x0300U
static uint32_t HAL_GetTick(void) { return s_test_clock; }
static uint32_t HAL_GetUIDw0(void) { return 0U; }
static uint32_t HAL_GetUIDw1(void) { return 0U; }
static uint32_t HAL_GetUIDw2(void) { return 0U; }

#include "../../Framework/Src/px4lite_platform_f407.c"
/* 只在状态复制返回后注入一次真实owner接收，模拟Health被sensor抢占。 */
#define Px4Lite_GetModuleStatus RecoveryRealGetModuleStatus
#include "../../Framework/Src/px4lite_modules.c"
#undef Px4Lite_GetModuleStatus
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *out);
#include "../../Framework/Src/px4lite_registry.c"
#include "../../Framework/Src/px4lite_recovery.c"
#include "../../Business/Src/app_data_api.c"

static Px4Lite_SensorGnss_t s_test_topic;
static Px4Lite_SystemHealth_t s_test_health;
static uint32_t s_test_publishes;
static uint8_t s_test_inject_after_status;
static uint32_t s_test_status_injections;
static uint8_t s_test_cues;

/** @brief 独立平台毫秒时钟，允许零点和uint32回绕。 */
uint32_t BSP_Time_GetTickMs(void) { return s_test_clock; }
/** @brief 保存真实采集链发布的数据，不修改测量、序号或接收时间。 */
void Px4Lite_PublishGnss(const Px4Lite_SensorGnss_t *value)
{ s_test_topic = *value; ++s_test_publishes; }
/** @brief 应用消费者读取同一已发布topic副本，不自行补有效位。 */
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *value)
{ if (!s_test_publishes) { return PX4LITE_NOT_READY; } *value = s_test_topic; return PX4LITE_OK; }
/** @brief 仅保存真实Health结果；告警、时间和声光出口不产生硬件副作用。 */
void Px4Lite_PublishHealth(const Px4Lite_SystemHealth_t *value) { s_test_health = *value; }
void Px4Lite_RuntimeReportSensorWork(uint32_t elapsed_us) { (void)elapsed_us; }
void Px4Lite_TimeRun(uint32_t now_ms) { (void)now_ms; }
void Px4Lite_GetImuStats(Px4Lite_FifoStats_t *out) { memset(out, 0, sizeof(*out)); }
void Px4Lite_AlarmUpdateFromStatuses(const Px4Lite_ModuleStatus_t *s, uint16_t n, uint32_t now)
{ assert(s && n == PX4LITE_MODULE_COUNT); (void)now; }
void Px4Lite_StartupToneNotifyModuleConnection(uint8_t connected) { (void)connected; }

/** @brief 验证真实 owner 停收；独立 topic/通信专项验证这三个出口的内容。 */
void Px4Lite_InvalidateSensorTopics(Px4Lite_ModuleId_t id)
{ assert(id == PX4LITE_MODULE_GNSS); memset(&s_test_topic, 0, sizeof(s_test_topic)); }
void Px4Lite_GnssRawUplinkDiscard(void) { }
void Px4Lite_GnssUplinkDiscard(uint32_t now_ms) { assert(now_ms == s_test_clock); }
uint8_t Px4Lite_ControlModulePauseAllowed(Px4Lite_ModuleId_t id) { (void)id; assert(0); return 0U; }
void Px4Lite_StartupToneNotifyRpiResult(uint8_t success) { assert(success); s_test_cues++; }

/** @brief 模拟整板冷启动，仅在每场景起点清全局事实；场景内恢复全部走生产入口。 */
static void RecoveryBoard(void)
{
  Px4Lite_ModuleDescriptor_t descriptor;
  NewBoard(); s_test_clock = 0U; s_test_timer.CNT = s_test_timer.SR = 0U;
  memset(&s_test_topic, 0, sizeof(s_test_topic)); memset(&s_test_health, 0, sizeof(s_test_health));
  s_test_publishes = 0U;
  s_test_inject_after_status = 0U; s_test_status_injections = 0U;
  s_gnss_last_rx_sequence = 0U; s_gnss_reinit_pending = 0U;
  assert(Px4Lite_ModulesInit() == PX4LITE_OK);
  assert(Px4Lite_RegistryInit() == PX4LITE_OK);
  memset(s_last_attempt_ms, 0, sizeof(s_last_attempt_ms)); memset(s_attempt_count, 0, sizeof(s_attempt_count));
  memset(&descriptor, 0, sizeof(descriptor));
  descriptor.module_id = PX4LITE_MODULE_GNSS; descriptor.enabled = 1U;
  descriptor.name = "gnss"; descriptor.recover = Px4Lite_GnssRecover;
  assert(Px4Lite_RegistryRegister(&descriptor) == PX4LITE_OK);
  assert(Px4Lite_GnssModuleInit() == PX4LITE_OK);
}

/** @brief 只推进真实owner，UART完成仍由生产IRQ逻辑确认。 */
static void RecoverySensor(uint32_t now_ms)
{ s_test_clock = now_ms; Px4Lite_SensorWorkRun(now_ms); CompleteTx(); }
/** @brief 保留真实状态复制；仅在复制已离线状态后模拟一次高优先级sensor收新帧。 */
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *out)
{
  Px4Lite_Result_t result = RecoveryRealGetModuleStatus(id, out);
  if (id == PX4LITE_MODULE_GNSS && result == PX4LITE_OK && s_test_inject_after_status) {
    assert(out->state == PX4LITE_STATE_OFFLINE);
    s_test_inject_after_status = 0U; ++s_test_status_injections;
    Wire(s_good_gga, sizeof(s_good_gga)-1U); RecoverySensor(s_test_clock);
  }
  return result;
}
/** @brief 真实Health含真实通用恢复监视器，不能把回调替换成空函数。 */
static void RecoveryHealth(uint32_t now_ms)
{ s_test_clock = now_ms; Px4Lite_HealthRun(now_ms); }
/** @brief 标准完整UBX线上输入，包括独立CRC。 */
static void RecoveryUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t size, uint32_t now)
{
  uint8_t frame[128]; uint16_t length;
  length = Ubx_BuildFrame(cls, id, payload, size, frame, sizeof(frame)); assert(length);
  Wire(frame, length); RecoverySensor(now);
}

/** @brief 监视器已复制旧OFFLINE后，新帧恢复必须使旧recover回调失效。 */
static void TestStaleRecoveryCallback(void)
{
  RecoveryBoard(); Wire(s_good_gga, sizeof(s_good_gga)-1U); RecoverySensor(100U); RecoveryHealth(100U);
  RecoveryHealth(2100U); RecoveryHealth(2201U);
  assert(s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_OFFLINE && !s_gnss_reinit_pending);
  s_test_inject_after_status = 1U; RecoveryHealth(4201U);
  assert(s_test_status_injections == 1U && !s_gnss_reinit_pending);
  assert(s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_ONLINE);
  assert(s_test_publishes == 2U && s_test_topic.last_rx_ms == 4201U && Info().recoveries == 0U);
}

/** @brief 已排恢复请求先解析新完整帧；owner不能在同拍后清掉刚收到的数据。 */
static void TestPendingRecoveryWithCompleteFrame(void)
{
  uint32_t sequence;
  RecoveryBoard(); Wire(s_good_gga, sizeof(s_good_gga)-1U); RecoverySensor(100U); RecoveryHealth(100U);
  sequence = Snapshot().rx_sequence;
  RecoveryHealth(2201U); assert(s_gnss_reinit_pending);
  Wire(s_good_gga, sizeof(s_good_gga)-1U); RecoverySensor(2210U); RecoveryHealth(2210U);
  assert(Snapshot().rx_sequence > sequence && s_test_topic.last_rx_ms == 2210U);
  assert(s_test_publishes == 2U && s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_ONLINE);
  assert(Info().recoveries == 0U && !s_gnss_reinit_pending);
  assert(Px4Lite_GnssRecover() == PX4LITE_IDLE && !s_gnss_reinit_pending);
  RecoverySensor(2220U); assert(s_test_publishes == 2U);
}

/** @brief 恢复首帧跨两拍时保留半帧；期间不把旧快照重发成新在线。 */
static void TestPendingRecoveryWithHalfFrame(void)
{
  const uint16_t length = sizeof(s_good_gga)-1U;
  App_GnssDetails_t details;
  RecoveryBoard(); Wire(s_good_gga, length); RecoverySensor(100U); RecoveryHealth(100U);
  Wire(s_good_gga, length/2U); RecoverySensor(2200U); RecoveryHealth(2201U);
  assert(s_line_pos > 0U && s_gnss_reinit_pending);
  RecoverySensor(2210U);
  assert(s_test_publishes == 1U && s_line_pos > 0U && s_test_topic.last_rx_ms == 100U);
  assert(s_status[PX4LITE_MODULE_GNSS].state == PX4LITE_STATE_OFFLINE);
  assert(App_CopyGnssDetails(&details, 2210U) == PX4LITE_STALE && details.valid_mask == 0U);
  Wire(s_good_gga+length/2U, (uint16_t)(length-length/2U)); RecoverySensor(2220U); RecoveryHealth(2220U);
  assert(s_test_publishes == 2U && Snapshot().rx_sequence == 2U && s_test_topic.last_rx_ms == 2220U);
  assert(s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_ONLINE && Info().recoveries == 0U);
}

/** @brief 确认静默后真实warm复位仍保持流水号单调，Adapter无需清序号而漏掉首帧。 */
static void TestActualWarmResetSequence(void)
{
  Gnss_Snapshot_t snapshot;
  RecoveryBoard(); Wire(s_good_gga, sizeof(s_good_gga)-1U); RecoverySensor(100U); RecoveryHealth(100U);
  RecoveryHealth(2201U); assert(s_gnss_reinit_pending);
  RecoverySensor(2210U);
  assert(Sensor_GNSS_CopySnapshot(&snapshot) == GNSS_RESULT_NO_DATA && s_test_publishes == 1U);
  Wire(s_good_gga, sizeof(s_good_gga)-1U); RecoverySensor(2220U); RecoveryHealth(2220U);
  assert(Snapshot().rx_sequence == 2U && s_test_publishes == 2U && s_test_topic.last_rx_ms == 2220U);
  assert(Info().recoveries == 0U && s_diag_work.valid_nmea == 2U);
}

/** @brief CRC正确而不支持的UBX证明链路活跃，但不能生成位置、时间或定位授权。 */
static void TestUnknownUbxLiveness(void)
{
  uint32_t now_ms; uint8_t other[28] = {0U}; App_GnssDetails_t details;
  RecoveryBoard();
  for (now_ms=100U; now_ms<=9100U; now_ms+=1000U) {
    RecoveryUbx(1U, 2U, other, sizeof(other), now_ms); RecoveryHealth(now_ms);
    assert(s_test_topic.last_rx_ms == now_ms && s_test_topic.valid_mask == 0U && s_test_topic.fix_type == 0U);
    assert(s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_DEGRADED);
    assert(App_CopyGnssDetails(&details, now_ms) == PX4LITE_OK && details.valid_mask == 0U);
  }
  assert(s_test_publishes == 10U && Info().recoveries == 0U && !s_gnss_reinit_pending);
}

/** @brief 坏CRC原始输入不能续有效在线；反复自动请求也不能重发旧快照使状态假恢复。 */
static void TestBadCrcRemainsOffline(void)
{
  uint32_t now_ms; uint8_t other[28] = {0U}, frame[36]; uint16_t length;
  RecoveryBoard(); Wire(s_good_gga, sizeof(s_good_gga)-1U); RecoverySensor(100U); RecoveryHealth(100U);
  length = Ubx_BuildFrame(1U,2U,other,sizeof(other),frame,sizeof(frame)); assert(length == sizeof(frame));
  frame[length-1U] ^= 1U;
  for (now_ms=1100U; now_ms<=7100U; now_ms+=1000U) {
    Wire(frame, length); RecoverySensor(now_ms);
    assert(s_test_publishes == 1U && s_test_topic.last_rx_ms == 100U);
    RecoveryHealth(now_ms);
    if (now_ms > 2100U) { assert(s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_OFFLINE); }
  }
  assert(s_diag_work.checksum_errors == 7U && s_diag_work.valid_ubx == 0U && Info().recoveries == 0U);
}

/** @brief Health旧调用时刻遇到未来1ms接收时间仍不误离线，完整2秒门限保持。 */
static void TestFutureTimestamp(void)
{
  RecoveryBoard(); Wire(s_good_gga, sizeof(s_good_gga)-1U); RecoverySensor(1001U);
  RecoveryHealth(1000U); assert(s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_ONLINE);
  RecoveryHealth(3001U); assert(s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_ONLINE);
  RecoveryHealth(3002U); assert(s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_OFFLINE);
}

/** @brief 收到暂停后停止 GNSS IRQ/DMA，恢复后旧原始和半帧不可重发，首个新帧仍可定位。 */
static void TestIntentionalPauseAndResume(void)
{
  Px4Lite_ModuleExperimentDisplay_t display;
  Gnss_Snapshot_t snapshot;
  Gnss_RawFrameInfo_t frame_info;
  uint8_t raw[128];
  const uint16_t length = sizeof(s_good_gga) - 1U;
  RecoveryBoard(); s_test_cues = 0U;
  Wire(s_good_gga, length); RecoverySensor(100U);
#if DEBUG_GNSS_MONITOR_ENABLE
  assert(Sensor_GNSS_CopyRawFrame(2U, raw, sizeof(raw), &frame_info) == GNSS_RESULT_OK);
#else
  assert(Sensor_GNSS_CopyRawFrame(2U, raw, sizeof(raw), &frame_info) == GNSS_RESULT_NO_DATA);
#endif
  Wire(s_good_gga, length / 2U); RecoverySensor(200U);

  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_GNSS, 0U, 200U) == PX4LITE_OK);
  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_GNSS, 1U, 200U) == PX4LITE_BUSY);
  assert(Info().initialized && !s_test_cues);
  RecoverySensor(210U); Px4Lite_ModuleExperimentAck(PX4LITE_MODULE_GNSS, 0U);
  RecoveryHealth(12200U);
  assert(!Info().initialized && !irq_enabled[BSP_GNSS_IRQn] && !irq_enabled[BSP_GNSS_RX_DMA_IRQn]);
  assert(Sensor_GNSS_CopySnapshot(&snapshot) == GNSS_RESULT_NO_DATA);
  assert(Sensor_GNSS_CopyRawFrame(2U, raw, sizeof(raw), &frame_info) == GNSS_RESULT_NO_DATA);
  assert(s_test_publishes == 1U && s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_DISABLED);

  assert(Px4Lite_ModuleExperimentRequest(PX4LITE_MODULE_GNSS, 1U, 12200U) == PX4LITE_OK);
  RecoveryHealth(12201U); Px4Lite_ModuleExperimentCopyDisplay(&display);
  assert(s_test_health.module_state[PX4LITE_MODULE_GNSS] == PX4LITE_STATE_STARTING);
  assert(display.page_sequence == 0U && s_test_cues == 1U);
  RecoverySensor(12210U); Px4Lite_ModuleExperimentAck(PX4LITE_MODULE_GNSS, 1U);
  assert(Info().initialized && s_test_publishes == 1U);
  Wire(s_good_gga + length / 2U, (uint16_t)(length - length / 2U)); RecoverySensor(12220U);
  assert(s_test_publishes == 1U);
  Wire(s_good_gga, length); RecoverySensor(12230U);
  assert(s_test_publishes == 2U && Snapshot().rx_sequence == 2U);
  Px4Lite_ModuleExperimentCopyDisplay(&display);
  assert(display.page_module == PX4LITE_MODULE_GNSS && display.page_sequence == 1U && s_test_cues == 2U);
}

int main(void)
{
  TestStaleRecoveryCallback(); TestPendingRecoveryWithCompleteFrame(); TestPendingRecoveryWithHalfFrame();
  TestActualWarmResetSequence(); TestUnknownUbxLiveness(); TestBadCrcRemainsOffline(); TestFutureTimestamp();
  TestIntentionalPauseAndResume();
  assert(primask == 0U);
  puts("GNSS recovery liveness: real BSP+Driver+Adapter+Health+Registry stale-request gate / half-frame / no stale republish / monotonic warm sequence / unknown UBX / bad CRC / future clock / intentional pause+resume PASS");
  return 0;
}
