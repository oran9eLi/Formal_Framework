/**
 * @file test_gnss_protocol_capture.c
 * @brief 真实Framework/App捕获回归：双协议、筛选、常驻发现、稳定副本及MAV共享区隔离。
 * @details 单线程短临界区替身可在提交前注入换会话；不启动任务、不读UART或Sensor队列。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
static void TestEnter(void);
static void TestExit(void);
#define taskENTER_CRITICAL() TestEnter()
#define taskEXIT_CRITICAL() TestExit()
#include "../../Framework/Src/px4lite_diagnostic_capture.c"
#include "../../Business/Src/app_data_api.c"

static void (*s_before_enter)(void);
static unsigned s_depth;
static uint32_t s_now;

/** @brief 在临界区真正进入前注入一次动作，验证生产代码提交时重新检查会话。 */
static void TestEnter(void)
{
  if (s_before_enter != NULL) { void (*hook)(void) = s_before_enter; s_before_enter = NULL; hook(); }
  s_depth++;
}
static void TestExit(void) { assert(s_depth != 0U); s_depth--; }
uint32_t Px4Lite_PlatformGetMs(void) { return s_now; }
void Px4Lite_RpiH1EchoGetStats(Px4Lite_RpiH1EchoStats_t *out) { memset(out, 0, sizeof(*out)); }
void Px4Lite_RpiBaudSwitchGetStats(Px4Lite_RpiBaudSwitchStats_t *out) { memset(out, 0, sizeof(*out)); }
Px4Lite_Result_t Px4Lite_RpiMavlinkCopyMonitor(Px4Lite_SerialMonitor_t *out)
{ memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }

/** @brief 经真实App请求槽与Comm动作处理切换，不直接篡改会话字段。 */
static Px4Lite_CaptureStatus_t Action(uint8_t operation, uint8_t source, uint8_t filter)
{
  App_CaptureRequest_t request;
  Px4Lite_CaptureStatus_t status;
  memset(&request, 0, sizeof(request)); request.operation = operation; request.source = source; request.filter = filter;
  assert(App_RequestCapture(&request) == PX4LITE_OK);
  Px4Lite_CaptureWorkRun(); assert(App_CopyCaptureStatus(&status) == PX4LITE_OK);
  assert(!status.pending && status.completed_request_id == status.request_id && status.result == PX4LITE_OK);
  return status;
}

static Px4Lite_CaptureStatus_t Status(void)
{ Px4Lite_CaptureStatus_t status; assert(App_CopyCaptureStatus(&status) == PX4LITE_OK); return status; }

/** @brief 构造有界完整UBX，坏校验时只改变实际线上CK_B，保持字节保真。 */
static uint16_t Ubx(uint8_t *out, uint8_t cls, uint8_t id, uint16_t payload, uint8_t bad)
{
  uint16_t i; uint8_t a = 0U, b = 0U;
  /* 测试向量也覆盖1024B接收与392B屏幕存储分离，调用方负责提供相应数组。 */
  assert(payload + 8U <= 1024U);
  out[0] = 0xB5U; out[1] = 0x62U; out[2] = cls; out[3] = id; out[4] = (uint8_t)payload; out[5] = (uint8_t)(payload >> 8);
  for (i = 0U; i < payload; ++i) { out[i + 6U] = (uint8_t)(i * 7U + id); }
  for (i = 2U; i < payload + 6U; ++i) { a = (uint8_t)(a + out[i]); b = (uint8_t)(b + a); }
  out[payload + 6U] = a; out[payload + 7U] = bad ? (uint8_t)(b ^ 1U) : b;
  return (uint16_t)(payload + 8U);
}

/** @brief 构造完整NMEA，具体测量含义不在捕获层解析。 */
static uint16_t Nmea(uint8_t *out, uint8_t type)
{
  static const char * const bodies[5] = {"GNTXT,01,01,02,hello", "GPGGA,1", "GNRMC,1", "GPGSA,A,1", "GPGSV,1,1,00"};
  uint8_t sum = 0U; const char *cursor = bodies[type];
  while (*cursor) { sum ^= (uint8_t)*cursor++; }
  return (uint16_t)sprintf((char *)out, "$%s*%02X\r\n", bodies[type], (unsigned)sum);
}

/** @brief 生产MAV入口只检查完整边界，CRC结论由原解析器提供；本夹具保留任意原CRC字节。 */
static uint16_t Mav(uint8_t *out, uint8_t sequence)
{
  memset(out, 0, 16U); out[0] = 0xFDU; out[1] = 4U; out[4] = sequence;
  out[5] = 42U; out[6] = 193U; out[7] = 24U; out[10] = 0x55U; out[14] = 0xABU; out[15] = 0xCDU;
  return 16U;
}

/** @brief 协议发现独立于入口/暂停/筛选，坏CRC和残帧不续期，0ms及回绕合法。 */
static void TestDiscovery(void)
{
  uint8_t wire[392]; uint16_t length;
  Px4Lite_CaptureStatus_t status;
  Px4Lite_CaptureInit(); assert(PX4LITE_CAPTURE_NMEA == 4U && PX4LITE_CAPTURE_GNSS == 4U);
  length = Ubx(wire, 1U, 7U, 4U, 1U); Px4Lite_CaptureUbx(wire, length, 1U, 7U, 0U, 99U);
  assert(Status().gnss_seen_mask == 0U);
  length = Nmea(wire, 1U); Px4Lite_CaptureNmea(wire, length, 1U, 1U, 0U);
  status = Status(); assert(status.gnss_seen_mask == 1U && status.last_nmea_ms == 0U && status.count == 0U);
  Action(PX4LITE_CAPTURE_SELECT, PX4LITE_CAPTURE_GNSS, PX4LITE_CAPTURE_FILTER_GGA);
  length = Ubx(wire, 1U, 4U, 4U, 0U); Px4Lite_CaptureUbx(wire, length, 1U, 4U, 1U, UINT32_MAX - 1U);
  status = Status(); assert(status.gnss_seen_mask == 3U && status.last_ubx_ms == UINT32_MAX - 1U && status.count == 0U);
  Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_GNSS, PX4LITE_CAPTURE_FILTER_GGA);
  Px4Lite_CaptureUbx(wire, length, 1U, 4U, 1U, 0U);
  assert(Status().last_ubx_ms == 0U && Status().count == 0U);
  length = Nmea(wire, 1U); Px4Lite_CaptureNmea(wire, length, 1U, 0U, 5U);
  status = Status(); assert(status.count == 1U && status.last_nmea_ms == 0U);
  Action(PX4LITE_CAPTURE_PAUSE, PX4LITE_CAPTURE_GNSS, PX4LITE_CAPTURE_FILTER_GGA);
  Px4Lite_CaptureNmea(wire, length, 1U, 1U, 6U);
  status = Status(); assert(status.count == 1U && status.last_nmea_ms == 6U);
  Action(PX4LITE_CAPTURE_CLEAR, PX4LITE_CAPTURE_GNSS, PX4LITE_CAPTURE_FILTER_GGA);
  status = Status(); assert(status.count == 0U && status.gnss_seen_mask == 3U && status.last_nmea_ms == 6U);
}

/** @brief NMEA/UBX混合保留真实字节、BAD标志和不同汇总键，旧包装只能取NMEA。 */
static void TestMixedAndLegacy(void)
{
  uint8_t wire[392], expected[392]; uint16_t length;
  App_GnssCapturedFrame_t frame, before;
  App_NmeaSentence_t old, old_before;
  App_CaptureSummary_t summary;
  Px4Lite_CaptureStatus_t status;
  Px4Lite_CaptureInit(); status = Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_GNSS, 0U);
  length = Nmea(wire, 4U); memcpy(expected, wire, length); Px4Lite_CaptureNmea(wire, length, 4U, 1U, 10U);
  assert(App_CopyCapturedGnssFrame(status.session_id, 1U, &frame) == PX4LITE_OK);
  assert(frame.protocol == 1U && frame.nmea_type == 4U && frame.class_id == 0U && frame.checksum_valid);
  assert(frame.length == length && !memcmp(frame.bytes, expected, length));
  assert(App_CopyNmeaSentence(status.session_id, 1U, &old) == PX4LITE_OK && old.type == 4U);
  assert(App_CopyCaptureSummaryRow(status.session_id, 0U, &summary) == PX4LITE_OK);
  assert(summary.system_id == 1U && summary.component_id == 0U && summary.msg_id == 4U);
  length = Ubx(wire, 1U, 4U, 384U, 1U); memcpy(expected, wire, length);
  Px4Lite_CaptureUbx(wire, length, 1U, 4U, 0U, 20U); memset(wire, 0, length);
  assert(App_CopyCapturedGnssFrame(status.session_id, 2U, &frame) == PX4LITE_OK);
  assert(frame.length == 392U && frame.protocol == 2U && frame.class_id == 1U && frame.message_id == 4U && !frame.checksum_valid);
  assert(!memcmp(frame.bytes, expected, 392U));
  memset(&old, 0xA5, sizeof(old)); old_before = old;
  assert(App_CopyNmeaSentence(status.session_id, 2U, &old) == PX4LITE_NOT_READY && !memcmp(&old, &old_before, sizeof(old)));
  assert(App_CopyCaptureSummaryRow(status.session_id, 1U, &summary) == PX4LITE_OK);
  assert(summary.system_id == 2U && summary.component_id == 1U && summary.msg_id == 4U && summary.count == 1U);
  assert(Status().count == 2U && Status().gnss_seen_mask == 1U);
  memset(&frame, 0x5A, sizeof(frame)); before = frame;
  assert(App_CopyCapturedGnssFrame(status.session_id, 99U, &frame) == PX4LITE_NOT_READY && !memcmp(&frame, &before, sizeof(frame)));
  assert(App_CopyCapturedGnssFrame(status.session_id, 1U, NULL) == PX4LITE_INVALID_PARAM);
}

/** @brief 验证全部10个filter，发现事实不被筛选遮蔽。 */
static void TestFilters(void)
{
  static const unsigned expected[] = {9U,1U,1U,1U,1U,5U,4U,1U,1U,1U};
  static const uint8_t classes[] = {1U,1U,1U,0x0AU};
  static const uint8_t ids[] = {7U,4U,0x35U,4U};
  uint8_t wire[392], filter, type; unsigned i; uint16_t length;
  App_CaptureRequest_t bad = {PX4LITE_CAPTURE_START,PX4LITE_CAPTURE_GNSS,10U,0U};
  assert(App_RequestCapture(&bad) == PX4LITE_INVALID_PARAM);
  for (filter = 0U; filter <= 9U; ++filter) {
    Px4Lite_CaptureInit(); Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_GNSS, filter);
    for (type = 0U; type <= 4U; ++type) { length = Nmea(wire, type); Px4Lite_CaptureNmea(wire, length, type, 1U, type); }
    for (i = 0U; i < 4U; ++i) { length = Ubx(wire, classes[i], ids[i], 2U, 0U); Px4Lite_CaptureUbx(wire, length, classes[i], ids[i], 1U, 10U + i); }
    assert(Status().count == expected[filter] && Status().gnss_seen_mask == 3U);
  }
}

static void SwitchToMav(void) { Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_LORA_RX, 0U); }

/** @brief 四槽覆盖和外部选帧稳定；提交前换会话必须拒绝迟到GNSS记录。 */
static void TestOverwriteAndSession(void)
{
  uint8_t wire[392]; uint16_t length; unsigned i;
  App_GnssCapturedFrame_t selected, saved, rejected;
  Px4Lite_CaptureStatus_t status;
  Px4Lite_CaptureInit(); status = Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_GNSS, 0U);
  length = Ubx(wire, 1U, 7U, 4U, 0U); Px4Lite_CaptureUbx(wire, length, 1U, 7U, 1U, 1U);
  assert(App_CopyCapturedGnssFrame(status.session_id, 1U, &selected) == PX4LITE_OK); saved = selected;
  for (i = 0U; i < 4U; ++i) { wire[6] = (uint8_t)i; Px4Lite_CaptureUbx(wire, length, 1U, 7U, 0U, 2U + i); }
  assert(Status().count == 5U && Status().stored_count == 4U && Status().overwritten == 1U);
  assert(App_CopyCapturedGnssFrame(status.session_id, 1U, &rejected) == PX4LITE_NOT_READY);
  assert(!memcmp(&selected, &saved, sizeof(saved)));
  Action(PX4LITE_CAPTURE_CLEAR, PX4LITE_CAPTURE_GNSS, 0U);
  assert(App_CopyCapturedGnssFrame(status.session_id, 5U, &rejected) == PX4LITE_NOT_READY);
  status = Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_GNSS, 0U);
  s_before_enter = SwitchToMav; Px4Lite_CaptureUbx(wire, length, 1U, 7U, 0U, 100U);
  assert(Status().source == PX4LITE_CAPTURE_LORA_RX && Status().count == 0U && Status().stored_count == 0U);
  assert(Status().session_id != status.session_id);
}

/** @brief 不完整帧不冒充协议发现；合法长UBX可识别协议，但不截断存进392B屏幕槽。 */
static void TestBoundaryDrops(void)
{
  uint8_t wire[1024]; uint16_t length;
  Px4Lite_CaptureInit(); Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_GNSS, 0U);
  length = Ubx(wire, 1U, 7U, 4U, 0U);
  Px4Lite_CaptureUbx(NULL, 0U, 1U, 7U, 1U, 0U);
  Px4Lite_CaptureUbx(wire, 393U, 1U, 7U, 1U, 1U);
  Px4Lite_CaptureUbx(wire, length, 1U, 4U, 1U, 2U);
  Px4Lite_CaptureNmea((const uint8_t *)"x", 1U, 1U, 1U, 3U);
  assert(Status().capture_drops == 4U && Status().count == 0U && Status().gnss_seen_mask == 0U);
  length = Ubx(wire, 1U, 0x35U, 776U, 1U);
  Px4Lite_CaptureUbx(wire, length, 1U, 0x35U, 1U, 5U);
  assert(Status().capture_drops == 5U && Status().stored_count == 0U && Status().count == 0U);
  assert(Status().gnss_seen_mask == 2U && Status().last_ubx_ms == 5U);
  Px4Lite_CaptureUbx(wire, length, 1U, 0x35U, 0U, 6U);
  assert(Status().capture_drops == 6U && Status().last_ubx_ms == 5U);
  /* MAV入口不能在source4把共享union写成另一种结构。 */
  length = Mav(wire, 1U); Px4Lite_CaptureRx(PX4LITE_CAPTURE_GNSS, wire, length, 4U);
  assert(Status().count == 0U && Status().stored_count == 0U);
}

/** @brief 合法MAV RX稳定copy、TX阶段和换会话迟到拒绝均维持原语义。 */
static void TestMavCompatibility(void)
{
  uint8_t wire[392], expected[16]; uint16_t length; uint32_t token;
  App_CaptureFrame_t frame, saved; uint8_t stage;
  Px4Lite_CaptureStatus_t status;
  Px4Lite_CaptureInit(); status = Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_LORA_RX, 0U);
  length = Mav(wire, 3U); memcpy(expected, wire, length); Px4Lite_CaptureRx(PX4LITE_CAPTURE_LORA_RX, wire, length, 1U);
  assert(App_CopyCapturedFrame(status.session_id, 1U, &frame) == PX4LITE_OK);
  assert(frame.system_id == 42U && frame.component_id == 193U && frame.msg_id == 24U && frame.sequence == 3U);
  assert(frame.stage == PX4LITE_CAPTURE_RX_VALID && !memcmp(frame.bytes, expected, 16U)); saved = frame;
  memset(wire, 0, 16U); assert(!memcmp(frame.bytes, expected, 16U));
  status = Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_LORA_TX, 0U);
  length = Mav(wire, 4U); token = Px4Lite_CaptureTxBegin(wire, length, 10U); assert(token == 1U);
  assert(Status().tx_attempts == 1U && Status().count == 0U);
  Px4Lite_CaptureTxStage(PX4LITE_CAPTURE_QUEUED); Px4Lite_CaptureTxStage(PX4LITE_CAPTURE_QUEUED);
  assert(Status().count == 1U);
  assert(App_CopyCapturedStage(status.session_id, token, &stage) == PX4LITE_OK && stage == PX4LITE_CAPTURE_QUEUED);
  Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_GNSS, 0U);
  Px4Lite_CaptureTxStage(PX4LITE_CAPTURE_QUEUED); Px4Lite_CaptureTxStage(PX4LITE_CAPTURE_MODULE_IDLE);
  assert(Status().count == 0U && Status().stored_count == 0U);
  assert(App_CopyCapturedFrame(status.session_id, token, &frame) == PX4LITE_NOT_READY);
  assert(!memcmp(&saved.bytes, expected, 16U));
  status = Action(PX4LITE_CAPTURE_START, PX4LITE_CAPTURE_RPI_TX, 0U);
  Px4Lite_CaptureRpiTx(wire, length, 20U, PX4LITE_CAPTURE_UART_DONE);
  Px4Lite_CaptureRpiTx(wire, length, 21U, PX4LITE_CAPTURE_TIMEOUT);
  assert(Status().count == 1U && Status().tx_attempts == 2U && Status().tx_timeouts == 1U);
  assert(App_CopyCapturedFrame(status.session_id, 2U, &frame) == PX4LITE_OK && frame.stage == PX4LITE_CAPTURE_TIMEOUT);
}

int main(void)
{
  TestDiscovery(); TestMixedAndLegacy(); TestFilters(); TestOverwriteAndSession(); TestBoundaryDrops(); TestMavCompatibility();
  assert(s_depth == 0U);
  printf("GNSS protocol capture: PASS (mixed/10 filters/legacy/stable/4 slots/session/MAV RX-TX), store=%uB GNSSframe=%uB\n", (unsigned)sizeof(s_store), (unsigned)sizeof(Px4Lite_GnssCapturedFrame_t));
  return 0;
}
