/**
 * @file px4lite_diagnostic_capture.c
 * @brief Framework 固定捕获环、互斥会话及通用串口监视快照发布。
 * @details 无任务/堆分配；Comm执行动作，Adapter提交副本，不读生产队列或DMA。
 */
#include "px4lite_diagnostic_capture.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "px4lite_platform.h"
static Px4Lite_SerialExperiment_t s_serial;
/** @brief Comm 发布、应用只读的通用串口事实，独立于捕获会话。 */
static Px4Lite_SerialMonitor_t s_serial_monitor;
/** @brief 固定互斥区；切类型必须换会话且暂停。 */
typedef union {
  Px4Lite_CaptureFrame_t mav[PX4LITE_CAPTURE_CAPACITY]; /**< MAVLink完整帧。 */
  Px4Lite_GnssCapturedFrame_t gnss[PX4LITE_CAPTURE_CAPACITY]; /**< NMEA/UBX完整原帧。 */
} CaptureStore_t;
static CaptureStore_t s_store;
static Px4Lite_CaptureSummary_t s_summary[PX4LITE_CAPTURE_SUMMARIES];
static Px4Lite_CaptureStatus_t s_status;
static Px4Lite_CaptureRequest_t s_request;
static uint32_t s_tx_session, s_tx_token;
static uint8_t s_head;
/** @brief 非零代数递增，处理uint32回绕。 */
static uint32_t CaptureNext(uint32_t value) { return value == UINT32_MAX ? 1U : value + 1U; }
/** @brief 调度前一次初始化。 */
void Px4Lite_CaptureInit(void)
{
  memset(&s_serial_monitor, 0, sizeof(s_serial_monitor));
  memset(&s_store, 0, sizeof(s_store)); memset(s_summary, 0, sizeof(s_summary)); memset(&s_status, 0, sizeof(s_status));
  s_status.session_id = 1U; s_status.source = PX4LITE_CAPTURE_LORA_RX; s_head = 0U; s_tx_session = 0U; s_tx_token = 0U;
}
/** @brief 固定请求槽只复制，不在调用方执行切换。 */
Px4Lite_Result_t Px4Lite_CaptureRequest(const Px4Lite_CaptureRequest_t *request)
{
  if (!request || request->operation > PX4LITE_CAPTURE_SELECT || request->source > PX4LITE_CAPTURE_GNSS || request->filter > PX4LITE_CAPTURE_FILTER_SAT) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  if (s_status.pending) { taskEXIT_CRITICAL(); return PX4LITE_BUSY; }
  s_request = *request; s_status.request_id = CaptureNext(s_status.request_id); s_status.pending = 1U;
  taskEXIT_CRITICAL(); return PX4LITE_OK;
}
/** @brief Comm处理捕获动作；换会话只清记录统计，保留端口统计和独立GNSS协议发现事实。 */
void Px4Lite_CaptureWorkRun(void)
{
  Px4Lite_SerialExperiment_t serial;
  taskENTER_CRITICAL();
  if (s_status.pending) {
    uint8_t reset = (uint8_t)(s_request.operation == PX4LITE_CAPTURE_START || s_request.operation == PX4LITE_CAPTURE_CLEAR || s_request.operation == PX4LITE_CAPTURE_SELECT || s_request.source != s_status.source);
    if (reset) {
      s_status.running = 0U; s_status.session_id = CaptureNext(s_status.session_id);
      s_status.source = s_request.source; s_status.filter = s_request.filter;
      s_status.count = 0U; s_status.overwritten = 0U; s_status.capture_drops = 0U;
      s_status.tx_attempts = 0U; s_status.tx_timeouts = 0U; s_status.tx_errors = 0U;
      s_status.summary_drops = 0U; s_status.rejected = 0U; s_status.latest_token = 0U;
      s_status.summary_count = 0U; s_status.stored_count = 0U; s_head = 0U;
    }
    if (s_request.operation == PX4LITE_CAPTURE_START) { s_status.running = 1U; }
    if (s_request.operation == PX4LITE_CAPTURE_PAUSE) { s_status.running = 0U; }
    s_status.completed_request_id = s_status.request_id; s_status.result = PX4LITE_OK; s_status.pending = 0U;
  }
  taskEXIT_CRITICAL();
  Px4Lite_RpiH1EchoGetStats(&serial.h1); Px4Lite_RpiBaudSwitchGetStats(&serial.b1);
  serial.measured_ms=Px4Lite_PlatformGetMs();
  taskENTER_CRITICAL(); s_serial=serial; taskEXIT_CRITICAL();
  {
    Px4Lite_SerialMonitor_t monitor;
    (void)Px4Lite_RpiMavlinkCopyMonitor(&monitor);
    taskENTER_CRITICAL(); s_serial_monitor = monitor; taskEXIT_CRITICAL();
  }
}
/** @brief 只复制已发布事实，不让显示任务触碰 UART 或消耗接收字节。 */
Px4Lite_Result_t Px4Lite_CopySerialMonitor(Px4Lite_SerialMonitor_t *out)
{
  if (out == 0) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL(); *out = s_serial_monitor; taskEXIT_CRITICAL();
  return out->available ? PX4LITE_OK : PX4LITE_NOT_READY;
}
/** @brief 只复制Comm已发布状态，不让消费者访问外设。 */
Px4Lite_Result_t Px4Lite_CopySerialExperiment(Px4Lite_SerialExperiment_t *out)
{ if(!out) { return PX4LITE_INVALID_PARAM; } taskENTER_CRITICAL(); *out=s_serial; taskEXIT_CRITICAL(); return PX4LITE_OK; }
/** @brief 复制摘要。 */
Px4Lite_Result_t Px4Lite_CaptureCopyStatus(Px4Lite_CaptureStatus_t *out)
{ if (!out) { return PX4LITE_INVALID_PARAM; } taskENTER_CRITICAL(); *out = s_status; taskEXIT_CRITICAL(); return PX4LITE_OK; }
/** @brief 汇总键保留失败Raw，但只有accepted更新Count及均频时间边界。 */
static void CaptureSummary(uint32_t session, uint32_t token, uint32_t msg, uint8_t sys, uint8_t comp, uint8_t source, uint32_t now, uint8_t accepted)
{
  uint8_t i;
  Px4Lite_CaptureSummary_t row;
  uint8_t count = s_status.summary_count;
  for (i = 0U; i < count; i++) { if (s_summary[i].msg_id == msg && s_summary[i].system_id == sys && s_summary[i].component_id == comp && s_summary[i].source == source) { break; } }
  if (i == count) {
    if (i >= PX4LITE_CAPTURE_SUMMARIES) {
      /* LoRa入环尝试已记一次满表；Queued晋级不重复记诊断舍弃。 */
      taskENTER_CRITICAL();
      if (s_status.session_id == session && !(source == PX4LITE_CAPTURE_LORA_TX && accepted)) { s_status.summary_drops++; }
      taskEXIT_CRITICAL(); return;
    }
    memset(&row, 0, sizeof(row)); row.msg_id = msg; row.system_id = sys; row.component_id = comp; row.source = source;
  } else { row = s_summary[i]; }
  if (accepted) {
    if (!row.count) { row.first_ms = now; }
    if (row.count != UINT32_MAX) { row.count++; }
    row.last_ms = now;
  }
  row.latest_token = token;
  taskENTER_CRITICAL();
  if (s_status.session_id == session && s_status.source == source) { s_summary[i] = row; if (i == count) { s_status.summary_count++; } }
  taskEXIT_CRITICAL();
}
/** @brief 验证MAV入口0..3和原始帧边界，防止GNSS union被另一类型写入；解析在临界区外。 */
static uint32_t CaptureFrame(uint8_t source, const uint8_t *bytes, uint16_t length, uint32_t now, uint8_t stage)
{
  Px4Lite_CaptureFrame_t frame;
  uint32_t session;
  uint16_t expected;
  uint8_t v2, accepted, tx;
  if (source > PX4LITE_CAPTURE_RPI_TX || !s_status.running || s_status.source != source) { return 0U; }
  session = s_status.session_id;
  if (!bytes || length < 8U || length > PX4LITE_CAPTURE_FRAME_BYTES || (bytes[0] != 0xFD && bytes[0] != 0xFE)) { Px4Lite_CaptureReject(source); return 0U; }
  v2 = (uint8_t)(bytes[0] == 0xFD);
  expected = (uint16_t)(bytes[1] + (v2 ? 12U : 8U) + (v2 && (bytes[2] & 1U) ? 13U : 0U));
  if (length != expected) { Px4Lite_CaptureReject(source); return 0U; }
  frame.session_id = session; frame.time_ms = now; frame.length = length; frame.source = source; frame.stage = stage;
  frame.flags = (uint8_t)(v2 && (bytes[2] & 1U));
  tx = (uint8_t)(source == PX4LITE_CAPTURE_LORA_TX || source == PX4LITE_CAPTURE_RPI_TX);
  accepted = (uint8_t)(stage == PX4LITE_CAPTURE_RX_VALID || stage == PX4LITE_CAPTURE_UART_DONE);
  if (accepted) { frame.flags |= 2U; }
  frame.msg_id = v2 ? (uint32_t)bytes[7] | ((uint32_t)bytes[8]<<8) | ((uint32_t)bytes[9]<<16) : bytes[5];
  frame.sequence = bytes[v2 ? 4U : 2U]; frame.system_id = bytes[v2 ? 5U : 3U]; frame.component_id = bytes[v2 ? 6U : 4U];
  memset(frame.bytes, 0, sizeof(frame.bytes)); memcpy(frame.bytes, bytes, length);
  taskENTER_CRITICAL();
  if (!s_status.running || s_status.source != source || s_status.session_id != session) { taskEXIT_CRITICAL(); return 0U; }
  frame.token = CaptureNext(s_status.latest_token); s_status.latest_token = frame.token;
  s_store.mav[s_head] = frame; s_head = (uint8_t)((s_head + 1U) % PX4LITE_CAPTURE_CAPACITY);
  if (accepted) { s_status.count++; }
  if (tx) { s_status.tx_attempts++; }
  if (stage == PX4LITE_CAPTURE_ERROR) { s_status.tx_errors++; }
  if (stage == PX4LITE_CAPTURE_TIMEOUT) { s_status.tx_timeouts++; }
  if (s_status.stored_count < PX4LITE_CAPTURE_CAPACITY) { s_status.stored_count++; } else { s_status.overwritten++; }
  taskEXIT_CRITICAL();
  CaptureSummary(session, frame.token, frame.msg_id, frame.system_id, frame.component_id, source, now, accepted);
  return frame.token;
}
/** @brief 生产解析器已完成CRC检查的RX副本，不消费队列。 */
void Px4Lite_CaptureRx(uint8_t source, const uint8_t *bytes, uint16_t length, uint32_t now)
{ (void)CaptureFrame(source, bytes, length, now, PX4LITE_CAPTURE_RX_VALID); }
/** @brief 选中入口拒绝计数。 */
void Px4Lite_CaptureReject(uint8_t source)
{ taskENTER_CRITICAL(); if (s_status.running && s_status.source == source) { s_status.rejected++; } taskEXIT_CRITICAL(); }
/** @brief 单在途关联在实际Send之前建立，防止立即启动或失败事件丢失。 */
uint32_t Px4Lite_CaptureTxBegin(const uint8_t *bytes, uint16_t length, uint32_t now)
{
  if (s_tx_token) { return 0U; }
  s_tx_token = CaptureFrame(PX4LITE_CAPTURE_LORA_TX, bytes, length, now, PX4LITE_CAPTURE_SUBMITTING);
  if (s_tx_token) { s_tx_session = s_status.session_id; }
  return s_tx_token;
}
/** @brief 更新在途token，仅同会话同槽仍存在时改变阶段，终态释放关联。 */
void Px4Lite_CaptureTxStage(uint8_t stage)
{
  uint8_t i, accepted = 0U;
  Px4Lite_CaptureSummary_t row;
  uint32_t session = s_tx_session, token = s_tx_token;
  if (!s_tx_token) { return; }
  taskENTER_CRITICAL();
  if (s_status.session_id == s_tx_session && s_status.source == PX4LITE_CAPTURE_LORA_TX) {
    for (i = 0U; i < s_status.stored_count; i++) {
      Px4Lite_CaptureFrame_t *frame = &s_store.mav[i];
      if (frame->token != s_tx_token || frame->session_id != s_tx_session) { continue; }
      frame->stage = stage;
      if (stage == PX4LITE_CAPTURE_QUEUED && !(frame->flags & 2U)) {
        frame->flags |= 2U; s_status.count++; accepted = 1U;
        row.msg_id = frame->msg_id; row.system_id = frame->system_id;
        row.component_id = frame->component_id; row.last_ms = frame->time_ms;
      }
      break;
    }
    if (stage == PX4LITE_CAPTURE_ERROR) { s_status.tx_errors++; }
    if (stage == PX4LITE_CAPTURE_TIMEOUT) { s_status.tx_timeouts++; }
  }
  if (stage == PX4LITE_CAPTURE_MODULE_IDLE || stage == PX4LITE_CAPTURE_ERROR || stage == PX4LITE_CAPTURE_TIMEOUT) { s_tx_token = 0U; s_tx_session = 0U; }
  taskEXIT_CRITICAL();
  if (accepted) { CaptureSummary(session, token, row.msg_id, row.system_id, row.component_id, PX4LITE_CAPTURE_LORA_TX, row.last_ms, 1U); }
}
/** @brief 不用OK覆盖即时失败终态；Queued来自Driver真正锁存事实。 */
void Px4Lite_CaptureTxSubmit(uint32_t token, Px4Lite_Result_t result)
{ if (token && token == s_tx_token && result != PX4LITE_OK) { Px4Lite_CaptureTxStage(PX4LITE_CAPTURE_ERROR); } }
/** @brief RPi捕获在最终重标编码后；成功仅为本机UART完整发送。 */
void Px4Lite_CaptureRpiTx(const uint8_t *bytes, uint16_t length, uint32_t now, uint8_t stage)
{ (void)CaptureFrame(PX4LITE_CAPTURE_RPI_TX, bytes, length, now, stage); }
/** @brief 复制当前会话汇总行。 */
Px4Lite_Result_t Px4Lite_CaptureCopySummary(uint32_t session, uint8_t index, Px4Lite_CaptureSummary_t *out)
{
  if (!out) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  if (session != s_status.session_id || index >= s_status.summary_count) { taskEXIT_CRITICAL(); return PX4LITE_NOT_READY; }
  *out = s_summary[index]; taskEXIT_CRITICAL(); return PX4LITE_OK;
}
/** @brief 稳定复制完整帧，覆盖/切类型后不返回伪造记录。 */
Px4Lite_Result_t Px4Lite_CaptureCopyFrame(uint32_t session, uint32_t token, Px4Lite_CaptureFrame_t *out)
{
  uint8_t i;
  if (!out) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  if (session == s_status.session_id && s_status.source != PX4LITE_CAPTURE_NMEA) {
    for (i = 0U; i < s_status.stored_count; i++) { if (s_store.mav[i].token == token && s_store.mav[i].session_id == session) { *out = s_store.mav[i]; taskEXIT_CRITICAL(); return PX4LITE_OK; } }
  }
  taskEXIT_CRITICAL(); return PX4LITE_NOT_READY;
}
/** @brief 只复制阶段；旧会话或已覆盖槽不提供迟到事实。 */
Px4Lite_Result_t Px4Lite_CaptureCopyStage(uint32_t session, uint32_t token, uint8_t *stage)
{
  uint8_t i;
  if (!stage) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  if (session == s_status.session_id && s_status.source != PX4LITE_CAPTURE_NMEA) {
    for (i=0U; i<s_status.stored_count; i++) { if (s_store.mav[i].session_id==session && s_store.mav[i].token==token) { *stage=s_store.mav[i].stage; taskEXIT_CRITICAL(); return PX4LITE_OK; } }
  }
  taskEXIT_CRITICAL(); return PX4LITE_NOT_READY;
}
/** @brief 仅按Driver元数据筛选原帧，不解析导航字段或重新校验协议。 */
static uint8_t CaptureGnssMatches(uint8_t filter, uint8_t protocol, uint8_t type, uint8_t cls, uint8_t id)
{
  if (filter == PX4LITE_CAPTURE_FILTER_ALL) { return 1U; }
  if (filter <= PX4LITE_CAPTURE_FILTER_GSV) { return (uint8_t)(protocol == PX4LITE_CAPTURE_PROTOCOL_NMEA && type == filter); }
  if (filter == PX4LITE_CAPTURE_FILTER_NMEA) { return (uint8_t)(protocol == PX4LITE_CAPTURE_PROTOCOL_NMEA); }
  if (filter == PX4LITE_CAPTURE_FILTER_UBX) { return (uint8_t)(protocol == PX4LITE_CAPTURE_PROTOCOL_UBX); }
  if (protocol != PX4LITE_CAPTURE_PROTOCOL_UBX || cls != 1U) { return 0U; }
  return (uint8_t)((filter == PX4LITE_CAPTURE_FILTER_PVT && id == 7U) ||
    (filter == PX4LITE_CAPTURE_FILTER_DOP && id == 4U) || (filter == PX4LITE_CAPTURE_FILTER_SAT && id == 0x35U));
}

/** @brief 在临界区外检查完整wire边界；协议发现独立于屏幕存储容量，真实校验结论由Driver传入。 */
static uint8_t CaptureGnssComplete(const uint8_t *bytes, uint16_t length, uint8_t protocol, uint8_t type, uint8_t cls, uint8_t id)
{
  if (bytes == 0 || length < 8U) { return 0U; }
  if (protocol == PX4LITE_CAPTURE_PROTOCOL_NMEA) {
    return (uint8_t)(length <= PX4LITE_CAPTURE_NMEA_BYTES && type <= 4U && bytes[0] == '$' && bytes[length - 1U] == '\n');
  }
  return (uint8_t)(protocol == PX4LITE_CAPTURE_PROTOCOL_UBX && bytes[0] == 0xB5U && bytes[1] == 0x62U &&
    bytes[2] == cls && bytes[3] == id && (uint32_t)length == 8U + (uint32_t)bytes[4] + ((uint32_t)bytes[5] << 8));
}

/**
 * @brief 发布常驻协议事实并按会话门禁直接入静态槽，sensor栈不放392B帧副本。
 * @details 发现协议不依赖running/filter/source，只有完整且Driver确认CRC正确才刷新。
 *          记录含坏CRC帧；临界窗口只复制metadata/bytes和更新索引，不做解析或I/O。
 */
static void CaptureGnss(const uint8_t *bytes, uint16_t length, uint8_t protocol, uint8_t type, uint8_t cls, uint8_t id, uint8_t valid, uint32_t now)
{
  uint32_t session, token;
  uint8_t filter;
  uint8_t complete = CaptureGnssComplete(bytes, length, protocol, type, cls, id);
  Px4Lite_GnssCapturedFrame_t *frame;
  if (complete && valid) {
    taskENTER_CRITICAL();
    if (protocol == PX4LITE_CAPTURE_PROTOCOL_NMEA) { s_status.gnss_seen_mask |= 1U; s_status.last_nmea_ms = now; }
    else { s_status.gnss_seen_mask |= 2U; s_status.last_ubx_ms = now; }
    taskEXIT_CRITICAL();
  }
  if (!s_status.running || s_status.source != PX4LITE_CAPTURE_GNSS) { return; }
  session = s_status.session_id; filter = s_status.filter;
  if (!CaptureGnssMatches(filter, protocol, type, cls, id)) { return; }
  taskENTER_CRITICAL();
  if (!s_status.running || s_status.source != PX4LITE_CAPTURE_GNSS || s_status.session_id != session || s_status.filter != filter) { taskEXIT_CRITICAL(); return; }
  if (!complete || length > PX4LITE_CAPTURE_GNSS_BYTES) { s_status.capture_drops++; taskEXIT_CRITICAL(); return; }
  token = CaptureNext(s_status.latest_token); s_status.latest_token = token;
  frame = &s_store.gnss[s_head];
  frame->session_id = session; frame->token = token; frame->time_ms = now; frame->length = length;
  frame->protocol = protocol; frame->nmea_type = type; frame->class_id = cls; frame->message_id = id;
  frame->checksum_valid = (uint8_t)(valid != 0U);
  memcpy(frame->bytes, bytes, length);
  s_head = (uint8_t)((s_head + 1U) % PX4LITE_CAPTURE_CAPACITY);
  s_status.count++;
  if (s_status.stored_count < PX4LITE_CAPTURE_CAPACITY) { s_status.stored_count++; } else { s_status.overwritten++; }
  taskEXIT_CRITICAL();
  CaptureSummary(session, token, protocol == PX4LITE_CAPTURE_PROTOCOL_UBX ? id : type, protocol, cls, PX4LITE_CAPTURE_GNSS, now, 1U);
}

/** @brief NMEA兼容提交入口，记录与发现均复用通用GNSS路径。 */
void Px4Lite_CaptureNmea(const uint8_t *bytes, uint16_t length, uint8_t type, uint8_t valid, uint32_t now)
{ CaptureGnss(bytes, length, PX4LITE_CAPTURE_PROTOCOL_NMEA, type, 0U, 0U, valid, now); }

/** @brief UBX完整原帧提交入口，坏CRC原字节照录，不能据Count判断定位成功。 */
void Px4Lite_CaptureUbx(const uint8_t *bytes, uint16_t length, uint8_t cls, uint8_t id, uint8_t valid, uint32_t now)
{ CaptureGnss(bytes, length, PX4LITE_CAPTURE_PROTOCOL_UBX, 0U, cls, id, valid, now); }

/** @brief 短窗口内复制匹配槽；零填长度之外的输出，但不解析或按类型重解释payload。 */
Px4Lite_Result_t Px4Lite_CaptureCopyGnss(uint32_t session, uint32_t token, Px4Lite_GnssCapturedFrame_t *out)
{
  uint8_t i;
  if (!out) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  if (session == s_status.session_id && s_status.source == PX4LITE_CAPTURE_GNSS) {
    for (i = 0U; i < s_status.stored_count; ++i) {
      const Px4Lite_GnssCapturedFrame_t *frame = &s_store.gnss[i];
      if (frame->session_id != session || frame->token != token) { continue; }
      memset(out, 0, sizeof(*out));
      out->session_id = frame->session_id; out->token = frame->token; out->time_ms = frame->time_ms; out->length = frame->length;
      out->protocol = frame->protocol; out->nmea_type = frame->nmea_type; out->class_id = frame->class_id; out->message_id = frame->message_id;
      out->checksum_valid = frame->checksum_valid; memcpy(out->bytes, frame->bytes, frame->length);
      taskEXIT_CRITICAL(); return PX4LITE_OK;
    }
  }
  taskEXIT_CRITICAL(); return PX4LITE_NOT_READY;
}

/** @brief 旧API仅复制NMEA，拒绝UBX且失败不写输出，不在栈上建立通用大帧临时副本。 */
Px4Lite_Result_t Px4Lite_CaptureCopyNmea(uint32_t session, uint32_t token, Px4Lite_NmeaSentence_t *out)
{
  uint8_t i;
  if (!out) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  if (session == s_status.session_id && s_status.source == PX4LITE_CAPTURE_GNSS) {
    for (i = 0U; i < s_status.stored_count; ++i) {
      const Px4Lite_GnssCapturedFrame_t *frame = &s_store.gnss[i];
      if (frame->session_id != session || frame->token != token) { continue; }
      if (frame->protocol != PX4LITE_CAPTURE_PROTOCOL_NMEA || frame->length > PX4LITE_CAPTURE_NMEA_BYTES) { break; }
      memset(out, 0, sizeof(*out));
      out->session_id = frame->session_id; out->token = frame->token; out->time_ms = frame->time_ms;
      out->length = frame->length; out->type = frame->nmea_type; out->checksum_valid = frame->checksum_valid;
      memcpy(out->bytes, frame->bytes, frame->length); taskEXIT_CRITICAL(); return PX4LITE_OK;
    }
  }
  taskEXIT_CRITICAL(); return PX4LITE_NOT_READY;
}
