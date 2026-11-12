/**
 * @file sensor_gnss.c
 * @brief Sensor层在固定115200bit/s下实现NMEA/UBX解析、主动识别及确认式接入。
 * @details sensor 为唯一硬件消费者；依赖 BSP 原始收发与 UBX/NMEA 编解码，不调用 HAL，
 *          不判定 OFFLINE/FAILED，不发布 Framework topic。配置仅改 RAM，不保存模块 Flash。
 */

#include "sensor_gnss.h"

#include "bsp_critical.h"
#include "bsp_gnss.h"
#include "sensor_nmea.h"
#include "sensor_ubx.h"

#if (UBX_PAYLOAD_CAPACITY + UBX_FRAME_OVERHEAD) > GNSS_RAW_FORWARD_MAX_BYTES
#error "UBX payload exceeds the independent raw forwarding buffer"
#endif


#include <string.h>
#include <stdlib.h>

#define GNSS_FIX_MIN_SATS 3U
#define GNSS_RECOVER_RETRY_MS 2000U
#define GNSS_RX_BUDGET 512U
#define GNSS_FIXED_BAUD_BPS 115200U
#define GNSS_PROBE_INTERVAL_MS 1100U
#define GNSS_ACK_WAIT_MS 1200U
#define GNSS_LINK_LOSS_MS 6000U
/** @brief 原始字节静默后请求接收自愈的间隔，ms；与有效帧/定位健康判定独立。 */
#define GNSS_RX_SILENCE_MS 6000U
#define GNSS_CONFIG_RETRY_MS 10000U
#define GNSS_PVT_FRESH_MS 3000U
/** @brief UBX半帧的绝对寿命，ms；1016B载荷在115200下约89ms，保留调度余量但不跨离线门限。 */
#define GNSS_UBX_FRAME_TIMEOUT_MS 500U

static Gnss_Snapshot_t s_snapshot;
/** @brief 独立卫星表；sensor在区外完整解析后一次发布，不扩展主导航快照或创建任务。 */
typedef struct {
  uint32_t sequence;    /**< NAV-SAT有效帧代数，自然回绕。 */
  uint32_t received_ms; /**< 完整帧消费时刻，单位ms。 */
  uint32_t itow_ms;     /**< 原GPS周内导航历元，单位ms。 */
  uint8_t total, stored, truncated, available; /**< 报告数、保留数、截断标志和独立就绪标志。 */
  uint8_t used_count;   /**< 完整报告中used标志数量。 */
  Gnss_Satellite_t entries[GNSS_SAT_CAPACITY]; /**< 静态64项，未保留项保持0。 */
} Gnss_SatelliteTable_t;
static Gnss_SatelliteTable_t s_satellite_work, s_satellite_published;
static uint8_t s_line_buf[NMEA_SENTENCE_MAX_LEN + 1U];
static Gnss_DiagnosticSentence_t s_diagnostic_sentence;
static Gnss_UbxDiagnostic_t s_diagnostic_ubx;
static Gnss_RawReject_t s_raw_reject;
static uint32_t s_ubx_candidate_bytes; /**< 已消费的UBX候选字节，不依赖codec私有状态布局。 */
static uint32_t s_ubx_candidate_started_ms; /**< 当前UBX前导首字节的消费时刻，真实ms，可回绕。 */
static uint8_t s_ubx_candidate_timed; /**< 1表示半帧计时已建立；合法0ms不能用时间值判空。 */
static uint32_t s_pending_raw_reject[2]; /**< 无时钟Init丢弃的NMEA/UBX半帧，下一Service各报告一次。 */
/** @brief 只登记轻量副本入口，无额外采集者。 */
void Gnss_SetDiagnosticCallback(Gnss_DiagnosticSentence_t callback) { s_diagnostic_sentence=callback; }
/** @brief 调度前登记完整UBX观察回调，不改变生产解析或接收所有权。 */
void Gnss_SetUbxDiagnosticCallback(Gnss_UbxDiagnostic_t callback) { s_diagnostic_ubx=callback; }
/** @brief 调度前登记来源拒绝回调，不读取串口或产生额外消费者。 */
void Gnss_SetRawRejectCallback(Gnss_RawReject_t callback) { s_raw_reject=callback; }
static uint16_t s_line_pos;

static uint8_t s_recover_rx_pending;
static uint32_t s_recover_rx_last_ms;
static volatile uint8_t s_reinit_request;
static uint32_t s_rx_silence_since_ms; /**< 最近字节消费或实际重建尝试时刻，真实ms。 */
static uint8_t s_rx_silence_timed; /**< 静默观察已建立为1；合法0ms不能用时间值判空。 */
static uint8_t s_silence_recovery_pending; /**< 本owner因无原始字节请求重建为1，不冒充硬件错误。 */

/** @brief 仅 sensor 修改工作副本；对外副本只在 Service 尾部短临界复制。 */
static Gnss_Diagnostics_t s_diag_work, s_diag_published;
static uint8_t s_driver_started, s_diag_ready;
static uint8_t s_recover_attempt_seen;
static Ubx_Parser_t s_ubx;
static uint8_t s_tx_command[64];

/** @brief owner写入、其它任务短临界复制的完整原帧采样槽，不承担队列或测量职责。 */
typedef struct {
  uint8_t bytes[GNSS_RAW_MAX_BYTES]; /**< 最近完整原帧，保留收到的校验字节。 */
  Gnss_RawFrameInfo_t info;         /**< 原帧独立元数据，序号0和时刻0均合法。 */
  uint8_t ready;                    /**< 已捕获完整原帧为1，不用sequence判空。 */
} Gnss_RawSlot_t;
static Gnss_RawSlot_t s_raw_slots[GNSS_RAW_SLOT_COUNT];
static uint8_t s_raw_work[GNSS_RAW_FORWARD_MAX_BYTES]; /**< 独立forward静态工作区；旧3槽仍只有392B，长帧不假截断。 */

/** @brief 非阻塞协议接入的私有上下文；所有时间字段均为真实 ms。 */
typedef struct {
  uint32_t stage_ms;        /**< 当前阶段起点，ms，stage_seen区分合法0。 */
  uint32_t last_tx_ms;      /**< 最近命令被BSP接受的时刻，ms。 */
  uint32_t pvt_ms;          /**< 最近结构有效PVT时刻，ms。 */
  uint32_t pvt_itow;        /**< 最近PVT导航周内时刻，ms。 */
  uint32_t dop_ms;          /**< 最近DOP消费时刻，ms。 */
  uint32_t dop_itow;        /**< 最近DOP导航周内时刻，ms。 */
  uint32_t pvt_count;       /**< 结构有效PVT累计计数。 */
  uint32_t confirm_pvt_count; /**< 最后配置ACK之后的PVT计数起点。 */
  uint32_t observed_drop;   /**< 最近观察到的BSP覆盖累计值，byte。 */
  uint32_t tx_watchdog_ms;  /**< 最近被BSP接受发送的独立时刻，不随协议阶段重置，ms。 */
  uint16_t hdop;           /**< NAV-DOP的HDOP*100，0表示未知。 */
  uint8_t stage_seen;      /**< 1表示阶段计时已建立。 */
  uint8_t tx_seen;         /**< 1表示last_tx_ms已存在。 */
  uint8_t probes;          /**< 当前阶段已提交MON-VER次数，0..3。 */
  uint8_t attempts;        /**< 当前配置已提交次数，0..3。 */
  uint8_t awaiting_ack;    /**< 1表示仅接受匹配当前命令的ACK/NAK。 */
  uint8_t expected_id;     /**< 当前等待CFG消息id。 */
  uint8_t waiting_nav;     /**< 1表示ACK已齐，仍等待后续PVT确认。 */
  uint8_t pvt_seen;        /**< 1曾收到结构有效PVT。 */
  uint8_t pvt_position_ok; /**< 最近PVT自身位置成立，用于同iTOW DOP授权，不由GSA/GGA代替。 */
  uint8_t dop_seen;        /**< 1曾收到DOP。 */
  uint8_t probe_timeout_marked; /**< 本次版本查询超时已记账，BUSY时不得每周期重复计数。 */
  uint8_t tx_watchdog_seen; /**< 独立发送事务看门狗有效，0ms合法。 */
} Gnss_LinkContext_t;
static Gnss_LinkContext_t s_link;

static void Gnss_FeedAndParse(uint32_t now);
static void Gnss_UpdateState(void);
static void Gnss_ParseGga(const Nmea_Sentence_t *sentence, uint32_t now);
static void Gnss_ParseRmc(const Nmea_Sentence_t *sentence, uint32_t now);
static void Gnss_ParseGsv(const Nmea_Sentence_t *sentence, uint32_t now);
static void Gnss_ParseGsa(const Nmea_Sentence_t *sentence, uint32_t now);
static void Gnss_ProcessUbx(uint32_t now);
static void Gnss_LinkFrame(uint8_t protocol, uint32_t now);
static void Gnss_LinkStep(uint32_t now);
static void Gnss_ResetFraming(uint32_t now, uint8_t report_now);
static void Gnss_ExpireUbxCandidate(uint32_t now);
static void Gnss_Unconfirmed(uint32_t now);
static void Gnss_RecordValidSentence(uint32_t now);
static void Gnss_CaptureRaw(uint8_t slot, const uint8_t *bytes, uint16_t length,
                            uint8_t protocol, uint8_t class_id, uint8_t message_id,
                            uint8_t checksum_valid, uint32_t now);
static void Gnss_CaptureUbxRaw(uint8_t checksum_valid, uint32_t now);

/** @brief 在owner上下文报告有依据的软件拒绝事件；绝不构造完整帧或修改typed状态。 */
static void Gnss_RejectRaw(uint8_t protocol, uint32_t length, uint8_t reason, uint32_t now)
{ if (s_raw_reject != NULL) { s_raw_reject(protocol, length, reason, now); } }

/** @brief 用当前真实服务时刻交付无时钟Init留下的半帧事件，不用0冒充复位时刻。 */
static void Gnss_FlushRawReject(uint32_t now)
{
  uint8_t i;
  for (i = 0U; i < 2U; ++i) {
    uint32_t length = s_pending_raw_reject[i];
    s_pending_raw_reject[i] = 0U;
    if (length != 0U) { Gnss_RejectRaw((uint8_t)(i + 1U), length, GNSS_RAW_REJECT_FRAMING, now); }
  }
}

/** @brief 完整有界复制一个NMEA字段；空字段成功返回空串，缺字段或容量不足失败，不接受截断前缀。 */
static uint8_t Gnss_ReadField(const Nmea_Sentence_t *sentence, uint8_t index, char *out, uint16_t capacity)
{
  uint16_t at = 0U, count = 0U;
  uint8_t field = 0U;
  char ch;
  if (sentence == NULL || out == NULL || capacity == 0U) { return 0U; }
  while (at < sentence->raw_len && field < index) {
    ch = sentence->raw[at++];
    if (ch == '\0' || ch == '*' || ch == '\r' || ch == '\n') { return 0U; }
    if (ch == ',') { field++; }
  }
  if (field != index) { return 0U; }
  while (at < sentence->raw_len) {
    ch = sentence->raw[at++];
    if (ch == '\0' || ch == ',' || ch == '*' || ch == '\r' || ch == '\n') { break; }
    if (count >= (uint16_t)(capacity - 1U)) { return 0U; }
    out[count++] = ch;
  }
  out[count] = '\0';
  return 1U;
}

/** @brief 当前PVT仅在其接收服务时刻后3秒内保护基础测量；不屏蔽独立NMEA统计事实。 */
static uint8_t Gnss_PvtFresh(uint32_t now)
{ return (uint8_t)(s_link.pvt_seen && (uint32_t)(now - s_link.pvt_ms) <= GNSS_PVT_FRESH_MS); }

/** @brief caller持有快照临界区时更新一个DOP事实；0和65535撤授权而不伪造新时刻。 */
static void Gnss_SetDop(uint8_t index, uint16_t value, uint8_t source, uint32_t now, uint8_t itow_known, uint32_t itow, uint8_t solution_ok)
{
  uint8_t bit = (uint8_t)(1U << index);
  if (value == 0U || value == UINT16_MAX) {
    s_snapshot.dop_present_mask &= (uint8_t)~bit;
    s_snapshot.dop_itow_known_mask &= (uint8_t)~bit;
    s_snapshot.dop_solution_mask &= (uint8_t)~bit;
    return;
  }
  s_snapshot.dop_x100[index] = value; s_snapshot.dop_source[index] = source;
  s_snapshot.dop_update_ms[index] = now; s_snapshot.dop_present_mask |= bit;
  if (itow_known) { s_snapshot.dop_itow_ms[index] = itow; s_snapshot.dop_itow_known_mask |= bit; }
  else { s_snapshot.dop_itow_known_mask &= (uint8_t)~bit; }
  if (solution_ok) { s_snapshot.dop_solution_mask |= bit; }
  else { s_snapshot.dop_solution_mask &= (uint8_t)~bit; }
}

/** @brief caller持有临界区时仅重算UBX-DOP与当前PVT的授权位，不更新DOP值、来源或采样时刻。 */
static void Gnss_PairDopSolution(uint32_t itow, uint8_t position_ok, uint32_t now)
{
  uint8_t i, bit;
  for (i = 0U; i < GNSS_DOP_COUNT; ++i) {
    if (s_snapshot.dop_source[i] != GNSS_SOURCE_UBX_DOP) { continue; }
    bit = (uint8_t)(1U << i);
    s_snapshot.dop_solution_mask &= (uint8_t)~bit;
    if (position_ok && (s_snapshot.dop_present_mask & bit) && (s_snapshot.dop_itow_known_mask & bit) &&
        s_snapshot.dop_itow_ms[i] == itow && (uint32_t)(now - s_snapshot.dop_update_ms[i]) <= GNSS_PVT_FRESH_MS) {
      s_snapshot.dop_solution_mask |= bit;
    }
  }
}

/** @brief 严格检查有界十进制字段；空字段、尾随字符、NaN 和超范围均不产生测量。 */
static uint8_t Gnss_ReadDecimal(const Nmea_Sentence_t *sentence, uint8_t index, double minimum, double maximum, double *value)
{
  char field[32];
  char *end;
  uint8_t i = 0U;
  uint8_t digits = 0U;
  uint8_t points = 0U;
  if (Gnss_ReadField(sentence, index, field, sizeof(field)) == 0U) { return 0U; }
  if ((field[0] == '-') || (field[0] == '+')) { i++; }
  for (; field[i] != '\0'; i++) {
    if ((field[i] >= '0') && (field[i] <= '9')) { digits++; }
    else if ((field[i] == '.') && (points == 0U)) { points++; }
    else { return 0U; }
  }
  if (digits == 0U) { return 0U; }
  *value = strtod(field, &end);
  return (uint8_t)((*end == '\0') && (*value >= minimum) && (*value <= maximum));
}

/** @brief 检查经纬度字段的度分范围和半球，防止缺字段复用上帧坐标。 */
static uint8_t Gnss_CoordinateValid(const Nmea_Sentence_t *sentence, uint8_t index, uint8_t latitude, int32_t *coordinate)
{
  char hemisphere[4];
  double value;
  uint32_t degree;
  uint32_t maximum = (latitude != 0U) ? 90U : 180U;
  if ((Gnss_ReadDecimal(sentence, index, 0.0, (double)maximum * 100.0, &value) == 0U) ||
      (Gnss_ReadField(sentence, (uint8_t)(index + 1U), hemisphere, sizeof(hemisphere)) == 0U)) { return 0U; }
  if ((hemisphere[1] != '\0') || ((latitude != 0U) ? ((hemisphere[0] != 'N') && (hemisphere[0] != 'S')) : ((hemisphere[0] != 'E') && (hemisphere[0] != 'W')))) { return 0U; }
  degree = (uint32_t)(value / 100.0);
  if ((value - (double)degree * 100.0) >= 60.0) { return 0U; }
  *coordinate = (int32_t)((double)degree * 10000000.0 + (value - (double)degree * 100.0) * (10000000.0 / 60.0));
  if ((hemisphere[0] == 'S') || (hemisphere[0] == 'W')) { *coordinate = -*coordinate; }
  return 1U;
}

/**
 * @brief 严格读取完整NMEA整数状态字段，拒绝截断、符号、小数及越界值。
 * @param[in] sentence 有界原句；读取不超过raw_len，不额外申请句长栈缓冲。
 * @param[in] index 从0开始的逗号字段编号。
 * @param[in] maximum 允许的最大值，范围0..255。
 * @param[out] value 成功时的完整整数值，失败时不写输出。
 * @return 完整非空字段仅含数字且不越界时返回1，否则返回0。
 */
static uint8_t Gnss_ReadStatusUint8(const Nmea_Sentence_t *sentence, uint8_t index, uint8_t maximum, uint8_t *value)
{
  uint16_t cursor = 0U, parsed = 0U;
  uint8_t field = 0U, digits = 0U;
  char ch;
  while (cursor < sentence->raw_len && field < index) {
    ch = sentence->raw[cursor++];
    if (ch == '\0' || ch == '*' || ch == '\r' || ch == '\n') { return 0U; }
    if (ch == ',') { field++; }
  }
  if (field != index) { return 0U; }
  while (cursor < sentence->raw_len) {
    ch = sentence->raw[cursor++];
    if (ch == '\0' || ch == ',' || ch == '*' || ch == '\r' || ch == '\n') { break; }
    if (ch < '0' || ch > '9') { return 0U; }
    parsed = (uint16_t)(parsed * 10U + (uint16_t)(ch - '0'));
    if (parsed > maximum) { return 0U; }
    digits = 1U;
  }
  if (digits == 0U) { return 0U; }
  *value = (uint8_t)parsed;
  return 1U;
}

/** @brief 将完整NMEA DOP字段按最近百分之一归一化；缺失/非数/超范围/0均返回未知0。 */
static uint16_t Gnss_ReadNmeaDop(const Nmea_Sentence_t *sentence, uint8_t index)
{
  double value;
  uint32_t scaled;
  if (!Gnss_ReadDecimal(sentence, index, 0.0, 655.34, &value) || value <= 0.0) { return 0U; }
  scaled = (uint32_t)(value * 100.0 + 0.5);
  return (scaled > 0U && scaled < UINT16_MAX) ? (uint16_t)scaled : 0U;
}

/**
 * @brief GGA独立更新完整定位状态/总卫星事实和有效位置/海拔，不续期速度或UTC。
 * @details quality和总卫星数字段完整有效时，即使无fix、卫星不足或坐标缺失也保留真实卫星数；
 *          状态字段畸形则撤销状态有效位，不续期旧状态，不把未知包装为已知0颗。
 */
static void Gnss_ParseGga(const Nmea_Sentence_t *sentence, uint32_t now)
{
  Nmea_GeoData_t geo;
  double altitude;
  uint8_t quality = 0U, satellites = 0U, status_ok;
  uint8_t position_ok;
  uint8_t altitude_ok;
  char unit[2];
  uint16_t reported_hdop = Gnss_ReadNmeaDop(sentence, 8U);
  int32_t altitude_mm = 0;
  uint32_t primask;
  memset(&geo, 0, sizeof(geo));
  status_ok = (uint8_t)(Gnss_ReadStatusUint8(sentence, 6U, 8U, &quality) &&
                       Gnss_ReadStatusUint8(sentence, 7U, 255U, &satellites));
  position_ok = (uint8_t)(status_ok && (quality == 1U || quality == 2U || quality == 4U || quality == 5U) && satellites >= GNSS_FIX_MIN_SATS &&
    (Gnss_CoordinateValid(sentence, 2U, 1U, &geo.latitude_deg_e7) != 0U) &&
    (Gnss_CoordinateValid(sentence, 4U, 0U, &geo.longitude_deg_e7) != 0U));
  primask = BSP_Critical_Enter();
  Gnss_SetDop(GNSS_DOP_H, reported_hdop, GNSS_SOURCE_NMEA_GGA, now, 0U, 0U, position_ok);
  BSP_Critical_Exit(primask);
  if (Gnss_PvtFresh(now)) { return; }
  altitude_ok = (uint8_t)((position_ok != 0U) && (Gnss_ReadDecimal(sentence, 9U, -100000.0, 100000.0, &altitude) != 0U) &&
    Gnss_ReadField(sentence, 10U, unit, sizeof(unit)) && unit[0] == 'M' && unit[1] == '\0');
  if (position_ok != 0U) {
    geo.fix_quality = (uint8_t)quality;
    geo.satellites = (uint8_t)satellites;
    geo.hdop_cm = reported_hdop;
  }
  if (altitude_ok != 0U) { altitude_mm = (int32_t)(altitude * 1000.0); }
  primask = BSP_Critical_Enter();
  s_snapshot.valid_mask &= ~(GNSS_VALID_POSITION | GNSS_VALID_ALTITUDE | GNSS_VALID_NAV_STATUS);
  s_snapshot.fix_valid = position_ok;
  s_snapshot.fix_quality = (position_ok != 0U) ? geo.fix_quality : 0U;
  if (status_ok) {
    s_snapshot.satellites = satellites;
    s_snapshot.status_update_ms = now;
    s_snapshot.status_source = GNSS_SOURCE_NMEA_GGA; s_snapshot.raw_status_type = quality;
    s_snapshot.valid_mask |= GNSS_VALID_NAV_STATUS;
  }
  if (position_ok != 0U) {
    s_snapshot.latitude_deg_e7 = geo.latitude_deg_e7;
    s_snapshot.longitude_deg_e7 = geo.longitude_deg_e7;
    s_snapshot.hdop_cm = geo.hdop_cm;
    s_snapshot.position_update_ms = now;
    s_snapshot.position_source = GNSS_SOURCE_NMEA_GGA;
    s_snapshot.last_fix_ms = now;
    s_snapshot.valid_mask |= GNSS_VALID_POSITION;
  } else {
    s_snapshot.fix_dimension = 0U;
    s_snapshot.dimension_source = GNSS_SOURCE_NONE;
    s_snapshot.valid_mask &= ~GNSS_VALID_VELOCITY;
  }
  if (altitude_ok != 0U) {
    s_snapshot.altitude_mm = altitude_mm;
    s_snapshot.altitude_update_ms = now;
    s_snapshot.altitude_source = GNSS_SOURCE_NMEA_GGA;
    s_snapshot.valid_mask |= GNSS_VALID_ALTITUDE;
  }
  BSP_Critical_Exit(primask);
}

/** @brief 校验 RMC 的完整 UTC 日期和时间；合法午夜 000000 必须保留。 */
static uint8_t Gnss_ParseUtc(const Nmea_Sentence_t *sentence, uint32_t *date, uint32_t *seconds)
{
  char time[20], day[12];
  uint32_t hour, minute, second, dd, mm, yy, limit;
  uint8_t i;
  static const uint8_t days[12] = {31U,28U,31U,30U,31U,30U,31U,31U,30U,31U,30U,31U};
  if ((Gnss_ReadField(sentence, 1U, time, sizeof(time)) == 0U) ||
      (Gnss_ReadField(sentence, 9U, day, sizeof(day)) == 0U)) { return 0U; }
  if ((strlen(time) < 6U) || (strlen(day) != 6U)) { return 0U; }
  for (i = 0U; i < 6U; i++) { if ((time[i] < '0') || (time[i] > '9') || (day[i] < '0') || (day[i] > '9')) { return 0U; } }
  if ((time[6] != '\0') && (time[6] != '.')) { return 0U; }
  if (time[6] == '.') { for (i = 7U; time[i] != '\0'; i++) { if ((time[i] < '0') || (time[i] > '9')) { return 0U; } } }
  hour = (uint32_t)(time[0]-'0')*10U+(uint32_t)(time[1]-'0');
  minute = (uint32_t)(time[2]-'0')*10U+(uint32_t)(time[3]-'0');
  second = (uint32_t)(time[4]-'0')*10U+(uint32_t)(time[5]-'0');
  dd = (uint32_t)(day[0]-'0')*10U+(uint32_t)(day[1]-'0');
  mm = (uint32_t)(day[2]-'0')*10U+(uint32_t)(day[3]-'0');
  yy = (uint32_t)(day[4]-'0')*10U+(uint32_t)(day[5]-'0');
  if ((hour >= 24U) || (minute >= 60U) || (second >= 60U) || (mm == 0U) || (mm > 12U)) { return 0U; }
  limit = days[mm-1U]; if ((mm == 2U) && ((yy % 4U) == 0U)) { limit++; }
  if ((dd == 0U) || (dd > limit)) { return 0U; }
  *date = yy*10000U+mm*100U+dd; *seconds = hour*3600U+minute*60U+second;
  return 1U;
}

/** @brief RMC 只更新速度与完整 UTC；无效状态立即撤销位置事实，但不续期 GGA。 */
static void Gnss_ParseRmc(const Nmea_Sentence_t *sentence, uint32_t now)
{
  char active[4];
  double speed, course;
  uint32_t date = 0U, seconds = 0U;
  uint8_t active_ok, velocity_ok, time_ok;
  uint32_t speed_cms = 0U, heading_deg100 = 0U;
  uint32_t primask;
  active_ok = (uint8_t)((Gnss_ReadField(sentence, 2U, active, sizeof(active)) != 0U) && (active[0] == 'A') && (active[1] == '\0'));
  velocity_ok = (uint8_t)((active_ok != 0U) && (Gnss_ReadDecimal(sentence, 7U, 0.0, 1274.0, &speed) != 0U) && (Gnss_ReadDecimal(sentence, 8U, 0.0, 360.0, &course) != 0U));
  time_ok = (uint8_t)((active_ok != 0U) && (Gnss_ParseUtc(sentence, &date, &seconds) != 0U));
  if (velocity_ok != 0U) {
    speed_cms = (uint32_t)(speed * 51.4444444444);
    heading_deg100 = ((uint32_t)(course * 100.0)) % 36000U;
  }
  primask = BSP_Critical_Enter();
  s_snapshot.valid_mask &= ~(GNSS_VALID_VELOCITY | GNSS_VALID_TIME);
  if (active_ok == 0U) {
    s_snapshot.valid_mask &= ~(GNSS_VALID_POSITION | GNSS_VALID_ALTITUDE);
    s_snapshot.fix_valid = 0U; s_snapshot.fix_quality = 0U; s_snapshot.fix_dimension = 0U;
    s_snapshot.dimension_source = GNSS_SOURCE_NONE;
  }
  if (velocity_ok != 0U) {
    s_snapshot.speed_cms = speed_cms;
    s_snapshot.heading_deg100 = heading_deg100;
    s_snapshot.velocity_update_ms = now;
    s_snapshot.velocity_source = GNSS_SOURCE_NMEA_RMC;
    s_snapshot.valid_mask |= GNSS_VALID_VELOCITY;
  }
  if (time_ok != 0U) {
    s_snapshot.utc_date = date; s_snapshot.utc_sec = seconds;
    s_snapshot.time_update_ms = now;
    s_snapshot.time_source = GNSS_SOURCE_NMEA_RMC;
    s_snapshot.valid_mask |= GNSS_VALID_TIME;
  }
  BSP_Critical_Exit(primask);
}

/** @brief caller持有快照临界区时发布一项计数，0合法；无效字段只撤授权，不续期旧值。 */
static void Gnss_SetCount(uint8_t index, uint8_t value, uint8_t source, uint32_t now, uint8_t valid)
{
  uint8_t bit = (uint8_t)(1U << index);
  if (!valid) { s_snapshot.counts_present_mask &= (uint8_t)~bit; return; }
  switch (index) {
    case GNSS_COUNT_GPS_VISIBLE: s_snapshot.gps_visible_sats = value; break;
    case GNSS_COUNT_BDS_VISIBLE: s_snapshot.bds_visible_sats = value; break;
    case GNSS_COUNT_GPS_USED: s_snapshot.gps_used_sats = value; break;
    default: s_snapshot.bds_used_sats = value; break;
  }
  s_snapshot.counts_source[index] = source; s_snapshot.counts_update_ms[index] = now;
  s_snapshot.counts_present_mask |= bit;
}

/** @brief 仅识别明确GP/BD/GB报告来源，不把GN/GL/GA等未知类别计作GPS或北斗。 */
static uint8_t Gnss_CountSystem(const Nmea_Sentence_t *sentence)
{
  if (!memcmp(sentence->raw, "$GP", 3U)) { return 0U; }
  if (!memcmp(sentence->raw, "$BD", 3U) || !memcmp(sentence->raw, "$GB", 3U)) { return 1U; }
  return UINT8_MAX;
}

/** @brief GSV按完整整数页号及报告总数更新独立计数；同组分页或不同信号不相加。 */
static void Gnss_ParseGsv(const Nmea_Sentence_t *sentence, uint32_t now)
{
  uint8_t system = Gnss_CountSystem(sentence), total = 0U, page = 0U, visible = 0U, valid;
  uint32_t primask;
  if (system == UINT8_MAX) { return; }
  valid = (uint8_t)(Gnss_ReadStatusUint8(sentence, 1U, 255U, &total) && total != 0U &&
    Gnss_ReadStatusUint8(sentence, 2U, 255U, &page) && page != 0U && page <= total &&
    Gnss_ReadStatusUint8(sentence, 3U, 255U, &visible));
  primask = BSP_Critical_Enter();
  Gnss_SetCount(system, visible, GNSS_SOURCE_NMEA_GSV, now, valid);
  BSP_Critical_Exit(primask);
}

/** @brief 逐个验证GSA的12个ID字段并按数值去重；不跨语句累加，非数字/越界/缺字段均失败。 */
static uint8_t Gnss_CountGsaSatellites(const Nmea_Sentence_t *sentence, uint8_t *count)
{
  uint16_t ids[12], value;
  uint8_t field_index, length = 0U, j, i;
  char field[8];
  for (field_index = 3U; field_index <= 14U; field_index++) {
    if (!Gnss_ReadField(sentence, field_index, field, sizeof(field))) { return 0U; }
    if (field[0] == '\0') { continue; }
    value = 0U;
    for (i = 0U; field[i] != '\0'; ++i) {
      if (field[i] < '0' || field[i] > '9') { return 0U; }
      value = (uint16_t)(value * 10U + (uint16_t)(field[i] - '0'));
      if (value > 999U) { return 0U; }
    }
    if (value == 0U) { return 0U; }
    for (j = 0U; j < length && ids[j] != value; ++j) { }
    if (j == length) { ids[length++] = value; }
  }
  *count = length;
  return 1U;
}

/** @brief GSA提供独立P/H/V DOP与明确星座的使用数；近期PVT期间不覆盖全局维度/位置。 */
static void Gnss_ParseGsa(const Nmea_Sentence_t *sentence, uint32_t now)
{
  uint16_t pdop = Gnss_ReadNmeaDop(sentence, 15U), hdop = Gnss_ReadNmeaDop(sentence, 16U);
  uint16_t vdop = Gnss_ReadNmeaDop(sentence, 17U);
  uint8_t system = Gnss_CountSystem(sentence), used = 0U, dimension = 0U, dimension_ok, counts_ok;
  uint8_t pvt_protected = Gnss_PvtFresh(now);
  char mode[2];
  uint32_t primask;
  dimension_ok = (uint8_t)(Gnss_ReadField(sentence, 1U, mode, sizeof(mode)) &&
    (mode[0] == 'A' || mode[0] == 'M') && mode[1] == '\0' &&
    Gnss_ReadStatusUint8(sentence, 2U, 3U, &dimension) && dimension >= 1U);
  counts_ok = (uint8_t)(dimension_ok && Gnss_CountGsaSatellites(sentence, &used));
  if ((dimension == 1U && used != 0U) || (dimension > 1U && used == 0U)) { counts_ok = 0U; }
  primask = BSP_Critical_Enter();
  Gnss_SetDop(GNSS_DOP_P, pdop, GNSS_SOURCE_NMEA_GSA, now, 0U, 0U, (uint8_t)(dimension_ok && dimension >= 2U));
  Gnss_SetDop(GNSS_DOP_H, hdop, GNSS_SOURCE_NMEA_GSA, now, 0U, 0U, (uint8_t)(dimension_ok && dimension >= 2U));
  Gnss_SetDop(GNSS_DOP_V, vdop, GNSS_SOURCE_NMEA_GSA, now, 0U, 0U, (uint8_t)(dimension_ok && dimension >= 2U));
  if (!pvt_protected) {
    if (dimension_ok && dimension >= 2U) {
      s_snapshot.fix_dimension = dimension; s_snapshot.dimension_source = GNSS_SOURCE_NMEA_GSA;
      s_snapshot.dimension_update_ms = now;
    } else if (s_snapshot.dimension_source == GNSS_SOURCE_NMEA_GSA) { s_snapshot.dimension_source = GNSS_SOURCE_NONE; }
  }
  if (system != UINT8_MAX) { Gnss_SetCount((uint8_t)(system + GNSS_COUNT_GPS_USED), used, GNSS_SOURCE_NMEA_GSA, now, counts_ok); }
  BSP_Critical_Exit(primask);
}

/**
 * @brief 由链路接收和最近定位事实更新驱动数据状态。
 */
static void Gnss_UpdateState(void)
{
  uint32_t primask = BSP_Critical_Enter();

  if (s_snapshot.rx_sequence == 0U) {
    s_snapshot.data_state = GNSS_DATA_NONE;
  } else {
    s_snapshot.data_state = (s_snapshot.fix_valid != 0U) ? GNSS_DATA_FIX_OK : GNSS_DATA_NO_FIX;
  }

  BSP_Critical_Exit(primask);
}

/**
 * @brief 复位 GNSS 拆句缓存及强类型快照。
 */
Gnss_Result_t Sensor_GNSS_Init(void)
{
  uint32_t primask;
  if (!s_driver_started) {
    memset(&s_link, 0, sizeof(s_link));
    memset(&s_diag_work, 0, sizeof(s_diag_work));
    memset(&s_diag_published, 0, sizeof(s_diag_published));
    memset(s_raw_slots, 0, sizeof(s_raw_slots));
    memset(&s_satellite_work, 0, sizeof(s_satellite_work));
    memset(&s_satellite_published, 0, sizeof(s_satellite_published));
    s_diag_work.state = GNSS_LINK_SCAN;
    s_driver_started = 1U;
    s_recover_rx_pending = 0U;
    s_recover_attempt_seen = 0U;
    s_rx_silence_since_ms = 0U;s_rx_silence_timed = 0U;s_silence_recovery_pending = 0U;
  }
  /* 注册表软恢复只清解析/测量，不擦累计诊断、版本探测进度或正在等待的配置阶段。 */
  primask = BSP_Critical_Enter();
  memset(&s_snapshot, 0, sizeof(s_snapshot));
  s_link.pvt_position_ok = 0U;
  s_snapshot.data_state = GNSS_DATA_NONE;
  BSP_Critical_Exit(primask);
  Gnss_ResetFraming(0U, 0U);
  s_reinit_request = 0U;
  return GNSS_RESULT_OK;
}

/**
 * @brief 请求在 sensor 所属任务下一周期复位解析器。
 */
void Sensor_GNSS_RequestReinit(void)
{
  s_reinit_request = 1U;
}

/** @brief 记录真实原始字节活动或owner实际恢复尝试，warm解析器Init不得调用。 */
static void Gnss_MarkRxObservation(uint32_t now)
{
  s_rx_silence_since_ms=now;s_rx_silence_timed=1U;
}

/**
 * @brief 仅原始字节连续静默达到6秒时登记硬件自愈，下一个owner周期执行重建。
 * @details 必须位于本轮Feed/LinkStep之后，当前有字节即取消静默计时，不因CRC坏而重建。
 *          不改变有效RX/测量时标、Health门限或现有协议查询退避。
 */
static void Gnss_CheckRxSilence(uint32_t now)
{
  if(!s_rx_silence_timed) { Gnss_MarkRxObservation(now);return; }
  if(!s_recover_rx_pending && !s_silence_recovery_pending &&
      (uint32_t)(now-s_rx_silence_since_ms)>=GNSS_RX_SILENCE_MS) {
    s_silence_recovery_pending=1U;BSP_GNSS_RequestRecoverRx();
  }
}

/**
 * @brief sensor唯一采集任务处理恢复请求和协议字节，now_ms为真实毫秒。
 * @details 串口恢复始终保持115200bit/s；软复位及硬件恢复均保留上电累计诊断。
 *          收新字节前撤销寿命达到500ms的UBX半帧，不重启UART或续期有效数据。
 *          TX单独中止成功只标配置未确认；真实接收错误仍保留IO_ERROR和原恢复。
 *          连续6秒没有原始字节才请求下拍重建；实际重建尝试重新观察，失败仍2秒退避。
 *          本入口只由sensor任务调用且不可重入；恢复传输事实用owner静态暂存，
 *          每次尝试清零，不向外部借用地址或改变任何快照发布/失败语义。
 */
Gnss_Result_t Sensor_GNSS_Service(uint32_t now_ms)
{
  Gnss_Result_t result = GNSS_RESULT_OK;
  uint32_t primask;
  if (!s_driver_started) { (void)Sensor_GNSS_Init(); }
  s_diag_work.measured_ms = now_ms;
  s_diag_work.service_count++;
  if (s_reinit_request != 0U) {
    s_reinit_request = 0U;
    (void)Sensor_GNSS_Init();
  }
  Gnss_FlushRawReject(now_ms);
  Gnss_ExpireUbxCandidate(now_ms);
  if (BSP_GNSS_ConsumeRecoverRxRequest() != 0U) { s_recover_rx_pending = 1U; }
  if (s_recover_rx_pending != 0U) {
    if ((!s_recover_attempt_seen) || ((uint32_t)(now_ms - s_recover_rx_last_ms) >= GNSS_RECOVER_RETRY_MS)) {
      /** @brief sensor所有者独占的重建前后事实，避免嵌套接收解析占用80B调用栈。 */
      static BSP_GNSS_TransportInfo_t before,after;
      uint8_t preserve_link=0U,interrupted_tx=0U;
      memset(&before,0,sizeof(before));memset(&after,0,sizeof(after));
      if(s_silence_recovery_pending && BSP_GNSS_CopyTransportInfo(&before)==BSP_STATUS_OK && before.initialized) {
        preserve_link=1U;interrupted_tx=before.tx_busy;
      }
      s_recover_rx_last_ms = now_ms; s_recover_attempt_seen = 1U;
      Gnss_MarkRxObservation(now_ms);
      if (BSP_GNSS_RecoverRx() == BSP_STATUS_OK) {
        if(preserve_link && (BSP_GNSS_CopyTransportInfo(&after)!=BSP_STATUS_OK || !after.initialized ||
            after.uart_errors!=before.uart_errors || after.dma_errors!=before.dma_errors)) { preserve_link=0U; }
        s_recover_rx_pending = 0U;
        Gnss_ResetFraming(now_ms, 1U);
        s_link.tx_watchdog_seen=0U;
        if(!preserve_link) {
          s_link.stage_seen = 0U; s_link.probes = 0U; s_link.awaiting_ack = 0U; s_link.waiting_nav = 0U;
          s_diag_work.state = GNSS_LINK_SCAN; s_diag_work.version_seen = 0U;
          s_diag_work.protocol_version_x100 = 0U; s_diag_work.config_confirmed = 0U;
        } else if(interrupted_tx) { s_link.tx_seen=0U;Gnss_Unconfirmed(now_ms); }
      }
      s_silence_recovery_pending=0U;
    }
    if (s_recover_rx_pending != 0U) { s_diag_work.state = GNSS_LINK_IO_ERROR; result = GNSS_RESULT_IO_ERROR; }
  }
  if (result == GNSS_RESULT_OK) {
    BSP_GNSS_RxIdleCallback(0U);
    Gnss_FeedAndParse(now_ms);
    Gnss_LinkStep(now_ms);
    Gnss_UpdateState();
    if (s_diag_work.state == GNSS_LINK_IO_ERROR) { result = GNSS_RESULT_IO_ERROR; }
    else { Gnss_CheckRxSilence(now_ms); }
  }
  primask = BSP_Critical_Enter();
  s_diag_published = s_diag_work; s_diag_ready = 1U;
  BSP_Critical_Exit(primask);
  return result;
}

/** @brief 只复制已发布驱动诊断；不读硬件或代替sensor推进接收。 */
Gnss_Result_t Sensor_GNSS_CopyDiagnostics(Gnss_Diagnostics_t *out)
{
  uint8_t ready;
  uint32_t primask;
  if (out == NULL) { return GNSS_RESULT_INVALID_PARAM; }
  primask = BSP_Critical_Enter();
  *out = s_diag_published; ready = s_diag_ready;
  BSP_Critical_Exit(primask);
  return ready ? GNSS_RESULT_OK : GNSS_RESULT_NO_DATA;
}

/** @brief 只复制完整原帧采样及元数据，保持槽ready/序号和硬件接收游标不变。 */
Gnss_Result_t Sensor_GNSS_CopyRawFrame(uint8_t slot, uint8_t *bytes, uint16_t capacity, Gnss_RawFrameInfo_t *info)
{
  uint32_t primask;
  Gnss_Result_t result;
  if (slot >= GNSS_RAW_SLOT_COUNT || bytes == NULL || info == NULL || capacity == 0U) { return GNSS_RESULT_INVALID_PARAM; }
  primask = BSP_Critical_Enter();
  if (!s_raw_slots[slot].ready) { result = GNSS_RESULT_NO_DATA; }
  else if (capacity < s_raw_slots[slot].info.length) { result = GNSS_RESULT_INVALID_PARAM; }
  else {
    memcpy(bytes, s_raw_slots[slot].bytes, s_raw_slots[slot].info.length);
    *info = s_raw_slots[slot].info;
    result = GNSS_RESULT_OK;
  }
  BSP_Critical_Exit(primask);
  return result;
}

/** @brief owner发布一个有界完整原帧；临界区内仅复制字节/元数据和设置ready。 */
static void Gnss_CaptureRaw(uint8_t slot, const uint8_t *bytes, uint16_t length,
                            uint8_t protocol, uint8_t class_id, uint8_t message_id,
                            uint8_t checksum_valid, uint32_t now)
{
  Gnss_RawFrameInfo_t info;
  uint32_t primask;
  if (slot >= GNSS_RAW_SLOT_COUNT || bytes == NULL || length == 0U || length > GNSS_RAW_MAX_BYTES) { return; }
  memset(&info, 0, sizeof(info));
  info.sequence = s_raw_slots[slot].ready ? s_raw_slots[slot].info.sequence + 1U : 0U;
  info.received_ms = now; info.length = length; info.protocol = protocol;
  info.class_id = class_id; info.message_id = message_id; info.checksum_valid = checksum_valid;
  primask = BSP_Critical_Enter();
  memcpy(s_raw_slots[slot].bytes, bytes, length);
  s_raw_slots[slot].info = info;
  s_raw_slots[slot].ready = 1U;
  BSP_Critical_Exit(primask);
}

/** @brief 用刚收到的头/载荷/原校验保存完整UBX字节，绝不重算或修正坏校验。 */
static void Gnss_CaptureUbxRaw(uint8_t checksum_valid, uint32_t now)
{
  uint16_t length;
  uint8_t slot = 2U;
  if (s_ubx.length > UBX_PAYLOAD_CAPACITY) {
    Gnss_RejectRaw(GNSS_PROTOCOL_UBX, (uint32_t)s_ubx.length + UBX_FRAME_OVERHEAD, GNSS_RAW_REJECT_OVERSIZE, now);
    return;
  }
  length = (uint16_t)(s_ubx.length + UBX_FRAME_OVERHEAD);
  s_raw_work[0] = 0xB5U; s_raw_work[1] = 0x62U;
  s_raw_work[2] = s_ubx.class_id; s_raw_work[3] = s_ubx.message_id;
  s_raw_work[4] = (uint8_t)s_ubx.length; s_raw_work[5] = (uint8_t)(s_ubx.length >> 8);
  memcpy(&s_raw_work[6], s_ubx.payload, s_ubx.length);
  s_raw_work[length - 2U] = s_ubx.received_ck_a; s_raw_work[length - 1U] = s_ubx.received_ck_b;
  if (s_ubx.class_id == 0x01U && s_ubx.message_id == 0x07U) { slot = 0U; }
  else if (s_ubx.class_id == 0x01U && s_ubx.message_id == 0x04U) { slot = 1U; }
  Gnss_CaptureRaw(slot, s_raw_work, length, GNSS_PROTOCOL_UBX, s_ubx.class_id, s_ubx.message_id, checksum_valid, now);
  if (s_diagnostic_ubx != NULL) {
    s_diagnostic_ubx(s_raw_work, length, s_ubx.class_id, s_ubx.message_id, checksum_valid, now);
  }
}

/**
 * @brief 短临界区复制 Service 已生成的快照，不访问总线。
 */
Gnss_Result_t Sensor_GNSS_CopySnapshot(Gnss_Snapshot_t *out)
{
  uint32_t primask;

  if (out == NULL) { return GNSS_RESULT_INVALID_PARAM; }

  primask = BSP_Critical_Enter();
  *out    = s_snapshot;
  BSP_Critical_Exit(primask);

  if (out->rx_sequence == 0U) { return GNSS_RESULT_NO_DATA; }
  return (out->fix_valid != 0U) ? GNSS_RESULT_OK : GNSS_RESULT_NO_FIX;
}

/** @brief 只复制已发布卫星页，临界区内不解析、计数、排序、访问UART或执行过期策略。 */
Gnss_Result_t Sensor_GNSS_CopySatellitePage(uint8_t page, Gnss_SatellitePage_t *out)
{
  Gnss_SatellitePage_t copy;
  uint16_t start = (uint16_t)page * GNSS_SAT_PAGE_CAPACITY;
  uint16_t remaining;
  uint16_t bytes;
  uint32_t primask;
  if (out == NULL || start >= GNSS_SAT_CAPACITY) { return GNSS_RESULT_INVALID_PARAM; }
  remaining = (uint16_t)(GNSS_SAT_CAPACITY - start);
  if (remaining > GNSS_SAT_PAGE_CAPACITY) { remaining = GNSS_SAT_PAGE_CAPACITY; }
  bytes = (uint16_t)(remaining * sizeof(Gnss_Satellite_t));
  memset(&copy, 0, sizeof(copy));
  primask = BSP_Critical_Enter();
  copy.sequence = s_satellite_published.sequence;
  copy.received_ms = s_satellite_published.received_ms;
  copy.itow_ms = s_satellite_published.itow_ms;
  copy.total = s_satellite_published.total;
  copy.stored = s_satellite_published.stored;
  copy.truncated = s_satellite_published.truncated;
  copy.available = s_satellite_published.available;
  copy.used_count = s_satellite_published.used_count;
  memcpy(copy.entries, &s_satellite_published.entries[start], bytes);
  BSP_Critical_Exit(primask);
  if (copy.available == 0U) { return GNSS_RESULT_NO_DATA; }
  if (page != 0U && start >= copy.stored) { return GNSS_RESULT_INVALID_PARAM; }
  copy.page = page;
  remaining = (copy.stored > start) ? (uint16_t)(copy.stored - start) : 0U;
  copy.count = (uint8_t)((remaining > GNSS_SAT_PAGE_CAPACITY) ? GNSS_SAT_PAGE_CAPACITY : remaining);
  *out = copy;
  return GNSS_RESULT_OK;
}

/**
 * @brief 复制轻量 GNSS 设备事实状态，不进行超时策略判断。
 */
Gnss_Result_t Sensor_GNSS_GetStatus(Gnss_Status_t *out)
{
  uint32_t primask;

  if (out == NULL) { return GNSS_RESULT_INVALID_PARAM; }

  primask          = BSP_Critical_Enter();
  out->data_state  = s_snapshot.data_state;
  out->fix_valid   = s_snapshot.fix_valid;
  out->satellites  = s_snapshot.satellites;
  out->rx_sequence = s_snapshot.rx_sequence;
  out->last_rx_ms  = s_snapshot.last_rx_ms;
  out->last_fix_ms = s_snapshot.last_fix_ms;
  BSP_Critical_Exit(primask);

  return (out->rx_sequence == 0U) ? GNSS_RESULT_NO_DATA : GNSS_RESULT_OK;
}

/**
 * @brief 记录完整且校验通过的 NMEA 链路活动，不更新测量时间。
 */
static void Gnss_RecordValidSentence(uint32_t now)
{
  uint32_t primask = BSP_Critical_Enter();

  s_snapshot.last_rx_ms = now;
  s_snapshot.rx_sequence++;
  if (s_snapshot.rx_sequence == 0U) { s_snapshot.rx_sequence = 1U; }

  BSP_Critical_Exit(primask);
}

/**
 * @brief 消费 BSP 字节并组装完整 NMEA 句子，按句型更新所属字段。
 */
static void Gnss_FeedAndParse(uint32_t now)
{
  uint8_t temp[128];
  uint16_t available;
  uint16_t received;
  uint16_t i;
  uint16_t budget = GNSS_RX_BUDGET;
  uint32_t drop;

  while (budget && ((available = BSP_GNSS_GetRxCount()) > 0U)) {
    uint16_t request = (available > sizeof(temp)) ? (uint16_t)sizeof(temp) : available;
    if (request > budget) { request = budget; }
    drop = BSP_GNSS_GetDropCount();
    if (drop != s_link.observed_drop) { Gnss_ResetFraming(now, 1U); s_link.observed_drop = drop; }
    received = BSP_GNSS_GetRxData(temp, request);

    if (received == 0U) { break; }
    budget = (uint16_t)(budget - received);
    s_diag_work.rx_bytes += received; s_diag_work.last_rx_ms = now; s_diag_work.rx_seen = 1U;
    Gnss_MarkRxObservation(now);
    drop = BSP_GNSS_GetDropCount();
    if (drop != s_link.observed_drop) { Gnss_ResetFraming(now, 1U); s_link.observed_drop = drop; continue; }

    for (i = 0U; i < received; i++) {
      uint8_t ch = temp[i];
      uint8_t busy_before = Ubx_IsReceiving(&s_ubx);
      Ubx_Result_t ubx_result = Ubx_Feed(&s_ubx, ch);
      uint8_t busy_after = Ubx_IsReceiving(&s_ubx);
      if (!busy_before && busy_after) {
        s_ubx_candidate_bytes = 1U;
        s_ubx_candidate_started_ms = now; s_ubx_candidate_timed = 1U;
      }
      else if (busy_before && s_ubx_candidate_bytes != UINT32_MAX) { s_ubx_candidate_bytes++; }
      if (ubx_result == UBX_RESULT_FRAME || ubx_result == UBX_RESULT_CHECKSUM_ERROR) {
        Gnss_CaptureUbxRaw((uint8_t)(ubx_result == UBX_RESULT_FRAME), now);
      }
      if (ubx_result == UBX_RESULT_FRAME) {
        s_diag_work.valid_ubx++;
        Gnss_ProcessUbx(now);
      } else if (ubx_result == UBX_RESULT_CHECKSUM_ERROR) { s_diag_work.checksum_errors++; }
      else if (ubx_result == UBX_RESULT_OVERSIZE) {
        s_diag_work.framing_resets++;
        Gnss_RejectRaw(GNSS_PROTOCOL_UBX, (uint32_t)s_ubx.length + UBX_FRAME_OVERHEAD, GNSS_RAW_REJECT_OVERSIZE, now);
      }
      if (ubx_result != UBX_RESULT_NONE) {
        s_ubx_candidate_bytes = busy_after ? 1U : 0U;
        s_ubx_candidate_timed = busy_after;
        /* 坏CK_B若为B5，可成为下一候选首字节；不得沿用上一坏帧的寿命。 */
        if (busy_after) { s_ubx_candidate_started_ms = now; }
      }
      else if (busy_before && !busy_after) {
        Gnss_RejectRaw(GNSS_PROTOCOL_UBX, s_ubx_candidate_bytes, GNSS_RAW_REJECT_FRAMING, now);
        s_ubx_candidate_bytes = 0U; s_ubx_candidate_timed = 0U;
      }
      /* UBX内部的$和LF不能成为另一条NMEA；坏前导后新的$允许立即重新同步。 */
      if (busy_before || busy_after || (ubx_result != UBX_RESULT_NONE)) {
        if (s_line_pos) {
          Gnss_RejectRaw(GNSS_PROTOCOL_NMEA, s_line_pos, GNSS_RAW_REJECT_FRAMING, now);
          s_line_pos = 0U; s_diag_work.framing_resets++;
        }
        if (!(ch == '$' && !busy_after && ubx_result == UBX_RESULT_NONE)) { continue; }
      }

      if (ch == '$') {
        if (s_line_pos) { Gnss_RejectRaw(GNSS_PROTOCOL_NMEA, s_line_pos, GNSS_RAW_REJECT_FRAMING, now); }
        s_line_pos               = 0U;
        s_line_buf[s_line_pos++] = ch;
      } else if ((s_line_pos > 0U) && (s_line_pos < NMEA_SENTENCE_MAX_LEN)) {
        s_line_buf[s_line_pos++] = ch;

        if (ch == '\n') {
          Nmea_Sentence_t sentence;
          uint16_t cursor = 0U;

          s_line_buf[s_line_pos] = '\0';
          memset(&sentence,0,sizeof(sentence));
          (void)Nmea_ExtractSentence(s_line_buf,s_line_pos,&cursor,&sentence);
          Gnss_CaptureRaw(2U, s_line_buf, s_line_pos, GNSS_PROTOCOL_NMEA, 0U, 0U, sentence.checksum_ok, now);
          if (s_diagnostic_sentence) {
            uint8_t type=0U;
            if (s_line_pos>=6U) {
              if (!memcmp(&s_line_buf[3],"GGA",3U)) { type=1U; }
              else if (!memcmp(&s_line_buf[3],"RMC",3U)) { type=2U; }
              else if (!memcmp(&s_line_buf[3],"GSA",3U)) { type=3U; }
              else if (!memcmp(&s_line_buf[3],"GSV",3U)) { type=4U; }
            }
            s_diagnostic_sentence(s_line_buf,s_line_pos,type,sentence.checksum_ok,now);
          }
          if (sentence.raw_len && sentence.checksum_ok) {
            s_diag_work.valid_nmea++;
            Gnss_LinkFrame(GNSS_PROTOCOL_NMEA, now);
            Gnss_RecordValidSentence(now);
            /* 近期PVT保护基础测量；NMEA DOP/计数仍按独立来源和时刻发布。 */
            switch (sentence.type) {
            case NMEA_TYPE_GGA: Gnss_ParseGga(&sentence, now); break;
            case NMEA_TYPE_RMC:
              if (!Gnss_PvtFresh(now)) { Gnss_ParseRmc(&sentence, now); }
              break;
            case NMEA_TYPE_GSV: Gnss_ParseGsv(&sentence, now); break;
            case NMEA_TYPE_GSA: Gnss_ParseGsa(&sentence, now); break;
            default: break;
            }
          } else if (sentence.raw_len) { s_diag_work.checksum_errors++; }

          s_line_pos = 0U;
        }
      } else {
        if (s_line_pos >= NMEA_SENTENCE_MAX_LEN) {
          s_diag_work.framing_resets++;
          Gnss_RejectRaw(GNSS_PROTOCOL_NMEA, (uint32_t)s_line_pos + 1U, GNSS_RAW_REJECT_OVERSIZE, now);
        }
        if(s_line_pos>=NMEA_SENTENCE_MAX_LEN && s_diagnostic_sentence) {
          uint8_t type=0U;
          if(!memcmp(&s_line_buf[3],"GGA",3U)) { type=1U; }else if(!memcmp(&s_line_buf[3],"RMC",3U)) { type=2U; }
          else if(!memcmp(&s_line_buf[3],"GSA",3U)) { type=3U; }else if(!memcmp(&s_line_buf[3],"GSV",3U)) { type=4U; }
          /* NULL/0为超容量事件；不提交截断句或伪造分隔符。 */
          s_diagnostic_sentence(0,0U,type,0U,now);
        }
        s_line_pos = 0U;
      }
    }
  }
}

/**
 * @brief 丢弃不连续协议片段，不清历史typed测量或累计事实。
 * @details 仅存在半帧时缓存来源事件；已知owner时钟立即报告，无时钟Init延后到下一Service。
 */
static void Gnss_ResetFraming(uint32_t now, uint8_t report_now)
{
  if (s_line_pos != 0U) { s_pending_raw_reject[0] = s_line_pos; }
  if (Ubx_IsReceiving(&s_ubx) && s_ubx_candidate_bytes != 0U) { s_pending_raw_reject[1] = s_ubx_candidate_bytes; }
  memset(s_line_buf, 0, sizeof(s_line_buf)); s_line_pos = 0U;
  Ubx_Init(&s_ubx);
  s_ubx_candidate_bytes = 0U; s_ubx_candidate_started_ms = 0U; s_ubx_candidate_timed = 0U;
  if (s_driver_started) { s_diag_work.framing_resets++; }
  if (report_now) { Gnss_FlushRawReject(now); }
}

/**
 * @brief 丢弃超过绝对寿命的UBX半帧，在本轮新字节进入前恢复双协议同步。
 * @details codec仍完整消费未过期帧，绝不扫描其载荷内伪NMEA/UBX前导；
 *          后续字节不延长寿命，不能由畸形65535B长度持续吞掉正常输出。
 *          只撤销解析片段并报告一次RAW拒绝，不停DMA、不更新任何有效接收/测量时间。
 */
static void Gnss_ExpireUbxCandidate(uint32_t now)
{
  if (s_ubx_candidate_timed && Ubx_IsReceiving(&s_ubx) &&
      (uint32_t)(now - s_ubx_candidate_started_ms) >= GNSS_UBX_FRAME_TIMEOUT_MS) {
    Gnss_ResetFraming(now, 1U);
  }
}

/** @brief 逐字节读取UBX小端数，禁止非对齐指针转换。 */
static uint16_t Gnss_U16(const uint8_t *p)
{ return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

/** @brief 逐字节读取32位UBX小端无符号数。 */
static uint32_t Gnss_U32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

/** @brief 从线上补码恢复I4，避免超范围无符号到有符号转换的实现依赖。 */
static int32_t Gnss_I32(const uint8_t *p)
{
  uint32_t value = Gnss_U32(p);
  return (value & 0x80000000UL) ? (-1 - (int32_t)(~value)) : (int32_t)value;
}

/** @brief 逐字段写入U2，供命令构造使用。 */
static void Gnss_Put16(uint8_t *p, uint16_t value)
{ p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8); }

/** @brief 逐字段写入U4配置键，不发送C结构体内存。 */
static void Gnss_Put32(uint8_t *p, uint32_t value)
{ p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8); p[2] = (uint8_t)(value >> 16); p[3] = (uint8_t)(value >> 24); }

/** @brief 记录校验正确协议帧；无定位报文同样证明链路活跃。 */
static void Gnss_LinkFrame(uint8_t protocol, uint32_t now)
{
  s_diag_work.frame_seen = 1U; s_diag_work.last_frame_ms = now;
  if ((protocol == GNSS_PROTOCOL_UBX) || (s_diag_work.protocol != GNSS_PROTOCOL_UBX)) { s_diag_work.protocol = protocol; }
  if (s_diag_work.state == GNSS_LINK_SCAN) {
    s_diag_work.state = GNSS_LINK_VERSION;
    s_link.stage_ms = now; s_link.stage_seen = 1U; s_link.probes = 0U; s_link.tx_seen = 0U;
  }
}

/** @brief 校验PVT日期/时间可被现有yyMMdd/秒类型表示，闰秒60不伪写为59。 */
static uint8_t Gnss_PvtUtc(const uint8_t *p, uint32_t *date, uint32_t *seconds)
{
  uint16_t year = Gnss_U16(p + 4);
  uint8_t month = p[6], day = p[7];
  uint8_t limit;
  static const uint8_t days[12] = {31U,28U,31U,30U,31U,30U,31U,31U,30U,31U,30U,31U};
  if ((p[11] & 3U) != 3U || year < 2000U || year > 2099U || !month || month > 12U || p[8] >= 24U || p[9] >= 60U || p[10] >= 60U) { return 0U; }
  limit = days[month - 1U]; if (month == 2U && year % 4U == 0U) { limit++; }
  if (!day || day > limit) { return 0U; }
  *date = (uint32_t)(year % 100U) * 10000U + (uint32_t)month * 100U + day;
  *seconds = (uint32_t)p[8] * 3600U + (uint32_t)p[9] * 60U + p[10];
  return 1U;
}

/**
 * @brief 转换NAV-PVT；经纬为1e-7度，hMSL保留mm，速度mm/s转cm/s，航迹1e-5转0.01度。
 * @details fix_quality保持GGA质量码，绝不把UBX二维fixType=2误当差分质量码2；pDOP不冒充HDOP。
 *          fixType为定义范围0..5时独立刷新状态及numSV；无fix也保留真实卫星数，不授权位置。
 */
static void Gnss_ParsePvt(uint32_t now)
{
  const uint8_t *p = s_ubx.payload;
  uint8_t kind = p[20], flags = p[21], position_ok, velocity_ok, altitude_ok, time_ok, quality = 0U;
  uint8_t status_ok = (uint8_t)(kind <= 5U);
  int32_t lat = Gnss_I32(p + 28), lon = Gnss_I32(p + 24), alt = Gnss_I32(p + 36);
  int32_t speed = Gnss_I32(p + 60), heading = Gnss_I32(p + 64);
  uint32_t date = 0U, seconds = 0U, primask;
  uint32_t itow = Gnss_U32(p), course = 0U;
  uint16_t hdop = 0U;
  uint16_t reported_pdop = Gnss_U16(p + 76);
  /* 旧84B以及旧协议92B的78字段/flags高位为保留位，必须按已识别协议门控。 */
  position_ok = (uint8_t)((flags & 1U) && (kind >= 2U && kind <= 4U) &&
               !(s_ubx.length == 92U && s_diag_work.protocol_version_x100 >= 2700U && (p[78] & 1U)) &&
               lat >= -900000000L && lat <= 900000000L && lon >= -1800000000L && lon <= 1800000000L);
  altitude_ok = (uint8_t)(position_ok && kind != 2U && alt >= -100000000L && alt <= 100000000L);
  velocity_ok = (uint8_t)(position_ok && speed >= 0 && speed <= 2000000L && heading >= -36000000L && heading <= 72000000L);
  time_ok = Gnss_PvtUtc(p, &date, &seconds);
  if (position_ok) {
    uint8_t carrier = (s_ubx.length == 92U && s_diag_work.protocol_version_x100 >= 2000U) ? (uint8_t)((flags >> 6) & 3U) : 0U;
    quality = (carrier == 2U) ? 4U : (carrier == 1U) ? 5U : (flags & 2U) ? 2U : 1U;
  }
  if (velocity_ok) { int32_t normal = heading % 36000000L; if (normal < 0) { normal += 36000000L; } course = (uint32_t)normal / 1000U; }
  if (s_link.dop_seen && s_link.dop_itow == itow && (uint32_t)(now - s_link.dop_ms) <= GNSS_PVT_FRESH_MS) { hdop = s_link.hdop; }
  s_link.pvt_seen = 1U; s_link.pvt_ms = now; s_link.pvt_itow = itow; s_link.pvt_count++; s_link.pvt_position_ok = position_ok;
  primask = BSP_Critical_Enter();
  s_snapshot.valid_mask &= ~(GNSS_VALID_POSITION | GNSS_VALID_VELOCITY | GNSS_VALID_ALTITUDE | GNSS_VALID_TIME | GNSS_VALID_NAV_STATUS);
  s_snapshot.fix_valid = position_ok; s_snapshot.fix_quality = quality;
  s_snapshot.fix_dimension = position_ok ? ((kind == 2U) ? 2U : 3U) : 0U;
  if (status_ok) { s_snapshot.dimension_source = GNSS_SOURCE_UBX_PVT; s_snapshot.dimension_update_ms = now; }
  else { s_snapshot.dimension_source = GNSS_SOURCE_NONE; }
  if (!(s_snapshot.dop_source[GNSS_DOP_P] == GNSS_SOURCE_UBX_DOP &&
        (s_snapshot.dop_present_mask & (1U << GNSS_DOP_P)) && (s_snapshot.dop_itow_known_mask & (1U << GNSS_DOP_P)) &&
        s_snapshot.dop_itow_ms[GNSS_DOP_P] == itow && (uint32_t)(now - s_snapshot.dop_update_ms[GNSS_DOP_P]) <= GNSS_PVT_FRESH_MS)) {
    Gnss_SetDop(GNSS_DOP_P, reported_pdop, GNSS_SOURCE_UBX_PVT, now, 1U, itow, position_ok);
  }
  Gnss_PairDopSolution(itow, position_ok, now);
  if (status_ok) {
    s_snapshot.satellites = p[23];
    s_snapshot.status_update_ms = now;
    s_snapshot.status_source = GNSS_SOURCE_UBX_PVT; s_snapshot.raw_status_type = kind;
    s_snapshot.valid_mask |= GNSS_VALID_NAV_STATUS;
  }
  s_snapshot.hdop_cm = hdop;
  if (position_ok) {
    s_snapshot.latitude_deg_e7 = lat; s_snapshot.longitude_deg_e7 = lon;
    s_snapshot.position_update_ms = now; s_snapshot.last_fix_ms = now; s_snapshot.valid_mask |= GNSS_VALID_POSITION;
    s_snapshot.position_source = GNSS_SOURCE_UBX_PVT;
  }
  if (altitude_ok) { s_snapshot.altitude_mm = alt; s_snapshot.altitude_update_ms = now; s_snapshot.altitude_source = GNSS_SOURCE_UBX_PVT; s_snapshot.valid_mask |= GNSS_VALID_ALTITUDE; }
  if (velocity_ok) { s_snapshot.speed_cms = (uint32_t)speed / 10U; s_snapshot.heading_deg100 = course; s_snapshot.velocity_update_ms = now; s_snapshot.velocity_source = GNSS_SOURCE_UBX_PVT; s_snapshot.valid_mask |= GNSS_VALID_VELOCITY; }
  if (time_ok) { s_snapshot.utc_date = date; s_snapshot.utc_sec = seconds; s_snapshot.time_update_ms = now; s_snapshot.time_source = GNSS_SOURCE_UBX_PVT; s_snapshot.valid_mask |= GNSS_VALID_TIME; }
  BSP_Critical_Exit(primask);
}

/** @brief 在固定30B扩展块内读取PROTVER，禁止无界字符串扫描。 */
static uint16_t Gnss_ParseProtocolVersion(const uint8_t *p)
{
  uint8_t i = 8U, digits = 0U;
  uint16_t major = 0U, minor = 0U;
  if (memcmp(p, "PROTVER=", 8U) != 0) { return 0U; }
  while (i < 30U && p[i] >= '0' && p[i] <= '9' && digits < 2U) { major = (uint16_t)(major * 10U + p[i++] - '0'); digits++; }
  if (!digits || i >= 30U || p[i++] != '.') { return 0U; }
  digits = 0U;
  while (i < 30U && p[i] >= '0' && p[i] <= '9' && digits < 2U) { minor = (uint16_t)(minor * 10U + p[i++] - '0'); digits++; }
  if (!digits || !major || (i < 30U && p[i] != 0U && p[i] != ' ')) { return 0U; }
  if (digits == 1U) { minor *= 10U; }
  return (uint16_t)(major * 100U + minor);
}

/** @brief 标记主动配置未确认，继续原始接收；稍后重新查询而非冒充初始化成功。 */
static void Gnss_Unconfirmed(uint32_t now)
{
  s_diag_work.state = GNSS_LINK_UNCONFIRMED; s_diag_work.config_confirmed = 0U;
  s_link.awaiting_ack = 0U; s_link.waiting_nav = 0U; s_link.stage_ms = now; s_link.stage_seen = 1U;
}

/** @brief MON-VER基础字段必须是有界、非空且NUL结尾的可打印版本文本。 */
static uint8_t Gnss_VersionText(const uint8_t *p, uint16_t length)
{
  uint16_t i;
  if (!length || !p[0]) { return 0U; }
  for (i = 0U; i < length; i++) {
    if (!p[i]) { return 1U; }
    if (p[i] < 0x20U || p[i] > 0x7EU) { return 0U; }
  }
  return 0U;
}

/**
 * @brief 在sensor上下文解析结构有效的NAV-SAT，并一次发布独立卫星表。
 * @param[in] now 本机消费时刻，单位ms；不使用模块iTOW充当本机时钟。
 * @return 结构和版本合法返回1，否则不改旧表、返回0。
 * @details 已知可见/跟踪列表不等于参与定位列表。只按原bit3统计used；不借卫星表
 *          更新主导航位置、UTC、DOP或总使用卫星数，不在临界区解析或遍历条目。
 */
static uint8_t Gnss_ParseSatellites(uint32_t now)
{
  const uint8_t *payload = s_ubx.payload;
  uint8_t total;
  uint16_t i;
  uint32_t primask;
  if (s_ubx.length < 8U || payload[4] != 1U) { return 0U; }
  total = payload[5];
  if ((uint32_t)s_ubx.length != 8UL + 12UL * total) { return 0U; }
  memset(&s_satellite_work, 0, sizeof(s_satellite_work));
  s_satellite_work.sequence = s_satellite_published.sequence + 1U;
  s_satellite_work.received_ms = now;
  s_satellite_work.itow_ms = Gnss_U32(payload);
  s_satellite_work.total = total;
  s_satellite_work.stored = (total > GNSS_SAT_CAPACITY) ? GNSS_SAT_CAPACITY : total;
  s_satellite_work.truncated = (uint8_t)(total > GNSS_SAT_CAPACITY);
  s_satellite_work.available = 1U;
  for (i = 0U; i < total; ++i) {
    const uint8_t *record = payload + 8U + 12U * i;
    uint32_t flags = Gnss_U32(record + 8U);
    uint8_t used = (uint8_t)((flags >> 3U) & 1U);
    s_satellite_work.used_count = (uint8_t)(s_satellite_work.used_count + used);
    if (i < GNSS_SAT_CAPACITY) {
      Gnss_Satellite_t *sat = &s_satellite_work.entries[i];
      int16_t elevation = (record[3] < 128U) ? (int16_t)record[3] : (int16_t)((int16_t)record[3] - 256);
      uint16_t azimuth_raw = Gnss_U16(record + 4U);
      int32_t azimuth = (azimuth_raw & 0x8000U) ? (int32_t)azimuth_raw - 65536L : (int32_t)azimuth_raw;
      sat->gnss_id = record[0]; sat->sv_id = record[1]; sat->cno_dbhz = record[2];
      sat->quality = (uint8_t)(flags & 7U); sat->used = used;
      sat->elevation_deg = (int8_t)elevation; sat->azimuth_deg = (int16_t)azimuth;
      if (elevation >= -90 && elevation <= 90) {
        sat->valid_mask |= GNSS_SAT_VALID_ELEVATION;
        if (azimuth >= 0 && azimuth <= 360) { sat->valid_mask |= GNSS_SAT_VALID_AZIMUTH; }
      }
    }
  }
  primask = BSP_Critical_Enter();
  s_satellite_published = s_satellite_work;
  BSP_Critical_Exit(primask);
  return 1U;
}

/** @brief 消费校验完成帧，ACK必须匹配当前命令，导航、UTC与逐星事实独立更新。 */
static void Gnss_ProcessUbx(uint32_t now)
{
  const uint8_t *p = s_ubx.payload;
  uint16_t length = s_ubx.length;
  uint8_t recognized = 0U;
  if (s_ubx.class_id == 0x01U && s_ubx.message_id == 0x07U && (length == 92U || length == 84U)) {
    Gnss_ParsePvt(now); recognized = 1U;
  } else if (s_ubx.class_id == 0x01U && s_ubx.message_id == 0x04U && length == 18U) {
    uint16_t value = Gnss_U16(p + 12);
    uint16_t dop[GNSS_DOP_COUNT];
    uint8_t i;
    uint8_t solution_ok;
    uint32_t primask;
    for (i = 0U; i < GNSS_DOP_COUNT; ++i) { dop[i] = Gnss_U16(p + 4U + 2U * i); }
    s_link.dop_seen = 1U; s_link.dop_ms = now; s_link.dop_itow = Gnss_U32(p); s_link.hdop = (value == UINT16_MAX) ? 0U : value;
    solution_ok = (uint8_t)(Gnss_PvtFresh(now) && s_link.pvt_position_ok && s_link.pvt_itow == s_link.dop_itow);
    primask = BSP_Critical_Enter();
    for (i = 0U; i < GNSS_DOP_COUNT; ++i) { Gnss_SetDop(i, dop[i], GNSS_SOURCE_UBX_DOP, now, 1U, s_link.dop_itow, solution_ok); }
    BSP_Critical_Exit(primask);
    if (s_link.pvt_seen && s_link.pvt_itow == s_link.dop_itow && (uint32_t)(now - s_link.pvt_ms) <= GNSS_PVT_FRESH_MS) {
      primask = BSP_Critical_Enter(); s_snapshot.hdop_cm = s_link.hdop; BSP_Critical_Exit(primask);
    }
    recognized = 1U;
  } else if (s_ubx.class_id == 0x01U && s_ubx.message_id == 0x35U) {
    recognized = Gnss_ParseSatellites(now);
  } else if (s_ubx.class_id == 0x0AU && s_ubx.message_id == 0x04U && length >= 40U && (length - 40U) % 30U == 0U &&
             Gnss_VersionText(p, 30U) && Gnss_VersionText(p + 30, 10U)) {
    uint16_t offset, version = 0U;
    for (offset = 40U; offset + 30U <= length; offset = (uint16_t)(offset + 30U)) {
      uint16_t found = Gnss_ParseProtocolVersion(p + offset);
      if (found) { version = found; }
    }
    s_diag_work.version_seen = 1U; s_diag_work.protocol_version_x100 = version; recognized = 1U;
  } else if (s_ubx.class_id == 0x05U && s_ubx.message_id <= 1U && length == 2U) {
    recognized = 1U;
    if (s_diag_work.state == GNSS_LINK_CONFIGURE && s_link.awaiting_ack && p[0] == 0x06U && p[1] == s_link.expected_id) {
      s_link.awaiting_ack = 0U;
      if (s_ubx.message_id == 0U) { s_diag_work.nak_count++; Gnss_Unconfirmed(now); }
      else {
        s_diag_work.ack_count++; s_diag_work.config_step++; s_link.attempts = 0U;
        if (s_diag_work.config_step == s_diag_work.config_total) {
          s_link.waiting_nav = 1U; s_link.confirm_pvt_count = s_link.pvt_count; s_link.stage_ms = now;
        }
      }
    }
  }
  if (recognized) { Gnss_LinkFrame(GNSS_PROTOCOL_UBX, now); Gnss_RecordValidSentence(now); }
}

/** @brief 将已编码协议命令复制交给非阻塞BSP；BUSY不会消耗重试额度。 */
static BSP_Status_t Gnss_SendCommand(uint8_t message_class, uint8_t message_id, const uint8_t *payload, uint16_t length, uint32_t now)
{
  uint16_t size = Ubx_BuildFrame(message_class, message_id, payload, length, s_tx_command, sizeof(s_tx_command));
  BSP_Status_t result;
  if (!size) { return BSP_STATUS_ERROR; }
  result = BSP_GNSS_Transmit(s_tx_command, size);
  if (result == BSP_STATUS_OK) {
    s_diag_work.tx_commands++; s_link.last_tx_ms = now; s_link.tx_seen = 1U;
    s_link.tx_watchdog_ms = now; s_link.tx_watchdog_seen = 1U;
    if (message_class == 0x0AU && message_id == 0x04U) { s_link.probe_timeout_marked = 0U; }
  }
  return result;
}

/** @brief 构造PVT/DOP/SAT及1Hz导航四项课堂配置；仅RAM，不保存Flash，不关闭NMEA。 */
static BSP_Status_t Gnss_SendConfiguration(uint32_t now)
{
  uint8_t payload[24];
  uint16_t length, rate_ms = 1000U;
  uint8_t step = s_diag_work.config_step;
  memset(payload, 0, sizeof(payload));
  if (s_diag_work.protocol_version_x100 >= 2700U) {
    payload[1] = 1U; /* CFG-VALSET layers.bit0=RAM；BBR/Flash位不置。 */
    s_link.expected_id = 0x8AU;
    if (step < 3U) {
      uint32_t key = (step == 0U) ? 0x20910007UL : ((step == 1U) ? 0x20910039UL : 0x20910016UL);
      Gnss_Put32(payload + 4, key);
      payload[8] = 1U; length = 9U;
    } else {
      Gnss_Put32(payload + 4, 0x30210001UL); Gnss_Put16(payload + 8, rate_ms);
      Gnss_Put32(payload + 10, 0x30210002UL); Gnss_Put16(payload + 14, 1U);
      Gnss_Put32(payload + 16, 0x20210003UL); payload[20] = 1U; length = 21U;
    }
  } else {
    if (step < 3U) {
      s_link.expected_id = 0x01U; payload[0] = 0x01U;
      payload[1] = (step == 0U) ? 0x07U : ((step == 1U) ? 0x04U : 0x35U);
      payload[3] = 1U; length = 8U; /* CFG-MSG UART1字段；不改变其它输出协议。 */
    } else {
      s_link.expected_id = 0x08U; Gnss_Put16(payload, rate_ms); Gnss_Put16(payload + 2, 1U); Gnss_Put16(payload + 4, 1U); length = 6U;
    }
  }
  return Gnss_SendCommand(0x06U, s_link.expected_id, payload, length, now);
}

/**
 * @brief 固定速率纠正或有效帧超时后重新探测协议，保留累计诊断。
 * @details 清除不连续的协议片段；正常断流只重启协议探测，不停DMA或切换到其它速率。
 */
static void Gnss_StartDetection(uint32_t now)
{
  s_diag_work.state = GNSS_LINK_SCAN; s_diag_work.protocol = GNSS_PROTOCOL_UNKNOWN;
  s_diag_work.version_seen = 0U; s_diag_work.protocol_version_x100 = 0U;
  s_diag_work.config_confirmed = 0U; s_diag_work.config_step = 0U; s_diag_work.config_total = 0U;
  s_link.stage_ms = now; s_link.stage_seen = 1U; s_link.probes = 0U; s_link.tx_seen = 0U;
  s_link.awaiting_ack = 0U; s_link.waiting_nav = 0U; s_link.pvt_seen = 0U; s_link.dop_seen = 0U;
  Gnss_ResetFraming(now, 1U);
}

/**
 * @brief 发送故障只终止TX并退回未确认状态，不牵连仍工作的RX DMA或解析片段。
 * @param now 当前sensor服务真实毫秒。
 * @param timed_out 1表示发送完成看门狗超时，累计一次timeout事实；0为提交失败。
 * @details BSP保留TX失败计数；只有TX中止失败或RX硬件已异常才请求原接收恢复。
 *          迟到ACK因awaiting_ack清零不会推进旧配置，重试沿用10秒退避。
 */
static void Gnss_HandleTxFault(uint32_t now,uint8_t timed_out)
{
  BSP_Status_t result=BSP_GNSS_AbortTx();
  if(timed_out) { s_diag_work.timeout_count++; }
  s_link.tx_watchdog_seen=0U;s_link.tx_seen=0U;
  if(result==BSP_STATUS_OK) { Gnss_Unconfirmed(now); }
  else {
    s_diag_work.state=GNSS_LINK_IO_ERROR;BSP_GNSS_RequestRecoverRx();
  }
}

/**
 * @brief 每10ms由sensor推进少量接入步骤；不sleep、不等待串口完成、不新建任务。
 * @details 固定115200bit/s；有效NMEA可被动工作，仅合法MON-VER与PROTVER之后允许写配置。
 *          无有效数据时每轮最多查询三次，距第三次发送满10秒后再查，不为探测反复停DMA。
 *          同class/id步骤之间至少隔1200ms，以排空前一步迟到ACK；最终仍需后续PVT确认。
 */
static void Gnss_LinkStep(uint32_t now)
{
  BSP_GNSS_TransportInfo_t transport;
  BSP_Status_t io;
  if (BSP_GNSS_CopyTransportInfo(&transport) != BSP_STATUS_OK || !transport.initialized) {
    s_diag_work.state = GNSS_LINK_IO_ERROR; BSP_GNSS_RequestRecoverRx(); return;
  }
  s_diag_work.baud_bps = transport.baud_bps;
  if (!transport.tx_busy) { s_link.tx_watchdog_seen = 0U; }
  if (transport.tx_busy && s_link.tx_watchdog_seen && (uint32_t)(now - s_link.tx_watchdog_ms) >= GNSS_RECOVER_RETRY_MS) {
    Gnss_HandleTxFault(now,1U);return;
  }
  if (transport.baud_bps != GNSS_FIXED_BAUD_BPS) {
    io = BSP_GNSS_SetBaudRate(GNSS_FIXED_BAUD_BPS);
    if (io == BSP_STATUS_OK) {
      s_diag_work.baud_bps = GNSS_FIXED_BAUD_BPS; s_diag_work.baud_changes++;
      Gnss_StartDetection(now);
    } else if (io != BSP_STATUS_BUSY) { s_diag_work.state = GNSS_LINK_IO_ERROR; BSP_GNSS_RequestRecoverRx(); }
    return;
  }
  if (!s_link.stage_seen) { s_link.stage_seen = 1U; s_link.stage_ms = now; }
  /* 无帧重探测不能绕过发送故障/配置未确认的10秒退避，避免异常时每拍重发。 */
  if (s_diag_work.state != GNSS_LINK_SCAN && s_diag_work.frame_seen &&
      (uint32_t)(now - s_diag_work.last_frame_ms) > GNSS_LINK_LOSS_MS &&
      (s_diag_work.state != GNSS_LINK_UNCONFIRMED ||
       (uint32_t)(now-s_link.stage_ms)>=GNSS_CONFIG_RETRY_MS)) { Gnss_StartDetection(now); }

  if (s_diag_work.state == GNSS_LINK_SCAN) {
    if (s_link.probes >= 3U) {
      if (s_link.tx_seen && (uint32_t)(now - s_link.last_tx_ms) < GNSS_CONFIG_RETRY_MS) { return; }
      s_link.probes = 0U; s_link.tx_seen = 0U;
    }
    if (s_link.probes < 3U && (!s_link.tx_seen || (uint32_t)(now - s_link.last_tx_ms) >= GNSS_PROBE_INTERVAL_MS)) {
      io = Gnss_SendCommand(0x0AU, 0x04U, NULL, 0U, now);
      if (io == BSP_STATUS_OK) { s_link.probes++; }
      else if (io != BSP_STATUS_BUSY) { Gnss_HandleTxFault(now,0U); }
    }
    return;
  }

  if (s_diag_work.state == GNSS_LINK_VERSION) {
    if (s_diag_work.version_seen) {
      if (!s_diag_work.protocol_version_x100) { Gnss_Unconfirmed(now); return; }
      s_diag_work.state = GNSS_LINK_CONFIGURE; s_diag_work.config_total = 4U; s_diag_work.config_step = 0U;
      s_diag_work.config_confirmed = 0U; s_link.awaiting_ack = 0U; s_link.waiting_nav = 0U; s_link.attempts = 0U;
      s_link.tx_seen = 0U; s_link.stage_ms = now;
    } else {
      if (s_link.tx_seen && (uint32_t)(now - s_link.last_tx_ms) < GNSS_ACK_WAIT_MS) { return; }
      if (s_link.tx_seen && !s_link.probe_timeout_marked) { s_diag_work.timeout_count++; s_link.probe_timeout_marked = 1U; }
      if (s_link.probes >= 3U) {
        if (s_diag_work.protocol == GNSS_PROTOCOL_NMEA) { s_diag_work.state = GNSS_LINK_RECEIVING; s_link.stage_ms = now; }
        else { Gnss_Unconfirmed(now); }
        return;
      }
      io = Gnss_SendCommand(0x0AU, 0x04U, NULL, 0U, now);
      if (io == BSP_STATUS_OK) { s_link.probes++; }
      else if (io != BSP_STATUS_BUSY) { Gnss_HandleTxFault(now,0U); }
      return;
    }
  }

  if (s_diag_work.state == GNSS_LINK_CONFIGURE) {
    if (s_link.waiting_nav) {
      if (s_link.pvt_count != s_link.confirm_pvt_count) {
        s_diag_work.config_confirmed = 1U; s_diag_work.state = GNSS_LINK_RECEIVING; s_link.waiting_nav = 0U;
      } else if ((uint32_t)(now - s_link.stage_ms) >= GNSS_LINK_LOSS_MS) { s_diag_work.timeout_count++; Gnss_Unconfirmed(now); }
      return;
    }
    if (s_link.awaiting_ack) {
      if ((uint32_t)(now - s_link.last_tx_ms) < GNSS_ACK_WAIT_MS) { return; }
      s_diag_work.timeout_count++; s_link.awaiting_ack = 0U;
      if (s_link.attempts >= 3U) { Gnss_Unconfirmed(now); return; }
    }
    if (s_link.tx_seen && (uint32_t)(now - s_link.last_tx_ms) < GNSS_ACK_WAIT_MS) { return; }
    io = Gnss_SendConfiguration(now);
    if (io == BSP_STATUS_OK) { s_link.awaiting_ack = 1U; s_link.attempts++; }
    else if (io != BSP_STATUS_BUSY) { Gnss_HandleTxFault(now,0U); }
    return;
  }
  if ((s_diag_work.state == GNSS_LINK_UNCONFIRMED || (s_diag_work.state == GNSS_LINK_RECEIVING && !s_diag_work.version_seen)) &&
      (uint32_t)(now - s_link.stage_ms) >= GNSS_CONFIG_RETRY_MS) {
    s_diag_work.state = GNSS_LINK_VERSION; s_diag_work.version_seen = 0U; s_diag_work.protocol_version_x100 = 0U;
    s_link.probes = 0U; s_link.tx_seen = 0U; s_link.stage_ms = now;
  }
}
