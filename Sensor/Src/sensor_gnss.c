/**
 * @file sensor_gnss.c
 * @brief Sensor 层拆分 NMEA 链路活动与各测量更新事实。
 * @details 依赖 BSP 字节接口与 NMEA 解析器；不调用 HAL、不判定 OFFLINE、不发布 Framework topic。
 */

#include "sensor_gnss.h"

#include "bsp_critical.h"
#include "bsp_gnss.h"
#include "sensor_nmea.h"


#include <string.h>
#include <stdlib.h>

#define GNSS_FIX_MIN_SATS 3U
#define GNSS_RECOVER_RETRY_MS 100U

static Gnss_Snapshot_t s_snapshot;
static uint8_t s_line_buf[NMEA_SENTENCE_MAX_LEN];
static uint16_t s_line_pos;

static uint8_t s_recover_rx_pending;
static uint32_t s_recover_rx_last_ms;
static volatile uint8_t s_reinit_request;

static void Gnss_FeedAndParse(uint32_t now);
static void Gnss_UpdateState(void);
static void Gnss_ParseGga(const Nmea_Sentence_t *sentence, uint32_t now);
static void Gnss_ParseRmc(const Nmea_Sentence_t *sentence, uint32_t now);
static void Gnss_ParseGsv(const Nmea_Sentence_t *sentence);
static void Gnss_ParseGsa(const Nmea_Sentence_t *sentence);

static uint8_t Gnss_CountGsaSatellites(const Nmea_Sentence_t *sentence);

/** @brief 严格检查有界十进制字段；空字段、尾随字符、NaN 和超范围均不产生测量。 */
static uint8_t Gnss_ReadDecimal(const Nmea_Sentence_t *sentence, uint8_t index, double minimum, double maximum, double *value)
{
  char field[32];
  char *end;
  uint8_t i = 0U;
  uint8_t digits = 0U;
  uint8_t points = 0U;
  if (Nmea_GetField(sentence->raw, index, field, sizeof(field)) == 0U) { return 0U; }
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
      (Nmea_GetField(sentence->raw, (uint8_t)(index + 1U), hemisphere, sizeof(hemisphere)) == 0U)) { return 0U; }
  if ((hemisphere[1] != '\0') || ((latitude != 0U) ? ((hemisphere[0] != 'N') && (hemisphere[0] != 'S')) : ((hemisphere[0] != 'E') && (hemisphere[0] != 'W')))) { return 0U; }
  degree = (uint32_t)(value / 100.0);
  if ((value - (double)degree * 100.0) >= 60.0) { return 0U; }
  *coordinate = (int32_t)((double)degree * 10000000.0 + (value - (double)degree * 100.0) * (10000000.0 / 60.0));
  if ((hemisphere[0] == 'S') || (hemisphere[0] == 'W')) { *coordinate = -*coordinate; }
  return 1U;
}

/** @brief GGA 只更新位置、定位质量和海拔；不续期 RMC 速度或完整 UTC。 */
static void Gnss_ParseGga(const Nmea_Sentence_t *sentence, uint32_t now)
{
  Nmea_GeoData_t geo;
  double quality, satellites, altitude, hdop;
  uint8_t position_ok;
  uint8_t altitude_ok;
  int32_t altitude_mm = 0;
  uint32_t primask;
  memset(&geo, 0, sizeof(geo));
  position_ok = (uint8_t)((Gnss_CoordinateValid(sentence, 2U, 1U, &geo.latitude_deg_e7) != 0U) &&
    (Gnss_CoordinateValid(sentence, 4U, 0U, &geo.longitude_deg_e7) != 0U) &&
    (Gnss_ReadDecimal(sentence, 6U, 1.0, 8.0, &quality) != 0U) &&
    (Gnss_ReadDecimal(sentence, 7U, (double)GNSS_FIX_MIN_SATS, 255.0, &satellites) != 0U));
  altitude_ok = (uint8_t)((position_ok != 0U) && (Gnss_ReadDecimal(sentence, 9U, -100000.0, 100000.0, &altitude) != 0U));
  if (position_ok != 0U) {
    geo.fix_quality = (uint8_t)quality;
    geo.satellites = (uint8_t)satellites;
    if (Gnss_ReadDecimal(sentence, 8U, 0.0, 655.35, &hdop) != 0U) { geo.hdop_cm = (uint16_t)(hdop * 100.0); }
  }
  if (altitude_ok != 0U) { altitude_mm = (int32_t)(altitude * 1000.0); }
  primask = BSP_Critical_Enter();
  s_snapshot.valid_mask &= ~(GNSS_VALID_POSITION | GNSS_VALID_ALTITUDE);
  s_snapshot.fix_valid = position_ok;
  s_snapshot.fix_quality = (position_ok != 0U) ? geo.fix_quality : 0U;
  s_snapshot.satellites = (position_ok != 0U) ? geo.satellites : 0U;
  if (position_ok != 0U) {
    s_snapshot.latitude_deg_e7 = geo.latitude_deg_e7;
    s_snapshot.longitude_deg_e7 = geo.longitude_deg_e7;
    s_snapshot.hdop_cm = geo.hdop_cm;
    s_snapshot.position_update_ms = now;
    s_snapshot.last_fix_ms = now;
    s_snapshot.valid_mask |= GNSS_VALID_POSITION;
  } else {
    s_snapshot.fix_dimension = 0U;
    s_snapshot.valid_mask &= ~GNSS_VALID_VELOCITY;
  }
  if (altitude_ok != 0U) {
    s_snapshot.altitude_mm = altitude_mm;
    s_snapshot.altitude_update_ms = now;
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
  if ((Nmea_GetField(sentence->raw, 1U, time, sizeof(time)) == 0U) ||
      (Nmea_GetField(sentence->raw, 9U, day, sizeof(day)) == 0U)) { return 0U; }
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
  active_ok = (uint8_t)((Nmea_GetField(sentence->raw, 2U, active, sizeof(active)) != 0U) && (active[0] == 'A') && (active[1] == '\0'));
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
  }
  if (velocity_ok != 0U) {
    s_snapshot.speed_cms = speed_cms;
    s_snapshot.heading_deg100 = heading_deg100;
    s_snapshot.velocity_update_ms = now;
    s_snapshot.valid_mask |= GNSS_VALID_VELOCITY;
  }
  if (time_ok != 0U) {
    s_snapshot.utc_date = date; s_snapshot.utc_sec = seconds;
    s_snapshot.time_update_ms = now;
    s_snapshot.valid_mask |= GNSS_VALID_TIME;
  }
  BSP_Critical_Exit(primask);
}

/**
 * @brief 解析一个有界 NMEA 无符号字段为 8 位数值。
 */
static uint8_t Gnss_ParseUint8Field(const Nmea_Sentence_t *sentence, uint8_t field_index, uint8_t *value)
{
  char field[4];
  uint16_t result = 0U;
  uint16_t i;

  if ((sentence == NULL) || (value == NULL) || (Nmea_GetField(sentence->raw, field_index, field, sizeof(field)) == 0U)) { return 0U; }

  for (i = 0U; field[i] != '\0'; i++) {
    if ((field[i] < '0') || (field[i] > '9')) { return 0U; }

    result = (uint16_t)(result * 10U) + (uint16_t)(field[i] - '0');
  }

  if (result > 255U) { result = 255U; }

  *value = (uint8_t)result;
  return 1U;
}

/**
 * @brief 解析 GSV 可见卫星诊断信息，不推进测量时间。
 */
static void Gnss_ParseGsv(const Nmea_Sentence_t *sentence)
{
  uint8_t visible_satellites;
  uint32_t primask;

  if (Gnss_ParseUint8Field(sentence, 3U, &visible_satellites) == 0U) { return; }

  primask = BSP_Critical_Enter();

  if (memcmp(sentence->raw, "$GPGSV", 6U) == 0) {
    s_snapshot.gps_visible_sats = visible_satellites;
  } else if ((memcmp(sentence->raw, "$BDGSV", 6U) == 0) || (memcmp(sentence->raw, "$GBGSV", 6U) == 0)) {
    s_snapshot.bds_visible_sats = visible_satellites;
  }

  BSP_Critical_Exit(primask);
}

/**
 * @brief 统计 GSA 中非空的参与解算卫星编号。
 */
static uint8_t Gnss_CountGsaSatellites(const Nmea_Sentence_t *sentence)
{
  char field[8];
  uint8_t field_index;
  uint8_t count = 0U;

  /*
   * GSA fields 3 through 14 contain up to 12 satellites used in the fix.
   */
  for (field_index = 3U; field_index <= 14U; field_index++) {
    if (Nmea_GetField(sentence->raw, field_index, field, sizeof(field)) != 0U) { count++; }
  }

  return count;
}

/**
 * @brief 更新 GSA 定位维度和卫星诊断，不续期 GGA 坐标。
 */
static void Gnss_ParseGsa(const Nmea_Sentence_t *sentence)
{
  uint8_t used_satellites;
  uint8_t fix_dimension;
  uint32_t primask;

  if (sentence == NULL) { return; }

  used_satellites = Gnss_CountGsaSatellites(sentence);
  if (Gnss_ParseUint8Field(sentence, 2U, &fix_dimension) == 0U) { fix_dimension = 0U; }

  primask = BSP_Critical_Enter();

  if ((fix_dimension >= 1U) && (fix_dimension <= 3U)) { s_snapshot.fix_dimension = fix_dimension; }
  if (fix_dimension == 1U) {
    s_snapshot.fix_valid = 0U;
    s_snapshot.fix_quality = 0U;
    s_snapshot.valid_mask &= ~(GNSS_VALID_POSITION | GNSS_VALID_VELOCITY | GNSS_VALID_ALTITUDE);
  }

  if (memcmp(sentence->raw, "$GPGSA", 6U) == 0) {
    s_snapshot.gps_used_sats = used_satellites;
  } else if ((memcmp(sentence->raw, "$BDGSA", 6U) == 0) || (memcmp(sentence->raw, "$GBGSA", 6U) == 0)) {
    s_snapshot.bds_used_sats = used_satellites;
  }

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
  memset(&s_snapshot, 0, sizeof(s_snapshot));
  memset(s_line_buf, 0, sizeof(s_line_buf));

  s_line_pos            = 0U;
  s_snapshot.data_state = GNSS_DATA_NONE;
  s_recover_rx_pending  = 0U;
  s_recover_rx_last_ms  = 0U;
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

/** @brief sensor 唯一采集任务处理恢复请求和 NMEA，now_ms 为真实毫秒。 */
Gnss_Result_t Sensor_GNSS_Service(uint32_t now_ms)
{
  if (s_reinit_request != 0U) {
    s_reinit_request = 0U;
    (void)Sensor_GNSS_Init();
  }
  if (BSP_GNSS_ConsumeRecoverRxRequest() != 0U) { s_recover_rx_pending = 1U; }
  if ((s_recover_rx_pending != 0U) && ((s_recover_rx_last_ms == 0U) || ((uint32_t)(now_ms - s_recover_rx_last_ms) >= GNSS_RECOVER_RETRY_MS))) {
    s_recover_rx_last_ms = now_ms;
    if (BSP_GNSS_RecoverRx() != BSP_STATUS_OK) { return GNSS_RESULT_IO_ERROR; }
    s_recover_rx_pending = 0U;
  }
  BSP_GNSS_RxIdleCallback(0U);
  Gnss_FeedAndParse(now_ms);
  Gnss_UpdateState();
  return GNSS_RESULT_OK;
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

  while ((available = BSP_GNSS_GetRxCount()) > 0U) {
    received = BSP_GNSS_GetRxData(temp, (available > sizeof(temp)) ? (uint16_t)sizeof(temp) : available);

    if (received == 0U) { break; }

    for (i = 0U; i < received; i++) {
      uint8_t ch = temp[i];

      if (ch == '$') {
        s_line_pos               = 0U;
        s_line_buf[s_line_pos++] = ch;
      } else if ((s_line_pos > 0U) && (s_line_pos < (NMEA_SENTENCE_MAX_LEN - 1U))) {
        s_line_buf[s_line_pos++] = ch;

        if (ch == '\n') {
          Nmea_Sentence_t sentence;
          uint16_t cursor = 0U;

          s_line_buf[s_line_pos] = '\0';

          if ((Nmea_ExtractSentence(s_line_buf, s_line_pos, &cursor, &sentence) != 0U) && (sentence.checksum_ok != 0U)) {
            Gnss_RecordValidSentence(now);

            switch (sentence.type) {
              case NMEA_TYPE_GGA:
                Gnss_ParseGga(&sentence, now);
                break;
              case NMEA_TYPE_RMC:
                Gnss_ParseRmc(&sentence, now);
                break;

              case NMEA_TYPE_GSV:
                Gnss_ParseGsv(&sentence);
                break;

              case NMEA_TYPE_GSA:
                Gnss_ParseGsa(&sentence);
                break;

              default:
                break;
            }
          }

          s_line_pos = 0U;
        }
      } else {
        s_line_pos = 0U;
      }
    }
  }
}
