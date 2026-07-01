/**
 * @file sensor_gnss.c
 * @brief Parse GNSS input and publish a coherent driver snapshot.
 */

#include "sensor_gnss.h"

#include "bsp_critical.h"
#include "bsp_gnss.h"
#include "sensor_nmea.h"


#include <string.h>

#define GNSS_FIX_MIN_SATS 4U
#define GNSS_RECOVER_RETRY_MS 100U

static Nmea_GeoData_t s_geo;
static Gnss_Snapshot_t s_snapshot;
static uint8_t s_line_buf[NMEA_SENTENCE_MAX_LEN];
static uint16_t s_line_pos;

static uint8_t s_geo_seen;
static uint8_t s_recover_rx_pending;
static uint32_t s_recover_rx_last_ms;
static volatile uint8_t s_reinit_request;

static void Gnss_FeedAndParse(uint32_t now);
static void Gnss_UpdateState(void);
static void Gnss_UpdateSnapshotFromGeo(uint32_t now);
static void Gnss_ParseGsv(const Nmea_Sentence_t *sentence);
static void Gnss_ParseGsa(const Nmea_Sentence_t *sentence);

static uint8_t Gnss_IsFixUsable(void);
static uint8_t Gnss_CountGsaSatellites(const Nmea_Sentence_t *sentence);

/**
 * @brief Check whether the parsed GNSS fix is valid for navigation output.
 */
static uint8_t Gnss_IsFixUsable(void)
{
  return (uint8_t)((s_geo_seen != 0U) && (s_geo.fix_quality >= 1U) && (s_geo.satellites >= GNSS_FIX_MIN_SATS));
}

/**
 * @brief Update position, velocity, quality, and fix facts from parsed NMEA data.
 */
static void Gnss_UpdateSnapshotFromGeo(uint32_t now)
{
  s_snapshot.fix_quality = s_geo.fix_quality;
  if (s_geo.fix_quality == 0U) { s_snapshot.fix_dimension = 0U; }
  s_snapshot.satellites = s_geo.satellites;
  s_snapshot.hdop_cm    = s_geo.hdop_cm;
  s_snapshot.utc_sec    = s_geo.utc_sec;
  s_snapshot.utc_date   = s_geo.utc_date;

  s_snapshot.latitude_deg_e7  = s_geo.latitude_deg_e7;
  s_snapshot.longitude_deg_e7 = s_geo.longitude_deg_e7;
  s_snapshot.altitude_mm      = s_geo.altitude_mm;

  s_snapshot.speed_cms      = s_geo.speed_cms;
  s_snapshot.heading_deg100 = s_geo.heading_deg100;

  s_snapshot.fix_valid = Gnss_IsFixUsable();

  if (s_snapshot.fix_valid != 0U) { s_snapshot.last_fix_ms = now; }
}

/**
 * @brief Parse one bounded unsigned NMEA field into an 8-bit value.
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
 * @brief Parse GPS or BeiDou GSV metadata and update visible satellite counts.
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
 * @brief Count non-empty satellite identifiers in one GSA sentence.
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
 * @brief Parse GPS or BeiDou GSA data and update used satellite counts.
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

  if (memcmp(sentence->raw, "$GPGSA", 6U) == 0) {
    s_snapshot.gps_used_sats = used_satellites;
  } else if ((memcmp(sentence->raw, "$BDGSA", 6U) == 0) || (memcmp(sentence->raw, "$GBGSA", 6U) == 0)) {
    s_snapshot.bds_used_sats = used_satellites;
  }

  BSP_Critical_Exit(primask);
}

/**
 * @brief Update the GNSS data fact from receive and fix timestamps.
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
 * @brief Reset the typed GNSS parser and coherent snapshot state.
 */
Gnss_Result_t Sensor_GNSS_Init(void)
{
  memset(&s_geo, 0, sizeof(s_geo));
  memset(&s_snapshot, 0, sizeof(s_snapshot));
  memset(s_line_buf, 0, sizeof(s_line_buf));

  s_geo_seen            = 0U;
  s_line_pos            = 0U;
  s_snapshot.data_state = GNSS_DATA_NONE;
  s_recover_rx_pending  = 0U;
  s_recover_rx_last_ms  = 0U;
  return GNSS_RESULT_OK;
}

/**
 * @brief Service pending GNSS input; the caller supplies the cycle timestamp.
 */
void Sensor_GNSS_RequestReinit(void)
{
  s_reinit_request = 1U;
}

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
 * @brief Copy the latest coherent GNSS snapshot already produced by Service().
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
 * @brief Copy the compact GNSS status without the position payload.
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
 * @brief Record that one checksum-valid NMEA sentence was received.
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
 * @brief Consume BSP bytes, assemble complete NMEA sentences, and update the snapshot.
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
              case NMEA_TYPE_GGA: {
                uint32_t primask;

                Nmea_ParseGGA(&sentence, &s_geo);
                s_geo_seen = 1U;

                primask = BSP_Critical_Enter();
                Gnss_UpdateSnapshotFromGeo(now);
                BSP_Critical_Exit(primask);
                break;
              }

              case NMEA_TYPE_RMC: {
                uint32_t primask;

                Nmea_ParseRMC(&sentence, &s_geo);

                primask = BSP_Critical_Enter();
                Gnss_UpdateSnapshotFromGeo(now);
                BSP_Critical_Exit(primask);
                break;
              }

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
