/**
 * @file sensor_nmea.c
 * @brief Implement bounded NMEA field extraction and navigation parsing.
 */

#include "sensor_nmea.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief Validate the hexadecimal XOR checksum of one complete NMEA sentence.
 */
uint8_t Nmea_Checksum(const char *sentence, uint16_t len)
{
  uint8_t checksum = 0U;
  uint16_t i;

  if ((sentence == NULL) ||
      (len == 0U) ||
      (sentence[0] != '$')) {
    return 0U;
  }

  for (i = 1U; i < len; i++) {
    if ((sentence[i] == '*') ||
        (sentence[i] == '\r') ||
        (sentence[i] == '\n')) {
      break;
    }

    checksum ^= (uint8_t)sentence[i];
  }

  return checksum;
}

/**
 * @brief Identify the supported NMEA sentence type from its talker and message ID.
 */
static Nmea_Type_t Nmea_DetectType(const char *raw,
                                   uint16_t raw_len)
{
  if ((raw == NULL) ||
      (raw_len < 6U) ||
      (raw[0] != '$')) {
    return NMEA_TYPE_UNKNOWN;
  }

  /*
   * ��׼��ʽΪ $ttXXX��
   * tt  = talker ID������ GP/GN/BD/GB
   * XXX = GGA/RMC/GSV/GSA/VTG
   */
  if (memcmp(&raw[3], "GGA", 3U) == 0) {
    return NMEA_TYPE_GGA;
  }

  if (memcmp(&raw[3], "RMC", 3U) == 0) {
    return NMEA_TYPE_RMC;
  }

  if (memcmp(&raw[3], "GSV", 3U) == 0) {
    return NMEA_TYPE_GSV;
  }

  if (memcmp(&raw[3], "GSA", 3U) == 0) {
    return NMEA_TYPE_GSA;
  }

  if (memcmp(&raw[3], "VTG", 3U) == 0) {
    return NMEA_TYPE_VTG;
  }

  return NMEA_TYPE_UNKNOWN;
}

/**
 * @brief Extract one complete NMEA sentence from a byte buffer and advance the cursor.
 */
uint8_t Nmea_ExtractSentence(const uint8_t *buf,
                             uint16_t buf_len,
                             uint16_t *cursor,
                             Nmea_Sentence_t *out)
{
  const char *data = (const char *)buf;
  uint16_t start;
  uint16_t end;
  uint16_t i;
  char checksum_text[3];
  uint8_t checksum_calc;
  uint8_t checksum_recv;

  if ((buf == NULL) ||
      (out == NULL) ||
      (buf_len < 8U)) {
    return 0U;
  }

  for (start = 0U; start < buf_len; start++) {
    if (data[start] == '$') {
      break;
    }
  }

  if (start >= buf_len) {
    return 0U;
  }

  for (end = (uint16_t)(start + 1U);
       end < buf_len;
       end++) {
    if (data[end] == '\n') {
      break;
    }
  }

  if (end >= buf_len) {
    return 0U;
  }

  out->raw_len = (uint16_t)(end - start + 1U);

  if (out->raw_len >= NMEA_SENTENCE_MAX_LEN) {
    out->raw_len = NMEA_SENTENCE_MAX_LEN - 1U;
  }

  memcpy(out->raw, &data[start], out->raw_len);
  out->raw[out->raw_len] = '\0';

  checksum_calc = Nmea_Checksum(out->raw, out->raw_len);
  out->checksum_ok = 0U;

  for (i = 0U; i < out->raw_len; i++) {
    if (out->raw[i] == '*') {
      if ((uint16_t)(i + 2U) < out->raw_len) {
        checksum_text[0] = out->raw[i + 1U];
        checksum_text[1] = out->raw[i + 2U];
        checksum_text[2] = '\0';

        checksum_recv =
            (uint8_t)strtol(checksum_text, NULL, 16);

        out->checksum_ok =
            (checksum_calc == checksum_recv) ? 1U : 0U;
      }

      break;
    }
  }

  out->type = Nmea_DetectType(out->raw, out->raw_len);

  if (cursor != NULL) {
    *cursor = (uint16_t)(end + 1U);
  }

  return 1U;
}

/**
 * @brief Copy one comma-separated NMEA field into a caller buffer.
 */
uint8_t Nmea_GetField(const char *raw,
                      uint8_t index,
                      char *dst,
                      uint16_t dst_len)
{
  uint8_t field = 0U;
  uint16_t read_pos = 0U;
  uint16_t write_pos = 0U;

  if ((raw == NULL) ||
      (dst == NULL) ||
      (dst_len == 0U)) {
    return 0U;
  }

  while ((raw[read_pos] != '\0') &&
         (field < index)) {
    if (raw[read_pos] == ',') {
      field++;
    }

    read_pos++;
  }

  while ((raw[read_pos] != '\0') &&
         (raw[read_pos] != ',') &&
         (raw[read_pos] != '*') &&
         (raw[read_pos] != '\r') &&
         (raw[read_pos] != '\n') &&
         (write_pos < (uint16_t)(dst_len - 1U))) {
    dst[write_pos++] = raw[read_pos++];
  }

  dst[write_pos] = '\0';

  return (write_pos > 0U) ? 1U : 0U;
}

/**
 * @brief Parse a signed decimal string into fixed-point units of 1e-7.
 */
static uint32_t Nmea_ParseDecimalE7(const char *text)
{
  uint32_t whole = 0U;
  uint32_t fraction = 0U;
  uint32_t scale = 1000000U;

  if (text == NULL) {
    return 0U;
  }

  while ((*text >= '0') && (*text <= '9')) {
    whole = whole * 10U + (uint32_t)(*text - '0');
    text++;
  }

  if (*text == '.') {
    text++;

    while ((*text >= '0') &&
           (*text <= '9') &&
           (scale > 0U)) {
      fraction += (uint32_t)(*text - '0') * scale;
      scale /= 10U;
      text++;
    }
  }

  return whole * 10000000UL + fraction;
}

/**
 * @brief Convert NMEA degrees-and-minutes coordinates into signed degrees times 1e7.
 */
static int32_t Nmea_ParseLatLon(const char *ddmm,
                                const char *hemisphere)
{
  uint8_t degree_length;
  char degree_text[4];
  uint32_t degrees;
  uint32_t minutes_e7;
  int32_t result;

  if ((ddmm == NULL) ||
      (hemisphere == NULL)) {
    return 0;
  }

  if ((hemisphere[0] == 'N') ||
      (hemisphere[0] == 'S')) {
    degree_length = 2U;
  } else if ((hemisphere[0] == 'E') ||
             (hemisphere[0] == 'W')) {
    degree_length = 3U;
  } else {
    return 0;
  }

  if (strlen(ddmm) <= degree_length) {
    return 0;
  }

  memset(degree_text, 0, sizeof(degree_text));
  memcpy(degree_text, ddmm, degree_length);

  degrees = (uint32_t)atol(degree_text);
  minutes_e7 = Nmea_ParseDecimalE7(ddmm + degree_length);

  result = (int32_t)(
      degrees * 10000000UL +
      minutes_e7 / 60UL);

  if ((hemisphere[0] == 'S') ||
      (hemisphere[0] == 'W')) {
    result = -result;
  }

  return result;
}

/**
 * @brief Parse GGA position, fix, satellite, HDOP, and altitude fields.
 */
void Nmea_ParseGGA(const Nmea_Sentence_t *sentence,
                   Nmea_GeoData_t *geo)
{
  char field[32];
  int hour;
  int minute;
  int second;

  if ((sentence == NULL) ||
      (geo == NULL) ||
      (sentence->checksum_ok == 0U)) {
    return;
  }

  if (Nmea_GetField(sentence->raw,
                    1U,
                    field,
                    sizeof(field)) != 0U) {
    if (sscanf(field,
               "%2d%2d%2d",
               &hour,
               &minute,
               &second) == 3) {
      geo->utc_sec =
          (uint32_t)hour * 3600UL +
          (uint32_t)minute * 60UL +
          (uint32_t)second;
    }
  }

  if (Nmea_GetField(sentence->raw,
                    2U,
                    field,
                    sizeof(field)) != 0U) {
    char hemisphere[2];

    if (Nmea_GetField(sentence->raw,
                      3U,
                      hemisphere,
                      sizeof(hemisphere)) != 0U) {
      geo->latitude_deg_e7 =
          Nmea_ParseLatLon(field, hemisphere);
    }
  }

  if (Nmea_GetField(sentence->raw,
                    4U,
                    field,
                    sizeof(field)) != 0U) {
    char hemisphere[2];

    if (Nmea_GetField(sentence->raw,
                      5U,
                      hemisphere,
                      sizeof(hemisphere)) != 0U) {
      geo->longitude_deg_e7 =
          Nmea_ParseLatLon(field, hemisphere);
    }
  }

  if (Nmea_GetField(sentence->raw,
                    6U,
                    field,
                    sizeof(field)) != 0U) {
    geo->fix_quality = (uint8_t)atoi(field);
  } else {
    geo->fix_quality = 0U;
  }

  if (Nmea_GetField(sentence->raw,
                    7U,
                    field,
                    sizeof(field)) != 0U) {
    geo->satellites = (uint8_t)atoi(field);
  } else {
    geo->satellites = 0U;
  }

  if (Nmea_GetField(sentence->raw,
                    8U,
                    field,
                    sizeof(field)) != 0U) {
    geo->hdop_cm =
        (uint16_t)(atof(field) * 100.0f);
  } else {
    geo->hdop_cm = 0U;
  }

  if (Nmea_GetField(sentence->raw,
                    9U,
                    field,
                    sizeof(field)) != 0U) {
    geo->altitude_mm =
        (int32_t)(atof(field) * 1000.0f);
  }
}

/**
 * @brief Parse RMC position, speed, heading, and validity fields.
 */
void Nmea_ParseRMC(const Nmea_Sentence_t *sentence,
                   Nmea_GeoData_t *geo)
{
  char field[32];

  if ((sentence == NULL) ||
      (geo == NULL) ||
      (sentence->checksum_ok == 0U)) {
    return;
  }

  if (Nmea_GetField(sentence->raw,
                    7U,
                    field,
                    sizeof(field)) != 0U) {
    geo->speed_cms =
        (uint16_t)(atof(field) * 51.44f);
  } else {
    geo->speed_cms = 0U;
  }

  if (Nmea_GetField(sentence->raw,
                    8U,
                    field,
                    sizeof(field)) != 0U) {
    geo->heading_deg100 =
        (uint16_t)(atof(field) * 100.0f);
  } else {
    geo->heading_deg100 = 0U;
  }

  /* RMC field 9 carries the UTC date as ddmmyy; store it packed as yymmdd. */
  if (Nmea_GetField(sentence->raw,
                    9U,
                    field,
                    sizeof(field)) != 0U) {
    int day;
    int month;
    int year;

    if (sscanf(field,
               "%2d%2d%2d",
               &day,
               &month,
               &year) == 3) {
      geo->utc_date =
          (uint32_t)year * 10000UL +
          (uint32_t)month * 100UL +
          (uint32_t)day;
    }
  }
}
