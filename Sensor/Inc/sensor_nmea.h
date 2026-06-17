/**
 * @file sensor_nmea.h
 * @brief Declare NMEA extraction, checksum, and sentence parsing helpers.
 */

#ifndef SENSOR_NMEA_H
#define SENSOR_NMEA_H

#include <stdint.h>

#define NMEA_SENTENCE_MAX_LEN 128U

typedef enum {
  NMEA_TYPE_GGA = 0,
  NMEA_TYPE_RMC,
  NMEA_TYPE_GSV,
  NMEA_TYPE_GSA,
  NMEA_TYPE_VTG,
  NMEA_TYPE_UNKNOWN
} Nmea_Type_t;

typedef struct {
  Nmea_Type_t type;
  char raw[NMEA_SENTENCE_MAX_LEN];
  uint16_t raw_len;
  uint8_t checksum_ok;
} Nmea_Sentence_t;

typedef struct {
  int32_t latitude_deg_e7;
  int32_t longitude_deg_e7;
  int32_t altitude_mm;

  uint16_t speed_cms;
  uint16_t heading_deg100;

  uint8_t satellites;
  uint8_t fix_quality;

  uint16_t hdop_cm;
  uint32_t utc_sec;
  uint32_t utc_date;   /* Packed yymmdd from RMC; 0 when no valid date. */
} Nmea_GeoData_t;

/**
 * @brief Validate the hexadecimal XOR checksum of one complete NMEA sentence.
 */
uint8_t Nmea_Checksum(const char *sentence, uint16_t len);

/**
 * @brief Extract one complete NMEA sentence from a byte buffer and advance the cursor.
 */
uint8_t Nmea_ExtractSentence(const uint8_t *buf,
                             uint16_t buf_len,
                             uint16_t *cursor,
                             Nmea_Sentence_t *out);

/**
 * @brief Copy one comma-separated NMEA field into a caller buffer.
 */
uint8_t Nmea_GetField(const char *raw,
                      uint8_t index,
                      char *dst,
                      uint16_t dst_len);

/**
 * @brief Parse GGA position, fix, satellite, HDOP, and altitude fields.
 */
void Nmea_ParseGGA(const Nmea_Sentence_t *s,
                   Nmea_GeoData_t *geo);

/**
 * @brief Parse RMC position, speed, heading, and validity fields.
 */
void Nmea_ParseRMC(const Nmea_Sentence_t *s,
                   Nmea_GeoData_t *geo);

#endif


