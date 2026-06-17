/**
 * @file sensor_gnss.h
 * @brief Define the ATGM336H GNSS snapshot and driver interface.
 */

#ifndef SENSOR_GNSS_H
#define SENSOR_GNSS_H

#include <stdint.h>

typedef enum {
  GNSS_DATA_NONE = 0,
  GNSS_DATA_NO_FIX,
  GNSS_DATA_FIX_OK
} Gnss_DataState_t;

typedef enum {
  GNSS_RESULT_OK = 0,
  GNSS_RESULT_NO_DATA,
  GNSS_RESULT_NO_FIX,
  GNSS_RESULT_INVALID_PARAM
} Gnss_Result_t;

typedef struct {
  /* Data fact only. Component timeout/offline state belongs to Framework. */
  Gnss_DataState_t data_state;

  uint8_t fix_valid;
  uint8_t fix_quality;
  uint8_t fix_dimension;

  /* Combined satellites used by the current GGA fix. */
  uint8_t satellites;

  /* Satellites visible per constellation, reported by GSV. */
  uint8_t gps_visible_sats;
  uint8_t bds_visible_sats;

  /* Satellites used per constellation, reported by GSA. */
  uint8_t gps_used_sats;
  uint8_t bds_used_sats;

  uint16_t hdop_cm;
  uint32_t utc_sec;
  uint32_t utc_date;   /* Packed yymmdd from RMC; 0 when no valid date. */

  int32_t latitude_deg_e7;
  int32_t longitude_deg_e7;
  int32_t altitude_mm;

  uint32_t speed_cms;
  uint32_t heading_deg100;

  uint32_t rx_sequence;
  uint32_t last_rx_ms;
  uint32_t last_fix_ms;
} Gnss_Snapshot_t;

/* Lightweight status (no position payload); never touches a slow bus. */
typedef struct {
  Gnss_DataState_t data_state;
  uint8_t  fix_valid;
  uint8_t  satellites;
  uint32_t rx_sequence;
  uint32_t last_rx_ms;
  uint32_t last_fix_ms;
} Gnss_Status_t;

/**
 * @brief Reset the typed GNSS parser and snapshot state.
 */
Gnss_Result_t Sensor_GNSS_Init(void);

/**
 * @brief Service pending GNSS input; the caller supplies the cycle timestamp.
 */
Gnss_Result_t Sensor_GNSS_Service(uint32_t now_ms);

/**
 * @brief Copy the latest coherent GNSS snapshot already produced by Service().
 */
Gnss_Result_t Sensor_GNSS_CopySnapshot(Gnss_Snapshot_t *out);

/**
 * @brief Copy the compact GNSS status without the position payload.
 */
Gnss_Result_t Sensor_GNSS_GetStatus(Gnss_Status_t *out);

/**
 * @brief Request a parser/snapshot re-init (cheap; performed on the next Service call).
 */
void Sensor_GNSS_RequestReinit(void);

#endif

