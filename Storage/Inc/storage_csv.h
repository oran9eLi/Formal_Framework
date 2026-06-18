/**
 * @file storage_csv.h
 * @brief Declare fixed-width CSV formatting helpers for SD logging.
 */

#ifndef STORAGE_CSV_H
#define STORAGE_CSV_H

#include <stddef.h>
#include <stdint.h>
#include "px4lite_types.h"
#include "storage_config.h"

typedef struct
{
    uint32_t time_ms;
    uint8_t gnss_valid;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t roll_deg100;
    int32_t pitch_deg100;
    int32_t temperature_c100;
    uint32_t pressure_hpa100;
    uint32_t humidity_pct100;
    uint32_t voltage_mv;
    uint8_t battery_pct;
} Storage_CsvData_t;

const char *StorageCsv_DataHeader(void);
const char *StorageCsv_ErrorHeader(void);
Px4Lite_Result_t StorageCsv_FormatDataLine(
    const Storage_CsvData_t *data,
    char *line,
    size_t line_size);
Px4Lite_Result_t StorageCsv_FormatErrorLine(
    uint32_t time_ms,
    const char *module,
    uint32_t state,
    uint32_t fault,
    uint32_t error_count,
    const char *message,
    char *line,
    size_t line_size);

#endif
