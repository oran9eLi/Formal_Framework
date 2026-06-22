/**
 * @file storage_csv.c
 * @brief Implement CSV formatting without floating-point printf.
 */

#include "storage_csv.h"

#include <stdio.h>

typedef struct {
  char sign;
  uint32_t whole;
  uint32_t frac;
} Storage_Fixed2_t;

static void Storage_FormatSignedCenti(int32_t raw, Storage_Fixed2_t *out)
{
  uint32_t magnitude;

  if (raw < 0) {
    out->sign = '-';
    magnitude = (uint32_t)(-raw);
  } else {
    out->sign = '+';
    magnitude = (uint32_t)raw;
  }
  out->whole = magnitude / 100U;
  out->frac  = magnitude % 100U;
}

const char *StorageCsv_DataHeader(void)
{
  return "time_ms,local_date,local_time,time_sync_state,gnss_valid,lat_e7,lon_e7,roll_deg,pitch_deg,"
         "temp_c,pressure_hpa,humidity_pct,voltage_v,battery_pct,"
         "motor1_pct,motor2_pct,motor3_pct,motor4_pct,motor_run_state,"
         "active_alarm_count,highest_fault_code,lora_rx_count,lora_tx_count,lora_parse_error_count,lora_send_error_count,"
         "storage_queue_count,storage_drop_count\r\n";
}

const char *StorageCsv_ErrorHeader(void)
{
  return "time_ms,module,state,fault,error_count,message\r\n";
}

Px4Lite_Result_t StorageCsv_FormatDataLine(const Storage_CsvData_t *data, char *line, size_t line_size)
{
  Storage_Fixed2_t roll;
  Storage_Fixed2_t pitch;
  Storage_Fixed2_t temperature;
  int written;

  if ((data == 0) || (line == 0) || (line_size == 0U)) { return PX4LITE_INVALID_PARAM; }

  Storage_FormatSignedCenti(data->roll_deg100, &roll);
  Storage_FormatSignedCenti(data->pitch_deg100, &pitch);
  Storage_FormatSignedCenti(data->temperature_c100, &temperature);

  written = snprintf(line, line_size,
                     "%lu,%lu,%06lu,%u,%u,%ld,%ld,%c%lu.%02lu,%c%lu.%02lu,"
                     "%c%lu.%02lu,%lu.%02lu,%lu.%02lu,%lu.%03lu,%u,"
                     "%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%u,%lu\r\n",
                     (unsigned long)data->time_ms, (unsigned long)data->local_date_ymd, (unsigned long)data->local_time_hhmmss, (unsigned int)data->time_sync_state, (unsigned int)data->gnss_valid, (long)data->latitude_e7, (long)data->longitude_e7, roll.sign, (unsigned long)roll.whole, (unsigned long)roll.frac, pitch.sign, (unsigned long)pitch.whole, (unsigned long)pitch.frac, temperature.sign, (unsigned long)temperature.whole, (unsigned long)temperature.frac, (unsigned long)(data->pressure_hpa100 / 100U), (unsigned long)(data->pressure_hpa100 % 100U), (unsigned long)(data->humidity_pct100 / 100U), (unsigned long)(data->humidity_pct100 % 100U), (unsigned long)(data->voltage_mv / 1000U), (unsigned long)(data->voltage_mv % 1000U), (unsigned int)data->battery_pct, (unsigned int)data->motor_pct[0], (unsigned int)data->motor_pct[1], (unsigned int)data->motor_pct[2], (unsigned int)data->motor_pct[3], (unsigned int)data->motor_run_state, (unsigned int)data->active_alarm_count, (unsigned int)data->highest_fault_code, (unsigned long)data->lora_rx_count, (unsigned long)data->lora_tx_count, (unsigned long)data->lora_parse_error_count, (unsigned long)data->lora_send_error_count, (unsigned int)data->storage_queue_count, (unsigned long)data->storage_drop_count);
  return ((written > 0) && ((size_t)written < line_size)) ? PX4LITE_OK : PX4LITE_OVERFLOW;
}

Px4Lite_Result_t StorageCsv_FormatErrorLine(uint32_t time_ms, const char *module, uint32_t state, uint32_t fault, uint32_t error_count, const char *message, char *line, size_t line_size)
{
  int written;

  if ((module == 0) || (message == 0) || (line == 0) || (line_size == 0U)) { return PX4LITE_INVALID_PARAM; }

  written = snprintf(line, line_size, "%lu,%s,%lu,%lu,%lu,%s\r\n", (unsigned long)time_ms, module, (unsigned long)state, (unsigned long)fault, (unsigned long)error_count, message);

  return ((written > 0) && ((size_t)written < line_size)) ? PX4LITE_OK : PX4LITE_OVERFLOW;
}
