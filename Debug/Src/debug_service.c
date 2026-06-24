/**
 * @file debug_service.c
 * @brief Run enabled module diagnostics outside production worker tasks.
 */

#include "debug_service.h"

#include "debug_config.h"
#include "debug_console.h"
#include "debug_task_monitor.h"
#include "app_data_api.h"
#include "px4lite_config.h"
#include "px4lite_modules.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"
#include "task.h"

#if DEBUG_PERIODIC_SERVICE_ENABLE

static void DebugService_Task(void *argument);

#if DEBUG_GNSS_MONITOR_ENABLE
/**
 * @brief Print one coherent GNSS measurement and module-state report.
 */
static void DebugService_ReportGnss(uint32_t now_ms)
{
  Px4Lite_SensorGnss_t gnss;
  Px4Lite_ModuleStatus_t status;
  uint32_t report_ms;

  (void)now_ms;

  if ((Px4Lite_CopyGnss(&gnss) == PX4LITE_OK) && (Px4Lite_GetModuleStatus(PX4LITE_MODULE_GNSS, &status) == PX4LITE_OK)) {
    report_ms = Px4Lite_PlatformGetMs();
    DBG_PRINT("GNSS: state=%u fix=%u sats=%u gps=%u/%u bds=%u/%u "
              "lat=%ld lon=%ld age=%lu",
              (unsigned int)status.state, (unsigned int)gnss.fix_type, (unsigned int)gnss.satellites_used, (unsigned int)gnss.gps_used, (unsigned int)gnss.gps_visible, (unsigned int)gnss.bds_used, (unsigned int)gnss.bds_visible, (long)gnss.latitude_e7, (long)gnss.longitude_e7, (unsigned long)Px4Lite_ElapsedMs(report_ms, status.last_rx_ms));
  } else {
    DBG_PRINT("GNSS: waiting for valid NMEA data");
  }
}
#endif

#if DEBUG_IMU_MONITOR_ENABLE
/**
 * @brief Print one coherent MPU6050 six-axis snapshot and module-state report.
 *
 * Values are scaled to integers (milli-g, milli-dps, centi-degree-C) so the
 * report avoids floating-point formatting in the debug console.
 */
static void DebugService_ReportImu(uint32_t now_ms)
{
  App_NavigationSnapshot_t nav;
  Px4Lite_ModuleStatus_t status;
  Px4Lite_Result_t status_rc;
  Px4Lite_Result_t nav_rc;
  uint32_t report_ms;
  uint32_t imu_age_ms;
  uint8_t imu_fresh;

  (void)now_ms;

  status_rc = Px4Lite_GetModuleStatus(PX4LITE_MODULE_IMU, &status);
  report_ms = Px4Lite_PlatformGetMs();
  nav_rc    = App_CopyNavigation(&nav, report_ms);

  if (status_rc == PX4LITE_OK) {
    imu_age_ms = Px4Lite_ElapsedMs(report_ms, status.last_rx_ms);
    imu_fresh  = ((status.state == PX4LITE_STATE_ONLINE) && (imu_age_ms <= PX4LITE_IMU_MAX_AGE_MS)) ? 1U : 0U;
    if ((nav_rc == PX4LITE_OK) && ((nav.valid_mask & PX4LITE_NAV_VALID_ATTITUDE) != 0U)) {
      DBG_PRINT("IMU: state=%u err=%lu age=%lu fresh=%u "
                "roll_cdeg=%ld pitch_cdeg=%ld yaw_cdeg=%ld "
                "rate_cdps=%ld,%ld,%ld",
                (unsigned int)status.state, (unsigned long)status.error_count, (unsigned long)imu_age_ms, (unsigned int)imu_fresh, (long)nav.roll_deg100, (long)nav.pitch_deg100, (long)nav.yaw_deg100, (long)nav.roll_rate_dps100, (long)nav.pitch_rate_dps100, (long)nav.yaw_rate_dps100);
    } else {
      DBG_PRINT("IMU: state=%u fault=%u err=%lu age=%lu fresh=%u "
                "no valid attitude rc=%d mask=0x%08lX",
                (unsigned int)status.state, (unsigned int)status.fault_code, (unsigned long)status.error_count, (unsigned long)imu_age_ms, (unsigned int)imu_fresh, (int)nav_rc, (nav_rc == PX4LITE_OK) ? (unsigned long)nav.valid_mask : 0UL);
    }
  } else {
    DBG_PRINT("IMU: framework status unavailable rc=%d", (int)status_rc);
  }
}
#endif

#if DEBUG_BARO_MONITOR_ENABLE
static void DebugService_ReportBaro(uint32_t now_ms)
{
  Px4Lite_SensorBaro_t baro;
  Px4Lite_ModuleStatus_t status;
  uint32_t report_ms;

  (void)now_ms;

  if ((Px4Lite_CopyBaro(&baro) == PX4LITE_OK) && (Px4Lite_GetModuleStatus(PX4LITE_MODULE_BARO, &status) == PX4LITE_OK)) {
    report_ms = Px4Lite_PlatformGetMs();
    DBG_PRINT("BARO: state=%u pressure_pa=%ld temp_cC=%d alt_mm=%ld "
              "vs_cms=%ld age=%lu",
              (unsigned int)status.state, (long)baro.pressure_pa, (int)(baro.temperature_c * 100.0f), (long)baro.pressure_altitude_mm, (long)baro.vertical_speed_cms, (unsigned long)Px4Lite_ElapsedMs(report_ms, status.last_rx_ms));
  } else {
    DBG_PRINT("BARO: no valid data");
  }
}
#endif

#if DEBUG_POWER_MONITOR_ENABLE
static void DebugService_ReportPower(uint32_t now_ms)
{
  Px4Lite_BatteryStatus_t battery;
  Px4Lite_ModuleStatus_t status;
  uint32_t report_ms;

  (void)now_ms;

  if ((Px4Lite_CopyBattery(&battery) == PX4LITE_OK) && (Px4Lite_GetModuleStatus(PX4LITE_MODULE_BATTERY, &status) == PX4LITE_OK)) {
    report_ms = Px4Lite_PlatformGetMs();
    DBG_PRINT("POWER: state=%u voltage_mv=%lu current_ma=%ld "
              "percent=%u low=%u age=%lu",
              (unsigned int)status.state, (unsigned long)battery.voltage_mv, (long)battery.current_ma, (unsigned int)battery.percent, (unsigned int)battery.low_voltage, (unsigned long)Px4Lite_ElapsedMs(report_ms, status.last_rx_ms));
  } else {
    DBG_PRINT("POWER: no valid data");
  }
}
#endif

#if DEBUG_LORA_MONITOR_ENABLE
static uint32_t DebugService_RatePermille(uint32_t error_count, uint32_t valid_count)
{
  uint64_t total = (uint64_t)error_count + (uint64_t)valid_count;

  if (total == 0ULL) { return 0U; }

  return (uint32_t)(((uint64_t)error_count * 1000ULL) / total);
}

/**
 * @brief Print visualized local LoRa/MAVLink link statistics.
 *
 * The RX loss percentage is a local bad-frame/drop indication derived from
 * parser errors, CRC errors, MAVLink drops, and UART overflow. True air-link
 * packet loss needs peer sequence or ACK telemetry and is intentionally not
 * inferred from one side only.
 */
static void DebugService_ReportLoRa(uint32_t now_ms)
{
  Px4Lite_CommDebugInfo_t info;
  Px4Lite_ModuleStatus_t status;
  uint32_t rx_error_total;
  uint32_t tx_error_total;
  uint32_t rx_loss_permille;
  uint32_t tx_fail_permille;
  uint32_t report_ms;

  (void)now_ms;

  Px4Lite_GetCommDebugInfo(&info);
  if (Px4Lite_GetModuleStatus(PX4LITE_MODULE_LORA, &status) != PX4LITE_OK) {
    DBG_PRINT("LORA: status unavailable");
    return;
  }

  rx_error_total   = info.crc_error_count + info.parse_error_count + info.rx_drop_count + info.rx_overflow_count;
  tx_error_total   = info.tx_busy_count + info.send_error_count + info.mav_error_count;
  rx_loss_permille = DebugService_RatePermille(rx_error_total, info.rx_frame_count);
  tx_fail_permille = DebugService_RatePermille(tx_error_total, info.tx_frame_count);

  report_ms = Px4Lite_PlatformGetMs();
  DBG_PRINT("LORA: state=%u rx=%lu tx=%lu rx_loss=%lu.%lu%% "
            "tx_fail=%lu.%lu%% busy=%lu crc=%lu parse=%lu "
            "drop=%lu ovf=%lu mav(hb/gps/gnss/att/sys/bat/baro/txt)="
            "%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu "
            "stale=%lu nodata=%lu "
            "age_rx=%lu age_tx=%lu msg=%lu",
            (unsigned int)status.state, (unsigned long)info.rx_frame_count, (unsigned long)info.tx_frame_count, (unsigned long)(rx_loss_permille / 10U), (unsigned long)(rx_loss_permille % 10U), (unsigned long)(tx_fail_permille / 10U), (unsigned long)(tx_fail_permille % 10U), (unsigned long)info.tx_busy_count, (unsigned long)info.crc_error_count, (unsigned long)info.parse_error_count, (unsigned long)info.rx_drop_count, (unsigned long)info.rx_overflow_count, (unsigned long)info.mav_heartbeat_count, (unsigned long)info.mav_gps_raw_count, (unsigned long)info.mav_gnss_detail_count, (unsigned long)info.mav_attitude_count, (unsigned long)info.mav_sys_status_count, (unsigned long)info.mav_battery_status_count, (unsigned long)info.mav_scaled_pressure_count, (unsigned long)info.mav_statustext_count, (unsigned long)info.mav_stale_count, (unsigned long)info.mav_no_data_count, (unsigned long)Px4Lite_ElapsedMs(report_ms, info.last_rx_ms), (unsigned long)Px4Lite_ElapsedMs(report_ms, info.last_tx_ms),
            (unsigned long)info.last_msg_id);
}
#endif

#if DEBUG_ALARM_MONITOR_ENABLE
static void DebugService_ReportAlarm(uint32_t now_ms)
{
  App_AlarmSnapshot_t alarm;
  Px4Lite_Result_t result;

  result = App_CopyAlarm(&alarm, now_ms);
  if (result == PX4LITE_OK) {
    DBG_PRINT("ALARM: count=%u highest_src=%u fault=0x%04X sev=%u", (unsigned int)alarm.active_count, (unsigned int)alarm.highest_source_id, (unsigned int)alarm.highest_fault_code, (unsigned int)alarm.highest_severity);
  } else {
    DBG_PRINT("ALARM: no snapshot rc=%d", (int)result);
  }
}
#endif

/**
 * @brief Run each independently enabled monitor at its configured period.
 */
static void DebugService_Task(void *argument)
{
  TickType_t last_wake;
  uint32_t now_ms;
#if DEBUG_STACK_MONITOR_ENABLE
  uint32_t last_stack_report_ms = 0U;
#endif
#if DEBUG_GNSS_MONITOR_ENABLE
  uint32_t last_gnss_report_ms = 0U;
#endif
#if DEBUG_IMU_MONITOR_ENABLE
  uint32_t last_imu_report_ms = 0U;
#endif
#if DEBUG_BARO_MONITOR_ENABLE
  uint32_t last_baro_report_ms = 0U;
#endif
#if DEBUG_POWER_MONITOR_ENABLE
  uint32_t last_power_report_ms = 0U;
#endif
#if DEBUG_LORA_MONITOR_ENABLE
  uint32_t last_lora_report_ms = 0U;
#endif
#if DEBUG_ALARM_MONITOR_ENABLE
  uint32_t last_alarm_report_ms = 0U;
#endif

  (void)argument;
  last_wake = xTaskGetTickCount();

  for (;;) {
    now_ms = Px4Lite_PlatformGetMs();

#if DEBUG_STACK_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_stack_report_ms) >= DEBUG_STACK_REPORT_PERIOD_MS) {
      DebugTaskMonitor_Report();
      last_stack_report_ms = now_ms;
    }
#endif

#if DEBUG_GNSS_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_gnss_report_ms) >= DEBUG_GNSS_REPORT_PERIOD_MS) {
      DebugService_ReportGnss(now_ms);
      last_gnss_report_ms = now_ms;
    }
#endif

#if DEBUG_IMU_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_imu_report_ms) >= DEBUG_IMU_REPORT_PERIOD_MS) {
      DebugService_ReportImu(now_ms);
      last_imu_report_ms = now_ms;
    }
#endif

#if DEBUG_BARO_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_baro_report_ms) >= DEBUG_BARO_REPORT_PERIOD_MS) {
      DebugService_ReportBaro(now_ms);
      last_baro_report_ms = now_ms;
    }
#endif

#if DEBUG_POWER_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_power_report_ms) >= DEBUG_POWER_REPORT_PERIOD_MS) {
      DebugService_ReportPower(now_ms);
      last_power_report_ms = now_ms;
    }
#endif

#if DEBUG_LORA_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_lora_report_ms) >= DEBUG_LORA_REPORT_PERIOD_MS) {
      DebugService_ReportLoRa(now_ms);
      last_lora_report_ms = now_ms;
    }
#endif

#if DEBUG_ALARM_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_alarm_report_ms) >= DEBUG_ALARM_REPORT_PERIOD_MS) {
      DebugService_ReportAlarm(now_ms);
      last_alarm_report_ms = now_ms;
    }
#endif

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DEBUG_SERVICE_PERIOD_MS));
  }
}

#endif

/**
 * @brief Create and register the optional low-priority DebugTask.
 */
BaseType_t DebugService_CreateTask(void)
{
#if DEBUG_PERIODIC_SERVICE_ENABLE
  TaskHandle_t task_handle = 0;

  if (xTaskCreate(DebugService_Task, "debug", DEBUG_SERVICE_TASK_STACK_WORDS, 0, DEBUG_SERVICE_TASK_PRIORITY, &task_handle) != pdPASS) { return pdFAIL; }

  (void)DebugTaskMonitor_Register(task_handle, "debug", DEBUG_SERVICE_TASK_STACK_WORDS);
#endif

  return pdPASS;
}
