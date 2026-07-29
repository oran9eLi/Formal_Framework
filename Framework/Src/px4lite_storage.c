/**
 * @file px4lite_storage.c
 * @brief 实现注册表驱动的 SD CSV 存储模块。
 *
 * @details
 * 本文件作为 PX4LITE_MODULE_STORAGE 的 Framework 侧实现，提供生命周期
 * init/recover 回调，并由 storage 任务周期性调用 Px4Lite_StorageWorkRun()。
 * 所有数据来自 Framework topic，不依赖 Business API。
 */

#include "px4lite_storage.h"

#include <string.h>
#include "px4lite_config.h"
#include "px4lite_alarm.h"
#include "px4lite_faults.h"
#include "px4lite_modules.h"
#include "px4lite_platform.h"
#include "px4lite_time.h"
#include "px4lite_topics.h"
#include "bsp_adc_current.h" /* [RAW-DIAG] 临时：SD 两路电流列改记原始引脚 mV；查完删本行 */
#include "storage_config.h"
#include "storage_csv.h"
#include "storage_drop_event.h"
#include "storage_event.h"
#include "storage_module_event.h"
#include "storage_queue.h"
#include "storage_sd.h"
#include "storage_time.h"

#if PX4LITE_ENABLE_STORAGE

static volatile uint8_t s_remount_request;
static uint32_t s_last_data_ms;
static uint32_t s_last_sync_ms;
static Storage_Record_t s_work_record;
static Storage_Record_t s_pop_record;
static Px4Lite_ModuleStatus_t s_module_status[PX4LITE_MODULE_COUNT];

/**
 * @brief 将浮点值按四舍五入转换为 int32。
 *
 * @param[in] value 待转换浮点值。
 *
 * @return 四舍五入后的整数。
 */
static int32_t Storage_RoundFloatToI32(float value)
{
  if (value >= 0.0f) { return (int32_t)(value + 0.5f); }
  return (int32_t)(value - 0.5f);
}

/**
 * @brief 格式化一条存储错误记录并压入记录队列。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 * @param[in] module 错误来源模块名。
 * @param[in] state 模块状态值。
 * @param[in] fault Framework 故障码。
 * @param[in] error_count 附加错误计数。
 * @param[in] message 错误说明文本。
 */
static void Storage_EnqueueEvent(uint32_t now_ms, const char *event_type, const char *source, uint32_t state, uint32_t fault, uint32_t severity, uint8_t active, uint32_t count, const char *message)
{
  Storage_CsvEvent_t event;
  uint32_t local_date_ymd = 0U;
  uint32_t local_time_hhmmss = 0U;

  if (Storage_TimeIsValid() != 0U) {
    local_date_ymd    = Storage_TimeDateYmd();
    local_time_hhmmss = Storage_TimeTimeHhmmss();
  }

  memset(&event, 0, sizeof(event));
  event.time_ms           = now_ms;
  event.local_date_ymd    = local_date_ymd;
  event.local_time_hhmmss = local_time_hhmmss;
  event.event_type        = event_type;
  event.source            = source;
  event.state             = state;
  event.fault             = fault;
  event.severity          = severity;
  event.active            = active;
  event.count             = count;
  event.message           = message;

  memset(&s_work_record, 0, sizeof(s_work_record));
  s_work_record.type            = STORAGE_RECORD_EVENT;
  s_work_record.enqueue_time_ms = now_ms;
  s_work_record.target_date_ymd = local_date_ymd;
  s_work_record.target_time_hhmmss = local_time_hhmmss;
  if (StorageCsv_FormatEventLine(&event, s_work_record.line, sizeof(s_work_record.line)) == PX4LITE_OK) { (void)StorageQueue_Push(&s_work_record); }
}

static void Storage_ReportEventState(uint32_t now_ms, const char *source, uint32_t state, uint32_t fault, uint32_t severity, uint8_t active, const char *message)
{
  Storage_EventAction_t action;
  uint32_t count = 0U;

  action = StorageEventTracker_Update(source, fault, message, active, &count);
  if (action == STORAGE_EVENT_ACTION_ACTIVE) {
    Storage_EnqueueEvent(now_ms, "ALARM_ACTIVE", source, state, fault, severity, 1U, count, message);
  } else if (action == STORAGE_EVENT_ACTION_CLEAR) {
    Storage_EnqueueEvent(now_ms, "STATUS_RECOVERED", source, state, PX4LITE_FAULT_NONE, PX4LITE_SEVERITY_INFO, 0U, count, message);
  }
}

static void Storage_EnqueueError(uint32_t now_ms, const char *module, uint32_t state, uint32_t fault, uint32_t error_count, const char *message)
{
  Storage_ReportEventState(now_ms, module, state, fault, error_count, 1U, message);
}

static void Storage_CheckModuleStatusEvents(uint32_t now_ms)
{
  Storage_ModuleEvent_t event;
  uint32_t version;
  uint32_t i;

  if (Px4Lite_CopyModuleStatuses(s_module_status, (uint16_t)PX4LITE_MODULE_COUNT, &version) != PX4LITE_OK) { return; }
  (void)version;

  for (i = 0U; i < (uint32_t)PX4LITE_MODULE_COUNT; i++) {
    if (StorageModuleEvent_Update(&s_module_status[i], &event) != PX4LITE_OK) { continue; }
    Storage_EnqueueEvent(now_ms,
                         (event.active != 0U) ? "ALARM_ACTIVE" : "STATUS_RECOVERED",
                         event.source,
                         event.state,
                         event.fault,
                         event.severity,
                         event.active,
                         event.count,
                         event.message);
  }
}

static void Storage_CheckQueueDropEvent(uint32_t now_ms)
{
  uint32_t drop_count;

  if (StorageDropEvent_Update(StorageQueue_DropCount(), &drop_count) != PX4LITE_OK) { return; }
  Storage_EnqueueEvent(now_ms,
                       "ALARM_ACTIVE",
                       "STORAGE",
                       PX4LITE_STATE_DEGRADED,
                       PX4LITE_FAULT_STORAGE_FULL,
                       PX4LITE_SEVERITY_WARNING,
                       1U,
                       drop_count,
                       "storage_queue_drop");
}

/**
 * @brief 从 Framework topic 构造一条 CSV 数据记录。
 *
 * @details
 * 任一数据源缺失时会额外写入当日 YYMMDD_E.CSV 记录，而不是静默记录全零字段。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 */
static void Storage_ProduceDataRecord(uint32_t now_ms)
{
  Px4Lite_VehicleNavigation_t navigation;
  Px4Lite_SensorBaro_t baro;
  Px4Lite_BatteryStatus_t battery;
  Px4Lite_BatteryStatus_t battery2;
  Px4Lite_TimeSnapshot_t time;
  Px4Lite_MotorOutputs_t motor;
  Px4Lite_AlarmSnapshot_t alarm;
  Px4Lite_CommDebugInfo_t comm;
  Storage_CsvData_t data;
  uint8_t i;

  memset(&data, 0, sizeof(data));
  data.time_ms = now_ms;

  if (Px4Lite_CopyTime(&time) == PX4LITE_OK) {
    data.local_date_ymd    = time.local_date_ymd;
    data.local_time_hhmmss = time.local_time_hhmmss;
    data.time_sync_state   = (uint8_t)time.sync_state;
  } else {
    data.local_date_ymd    = 0U;
    data.local_time_hhmmss = 0U;
    data.time_sync_state   = 0U;
  }

  if (Px4Lite_CopyNavigation(&navigation) == PX4LITE_OK) {
    data.gnss_valid   = ((navigation.valid_mask & PX4LITE_NAV_VALID_POSITION) != 0U) ? 1U : 0U;
    data.latitude_e7  = navigation.latitude_e7;
    data.longitude_e7 = navigation.longitude_e7;
    data.roll_deg100  = navigation.roll_deg100;
    data.pitch_deg100 = navigation.pitch_deg100;
    data.yaw_deg100   = navigation.yaw_deg100;
    Storage_ReportEventState(now_ms, "NAV", PX4LITE_STATE_ONLINE, PX4LITE_FAULT_SENSOR_INVALID, 0U, 0U, "nav_not_ready");
  } else {
    Storage_EnqueueError(now_ms, "NAV", PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_SENSOR_INVALID, 0U, "nav_not_ready");
  }

  if (Px4Lite_CopyBaro(&baro) == PX4LITE_OK) {
    data.temperature_c100 = Storage_RoundFloatToI32(baro.temperature_c * 100.0f);
    /* pressure_pa 单位为 Pa；1 hPa = 100 Pa，因此 Pa 数值等价于 hPa*100。 */
    data.pressure_hpa100 = (uint32_t)Storage_RoundFloatToI32(baro.pressure_pa);
    data.humidity_pct100 = (uint32_t)Storage_RoundFloatToI32(baro.relative_humidity_pct * 100.0f);
    Storage_ReportEventState(now_ms, "BARO", PX4LITE_STATE_ONLINE, PX4LITE_FAULT_SENSOR_INVALID, 0U, 0U, "baro_not_ready");
  } else {
    Storage_EnqueueError(now_ms, "BARO", PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_SENSOR_INVALID, 0U, "baro_not_ready");
  }

  if (Px4Lite_CopyBattery(&battery) == PX4LITE_OK) {
    data.voltage_mv  = battery.voltage_mv;
    data.current_ma  = battery.current_ma;
    data.power_mw    = (battery.current_ma > 0) ? (uint32_t)((((uint64_t)battery.voltage_mv) * (uint32_t)battery.current_ma) / 1000ULL) : 0U;
    data.battery_pct = battery.percent;
    data.low_voltage = battery.low_voltage;
    Storage_ReportEventState(now_ms, "BATTERY", PX4LITE_STATE_ONLINE, PX4LITE_FAULT_SENSOR_INVALID, 0U, 0U, "battery_not_ready");
    Storage_ReportEventState(now_ms, "BATTERY", (battery.low_voltage != 0U) ? PX4LITE_STATE_DEGRADED : PX4LITE_STATE_ONLINE, PX4LITE_FAULT_SENSOR_INVALID, 0U, battery.low_voltage, "low_voltage");
  } else {
    Storage_EnqueueError(now_ms, "BATTERY", PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_SENSOR_INVALID, 0U, "battery_not_ready");
  }

  if (Px4Lite_CopyBattery2(&battery2) == PX4LITE_OK) {
    data.voltage2_mv  = battery2.voltage_mv;
    data.current2_ma  = battery2.current_ma;
    data.power2_mw    = (battery2.current_ma > 0) ? (uint32_t)((((uint64_t)battery2.voltage_mv) * (uint32_t)battery2.current_ma) / 1000ULL) : 0U;
    data.battery2_pct = battery2.percent;
    data.low_voltage2 = battery2.low_voltage;
    Storage_ReportEventState(now_ms, "BATTERY2", PX4LITE_STATE_ONLINE, PX4LITE_FAULT_SENSOR_INVALID, 0U, 0U, "battery2_not_ready");
    Storage_ReportEventState(now_ms, "BATTERY2", (battery2.low_voltage != 0U) ? PX4LITE_STATE_DEGRADED : PX4LITE_STATE_ONLINE, PX4LITE_FAULT_SENSOR_INVALID, 0U, battery2.low_voltage, "low_voltage2");
  } else {
    Storage_EnqueueError(now_ms, "BATTERY2", PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_SENSOR_INVALID, 0U, "battery2_not_ready");
  }

  /* [RAW-DIAG] 临时诊断：把 current_a / current2_a 两列改记「原始引脚电压 mV」。
     ×1000 使 CSV 数值直接等于 mV(与串口 adc_mv 一致)，绕过零点/灵敏度/滤波，
     用于判定电流信号是否真进了 ADC、以及交换线后是否跟随传感器。
     注意：power_w / power2_w 两列仍是旧标定值，忽略即可。
     还原：删除本段 + 顶部 bsp_adc_current.h 包含。 */
  {
    uint32_t raw_cur_mv1 = 0U;
    uint32_t raw_cur_mv2 = 0U;
    BSP_ADC_Current_GetDiag(BSP_ADC_CURRENT_BATTERY1, &raw_cur_mv1, 0);
    BSP_ADC_Current_GetDiag(BSP_ADC_CURRENT_BATTERY2, &raw_cur_mv2, 0);
    data.current_ma  = (int32_t)raw_cur_mv1 * 1000;
    data.current2_ma = (int32_t)raw_cur_mv2 * 1000;
  }

  if (Px4Lite_CopyMotor(&motor) == PX4LITE_OK) {
    for (i = 0U; (i < PX4LITE_MOTOR_COUNT) && (i < 4U); i++) {
      data.motor_pct[i] = motor.duty_percent[i];
    }
    data.motor_run_state = motor.run_state;
  }

  if (Px4Lite_CopyAlarmSnapshot(&alarm) == PX4LITE_OK) {
    data.active_alarm_count = alarm.active_count;
    data.highest_fault_code = alarm.highest_fault_code;
  }

  Px4Lite_GetCommDebugInfo(&comm);
  data.lora_rx_count          = comm.rx_frame_count;
  data.lora_tx_count          = comm.tx_frame_count;
  data.lora_parse_error_count = comm.parse_error_count;
  data.lora_send_error_count  = comm.send_error_count;
  data.storage_queue_count    = StorageQueue_Count();
  data.storage_drop_count     = StorageQueue_DropCount();

  memset(&s_work_record, 0, sizeof(s_work_record));
  s_work_record.type            = STORAGE_RECORD_DATA;
  s_work_record.enqueue_time_ms = now_ms;
  s_work_record.target_date_ymd = data.local_date_ymd;
  s_work_record.target_time_hhmmss = data.local_time_hhmmss;
  if (StorageCsv_FormatDataLine(&data, s_work_record.line, sizeof(s_work_record.line)) != PX4LITE_OK) {
    Storage_EnqueueError(now_ms, "STORAGE", PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_STORAGE_WRITE, 0U, "csv_line_truncated");
    return;
  }

  if (StorageQueue_Push(&s_work_record) != PX4LITE_OK) { Storage_EnqueueError(now_ms, "STORAGE", PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_STORAGE_FULL, StorageQueue_DropCount(), "storage_queue_full"); }
}
/**
 * @brief 从队列取出一条记录并写入 SD 卡。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 */
static void Storage_ConsumeOne(uint32_t now_ms)
{
  Px4Lite_Result_t result;

  if (Storage_SD_IsReady() == 0U) { return; }
  if (StorageQueue_Pop(&s_pop_record) != PX4LITE_OK) { return; }

  result = (s_pop_record.type == STORAGE_RECORD_EVENT) ? Storage_SD_WriteEventLine(s_pop_record.line, s_pop_record.target_date_ymd, now_ms) : Storage_SD_WriteDataLine(s_pop_record.line, s_pop_record.target_date_ymd, now_ms);
  if (result != PX4LITE_OK) { Storage_EnqueueError(now_ms, "STORAGE", PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_STORAGE_WRITE, (uint32_t)result, "sd_write_failed"); }
}

/**
 * @brief 根据 SD 就绪状态发布 storage 模块公开状态。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 */
static void Storage_PublishState(uint32_t now_ms)
{
  if (Storage_SD_IsReady() != 0U) {
    Px4Lite_SetExternalModuleState(PX4LITE_MODULE_STORAGE, PX4LITE_STATE_ONLINE, PX4LITE_FAULT_NONE, now_ms);
  } else {
    Px4Lite_SetExternalModuleState(PX4LITE_MODULE_STORAGE, PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_STORAGE_NOT_READY, now_ms);
  }
}

Px4Lite_Result_t Px4Lite_StorageModuleInit(void)
{
  s_remount_request = 0U;
  s_last_data_ms    = 0U;
  s_last_sync_ms    = 0U;
  Storage_TimeInit();
  StorageDropEvent_Init();
  StorageEventTracker_Init();
  StorageModuleEvent_Init();
  StorageQueue_Init();
  Storage_SD_Init();
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_StorageRecover(void)
{
  s_remount_request = 1U;
  return PX4LITE_OK;
}

void Px4Lite_StorageWorkRun(uint32_t now_ms)
{
  Px4Lite_TimeSnapshot_t time;

  if (s_remount_request != 0U) {
    s_remount_request = 0U;
    /* 强制清理状态，使下一次 Service() 立即重新挂载，而不是等待退避超时。 */
    Storage_SD_Init();
  }

  if (Px4Lite_CopyTime(&time) == PX4LITE_OK) {
    Storage_TimeUpdate(time.local_date_ymd, time.local_time_hhmmss, 1U);
  } else {
    Storage_TimeUpdate(0U, 0U, 0U);
  }

  Storage_SD_Service(now_ms);
  Storage_CheckModuleStatusEvents(now_ms);

  if ((s_last_data_ms == 0U) || ((uint32_t)(now_ms - s_last_data_ms) >= STORAGE_DATA_PERIOD_MS)) {
    Storage_ProduceDataRecord(now_ms);
    s_last_data_ms = now_ms;
  }

  Storage_ConsumeOne(now_ms);
  Storage_CheckQueueDropEvent(now_ms);

  if ((s_last_sync_ms == 0U) || ((uint32_t)(now_ms - s_last_sync_ms) >= STORAGE_SYNC_PERIOD_MS)) {
    if (Storage_SD_Sync(now_ms) == PX4LITE_IO_ERROR) { Storage_EnqueueError(now_ms, "STORAGE", PX4LITE_STATE_DEGRADED, PX4LITE_FAULT_STORAGE_WRITE, 0U, "sd_sync_failed"); }
    s_last_sync_ms = now_ms;
  }

  Storage_PublishState(now_ms);
}

#endif /* PX4LITE_ENABLE_STORAGE */
