/**
 * @file px4lite_storage.c
 * @brief Implement the registry-driven SD CSV storage module.
 *
 * Registered as PX4LITE_MODULE_STORAGE with lifecycle init/recover callbacks.
 * The storage task drives Px4Lite_StorageWorkRun() through the generic work
 * item. Data comes only from framework topics via the public copy API, so this
 * file stays inside the Framework layer (no Business / app_data_api include).
 */

#include "px4lite_storage.h"

#include <string.h>
#include "px4lite_config.h"
#include "px4lite_faults.h"
#include "px4lite_modules.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"
#include "storage_config.h"
#include "storage_csv.h"
#include "storage_queue.h"
#include "storage_sd.h"

#if PX4LITE_ENABLE_STORAGE

static volatile uint8_t s_remount_request;
static uint32_t s_last_data_ms;
static uint32_t s_last_sync_ms;
static Storage_Record_t s_work_record;
static Storage_Record_t s_pop_record;

static int32_t Storage_RoundFloatToI32(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

static void Storage_EnqueueError(uint32_t now_ms,
                                 const char *module,
                                 uint32_t state,
                                 uint32_t fault,
                                 uint32_t error_count,
                                 const char *message)
{
    memset(&s_work_record, 0, sizeof(s_work_record));
    s_work_record.type = STORAGE_RECORD_ERROR;
    s_work_record.enqueue_time_ms = now_ms;
    if (StorageCsv_FormatErrorLine(now_ms,
                                   module,
                                   state,
                                   fault,
                                   error_count,
                                   message,
                                   s_work_record.line,
                                   sizeof(s_work_record.line)) == PX4LITE_OK)
    {
        (void)StorageQueue_Push(&s_work_record);
    }
}

/*
 * Build one data row from framework topics. Each missing source produces an
 * ERROR.CSV row instead of silently logging a zeroed field.
 */
static void Storage_ProduceDataRecord(uint32_t now_ms)
{
    Px4Lite_VehicleNavigation_t navigation;
    Px4Lite_SensorBaro_t baro;
    Px4Lite_BatteryStatus_t battery;
    Storage_CsvData_t data;

    memset(&data, 0, sizeof(data));
    data.time_ms = now_ms;

    if (Px4Lite_CopyNavigation(&navigation) == PX4LITE_OK)
    {
        data.gnss_valid =
            ((navigation.valid_mask & PX4LITE_NAV_VALID_POSITION) != 0U)
                ? 1U
                : 0U;
        data.latitude_e7 = navigation.latitude_e7;
        data.longitude_e7 = navigation.longitude_e7;
        data.roll_deg100 = navigation.roll_deg100;
        data.pitch_deg100 = navigation.pitch_deg100;
    }
    else
    {
        Storage_EnqueueError(now_ms,
                             "NAV",
                             PX4LITE_STATE_DEGRADED,
                             PX4LITE_FAULT_SENSOR_INVALID,
                             0U,
                             "nav_not_ready");
    }

    if (Px4Lite_CopyBaro(&baro) == PX4LITE_OK)
    {
        data.temperature_c100 =
            Storage_RoundFloatToI32(baro.temperature_c * 100.0f);
        /* baro.pressure_pa is in pascals; 1 hPa = 100 Pa, so the pascal value
         * already equals the desired hPa*100 fixed-point field. */
        data.pressure_hpa100 =
            (uint32_t)Storage_RoundFloatToI32(baro.pressure_pa);
        data.humidity_pct100 =
            (uint32_t)Storage_RoundFloatToI32(
                baro.relative_humidity_pct * 100.0f);
    }
    else
    {
        Storage_EnqueueError(now_ms,
                             "BARO",
                             PX4LITE_STATE_DEGRADED,
                             PX4LITE_FAULT_SENSOR_INVALID,
                             0U,
                             "baro_not_ready");
    }

    if (Px4Lite_CopyBattery(&battery) == PX4LITE_OK)
    {
        data.voltage_mv = battery.voltage_mv;
        data.battery_pct = battery.percent;
        if (battery.low_voltage != 0U)
        {
            Storage_EnqueueError(now_ms,
                                 "BATTERY",
                                 PX4LITE_STATE_DEGRADED,
                                 PX4LITE_FAULT_SENSOR_INVALID,
                                 0U,
                                 "low_voltage");
        }
    }
    else
    {
        Storage_EnqueueError(now_ms,
                             "BATTERY",
                             PX4LITE_STATE_DEGRADED,
                             PX4LITE_FAULT_SENSOR_INVALID,
                             0U,
                             "battery_not_ready");
    }

    memset(&s_work_record, 0, sizeof(s_work_record));
    s_work_record.type = STORAGE_RECORD_DATA;
    s_work_record.enqueue_time_ms = now_ms;
    if (StorageCsv_FormatDataLine(&data,
                                  s_work_record.line,
                                  sizeof(s_work_record.line)) != PX4LITE_OK)
    {
        Storage_EnqueueError(now_ms,
                             "STORAGE",
                             PX4LITE_STATE_DEGRADED,
                             PX4LITE_FAULT_STORAGE_WRITE,
                             0U,
                             "csv_line_truncated");
        return;
    }

    if (StorageQueue_Push(&s_work_record) != PX4LITE_OK)
    {
        Storage_EnqueueError(now_ms,
                             "STORAGE",
                             PX4LITE_STATE_DEGRADED,
                             PX4LITE_FAULT_STORAGE_FULL,
                             StorageQueue_DropCount(),
                             "storage_queue_full");
    }
}

static void Storage_ConsumeOne(uint32_t now_ms)
{
    Px4Lite_Result_t result;

    if (Storage_SD_IsReady() == 0U)
    {
        return;
    }
    if (StorageQueue_Pop(&s_pop_record) != PX4LITE_OK)
    {
        return;
    }

    result = (s_pop_record.type == STORAGE_RECORD_ERROR)
                 ? Storage_SD_WriteErrorLine(s_pop_record.line, now_ms)
                 : Storage_SD_WriteDataLine(s_pop_record.line, now_ms);
    if (result != PX4LITE_OK)
    {
        Storage_EnqueueError(now_ms,
                             "STORAGE",
                             PX4LITE_STATE_DEGRADED,
                             PX4LITE_FAULT_STORAGE_WRITE,
                             (uint32_t)result,
                             "sd_write_failed");
    }
}

static void Storage_PublishState(uint32_t now_ms)
{
    if (Storage_SD_IsReady() != 0U)
    {
        Px4Lite_SetExternalModuleState(PX4LITE_MODULE_STORAGE,
                                       PX4LITE_STATE_ONLINE,
                                       PX4LITE_FAULT_NONE,
                                       now_ms);
    }
    else
    {
        Px4Lite_SetExternalModuleState(PX4LITE_MODULE_STORAGE,
                                       PX4LITE_STATE_DEGRADED,
                                       PX4LITE_FAULT_STORAGE_NOT_READY,
                                       now_ms);
    }
}

Px4Lite_Result_t Px4Lite_StorageModuleInit(void)
{
    s_remount_request = 0U;
    s_last_data_ms = 0U;
    s_last_sync_ms = 0U;
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
    if (s_remount_request != 0U)
    {
        s_remount_request = 0U;
        /* Force a clean re-init so the next Service() remounts immediately
         * instead of waiting out the retry backoff. */
        Storage_SD_Init();
    }

    Storage_SD_Service(now_ms);

    if ((s_last_data_ms == 0U) ||
        ((uint32_t)(now_ms - s_last_data_ms) >= STORAGE_DATA_PERIOD_MS))
    {
        Storage_ProduceDataRecord(now_ms);
        s_last_data_ms = now_ms;
    }

    Storage_ConsumeOne(now_ms);

    if ((s_last_sync_ms == 0U) ||
        ((uint32_t)(now_ms - s_last_sync_ms) >= STORAGE_SYNC_PERIOD_MS))
    {
        if (Storage_SD_Sync(now_ms) == PX4LITE_IO_ERROR)
        {
            Storage_EnqueueError(now_ms,
                                 "STORAGE",
                                 PX4LITE_STATE_DEGRADED,
                                 PX4LITE_FAULT_STORAGE_WRITE,
                                 0U,
                                 "sd_sync_failed");
        }
        s_last_sync_ms = now_ms;
    }

    Storage_PublishState(now_ms);
}

#endif /* PX4LITE_ENABLE_STORAGE */
