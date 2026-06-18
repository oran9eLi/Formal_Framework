/**
 * @file sensor_power.c
 * @brief Implement the typed power-sense ADC driver.
 */

#include "sensor_power.h"

#include "bsp_adc.h"

#include <string.h>

#define POWER_ZERO_MV         9900U
#define POWER_STEP5_MV       10200U
#define POWER_STEP10_MV      10500U
#define POWER_FULL_MV        12550U
#define POWER_FILTER_OLD_WEIGHT 3U
#define POWER_FILTER_TOTAL      4U
#define POWER_PERCENT_STEP      5U
#define POWER_PERCENT_CONFIRM_COUNT 10U

static Power_Snapshot_t s_snapshot;
static uint32_t s_filtered_voltage_mv;
static uint8_t s_pending_percent_count;
static uint8_t s_initialized;
static uint8_t s_filter_valid;
static volatile uint8_t s_reinit_request;

static uint8_t Power_CalcPercent(uint32_t voltage_mv)
{
    uint32_t range_mv;
    uint32_t percent;

    if (voltage_mv < POWER_ZERO_MV)
    {
        return 0U;
    }
    if (voltage_mv < POWER_STEP5_MV)
    {
        return 5U;
    }
    if (voltage_mv < POWER_STEP10_MV)
    {
        return 10U;
    }
    if (voltage_mv >= POWER_FULL_MV)
    {
        return 100U;
    }

    range_mv = POWER_FULL_MV - POWER_STEP10_MV;
    percent = 10U +
              ((((voltage_mv - POWER_STEP10_MV) * 90U) +
               (range_mv / 2U)) /
               range_mv);
    percent = ((percent + (POWER_PERCENT_STEP / 2U)) /
               POWER_PERCENT_STEP) *
              POWER_PERCENT_STEP;
    if (percent > 100U)
    {
        percent = 100U;
    }
    return (uint8_t)percent;
}

static uint32_t Power_FilterVoltage(uint32_t voltage_mv)
{
    if (s_filter_valid == 0U)
    {
        s_filtered_voltage_mv = voltage_mv;
        s_filter_valid = 1U;
    }
    else
    {
        s_filtered_voltage_mv =
            ((s_filtered_voltage_mv * POWER_FILTER_OLD_WEIGHT) +
             voltage_mv +
             (POWER_FILTER_TOTAL / 2U)) /
            POWER_FILTER_TOTAL;
    }

    return s_filtered_voltage_mv;
}

static uint8_t Power_ApplyPercentConfirm(
    uint8_t previous,
    uint8_t candidate)
{
    if (candidate == previous)
    {
        s_pending_percent_count = 0U;
        return previous;
    }

    if (s_pending_percent_count < POWER_PERCENT_CONFIRM_COUNT)
    {
        s_pending_percent_count++;
    }
    if (s_pending_percent_count >= POWER_PERCENT_CONFIRM_COUNT)
    {
        s_pending_percent_count = 0U;
        return candidate;
    }
    return previous;
}

Power_Result_t Sensor_Power_Init(void)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_filtered_voltage_mv = 0U;
    s_pending_percent_count = 0U;
    s_filter_valid = 0U;
    s_initialized = 1U;
    return POWER_RESULT_OK;
}

void Sensor_Power_RequestReinit(void)
{
    s_reinit_request = 1U;
}

Power_Result_t Sensor_Power_Service(uint32_t now_ms)
{
    uint32_t voltage_mv = 0U;
    uint32_t filtered_voltage_mv;
    uint8_t candidate_percent;

    if (s_reinit_request != 0U)
    {
        s_reinit_request = 0U;
        (void)Sensor_Power_Init();
    }
    if (s_initialized == 0U)
    {
        return POWER_RESULT_NO_DATA;
    }

    if (BSP_ADC_ReadVoltageMv(&voltage_mv) != BSP_STATUS_OK)
    {
        s_snapshot.error_count++;
        return POWER_RESULT_IO_ERROR;
    }

    s_snapshot.rx_sequence++;
    if (s_snapshot.rx_sequence == 0U)
    {
        s_snapshot.rx_sequence = 1U;
    }
    s_snapshot.sample_time_ms = now_ms;
    s_snapshot.voltage_v = ((float)voltage_mv) / 1000.0f;
    filtered_voltage_mv = Power_FilterVoltage(voltage_mv);
    candidate_percent = Power_CalcPercent(filtered_voltage_mv);
    s_snapshot.percent =
        (s_snapshot.rx_sequence == 1U)
            ? candidate_percent
            : Power_ApplyPercentConfirm(
                  s_snapshot.percent,
                  candidate_percent);
    s_snapshot.low_voltage =
        (voltage_mv < POWER_ZERO_MV) ? 1U : 0U;

    return POWER_RESULT_OK;
}

Power_Result_t Sensor_Power_CopySnapshot(Power_Snapshot_t *out)
{
    if (out == 0)
    {
        return POWER_RESULT_INVALID_PARAM;
    }

    *out = s_snapshot;
    return (s_snapshot.rx_sequence != 0U)
               ? POWER_RESULT_OK
               : POWER_RESULT_NO_DATA;
}

Power_Result_t Sensor_Power_GetStatus(Power_Status_t *out)
{
    if (out == 0)
    {
        return POWER_RESULT_INVALID_PARAM;
    }

    out->rx_sequence = s_snapshot.rx_sequence;
    out->sample_time_ms = s_snapshot.sample_time_ms;
    out->error_count = s_snapshot.error_count;
    out->percent = s_snapshot.percent;
    out->low_voltage = s_snapshot.low_voltage;
    out->reserved = 0U;
    return (s_snapshot.rx_sequence != 0U)
               ? POWER_RESULT_OK
               : POWER_RESULT_NO_DATA;
}
