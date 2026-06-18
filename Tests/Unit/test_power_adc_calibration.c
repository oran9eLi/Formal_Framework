#include <stdint.h>
#include <stdio.h>

#include "bsp_config.h"
#include "bsp_status.h"
#include "sensor_power.h"

static const uint32_t *s_voltage_sequence;
static uint32_t s_voltage_count;
static uint32_t s_voltage_index;

BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *voltage_mv)
{
    if ((voltage_mv == 0) || (s_voltage_sequence == 0) ||
        (s_voltage_count == 0U))
    {
        return BSP_STATUS_ERROR;
    }

    if (s_voltage_index >= s_voltage_count)
    {
        *voltage_mv = s_voltage_sequence[s_voltage_count - 1U];
    }
    else
    {
        *voltage_mv = s_voltage_sequence[s_voltage_index];
        s_voltage_index++;
    }
    return BSP_STATUS_OK;
}

static void LoadVoltageSequence(const uint32_t *sequence, uint32_t count)
{
    s_voltage_sequence = sequence;
    s_voltage_count = count;
    s_voltage_index = 0U;
}

static uint32_t SnapshotVoltageMv(void)
{
    Power_Snapshot_t snapshot;

    if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK)
    {
        return 0U;
    }
    return (uint32_t)((snapshot.voltage_v * 1000.0f) + 0.5f);
}

static uint8_t SnapshotPercent(void)
{
    Power_Snapshot_t snapshot;

    if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK)
    {
        return 255U;
    }
    return snapshot.percent;
}

static int ExpectUint32(const char *label,
                        uint32_t actual,
                        uint32_t expected)
{
    if (actual != expected)
    {
        printf("%s: expected %lu, got %lu\n",
               label,
               (unsigned long)expected,
               (unsigned long)actual);
        return 0;
    }
    return 1;
}

static int TestDividerMatchesMeasuredCalibration(void)
{
    int ok = 1;

    ok &= ExpectUint32("divider numerator",
                       BSP_ADC_DIVIDER_NUM,
                       10080U);
    ok &= ExpectUint32("divider denominator",
                       BSP_ADC_DIVIDER_DEN,
                       1000U);
    return ok;
}

static int TestPublishedVoltageKeepsMillivoltPrecision(void)
{
    static const uint32_t voltages_mv[] = {
        10750U,
        10790U,
        10740U,
        10760U
    };
    uint32_t i;
    int ok = 1;

    LoadVoltageSequence(voltages_mv,
                        (uint32_t)(sizeof(voltages_mv) /
                                   sizeof(voltages_mv[0])));
    (void)Sensor_Power_Init();

    for (i = 0U; i < (uint32_t)(sizeof(voltages_mv) /
                                sizeof(voltages_mv[0])); ++i)
    {
        if (Sensor_Power_Service((i + 1U) * 1000U) != POWER_RESULT_OK)
        {
            printf("service failed at sample %lu\n", (unsigned long)i);
            return 0;
        }
        ok &= ExpectUint32("millivolt voltage output",
                           SnapshotVoltageMv(),
                           voltages_mv[i]);
    }
    return ok;
}

static int TestBatteryPercentClampsAndUsesFivePercentSteps(void)
{
    static const uint32_t voltages_mv[] = {
        9800U,
        9900U,
        10199U,
        10200U,
        10499U,
        10500U,
        11525U,
        12600U
    };
    uint8_t expected_pct[] = {
        0U,
        5U,
        5U,
        10U,
        10U,
        10U,
        55U,
        100U
    };
    uint32_t i;
    int ok = 1;

    for (i = 0U; i < (uint32_t)(sizeof(voltages_mv) /
                                sizeof(voltages_mv[0])); ++i)
    {
        LoadVoltageSequence(&voltages_mv[i], 1U);
        (void)Sensor_Power_Init();
        if (Sensor_Power_Service((i + 1U) * 1000U) != POWER_RESULT_OK)
        {
            printf("service failed for percent clamp sample %lu\n",
                   (unsigned long)i);
            return 0;
        }
        ok &= ExpectUint32("five-percent battery step",
                           SnapshotPercent(),
                           expected_pct[i]);
    }

    return ok;
}

static int TestBatteryLowVoltageFlagFollowsZeroBand(void)
{
    static const uint32_t voltages_mv[] = {
        9800U,
        9900U
    };
    uint8_t expected_low[] = {
        1U,
        0U
    };
    uint32_t i;
    int ok = 1;

    for (i = 0U; i < (uint32_t)(sizeof(voltages_mv) /
                                sizeof(voltages_mv[0])); ++i)
    {
        Power_Snapshot_t snapshot;

        LoadVoltageSequence(&voltages_mv[i], 1U);
        (void)Sensor_Power_Init();
        if (Sensor_Power_Service((i + 1U) * 1000U) != POWER_RESULT_OK)
        {
            printf("service failed for low voltage sample %lu\n",
                   (unsigned long)i);
            return 0;
        }
        if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK)
        {
            printf("snapshot copy failed for low voltage sample %lu\n",
                   (unsigned long)i);
            return 0;
        }
        ok &= ExpectUint32("low voltage flag",
                           snapshot.low_voltage,
                           expected_low[i]);
    }

    return ok;
}

static int TestBatteryPercentRequiresTenConsecutiveNewSteps(void)
{
    static const uint32_t initial_mv = 11525U;
    static const uint32_t higher_mv = 12550U;
    uint32_t i;
    int ok = 1;

    LoadVoltageSequence(&initial_mv, 1U);
    (void)Sensor_Power_Init();
    if (Sensor_Power_Service(1000U) != POWER_RESULT_OK)
    {
        printf("initial service failed\n");
        return 0;
    }
    ok &= ExpectUint32("initial battery step", SnapshotPercent(), 55U);

    LoadVoltageSequence(&higher_mv, 1U);
    for (i = 0U; i < 9U; ++i)
    {
        if (Sensor_Power_Service((i + 2U) * 1000U) != POWER_RESULT_OK)
        {
            printf("pre-confirm service failed at sample %lu\n",
                   (unsigned long)i);
            return 0;
        }
        ok &= ExpectUint32("battery step before confirm",
                           SnapshotPercent(),
                           55U);
    }

    if (Sensor_Power_Service(11000U) != POWER_RESULT_OK)
    {
        printf("confirm service failed\n");
        return 0;
    }
    ok &= ExpectUint32("battery step after ten confirms",
                       SnapshotPercent(),
                       95U);

    return ok;
}

int main(void)
{
    int ok = 1;

    ok &= TestDividerMatchesMeasuredCalibration();
    ok &= TestPublishedVoltageKeepsMillivoltPrecision();
    ok &= TestBatteryPercentClampsAndUsesFivePercentSteps();
    ok &= TestBatteryLowVoltageFlagFollowsZeroBand();
    ok &= TestBatteryPercentRequiresTenConsecutiveNewSteps();

    if (ok)
    {
        printf("power adc calibration tests passed\n");
        return 0;
    }

    return 1;
}
