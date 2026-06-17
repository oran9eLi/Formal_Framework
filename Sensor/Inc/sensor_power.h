/**
 * @file sensor_power.h
 * @brief Declare the typed power-sense ADC driver.
 */

#ifndef SENSOR_POWER_H
#define SENSOR_POWER_H

#include <stdint.h>

typedef enum
{
    POWER_RESULT_OK = 0,
    POWER_RESULT_NO_DATA,
    POWER_RESULT_IO_ERROR,
    POWER_RESULT_INVALID_PARAM
} Power_Result_t;

typedef struct
{
    uint32_t rx_sequence;
    uint32_t sample_time_ms;
    float voltage_v;
    uint8_t percent;
    uint8_t low_voltage;
    uint16_t reserved;
    uint32_t error_count;
} Power_Snapshot_t;

/* Lightweight status (no raw measurement payload); never touches the ADC. */
typedef struct
{
    uint32_t rx_sequence;
    uint32_t sample_time_ms;
    uint32_t error_count;
    uint8_t percent;
    uint8_t low_voltage;
    uint16_t reserved;
} Power_Status_t;

/**
 * @brief Initialize the board power-sense ADC driver.
 */
Power_Result_t Sensor_Power_Init(void);

/**
 * @brief Service the power ADC: read one sample and cache it. Caller supplies the cycle timestamp.
 */
Power_Result_t Sensor_Power_Service(uint32_t now_ms);

/**
 * @brief Copy the latest cached power snapshot already produced by Service().
 */
Power_Result_t Sensor_Power_CopySnapshot(Power_Snapshot_t *out);

/**
 * @brief Copy the compact power status (no raw measurement payload, no ADC access).
 */
Power_Result_t Sensor_Power_GetStatus(Power_Status_t *out);

/**
 * @brief Request a re-init (cheap; performed on the next Service call).
 */
void Sensor_Power_RequestReinit(void);

#endif
