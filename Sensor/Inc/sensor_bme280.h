/**
 * @file sensor_bme280.h
 * @brief Declare the typed BME280 environmental sensor driver.
 */

#ifndef SENSOR_BME280_H
#define SENSOR_BME280_H

#include <stdint.h>

typedef enum
{
    BME280_RESULT_OK = 0,
    BME280_RESULT_NO_DATA,
    BME280_RESULT_BAD_ID,
    BME280_RESULT_IO_ERROR,
    BME280_RESULT_TIMEOUT,
    BME280_RESULT_INVALID_PARAM
} Bme280_Result_t;

typedef struct
{
    uint32_t rx_sequence;
    uint32_t sample_time_ms;
    float temperature_c;
    float pressure_pa;
    float relative_humidity_pct;
    uint32_t error_count;
} Bme280_Snapshot_t;

/* Lightweight status (no measurement payload); never touches the I2C bus. */
typedef struct
{
    uint32_t rx_sequence;
    uint32_t sample_time_ms;
    uint32_t error_count;
} Bme280_Status_t;

/**
 * @brief Initialize the BME280 bus, verify chip identity, and read calibration data.
 */
Bme280_Result_t Sensor_BME280_Init(void);

/**
 * @brief Service the BME280: trigger a forced measurement and cache it. Caller supplies the cycle timestamp.
 */
Bme280_Result_t Sensor_BME280_Service(uint32_t now_ms);

/**
 * @brief Copy the latest cached BME280 snapshot already produced by Service().
 */
Bme280_Result_t Sensor_BME280_CopySnapshot(Bme280_Snapshot_t *out);

/**
 * @brief Copy the compact BME280 status (no measurement payload, no bus access).
 */
Bme280_Result_t Sensor_BME280_GetStatus(Bme280_Status_t *out);

/**
 * @brief Request a re-init (cheap; performed on the next Service call).
 */
void Sensor_BME280_RequestReinit(void);

#endif
