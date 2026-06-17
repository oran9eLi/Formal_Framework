/**
 * @file sensor_mpu6050.h
 * @brief Declare the typed MPU6050 six-axis sensor driver.
 */

#ifndef SENSOR_MPU6050_H
#define SENSOR_MPU6050_H

#include <stdint.h>

typedef enum
{
    MPU6050_RESULT_OK = 0,
    MPU6050_RESULT_NO_DATA,
    MPU6050_RESULT_BAD_ID,
    MPU6050_RESULT_IO_ERROR,
    MPU6050_RESULT_TIMEOUT,
    MPU6050_RESULT_INVALID_PARAM
} Mpu6050_Result_t;

typedef enum
{
    MPU6050_STAGE_NONE = 0,
    MPU6050_STAGE_PROBE,
    MPU6050_STAGE_WHO_AM_I,
    MPU6050_STAGE_CONFIG,
    MPU6050_STAGE_DATA_READ,
    MPU6050_STAGE_ZERO_FRAME,
    MPU6050_STAGE_SPIKE_REJECT,
    MPU6050_STAGE_RANGE_REJECT,
} Mpu6050_Stage_t;

typedef struct
{
    uint32_t rx_sequence;
    uint32_t sample_time_ms;
    float accel_g[3];
    float gyro_dps[3];
    float temperature_c;
    uint32_t error_count;
} Mpu6050_Snapshot_t;

/* Lightweight status (no measurement payload); never touches the I2C bus. */
typedef struct
{
    uint32_t rx_sequence;
    uint32_t sample_time_ms;
    uint32_t error_count;
    Mpu6050_Result_t last_result;
    Mpu6050_Stage_t last_stage;
    uint32_t reinit_count;
    uint8_t last_chip_id;
    uint8_t i2c_addr_7bit;
    uint8_t last_bsp_status;
} Mpu6050_Status_t;

/**
 * @brief Initialize the MPU6050 bus, verify chip identity, and configure ranges.
 */
Mpu6050_Result_t Sensor_MPU6050_Init(void);

/**
 * @brief Service the MPU6050: read one sample and cache it. Caller supplies the cycle timestamp.
 */
Mpu6050_Result_t Sensor_MPU6050_Service(uint32_t now_ms);

/**
 * @brief Copy the latest cached MPU6050 snapshot already produced by Service().
 */
Mpu6050_Result_t Sensor_MPU6050_CopySnapshot(Mpu6050_Snapshot_t *out);

/**
 * @brief Copy the compact MPU6050 status (no measurement payload, no bus access).
 */
Mpu6050_Result_t Sensor_MPU6050_GetStatus(Mpu6050_Status_t *out);

/**
 * @brief Request a re-init (cheap; performed on the next Service call).
 */
void Sensor_MPU6050_RequestReinit(void);

#endif
