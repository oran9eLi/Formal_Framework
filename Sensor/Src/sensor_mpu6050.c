/**
 * @file sensor_mpu6050.c
 * @brief Implement the typed MPU6050 six-axis sensor driver.
 */

#include "sensor_mpu6050.h"

#include "bsp_i2c.h"
#include "bsp_time.h"

#include <string.h>

#define MPU6050_I2C_ADDR_PRIMARY       (0x68U << 1)
#define MPU6050_I2C_ADDR_SECONDARY     (0x69U << 1)

#define MPU6050_REG_SMPLRT_DIV         0x19U
#define MPU6050_REG_CONFIG             0x1AU
#define MPU6050_REG_GYRO_CONFIG        0x1BU
#define MPU6050_REG_ACCEL_CONFIG       0x1CU
#define MPU6050_REG_ACCEL_XOUT_H       0x3BU
#define MPU6050_REG_PWR_MGMT_1         0x6BU
#define MPU6050_REG_WHO_AM_I           0x75U

#define MPU6050_CHIP_ID                0x68U
#define MPU6050_WAKEUP_VALUE           0x00U
#define MPU6050_SAMPLE_RATE_DIV        0x09U
#define MPU6050_DLPF_CFG               0x03U
#define MPU6050_GYRO_RANGE_250DPS      0x00U
#define MPU6050_ACCEL_RANGE_2G         0x00U

#define MPU6050_I2C_TIMEOUT_MS         20U
#define MPU6050_WAKEUP_DELAY_MS        10U
#define MPU6050_INIT_RETRY_FAST_MS     100U
#define MPU6050_INIT_RETRY_SLOW_MS     500U
#define MPU6050_INIT_FAST_RETRY_LIMIT  5U
#define MPU6050_REINIT_FAIL_LIMIT      5U
#define MPU6050_ACCEL_SCALE_2G         16384.0f
#define MPU6050_GYRO_SCALE_250DPS      131.0f
#define MPU6050_TEMP_SCALE             340.0f
#define MPU6050_TEMP_OFFSET_C          36.53f
#define MPU6050_DELTA_SPIKE_FILTER_ENABLE 0U
#if MPU6050_DELTA_SPIKE_FILTER_ENABLE
#define MPU6050_ACCEL_SPIKE_G          0.3f
#define MPU6050_GYRO_SPIKE_DPS         10.0f
#define MPU6050_TEMP_SPIKE_C           2.0f
#endif

static uint16_t s_addr = MPU6050_I2C_ADDR_PRIMARY;
static Mpu6050_Snapshot_t s_snapshot;
static Mpu6050_Result_t s_last_result = MPU6050_RESULT_NO_DATA;
static Mpu6050_Stage_t s_last_stage = MPU6050_STAGE_NONE;
static uint32_t s_reinit_count;
static uint32_t s_next_init_ms;
static uint8_t s_last_chip_id;
static uint8_t s_last_bsp_status;
static uint8_t s_init_fail_count;
static uint8_t s_read_fail_count;
static uint8_t s_initialized;
#if MPU6050_DELTA_SPIKE_FILTER_ENABLE
static uint8_t s_accept_next_sample;
#endif
static volatile uint8_t s_reinit_request;

static int16_t Mpu6050_ReadS16BE(const uint8_t *data, uint16_t offset)
{
    return (int16_t)((uint16_t)(((uint16_t)data[offset] << 8) |
                                data[offset + 1U]));
}

static Mpu6050_Result_t Mpu6050_MapBspStatus(BSP_Status_t status)
{
    s_last_bsp_status = (uint8_t)status;
    if (status == BSP_STATUS_OK)
    {
        return MPU6050_RESULT_OK;
    }
    if (status == BSP_STATUS_TIMEOUT)
    {
        return MPU6050_RESULT_TIMEOUT;
    }
    return MPU6050_RESULT_IO_ERROR;
}

#if MPU6050_DELTA_SPIKE_FILTER_ENABLE
static float Mpu6050_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}
#endif

static uint8_t Mpu6050_IsAllZeroFrame(
    const int16_t accel_raw[3],
    int16_t temperature_raw,
    const int16_t gyro_raw[3])
{
    return ((accel_raw[0] == 0) &&
            (accel_raw[1] == 0) &&
            (accel_raw[2] == 0) &&
            (temperature_raw == 0) &&
            (gyro_raw[0] == 0) &&
            (gyro_raw[1] == 0) &&
            (gyro_raw[2] == 0))
               ? 1U
               : 0U;
}

#if MPU6050_DELTA_SPIKE_FILTER_ENABLE
static uint8_t Mpu6050_HasSampleSpike(
    const Mpu6050_Snapshot_t *candidate)
{
    uint8_t i;

    if ((candidate == 0) || (s_snapshot.rx_sequence == 0U))
    {
        return 0U;
    }

    for (i = 0U; i < 3U; ++i)
    {
        if (Mpu6050_AbsFloat(candidate->accel_g[i] -
                             s_snapshot.accel_g[i]) >
            MPU6050_ACCEL_SPIKE_G)
        {
            return 1U;
        }

        if (Mpu6050_AbsFloat(candidate->gyro_dps[i] -
                             s_snapshot.gyro_dps[i]) >
            MPU6050_GYRO_SPIKE_DPS)
        {
            return 1U;
        }
    }

    return (Mpu6050_AbsFloat(candidate->temperature_c -
                             s_snapshot.temperature_c) >
            MPU6050_TEMP_SPIKE_C)
               ? 1U
               : 0U;
}
#endif

static uint8_t Mpu6050_IsOutOfRange(
    const int16_t accel_raw[3],
    const int16_t gyro_raw[3])
{
    uint8_t i;
    for (i = 0U; i < 3U; i++)
    {
        if ((gyro_raw[i] > 29500) || (gyro_raw[i] < -29500))
        {
            return 1U;
        }
    }
    for (i = 0U; i < 3U; i++)
    {
        if ((accel_raw[i] > 29500) || (accel_raw[i] < -29500))
        {
            return 1U;
        }
    }
    return 0U;
}


static void Mpu6050_ClearSnapshotPreserveErrors(void)
{
    uint32_t error_count = s_snapshot.error_count;

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.error_count = error_count;
}

static void Mpu6050_NoteSampleFailure(uint32_t now_ms,
                                      Mpu6050_Stage_t stage,
                                      Mpu6050_Result_t result)
{
    s_snapshot.error_count++;
    s_last_stage = stage;
    s_last_result = result;
#if MPU6050_DELTA_SPIKE_FILTER_ENABLE
    s_accept_next_sample = 1U;
#endif

    if (stage == MPU6050_STAGE_DATA_READ)
     {

         if (s_read_fail_count < 255U)
         {
             s_read_fail_count++;
         }
         if (s_read_fail_count >= MPU6050_REINIT_FAIL_LIMIT)
         {
             s_initialized = 0U;
             s_reinit_request = 1U;
             s_next_init_ms = now_ms + MPU6050_INIT_RETRY_FAST_MS;
         }
     }
//    if (s_read_fail_count < 255U)
//    {
//        s_read_fail_count++;
//    }
//    if (s_read_fail_count >= MPU6050_REINIT_FAIL_LIMIT)
//    {
//        s_initialized = 0U;
//        s_reinit_request = 1U;
//        s_next_init_ms = now_ms + MPU6050_INIT_RETRY_FAST_MS;
//    }
}

static uint32_t Mpu6050_InitRetryDelayMs(void)
{
    return (s_init_fail_count <= MPU6050_INIT_FAST_RETRY_LIMIT)
               ? MPU6050_INIT_RETRY_FAST_MS
               : MPU6050_INIT_RETRY_SLOW_MS;
}

static Mpu6050_Result_t Mpu6050_TryReinit(uint32_t now_ms)
{
    Mpu6050_Result_t result;

    if ((s_next_init_ms != 0U) &&
        ((int32_t)(now_ms - s_next_init_ms) < 0))
    {
        return MPU6050_RESULT_NO_DATA;
    }

    s_reinit_request = 0U;
    result = Sensor_MPU6050_Init();
    if (result != MPU6050_RESULT_OK)
    {
        if (s_init_fail_count < 255U)
        {
            s_init_fail_count++;
        }
        s_reinit_request = 1U;
        s_next_init_ms = now_ms + Mpu6050_InitRetryDelayMs();
    }
    else
    {
        s_init_fail_count = 0U;
    }
    return result;
}

static Mpu6050_Result_t Mpu6050_ReadReg(
    uint8_t reg,
    uint8_t *data,
    uint16_t len)
{
    return Mpu6050_MapBspStatus(
        BSP_I2C_MemRead(s_addr,
                        reg,
                        data,
                        len,
                        MPU6050_I2C_TIMEOUT_MS));
}

static Mpu6050_Result_t Mpu6050_WriteReg(uint8_t reg, uint8_t value)
{
    return Mpu6050_MapBspStatus(
        BSP_I2C_MemWrite(s_addr,
                         reg,
                         &value,
                         1U,
                         MPU6050_I2C_TIMEOUT_MS));
}

static Mpu6050_Result_t Mpu6050_ProbeAddress(void)
{
    uint8_t retry;

    for (retry = 0U; retry < 3U; retry++)
    {
        if (BSP_I2C_IsDeviceReady(MPU6050_I2C_ADDR_PRIMARY,
                                  MPU6050_I2C_TIMEOUT_MS) == BSP_STATUS_OK)
        {
            s_addr = MPU6050_I2C_ADDR_PRIMARY;
            return MPU6050_RESULT_OK;
        }

        if (BSP_I2C_IsDeviceReady(MPU6050_I2C_ADDR_SECONDARY,
                                  MPU6050_I2C_TIMEOUT_MS) == BSP_STATUS_OK)
        {
            s_addr = MPU6050_I2C_ADDR_SECONDARY;
            return MPU6050_RESULT_OK;
        }

        if (retry < 2U)
        {
            BSP_Time_DelayMs(5U);  // 5ms 冷却, 给芯片/PLL 稳定时间
        }
    }

    return MPU6050_RESULT_IO_ERROR;
}

static Mpu6050_Result_t Mpu6050_CheckChipId(void)
{
    uint8_t chip_id = 0U;
    Mpu6050_Result_t result;

    result = Mpu6050_ReadReg(MPU6050_REG_WHO_AM_I, &chip_id, 1U);
    s_last_chip_id = chip_id;
    if (result != MPU6050_RESULT_OK)
    {
        return result;
    }

    return (chip_id == MPU6050_CHIP_ID)
               ? MPU6050_RESULT_OK
               : MPU6050_RESULT_BAD_ID;
}

static Mpu6050_Result_t Mpu6050_Configure(void)
{
    if ((Mpu6050_WriteReg(MPU6050_REG_PWR_MGMT_1,
                          MPU6050_WAKEUP_VALUE) != MPU6050_RESULT_OK) ||
        (Mpu6050_WriteReg(MPU6050_REG_SMPLRT_DIV,
                          MPU6050_SAMPLE_RATE_DIV) != MPU6050_RESULT_OK) ||
        (Mpu6050_WriteReg(MPU6050_REG_CONFIG,
                          MPU6050_DLPF_CFG) != MPU6050_RESULT_OK) ||
        (Mpu6050_WriteReg(MPU6050_REG_GYRO_CONFIG,
                          MPU6050_GYRO_RANGE_250DPS) != MPU6050_RESULT_OK) ||
        (Mpu6050_WriteReg(MPU6050_REG_ACCEL_CONFIG,
                          MPU6050_ACCEL_RANGE_2G) != MPU6050_RESULT_OK))
    {
        return MPU6050_RESULT_IO_ERROR;
    }

    BSP_Time_DelayMs(MPU6050_WAKEUP_DELAY_MS);
    return MPU6050_RESULT_OK;
}

Mpu6050_Result_t Sensor_MPU6050_Init(void)
{
    Mpu6050_Result_t result;

    s_initialized = 0U;
    Mpu6050_ClearSnapshotPreserveErrors();
    s_read_fail_count = 0U;
#if MPU6050_DELTA_SPIKE_FILTER_ENABLE
    s_accept_next_sample = 1U;
#endif
    s_reinit_count++;

    s_last_stage = MPU6050_STAGE_PROBE;
    result = Mpu6050_ProbeAddress();
    if (result != MPU6050_RESULT_OK)
    {
        s_last_result = result;
        return result;
    }

    s_last_stage = MPU6050_STAGE_WHO_AM_I;
    result = Mpu6050_CheckChipId();
    if (result != MPU6050_RESULT_OK)
    {
        s_last_result = result;
        return result;
    }

    s_last_stage = MPU6050_STAGE_CONFIG;
    result = Mpu6050_Configure();
    if (result != MPU6050_RESULT_OK)
    {
        s_last_result = result;
        return result;
    }

    s_initialized = 1U;
    s_next_init_ms = 0U;
    s_init_fail_count = 0U;
    s_last_stage = MPU6050_STAGE_NONE;
    s_last_result = MPU6050_RESULT_OK;
    return MPU6050_RESULT_OK;
}

void Sensor_MPU6050_RequestReinit(void)
{
    s_reinit_request = 1U;
}

Mpu6050_Result_t Sensor_MPU6050_Service(uint32_t now_ms)
{
    uint8_t data[14];
    int16_t accel_raw[3];
    int16_t temperature_raw;
    int16_t gyro_raw[3];
    Mpu6050_Result_t result;
    Mpu6050_Snapshot_t candidate;

    if (s_reinit_request != 0U)
    {
        result = Mpu6050_TryReinit(now_ms);
        if (result != MPU6050_RESULT_OK)
        {
            return (result == MPU6050_RESULT_NO_DATA)
                       ? MPU6050_RESULT_NO_DATA
                       : result;
        }
    }
    if (s_initialized == 0U)
    {
        return MPU6050_RESULT_NO_DATA;
    }

    s_last_stage = MPU6050_STAGE_DATA_READ;
    result = Mpu6050_ReadReg(MPU6050_REG_ACCEL_XOUT_H,
                             data,
                             sizeof(data));
    if (result != MPU6050_RESULT_OK)
    {
        Mpu6050_NoteSampleFailure(now_ms,
                                  MPU6050_STAGE_DATA_READ,
                                  result);
        return result;
    }

    accel_raw[0] = Mpu6050_ReadS16BE(data, 0U);
    accel_raw[1] = Mpu6050_ReadS16BE(data, 2U);
    accel_raw[2] = Mpu6050_ReadS16BE(data, 4U);
    temperature_raw = Mpu6050_ReadS16BE(data, 6U);
    gyro_raw[0] = Mpu6050_ReadS16BE(data, 8U);
    gyro_raw[1] = Mpu6050_ReadS16BE(data, 10U);
    gyro_raw[2] = Mpu6050_ReadS16BE(data, 12U);

    if (Mpu6050_IsAllZeroFrame(accel_raw,
                               temperature_raw,
                               gyro_raw) != 0U)
    {
        Mpu6050_NoteSampleFailure(now_ms,
                                  MPU6050_STAGE_ZERO_FRAME,
                                  MPU6050_RESULT_IO_ERROR);
        return MPU6050_RESULT_IO_ERROR;
    }

     if (Mpu6050_IsOutOfRange(accel_raw, gyro_raw) != 0U)
    {
       Mpu6050_NoteSampleFailure(now_ms,
                                  MPU6050_STAGE_RANGE_REJECT,
                                  MPU6050_RESULT_IO_ERROR);
       return MPU6050_RESULT_IO_ERROR;
    }

    candidate = s_snapshot;
    candidate.rx_sequence++;
    if (candidate.rx_sequence == 0U)
    {
        candidate.rx_sequence = 1U;
    }
    candidate.sample_time_ms = now_ms;
    candidate.accel_g[0] =
        ((float)accel_raw[0]) / MPU6050_ACCEL_SCALE_2G;
    candidate.accel_g[1] =
        ((float)accel_raw[1]) / MPU6050_ACCEL_SCALE_2G;
    candidate.accel_g[2] =
        ((float)accel_raw[2]) / MPU6050_ACCEL_SCALE_2G;
    candidate.gyro_dps[0] =
        ((float)gyro_raw[0]) / MPU6050_GYRO_SCALE_250DPS;
    candidate.gyro_dps[1] =
        ((float)gyro_raw[1]) / MPU6050_GYRO_SCALE_250DPS;
    candidate.gyro_dps[2] =
        ((float)gyro_raw[2]) / MPU6050_GYRO_SCALE_250DPS;
    candidate.temperature_c =
        (((float)temperature_raw) / MPU6050_TEMP_SCALE) +
        MPU6050_TEMP_OFFSET_C;

#if MPU6050_DELTA_SPIKE_FILTER_ENABLE
    if ((s_accept_next_sample == 0U) &&
        (Mpu6050_HasSampleSpike(&candidate) != 0U))
    {
        Mpu6050_NoteSampleFailure(now_ms,
                                  MPU6050_STAGE_SPIKE_REJECT,
                                  MPU6050_RESULT_IO_ERROR);
        return MPU6050_RESULT_IO_ERROR;
    }
#endif

    s_snapshot = candidate;
    s_read_fail_count = 0U;
#if MPU6050_DELTA_SPIKE_FILTER_ENABLE
    s_accept_next_sample = 0U;
#endif
    s_last_stage = MPU6050_STAGE_NONE;
    s_last_result = MPU6050_RESULT_OK;
    return MPU6050_RESULT_OK;
}

Mpu6050_Result_t Sensor_MPU6050_CopySnapshot(Mpu6050_Snapshot_t *out)
{
    if (out == 0)
    {
        return MPU6050_RESULT_INVALID_PARAM;
    }

    *out = s_snapshot;
    return (s_snapshot.rx_sequence != 0U)
               ? MPU6050_RESULT_OK
               : MPU6050_RESULT_NO_DATA;
}

Mpu6050_Result_t Sensor_MPU6050_GetStatus(Mpu6050_Status_t *out)
{
    if (out == 0)
    {
        return MPU6050_RESULT_INVALID_PARAM;
    }

    out->rx_sequence = s_snapshot.rx_sequence;
    out->sample_time_ms = s_snapshot.sample_time_ms;
    out->error_count = s_snapshot.error_count;
    out->last_result = s_last_result;
    out->last_stage = s_last_stage;
    out->reinit_count = s_reinit_count;
    out->last_chip_id = s_last_chip_id;
    out->i2c_addr_7bit = (uint8_t)(s_addr >> 1);
    out->last_bsp_status = s_last_bsp_status;
    return (s_snapshot.rx_sequence != 0U)
               ? MPU6050_RESULT_OK
               : MPU6050_RESULT_NO_DATA;
}
