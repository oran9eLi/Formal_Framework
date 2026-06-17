/**
 * @file sensor_mpu6050.h
 * @brief MPU6050 六轴 IMU 强类型驱动接口。
 *
 * @details
 * 本驱动负责 MPU6050 的芯片识别、寄存器配置、采样读取和基础异常记录。
 * 驱动可执行自身的轻量自动 reinit，但 OFFLINE/FAILED 公开状态仍由 Health task 判定。
 */

#ifndef SENSOR_MPU6050_H
#define SENSOR_MPU6050_H

#include <stdint.h>

/**
 * @brief MPU6050 驱动返回值。
 */
typedef enum
{
    MPU6050_RESULT_OK = 0,      /**< 操作成功。 */
    MPU6050_RESULT_NO_DATA,     /**< 当前无可用新样本。 */
    MPU6050_RESULT_BAD_ID,      /**< WHO_AM_I 芯片 ID 不匹配。 */
    MPU6050_RESULT_IO_ERROR,    /**< I2C 访问失败。 */
    MPU6050_RESULT_TIMEOUT,     /**< 设备响应或采样等待超时。 */
    MPU6050_RESULT_INVALID_PARAM /**< 参数非法。 */
} Mpu6050_Result_t;

/**
 * @brief 最近一次 MPU6050 操作阶段。
 */
typedef enum
{
    MPU6050_STAGE_NONE = 0,  /**< 尚未执行操作。 */
    MPU6050_STAGE_PROBE,     /**< 探测设备阶段。 */
    MPU6050_STAGE_WHO_AM_I,  /**< 读取 WHO_AM_I 阶段。 */
    MPU6050_STAGE_CONFIG,    /**< 配置寄存器阶段。 */
    MPU6050_STAGE_DATA_READ, /**< 读取数据阶段。 */
    MPU6050_STAGE_ZERO_FRAME,    /**< 拒绝全零帧阶段。 */
    MPU6050_STAGE_SPIKE_REJECT, /**< 拒绝尖峰数据阶段。 */
    MPU6050_STAGE_RANGE_REJECT, /**< 拒绝超范围数据阶段。 */
} Mpu6050_Stage_t;

/**
 * @brief MPU6050 最近一次有效采样快照。
 */
typedef struct
{
    uint32_t rx_sequence;    /**< 样本序号，每次有效采样递增。 */
    uint32_t sample_time_ms; /**< 采样完成时间，单位：ms。 */
    float accel_g[3];        /**< X/Y/Z 加速度，单位：g。 */
    float gyro_dps[3];       /**< X/Y/Z 角速度，单位：degree/s。 */
    float temperature_c;     /**< 芯片温度，单位：摄氏度。 */
    uint32_t error_count;    /**< 累计错误次数。 */
} Mpu6050_Snapshot_t;

/**
 * @brief MPU6050 轻量状态。
 *
 * @details
 * 状态查询不携带测量负载，不访问 I2C 总线，只复制驱动内部缓存。
 */
typedef struct
{
    uint32_t rx_sequence;          /**< 最近有效样本序号。 */
    uint32_t sample_time_ms;       /**< 最近有效采样时间，单位：ms。 */
    uint32_t error_count;          /**< 累计错误次数。 */
    Mpu6050_Result_t last_result;  /**< 最近一次驱动返回值。 */
    Mpu6050_Stage_t last_stage;    /**< 最近一次操作阶段。 */
    uint32_t reinit_count;         /**< 驱动内部重新初始化次数。 */
    uint8_t last_chip_id;          /**< 最近读取到的 WHO_AM_I 芯片 ID。 */
    uint8_t i2c_addr_7bit;         /**< 7 bit I2C 地址。 */
    uint8_t last_bsp_status;       /**< 最近一次 BSP I2C 状态。 */
} Mpu6050_Status_t;

/**
 * @brief 初始化 MPU6050 总线访问、校验芯片 ID 并配置量程。
 *
 * @return 初始化结果。
 */
Mpu6050_Result_t Sensor_MPU6050_Init(void);

/**
 * @brief 执行一次 MPU6050 周期服务，读取并缓存一个样本。
 *
 * @param[in] now_ms 当前 sensor 周期时间，单位：ms。
 *
 * @return 服务结果。
 *
 * @note 本函数由 sensor 任务调用，不在 ISR 中调用。
 */
Mpu6050_Result_t Sensor_MPU6050_Service(uint32_t now_ms);

/**
 * @brief 复制最近一次由 `Sensor_MPU6050_Service()` 生成的样本。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 */
Mpu6050_Result_t Sensor_MPU6050_CopySnapshot(Mpu6050_Snapshot_t *out);

/**
 * @brief 复制 MPU6050 轻量状态。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 *
 * @note 本函数不访问 I2C 总线。
 */
Mpu6050_Result_t Sensor_MPU6050_GetStatus(Mpu6050_Status_t *out);

/**
 * @brief 请求 MPU6050 在下一次 Service 中重新初始化。
 *
 * @note 本函数只置位请求标志，适合 recovery 回调调用。
 */
void Sensor_MPU6050_RequestReinit(void);

#endif
