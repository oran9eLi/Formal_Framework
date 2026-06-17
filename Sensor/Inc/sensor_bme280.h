/**
 * @file sensor_bme280.h
 * @brief BME280 气压计/环境传感器强类型驱动接口。
 *
 * @details
 * 本驱动负责 BME280 的芯片识别、补偿参数读取、强制测量和工程值换算。
 * 驱动层只记录设备事实和采样结果，不直接发布 Framework topic。
 */

#ifndef SENSOR_BME280_H
#define SENSOR_BME280_H

#include <stdint.h>

/**
 * @brief BME280 驱动返回值。
 */
typedef enum
{
    BME280_RESULT_OK = 0,       /**< 操作成功。 */
    BME280_RESULT_NO_DATA,      /**< 当前无可用新样本。 */
    BME280_RESULT_BAD_ID,       /**< 芯片 ID 不匹配。 */
    BME280_RESULT_IO_ERROR,     /**< I2C 访问失败。 */
    BME280_RESULT_TIMEOUT,      /**< 测量等待超时。 */
    BME280_RESULT_INVALID_PARAM /**< 参数非法。 */
} Bme280_Result_t;

/**
 * @brief BME280 最近一次有效环境采样快照。
 */
typedef struct
{
    uint32_t rx_sequence;           /**< 样本序号，每次有效采样递增。 */
    uint32_t sample_time_ms;        /**< 采样完成时间，单位：ms。 */
    float temperature_c;            /**< 温度，单位：摄氏度。 */
    float pressure_pa;              /**< 气压，单位：Pa。 */
    float relative_humidity_pct;    /**< 相对湿度，单位：%。 */
    uint32_t error_count;           /**< 累计错误次数。 */
} Bme280_Snapshot_t;

/**
 * @brief BME280 轻量状态。
 *
 * @details
 * 状态查询不携带测量负载，不访问 I2C 总线，只复制驱动内部缓存。
 */
typedef struct
{
    uint32_t rx_sequence;    /**< 最近有效样本序号。 */
    uint32_t sample_time_ms; /**< 最近有效采样时间，单位：ms。 */
    uint32_t error_count;    /**< 累计错误次数。 */
} Bme280_Status_t;

/**
 * @brief 初始化 BME280 总线访问、校验芯片 ID 并读取补偿参数。
 *
 * @return 初始化结果。
 */
Bme280_Result_t Sensor_BME280_Init(void);

/**
 * @brief 执行一次 BME280 周期服务，触发强制测量并缓存结果。
 *
 * @param[in] now_ms 当前 sensor 周期时间，单位：ms。
 *
 * @return 服务结果。
 *
 * @note 本函数由 sensor 任务调用，不在 ISR 中调用。
 */
Bme280_Result_t Sensor_BME280_Service(uint32_t now_ms);

/**
 * @brief 复制最近一次由 `Sensor_BME280_Service()` 生成的样本。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 */
Bme280_Result_t Sensor_BME280_CopySnapshot(Bme280_Snapshot_t *out);

/**
 * @brief 复制 BME280 轻量状态。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 *
 * @note 本函数不访问 I2C 总线。
 */
Bme280_Result_t Sensor_BME280_GetStatus(Bme280_Status_t *out);

/**
 * @brief 请求 BME280 在下一次 Service 中重新初始化。
 *
 * @note 本函数只置位请求标志，适合 recovery 回调调用。
 */
void Sensor_BME280_RequestReinit(void);

#endif
