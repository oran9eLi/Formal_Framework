/**
 * @file sensor_power.h
 * @brief 板载电源/电池检测强类型驱动接口。
 *
 * @details
 * 本驱动负责通过 ADC 获取电压相关测量，并维护电量百分比和低电压事实。
 * 驱动层不直接发布 Framework topic，也不负责系统级告警仲裁。
 */

#ifndef SENSOR_POWER_H
#define SENSOR_POWER_H

#include <stdint.h>

/**
 * @brief 电源检测驱动返回值。
 */
typedef enum {
  POWER_RESULT_OK = 0,       /**< 操作成功。 */
  POWER_RESULT_NO_DATA,      /**< 当前无可用新样本。 */
  POWER_RESULT_IO_ERROR,     /**< ADC 或 BSP 访问失败。 */
  POWER_RESULT_INVALID_PARAM /**< 参数非法。 */
} Power_Result_t;

/**
 * @brief 电源检测最近一次有效采样快照。
 */
typedef struct {
  uint32_t rx_sequence;    /**< 样本序号，每次有效采样递增。 */
  uint32_t sample_time_ms; /**< 采样完成时间，单位：ms。 */
  float voltage_v;         /**< 电压，单位：V。 */
  int32_t current_ma;      /**< 电流，单位：mA；电流计未接入或采样失败时为 0。 */
  uint32_t power_mw;       /**< 功率，单位：mW，由电压和电流计算得到。 */
  uint8_t percent;         /**< 电量百分比，范围：0 到 100。 */
  uint8_t low_voltage;     /**< 低电压标志，1 表示低电压。 */
  uint16_t reserved;       /**< 保留字段，保持结构体对齐。 */
  uint32_t error_count;    /**< 累计错误次数。 */
} Power_Snapshot_t;

/**
 * @brief 电源检测轻量状态。
 *
 * @details
 * 状态查询不携带原始 ADC 负载，不访问 ADC，只复制驱动内部缓存。
 */
typedef struct {
  uint32_t rx_sequence;    /**< 最近有效样本序号。 */
  uint32_t sample_time_ms; /**< 最近有效采样时间，单位：ms。 */
  uint32_t error_count;    /**< 累计错误次数。 */
  int32_t current_ma;      /**< 最近一次电流，单位：mA。 */
  uint32_t power_mw;       /**< 最近一次功率，单位：mW。 */
  uint8_t percent;         /**< 电量百分比，范围：0 到 100。 */
  uint8_t low_voltage;     /**< 低电压标志，1 表示低电压。 */
  uint16_t reserved;       /**< 保留字段，保持结构体对齐。 */
} Power_Status_t;

/**
 * @brief 初始化板载电源检测 ADC 驱动。
 *
 * @return 初始化结果。
 */
Power_Result_t Sensor_Power_Init(void);

/**
 * @brief 执行一次电源检测周期服务，读取并缓存一个样本。
 *
 * @param[in] now_ms 当前 sensor 周期时间，单位：ms。
 *
 * @return 服务结果。
 *
 * @note 本函数由 sensor 任务调用，不在 ISR 中调用。
 */
Power_Result_t Sensor_Power_Service(uint32_t now_ms);

/**
 * @brief 复制最近一次由 `Sensor_Power_Service()` 生成的样本。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 */
Power_Result_t Sensor_Power_CopySnapshot(Power_Snapshot_t *out);

/**
 * @brief 复制电源检测轻量状态。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 *
 * @note 本函数不访问 ADC。
 */
Power_Result_t Sensor_Power_GetStatus(Power_Status_t *out);

/**
 * @brief 请求电源检测模块在下一次 Service 中重新初始化。
 *
 * @note 本函数只置位请求标志，适合 recovery 回调调用。
 */
void Sensor_Power_RequestReinit(void);

#endif
