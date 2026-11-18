/**
 * @file sensor_power2.h
 * @brief 第二块电池电源检测强类型驱动接口。
 *
 * @details
 * 本驱动通过BSP_ADC2获取逻辑第二组电压，通过ADC3的BATTERY2通道获取电流。
 * 物理路由由BSP选择：默认扩展板PA5/PC0，独立线兼容路由PA4/PC1；两者均使用相同的
 * POWER2_* 独立电量曲线、滤波、电流换算和低电压确认配置，不修改topic或协议组号。
 * 驱动层只记录硬件事实，不直接发布 Framework topic，也不负责系统级告警策略。
 */

#ifndef SENSOR_POWER2_H
#define SENSOR_POWER2_H

#include <stdint.h>

#include "sensor_power.h"

/**
 * @brief 初始化第二块电池电源检测驱动。
 *
 * @return 初始化结果。
 */
Power_Result_t Sensor_Power2_Init(void);

/**
 * @brief 执行一次第二块电池检测服务，读取并缓存一个样本。
 *
 * @param[in] now_ms 当前 sensor 周期时间，单位：ms。
 *
 * @return 服务结果。
 *
 * @note 本函数由 sensor 任务调用，不在 ISR 中调用。
 */
Power_Result_t Sensor_Power2_Service(uint32_t now_ms);

/**
 * @brief 复制最近一次由 `Sensor_Power2_Service()` 生成的样本。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 */
Power_Result_t Sensor_Power2_CopySnapshot(Power_Snapshot_t *out);

/**
 * @brief 复制第二块电池检测轻量状态。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 *
 * @note 本函数不访问 ADC。
 */
Power_Result_t Sensor_Power2_GetStatus(Power_Status_t *out);

/**
 * @brief 请求第二块电池检测模块在下一次 Service 中重新初始化。
 *
 * @note 本函数只置位请求标志，适合 recovery 回调调用。
 */
void Sensor_Power2_RequestReinit(void);

#endif
