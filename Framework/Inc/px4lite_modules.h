/**
 * @file px4lite_modules.h
 * @brief Framework 传感器、估计器、通信、健康和模块状态服务接口。
 *
 * @details
 * 本文件暴露 Framework 固定周期任务调用的服务函数。普通传感器由 `sensor`
 * 任务统一采集；Health 独占 OFFLINE/FAILED 判定；Business 和 Display 只能通过
 * 应用 API 消费快照。
 */

#ifndef PX4LITE_MODULES_H
#define PX4LITE_MODULES_H

#include "px4lite_types.h"

/**
 * @brief 复位 Framework 模块状态、统计计数和启动时间。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_ModulesInit(void);

/**
 * @brief 初始化 GNSS 模块并发布初始状态。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_GnssModuleInit(void);

/**
 * @brief 初始化 IMU 模块并发布初始状态。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_ImuModuleInit(void);

/**
 * @brief 初始化气压计模块并发布初始状态。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_BaroModuleInit(void);

/**
 * @brief 初始化电源/电池模块并发布初始状态。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_BatteryModuleInit(void);

/**
 * @brief 初始化告警模块。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_AlarmModuleInit(void);

/**
 * @brief 请求 GNSS 模块恢复。
 *
 * @return 请求结果。
 *
 * @note recovery 回调只置位请求标志，不执行 UART/I2C/SPI 等总线 I/O。
 */
Px4Lite_Result_t Px4Lite_GnssRecover(void);

/**
 * @brief 请求 IMU 模块恢复。
 *
 * @return 请求结果。
 */
Px4Lite_Result_t Px4Lite_ImuRecover(void);

/**
 * @brief 请求气压计模块恢复。
 *
 * @return 请求结果。
 */
Px4Lite_Result_t Px4Lite_BaroRecover(void);

/**
 * @brief 请求电源/电池模块恢复。
 *
 * @return 请求结果。
 */
Px4Lite_Result_t Px4Lite_BatteryRecover(void);

/**
 * @brief 请求 LoRa 模块恢复。
 *
 * @return 请求结果。
 */
Px4Lite_Result_t Px4Lite_LoraRecover(void);

/**
 * @brief 执行一次非阻塞传感器采集和 Framework 发布周期。
 *
 * @param[in] now_ms 当前 sensor 周期时间，单位：ms。
 *
 * @note 本函数由 `sensor` 任务以 10 ms 周期调用，是普通传感器唯一硬件采集入口。
 */
void Px4Lite_SensorWorkRun(uint32_t now_ms);

/**
 * @brief 初始化估计器模块状态。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_EstimatorInit(void);

/**
 * @brief 执行一次估计器周期，将新鲜测量转换为导航快照。
 *
 * @param[in] now_ms 当前 estimator 周期时间，单位：ms。
 *
 * @note 当前负责 GNSS 到 Navigation、IMU 到 Attitude 的基础转换。
 */
void Px4Lite_EstimatorRun(uint32_t now_ms);

/**
 * @brief 请求按当前姿态重做水平校准（把当前 roll/pitch 记为零位）。
 *
 * @note 仅置位请求标志，下一次 Px4Lite_EstimatorRun 消费，线程安全。
 */
void Px4Lite_RequestAttitudeLevelCalibration(void);

/**
 * @brief 初始化 LoRa/comm 模块和 MAVLink 发送器。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_CommModulesInit(void);

/**
 * @brief 执行一次非阻塞通信周期。
 *
 * @param[in] now_ms 当前 comm 周期时间，单位：ms。
 *
 * @note 本函数处理 LoRa RX 解析和 MAVLink TX 调度，单周期最多提交有限工作。
 */
void Px4Lite_CommWorkRun(uint32_t now_ms);

/**
 * @brief 复制聚合后的 LoRa 和 MAVLink 发送诊断统计。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 */
void Px4Lite_GetCommDebugInfo(Px4Lite_CommDebugInfo_t *out);

/**
 * @brief 初始化健康监控服务。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_HealthInit(void);

/**
 * @brief 执行一次健康监控周期并发布系统健康快照。
 *
 * @param[in] now_ms 当前 health 周期时间，单位：ms。
 *
 * @note Health task 统一负责模块超时、OFFLINE/FAILED 判定和 watchdog 喂狗门控。
 */
void Px4Lite_HealthRun(uint32_t now_ms);

/**
 * @brief 在 Framework 同步保护下复制单个模块状态。
 *
 * @param[in] module_id 模块编号。
 * @param[out] status 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 */
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleStatus_t *status);

/**
 * @brief 复制模块状态数组和对应版本号。
 *
 * @param[out] status 输出数组，不能为 NULL。
 * @param[in] count 输出数组容量，应不小于 `PX4LITE_MODULE_COUNT`。
 * @param[out] version 状态版本号输出，不能为 NULL。
 *
 * @return 复制结果。
 *
 * @note 调用方可结合 `Px4Lite_GetStatusVersion()` 做版本一致性确认。
 */
Px4Lite_Result_t Px4Lite_CopyModuleStatuses(Px4Lite_ModuleStatus_t *status, uint16_t count, uint32_t *version);

/**
 * @brief 获取当前模块状态版本号。
 *
 * @return 当前状态版本号。
 */
uint32_t Px4Lite_GetStatusVersion(void);

/**
 * @brief 允许外部服务适配器更新对应 Framework 模块状态。
 *
 * @param[in] module_id 模块编号。
 * @param[in] state 新的公开状态。
 * @param[in] fault_code 故障码，0 表示无故障。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @note 该接口用于 Display/Storage 等外部服务汇报状态，不应用于绕过 Health 策略。
 */
void Px4Lite_SetExternalModuleState(Px4Lite_ModuleId_t module_id, Px4Lite_State_t state, uint16_t fault_code, uint32_t now_ms);

#endif
