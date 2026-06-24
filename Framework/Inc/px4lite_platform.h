/**
 * @file px4lite_platform.h
 * @brief MCU 平台适配、硬件服务和任务心跳接口。
 *
 * @details
 * Platform Adapter 是 Framework 层接触 BSP 的唯一边界。Framework 其他文件不得
 * 包含 BSP 头文件，也不得直接调用 HAL/BSP。所有 Driver 类型到 Framework 类型的
 * 转换都应在平台适配层完成。
 */

#ifndef PX4LITE_PLATFORM_H
#define PX4LITE_PLATFORM_H

#include "px4lite_types.h"

/**
 * @brief 必须参与 watchdog 门控的任务心跳编号。
 */
typedef enum {
  PX4LITE_HEARTBEAT_SENSOR = 0, /**< sensor 采集任务心跳。 */
  PX4LITE_HEARTBEAT_ESTIMATOR,  /**< estimator 姿态/导航任务心跳。 */
  PX4LITE_HEARTBEAT_HEALTH,     /**< health 健康监控任务心跳。 */
  PX4LITE_HEARTBEAT_SYSTEM,     /**< biz_system 系统业务任务心跳。 */
  PX4LITE_HEARTBEAT_BUSINESS,   /**< biz_acq 业务采集任务心跳。 */
  PX4LITE_HEARTBEAT_DISPLAY,    /**< biz_display 显示业务任务心跳。 */
  PX4LITE_HEARTBEAT_COMM,       /**< comm 通信任务心跳。 */
  PX4LITE_HEARTBEAT_COUNT       /**< 心跳数量，必须保持为最后一项。 */
} Px4Lite_HeartbeatId_t;

/**
 * @brief 获取平台单调毫秒时间。
 *
 * @return 当前系统毫秒时间，单位：ms。
 *
 * @note Framework、Business、Sensor 的 `_ms` 字段应使用该时间基准或
 * `BSP_Time_GetTickMs()`，不得混用 RTOS ticks。
 */
uint32_t Px4Lite_PlatformGetMs(void);

/**
 * @brief 获取平台微秒时间戳。
 *
 * @return 当前平台微秒时间，单位：us。
 *
 * @note 该时间戳由 HAL tick 与 TIM6 组合得到，用于短周期诊断和调试统计。
 */
uint32_t Px4Lite_PlatformGetUs(void);

/**
 * @brief 记录一个必需任务最近一次成功执行时间。
 *
 * @param[in] id 心跳编号，必须小于 `PX4LITE_HEARTBEAT_COUNT`。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @note Debug 和 Storage 当前不纳入 watchdog 心跳集合。
 */
void Px4Lite_PlatformHeartbeat(Px4Lite_HeartbeatId_t id, uint32_t now_ms);

/**
 * @brief 检查所有必需任务心跳是否存在且未超时。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 1 表示全部必需任务心跳健康，0 表示至少一个心跳缺失或超时。
 */
uint8_t Px4Lite_PlatformHeartbeatsHealthy(uint32_t now_ms);

/**
 * @brief 在所有必需任务心跳健康时刷新硬件看门狗。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @note 调用方不应绕过本函数直接喂狗。
 */
void Px4Lite_PlatformWatchdogFeed(uint32_t now_ms);

/**
 * @brief 初始化平台适配层拥有的服务并复位心跳状态。
 *
 * @return 初始化结果。
 * @retval PX4LITE_OK 初始化成功。
 */
Px4Lite_Result_t Px4Lite_PlatformInit(void);

/**
 * @brief 通过平台适配层读取 RTC UTC 日期时间。
 *
 * @param[out] out 输出日期时间，不能为 NULL。
 *
 * @return 读取结果。
 */
Px4Lite_Result_t Px4Lite_PlatformRtcRead(Px4Lite_UtcDateTime_t *out);

/**
 * @brief 通过平台适配层写入 RTC UTC 日期时间。
 *
 * @param[in] date_time 待写入日期时间，不能为 NULL。
 *
 * @return 写入结果。
 */
Px4Lite_Result_t Px4Lite_PlatformRtcWrite(const Px4Lite_UtcDateTime_t *date_time);

/**
 * @brief 通过平台适配层初始化 GNSS BSP 和强类型 GNSS 驱动。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_GnssInit(void);

/**
 * @brief 将最新 GNSS 驱动快照转换为 Framework GNSS 测量。
 *
 * @param[out] measurement 输出缓冲区，不能为 NULL。
 *
 * @return 转换结果。
 * @retval PX4LITE_OK 成功输出测量。
 * @retval PX4LITE_INVALID_PARAM 参数为空。
 * @retval PX4LITE_NOT_READY 驱动暂无新数据或无有效定位。
 */
Px4Lite_Result_t Px4Lite_GnssRead(Px4Lite_SensorGnss_t *measurement);

/**
 * @brief 初始化 MPU6050 IMU 驱动。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_ImuInit(void);

/**
 * @brief 将最新 MPU6050 快照转换为 Framework IMU 测量。
 *
 * @param[out] measurement 输出缓冲区，不能为 NULL。
 *
 * @return 转换结果。
 */
Px4Lite_Result_t Px4Lite_ImuRead(Px4Lite_SensorImu_t *measurement);

/**
 * @brief 初始化 BME280 气压计/环境驱动。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_BaroInit(void);

/**
 * @brief 将最新 BME280 快照转换为 Framework 气压计测量。
 *
 * @param[out] measurement 输出缓冲区，不能为 NULL。
 *
 * @return 转换结果。
 */
Px4Lite_Result_t Px4Lite_BaroRead(Px4Lite_SensorBaro_t *measurement);

/**
 * @brief 初始化电源检测 ADC 驱动。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_BatteryInit(void);

/**
 * @brief 将最新电源驱动快照转换为 Framework 电池状态。
 *
 * @param[out] measurement 输出缓冲区，不能为 NULL。
 *
 * @return 转换结果。
 */
Px4Lite_Result_t Px4Lite_BatteryRead(Px4Lite_BatteryStatus_t *measurement);

/**
 * @brief 将第二块电池(PA4/ADC2_IN4)最新电源驱动快照转换为 Framework 电池状态。
 *
 * @param[out] measurement 输出缓冲区，不能为 NULL。
 *
 * @return 转换结果；无新样本时返回 PX4LITE_IDLE。
 *
 * @note 与 Px4Lite_BatteryRead 对等，由 battery2 工作块周期调用并发布到 battery2 topic。
 */
Px4Lite_Result_t Px4Lite_Battery2Read(Px4Lite_BatteryStatus_t *measurement);

/**
 * @brief 初始化电机 PWM 输出并强制进入安全脉宽。
 *
 * @return 初始化结果。
 *
 * @note 本接口是 Framework 访问 BSP PWM 的唯一入口。
 */
Px4Lite_Result_t Px4Lite_MotorInit(void);

/**
 * @brief 写入单路电机 PWM 高电平脉宽。
 *
 * @param[in] channel 电机通道，范围 0 到 `PX4LITE_MOTOR_COUNT - 1`。
 * @param[in] pulse_us 高电平脉宽，单位：us。
 *
 * @return 写入结果。
 */
Px4Lite_Result_t Px4Lite_MotorWritePulseUs(uint8_t channel, uint16_t pulse_us);

/**
 * @brief 将所有电机输出强制写为 ESC 最小脉宽。
 *
 * @return 写入结果。
 */
Px4Lite_Result_t Px4Lite_MotorDisarmAll(void);

/**
 * @brief 板载按键逻辑编号。
 */
typedef enum {
  PX4LITE_BUTTON_WKUP = 0, /**< WKUP 按键。 */
  PX4LITE_BUTTON_KEY0,     /**< KEY0 按键。 */
  PX4LITE_BUTTON_KEY1,     /**< KEY1 按键，当前作为电机急停输入。 */
  PX4LITE_BUTTON_KEY2,     /**< KEY2 按键。 */
  PX4LITE_BUTTON_COUNT     /**< 按键数量。 */
} Px4Lite_ButtonId_t;

/**
 * @brief 读取板载按键是否处于按下状态。
 *
 * @param[in] button 按键编号。
 *
 * @return 1 表示按下，0 表示未按下或编号非法。
 */
uint8_t Px4Lite_ButtonPressed(Px4Lite_ButtonId_t button);

/**
 * @brief 初始化 LoRa E22 通信驱动。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_LoRaInit(void);

/**
 * @brief 执行 LoRa 驱动周期服务。
 *
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 服务结果。
 *
 * @note 本函数由 comm 任务调用，用于处理 RX、TX 完成和延迟 reinit。
 */
Px4Lite_Result_t Px4Lite_LoRaService(uint32_t now_ms);

/**
 * @brief 发送一帧 LoRa/MAVLink 数据。
 *
 * @param[in] data 待发送字节缓冲区，不能为 NULL。
 * @param[in] len 待发送长度，单位：byte。
 *
 * @return 发送提交结果。
 * @retval PX4LITE_OK 数据已被驱动异步复制并进入发送流程。
 * @retval PX4LITE_BUSY 上一帧仍在发送中。
 * @retval PX4LITE_INVALID_PARAM 参数为空或长度非法。
 * @retval PX4LITE_IO_ERROR 底层发送提交失败。
 *
 * @note `PX4LITE_OK` 不表示空口发送完成；真正完成以 UART DMA TC 回调统计为准。
 * 调用方在返回 OK 后可以立即复用自己的输入缓冲区。
 */
Px4Lite_Result_t Px4Lite_LoRaSend(const uint8_t *data, uint16_t len);

/**
 * @brief 获取 LoRa 模块当前公开状态。
 *
 * @param[in] now_ms 当前系统毫秒时间，用于超时判断。
 *
 * @return LoRa 模块状态。
 */
Px4Lite_State_t Px4Lite_LoRaGetState(uint32_t now_ms);

/**
 * @brief 查询 LoRa 模块是否在位。
 *
 * @return 1 表示模块在位，0 表示未接入。
 */
uint8_t Px4Lite_LoRaIsPresent(void);

/**
 * @brief 复制 LoRa/MAVLink 通信统计。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @note 本函数不访问慢速总线，不阻塞。
 */
void Px4Lite_LoRaGetDebugInfo(Px4Lite_CommDebugInfo_t *out);

/**
 * @brief 请求 GNSS 在所属 service 中重新初始化。
 *
 * @note recovery 回调只能置位请求标志，不能在 Health task 中执行总线 I/O。
 */
void Px4Lite_GnssRequestReinit(void);

/**
 * @brief 请求 IMU 在所属 service 中重新初始化。
 */
void Px4Lite_ImuRequestReinit(void);

/**
 * @brief 请求气压计在所属 service 中重新初始化。
 */
void Px4Lite_BaroRequestReinit(void);

/**
 * @brief 请求电源检测模块在所属 service 中重新初始化。
 */
void Px4Lite_BatteryRequestReinit(void);

/**
 * @brief 请求 LoRa 在所属 service 中重新初始化。
 */
void Px4Lite_LoRaRequestReinit(void);

#endif
