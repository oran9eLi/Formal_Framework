/**
 * @file px4lite_control.h
 * @brief 声明 Framework 控制模块的电机油门命令接口。
 *
 * @details
 * Control 模块只处理业务命令到电机输出的限幅、急停和失效保护策略，不直接访问
 * BSP/HAL。实际 PWM 输出必须经过 `px4lite_platform.h` 中的平台适配接口。
 */

#ifndef PX4LITE_CONTROL_H
#define PX4LITE_CONTROL_H

#include "px4lite_types.h"

/**
 * @brief 电机控制输出模式。
 */
typedef enum {
  PX4LITE_CONTROL_MODE_DIRECT = 0,         /**< 直控模式：滑动条或下行命令直接给四路目标油门。 */
  PX4LITE_CONTROL_MODE_ATTITUDE_ASSIST = 1 /**< 姿态辅助模式：保留各路基础油门，叠加 roll/pitch/yaw 修正。 */
} Px4Lite_ControlMode_t;

/**
 * @brief 设置并锁存单个电机目标油门百分比。
 *
 * @details
 * 最近一次有效设置会持续生效，直到新的本地滑条命令、下行命令、急停、恢复复位或
 * Control 任务级失效保护改变该目标。
 *
 * @param[in] motor_index 电机编号，范围 0 到 `PX4LITE_MOTOR_COUNT - 1`。
 * @param[in] throttle_percent 目标油门百分比，范围 0 到 100，超过 100 时按 100 限幅。
 *
 * @return 设置结果。
 */
Px4Lite_Result_t Px4Lite_ControlSetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent);

/**
 * @brief 设置 Control 输出模式。
 *
 * @param[in] mode 输出模式；姿态辅助模式只在姿态数据新鲜且基础油门有效时修正输出。
 * @return 设置结果。
 */
Px4Lite_Result_t Px4Lite_ControlSetMode(Px4Lite_ControlMode_t mode);

/**
 * @brief 复制当前 Control 输出模式。
 *
 * @param[out] mode 输出模式缓存，不能为 NULL。
 * @return 复制结果。
 */
Px4Lite_Result_t Px4Lite_ControlGetMode(Px4Lite_ControlMode_t *mode);

/**
 * @brief 设置姿态辅助模式的三轴目标姿态。
 *
 * @param[in] roll_deg100 目标横滚角，单位 degree * 100，绝对值不得超过配置上限。
 * @param[in] pitch_deg100 目标俯仰角，单位 degree * 100，绝对值不得超过配置上限。
 * @param[in] yaw_deg100 目标相对偏航角，单位 degree * 100，范围为 -18000~18000。
 * @return 设置结果。
 * @retval PX4LITE_INVALID_PARAM 横滚/俯仰目标或偏航目标超过各自安全范围。
 */
Px4Lite_Result_t Px4Lite_ControlSetAttitudeTarget(int32_t roll_deg100, int32_t pitch_deg100, int32_t yaw_deg100);

/**
 * @brief 锁存控制模块急停并立即撤销所有电机输出。
 *
 * @param[in] now_ms 当前系统时间，单位：ms。
 *
 * @return 急停执行结果。
 * @retval PX4LITE_OK 四路电机已经写入最小安全脉宽。
 * @retval PX4LITE_IO_ERROR 至少一路底层 PWM 写入失败。
 */
Px4Lite_Result_t Px4Lite_ControlEmergencyStop(uint32_t now_ms);

/**
 * @brief 初始化控制模块状态并复位电机输出。
 *
 * @return 初始化结果。
 */
Px4Lite_Result_t Px4Lite_ControlModuleInit(void);

/**
 * @brief 请求控制模块恢复到安全输出。
 *
 * @return 恢复请求结果。
 */
Px4Lite_Result_t Px4Lite_ControlRecover(void);

/**
 * @brief 执行一次控制周期，刷新电机 PWM 输出和 Motor topic。
 *
 * @param[in] now_ms 当前系统时间，单位：ms。
 */
void Px4Lite_ControlRun(uint32_t now_ms);

#endif
