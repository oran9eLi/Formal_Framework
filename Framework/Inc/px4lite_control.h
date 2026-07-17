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
 * Control 任务级失效保护改变该目标。手动命令不经过姿态有效性门控，并会立即取消
 * 正在运行的自动起飞或降落斜坡，使用户在姿态不可用时仍能逐路接管电机。
 *
 * @param[in] motor_index 电机编号，范围 0 到 `PX4LITE_MOTOR_COUNT - 1`。
 * @param[in] throttle_percent 目标油门百分比，范围 0 到 100，超过 100 时按 100 限幅。
 *
 * @return 设置结果。
 */
Px4Lite_Result_t Px4Lite_ControlSetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent);

/**
 * @brief 设置并锁存一路电机精确 PWM 高电平脉宽。
 *
 * @details 精确脉宽命令属于手动接管，会取消正在运行的自动起飞或降落斜坡。
 *
 * @param[in] motor_index 电机编号，范围 0 到 `PX4LITE_MOTOR_COUNT - 1`。
 * @param[in] pulse_us PWM 高电平脉宽，单位 us。
 * @return 设置结果。
 */
Px4Lite_Result_t Px4Lite_ControlSetMotorPulseUs(uint8_t motor_index, uint16_t pulse_us);

/**
 * @brief 启动一键起飞油门斜坡。
 *
 * @details 仅当姿态快照新鲜、IMU 未明确离线/失败且环境模块满足起飞条件时允许启动；
 * 若自动起飞过程中确认姿态掉线，Control 会转入自动降落斜坡。斜坡到达 60% 后
 * 保持目标并退出自动起飞状态；启动成功时恢复姿态辅助混控。
 *
 * @return 命令结果。
 * @retval PX4LITE_OK 已启动自动起飞。
 * @retval PX4LITE_NOT_READY 姿态或起飞依赖模块不可用。
 */
Px4Lite_Result_t Px4Lite_ControlStartAutoTakeoff(void);

/**
 * @brief 启动一键降落油门斜坡。
 *
 * @details 用户主动一键降落同样要求 IMU 未明确离线/失败且姿态快照新鲜；启动成功时恢复姿态辅助混控。
 * 手动滑杆命令不使用该门控，姿态不可用时仍可逐路控制。
 *
 * @return 命令结果。
 * @retval PX4LITE_OK 已启动自动降落。
 * @retval PX4LITE_NOT_READY 姿态不可用。
 */
Px4Lite_Result_t Px4Lite_ControlStartAutoLanding(void);

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
 * @brief 设置或解除动力电池闭锁。
 *
 * @details 与急停分开：急停是用户动作、需要一次非零油门显式解锁；本标志表达"动力电池当前
 * 不满足运行条件"这一外部供电事实，由上层按电机电池档位持续维护，档位恢复即自动解除。
 * 置位沿立即清零四路目标并撤销自动油门；置位期间拒绝一切非零油门与一键起飞，一键降落
 * 不受限制。解除时不自动恢复任何油门，必须由用户重新拖动滑条。
 *
 * @param[in] inhibit 非 0 表示闭锁，0 表示允许输出。
 *
 * @return 总是 PX4LITE_OK。
 */
Px4Lite_Result_t Px4Lite_ControlSetPowerInhibit(uint8_t inhibit);

/**
 * @brief 查询动力电池闭锁是否生效。
 *
 * @return 1 表示闭锁中，0 表示允许输出。
 */
uint8_t Px4Lite_ControlIsPowerInhibited(void);

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
