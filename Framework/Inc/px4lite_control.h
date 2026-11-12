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

/** 本文件命令接口仅供任务上下文调用；内部使用调度互斥，不允许从 ISR 调用。 */

#include "px4lite_types.h"

/**
 * @brief 设置人工上锁/解锁；首次启动、恢复和急停后必须显式解锁。
 * @param[in] locked 1 上锁并立即清目标、撤收输出；0 解锁但只开始 2500 ms 最小脉宽预备。
 * @return OK 已执行；NOT_READY 未初始化、维护中或急停键仍按下；IO_ERROR 最小 PWM 写入失败。
 * @note 解锁不提交油门，不切换模式；上锁即使写入失败也保留闭锁，禁止普通命令自动解除。
 */
Px4Lite_Result_t Px4Lite_ControlSetMotorLocked(uint8_t locked);

/** @brief 复制人工锁状态，0 解锁、1 上锁；模块未初始化/关闭返回 NOT_READY。 */
Px4Lite_Result_t Px4Lite_ControlGetMotorLocked(uint8_t *locked);

/** @brief 四路百分比整体提交，保持当前模式；拒绝无副作用。仅任务上下文调用。 */
Px4Lite_Result_t Px4Lite_ControlSetMotorThrottles(const uint8_t percent[PX4LITE_MOTOR_COUNT]);
/** @brief 四路脉宽与 DIRECT 模式整体提交；拒绝无副作用。单位 us，仅任务上下文调用。 */
Px4Lite_Result_t Px4Lite_ControlSetMotorPulses(const uint16_t pulse[PX4LITE_MOTOR_COUNT]);

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
 * 仅显式解锁后接收非零油门。最近一次有效设置会持续生效，直到新的本地滑条命令、上锁、急停、恢复复位或
 * Control 任务级失效保护改变该目标。手动命令不经过姿态有效性门控，并可取消
 * 自动升速或保护缓降；用户主动缓降期间拒绝非零手动接管，0 命令不打断撤收过程。
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
 * 保持目标并退出自动起飞状态；保留学生明确选择的直控或姿态辅助模式。
 *
 * @return 命令结果。
 * @retval PX4LITE_OK 已启动自动起飞。
 * @retval PX4LITE_NOT_READY 姿态或起飞依赖模块不可用。
 */
Px4Lite_Result_t Px4Lite_ControlStartAutoTakeoff(void);

/**
 * @brief 启动一键降落油门斜坡。
 *
 * @details 降落是撤收动作，不要求姿态或电压；人工锁、急停和维护期间只保持最小输出。
 * 控制模式保持不变；解锁后的手动滑杆不使用姿态门控。
 *
 * @return 命令结果。
 * @retval PX4LITE_OK 已启动自动降落。
 * @retval PX4LITE_IO_ERROR 底层安全输出写入失败。
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
 * @brief 查询电压门禁闭锁是否生效；门禁关闭时为 0，不表示人工锁已解除。
 *
 * @return 1 表示闭锁中，0 表示允许输出。
 */
uint8_t Px4Lite_ControlIsPowerInhibited(void);

/** @brief 查询启用电压门禁时的低压事实；门禁旁路时为 0，不代替电源测量 topic。 */
uint8_t Px4Lite_ControlIsPowerLowVoltage(void);

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
 * @param[in] now_ms 调用者调度时刻，单位 ms；内部进入调度互斥后重新取当前时间。
 */
void Px4Lite_ControlRun(uint32_t now_ms);

/** @brief 电机目标及实际 PWM 输出停止后锁定维护；不要求动力电源接入，拒绝新非零输出。 */
Px4Lite_Result_t Px4Lite_ControlBeginConfigMaintenance(void);
/** @brief 复核维护锁、停止目标和成功的最小 PWM 输出，不检查动力电压。 */
Px4Lite_Result_t Px4Lite_ControlCheckConfigMaintenance(void);
/** @brief 解除维护，清除旧目标并重新开始预解锁。 */
void Px4Lite_ControlEndConfigMaintenance(void);
#endif
