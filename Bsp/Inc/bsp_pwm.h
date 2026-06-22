/**
 * @file bsp_pwm.h
 * @brief 声明四路电机 PWM 输出的 BSP 接口。
 *
 * @details
 * BSP PWM 只负责 TIM3/TIM4 的初始化、通道启动和脉宽写入。油门百分比、急停、
 * ESC 解锁和失效保护属于 Framework Control 模块。
 */

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>
#include "bsp_status.h"
#include "stm32f4xx_hal.h"

/**
 * @brief 电机 PWM 通道编号。
 */
typedef enum {
  BSP_PWM_MOTOR1 = 0, /**< 电机 1 PWM。 */
  BSP_PWM_MOTOR2,     /**< 电机 2 PWM。 */
  BSP_PWM_MOTOR3,     /**< 电机 3 PWM。 */
  BSP_PWM_MOTOR4,     /**< 电机 4 PWM。 */
  BSP_PWM_MOTOR_COUNT /**< PWM 通道数量，必须保持为最后一项。 */
} BSP_PwmChannel_t;

/**
 * @brief 初始化 TIM3/TIM4 PWM 并启动全部通道。
 *
 * @return 初始化结果。
 */
BSP_Status_t BSP_PWM_Init(void);

/**
 * @brief 停止全部 PWM 通道并释放 TIM 外设。
 *
 * @return 释放结果。
 */
BSP_Status_t BSP_PWM_DeInit(void);

/**
 * @brief 设置单个 PWM 通道高电平脉宽。
 *
 * @param[in] channel PWM 通道编号。
 * @param[in] pulse_us 高电平脉宽，单位：us，超过周期时按周期限幅。
 *
 * @return 写入结果。
 */
BSP_Status_t BSP_PWM_SetPulseUs(BSP_PwmChannel_t channel, uint16_t pulse_us);

/**
 * @brief 获取 PWM 通道所属 TIM 句柄。
 *
 * @param[in] channel PWM 通道编号。
 *
 * @return TIM 句柄指针；编号非法时返回 NULL。
 *
 * @note 该接口供 MSP 初始化和反初始化按 TIM 实例选择 GPIO/clock 使用。
 */
TIM_HandleTypeDef *BSP_PWM_GetTimHandle(BSP_PwmChannel_t channel);

#endif
