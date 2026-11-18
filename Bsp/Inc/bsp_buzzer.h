/**
 * @file bsp_buzzer.h
 * @brief BSP层PE5/TIM9蜂鸣器原始PWM接口。
 * @details 仅管理GPIO、时钟和PWM；不得承载旋律、页面或操作反馈策略。
 *          PE5只连接已确认的驱动输入，不默认直接驱动电磁线圈。
 */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include <stdint.h>
#include "bsp_status.h"

/** @brief 初始化TIM9_CH1，成功后保持低电平静音；仅资源所有者调用。 */
BSP_Status_t BSP_Buzzer_Init(void);
/** @brief 配置TIM9基础计数器和零占空比通道，不定义任何音符。 */
BSP_Status_t MX_TIM9_Init(void);
/** @brief 静音并释放TIM9；允许尚未初始化时调用。 */
BSP_Status_t BSP_Buzzer_DeInit(void);
/**
 * @brief 设置50%占空比频率；0表示静音，其他值限BSP配置范围，单位Hz。
 * @note 无阻塞等待；只供sensor任务经平台接口调用，不能从ISR调用。
 */
BSP_Status_t BSP_Buzzer_SetFrequency(uint16_t frequency_hz);
/** @brief fatal专用，直接固定PE5为低电平；不依赖句柄、tick或中断。 */
void BSP_Buzzer_EmergencyStop(void);
#endif
