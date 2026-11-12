/**
 * @file bsp_rgb.h
 * @brief BSP层PC9/TIM8_CH4/DMA2 Stream7原始PWM波形接口。
 * @details 只接收比较值序列，不解释GRB、旋律或告警含义；sensor任务独占正常接口。
 *          初始化、停止和恢复均由任务执行；中断只记录DMA完成/错误事实。
 */
#ifndef BSP_RGB_H
#define BSP_RGB_H

#include <stdint.h>
#include "bsp_status.h"

/** @brief 配置TIM8及DMA原始波形资源；不发送颜色，不等待器件响应。 */
BSP_Status_t MX_TIM8_Init(void);
/** @brief 初始化可选RGB通道；失败不影响其他外设，允许上层限速重试。 */
BSP_Status_t BSP_RGB_Init(void);
/** @brief 在任务中撤收输出和释放资源；DMA尚未停止时返回BUSY，禁止复用缓冲。 */
BSP_Status_t BSP_RGB_DeInit(void);
/**
 * @brief 异步复制并提交1..BSP_RGB_MAX_DUTY_COUNT个原始比较值。
 * @param[in] duty 比较值数组，每项为0..period-1；返回后调用者可立即复用原数组。
 * @param[in] count 比较值数量，单位半字；忙时不覆盖已接受帧。
 * @return OK仅表示已接受；完成、错误和超时必须继续调用Service确认。
 * @note BSP额外插入全低保护槽；协议要求的复位低电平槽由调用者提供。
 */
BSP_Status_t BSP_RGB_Send(const uint16_t *duty, uint16_t count);
/** @brief sensor每10ms持续调用；返回BUSY/OK/ERROR/TIMEOUT，故障锁存至重新初始化。 */
BSP_Status_t BSP_RGB_Service(uint32_t now_ms);
/** @brief 返回软件发送占用标志；DMA完成后仍需Service撤收才解除。 */
uint8_t BSP_RGB_IsBusy(void);
/** @brief 取得周期计数和建议高电平比较值；period为ARR+1，输出指针均不能为空。 */
BSP_Status_t BSP_RGB_GetTiming(uint16_t *period, uint16_t *zero, uint16_t *one);
/** @brief DMA2 Stream7中断入口，仅确认中断并记录完成/错误，不等待或重新初始化外设。 */
void BSP_RGB_DmaIrqHandler(void);
/** @brief 致命异常立即断开PC9复用并撤收TIM8/DMA，不等待HAL，不发送协议帧。 */
void BSP_RGB_EmergencyStop(void);

#endif
