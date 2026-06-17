/**
 * @file bsp_lora.h
 * @brief 声明 LoRa E22 的 USART3 DMA 收发和 GPIO 控制接口。
 *
 * @details
 * BSP 只负责 E22 串口、DMA、M0/M1/AUX 引脚和环形接收缓冲。MAVLink 解析和发送调度
 * 由上层 Sensor/Framework 完成。
 */

#ifndef BSP_LORA_H
#define BSP_LORA_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
 * @brief 初始化 LoRa E22 使用的 UART、DMA 和控制 GPIO。
 */
int32_t  BSP_LoRa_Init(void);
/**
 * @brief 返回 LoRa 私有 UART 句柄，仅供中断和 MSP 使用。
 */
UART_HandleTypeDef *BSP_LoRa_GetUartHandle(void);
/**
 * @brief 设置 E22 模块工作模式引脚。
 */
void     BSP_LoRa_SetMode(uint8_t m);
/**
 * @brief 读取 E22 AUX 就绪状态。
 */
uint8_t  BSP_LoRa_IsReady(void);
/**
 * @brief 判断 E22 是否正忙。
 */
uint8_t  BSP_LoRa_IsBusy(void);
/**
 * @brief 从 LoRa 接收环形缓冲复制可用字节。
 */
uint16_t BSP_LoRa_GetRxData(uint8_t *dst, uint16_t max_len);
/**
 * @brief 返回 LoRa 接收环形缓冲中的未读字节数。
 */
uint16_t BSP_LoRa_GetRxCount(void);
/**
 * @brief 返回 LoRa 接收环形缓冲累计溢出次数。
 */
uint32_t BSP_LoRa_GetRxOverflowCount(void);
/**
 * @brief 启动一次非阻塞 DMA 发送。
 *
 * @details
 * 该函数先把帧复制到内部 DMA 缓冲区，再启动 USART3 TX DMA。返回 0 表示发送已启动，
 * 1 表示上一帧仍在发送，-1 表示参数非法或 HAL 启动失败。
 */
int32_t  BSP_LoRa_StartSend(const uint8_t *data, uint16_t len);
/**
 * @brief 返回 LoRa TX DMA 是否仍在发送。
 */
uint8_t  BSP_LoRa_IsTxBusy(void);
/**
 * @brief 中止当前 LoRa TX DMA 发送。
 */
void     BSP_LoRa_AbortTx(void);
/**
 * @brief 处理 LoRa TX DMA 中断。
 */
void     BSP_LoRa_TxDmaIrqHandler(void);
/**
 * @brief 在 USART3 IDLE 后推进 LoRa 接收环形缓冲。
 */
void     BSP_LoRa_RxIdleCallback(uint16_t dummy);
/**
 * @brief 处理 LoRa RX DMA 中断。
 */
void     BSP_LoRa_DmaIrqHandler(void);
/**
 * @brief 恢复 LoRa UART 和 RX DMA 接收路径。
 */
void     BSP_LoRa_RecoverRx(void);

#endif
