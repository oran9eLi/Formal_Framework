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
 * @brief 尽早配置 E22 模式/AUX 引脚并驱动到正常模式，须在 BSP_Init 最先调用。
 */
void BSP_LoRa_PreInit(void);
/**
 * @brief 初始化 LoRa E22 使用的 UART、DMA 和控制 GPIO。
 */
int32_t BSP_LoRa_Init(void);
/**
 * @brief 返回 LoRa 私有 UART 句柄，仅供中断和 MSP 使用。
 */
UART_HandleTypeDef *BSP_LoRa_GetUartHandle(void);
/**
 * @brief 设置 E22 模块工作模式引脚。
 */
void BSP_LoRa_SetMode(uint8_t m);
/**
 * @brief 读取 E22 AUX 就绪状态。
 */
uint8_t BSP_LoRa_IsReady(void);
/**
 * @brief 判断 E22 是否正忙。
 */
uint8_t BSP_LoRa_IsBusy(void);
/**
 * @brief 用输入下拉释放法主动探测 E22 AUX 是否被在位模块拉高。
 *
 * @return 1 表示模块在位(取消下拉后 AUX 被模块拉高)，0 表示拔出(线路维持低)。
 *
 * @note 探测期间 AUX 始终为输入，只允许在 comm 任务上下文、模块非发送忙时调用。
 */
uint8_t BSP_LoRa_ProbeAuxPresent(void);
/**
 * @brief 返回 LoRa 本地 UART 波特率，单位：bit/s。
 */
uint32_t BSP_LoRa_GetUartBaud(void);
/**
 * @brief 返回 LoRa E22 当前空中速率，单位：bit/s。
 */
uint32_t BSP_LoRa_GetAirBps(void);
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
int32_t BSP_LoRa_StartSend(const uint8_t *data, uint16_t len);
/**
 * @brief 返回 LoRa TX DMA 是否仍在发送。
 */
uint8_t BSP_LoRa_IsTxBusy(void);
/**
 * @brief 中止当前 LoRa TX DMA 发送。
 */
void BSP_LoRa_AbortTx(void);
/**
 * @brief 处理 LoRa TX DMA 中断。
 */
void BSP_LoRa_TxDmaIrqHandler(void);
/**
 * @brief 在 USART3 IDLE 后推进 LoRa 接收环形缓冲。
 */
void BSP_LoRa_RxIdleCallback(uint16_t dummy);
/**
 * @brief 处理 LoRa RX DMA 中断。
 */
void BSP_LoRa_DmaIrqHandler(void);
/**
 * @brief 恢复 LoRa UART 和 RX DMA 接收路径。
 */
void BSP_LoRa_RecoverRx(void);
/**
 * @brief Request LoRa RX recovery from USART3 ISR; only sets a flag in ISR.
 */
void BSP_LoRa_RequestRecoverRx(void);
/**
 * @brief 在 USART3 错误中断中清错误并请求 LoRa RX 恢复。
 *
 * @param[in] sr_snapshot USART3 SR 快照。
 */
void BSP_LoRa_UartErrorIrqHandler(uint32_t sr_snapshot);
/**
 * @brief Consume one pending LoRa RX recovery request from the owner service.
 *
 * @return 1 if a request was pending, 0 otherwise.
 */
uint8_t BSP_LoRa_ConsumeRecoverRxRequest(void);

#endif
