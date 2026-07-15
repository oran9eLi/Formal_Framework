/**
 * @file bsp_remoteid.h
 * @brief 声明 ESP32-S3 RemoteID UART4 DMA 发送 BSP 接口。
 *
 * @details
 * BSP 只负责 PC10/PC11、UART4、TX DMA 和原始字节发送，不解析 MAVLink/OpenDroneID，
 * 不保存业务身份含义。上层必须通过 Platform Adapter 间接调用本文件能力。
 */

#ifndef BSP_REMOTEID_H
#define BSP_REMOTEID_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
 * @brief 初始化 RemoteID 使用的 UART4 和 TX DMA。
 *
 * @return 0 表示成功，-1 表示 HAL 初始化失败。
 */
int32_t BSP_RemoteId_Init(void);

/**
 * @brief 返回 RemoteID UART 句柄，仅供 MSP 和中断入口使用。
 */
UART_HandleTypeDef *BSP_RemoteId_GetUartHandle(void);

/**
 * @brief 启动一次非阻塞 DMA 发送。
 *
 * @param[in] data 待发送字节缓冲区，不能为 NULL。
 * @param[in] len 待发送长度，单位 byte，范围 1 到 BSP_REMOTEID_TX_BUF_SIZE。
 *
 * @return 0 表示发送已启动，1 表示上一帧仍在发送，-1 表示参数或 HAL 错误。
 *
 * @note 本函数先复制到内部静态 DMA 缓冲区，再启动 UART4 TX DMA；调用方返回后可复用原缓冲区。
 */
int32_t BSP_RemoteId_StartSend(const uint8_t *data, uint16_t len);

/**
 * @brief 查询 RemoteID TX DMA 是否忙。
 */
uint8_t BSP_RemoteId_IsTxBusy(void);

/**
 * @brief 查询 RemoteID UART4/DMA 通道是否已经完成本地初始化。
 *
 * @return 1 表示本地 UART4/DMA 发送通道可提交数据，0 表示尚未初始化或初始化失败。
 *
 * @note 该状态只证明 STM32 本地发送通道可用，不证明 ESP32-S3 已经收到或完成广播。
 */
uint8_t BSP_RemoteId_IsReady(void);

/**
 * @brief 查询 ESP32(RemoteID)硬件是否在位。
 *
 * @return 1 表示 PC8 检测到 ESP32 3.3V(已插并上电)，0 表示未插/未上电或模块被关闭。
 *
 * @note 该状态基于 PC8 下拉输入电平，直接反映 ESP32 是否供电，独立于本地 TX 通道。
 */
uint8_t BSP_RemoteId_IsPresent(void);

/**
 * @brief 中止当前 RemoteID TX DMA 发送。
 */
void BSP_RemoteId_AbortTx(void);

/**
 * @brief 处理 RemoteID TX DMA 中断。
 */
void BSP_RemoteId_TxDmaIrqHandler(void);

/**
 * @brief 处理 HAL UART TX 完成回调，供统一 UART 回调分发调用。
 */
void BSP_RemoteId_TxCompleteCallback(UART_HandleTypeDef *huart);

#endif
