/**
 * @file bsp_gnss.h
 * @brief 声明 GNSS UART DMA 接收接口。
 *
 * @details
 * BSP 只负责 USART2、DMA 和接收环形缓冲，不解析 NMEA。上层传感器驱动通过
 * BSP_GNSS_GetRxData() 取字节流。
 */

#ifndef BSP_GNSS_H
#define BSP_GNSS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "bsp_status.h"
#include "debug_config.h"

/**
 * @brief 配置 GNSS 模块使用的 USART2 和循环 DMA 接收。
 */
BSP_Status_t BSP_GNSS_Init(void);
/**
 * @brief 释放 GNSS UART 和 DMA 资源。
 */
BSP_Status_t BSP_GNSS_DeInit(void);
/**
 * @brief 返回 GNSS 私有 UART 句柄，仅供中断和 MSP 使用。
 */
UART_HandleTypeDef *BSP_GNSS_GetUartHandle(void);
/**
 * @brief 从 BSP 环形缓冲复制可用 GNSS 字节。
 */
uint16_t BSP_GNSS_GetRxData(uint8_t *dst, uint16_t max_len);
/**
 * @brief 返回 GNSS 接收环形缓冲中的未读字节数。
 */
uint16_t BSP_GNSS_GetRxCount(void);
/**
 * @brief 返回 GNSS 接收环形缓冲溢出累计丢字节数。
 */
uint32_t BSP_GNSS_GetDropCount(void);
/**
 * @brief 在 UART IDLE 中断报告新 DMA 字节后推进接收环形缓冲。
 */
void     BSP_GNSS_RxIdleCallback(uint16_t dummy);
/**
 * @brief 处理 GNSS DMA 接收中断并维护循环接收路径。
 */
void     BSP_GNSS_DmaIrqHandler(void);
/**
 * @brief 在错误后恢复 GNSS UART 和 DMA 接收路径。
 */
BSP_Status_t BSP_GNSS_RecoverRx(void);

#if DEBUG_GNSS_BSP_MONITOR_ENABLE
/**
 * @brief GNSS BSP 接收路径诊断信息。
 */
typedef struct {
  uint32_t usart2_irq_count;         /**< USART2 IRQ 进入次数。 */
  uint32_t idle_irq_count;           /**< IDLE 中断识别次数。 */
  uint32_t rx_idle_callback_count;   /**< 接收 IDLE 回调次数。 */
  uint32_t dma_irq_count;            /**< DMA IRQ 进入次数。 */

  uint32_t sr_snapshot;              /**< 最近一次 USART SR 快照。 */
  uint32_t cr1_snapshot;             /**< 最近一次 USART CR1 快照。 */
  uint32_t cr3_snapshot;             /**< 最近一次 USART CR3 快照。 */

  uint16_t dma_ndtr;                 /**< 当前 DMA NDTR。 */
  uint16_t dma_pos;                  /**< 当前 DMA 写入位置。 */
  uint16_t rx_head;                  /**< 软件环形缓冲写指针。 */
  uint16_t rx_tail;                  /**< 软件环形缓冲读指针。 */
  uint16_t rx_count;                 /**< 未读字节数。 */
  uint32_t rx_drop_count;            /**< 累计丢字节数。 */
} BSP_GNSS_DebugInfo_t;

/**
 * @brief 记录一次 USART2 中断和寄存器快照。
 */
void BSP_GNSS_DebugMarkUsart2Irq(uint32_t sr, uint32_t cr1, uint32_t cr3);
/**
 * @brief 记录一次 USART2 IDLE 中断。
 */
void BSP_GNSS_DebugMarkIdleIrq(void);
/**
 * @brief 复制当前 GNSS BSP 诊断计数和 DMA 状态。
 */
void BSP_GNSS_DebugGetInfo(BSP_GNSS_DebugInfo_t *info);
#endif

#endif


