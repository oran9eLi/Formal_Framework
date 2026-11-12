/**
 * @file bsp_gnss.h
 * @brief GNSS原始UART收发、状态副本及owner恢复接口。
 *
 * @details
 * BSP只负责USART2、RX DMA环、静态复制TX和硬件事实，不解析NMEA/UBX、不执行协议配置策略。
 * Init/SetBaudRate/Transmit/AbortTx/RecoverRx只能由sensor owner或调度前初始化路径调用。
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

/** @brief 本次上电累计原始传输事实；不代表协议有效或定位成功。 */
typedef struct {
  uint32_t baud_bps;       /**< 最近HAL配置波特率，bit/s；initialized=0时不表示链路正在工作。 */
  uint32_t rx_bytes;       /**< 已观察到的NDTR推进字节数，不还原漏过的完整DMA圈数。 */
  uint32_t rx_drop_bytes;  /**< 已观察环覆盖及重建时丢弃的未消费字节总数。 */
  uint32_t uart_errors;    /**< UART错误IRQ观察事件数，一次IRQ的多个错误位合计一次。 */
  uint32_t dma_errors;     /**< RX DMA错误会话数，同一故障锁存期间不重复计数。 */
  uint32_t tx_packets;     /**< 真正完成UART TC的发送帧数，不能代替对端ACK。 */
  uint32_t tx_failures;    /**< HAL提交错误或已接受发送被错误/独立TX中止/重建中止的累计次数。 */
  uint32_t recoveries;     /**< RecoverRx成功重建次数，不计冷初始化和正常波特率切换。 */
  uint16_t ndtr;          /**< 当前RX DMA剩余字节数；硬件可在复制窗口继续推进。 */
  uint16_t queued_bytes;  /**< 已提交软件环中的未读字节数。 */
  uint8_t initialized;   /**< 1表示完整接收会话已启动；错误后为0直至owner重建成功。 */
  uint8_t tx_busy;        /**< 1表示静态TX副本仍由HAL使用，只在实际完成或错误后解除。 */
} BSP_GNSS_TransportInfo_t;

#define BSP_GNSS_TX_MAX_BYTES 96U /**< 静态TX复制槽容量，单位byte。 */

/** @brief owner在TX空闲时重建串口/DMA为指定波特率；支持4800..921600 bit/s，失败可由RecoverRx重试。 */
BSP_Status_t BSP_GNSS_SetBaudRate(uint32_t baud_bps);

/**
 * @brief 非阻塞复制并提交1..96 byte原始TX，busy时不覆盖旧副本。
 * @return OK仅表示已接受；BUSY表示旧发送未完成；ERROR表示参数、硬件状态或HAL提交失败。
 * @note 接收DMA的BUSY_RX不影响TX；实际完成以TransportInfo.tx_packets和tx_busy为准。
 */
BSP_Status_t BSP_GNSS_Transmit(const uint8_t *data, uint16_t len);

/**
 * @brief owner只中止异常IT发送，不停RX DMA、不清接收队列或协议状态。
 * @return OK表示TX已解除且RX硬件仍健康；ERROR表示需走原接收恢复路径。
 * @note 未完成的已接受TX计一次tx_failures；不伪造成功发送，不增加RX recoveries。
 * 只短暂关闭本UART IRQ并保持原使能状态；RX异常或中止失败才请求完整硬件恢复。
 */
BSP_Status_t BSP_GNSS_AbortTx(void);

/** @brief 短临界区复制原始transport事实；不读取SR/DR、不推进DMA位置、不消费队列。 */
BSP_Status_t BSP_GNSS_CopyTransportInfo(BSP_GNSS_TransportInfo_t *out);

/** @brief USART2 IRQ统一入口，只服务HAL、记录事实及请求owner恢复。 */
void BSP_GNSS_UartIrqHandler(void);
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
 * @brief 返回本次上电已观察到的环覆盖及硬件重建丢弃字节总数，单位byte。
 */
uint32_t BSP_GNSS_GetDropCount(void);
/**
 * @brief 任务轮询或 UART IDLE 中断报告 DMA 字节后推进环形缓冲。
 * @note 内部保存 PRIMASK，从读取 DMA 位置到更新索引全程互斥，并按原中断状态恢复。
 */
void BSP_GNSS_RxIdleCallback(uint16_t dummy);
/**
 * @brief 处理 GNSS DMA 接收中断并维护循环接收路径。
 */
void BSP_GNSS_DmaIrqHandler(void);
/**
 * @brief 在错误后恢复 GNSS UART 和 DMA 接收路径。
 */
BSP_Status_t BSP_GNSS_RecoverRx(void);
/**
 * @brief Request GNSS RX recovery from USART2 ISR; only sets a flag in ISR.
 */
void BSP_GNSS_RequestRecoverRx(void);
/**
 * @brief Consume one pending GNSS RX recovery request from the owner service.
 *
 * @return 1 if a request was pending, 0 otherwise.
 */
uint8_t BSP_GNSS_ConsumeRecoverRxRequest(void);

#if DEBUG_GNSS_BSP_MONITOR_ENABLE
/**
 * @brief GNSS BSP 接收路径诊断信息。
 */
typedef struct {
  uint32_t usart2_irq_count;       /**< USART2 IRQ 进入次数。 */
  uint32_t idle_irq_count;         /**< IDLE 中断识别次数。 */
  uint32_t rx_idle_callback_count; /**< 接收 IDLE 回调次数。 */
  uint32_t dma_irq_count;          /**< DMA IRQ 进入次数。 */

  uint32_t sr_snapshot;  /**< 最近一次 USART SR 快照。 */
  uint32_t cr1_snapshot; /**< 最近一次 USART CR1 快照。 */
  uint32_t cr3_snapshot; /**< 最近一次 USART CR3 快照。 */

  uint16_t dma_ndtr;      /**< 当前 DMA NDTR。 */
  uint16_t dma_pos;       /**< 当前 DMA 写入位置。 */
  uint16_t rx_head;       /**< 软件环形缓冲写指针。 */
  uint16_t rx_tail;       /**< 软件环形缓冲读指针。 */
  uint16_t rx_count;      /**< 未读字节数。 */
  uint32_t rx_drop_count; /**< 累计丢字节数。 */
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
