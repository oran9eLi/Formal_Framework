/**
 * @file bsp_uart.h
 * @brief 声明板级调试 UART 原始接口。
 *
 * @details
 * 本文件属于 BSP 层，只提供调试串口初始化、原始发送和只读配置快照。日志格式化、
 * 任务互斥和业务诊断输出必须留在 DebugConsole/DebugTask 中完成。
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

#include "bsp_config.h"
#include "bsp_status.h"

/**
 * @brief 调试 UART 当前 HAL 配置诊断快照。
 *
 * @details
 * 本结构体用于确认 `bsp_config.h` 中的调试串口配置是否真实进入 HAL 句柄，不表示
 * 串口链路质量，也不替代 DebugConsole 的互斥发送接口。
 */
typedef struct {
  uint32_t instance;      /**< UART 外设寄存器基地址，调试打印用。 */
  uint32_t baud_rate;     /**< 波特率，单位 bit/s。 */
  uint32_t word_length;   /**< HAL UART 字长配置值。 */
  uint32_t stop_bits;     /**< HAL UART 停止位配置值。 */
  uint32_t parity;        /**< HAL UART 校验位配置值。 */
  uint32_t mode;          /**< HAL UART 收发模式配置值。 */
  uint32_t hw_flow_ctl;   /**< HAL UART 硬件流控配置值。 */
  uint32_t over_sampling; /**< HAL UART 过采样配置值。 */
  uint8_t initialized;    /**< HAL 句柄是否已经离开 RESET 状态，1 表示已初始化。 */
} BSP_UART_DebugInfo_t;

/**
 * @brief 按 `bsp_config.h` 配置初始化调试 UART。
 *
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_UART_Init(void);

/**
 * @brief 通过调试 UART 在超时时间内发送完整字节缓冲区。
 *
 * @param[in] data 待发送数据缓冲区，不能为 NULL。
 * @param[in] length 待发送字节数。
 * @param[in] timeout_ms HAL 阻塞发送超时时间，单位 ms。
 *
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_UART_Send(const uint8_t *data, uint16_t length, uint32_t timeout_ms);

/**
 * @brief 复制调试 UART 当前 HAL 配置。
 *
 * @param[out] out 输出缓冲区，允许为 NULL；为 NULL 时函数不执行任何操作。
 */
void BSP_UART_GetDebugInfo(BSP_UART_DebugInfo_t *out);

#if (BSP_ENABLE_RPI_UART == 1U)
/**
 * @brief USART6 原始配置及接收事实；不解析协议，不包含业务或实验状态。
 * @details HAL 配置编码仅供平台适配层转换；接收字节在 ISR 计数，快照短临界区复制。
 */
typedef struct {
  uint32_t baud_rate;               /**< HAL 句柄的波特率，bit/s。 */
  uint32_t word_length;             /**< HAL 字长编码，包含可能使用的校验位。 */
  uint32_t stop_bits;               /**< HAL 停止位编码。 */
  uint32_t parity;                  /**< HAL 校验编码。 */
  uint32_t hw_flow_ctl;             /**< HAL 硬件流控编码。 */
  uint32_t rx_byte_count;           /**< RX 回调交付字节数，含环满舍弃字节。 */
  uint32_t rx_uart_error_count;     /**< 含接收硬件错误标志的 IRQ 次数。 */
  uint32_t rx_drop_count;           /**< 环满未能入环的字节数。 */
  uint32_t rx_rearm_failure_count;  /**< 回调中重新挂接接收失败次数。 */
  uint32_t last_rx_ms;              /**< 最近 RX 回调的板级毫秒时间。 */
  uint8_t config_valid;             /**< 1=已初始化且不在重配。 */
  uint8_t rx_seen;                  /**< 1=曾收到字节，不依赖计数是否回绕。 */
} BSP_RpiUART_Diagnostics_t;

/** @brief 只复制 USART6 配置和接收计数；不读数据环、不触发 I/O，out 不得为 NULL。 */
void BSP_RpiUART_CopyDiagnostics(BSP_RpiUART_Diagnostics_t *out);

/**
 * @brief 按 `bsp_config.h` 配置初始化树莓派 MAVLink UART（USART6，PC6/PC7）。
 *
 * @details
 * 与调试 UART（USART1）相互独立，用于树莓派双向 MAVLink 字节流。
 *
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_RpiUART_Init(void);

/**
 * @brief 通过树莓派 UART 在超时时间内阻塞发送完整字节缓冲区。
 *
 * @param[in] data 待发送数据缓冲区，不能为 NULL。
 * @param[in] length 待发送字节数。
 * @param[in] timeout_ms HAL 阻塞发送超时时间，单位 ms。
 *
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_RpiUART_Send(const uint8_t *data, uint16_t length, uint32_t timeout_ms);

/**
 * @brief 读取 USART6 当前实际配置的波特率。
 * @param[out] baud_bps 输出波特率，单位 bit/s，不能为 NULL。
 * @return BSP 通用返回码；串口未就绪或正在重配置时返回 BUSY。
 */
BSP_Status_t BSP_RpiUART_GetBaudRate(uint32_t *baud_bps);

/**
 * @brief 同步重配置 USART6 波特率并重新挂接单字节中断接收。
 * @param[in] baud_bps 目标波特率，单位 bit/s；白名单由 Framework 业务层负责。
 * @return BSP 通用返回码。
 * @note 仅允许 CommTask 在旧速率响应完整发送后调用；失败时会尝试恢复旧速率。
 */
BSP_Status_t BSP_RpiUART_SetBaudRate(uint32_t baud_bps);

/**
 * @brief 从树莓派 UART RX 环形缓冲读取原始字节。
 * @param[out] data 输出缓冲区，不能为 NULL。
 * @param[in] max_length 最多读取字节数。
 * @return 实际读取字节数。
 * @note 本接口只暴露原始字节；MAVLink 解析必须在 Framework/Platform Adapter 层完成。
 */
uint16_t BSP_RpiUART_Read(uint8_t *data, uint16_t max_length);

/**
 * @brief USART6 中断入口转发函数。
 * @note 仅供 `USART6_IRQHandler` 调用，ISR 内只清 HAL 状态并重新挂接单字节接收。
 */
void BSP_RpiUART_IrqHandler(void);
#endif

#endif
