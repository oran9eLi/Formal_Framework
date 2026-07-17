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
 * @brief 按 `bsp_config.h` 配置初始化树莓派 MAVLink UART（USART6，PC6/PC7）。
 *
 * @details
 * 与调试 UART（USART1）相互独立，只用于飞控向树莓派单向阻塞发送 MAVLink 帧。
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
