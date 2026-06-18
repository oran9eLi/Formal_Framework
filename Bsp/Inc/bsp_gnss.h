/**
 * @file bsp_gnss.h
 * @brief Declare the GNSS UART DMA receive interface.
 */

#ifndef BSP_GNSS_H
#define BSP_GNSS_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "bsp_status.h"
#include "debug_config.h"

/**
 * @brief Configure USART2 and circular DMA reception for the GNSS module.
 */
BSP_Status_t BSP_GNSS_Init(void);
/**
 * @brief Release the GNSS UART and DMA so the peripheral consumes no resources.
 */
BSP_Status_t BSP_GNSS_DeInit(void);
/**
 * @brief Return the private GNSS UART handle for interrupt and MSP use only.
 */
UART_HandleTypeDef *BSP_GNSS_GetUartHandle(void);
/**
 * @brief Copy available GNSS bytes from the BSP ring buffer to the caller.
 */
uint16_t BSP_GNSS_GetRxData(uint8_t *dst, uint16_t max_len);
/**
 * @brief Return the number of unread bytes in the GNSS receive ring.
 */
uint16_t BSP_GNSS_GetRxCount(void);
/**
 * @brief Return the cumulative number of GNSS bytes dropped on ring overflow.
 */
uint32_t BSP_GNSS_GetDropCount(void);
/**
 * @brief Advance the receive ring when UART IDLE reports newly arrived DMA bytes.
 */
void     BSP_GNSS_RxIdleCallback(uint16_t dummy);
/**
 * @brief Handle GNSS DMA receive interrupts and maintain the circular receive path.
 */
void     BSP_GNSS_DmaIrqHandler(void);
/**
 * @brief Recover the GNSS UART and DMA receive path after an error.
 */
BSP_Status_t BSP_GNSS_RecoverRx(void);

#if DEBUG_GNSS_BSP_MONITOR_ENABLE
typedef struct {
  uint32_t usart2_irq_count;
  uint32_t idle_irq_count;
  uint32_t rx_idle_callback_count;
  uint32_t dma_irq_count;

  uint32_t sr_snapshot;
  uint32_t cr1_snapshot;
  uint32_t cr3_snapshot;

  uint16_t dma_ndtr;
  uint16_t dma_pos;
  uint16_t rx_head;
  uint16_t rx_tail;
  uint16_t rx_count;
  uint32_t rx_drop_count;
} BSP_GNSS_DebugInfo_t;

/**
 * @brief Record one USART2 interrupt and its register snapshot for diagnostics.
 */
void BSP_GNSS_DebugMarkUsart2Irq(uint32_t sr, uint32_t cr1, uint32_t cr3);
/**
 * @brief Record one detected USART2 IDLE interrupt for diagnostics.
 */
void BSP_GNSS_DebugMarkIdleIrq(void);
/**
 * @brief Copy the current GNSS BSP diagnostic counters and DMA state.
 */
void BSP_GNSS_DebugGetInfo(BSP_GNSS_DebugInfo_t *info);
#endif

#endif


