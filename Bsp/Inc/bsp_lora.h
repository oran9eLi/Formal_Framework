/**
 * @file bsp_lora.h
 * @brief Declare the LoRa E22 USART3 DMA receive and GPIO control interface.
 */

#ifndef BSP_LORA_H
#define BSP_LORA_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

int32_t  BSP_LoRa_Init(void);
/**
 * @brief Return the private LoRa UART handle for interrupt and MSP use only.
 */
UART_HandleTypeDef *BSP_LoRa_GetUartHandle(void);
void     BSP_LoRa_SetMode(uint8_t m);
uint8_t  BSP_LoRa_IsReady(void);
uint8_t  BSP_LoRa_IsBusy(void);
uint16_t BSP_LoRa_GetRxData(uint8_t *dst, uint16_t max_len);
uint16_t BSP_LoRa_GetRxCount(void);
uint32_t BSP_LoRa_GetRxOverflowCount(void);
/* Non-blocking transmit. Copies the frame into an internal DMA buffer and
   starts a USART3 TX DMA. Returns 0 on start, 1 if a transfer is still in
   flight, -1 on invalid argument or HAL error. */
int32_t  BSP_LoRa_StartSend(const uint8_t *data, uint16_t len);
uint8_t  BSP_LoRa_IsTxBusy(void);
void     BSP_LoRa_AbortTx(void);
void     BSP_LoRa_TxDmaIrqHandler(void);
void     BSP_LoRa_RxIdleCallback(uint16_t dummy);
void     BSP_LoRa_DmaIrqHandler(void);
void     BSP_LoRa_RecoverRx(void);

#endif
