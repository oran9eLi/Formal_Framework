/**
 ******************************************************************************
 * @file    Templates/Src/stm32f4xx_it.c
 * @author  MCD Application Team
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "bsp_gnss.h"
#include "bsp_config.h"
#include "debug_config.h"
#include "bsp_lora.h"

/** @addtogroup STM32F4xx_HAL_Examples
 * @{
 */

/** @addtogroup Templates
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1) {}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1) {}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1) {}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1) {}
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
// void SVC_Handler(void)
//{
// }

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
// void PendSV_Handler(void)
//{
// }

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
// void SysTick_Handler(void)
//{
//   HAL_IncTick();
// }

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */
void DMA1_Stream5_IRQHandler(void)
{
  BSP_GNSS_DmaIrqHandler();
}
void USART2_IRQHandler(void)
{
  UART_HandleTypeDef *huart = BSP_GNSS_GetUartHandle();
  uint32_t sr               = USART2->SR;

#if DEBUG_GNSS_BSP_MONITOR_ENABLE
  BSP_GNSS_DebugMarkUsart2Irq(USART2->SR, USART2->CR1, USART2->CR3);
#endif

  if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U) {
    BSP_GNSS_RequestRecoverRx();
    HAL_UART_IRQHandler(huart);
    return;
  }

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
#if DEBUG_GNSS_BSP_MONITOR_ENABLE
    BSP_GNSS_DebugMarkIdleIrq();
#endif

    __HAL_UART_CLEAR_IDLEFLAG(huart);
    BSP_GNSS_RxIdleCallback(0U);
  }

  HAL_UART_IRQHandler(huart);
}

void DMA1_Stream1_IRQHandler(void)
{
  BSP_LoRa_DmaIrqHandler();
}

void DMA1_Stream3_IRQHandler(void)
{
  BSP_LoRa_TxDmaIrqHandler();
}

void USART3_IRQHandler(void)
{
  UART_HandleTypeDef *huart = BSP_LoRa_GetUartHandle();
  uint32_t sr               = USART3->SR;

  if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U) {
    BSP_LoRa_RequestRecoverRx();
    HAL_UART_IRQHandler(huart);
    return;
  }

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    BSP_LoRa_RxIdleCallback(0U);
  }

  HAL_UART_IRQHandler(huart);
}

/**
 * @}
 */
