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
#include "bsp_remoteid.h"

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
volatile Fault_CrashSnapshot_t g_fault_crash_snapshot;

/* Private function prototypes -----------------------------------------------*/
void Fault_CaptureAndHalt(uint32_t *stack, uint32_t exc_return, uint32_t fault_type);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 保存 Cortex-M fault 现场并停机等待调试。
 *
 * @param[in] stack 异常自动压栈区域，来自 MSP 或 PSP。
 * @param[in] exc_return 异常入口 LR/EXC_RETURN 值。
 * @param[in] fault_type Fault 类型，见 `Fault_CrashType_t`。
 *
 * @note 本函数运行在 fault 上下文，只允许保存寄存器快照，不打印、不访问文件系统、不做恢复。
 */
void Fault_CaptureAndHalt(uint32_t *stack, uint32_t exc_return, uint32_t fault_type)
{
  __disable_irq();

  g_fault_crash_snapshot.magic      = 0xFA417A11UL;
  g_fault_crash_snapshot.fault_type = fault_type;
  g_fault_crash_snapshot.exc_return = exc_return;
  g_fault_crash_snapshot.msp        = __get_MSP();
  g_fault_crash_snapshot.psp        = __get_PSP();
  g_fault_crash_snapshot.active_sp  = (uint32_t)stack;

  if (stack != 0) {
    g_fault_crash_snapshot.stacked_r0   = stack[0];
    g_fault_crash_snapshot.stacked_r1   = stack[1];
    g_fault_crash_snapshot.stacked_r2   = stack[2];
    g_fault_crash_snapshot.stacked_r3   = stack[3];
    g_fault_crash_snapshot.stacked_r12  = stack[4];
    g_fault_crash_snapshot.stacked_lr   = stack[5];
    g_fault_crash_snapshot.stacked_pc   = stack[6];
    g_fault_crash_snapshot.stacked_xpsr = stack[7];
  }

  g_fault_crash_snapshot.cfsr  = SCB->CFSR;
  g_fault_crash_snapshot.hfsr  = SCB->HFSR;
  g_fault_crash_snapshot.dfsr  = SCB->DFSR;
  g_fault_crash_snapshot.afsr  = SCB->AFSR;
  g_fault_crash_snapshot.bfar  = SCB->BFAR;
  g_fault_crash_snapshot.mmfar = SCB->MMFAR;
  g_fault_crash_snapshot.icsr  = SCB->ICSR;
  g_fault_crash_snapshot.shcsr = SCB->SHCSR;

  while (1) {
    __NOP();
  }
}

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
__asm void HardFault_Handler(void)
{
  IMPORT Fault_CaptureAndHalt
  TST LR, #4
  ITE EQ
  MRSEQ R0, MSP
  MRSNE R0, PSP
  MOV R1, LR
  MOVS R2, #1
  B Fault_CaptureAndHalt
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
__asm void MemManage_Handler(void)
{
  IMPORT Fault_CaptureAndHalt
  TST LR, #4
  ITE EQ
  MRSEQ R0, MSP
  MRSNE R0, PSP
  MOV R1, LR
  MOVS R2, #2
  B Fault_CaptureAndHalt
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
__asm void BusFault_Handler(void)
{
  IMPORT Fault_CaptureAndHalt
  TST LR, #4
  ITE EQ
  MRSEQ R0, MSP
  MRSNE R0, PSP
  MOV R1, LR
  MOVS R2, #3
  B Fault_CaptureAndHalt
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
__asm void UsageFault_Handler(void)
{
  IMPORT Fault_CaptureAndHalt
  TST LR, #4
  ITE EQ
  MRSEQ R0, MSP
  MRSNE R0, PSP
  MOV R1, LR
  MOVS R2, #4
  B Fault_CaptureAndHalt
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

void DMA1_Stream4_IRQHandler(void)
{
  BSP_RemoteId_TxDmaIrqHandler();
}

void USART3_IRQHandler(void)
{
  UART_HandleTypeDef *huart = BSP_LoRa_GetUartHandle();
  uint32_t sr               = USART3->SR;

  /* 错误分支绝不能在 ISR 内直接执行 DMA 恢复：BSP_LoRa_RecoverRx 内部的
     HAL_DMA_Abort 以 HAL_GetTick 做超时，而 tick 中断优先级低于本 ISR，
     tick 在此期间被冻结，"有界等待"退化为死循环，整机只能断电恢复。
     这里只清错误标志并置恢复请求，真正的 DMAStop/重启由 comm 任务在
     Lora_E22_Service 中执行（与 USART2/GNSS 的恢复模式一致）。 */
  if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U) {
    BSP_LoRa_UartErrorIrqHandler(sr);
    return;
  }

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    BSP_LoRa_RxIdleCallback(0U);
  }

  HAL_UART_IRQHandler(huart);
}

void UART4_IRQHandler(void)
{
  UART_HandleTypeDef *huart = BSP_RemoteId_GetUartHandle();

  if (huart != 0) { HAL_UART_IRQHandler(huart); }
}

/**
 * @}
 */
