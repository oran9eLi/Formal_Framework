/**
 ******************************************************************************
 * @file    Templates/Inc/stm32f4xx_it.h
 * @author  MCD Application Team
 * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/**
 * @brief Cortex-M fault 类型编号。
 */
typedef enum {
  FAULT_CRASH_TYPE_HARDFAULT = 1, /**< HardFault 异常。 */
  FAULT_CRASH_TYPE_MEMMANAGE = 2, /**< MemManage 异常。 */
  FAULT_CRASH_TYPE_BUSFAULT  = 3, /**< BusFault 异常。 */
  FAULT_CRASH_TYPE_USAGE     = 4  /**< UsageFault 异常。 */
} Fault_CrashType_t;

/**
 * @brief Cortex-M fault 现场快照。
 *
 * @details
 * Fault handler 在进入死循环前保存异常入口寄存器、当前 MSP/PSP 和 SCB fault
 * 状态寄存器，便于生产环境复现后通过调试器或转储工具定位 stacked PC/LR。
 */
typedef struct {
  uint32_t magic;       /**< 快照有效标记，固定为 `0xFA417A11`。 */
  uint32_t fault_type;  /**< Fault 类型，见 `Fault_CrashType_t`。 */
  uint32_t exc_return;  /**< 异常入口 LR/EXC_RETURN 值。 */
  uint32_t msp;         /**< 捕获时 MSP 值。 */
  uint32_t psp;         /**< 捕获时 PSP 值。 */
  uint32_t active_sp;   /**< 本次异常使用的堆栈指针。 */
  uint32_t stacked_r0;  /**< 异常自动压栈 R0。 */
  uint32_t stacked_r1;  /**< 异常自动压栈 R1。 */
  uint32_t stacked_r2;  /**< 异常自动压栈 R2。 */
  uint32_t stacked_r3;  /**< 异常自动压栈 R3。 */
  uint32_t stacked_r12; /**< 异常自动压栈 R12。 */
  uint32_t stacked_lr;  /**< 异常自动压栈 LR。 */
  uint32_t stacked_pc;  /**< 异常自动压栈 PC，优先用于定位崩溃指令。 */
  uint32_t stacked_xpsr; /**< 异常自动压栈 xPSR。 */
  uint32_t cfsr;        /**< Configurable Fault Status Register。 */
  uint32_t hfsr;        /**< HardFault Status Register。 */
  uint32_t dfsr;        /**< Debug Fault Status Register。 */
  uint32_t afsr;        /**< Auxiliary Fault Status Register。 */
  uint32_t bfar;        /**< BusFault Address Register。 */
  uint32_t mmfar;       /**< MemManage Fault Address Register。 */
  uint32_t icsr;        /**< Interrupt Control and State Register。 */
  uint32_t shcsr;       /**< System Handler Control and State Register。 */
} Fault_CrashSnapshot_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern volatile Fault_CrashSnapshot_t g_fault_crash_snapshot;

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
