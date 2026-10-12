/** @file test_gnss_dma.c
 * @brief 真实 GNSS DMA callback 在 NDTR 读取点注入 ISR，验证串行更新和 PRIMASK 恢复。
 */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "debug_config.h"
#undef DEBUG_GNSS_BSP_MONITOR_ENABLE
#define DEBUG_GNSS_BSP_MONITOR_ENABLE 0
#define BSP_CONFIG_H
#define BSP_GNSS_RX_BUF_SIZE 64U
typedef struct { uint32_t SR,DR; } UART_Instance;
typedef struct { uint32_t BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,OverSampling; } UART_Init;
typedef struct { uint32_t Channel,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,FIFOMode; } DMA_Init;
typedef struct { void *Instance; DMA_Init Init; } DMA_HandleTypeDef;
typedef struct { UART_Instance *Instance; UART_Init Init; uint32_t ErrorCode; DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;
static UART_Instance uart_instance;
#define BSP_GNSS_UART (&uart_instance)
#define BSP_GNSS_RX_DMA_STREAM ((void *)0)
#define BSP_GNSS_UART_BAUD 9600U
#define BSP_GNSS_UART_WORD 0U
#define BSP_GNSS_UART_STOP 0U
#define BSP_GNSS_UART_PARITY 0U
#define BSP_GNSS_UART_MODE 0U
#define BSP_GNSS_UART_HWCTL 0U
#define BSP_GNSS_UART_OVERSAMP 0U
#define BSP_GNSS_RX_DMA_CHANNEL 0U
#define DMA_PERIPH_TO_MEMORY 0U
#define DMA_PINC_DISABLE 0U
#define DMA_MINC_ENABLE 0U
#define DMA_PDATAALIGN_BYTE 0U
#define DMA_MDATAALIGN_BYTE 0U
#define DMA_CIRCULAR 0U
#define DMA_PRIORITY_MEDIUM 0U
#define DMA_FIFOMODE_DISABLE 0U
#define HAL_OK 0
#define HAL_UART_ERROR_NONE 0U
#define UART_IT_IDLE 0U
#define HAL_UART_Init(x) (0)
#define HAL_DMA_Init(x) (0)
#define HAL_DMA_DeInit(x) (0)
#define HAL_UART_DeInit(x) (0)
#define HAL_UART_DMAStop(x) (0)
#define HAL_UART_Receive_DMA(a,b,c) (0)
#define HAL_DMA_IRQHandler(x) ((void)0)
#define __HAL_LINKDMA(u,f,d) ((u)->f = &(d))
#define __HAL_UART_ENABLE_IT(a,b) ((void)0)
#define __HAL_UART_DISABLE_IT(a,b) ((void)0)
static uint32_t sim_primask, pending, inject, ndtr;
static uint32_t Counter(void);
static void Enable(void);
#define __get_PRIMASK() (sim_primask)
#define __disable_irq() (sim_primask = 1U)
#define __enable_irq() Enable()
#define __HAL_DMA_GET_COUNTER(x) Counter()
#include "../../Bsp/Src/bsp_gnss.c"
static void Enable(void)
{
  sim_primask=0U;
  if(pending) { pending=0U; BSP_GNSS_RxIdleCallback(0U); }
}
static uint32_t Counter(void)
{
  uint32_t old=ndtr;
  if(inject) {
    inject=0U; ndtr=BSP_GNSS_RX_BUF_SIZE-10U;
    if(sim_primask) { pending=1U; } else { BSP_GNSS_RxIdleCallback(0U); }
  }
  return old;
}
int main(void)
{
  s_rx_head=1U; s_rx_tail=1U; ndtr=BSP_GNSS_RX_BUF_SIZE-5U; inject=1U;
  BSP_GNSS_RxIdleCallback(0U);
  assert(s_rx_head==10U && s_rx_tail==1U && s_rx_drop_count==0U);
  assert(sim_primask==0U);
  sim_primask=1U; ndtr=BSP_GNSS_RX_BUF_SIZE-11U;
  BSP_GNSS_RxIdleCallback(0U);
  assert(sim_primask==1U && s_rx_head==11U);
  puts("gnss DMA concurrency tests passed"); return 0;
}
