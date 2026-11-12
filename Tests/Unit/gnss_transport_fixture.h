/**
 * @file gnss_transport_fixture.h
 * @brief 真实GNSS BSP的原始DMA环、异步复制TX和可失败HAL恢复回归。
 * @details HAL替身显式建模BUSY/ABORT/Lock、TC及初始化失败，不代替实板UART验证。
 */
#ifndef GNSS_TRANSPORT_FIXTURE_H
#define GNSS_TRANSPORT_FIXTURE_H

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define BSP_CONFIG_H
#include "debug_config.h"
#undef DEBUG_GNSS_BSP_MONITOR_ENABLE
#define DEBUG_GNSS_BSP_MONITOR_ENABLE 0
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t SR,DR,BRR,CR1,CR2,CR3; } UartRegisters;
typedef struct { uint32_t CR,NDTR,PAR,M0AR,FCR; } DmaRegisters;
typedef struct { uint32_t BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,OverSampling; } UartInit;
typedef struct { uint32_t Channel,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,FIFOMode; } DmaInit;
typedef struct DmaHandle {
  DmaRegisters *Instance; DmaInit Init; uint32_t State,ErrorCode,Lock; void *Parent;
  void (*XferAbortCallback)(struct DmaHandle *);
} DMA_HandleTypeDef;
typedef struct {
  UartRegisters *Instance; UartInit Init; DMA_HandleTypeDef *hdmarx;
  uint32_t gState,RxState,ErrorCode; const uint8_t *pTxBuffPtr; uint16_t TxXferCount;
} UART_HandleTypeDef;
static UartRegisters uart_regs;
static DmaRegisters dma_regs;
#define BSP_GNSS_UART (&uart_regs)
#define BSP_GNSS_RX_DMA_STREAM (&dma_regs)
#define BSP_GNSS_UART_BAUD 115200U
#define BSP_GNSS_RX_BUF_SIZE 1024U
#define BSP_GNSS_IRQn 38
#define BSP_GNSS_RX_DMA_IRQn 16
#define BSP_GNSS_RX_DMA_CHANNEL 0x08000000U
#define BSP_GNSS_UART_WORD 0U
#define BSP_GNSS_UART_STOP 0U
#define BSP_GNSS_UART_PARITY 0U
#define BSP_GNSS_UART_MODE 0xCU
#define BSP_GNSS_UART_HWCTL 0U
#define BSP_GNSS_UART_OVERSAMP 0U
#define DMA_PERIPH_TO_MEMORY 0U
#define DMA_PINC_DISABLE 0U
#define DMA_MINC_ENABLE (1U<<10)
#define DMA_PDATAALIGN_BYTE 0U
#define DMA_MDATAALIGN_BYTE 0U
#define DMA_CIRCULAR (1U<<8)
#define DMA_PRIORITY_MEDIUM (1U<<16)
#define DMA_FIFOMODE_DISABLE 0U
#define DMA_SxCR_EN 1U
#define HAL_DMA_STATE_RESET 0U
#define HAL_DMA_STATE_READY 1U
#define HAL_DMA_STATE_BUSY 2U
#define HAL_DMA_STATE_ABORT 4U
#define HAL_DMA_STATE_TIMEOUT 3U
#define HAL_DMA_ERROR_NONE 0U
#define HAL_UART_STATE_RESET 0U
#define HAL_UART_STATE_READY 0x20U
#define HAL_UART_STATE_BUSY_TX 0x21U
#define HAL_UART_STATE_BUSY_RX 0x22U
#define HAL_UART_ERROR_NONE 0U
#define HAL_UART_ERROR_DMA 0x10U
#define USART_SR_PE 1U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_ORE 8U
#define USART_SR_IDLE 0x10U
#define USART_SR_TXE 0x80U
#define USART_SR_TC 0x40U
#define USART_CR1_UE 0x2000U
#define USART_CR1_TE 8U
#define USART_CR1_RE 4U
#define USART_CR1_IDLEIE 0x10U
#define USART_CR1_RXNEIE 0x20U
#define USART_CR1_TCIE 0x40U
#define USART_CR1_TXEIE 0x80U
#define USART_CR1_PEIE 0x100U
#define USART_CR3_DMAR 0x40U
#define USART_CR3_DMAT 0x80U
#define USART_CR3_EIE 1U
#define UART_IT_IDLE USART_CR1_IDLEIE
#define UART_IT_RXNE USART_CR1_RXNEIE
#define UART_IT_TC USART_CR1_TCIE
#define UART_IT_TXE USART_CR1_TXEIE
#define UART_IT_PE USART_CR1_PEIE
#define UART_IT_ERR 0x80000001U
#define CLEAR_BIT(reg,mask) ((reg)&=~(mask))
#define __HAL_LINKDMA(u,f,d) do { (u)->f=&(d); (d).Parent=(u); } while(0)
#define __HAL_DMA_GET_COUNTER(d) FakeCounter(d)
static uint32_t primask;
static uint8_t counter_inject, pending_rx_callback;
static uint32_t FakeCounter(DMA_HandleTypeDef *d);
static void FakeEnableInterrupts(void);
static uint8_t irq_enabled[64];
static unsigned uart_init_calls,dma_init_calls,abort_calls,clear_uart_calls;
static unsigned dma_deinit_calls,uart_deinit_calls,tx_submit_calls;
static unsigned rx_start_calls,dr_write_calls;
static unsigned tx_abort_calls;
static uint8_t fail_abort,fail_dma_deinit,fail_uart_deinit,fail_uart_init,fail_dma_init,fail_rx_start;
static uint8_t fail_tx_abort;
static uint8_t dma_irq_error,inject_error_on_enable,inject_dma_error_on_enable;
static HAL_StatusTypeDef tx_submit_result = HAL_OK;
static uint32_t fake_rx_bytes;
static uint16_t fake_write_pos;
void BSP_GNSS_UartIrqHandler(void);
void BSP_GNSS_DmaIrqHandler(void);
void BSP_GNSS_RxIdleCallback(uint16_t dummy);
#define __get_PRIMASK() (primask)
#define __disable_irq() (primask=1U)
#define __enable_irq() FakeEnableInterrupts()
static void HAL_NVIC_DisableIRQ(int irq) { irq_enabled[irq]=0U; }
static void HAL_NVIC_ClearPendingIRQ(int irq) { (void)irq; }
static uint32_t NVIC_GetEnableIRQ(int irq) { return irq_enabled[irq]; }
static void HAL_NVIC_EnableIRQ(int irq)
{
  irq_enabled[irq]=1U;
  if (irq==BSP_GNSS_IRQn && inject_error_on_enable) {
    inject_error_on_enable=0U; uart_regs.SR=USART_SR_FE|USART_SR_IDLE;
    BSP_GNSS_UartIrqHandler();
  }
  if (irq==BSP_GNSS_RX_DMA_IRQn && inject_dma_error_on_enable) {
    inject_dma_error_on_enable=0U;dma_irq_error=1U;
    BSP_GNSS_DmaIrqHandler();dma_irq_error=0U;
  }
}
static void UartInterrupt(UART_HandleTypeDef *u,uint32_t interrupt,uint8_t enabled)
{
  uint32_t *reg=(interrupt==UART_IT_ERR)?&u->Instance->CR3:&u->Instance->CR1;
  uint32_t mask=(interrupt==UART_IT_ERR)?USART_CR3_EIE:interrupt;
  if(enabled) { *reg|=mask; } else { *reg&=~mask; }
}
#define __HAL_UART_ENABLE_IT(u,it) UartInterrupt(u,it,1U)
#define __HAL_UART_DISABLE_IT(u,it) UartInterrupt(u,it,0U)
static void ClearUart(UART_HandleTypeDef *u) { ++clear_uart_calls; u->Instance->SR=0U; }
#define __HAL_UART_CLEAR_OREFLAG(u) ClearUart(u)
#define __HAL_UART_CLEAR_IDLEFLAG(u) ClearUart(u)
static HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *u)
{
  ++uart_init_calls; assert(!irq_enabled[16]&&!irq_enabled[38]);
  if(fail_uart_init) { return HAL_ERROR; }
  u->gState=HAL_UART_STATE_READY;u->RxState=HAL_UART_STATE_READY;u->ErrorCode=0U;
  u->Instance->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE;u->Instance->CR3=0U;
  return HAL_OK;
}
static HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *d)
{
  ++dma_init_calls;if(fail_dma_init) { return HAL_ERROR; }
  d->State=HAL_DMA_STATE_READY;d->Lock=0U;d->ErrorCode=0U;
  d->Instance->CR=d->Init.Channel|d->Init.MemInc|d->Init.Mode|d->Init.Priority;
  return HAL_OK;
}
static uint32_t HAL_DMA_GetState(DMA_HandleTypeDef *d) { return d->State; }
static uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *d) { return d->ErrorCode; }
static HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *d)
{
  ++abort_calls;
  if(fail_abort) { d->State=HAL_DMA_STATE_TIMEOUT;return HAL_TIMEOUT; }
  d->Instance->CR&=~DMA_SxCR_EN;d->State=HAL_DMA_STATE_READY;d->Lock=0U;
  return HAL_OK;
}
static HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *d)
{
  ++dma_deinit_calls;if(fail_dma_deinit) { return HAL_ERROR; }
  assert(d->State!=HAL_DMA_STATE_BUSY);
  d->Instance->CR=0U;d->Instance->NDTR=0U;d->State=HAL_DMA_STATE_RESET;
  d->Lock=0U;d->ErrorCode=0U;d->XferAbortCallback=NULL;
  return HAL_OK;
}
static HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *u)
{
  ++uart_deinit_calls;if(fail_uart_deinit) { return HAL_ERROR; }
  u->Instance->CR1=0U;u->Instance->CR3=0U;
  u->gState=HAL_UART_STATE_RESET;u->RxState=HAL_UART_STATE_RESET;
  return HAL_OK;
}
static HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *u,uint8_t *buffer,uint16_t length)
{
  ++rx_start_calls;(void)buffer;
  if(fail_rx_start) { return HAL_ERROR; }
  if(u->hdmarx->State!=HAL_DMA_STATE_READY||u->hdmarx->Lock) { return HAL_BUSY; }
  u->hdmarx->State=HAL_DMA_STATE_BUSY;u->hdmarx->Lock=1U;
  u->hdmarx->Instance->CR|=DMA_SxCR_EN;u->hdmarx->Instance->NDTR=length;
  u->RxState=HAL_UART_STATE_BUSY_RX;u->Instance->CR3|=USART_CR3_DMAR|USART_CR3_EIE;
  fake_write_pos=0U;
  return HAL_OK;
}
static HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *buffer,uint16_t length)
{
  ++tx_submit_calls;assert(!irq_enabled[16]&&!irq_enabled[38]);
  if(tx_submit_result!=HAL_OK) { return tx_submit_result; }
  if(u->gState!=HAL_UART_STATE_READY) { return HAL_BUSY; }
  assert(u->RxState==HAL_UART_STATE_BUSY_RX);
  u->pTxBuffPtr=buffer;u->TxXferCount=length;u->gState=HAL_UART_STATE_BUSY_TX;
  u->Instance->CR1|=USART_CR1_TXEIE;
  return HAL_OK;
}
/** @brief 建模本项目非DMA TX的真实HAL中止语义：仅TX中断、计数和gState。 */
static HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{
  tx_abort_calls++;assert(!irq_enabled[BSP_GNSS_IRQn]);
  if(fail_tx_abort) { return HAL_TIMEOUT; }
  u->Instance->CR1&=~(USART_CR1_TXEIE|USART_CR1_TCIE);
  u->TxXferCount=0U;u->gState=HAL_UART_STATE_READY;return HAL_OK;
}
static void FakeAbortCallback(DMA_HandleTypeDef *d) { (void)d; }
static void HAL_UART_IRQHandler(UART_HandleTypeDef *u)
{
  if((u->Instance->SR&(USART_SR_FE|USART_SR_ORE|USART_SR_NE|USART_SR_PE))&&(u->Instance->CR3&USART_CR3_EIE)) {
    u->RxState=HAL_UART_STATE_READY;u->Instance->CR3&=~(USART_CR3_DMAR|USART_CR3_EIE);
    u->hdmarx->State=HAL_DMA_STATE_ABORT;u->hdmarx->Lock=1U;u->hdmarx->Instance->CR&=~DMA_SxCR_EN;
    u->hdmarx->XferAbortCallback=FakeAbortCallback;return;
  }
  if((u->Instance->SR&USART_SR_TXE)&&(u->Instance->CR1&USART_CR1_TXEIE)) {
    u->Instance->DR=*u->pTxBuffPtr++;++dr_write_calls;
    if(--u->TxXferCount==0U) { u->Instance->CR1&=~USART_CR1_TXEIE;u->Instance->CR1|=USART_CR1_TCIE; }
    u->Instance->SR&=~USART_SR_TC;return;
  }
  if((u->Instance->SR&USART_SR_TC)&&(u->Instance->CR1&USART_CR1_TCIE)) {
    u->Instance->CR1&=~USART_CR1_TCIE;u->gState=HAL_UART_STATE_READY;
    /* 项目强HAL回调只分发其它UART，GNSS完成由真实BSP wrapper确认。 */
  }
}
static void HAL_DMA_IRQHandler(DMA_HandleTypeDef *d)
{
  if(dma_irq_error) { d->ErrorCode=1U;d->State=HAL_DMA_STATE_READY;d->Instance->CR&=~DMA_SxCR_EN; }
}
/** @brief NDTR读取点注入中断，验证真实BSP互斥提交不被旧位置覆盖。 */
static uint32_t FakeCounter(DMA_HandleTypeDef *d)
{
  uint32_t old=d->Instance->NDTR;
  if(counter_inject) {
    counter_inject=0U;d->Instance->NDTR=BSP_GNSS_RX_BUF_SIZE-10U;
    if(primask) { pending_rx_callback=1U; } else { BSP_GNSS_RxIdleCallback(0U); }
  }
  return old;
}
/** @brief 恢复前面允许的中断并交付已挂起回调。 */
static void FakeEnableInterrupts(void)
{
  primask=0U;
  if(pending_rx_callback) { pending_rx_callback=0U;BSP_GNSS_RxIdleCallback(0U); }
}
#include "../../Bsp/Src/bsp_gnss.c"

/** @brief 读取纯状态副本。 */
static BSP_GNSS_TransportInfo_t Info(void)
{ BSP_GNSS_TransportInfo_t i;assert(BSP_GNSS_CopyTransportInfo(&i)==BSP_STATUS_OK);return i; }
/** @brief 仅夹具模拟硬件冷上电，生产Init/warm操作均不得清累计统计。 */
static void Cold(void)
{
  counter_inject=pending_rx_callback=0U;
  memset(&s_gnss_uart,0,sizeof(s_gnss_uart));memset(&hdma_gnss_rx,0,sizeof(hdma_gnss_rx));
  memset((void*)&s_transport,0,sizeof(s_transport));memset(&uart_regs,0,sizeof(uart_regs));memset(&dma_regs,0,sizeof(dma_regs));
  memset(irq_enabled,0,sizeof(irq_enabled));s_rx_head=s_rx_tail=0U;s_rx_drop_count=0U;
  s_initialized=s_recovering=s_tx_busy=s_rx_recover_request=s_dma_error_latched=0U;
  s_target_baud_bps=BSP_GNSS_UART_BAUD;fake_write_pos=0U;fake_rx_bytes=0U;primask=0U;
  fail_abort=fail_dma_deinit=fail_uart_deinit=fail_uart_init=fail_dma_init=fail_rx_start=0U;
  fail_tx_abort=0U;
  dma_irq_error=inject_error_on_enable=inject_dma_error_on_enable=0U;tx_submit_result=HAL_OK;
  assert(BSP_GNSS_Init()==BSP_STATUS_OK);
  assert(Info().initialized&&Info().ndtr==1024U&&Info().baud_bps==115200U);
  assert(irq_enabled[16]&&irq_enabled[38]);
}

#endif
