/**
 * @file test_gnss_rx_health.c
 * @brief 真实GNSS BSP的完整RX健康事实、初始化短路及重开IRQ失败恢复回归。
 * @details 仅在HAL夹具注入原始寄存器/句柄损坏；不复制生产健康判断，不证明实板接线正确。
 */
#include "gnss_transport_fixture.h"

/** @brief 每项只改变一处真实会话事实，检出只看EN/DMAR的旧盲点。 */
static void BreakRx(unsigned kind)
{
  switch(kind) {
    case 0U: uart_regs.CR1&=~USART_CR1_UE;break;
    case 1U: uart_regs.CR1&=~USART_CR1_RE;break;
    case 2U: uart_regs.CR1&=~USART_CR1_IDLEIE;break;
    case 3U: uart_regs.CR3&=~USART_CR3_EIE;break;
    case 4U: s_gnss_uart.RxState=HAL_UART_STATE_READY;break;
    case 5U: s_gnss_uart.ErrorCode=HAL_UART_ERROR_DMA;break;
    case 6U: hdma_gnss_rx.ErrorCode=1U;break;
    case 7U: hdma_gnss_rx.State=HAL_DMA_STATE_READY;break;
    case 8U: dma_regs.CR&=~DMA_SxCR_EN;break;
    case 9U: uart_regs.CR3&=~USART_CR3_DMAR;break;
    case 10U: irq_enabled[BSP_GNSS_IRQn]=0U;break;
    case 11U: irq_enabled[BSP_GNSS_RX_DMA_IRQn]=0U;break;
    case 12U: s_initialized=0U;break;
    case 13U: s_rx_recover_request=1U;break;
    default: assert(0);break;
  }
}

/** @brief 不改变硬件的观察缓存可检测失效，随后真实重建恢复所有运行事实。 */
static void TestOwnerChecksAndNextRecovery(void)
{
  unsigned kind;
  for(kind=0U;kind<14U;kind++) {
    unsigned initializations;
    Cold();initializations=uart_init_calls;
    BreakRx(kind);BSP_GNSS_RxIdleCallback(0U);
    assert(!Info().initialized && s_rx_recover_request);
    assert(!irq_enabled[BSP_GNSS_IRQn] && !irq_enabled[BSP_GNSS_RX_DMA_IRQn]);
    assert(!(uart_regs.CR3&USART_CR3_DMAR));
    assert(BSP_GNSS_ConsumeRecoverRxRequest());
    assert(BSP_GNSS_RecoverRx()==BSP_STATUS_OK && BspGnss_RxSessionReady());
    assert(uart_init_calls==initializations+1U && Info().recoveries==1U);
    assert(Info().baud_bps==115200U && Info().ndtr==1024U);
  }
}

/** @brief Init不能对寄存器仍EN/DMAR但HAL或NVIC已坏的会话直接返回旧成功。 */
static void TestInitAndSameBaudShortcut(void)
{
  unsigned kind;
  for(kind=0U;kind<14U;kind++) {
    unsigned initializations;
    Cold();initializations=uart_init_calls;BreakRx(kind);
    assert(BSP_GNSS_Init()==BSP_STATUS_OK && BspGnss_RxSessionReady());
    assert(uart_init_calls==initializations+1U);
    assert(Info().recoveries==0U);
    initializations=uart_init_calls;
    assert(BSP_GNSS_Init()==BSP_STATUS_OK && uart_init_calls==initializations);
    BreakRx(kind);
    assert(BSP_GNSS_SetBaudRate(115200U)==BSP_STATUS_OK && BspGnss_RxSessionReady());
    assert(uart_init_calls==initializations+1U);
  }
}

/** @brief TX独立中止期间NVIC临时关闭不是RX故障，且PRIMASK按进入状态保持。 */
static void TestTxTemporaryIrqMaskStillAllowed(void)
{
  static const uint8_t command[3]={1U,2U,3U};unsigned previous;
  Cold();assert(BSP_GNSS_Transmit(command,sizeof(command))==BSP_STATUS_OK);
  previous=uart_init_calls;primask=1U;irq_enabled[BSP_GNSS_IRQn]=0U;
  assert(BSP_GNSS_AbortTx()==BSP_STATUS_OK && primask==1U);
  assert(!irq_enabled[BSP_GNSS_IRQn] && irq_enabled[BSP_GNSS_RX_DMA_IRQn]);
  assert(Info().initialized && !s_rx_recover_request && uart_init_calls==previous);
  primask=0U;
}

/** @brief 健康轮询不重建/清队列/读SR，累计接收事实保持。 */
static void TestHealthyReadHasNoRecoverySideEffects(void)
{
  unsigned clears,initializations;
  Cold();dma_regs.NDTR=1000U;BSP_GNSS_RxIdleCallback(0U);
  assert(Info().rx_bytes==24U && Info().queued_bytes==24U);
  clears=clear_uart_calls;initializations=uart_init_calls;
  BSP_GNSS_RxIdleCallback(0U);
  assert(Info().rx_bytes==24U && Info().queued_bytes==24U && !s_rx_recover_request);
  assert(clear_uart_calls==clears && uart_init_calls==initializations);
}

/** @brief 不在本测试改动其他模块资源或增大DMA缓冲。 */
int main(void)
{
  TestOwnerChecksAndNextRecovery();TestInitAndSameBaudShortcut();
  TestTxTemporaryIrqMaskStillAllowed();TestHealthyReadHasNoRecoverySideEffects();
  puts("GNSS RX health: 14 faults / Init+same-baud checks / next recovery / TX-only IRQ mask / healthy no-op PASS");
  return 0;
}
