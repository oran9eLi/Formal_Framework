/** @file test_p25_gnss_transport.c
 * @brief 真实BSP原始传输、异步TX和可失败HAL恢复回归。
 */
#include "gnss_transport_fixture.h"
/** @brief 受控DMA写入真实BSP私有环，驱动仍通过NDTR观察推进。 */
static void Advance(unsigned bytes)
{
  unsigned j;for(j=0U;j<bytes;j++) { s_rx_buf[fake_write_pos]=(uint8_t)fake_rx_bytes++;fake_write_pos=(fake_write_pos+1U)%BSP_GNSS_RX_BUF_SIZE; }
  dma_regs.NDTR=fake_write_pos?BSP_GNSS_RX_BUF_SIZE-fake_write_pos:BSP_GNSS_RX_BUF_SIZE;
}
/** @brief HAL_IT接受不等于完成；逐TXE后仍busy，最终TC才释放。 */
static void CompleteTx(void)
{
  while(s_gnss_uart.TxXferCount) { uart_regs.SR=USART_SR_TXE;BSP_GNSS_UartIrqHandler(); }
  assert(Info().tx_busy);
  uart_regs.SR=USART_SR_TC;BSP_GNSS_UartIrqHandler();assert(!Info().tx_busy);
}
static void TestReceiveAndCounters(void)
{
  uint8_t data[64];BSP_GNSS_TransportInfo_t i;uint16_t head,tail;unsigned clears;
  Cold();Advance(320U);BSP_GNSS_RxIdleCallback(0U);
  assert(Info().rx_bytes==320U&&Info().queued_bytes==320U);
  head=s_rx_head;tail=s_rx_tail;clears=clear_uart_calls;uart_regs.SR=0xABCDEU;
  i=Info();assert(s_rx_head==head&&s_rx_tail==tail&&clear_uart_calls==clears&&uart_regs.SR==0xABCDEU);
  assert(BSP_GNSS_CopyTransportInfo(NULL)==BSP_STATUS_ERROR);assert(!primask);
  primask=1U;(void)Info();assert(primask==1U);primask=0U;
  assert(BSP_GNSS_GetRxData(data,64U)==64U);assert(data[0]==0U&&data[63]==63U);
  Advance(130U);BSP_GNSS_DmaIrqHandler();BSP_GNSS_RxIdleCallback(0U);
  assert(Info().rx_bytes==450U&&Info().queued_bytes==386U);
  Advance(900U);BSP_GNSS_RxIdleCallback(0U);i=Info();
  assert(i.rx_bytes==1350U&&i.rx_drop_bytes==263U&&i.queued_bytes==1023U);
  assert(BSP_GNSS_GetRxData(data,1U)==1U&&data[0]==(uint8_t)(1350U-1023U));
  Cold();Advance(512U);BSP_GNSS_DmaIrqHandler();Advance(512U);BSP_GNSS_DmaIrqHandler();
  BSP_GNSS_RxIdleCallback(0U);assert(Info().rx_bytes==1024U&&Info().queued_bytes==1023U&&Info().rx_drop_bytes==1U);
}
static void TestCopyTxAndCompletion(void)
{
  uint8_t data[8]={1,2,3,4,5,6,7,8};const uint8_t original[8]={1,2,3,4,5,6,7,8};unsigned submitted;
  Cold();assert(BSP_GNSS_Transmit(NULL,1U)==BSP_STATUS_ERROR);assert(BSP_GNSS_Transmit(data,0U)==BSP_STATUS_ERROR);
  assert(BSP_GNSS_Transmit(data,97U)==BSP_STATUS_ERROR);
  assert(BSP_GNSS_Transmit(data,8U)==BSP_STATUS_OK&&Info().tx_busy&&Info().tx_packets==0U);
  assert(s_gnss_uart.pTxBuffPtr!=data);memset(data,0xAA,sizeof(data));assert(!memcmp(s_gnss_uart.pTxBuffPtr,original,8U));
  submitted=tx_submit_calls;assert(BSP_GNSS_Transmit(data,8U)==BSP_STATUS_BUSY&&tx_submit_calls==submitted);
  assert(BSP_GNSS_SetBaudRate(57600U)==BSP_STATUS_BUSY&&Info().baud_bps==115200U);
  CompleteTx();assert(Info().tx_packets==1U&&Info().tx_failures==0U);
  tx_submit_result=HAL_BUSY;assert(BSP_GNSS_Transmit(data,8U)==BSP_STATUS_BUSY&&!Info().tx_busy&&Info().tx_failures==0U);
  tx_submit_result=HAL_ERROR;assert(BSP_GNSS_Transmit(data,8U)==BSP_STATUS_ERROR&&!Info().tx_busy&&Info().tx_failures==1U);
  tx_submit_result=HAL_OK;uart_regs.CR1&=~USART_CR1_TE;
  assert(BSP_GNSS_Transmit(data,8U)==BSP_STATUS_ERROR);
  uart_regs.CR1|=USART_CR1_TE;irq_enabled[BSP_GNSS_IRQn]=0U;
  assert(BSP_GNSS_Transmit(data,8U)==BSP_STATUS_ERROR&&!Info().tx_busy);
  Cold();{
    uint8_t maximum[BSP_GNSS_TX_MAX_BYTES];memset(maximum,0x5A,sizeof(maximum));
    assert(BSP_GNSS_Transmit(maximum,sizeof(maximum))==BSP_STATUS_OK);
    assert(!memcmp(s_gnss_uart.pTxBuffPtr,maximum,sizeof(maximum)));CompleteTx();
    assert(Info().tx_packets==1U);
  }
}
static void TestRebuildFailures(void)
{
  BSP_GNSS_TransportInfo_t i;unsigned inits;
  Cold();Advance(37U);BSP_GNSS_RxIdleCallback(0U);
  fail_abort=1U;assert(BSP_GNSS_SetBaudRate(57600U)==BSP_STATUS_ERROR);
  i=Info();assert(!i.initialized&&i.rx_bytes==37U&&i.rx_drop_bytes==37U&&!i.queued_bytes);
  fail_abort=0U;fail_dma_deinit=1U;
  assert(BSP_GNSS_RecoverRx()==BSP_STATUS_ERROR);assert(BSP_GNSS_RecoverRx()==BSP_STATUS_ERROR);
  assert(Info().rx_bytes==37U&&Info().rx_drop_bytes==37U);
  fail_dma_deinit=0U;fail_uart_deinit=1U;assert(BSP_GNSS_RecoverRx()==BSP_STATUS_ERROR);
  assert(dma_regs.NDTR==0U&&s_rx_head==0U);
  fail_uart_deinit=0U;assert(BSP_GNSS_RecoverRx()==BSP_STATUS_OK);
  assert(Info().recoveries==1U&&Info().rx_bytes==37U&&Info().rx_drop_bytes==37U&&Info().baud_bps==57600U);
  inits=uart_init_calls;assert(BSP_GNSS_SetBaudRate(57600U)==BSP_STATUS_OK&&uart_init_calls==inits);
  assert(BSP_GNSS_SetBaudRate(0U)==BSP_STATUS_ERROR&&BSP_GNSS_SetBaudRate(1000000U)==BSP_STATUS_ERROR);
  fail_rx_start=1U;assert(BSP_GNSS_SetBaudRate(9600U)==BSP_STATUS_ERROR&&!Info().initialized);
  fail_rx_start=0U;assert(BSP_GNSS_RecoverRx()==BSP_STATUS_OK&&Info().baud_bps==9600U);
  fail_uart_init=1U;assert(BSP_GNSS_SetBaudRate(115200U)==BSP_STATUS_ERROR&&!Info().initialized);
  fail_uart_init=0U;assert(BSP_GNSS_RecoverRx()==BSP_STATUS_OK);
  fail_dma_init=1U;assert(BSP_GNSS_SetBaudRate(38400U)==BSP_STATUS_ERROR&&!Info().initialized);
  fail_dma_init=0U;assert(BSP_GNSS_RecoverRx()==BSP_STATUS_OK);
}
static void TestIrqErrorsAndAbort(void)
{
  uint8_t bytes[3]={10,20,30};BSP_GNSS_TransportInfo_t i;
  Cold();assert(BSP_GNSS_Transmit(bytes,3U)==BSP_STATUS_OK);Advance(9U);
  uart_regs.SR=USART_SR_FE|USART_SR_IDLE;BSP_GNSS_UartIrqHandler();i=Info();
  assert(i.uart_errors==1U&&i.tx_failures==1U&&!i.tx_busy&&!i.initialized&&i.tx_packets==0U);
  assert(!irq_enabled[16]&&!irq_enabled[38]&&!(uart_regs.CR1&(USART_CR1_IDLEIE|USART_CR1_TXEIE|USART_CR1_TCIE)));
  assert(hdma_gnss_rx.State==HAL_DMA_STATE_ABORT&&hdma_gnss_rx.Lock==1U);
  assert(BSP_GNSS_ConsumeRecoverRxRequest()==1U);
  assert(BSP_GNSS_RecoverRx()==BSP_STATUS_OK&&hdma_gnss_rx.Lock==1U&&hdma_gnss_rx.XferAbortCallback==NULL);
  assert(Info().tx_failures==1U&&Info().recoveries==1U&&Info().rx_bytes==9U);
  assert(BSP_GNSS_Transmit(bytes,3U)==BSP_STATUS_OK);CompleteTx();assert(Info().tx_packets==1U);
  dma_irq_error=1U;BSP_GNSS_DmaIrqHandler();BSP_GNSS_DmaIrqHandler();
  assert(Info().dma_errors==1U&&!Info().initialized);dma_irq_error=0U;
  assert(BSP_GNSS_RecoverRx()==BSP_STATUS_OK);
  inject_error_on_enable=1U;assert(BSP_GNSS_Transmit(bytes,3U)==BSP_STATUS_OK);
  assert(!Info().initialized&&!Info().tx_busy&&!irq_enabled[16]&&!irq_enabled[38]);
  Cold();assert(BSP_GNSS_Transmit(bytes,3U)==BSP_STATUS_OK);
  assert(BSP_GNSS_RecoverRx()==BSP_STATUS_OK&&Info().tx_failures==1U&&!Info().tx_busy&&Info().tx_packets==0U);
  Cold();assert(BSP_GNSS_Transmit(bytes,3U)==BSP_STATUS_OK);dma_regs.CR&=~DMA_SxCR_EN;
  BSP_GNSS_RxIdleCallback(0U);assert(!Info().initialized&&!Info().tx_busy&&Info().tx_failures==1U);
  assert(BSP_GNSS_ConsumeRecoverRxRequest()==1U&&BSP_GNSS_RecoverRx()==BSP_STATUS_OK);
}
/** @brief 独立TX中止保留DMA寄存器/队列/RX句柄，失败事实仅记一次且可继续接收。 */
static void TestAbortOnlyTxKeepsRx(void)
{
  uint8_t bytes[3]={1U,2U,3U},received[64];
  BSP_GNSS_TransportInfo_t before,after;
  uint32_t dma_control,dma_state,dma_lock,rx_state,rx_interrupts;
  unsigned initializations,aborts;
  Cold();Advance(64U);BSP_GNSS_RxIdleCallback(0U);
  assert(BSP_GNSS_Transmit(bytes,sizeof(bytes))==BSP_STATUS_OK);
  before=Info();dma_control=dma_regs.CR;dma_state=hdma_gnss_rx.State;dma_lock=hdma_gnss_rx.Lock;
  rx_state=s_gnss_uart.RxState;rx_interrupts=uart_regs.CR1&(USART_CR1_UE|USART_CR1_RE|USART_CR1_IDLEIE|USART_CR1_PEIE);
  initializations=uart_init_calls;aborts=abort_calls;
  assert(BSP_GNSS_AbortTx()==BSP_STATUS_OK);after=Info();
  assert(!after.tx_busy&&after.tx_failures==1U&&after.tx_packets==0U&&after.recoveries==0U);
  assert(after.initialized&&after.ndtr==before.ndtr&&after.queued_bytes==before.queued_bytes);
  assert(after.rx_bytes==before.rx_bytes&&after.rx_drop_bytes==before.rx_drop_bytes);
  assert(dma_regs.CR==dma_control&&hdma_gnss_rx.State==dma_state&&hdma_gnss_rx.Lock==dma_lock&&s_gnss_uart.RxState==rx_state);
  assert((uart_regs.CR1&(USART_CR1_UE|USART_CR1_RE|USART_CR1_IDLEIE|USART_CR1_PEIE))==rx_interrupts);
  assert((uart_regs.CR3&USART_CR3_DMAR)&&irq_enabled[16]&&irq_enabled[38]);
  assert(!s_gnss_uart.TxXferCount&&s_gnss_uart.gState==HAL_UART_STATE_READY&&uart_init_calls==initializations&&abort_calls==aborts);
  assert(BSP_GNSS_ConsumeRecoverRxRequest()==0U&&BSP_GNSS_GetRxData(received,64U)==64U&&received[0]==0U&&received[63]==63U);
  Advance(11U);BSP_GNSS_RxIdleCallback(0U);assert(Info().queued_bytes==11U&&Info().rx_bytes==75U);
  assert(BSP_GNSS_AbortTx()==BSP_STATUS_OK&&Info().tx_failures==1U);
  assert(BSP_GNSS_Transmit(bytes,sizeof(bytes))==BSP_STATUS_OK);CompleteTx();assert(Info().tx_packets==1U);
  tx_submit_result=HAL_ERROR;assert(BSP_GNSS_Transmit(bytes,sizeof(bytes))==BSP_STATUS_ERROR);
  assert(Info().tx_failures==2U&&BSP_GNSS_AbortTx()==BSP_STATUS_OK&&Info().tx_failures==2U);
  Cold();assert(BSP_GNSS_Transmit(bytes,sizeof(bytes))==BSP_STATUS_OK);irq_enabled[38]=0U;primask=1U;
  assert(BSP_GNSS_AbortTx()==BSP_STATUS_OK&&primask==1U&&!irq_enabled[38]&&irq_enabled[16]);primask=0U;
}
/** @brief TX中止失败/真实RX失效/重开IRQ立即故障均保留原接收重建策略。 */
static void TestAbortTxFaultFallback(void)
{
  uint8_t bytes[3]={1U,2U,3U};unsigned aborted;
  Cold();Advance(23U);BSP_GNSS_RxIdleCallback(0U);
  assert(BSP_GNSS_Transmit(bytes,sizeof(bytes))==BSP_STATUS_OK);fail_tx_abort=1U;
  assert(BSP_GNSS_AbortTx()==BSP_STATUS_ERROR&&!Info().initialized&&Info().tx_failures==1U);
  assert(!irq_enabled[16]&&!irq_enabled[38]&&BSP_GNSS_ConsumeRecoverRxRequest()==1U);
  fail_tx_abort=0U;assert(BSP_GNSS_RecoverRx()==BSP_STATUS_OK&&Info().recoveries==1U&&Info().tx_failures==1U);
  Cold();aborted=tx_abort_calls;uart_regs.CR3&=~USART_CR3_DMAR;
  assert(BSP_GNSS_AbortTx()==BSP_STATUS_ERROR&&tx_abort_calls==aborted&&BSP_GNSS_ConsumeRecoverRxRequest());
  Cold();aborted=tx_abort_calls;s_gnss_uart.ErrorCode=HAL_UART_ERROR_DMA;
  assert(BSP_GNSS_AbortTx()==BSP_STATUS_ERROR&&tx_abort_calls==aborted&&BSP_GNSS_ConsumeRecoverRxRequest());
  Cold();aborted=tx_abort_calls;hdma_gnss_rx.ErrorCode=1U;
  assert(BSP_GNSS_AbortTx()==BSP_STATUS_ERROR&&tx_abort_calls==aborted&&BSP_GNSS_ConsumeRecoverRxRequest());
  Cold();assert(BSP_GNSS_Transmit(bytes,sizeof(bytes))==BSP_STATUS_OK);uart_regs.CR3|=USART_CR3_DMAT;
  aborted=tx_abort_calls;assert(BSP_GNSS_AbortTx()==BSP_STATUS_ERROR&&tx_abort_calls==aborted&&BSP_GNSS_ConsumeRecoverRxRequest());
  Cold();assert(BSP_GNSS_Transmit(bytes,sizeof(bytes))==BSP_STATUS_OK);inject_error_on_enable=1U;
  assert(BSP_GNSS_AbortTx()==BSP_STATUS_ERROR&&Info().uart_errors==1U&&Info().tx_failures==1U);
  assert(!Info().initialized&&!irq_enabled[16]&&!irq_enabled[38]&&BSP_GNSS_ConsumeRecoverRxRequest());
}
/** @brief 重开任一路IRQ即故障必须判本次恢复失败，成功计数留给下一真实可运行会话。 */
static void TestReopenIrqFailureNotSuccess(void)
{
  unsigned mode;
  for(mode=0U;mode<2U;mode++) {
    BSP_GNSS_TransportInfo_t info;
    Cold();
    if(mode==0U) { inject_error_on_enable=1U; }
    else { inject_dma_error_on_enable=1U; }
    assert(BSP_GNSS_RecoverRx()==BSP_STATUS_ERROR);info=Info();
    assert(!info.initialized && info.recoveries==0U && s_rx_recover_request);
    assert(!irq_enabled[BSP_GNSS_IRQn] && !irq_enabled[BSP_GNSS_RX_DMA_IRQn]);
    assert(!(uart_regs.CR3&USART_CR3_DMAR));
    if(mode==0U) { assert(info.uart_errors==1U && info.dma_errors==0U); }
    else { assert(info.uart_errors==0U && info.dma_errors==1U); }
    assert(BSP_GNSS_ConsumeRecoverRxRequest());
    assert(BSP_GNSS_RecoverRx()==BSP_STATUS_OK && Info().initialized && Info().recoveries==1U);
    Advance(100U);BSP_GNSS_RxIdleCallback(0U);
    assert(Info().queued_bytes==100U && Info().rx_bytes==100U);
  }
}
int main(void)
{
  assert(!Info().initialized&&Info().rx_bytes==0U);
  TestReceiveAndCounters();TestCopyTxAndCompletion();TestRebuildFailures();TestIrqErrorsAndAbort();
  TestAbortOnlyTxKeepsRx();TestAbortTxFaultFallback();
  TestReopenIrqFailureNotSuccess();
  assert(primask==0U&&sizeof(BSP_GNSS_TransportInfo_t)==40U);
  puts("P25 GNSS transport: DMA ring / copy TX / TC / TX-only abort preserves RX / HAL failures / RX recovery passed");
  return 0;
}
