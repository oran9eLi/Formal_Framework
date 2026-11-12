/**
 * @file test_gnss_hardware_reconnect.c
 * @brief 真实BSP、HAL故障模型与GNSS Driver联合验证反复断流后重新接收。
 * @details 只模拟外部UART/DMA寄存器，不替换BSP或Driver恢复函数；不代表实板长稳。
 */
#include "gnss_transport_fixture.h"
#define BSP_CRITICAL_H
/** @brief Driver使用同一个PRIMASK模型，嵌套复制不能提前开放中断。 */
static uint32_t BSP_Critical_Enter(void)
{ uint32_t saved=__get_PRIMASK();__disable_irq();return saved; }
/** @brief 仅恢复原本允许的中断状态。 */
static void BSP_Critical_Exit(uint32_t saved) { if(!saved) { __enable_irq(); } }
#include "../../Sensor/Src/sensor_nmea.c"
#include "../../Sensor/Src/sensor_ubx.c"
#include "../../Sensor/Src/sensor_gnss.c"

static const uint8_t s_good_gga[]="$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

/** @brief 仅有完整接收硬件链时注入真实线字节，由生产BSP根据NDTR观察。 */
static void Wire(const uint8_t *bytes,uint16_t count)
{
  uint16_t i;
  assert(s_initialized && (uart_regs.CR1&USART_CR1_RE) && (uart_regs.CR3&USART_CR3_DMAR) && (dma_regs.CR&DMA_SxCR_EN));
  for(i=0U;i<count;i++) {
    s_rx_buf[fake_write_pos]=bytes[i];fake_write_pos=(uint16_t)((fake_write_pos+1U)%BSP_GNSS_RX_BUF_SIZE);
    dma_regs.NDTR=fake_write_pos?BSP_GNSS_RX_BUF_SIZE-fake_write_pos:BSP_GNSS_RX_BUF_SIZE;
  }
}
/** @brief 模拟独立UART发送中断与最后TC，防止测试夹具无意制造另一个TX卡死。 */
static void CompleteTx(void)
{
  unsigned limit=100U;
  while(s_tx_busy && s_initialized && irq_enabled[BSP_GNSS_IRQn] && limit--) {
    uart_regs.SR=s_gnss_uart.TxXferCount?USART_SR_TXE:USART_SR_TC;
    BSP_GNSS_UartIrqHandler();
  }
  assert(limit>0U);
}
/** @brief 推进真实owner Service；异步TX在其后独立完成，不注入协议ACK。 */
static Gnss_Result_t Run(uint32_t now)
{ Gnss_Result_t result=Sensor_GNSS_Service(now);CompleteTx();return result; }
/** @brief 每个场景一次真正BSP冷启动及Driver冷状态，累积事实在场景内部保持。 */
static void NewBoard(void)
{
  Cold();s_driver_started=0U;s_diag_ready=0U;
  Gnss_SetDiagnosticCallback(NULL);Gnss_SetUbxDiagnosticCallback(NULL);Gnss_SetRawRejectCallback(NULL);
  assert(Sensor_GNSS_Init()==GNSS_RESULT_OK);
}
/** @brief 模拟真实UART错误中断，其停收/请求/计数全由生产代码执行。 */
static void UartFault(void)
{ uart_regs.SR=USART_SR_FE|USART_SR_IDLE;BSP_GNSS_UartIrqHandler(); }
/** @brief 原样复制Driver事实，不通过修改snapshot模拟恢复。 */
static Gnss_Snapshot_t Snapshot(void)
{ Gnss_Snapshot_t out;(void)Sensor_GNSS_CopySnapshot(&out);return out; }

/** @brief 重开中断立即再报错必须失败且两IRQ保持关闭，后续2秒退避后真正收数。 */
static void TestFaultDuringRebuildThenRecover(void)
{
  uint32_t sequence;
  NewBoard();Wire(s_good_gga,sizeof(s_good_gga)-1U);assert(Run(100U)==GNSS_RESULT_OK);sequence=Snapshot().rx_sequence;
  UartFault();inject_error_on_enable=1U;
  assert(Run(200U)==GNSS_RESULT_IO_ERROR);
  assert(!Info().initialized && !irq_enabled[BSP_GNSS_IRQn] && !irq_enabled[BSP_GNSS_RX_DMA_IRQn]);
  assert(Info().recoveries==0U && Info().uart_errors==2U);
  assert(Run(2199U)==GNSS_RESULT_IO_ERROR && Info().recoveries==0U);
  assert(Run(2200U)==GNSS_RESULT_OK && Info().initialized && Info().recoveries==1U);
  Wire(s_good_gga,sizeof(s_good_gga)-1U);assert(Run(2210U)==GNSS_RESULT_OK);
  assert(Snapshot().rx_sequence>sequence && Snapshot().last_rx_ms==2210U && Snapshot().fix_valid);
}

/** @brief 反复UART故障不耗尽重试，也不能被解析器软复位取消硬件pending。 */
static void TestRepeatedFaultsAndWarmRequests(void)
{
  unsigned cycle;uint32_t now=100U,old_sequence;
  NewBoard();Wire(s_good_gga,sizeof(s_good_gga)-1U);assert(Run(now)==GNSS_RESULT_OK);
  for(cycle=0U;cycle<6U;cycle++) {
    old_sequence=Snapshot().rx_sequence;now+=2100U;UartFault();
    assert(Run(now)==GNSS_RESULT_OK && Info().initialized);
    Wire(s_good_gga,sizeof(s_good_gga)-1U);assert(Run(now+10U)==GNSS_RESULT_OK);
    assert(Snapshot().rx_sequence>old_sequence && Snapshot().last_rx_ms==now+10U);
  }
  assert(Info().recoveries==6U && Info().uart_errors==6U);
  now+=100U;UartFault();assert(Run(now)==GNSS_RESULT_IO_ERROR);
  Sensor_GNSS_RequestReinit();assert(Run(now+100U)==GNSS_RESULT_IO_ERROR);
  assert(Run(now+1900U)==GNSS_RESULT_OK && Info().recoveries==7U);
  Wire(s_good_gga,sizeof(s_good_gga)-1U);assert(Run(now+1910U)==GNSS_RESULT_OK);
  assert(Snapshot().rx_sequence!=0U && Snapshot().last_rx_ms==now+1910U);
}

/** @brief 无中断报告的接收使能故障也由owner健康轮询发现，不依赖TX碰巧失败。 */
static void TestOwnerDetectsMissingReceiveEnable(void)
{
  NewBoard();assert(Run(0U)==GNSS_RESULT_OK);uart_regs.CR1&=~USART_CR1_RE;
  assert(Run(100U)==GNSS_RESULT_IO_ERROR && !Info().initialized);
  assert(Run(110U)==GNSS_RESULT_OK && Info().initialized && Info().recoveries==1U);
  Wire(s_good_gga,sizeof(s_good_gga)-1U);assert(Run(120U)==GNSS_RESULT_OK && Snapshot().last_rx_ms==120U);
}

/** @brief 长时间静默仍有限重建，软请求不推迟监测；输入恢复后真实帧重新可用。 */
static void TestSilenceThenReconnect(void)
{
  uint32_t now;
  NewBoard();
  for(now=0U;now<=30000U;now+=10U) {
    if(now && now%2000U==0U) { Sensor_GNSS_RequestReinit(); }
    assert(Run(now)==GNSS_RESULT_OK);
  }
  assert(Info().recoveries>=4U && Info().recoveries<=5U && Snapshot().rx_sequence==0U);
  Wire(s_good_gga,sizeof(s_good_gga)-1U);assert(Run(30010U)==GNSS_RESULT_OK);
  assert(Snapshot().rx_sequence==1U && Snapshot().last_rx_ms==30010U && Snapshot().fix_valid);
}

/** @brief 联合真实恢复链检查，GNSS物理接线/信号质量仍需实板计数。 */
int main(void)
{
  TestFaultDuringRebuildThenRecover();TestRepeatedFaultsAndWarmRequests();
  TestOwnerDetectsMissingReceiveEnable();TestSilenceThenReconnect();
  assert(primask==0U);
  puts("GNSS real BSP+Driver reconnect: IRQ reopen failure / 2s retry / repeated FE / owner health / 30s silence / real GGA PASS");
  return 0;
}
