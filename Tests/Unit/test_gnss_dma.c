/** @file test_gnss_dma.c
 * @brief 复用可失败HAL夹具，保留原NDTR中断插入与PRIMASK断言。
 */
#include "gnss_transport_fixture.h"
int main(void)
{
  Cold();
  s_rx_head=1U;s_rx_tail=1U;dma_regs.NDTR=BSP_GNSS_RX_BUF_SIZE-5U;counter_inject=1U;
  BSP_GNSS_RxIdleCallback(0U);
  assert(s_rx_head==10U&&s_rx_tail==1U&&s_rx_drop_count==0U);
  assert(primask==0U);
  primask=1U;dma_regs.NDTR=BSP_GNSS_RX_BUF_SIZE-11U;
  BSP_GNSS_RxIdleCallback(0U);
  assert(primask==1U&&s_rx_head==11U);
  puts("gnss DMA concurrency tests passed");return 0;
}
