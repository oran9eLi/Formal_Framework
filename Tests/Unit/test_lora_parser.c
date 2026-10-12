/** @file test_lora_parser.c @brief 真实 LoRa 驱动的解析恢复及独立统计 host 回归。 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#define BSP_CRITICAL_H
typedef struct { int unused; } UART_HandleTypeDef;
static uint32_t BSP_Critical_Enter(void) { return 0; }
static void BSP_Critical_Exit(uint32_t x) { (void)x; }
#include "../../Sensor/Src/lora_e22.c"

static const uint8_t *input;
static uint16_t input_size;
static uint32_t mock_ms = 1000U;
static uint8_t recover_request;
static uint32_t recovered;
uint32_t BSP_Time_GetTickMs(void) { return mock_ms; }
int32_t BSP_LoRa_Init(void) { return 0; }
void BSP_LoRa_SetMode(uint8_t m) { (void)m; }
uint8_t BSP_LoRa_IsReady(void) { return 1; }
uint8_t BSP_LoRa_IsBusy(void) { return 0; }
uint8_t BSP_LoRa_ProbeAuxPresent(void) { return 1; }
uint32_t BSP_LoRa_GetUartBaud(void) { return 9600; }
uint32_t BSP_LoRa_GetAirBps(void) { return 9600; }
uint8_t BSP_LoRa_IsTxBusy(void) { return 0; }
void BSP_LoRa_AbortTx(void) {}
int32_t BSP_LoRa_StartSend(const uint8_t *d,uint16_t n) { (void)d;(void)n;return 0; }
uint8_t BSP_LoRa_ConsumeRecoverRxRequest(void) { uint8_t r=recover_request; recover_request=0; return r; }
void BSP_LoRa_RecoverRx(void) { recovered++; }
uint32_t BSP_LoRa_GetRxOverflowCount(void) { return 0; }
uint16_t BSP_LoRa_GetRxCount(void) { return input_size; }
uint16_t BSP_LoRa_GetRxData(uint8_t *d,uint16_t n)
{
  if(n>input_size) { n=input_size; }
  memcpy(d,input,n); input+=n; input_size-=n; return n;
}
static void Feed(const uint8_t *p,uint16_t n,uint32_t ms)
{
  input=p; input_size=n; mock_ms=ms;
  assert(Lora_E22_Service(ms)==LORA_RESULT_OK);
  assert(input_size==0U);
}
static uint16_t Pack(uint8_t *frame, uint8_t sequence, uint8_t v1)
{
  mavlink_message_t msg;
  mavlink_status_t *tx=mavlink_get_channel_status(MAVLINK_COMM_1);
  tx->current_tx_seq=sequence;
  tx->flags=v1 ? MAVLINK_STATUS_FLAG_OUT_MAVLINK1 : 0U;
  mavlink_msg_heartbeat_pack_chan(42,191,MAVLINK_COMM_1,&msg,2,8,0,0,4);
  return mavlink_msg_to_send_buffer(frame,&msg);
}
static void Reset(void)
{
  /* 不帮助被测驱动清理 MAVLink 内部状态。 */
  assert(Lora_E22_Init()==LORA_RESULT_OK);
}
int main(int argc,char **argv)
{
  uint8_t frame[300], bad[300], i;
  uint16_t len;
  Lora_RxFrame_t out;
  const char *mode=(argc>1)?argv[1]:"all";
  Reset(); len=Pack(frame,0,0);
  if(strcmp(mode,"crc")!=0) {
    Feed(frame,11,0U); Feed(frame,len,500U);
    assert(s_rx_frame_count==1U);
    assert(Lora_E22_CopyRxFrame(&out)==LORA_RESULT_OK && out.frame_len==len);
    Feed(frame,11,510U); recover_request=1U; Feed(frame,len,520U);
    assert(recovered==1U && s_rx_frame_count==2U);
    Feed(frame,11,530U); Reset(); Feed(frame,len,540U);
    assert(s_rx_frame_count==1U);
    Feed(frame,11,UINT32_MAX-50U); Feed(frame,len,300U);
    assert(s_rx_frame_count==2U);
  }
  Reset(); len=Pack(frame,20,0); memcpy(bad,frame,len); bad[len-1]^=0x20;
  Feed(bad,len,1000U);
  { const uint8_t noise[]={0,1,2,0}; Feed(noise,sizeof(noise),1010U); }
  assert(s_crc_error_count==1U && s_parse_error_count==1U);
  assert(s_rx_drop_count==0U && s_rx_frame_count==0U);
  Feed(frame,len,1020U);
  assert(s_rx_frame_count==1U && s_rx_sequence_lost_count==0U);
  /* 序号跨 255→0，单个缺号和本地队列丢弃各自统计。 */
  Reset();
  for(i=0;i<10U;i++) { len=Pack(frame,(uint8_t)(250U+i),0); Feed(frame,len,1100U+i); }
  assert(s_rx_frame_count==10U && s_rx_drop_count==2U && s_rx_sequence_lost_count==0U);
  len=Pack(frame,5U,0); Feed(frame,len,1200U);
  assert(s_rx_sequence_lost_count==1U && s_rx_drop_count==3U);
  Reset(); len=Pack(frame,0U,1U); Feed(frame,len,1300U);
  assert(Lora_E22_CopyRxFrame(&out)==LORA_RESULT_OK && out.frame_len==len);
  puts("lora parser recovery and statistics passed");
  return 0;
}
