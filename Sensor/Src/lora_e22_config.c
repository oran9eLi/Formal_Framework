/**
 * @file lora_e22_config.c
 * @brief Driver 层 E22-400T30D 配置协议状态机。
 * @details 依赖 BSP 原始 UART/GPIO 与 E22 驱动边界；不调用 HAL、不处理业务请求。
 * M1=1/M0=0、9600 8N1；C1 00 07 读取，C0 05 01 CH 永久写单寄存器，再独立读回。
 * 每步最多等待 500 ms，整个事务最多 3 s；失败也必须退出模式并清除接收边界。
 */
#include "lora_e22_config.h"
#include "lora_e22.h"
#include "bsp_lora.h"
#include "bsp_critical.h"
#include <string.h>
/** @brief 配置阶段；每个阶段只由 comm 任务推进。 */
typedef enum { CFG_IDLE=0, CFG_DRAIN, CFG_ENTER, CFG_READ_SEND, CFG_READ_WAIT,
  CFG_WRITE_SEND, CFG_WRITE_WAIT, CFG_VERIFY_SEND, CFG_VERIFY_WAIT, CFG_EXIT } ConfigPhase_t;
static ConfigPhase_t s_phase;
static Lora_ConfigSnapshot_t s_state,s_published;
static uint8_t s_before[7],s_rx[10],s_tx[4],s_rx_len,s_write,s_target,s_verified,s_exit_failed;
static uint32_t s_started_ms,s_step_ms,s_uart_error,s_tx_tc;
/** @brief 发布短快照；禁止在临界区执行 UART 操作。 */
static void Config_Publish(void)
{ uint32_t key=BSP_Critical_Enter(); s_published=s_state; BSP_Critical_Exit(key); }
/** @brief 完成绑定事务号与操作目标，后续启动读取不能冒充旧写请求。 */
static void Config_Complete(void)
{
  s_state.completed_request_id=s_state.request_id; s_state.completed_result=s_state.result;
  s_state.completed_write=s_write; s_state.completed_channel=s_target; s_state.generation++;
}
/** @brief 清除硬件失效后的配置事实，显式中止在途事务并回正常模式。 */
void Lora_E22_ConfigInvalidate(void)
{
  BSP_LoRa_SetMode(0U);
  s_state.ready=0U; s_state.channel_known=0U; s_state.result=LORA_CFG_IO;
  if(s_state.busy) { Config_Complete(); }
  s_state.busy=0U; s_phase=CFG_IDLE; s_verified=0U; s_exit_failed=1U; Config_Publish();
}
/** @brief 课堂基础参数校验，非目标寄存器不自动写入；加密密码为只写字段，需人工核对。 */
static uint8_t Config_Profile(const uint8_t *p)
{ return (p[0]==0U && p[1]==0U && p[2]==0U && p[3]==0x64U && (p[4]&0xE7U)==0U && (p[6]&0xF0U)==0U)?1U:0U; }
/** @brief 更换阶段并启动有界等待。 */
static void Config_Phase(ConfigPhase_t phase,uint32_t now)
{ s_phase=phase; s_step_ms=now; }
/** @brief 恢复正常模式；尚未确认退出之前禁止正常发送。 */
static void Config_Exit(Lora_ConfigResult_t result,uint32_t now)
{
  s_state.result=result; s_state.ready=0U; s_verified=(result==LORA_CFG_READY)?1U:0U;
  BSP_LoRa_AbortTx(); BSP_LoRa_SetMode(0U); Lora_E22_ResetReceiveBoundary();
  Config_Phase(CFG_EXIT,now);
}
/** @brief 建立读写事务，原发送通过 DRAIN 阶段收尾。 */
static void Config_Begin(uint8_t write,uint8_t channel,uint32_t now)
{
  s_write=write; s_target=channel; s_verified=0U; s_exit_failed=0U;
  s_state.request_id++;
  s_state.busy=1U; s_state.ready=0U; s_state.result=LORA_CFG_BUSY;
  s_started_ms=now; s_uart_error=BSP_LoRa_GetUartErrorCount();
  Config_Phase(CFG_DRAIN,now); Config_Publish();
}
/** @brief 硬件重新初始化后读回，保留完成序号但撤销旧信道事实。 */
void Lora_E22_ConfigInit(uint32_t now_ms)
{ if(s_state.busy) { Lora_E22_ConfigInvalidate(); } s_state.channel_known=0U; Config_Begin(0U,0U,now_ms); }
/** @brief 校验基础参数后才允许学生写信道；不会重写其他寄存器。 */
int32_t Lora_E22_ConfigRequest(uint8_t channel,uint32_t now_ms)
{
  if(s_state.busy) { return 1; }
  if(!s_state.ready || !Lora_E22_HasInitializedSession() || !Lora_E22_IsPresent() ||
     !(channel==23U || channel==26U || channel==29U || channel==32U)) { return -1; }
  Config_Begin(1U,channel,now_ms); return 0;
}
/** @brief 接收精确有界响应，不交给 MAVLink 解析器；多余字节视作协议异常。 */
static int32_t Config_ReadResponse(uint8_t expected)
{
  uint16_t available=BSP_LoRa_GetRxCount();
  if(available>(uint16_t)(expected-s_rx_len)) { return -1; }
  if(available) { s_rx_len=(uint8_t)(s_rx_len+BSP_LoRa_GetRxData(s_rx+s_rx_len,available)); }
  if(s_rx_len<expected || BSP_LoRa_IsTxBusy()) { return 0; }
  if(BSP_LoRa_GetTxCompleteCount()==s_tx_tc) { return -1; }
  return 1;
}
/** @brief 发送一次原始配置命令，缓存直到 DMA 完成。 */
static void Config_Send(uint8_t write,ConfigPhase_t wait,uint32_t now)
{
  int32_t result;
  s_tx[0]=write?0xC0U:0xC1U; s_tx[1]=write?5U:0U; s_tx[2]=write?1U:7U; s_tx[3]=s_target;
  s_rx_len=0U; BSP_LoRa_ResetRxBoundary(); s_tx_tc=BSP_LoRa_GetTxCompleteCount();
  result=BSP_LoRa_StartSend(s_tx,write?4U:3U);
  if(result==0) { if(write) { s_state.channel_known=0U; } Config_Phase(wait,now); }
  else if(result<0) { Config_Exit(LORA_CFG_IO,now); }
}
/** @brief 一次最多推进一阶段；所有失败均经过正常模式退出。 */
int32_t Lora_E22_ConfigService(uint32_t now)
{
  int32_t response;
  uint8_t profile;
  if(s_phase==CFG_IDLE) { return (s_exit_failed || (s_state.result!=LORA_CFG_READY && s_state.result!=LORA_CFG_PROFILE))?-1:0; }
  if(s_phase!=CFG_EXIT && (BSP_LoRa_GetUartErrorCount()!=s_uart_error || (uint32_t)(now-s_started_ms)>2500U || (uint32_t)(now-s_step_ms)>500U)) {
    Config_Exit((BSP_LoRa_GetUartErrorCount()!=s_uart_error)?LORA_CFG_IO:LORA_CFG_TIMEOUT,now);
  }
  switch(s_phase) {
    case CFG_DRAIN:
      if(Lora_E22_IsTransportIdle() && BSP_LoRa_IsReady()) {
        if(BSP_LoRa_GetUartBaud()!=9600U) { Config_Exit(LORA_CFG_PROFILE,now); break; }
        Lora_E22_ResetReceiveBoundary(); BSP_LoRa_SetMode(2U); Config_Phase(CFG_ENTER,now);
      }
      break;
    case CFG_ENTER:
      if((uint32_t)(now-s_step_ms)>=20U && BSP_LoRa_IsReady()) { Config_Phase(CFG_READ_SEND,now); }
      break;
    case CFG_READ_SEND: case CFG_VERIFY_SEND:
      if(BSP_LoRa_IsReady()) { Config_Send(0U,(s_phase==CFG_READ_SEND)?CFG_READ_WAIT:CFG_VERIFY_WAIT,now); }
      break;
    case CFG_WRITE_SEND:
      if(BSP_LoRa_IsReady()) { Config_Send(1U,CFG_WRITE_WAIT,now); }
      break;
    case CFG_WRITE_WAIT:
      response=Config_ReadResponse(4U);
      if(response<0) { Config_Exit(LORA_CFG_VERIFY,now); }
      else if(response>0) {
        if(s_rx[0]!=0xC1U || s_rx[1]!=5U || s_rx[2]!=1U || s_rx[3]!=s_target) { Config_Exit(LORA_CFG_VERIFY,now); }
        else { Config_Phase(CFG_VERIFY_SEND,now); }
      }
      break;
    case CFG_READ_WAIT: case CFG_VERIFY_WAIT:
      response=Config_ReadResponse(10U);
      if(response<0) { Config_Exit(LORA_CFG_VERIFY,now); }
      else if(response>0) {
        if(s_rx[0]!=0xC1U || s_rx[1]!=0U || s_rx[2]!=7U || s_rx[8]>83U) { Config_Exit(LORA_CFG_VERIFY,now); break; }
        s_state.channel=s_rx[8]; s_state.channel_known=1U; profile=Config_Profile(s_rx+3U);
        if(s_phase==CFG_VERIFY_WAIT) {
          s_before[5]=s_target;
          Config_Exit((profile && memcmp(s_before,s_rx+3U,7U)==0)?LORA_CFG_READY:LORA_CFG_VERIFY,now);
        } else if(!profile) { Config_Exit(LORA_CFG_PROFILE,now); }
        else if(!s_write || s_state.channel==s_target) { Config_Exit(LORA_CFG_READY,now); }
        else { memcpy(s_before,s_rx+3U,7U); Config_Phase(CFG_WRITE_SEND,now); }
      }
      break;
    case CFG_EXIT:
      if(BSP_LoRa_GetUartErrorCount()!=s_uart_error) {
        s_state.result=LORA_CFG_IO; s_state.ready=0U; s_state.channel_known=0U;
        s_state.busy=0U; s_exit_failed=1U; Config_Complete(); s_phase=CFG_IDLE; break;
      }
      if((uint32_t)(now-s_step_ms)>=20U && BSP_LoRa_IsReady()) {
        /* 恢复正常模式后再主动探测 AUX，不信任拔出后的悬空高电平。 */
        if(!BSP_LoRa_ProbeAuxPresent() || BSP_LoRa_GetUartErrorCount()!=s_uart_error) {
          s_state.result=LORA_CFG_IO; s_state.channel_known=0U; s_verified=0U; s_exit_failed=1U;
        }
        Lora_E22_ResetReceiveBoundary(); s_state.ready=s_verified; s_state.busy=0U; Config_Complete(); s_phase=CFG_IDLE;
      } else if((uint32_t)(now-s_step_ms)>500U) {
        s_state.ready=0U; s_state.channel_known=0U; s_state.busy=0U; s_state.result=LORA_CFG_TIMEOUT;
        s_exit_failed=1U; Config_Complete(); s_phase=CFG_IDLE;
      }
      break;
    default: break;
  }
  Config_Publish(); return (s_phase!=CFG_IDLE)?1:(s_exit_failed?-1:0);
}
/** @brief 配置期间暂停普通探测和恢复。 */
uint8_t Lora_E22_ConfigBusy(void) { return s_state.busy; }
/** @brief 只读正常发送准入。 */
uint8_t Lora_E22_ConfigReady(void) { return s_state.ready; }
/** @brief 临界区复制已发布配置，不读取总线。 */
void Lora_E22_CopyConfig(Lora_ConfigSnapshot_t *out)
{ uint32_t key; if(!out) { return; } key=BSP_Critical_Enter(); *out=s_published; BSP_Critical_Exit(key); }
