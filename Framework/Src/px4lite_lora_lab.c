/**
 * @file px4lite_lora_lab.c
 * @brief Framework 层手动教学发送状态机，默认停止且没有历史发送队列。
 * @details 依赖平台事实、配置服务和 MAVLink 编码入口；CommTask 是会话与配置唯一写者。
 * UI 请求通过短临界区复制；运行期无分配、阻塞等待或自动开始实验。
 */
#include "px4lite_lora_lab.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_config.h"
#include "px4lite_local_msglog.h"
#include "px4lite_log_message_ids.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
static Px4Lite_LabSnapshot_t s_state,s_published;
static Px4Lite_LabRequest_t s_request;
static uint8_t s_request_valid,s_stop_request;
static uint32_t s_next_ms;
static uint32_t s_request_serial;
static uint32_t s_node_generation,s_channel_request;
static uint8_t s_channel_target;
static uint8_t s_node_waiting,s_channel_waiting,s_no_data_logged;
/** @brief 短临界区发布状态，不执行计算或日志。 */
static void Lab_Publish(void)
{ taskENTER_CRITICAL(); s_published=s_state; s_published.request_pending=s_request_valid; taskEXIT_CRITICAL(); }
/** @brief 结束待发送组，已提交的 DMA 帧不撤回。 */
static void Lab_Stop(Px4Lite_LabResult_t result)
{
  if(s_state.running || s_state.pending || s_state.result!=result) { s_state.generation++; }
  s_state.running=0U; s_state.pending=0U; s_state.result=result; Px4Lite_MavlinkCancelLabGroup();
}
/** @brief 固定范围和保守空口预算，基于课堂 9600 bit/s、40% 上限。 */
static uint8_t Lab_Valid(const Px4Lite_LabRequest_t *r)
{
  uint32_t bytes;
  if(r->data>PX4LITE_LAB_ENVIRONMENT || !(r->interval_ms==1000U || r->interval_ms==2000U || r->interval_ms==5000U || r->interval_ms==10000U)) { return 0U; }
  bytes=(r->data==PX4LITE_LAB_POWER)?156U:((r->data==PX4LITE_LAB_ENVIRONMENT)?100U:90U);
  /* 基础广播预留 40 byte/s；按 10 bit/byte 与完整最长帧留有余量。 */
  return ((bytes*1000U/r->interval_ms+40U)*10U*100U<=PX4LITE_LORA_LAB_AIR_BPS*PX4LITE_LORA_TX_DUTY_LIMIT_PCT)?1U:0U;
}
/** @brief 初始化只建立会话，不影响已从 Flash 加载的本次身份。 */
void Px4Lite_LoraLabInit(uint32_t now_ms)
{
  memset(&s_state,0,sizeof(s_state)); s_state.data=PX4LITE_LAB_ATTITUDE;
  s_state.interval_ms=2000U; s_next_ms=now_ms;
  Px4Lite_CommConfigCopy(&s_state.address); Px4Lite_LoRaCopyConfig(&s_state.radio); Lab_Publish();
}
/** @brief 单槽命令入口；Stop 撤销所有未消费操作，后续点击需重新提交。 */
Px4Lite_Result_t Px4Lite_LoraLabRequest(const Px4Lite_LabRequest_t *request)
{
  Px4Lite_Result_t result=PX4LITE_OK;
  if(!request || request->operation>PX4LITE_LAB_RESTORE) { return PX4LITE_INVALID_PARAM; }
  taskENTER_CRITICAL();
  if(request->operation==PX4LITE_LAB_STOP) { s_stop_request=1U; s_request_valid=0U; }
  else if(request->operation==PX4LITE_LAB_EDIT && !s_stop_request) { s_request=*request; s_request_valid=1U; }
  else if(s_request_valid || s_stop_request) { result=PX4LITE_BUSY; }
  else { s_request=*request; s_request_valid=1U; }
  if(result==PX4LITE_OK) { s_request_serial++; }
  taskEXIT_CRITICAL(); return result;
}
/** @brief 配置及手动操作在 CommTask 内串行执行，每周期只消费一次。 */
void Px4Lite_LoraLabRun(uint32_t now_ms)
{
  Px4Lite_LabRequest_t request;
  Px4Lite_CommDebugInfo_t info;
  Px4Lite_Result_t result;
  uint8_t stop,valid,was_config_busy;
  uint32_t request_id;
  Px4Lite_CommConfigRun(now_ms);
  was_config_busy=(s_state.radio.busy || s_state.address.busy)?1U:0U;
  Px4Lite_LoRaCopyConfig(&s_state.radio); Px4Lite_CommConfigCopy(&s_state.address);
  if(was_config_busy && !s_state.radio.busy && !s_state.address.busy && s_state.result==PX4LITE_LAB_CONFIG_BUSY) {
    Lab_Stop(PX4LITE_LAB_STOPPED);
  }
  if(s_node_waiting && s_state.address.generation!=s_node_generation) {
    Px4Lite_LocalMsgLogEvent((s_state.address.result==PX4LITE_CONFIG_SAVED)?PX4LITE_LOGMSG_NODE_SAVED:PX4LITE_LOGMSG_NODE_SAVE_FAILED,now_ms,0U,PX4LITE_MODULE_LORA,0U);
    s_node_waiting=0U;
  }
  if(s_channel_waiting && s_state.radio.completed_request_id==s_channel_request) {
    uint8_t applied=(s_state.radio.completed_result==2U && s_state.radio.completed_write &&
                     s_state.radio.completed_channel==s_channel_target)?1U:0U;
    Px4Lite_LocalMsgLogEvent(applied?PX4LITE_LOGMSG_CHANNEL_APPLIED:PX4LITE_LOGMSG_CHANNEL_FAILED,now_ms,0U,PX4LITE_MODULE_LORA,0U);
    s_channel_waiting=0U;
  }
  Px4Lite_LoRaGetDebugInfo(&info); s_state.rx_frames=info.rx_frame_count;
  s_state.crc_errors=info.crc_error_count; s_state.rx_drops=info.rx_drop_count; s_state.loss_rate_x10=info.rx_loss_rate_x10;
  taskENTER_CRITICAL(); stop=s_stop_request; s_stop_request=0U; valid=s_request_valid;
  request=s_request; request_id=s_request_serial; s_request_valid=0U; taskEXIT_CRITICAL();
  if(stop || valid) { s_state.request_id=request_id; }
  if(stop) {
    if(s_state.running || s_state.pending) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_LAB_STOP,now_ms,0U,PX4LITE_MODULE_LORA,0U); }
    Lab_Stop(PX4LITE_LAB_STOPPED);
  }
  if(s_state.radio.busy || s_state.address.busy) {
    if(s_state.running || s_state.pending) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_LAB_ABORTED,now_ms,0U,PX4LITE_MODULE_LORA,0U); }
    Lab_Stop(PX4LITE_LAB_CONFIG_BUSY);
  }
  else if(!s_state.radio.ready || !Px4Lite_LoRaIsPresent()) {
    if(s_state.running || s_state.pending) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_LAB_ABORTED,now_ms,0U,PX4LITE_MODULE_LORA,0U); }
    Lab_Stop((s_state.radio.result==3U)?PX4LITE_LAB_PROFILE:PX4LITE_LAB_UNAVAILABLE);
  }
  if(valid && !stop) {
    if(request.operation==PX4LITE_LAB_EDIT) {
      Lab_Stop(PX4LITE_LAB_STOPPED);
      if(Lab_Valid(&request)) { s_state.data=request.data; s_state.interval_ms=request.interval_ms; }
      else { s_state.result=PX4LITE_LAB_RATE_LIMIT; }
    } else if(request.operation==PX4LITE_LAB_SAVE_NODE || request.operation==PX4LITE_LAB_APPLY_CHANNEL || request.operation==PX4LITE_LAB_RESTORE) {
      Lab_Stop(PX4LITE_LAB_STOPPED);
      if(s_state.address.busy || s_state.radio.busy) {
        s_state.result=PX4LITE_LAB_CONFIG_BUSY; s_state.generation++; Lab_Publish(); return;
      }
      if(request.operation==PX4LITE_LAB_SAVE_NODE && (request.node_id==0U || request.node_id>250U)) {
        s_state.result=PX4LITE_LAB_ERROR; s_state.generation++;
        Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_NODE_SAVE_FAILED,now_ms,0U,PX4LITE_MODULE_LORA,0U); Lab_Publish(); return;
      }
      if(request.operation!=PX4LITE_LAB_APPLY_CHANNEL) {
        s_node_generation=s_state.address.generation; s_node_waiting=1U;
        result=Px4Lite_CommConfigRequest((request.operation==PX4LITE_LAB_RESTORE)?0U:request.node_id,now_ms);
        if(result!=PX4LITE_OK) {
          s_state.result=PX4LITE_LAB_ERROR;
          if(result==PX4LITE_BUSY || result==PX4LITE_INVALID_PARAM) { s_node_waiting=0U; Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_NODE_SAVE_FAILED,now_ms,0U,PX4LITE_MODULE_LORA,0U); }
        }
      }
      if(request.operation!=PX4LITE_LAB_SAVE_NODE) {
        s_channel_target=(request.operation==PX4LITE_LAB_RESTORE)?23U:request.channel;
        result=Px4Lite_LoRaApplyChannel((request.operation==PX4LITE_LAB_RESTORE)?23U:request.channel,now_ms);
        if(result!=PX4LITE_OK) { s_state.result=PX4LITE_LAB_ERROR; Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_CHANNEL_FAILED,now_ms,0U,PX4LITE_MODULE_LORA,0U); }
        else { Px4Lite_LoRaCopyConfig(&s_state.radio); s_channel_request=s_state.radio.request_id; s_channel_waiting=1U; }
      }
    } else if(request.operation==PX4LITE_LAB_SEND || request.operation==PX4LITE_LAB_START) {
      if(s_state.radio.busy || s_state.address.busy) { s_state.result=PX4LITE_LAB_CONFIG_BUSY; }
      else if(!s_state.radio.ready || !Px4Lite_LoRaIsPresent()) { s_state.result=(s_state.radio.result==3U)?PX4LITE_LAB_PROFILE:PX4LITE_LAB_UNAVAILABLE; }
      else if(!Lab_Valid(&request)) { s_state.result=PX4LITE_LAB_RATE_LIMIT; }
      else if(s_state.pending) { s_state.result=PX4LITE_LAB_CONFIG_BUSY; }
      else if(Px4Lite_MavlinkLabDataReady((uint8_t)request.data,now_ms)!=PX4LITE_OK) {
        Lab_Stop(PX4LITE_LAB_NO_DATA);
        if(!s_no_data_logged) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_LAB_NO_DATA,now_ms,0U,PX4LITE_MODULE_LORA,0U); s_no_data_logged=1U; }
      }
      else {
        s_state.data=request.data; s_state.interval_ms=request.interval_ms; s_state.pending=1U;
        s_state.running=(request.operation==PX4LITE_LAB_START)?1U:s_state.running;
        s_state.result=s_state.running?PX4LITE_LAB_RUNNING:PX4LITE_LAB_STOPPED; s_next_ms=now_ms;
        Px4Lite_LocalMsgLogEvent((request.operation==PX4LITE_LAB_START)?PX4LITE_LOGMSG_LAB_START:PX4LITE_LOGMSG_LAB_SEND,now_ms,0U,PX4LITE_MODULE_LORA,1U);
      }
    }
    s_state.generation++;
  }
  if(s_state.running && !s_state.pending && (int32_t)(now_ms-s_next_ms)>=0) { s_state.pending=1U; }
  Px4Lite_LoRaCopyConfig(&s_state.radio); Px4Lite_CommConfigCopy(&s_state.address); Lab_Publish();
}
/** @brief 查询只由 CommTask 使用的待发送标志。 */
uint8_t Px4Lite_LoraLabPending(Px4Lite_LabData_t *data)
{ if(data) { *data=s_state.data; } return s_state.pending; }
/** @brief 提交事实与失败反馈，周期从完成后计算，不追赶积压。 */
void Px4Lite_LoraLabTxResult(Px4Lite_Result_t result,uint8_t complete,uint32_t now_ms)
{
  if(result==PX4LITE_OK) { s_state.queued_frames++; s_state.result=PX4LITE_LAB_QUEUED; s_no_data_logged=0U; }
  else if(result==PX4LITE_NOT_READY || result==PX4LITE_STALE) {
    s_state.result=PX4LITE_LAB_NO_DATA;
    if(!s_no_data_logged) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_LAB_NO_DATA,now_ms,0U,PX4LITE_MODULE_LORA,0U); s_no_data_logged=1U; }
  }
  else if(result!=PX4LITE_BUSY && result!=PX4LITE_IDLE) { Lab_Stop(PX4LITE_LAB_ERROR); }
  if(complete) { s_state.pending=0U; s_next_ms=now_ms+s_state.interval_ms; }
  s_state.generation++; Lab_Publish();
}
/** @brief 应用复制状态，禁止绕过此接口访问请求槽。 */
void Px4Lite_LoraLabCopy(Px4Lite_LabSnapshot_t *out)
{ if(out) { taskENTER_CRITICAL(); *out=s_published; taskEXIT_CRITICAL(); } }
