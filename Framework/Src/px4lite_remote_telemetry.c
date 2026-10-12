/**
 * @file px4lite_remote_telemetry.c
 * @brief Framework 层八项发现摘要和唯一活动对端快照。
 * @details CommTask 观察身份并发布活动数据，UI 人工选择；没有控制、租约或自动选择。
 * 调度互斥保护发现/选择的计算，中断保持可用；固定容量，无运行期分配。
 */
#include "px4lite_remote_telemetry.h"
#include "px4lite_identity.h"
#include "px4lite_platform.h"
#include "px4lite_local_msglog.h"
#include "px4lite_log_message_ids.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#define IDENTITY_MAX_AGE_MS PX4LITE_LORA_LAB_IDENTITY_MAX_AGE_MS
/** @brief 同地址的不同长期身份各占一个小槽。 */
typedef struct {
  Px4Lite_RemoteNodeStatus_t view; /**< 轻量摘要，不含完整遥测。 */
  uint32_t seen_ms; /**< 最近帧时间，单位 ms。 */
  uint32_t identity_ms; /**< 最近 BASIC_ID 时间，单位 ms。 */
  uint32_t conflict_ms; /**< 满表冲突时间，单位 ms。 */
  uint8_t overflow_conflict; /**< 无空槽仍保留冲突闭锁。 */
  uint8_t sequence_seen; /**< 是否有序号基准。 */
  uint8_t sequence; /**< 最近序号 0..255。 */
  uint8_t heartbeat_seen; /**< 时间 0 与从未收到心跳区分。 */
  uint8_t link_stale; /**< 后台连接事件过期状态。 */
} RemoteDiscovery_t;
static RemoteDiscovery_t s_discovery[PX4LITE_REMOTE_NODE_MAX];
static Px4Lite_RemoteTelemetry_t s_active;
static uint32_t s_epoch,s_publish_sequence;
static uint8_t s_selected;
static char s_local_product[21];
/** @brief 清缓存并变更世代，选择前的 RX scratch 不得回填。 */
static void Remote_ClearActive(void) { memset(&s_active,0,sizeof(s_active)); s_epoch++; }
/** @brief 调度互斥内只比较固定八项身份。 */
static uint8_t Remote_Conflict(uint8_t node,uint32_t now)
{
  uint8_t i,j;
  for(i=0U;i<PX4LITE_REMOTE_NODE_MAX;i++) {
    const RemoteDiscovery_t *a=&s_discovery[i];
    if(a->view.node_id!=node) { continue; }
    if(a->overflow_conflict && Px4Lite_ElapsedMs(now,a->conflict_ms)<=IDENTITY_MAX_AGE_MS) { return 1U; }
    if(!a->view.identity_known || Px4Lite_ElapsedMs(now,a->identity_ms)>IDENTITY_MAX_AGE_MS) { continue; }
    if(node==Px4Lite_IdentityGetNodeId()) { return 1U; }
    for(j=(uint8_t)(i+1U);j<PX4LITE_REMOTE_NODE_MAX;j++) {
      const RemoteDiscovery_t *b=&s_discovery[j];
      if(b->view.node_id==node && b->view.identity_known && Px4Lite_ElapsedMs(now,b->identity_ms)<=IDENTITY_MAX_AGE_MS && memcmp(a->view.product_id,b->view.product_id,20U)!=0) { return 1U; }
    }
  }
  return 0U;
}
/** @brief 在调度互斥内解析唯一新鲜身份；冲突/未知返回容量哨兵。 */
static uint8_t Remote_IdentityIndex(uint8_t node,uint32_t now)
{
  uint8_t i;
  if(!node || Remote_Conflict(node,now)) { return PX4LITE_REMOTE_NODE_MAX; }
  for(i=0U;i<PX4LITE_REMOTE_NODE_MAX;i++) {
    if(s_discovery[i].view.node_id==node && s_discovery[i].view.identity_known && Px4Lite_ElapsedMs(now,s_discovery[i].identity_ms)<=IDENTITY_MAX_AGE_MS) { return i; }
  }
  return PX4LITE_REMOTE_NODE_MAX;
}
/** @brief 必须选中且身份解析成功；接收、状态和显示共用同一规则。 */
static uint8_t Remote_Accept(uint8_t node,uint32_t now)
{ return (node==s_selected && Remote_IdentityIndex(node,now)<PX4LITE_REMOTE_NODE_MAX)?1U:0U; }
/** @brief 初始化本机长期身份与未选中状态。 */
void Px4Lite_RemoteTelemetryInit(uint32_t now_ms)
{ Px4Lite_IdentityFormatVendorProductId(s_local_product,sizeof(s_local_product)); Px4Lite_RemoteTelemetryResetLink(now_ms); }
/** @brief 本机硬件断开清表和选择；其他页面继续展示 local。 */
void Px4Lite_RemoteTelemetryResetLink(uint32_t now_ms)
{ (void)now_ms; vTaskSuspendAll(); memset(s_discovery,0,sizeof(s_discovery)); s_selected=0U; Remote_ClearActive(); (void)xTaskResumeAll(); }
/** @brief 旧全局 remote 模式退出，只接受 LOCAL。 */
Px4Lite_Result_t Px4Lite_RemoteTelemetrySetMode(Px4Lite_RemoteMode_t mode,uint32_t now_ms)
{ (void)now_ms; return (mode==PX4LITE_REMOTE_MODE_LOCAL)?PX4LITE_OK:PX4LITE_NOT_READY; }
/** @brief 一级页面数据源固定 local。 */
Px4Lite_RemoteMode_t Px4Lite_RemoteTelemetryGetMode(void) { return PX4LITE_REMOTE_MODE_LOCAL; }
/** @brief 只维护发现摘要；同地址不同长期身份不能覆盖已有身份。 */
void Px4Lite_RemoteObserve(uint8_t node,uint8_t component,uint8_t sequence,uint32_t msg,const char *identity,uint32_t now)
{
  uint8_t i,index=PX4LITE_REMOTE_NODE_MAX,empty=PX4LITE_REMOTE_NODE_MAX,oldest=PX4LITE_REMOTE_NODE_MAX,conflict,old_conflict,changed=0U,discovered=0U,restored=0U;
  uint32_t age,max_age=0U;
  RemoteDiscovery_t *entry;
  if(!node || node>250U || (identity && memcmp(identity,s_local_product,20U)==0)) { return; }
  vTaskSuspendAll(); old_conflict=Remote_Conflict(node,now);
  if(!identity) {
    /* 无产品身份的负载不能归属到冲突中的任一设备；冲突消退后归属当前新鲜槽。 */
    if(old_conflict) { (void)xTaskResumeAll(); return; }
    index=Remote_IdentityIndex(node,now);
  }
  for(i=0U;i<PX4LITE_REMOTE_NODE_MAX;i++) {
    entry=&s_discovery[i];
    if(index==PX4LITE_REMOTE_NODE_MAX && entry->view.node_id==node &&
       (!identity || !entry->view.identity_known || memcmp(entry->view.product_id,identity,20U)==0)) { index=i; break; }
    if(!entry->view.node_id) { empty=i; }
    age=Px4Lite_ElapsedMs(now,entry->seen_ms);
    if(entry->view.node_id!=s_selected && (oldest==PX4LITE_REMOTE_NODE_MAX || age>max_age)) { oldest=i; max_age=age; }
  }
  if(index==PX4LITE_REMOTE_NODE_MAX) {
    index=(empty<PX4LITE_REMOTE_NODE_MAX)?empty:oldest;
    if(index==PX4LITE_REMOTE_NODE_MAX) {
      if(identity) { for(i=0U;i<PX4LITE_REMOTE_NODE_MAX;i++) { if(s_discovery[i].view.node_id==node) { s_discovery[i].overflow_conflict=1U; s_discovery[i].conflict_ms=now; } } }
      Remote_ClearActive(); (void)xTaskResumeAll(); return;
    }
    memset(&s_discovery[index],0,sizeof(s_discovery[index])); s_discovery[index].view.node_id=node;
    discovered=1U;
  }
  entry=&s_discovery[index]; entry->seen_ms=now; entry->view.system_id=node; entry->view.component_id=component;
  if(entry->link_stale && (entry->heartbeat_seen?(msg==0U):1U)) { entry->link_stale=0U; restored=1U; }
  if(identity) {
    if(Px4Lite_ElapsedMs(now,entry->identity_ms)>IDENTITY_MAX_AGE_MS) { changed=1U; }
    if(!entry->view.identity_known) { memcpy(entry->view.product_id,identity,20U); entry->view.product_id[20]='\0'; entry->view.identity_known=1U; changed=1U; }
    entry->identity_ms=now;
  }
  if(entry->sequence_seen) {
    uint8_t delta=(uint8_t)(sequence-entry->sequence);
    if(delta>1U && delta<=64U) { entry->view.rx_sequence_lost_count+=(uint32_t)(delta-1U); }
  }
  entry->sequence_seen=1U; entry->sequence=sequence; entry->view.rx_frame_count++;
  if(msg==0U) { entry->heartbeat_seen=1U; entry->view.heartbeat_known=1U; entry->view.last_heartbeat_ms=now; }
  else if(msg!=12900U) { entry->view.last_data_ms=now; }
  conflict=Remote_Conflict(node,now);
  if(node==s_selected && (conflict || changed || !Remote_Accept(node,now))) { Remote_ClearActive(); }
  (void)xTaskResumeAll();
  if(discovered) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_REMOTE_DISCOVERED,now,0U,PX4LITE_MODULE_LORA,1U); }
  if(restored) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_REMOTE_RESTORED,now,0U,PX4LITE_MODULE_LORA,0U); }
  if(conflict && !old_conflict) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_ID_CONFLICT,now,0U,PX4LITE_MODULE_LORA,1U); }
}
/** @brief 定期产生失联事实；正常心跳间隔不会重复写日志，计算期间不关闭中断。 */
void Px4Lite_RemoteDiscoveryRun(uint32_t now)
{
  uint8_t i,stale,events=0U;
  vTaskSuspendAll();
  for(i=0U;i<PX4LITE_REMOTE_NODE_MAX;i++) {
    RemoteDiscovery_t *entry=&s_discovery[i];
    if(!entry->view.node_id) { continue; }
    stale=(entry->heartbeat_seen)?(Px4Lite_ElapsedMs(now,entry->view.last_heartbeat_ms)>PX4LITE_REMOTE_HEARTBEAT_STALE_MS):(Px4Lite_ElapsedMs(now,entry->seen_ms)>IDENTITY_MAX_AGE_MS);
    if(stale && !entry->link_stale) { entry->link_stale=1U; events++; }
  }
  (void)xTaskResumeAll();
  while(events) { Px4Lite_LocalMsgLogEvent(PX4LITE_LOGMSG_REMOTE_STALE,now,0U,PX4LITE_MODULE_LORA,1U); events--; }
}
/** @brief RX 解码准入，提交时必须再次核验。 */
uint8_t Px4Lite_RemoteCanAccept(uint8_t node,uint32_t now)
{ uint8_t result; vTaskSuspendAll(); result=Remote_Accept(node,now); (void)xTaskResumeAll(); return result; }
/** @brief 选中身份明确时复制唯一缓存及世代；尚无数据也返回 OK。 */
Px4Lite_Result_t Px4Lite_RemoteTelemetryCopyNode(uint8_t node,Px4Lite_RemoteTelemetry_t *out)
{
  uint8_t accept; uint32_t now=Px4Lite_PlatformGetMs();
  if(!out) { return PX4LITE_INVALID_PARAM; }
  vTaskSuspendAll(); accept=Remote_Accept(node,now);
  if(accept) { *out=s_active; out->observation_generation=s_epoch; } else { memset(out,0,sizeof(*out)); }
  (void)xTaskResumeAll(); return accept?PX4LITE_OK:PX4LITE_NOT_READY;
}
/** @brief 防止选择交错或发现冲突后旧 scratch 污染活动对象。 */
Px4Lite_Result_t Px4Lite_RemoteTelemetryCommitNode(uint8_t node,const Px4Lite_RemoteTelemetry_t *snapshot,uint32_t now)
{
  if(!snapshot || snapshot->system_id!=node) { return PX4LITE_INVALID_PARAM; }
  vTaskSuspendAll();
  if(!Remote_Accept(node,now) || snapshot->observation_generation!=s_epoch) { (void)xTaskResumeAll(); return PX4LITE_NOT_READY; }
  s_active=*snapshot; s_active.header.sequence=++s_publish_sequence; (void)xTaskResumeAll(); return PX4LITE_OK;
}
/** @brief 应用仅复制有效活动数据，再按真实时刻标记字段过期。 */
Px4Lite_Result_t Px4Lite_CopyRemoteTelemetry(Px4Lite_RemoteTelemetry_t *out)
{
  uint8_t valid; uint32_t now=Px4Lite_PlatformGetMs();
  if(!out) { return PX4LITE_INVALID_PARAM; }
  vTaskSuspendAll(); valid=(Remote_Accept(s_selected,now) && s_active.header.valid)?1U:0U;
  if(valid) { *out=s_active; } else { memset(out,0,sizeof(*out)); }
  (void)xTaskResumeAll(); if(valid) { Px4Lite_RemoteTelemetryRefreshFreshness(out,now); } return valid?PX4LITE_OK:PX4LITE_NOT_READY;
}
/** @brief 人工选择清空数据，等待选择之后的新帧；未知身份可选但不组装完整负载。 */
Px4Lite_Result_t Px4Lite_SelectRemoteNode(uint8_t node)
{
  uint8_t i,found=0U; uint32_t now=Px4Lite_PlatformGetMs();
  vTaskSuspendAll();
  for(i=0U;i<PX4LITE_REMOTE_NODE_MAX;i++) { if(s_discovery[i].view.node_id==node && node) { found=1U; } }
  if(!found || Remote_Conflict(node,now)) { (void)xTaskResumeAll(); return PX4LITE_NOT_READY; }
  s_selected=node; Remote_ClearActive(); (void)xTaskResumeAll(); return PX4LITE_OK;
}
/** @brief 短临界区复制当前观察对象。 */
Px4Lite_Result_t Px4Lite_GetSelectedRemoteNode(uint8_t *node)
{ if(!node) { return PX4LITE_INVALID_PARAM; } taskENTER_CRITICAL(); *node=s_selected; taskEXIT_CRITICAL(); return PX4LITE_OK; }
/** @brief 用于发现列表与活动观察的统一状态转换；调用方已持有调度互斥。 */
static void Remote_FillStatus(RemoteDiscovery_t *entry,Px4Lite_RemoteNodeStatus_t *out,uint32_t now)
{
  uint32_t total;
  *out=entry->view;
  out->identity_known=(entry->view.identity_known && Px4Lite_ElapsedMs(now,entry->identity_ms)<=IDENTITY_MAX_AGE_MS)?1U:0U;
  out->id_conflict=Remote_Conflict(entry->view.node_id,now);
  total=out->rx_frame_count+out->rx_sequence_lost_count;
  out->rx_loss_rate_x10=total?(uint16_t)(((uint64_t)out->rx_sequence_lost_count*1000U)/total):0U;
  out->state=((entry->heartbeat_seen && Px4Lite_ElapsedMs(now,out->last_heartbeat_ms)>PX4LITE_REMOTE_HEARTBEAT_STALE_MS) ||
    (!entry->heartbeat_seen && Px4Lite_ElapsedMs(now,entry->seen_ms)>IDENTITY_MAX_AGE_MS))?PX4LITE_REMOTE_NODE_STALE:PX4LITE_REMOTE_NODE_DISCOVERED;
  if(entry->view.node_id==s_selected && s_active.header.valid && out->identity_known &&
     !out->id_conflict && Px4Lite_ElapsedMs(now,out->last_data_ms)<=PX4LITE_REMOTE_DATA_STALE_MS) { out->state=PX4LITE_REMOTE_NODE_ACTIVE; }
}
/** @brief 活动观察取得唯一有效身份；无有效身份时仍传递冲突/未知事实。 */
Px4Lite_Result_t Px4Lite_CopySelectedRemoteStatus(Px4Lite_RemoteNodeStatus_t *out,uint32_t now)
{
  uint8_t i,index;
  if(!out) { return PX4LITE_INVALID_PARAM; }
  vTaskSuspendAll(); memset(out,0,sizeof(*out)); index=Remote_IdentityIndex(s_selected,now);
  if(index==PX4LITE_REMOTE_NODE_MAX) {
    for(i=0U;i<PX4LITE_REMOTE_NODE_MAX;i++) { if(s_selected && s_discovery[i].view.node_id==s_selected) { index=i; break; } }
  }
  if(index<PX4LITE_REMOTE_NODE_MAX) { Remote_FillStatus(&s_discovery[index],out,now); }
  (void)xTaskResumeAll(); return (index<PX4LITE_REMOTE_NODE_MAX)?PX4LITE_OK:PX4LITE_NOT_READY;
}
/** @brief 固定八项摘要计算，保持中断可用；身份未知/冲突/过期显式传递。 */
Px4Lite_Result_t Px4Lite_CopyRemoteNodeStatuses(Px4Lite_RemoteNodeStatus_t *out,uint8_t capacity,uint8_t *count,uint32_t now)
{
  uint8_t i,n=0U;
  if(!out || !count) { return PX4LITE_INVALID_PARAM; }
  vTaskSuspendAll();
  for(i=0U;i<PX4LITE_REMOTE_NODE_MAX && n<capacity;i++) {
    RemoteDiscovery_t *entry=&s_discovery[i]; if(!entry->view.node_id) { continue; }
    Remote_FillStatus(entry,&out[n],now);
    n++;
  }
  (void)xTaskResumeAll(); *count=n; return n?PX4LITE_OK:PX4LITE_NOT_READY;
}
