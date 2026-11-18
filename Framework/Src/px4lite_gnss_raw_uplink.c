/**
 * @file px4lite_gnss_raw_uplink.c
 * @brief Framework独立GNSS原wire有界队列、分片与异常统计。
 * @details Sensor经Adapter单写入；Comm单消费。静态4槽，活跃帧固定不覆盖，
 *          CRC16只在临界区外对固定帧计算；无解析、外设访问、动态分配或新增任务。
 */
#include "px4lite_gnss_raw_uplink.h"
#include "px4lite_config.h"
#include "px4lite_platform.h"
#include "FreeRTOS.h"
#include "task.h"
#include <limits.h>
#include <string.h>

#if PX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK
#define RAW_SLOT_FREE 0U
#define RAW_SLOT_WRITING 1U
#define RAW_SLOT_READY 2U
#define RAW_SLOT_ACTIVE 3U
#define RAW_OFFER_FRAME 1U
#define RAW_OFFER_STATS 2U

/** @brief 固定槽；Sensor先预留再复制，READY发布后内容只在释放前读取。 */
typedef struct {
  uint32_t frame_id, received_ms; /**< 输入代数及Sensor完整帧消费时刻，ms。 */
  uint16_t length; /**< 完整wire字节数，2..1024。 */
  uint8_t protocol, nmea_type, class_id, message_id; /**< 原协议1/2及真实类型元数据。 */
  uint8_t checksum_valid, state; /**< 原校验事实与FREE/WRITING/READY/ACTIVE所有权。 */
  uint8_t bytes[PX4LITE_GNSS_RAW_UPLINK_FRAME_MAX]; /**< 完整原wire，不修正校验或终止符。 */
} RawSlot_t;
/** @brief 八项累计软件事实，允许uint32自然回绕，不称为物理空口丢帧率。 */
typedef struct {
  uint32_t input_events, enqueued_frames, completed_frames, full_drops;
  uint32_t source_rejects, aborted_frames, completed_parts, error_attempts;
} RawCounters_t;
/** @brief 单生产者/单消费者状态；共享计数、队列元数据均经短临界区访问。 */
typedef struct {
  RawSlot_t slots[PX4LITE_GNSS_RAW_UPLINK_CAPACITY]; /**< 含WRITING与活跃帧的FIFO。 */
  RawCounters_t counters; /**< 源事件、队满、完成和发送失败计数。 */
  uint32_t frame_id, last_input_ms, last_completed_ms; /**< 独立代数及毫秒时间。 */
  uint32_t active_ms, frame_retry_ms, next_stats_ms, stats_retry_ms, stats_seq; /**< Comm期限与统计代数。 */
  uint16_t offset, wire_crc16; /**< 活跃wire偏移及整个wire的CCITT-FALSE。 */
  uint8_t head, depth, input_seen, completed_seen; /**< 队列和合法0ms就绪位。 */
  uint8_t active, frame_attempts, stats_attempts, stats_pending, offered, prefer_data; /**< Comm发送状态及统计后让数据一次。 */
  uint8_t payload[PX4LITE_GNSS_RAW_UPLINK_PAYLOAD_MAX]; /**< 当前一个RAW_FRAME分片。 */
  uint8_t stats_payload[PX4LITE_GNSS_RAW_UPLINK_PAYLOAD_MAX]; /**< 有界重试使用的同一统计副本。 */
} RawState_t;
static RawState_t s_raw;

/** @brief 固定小端写入，不能依赖C结构体布局。 */
static void RawPut16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8U); }
/** @brief 固定小端写入累计计数和真实毫秒。 */
static void RawPut32(uint8_t *p,uint32_t v) { p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8U);p[2]=(uint8_t)(v>>16U);p[3]=(uint8_t)(v>>24U); }
/** @brief 回绕安全期限判断，所有调度期限远小于2^31ms。 */
static uint8_t RawDue(uint32_t now,uint32_t deadline) { return (uint8_t)((int32_t)(now-deadline)>=0); }
/** @brief 0ms由seen授权；未来时间或未知来源为FFFFFFFF。 */
static uint32_t RawAge(uint32_t now,uint32_t source,uint8_t seen)
{ int32_t d=(int32_t)(now-source);return (!seen || d<0)?UINT32_MAX:(uint32_t)d; }
/** @brief 允许复制时被Sensor抢占导致的有界真实时钟向前推进。 */
static uint32_t RawNow(uint32_t caller)
{ uint32_t actual=Px4Lite_PlatformGetMs();int32_t d=(int32_t)(actual-caller);return (d>=0 && (uint32_t)d<=PX4LITE_GNSS_RAW_UPLINK_ACTIVE_MS)?actual:caller; }
/** @brief CRC-16/CCITT-FALSE(poly1021,initFFFF,无反射/xor)，在pin稳定后临界区外执行。 */
static uint16_t RawCrc16(const uint8_t *bytes,uint16_t length)
{
  uint16_t crc=0xFFFFU,i;uint8_t bit;
  for(i=0U;i<length;i++){crc^=(uint16_t)((uint16_t)bytes[i]<<8U);for(bit=0U;bit<8U;bit++){crc=(uint16_t)((crc&0x8000U)?(uint16_t)((crc<<1U)^0x1021U):(uint16_t)(crc<<1U));}}
  return crc;
}
/** @brief 只核对完整边界/元数据，不解析导航或重算原协议校验；短坏NMEA也保留。 */
static uint8_t RawComplete(const uint8_t *bytes,uint16_t length,uint8_t protocol,uint8_t type,uint8_t cls,uint8_t id,uint8_t valid)
{
  if(bytes==0 || length>PX4LITE_GNSS_RAW_UPLINK_FRAME_MAX || valid>1U) { return 0U; }
  if(protocol==1U) { return (uint8_t)(length>=2U && type<=4U && cls==0U && id==0U && bytes[0]=='$' && bytes[length-1U]=='\n'); }
  if(protocol==2U) { return (uint8_t)(length>=8U && type==0U && bytes[0]==0xB5U && bytes[1]==0x62U && bytes[2]==cls && bytes[3]==id && (uint32_t)length==8UL+(uint32_t)bytes[4]+((uint32_t)bytes[5]<<8U)); }
  return 0U;
}
/** @brief 在同一短临界区记录输入事实与输入代数；0ms可见，不依赖捕获是否运行。 */
static void RawInput(uint32_t now)
{ s_raw.counters.input_events++;s_raw.frame_id++;s_raw.last_input_ms=now;s_raw.input_seen=1U; }
/** @brief Comm释放当前pin；只改队列元数据，不触及Sensor正在写的其他槽。 */
static void RawRelease(uint8_t completed,uint32_t now)
{
  taskENTER_CRITICAL();
  if(completed){s_raw.counters.completed_frames++;s_raw.last_completed_ms=now;s_raw.completed_seen=1U;}
  else{s_raw.counters.aborted_frames++;}
  s_raw.slots[s_raw.head].state=RAW_SLOT_FREE;s_raw.head=(uint8_t)((s_raw.head+1U)%PX4LITE_GNSS_RAW_UPLINK_CAPACITY);s_raw.depth--;
  s_raw.active=0U;s_raw.offset=0U;s_raw.frame_attempts=0U;
  taskEXIT_CRITICAL();
}
#if PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
/** @brief 显式兼容配置下冻结8021统计；默认不生成，内部计数仍持续记录。 */
static void RawBuildStats(uint32_t now)
{
  RawCounters_t c;uint32_t input_ms,completed_ms,oldest_ms;uint8_t depth,active_protocol,flags,input_seen,completed_seen;
  taskENTER_CRITICAL();
  c=s_raw.counters;depth=s_raw.depth;input_ms=s_raw.last_input_ms;completed_ms=s_raw.last_completed_ms;input_seen=s_raw.input_seen;completed_seen=s_raw.completed_seen;
  oldest_ms=depth?s_raw.slots[s_raw.head].received_ms:0U;active_protocol=s_raw.active?s_raw.slots[s_raw.head].protocol:0U;flags=(uint8_t)(1U|(s_raw.active?2U:0U));
  taskEXIT_CRITICAL();
  memset(s_raw.stats_payload,0,sizeof(s_raw.stats_payload));s_raw.stats_payload[0]=1U;s_raw.stats_payload[1]=0x21U;s_raw.stats_payload[3]=1U;
  s_raw.stats_seq++;RawPut32(&s_raw.stats_payload[4],s_raw.stats_seq);RawPut32(&s_raw.stats_payload[8],now);
  RawPut32(&s_raw.stats_payload[12],c.input_events);RawPut32(&s_raw.stats_payload[16],c.enqueued_frames);RawPut32(&s_raw.stats_payload[20],c.completed_frames);RawPut32(&s_raw.stats_payload[24],c.full_drops);
  RawPut32(&s_raw.stats_payload[28],c.source_rejects);RawPut32(&s_raw.stats_payload[32],c.aborted_frames);RawPut32(&s_raw.stats_payload[36],c.completed_parts);RawPut32(&s_raw.stats_payload[40],c.error_attempts);
  s_raw.stats_payload[44]=depth;s_raw.stats_payload[45]=PX4LITE_GNSS_RAW_UPLINK_CAPACITY;s_raw.stats_payload[46]=active_protocol;s_raw.stats_payload[47]=flags;
  RawPut32(&s_raw.stats_payload[48],RawAge(now,input_ms,input_seen));RawPut32(&s_raw.stats_payload[52],RawAge(now,completed_ms,completed_seen));RawPut32(&s_raw.stats_payload[56],RawAge(now,oldest_ms,(uint8_t)(depth!=0U)));
  RawPut16(&s_raw.stats_payload[60],PX4LITE_GNSS_RAW_UPLINK_FRAME_MAX);s_raw.stats_payload[62]=PX4LITE_GNSS_RAW_UPLINK_DATA_MAX;
  s_raw.stats_pending=1U;s_raw.stats_attempts=0U;s_raw.stats_retry_ms=now;s_raw.next_stats_ms=now+PX4LITE_GNSS_RAW_UPLINK_STATS_MS;
}
#endif
/** @brief pin队首后计算CRC；Sensor只能预留其他空槽，不能覆盖此槽。 */
static uint8_t RawPin(uint32_t now)
{
  const RawSlot_t *slot;
  if(s_raw.active){return 1U;}
  taskENTER_CRITICAL();
  if(!s_raw.depth || s_raw.slots[s_raw.head].state!=RAW_SLOT_READY){taskEXIT_CRITICAL();return 0U;}
  s_raw.slots[s_raw.head].state=RAW_SLOT_ACTIVE;s_raw.active=1U;s_raw.active_ms=now;s_raw.frame_retry_ms=now;s_raw.offset=0U;s_raw.frame_attempts=0U;
  slot=&s_raw.slots[s_raw.head];taskEXIT_CRITICAL();
  s_raw.wire_crc16=RawCrc16(slot->bytes,slot->length);return 1U;
}
/** @brief 一个完整wire固定最多26片，最后一片只声明实际存在的数据长度。 */
static uint8_t RawEncodeFrame(void)
{
  const RawSlot_t *slot=&s_raw.slots[s_raw.head];uint16_t remaining=(uint16_t)(slot->length-s_raw.offset);
  uint8_t length=(uint8_t)((remaining>PX4LITE_GNSS_RAW_UPLINK_DATA_MAX)?PX4LITE_GNSS_RAW_UPLINK_DATA_MAX:remaining);
  memset(s_raw.payload,0,sizeof(s_raw.payload));s_raw.payload[0]=1U;s_raw.payload[1]=slot->protocol;s_raw.payload[2]=(uint8_t)(2U|slot->checksum_valid);s_raw.payload[3]=24U;
  RawPut32(&s_raw.payload[4],slot->frame_id);RawPut32(&s_raw.payload[8],slot->received_ms);RawPut16(&s_raw.payload[12],slot->length);RawPut16(&s_raw.payload[14],s_raw.offset);
  s_raw.payload[16]=(uint8_t)(s_raw.offset/PX4LITE_GNSS_RAW_UPLINK_DATA_MAX);s_raw.payload[17]=(uint8_t)((slot->length+PX4LITE_GNSS_RAW_UPLINK_DATA_MAX-1U)/PX4LITE_GNSS_RAW_UPLINK_DATA_MAX);s_raw.payload[18]=length;
  s_raw.payload[19]=slot->nmea_type;s_raw.payload[20]=slot->class_id;s_raw.payload[21]=slot->message_id;RawPut16(&s_raw.payload[22],s_raw.wire_crc16);
  memcpy(&s_raw.payload[24],&slot->bytes[s_raw.offset],length);return (uint8_t)(24U+length);
}
#endif

/** @brief 仅启动前或Comm独占停机时复位；同一临界区覆盖整个队列生命周期状态。 */
void Px4Lite_GnssRawUplinkInit(uint32_t now_ms)
{
#if PX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK
  taskENTER_CRITICAL();memset(&s_raw,0,sizeof(s_raw));s_raw.next_stats_ms=now_ms+PX4LITE_GNSS_RAW_UPLINK_STATS_MS;taskEXIT_CRITICAL();
#else
  (void)now_ms;
#endif
}
/** @brief 唯一Sensor生产者预留空槽、临界区外复制，随后READY发布；满时丢新，不碰活跃帧。 */
void Px4Lite_GnssRawEnqueue(const uint8_t *bytes,uint16_t length,uint8_t protocol,uint8_t nmea_type,uint8_t cls,uint8_t id,uint8_t valid,uint32_t now_ms)
{
#if PX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK
  uint8_t complete=RawComplete(bytes,length,protocol,nmea_type,cls,id,valid),index;RawSlot_t *slot;
  taskENTER_CRITICAL();RawInput(now_ms);
  if(!complete){s_raw.counters.source_rejects++;taskEXIT_CRITICAL();return;}
  if(s_raw.depth>=PX4LITE_GNSS_RAW_UPLINK_CAPACITY){s_raw.counters.full_drops++;taskEXIT_CRITICAL();return;}
  index=(uint8_t)((s_raw.head+s_raw.depth)%PX4LITE_GNSS_RAW_UPLINK_CAPACITY);s_raw.depth++;slot=&s_raw.slots[index];
  slot->state=RAW_SLOT_WRITING;slot->frame_id=s_raw.frame_id;slot->received_ms=now_ms;slot->length=length;slot->protocol=protocol;slot->nmea_type=nmea_type;slot->class_id=cls;slot->message_id=id;slot->checksum_valid=valid;
  taskEXIT_CRITICAL();memcpy(slot->bytes,bytes,length);
  taskENTER_CRITICAL();slot->state=RAW_SLOT_READY;s_raw.counters.enqueued_frames++;taskEXIT_CRITICAL();
#else
  (void)bytes;(void)length;(void)protocol;(void)nmea_type;(void)cls;(void)id;(void)valid;(void)now_ms;
#endif
}
/** @brief 源拒绝只记录观察事件，不推测尚未组完的帧数或丢失的物理字节数。 */
void Px4Lite_GnssRawSourceReject(uint8_t protocol,uint32_t length,uint8_t reason,uint32_t now_ms)
{
  (void)protocol;(void)length;(void)reason;
#if PX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK
  taskENTER_CRITICAL();RawInput(now_ms);s_raw.counters.source_rejects++;taskEXIT_CRITICAL();
#else
  (void)now_ms;
#endif
}
/** @brief 维护调用只释放超期active；高优先响应占用UART时也能回收，不启动新帧。 */
void Px4Lite_GnssRawUplinkPollTimeout(uint32_t now_ms)
{
#if PX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK
  now_ms=RawNow(now_ms);
  if(s_raw.active && RawAge(now_ms,s_raw.active_ms,1U)>PX4LITE_GNSS_RAW_UPLINK_ACTIVE_MS){RawRelease(0U,now_ms);if(s_raw.offered==RAW_OFFER_FRAME){s_raw.offered=0U;}}
#else
  (void)now_ms;
#endif
}
/** @brief Comm默认只借用一个RAW片；显式开启统计时共享机会且不会饿死数据。 */
Px4Lite_Result_t Px4Lite_GnssRawUplinkPeek(uint32_t now_ms,Px4Lite_GnssRawUplinkView_t *out)
{
  if(out==0){return PX4LITE_INVALID_PARAM;}
#if PX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK
  now_ms=RawNow(now_ms);
  Px4Lite_GnssRawUplinkPollTimeout(now_ms);
#if PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
  if(!s_raw.stats_pending && RawDue(now_ms,s_raw.next_stats_ms)){RawBuildStats(now_ms);}
  if(s_raw.stats_pending && RawDue(now_ms,s_raw.stats_retry_ms)){
    uint8_t data_ready=0U;
    if(s_raw.prefer_data){
      if(s_raw.active){data_ready=RawDue(now_ms,s_raw.frame_retry_ms);}
      else{taskENTER_CRITICAL();data_ready=(uint8_t)(s_raw.depth && s_raw.slots[s_raw.head].state==RAW_SLOT_READY);taskEXIT_CRITICAL();}
    }
    if(!data_ready){out->payload=s_raw.stats_payload;out->payload_type=PX4LITE_GNSS_RAW_UPLINK_STATS_TYPE;out->length=64U;s_raw.offered=RAW_OFFER_STATS;return PX4LITE_OK;}
  }
#endif
  if(!RawPin(now_ms) || !RawDue(now_ms,s_raw.frame_retry_ms)){return PX4LITE_IDLE;}
  out->payload=s_raw.payload;out->payload_type=PX4LITE_GNSS_RAW_UPLINK_FRAME_TYPE;out->length=RawEncodeFrame();s_raw.offered=RAW_OFFER_FRAME;return PX4LITE_OK;
#else
  (void)now_ms;return PX4LITE_IDLE;
#endif
}
/** @brief UART成功仅代表本机完整发出；片重试耗尽直接中止整帧，不发送缺片伪完整记录。 */
void Px4Lite_GnssRawUplinkResult(Px4Lite_Result_t result,uint32_t now_ms)
{
#if PX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK
  uint8_t offered=s_raw.offered;s_raw.offered=0U;if(!offered){return;}
  if(result!=PX4LITE_OK){taskENTER_CRITICAL();s_raw.counters.error_attempts++;taskEXIT_CRITICAL();}
  if(offered==RAW_OFFER_STATS){s_raw.prefer_data=1U;s_raw.stats_attempts++;if(result==PX4LITE_OK || s_raw.stats_attempts>=PX4LITE_GNSS_RAW_UPLINK_ATTEMPTS){s_raw.stats_pending=0U;}else{s_raw.stats_retry_ms=now_ms+PX4LITE_GNSS_RAW_UPLINK_RETRY_MS;}return;}
  if(!s_raw.active){return;}
  s_raw.prefer_data=0U;
  s_raw.frame_attempts++;
  if(result==PX4LITE_OK){uint16_t length=s_raw.slots[s_raw.head].length;taskENTER_CRITICAL();s_raw.counters.completed_parts++;taskEXIT_CRITICAL();s_raw.offset=(uint16_t)(s_raw.offset+s_raw.payload[18]);s_raw.frame_attempts=0U;s_raw.frame_retry_ms=now_ms;if(s_raw.offset>=length){RawRelease(1U,now_ms);}}
  else if(s_raw.frame_attempts>=PX4LITE_GNSS_RAW_UPLINK_ATTEMPTS){RawRelease(0U,now_ms);}
  else{s_raw.frame_retry_ms=now_ms+PX4LITE_GNSS_RAW_UPLINK_RETRY_MS;}
#else
  (void)result;(void)now_ms;
#endif
}
