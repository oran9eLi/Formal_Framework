/**
 * @file px4lite_gnss_uplink.c
 * @brief Framework GNSS规范化快照与本地TUNNEL载荷编码。
 * @details Comm唯一所有者；一秒一组冻结数据，六包分轮发送。无BSP/Driver依赖、
 *          无动态分配、无导航解析；字段有效性不由链路活动或配置ACK续期。
 */
#include "px4lite_gnss_uplink.h"
#include "px4lite_config.h"
#include "px4lite_platform.h"
#include "px4lite_time.h"
#include "px4lite_topics.h"
#include <limits.h>
#include <string.h>

#if PX4LITE_ENABLE_RPI_GNSS_UPLINK
/** @brief Comm静态冻结区；Driver/传输/系统时钟分别复制，并非硬件同时采样。 */
typedef struct {
  Px4Lite_SensorGnss_t gnss; /**< 同组NAV/DOP/COUNT只用这一个副本。 */
  Px4Lite_GnssDiagnostics_t diag; /**< 软件与BSP独立可用标志。 */
  Px4Lite_TimeSnapshot_t time; /**< 系统UTC，绝不填入NAV真实GNSS时间。 */
  uint32_t bundle_id, snapshot_ms, encode_ms, next_bundle_ms, retry_ms; /**< 代数、冻结/编码时刻及真实毫秒期限。 */
  uint8_t payload[PX4LITE_GNSS_UPLINK_PAYLOAD_MAX]; /**< 当前一个编码包。 */
  uint8_t gnss_available, diag_available, time_available; /**< 各次复制与时效授权。 */
  uint8_t part, attempts, offered; /**< 待发包0..5、已失败次数、Peek配对标志。 */
} GnssUplinkState_t;
static GnssUplinkState_t s_uplink;

/** @brief 固定小端写入，不发送结构体布局。 */
static void UplinkPut16(uint8_t *out, uint16_t value)
{ out[0]=(uint8_t)value; out[1]=(uint8_t)(value>>8U); }
/** @brief 固定小端写入32位整数，包括有符号物理量的二补码位模式。 */
static void UplinkPut32(uint8_t *out, uint32_t value)
{ out[0]=(uint8_t)value; out[1]=(uint8_t)(value>>8U); out[2]=(uint8_t)(value>>16U); out[3]=(uint8_t)(value>>24U); }
/** @brief 回绕安全截止判断；所有期限均小于2^31ms。 */
static uint8_t UplinkDue(uint32_t now, uint32_t deadline)
{ return (uint8_t)((int32_t)(now-deadline)>=0); }
/** @brief 复制或编码期间可能被Sensor抢占；仅接受真实时钟小窗口向前推进。 */
static uint32_t UplinkNow(uint32_t caller)
{
  uint32_t actual=Px4Lite_PlatformGetMs();int32_t advance=(int32_t)(actual-caller);
  return (advance>=0 && (uint32_t)advance<=PX4LITE_GNSS_MAX_AGE_MS)?actual:caller;
}
/** @brief seen独立授权0ms；未来源时间不能被当作0龄期。 */
static uint32_t UplinkAge(uint32_t now, uint32_t source, uint8_t seen)
{ int32_t age=(int32_t)(now-source); return (!seen || age<0) ? UINT32_MAX : (uint32_t)age; }
/** @brief 16位年龄：FFFF未知，FFFE为>=65534ms，0合法。 */
static uint16_t UplinkAge16(uint32_t age)
{ return age==UINT32_MAX ? UINT16_MAX : (age>=65534U ? 65534U : (uint16_t)age); }
/** @brief 合法来源0为未知；不能把最后识别协议充当测量来源。 */
static uint8_t UplinkSource(uint8_t source)
{ return source<=PX4LITE_GNSS_SOURCE_UBX_DOP ? source : PX4LITE_GNSS_SOURCE_NONE; }
/** @brief 仅接受Driver已经支持的真实定位质量，不从卫星数猜定位。 */
static uint8_t UplinkHasFix(void)
{ uint8_t fix=s_uplink.gnss.fix_type; return (uint8_t)(fix==1U || fix==2U || fix==4U || fix==5U); }
/** @brief 日期校验；GNSS两位年遵循现有统一时间服务的2000..2099合同。 */
static uint8_t UplinkDateValid(uint32_t ymd)
{
  static const uint8_t days[12]={31U,28U,31U,30U,31U,30U,31U,31U,30U,31U,30U,31U};
  uint32_t year=ymd/10000U, month=(ymd/100U)%100U, day=ymd%100U;
  uint8_t max_day;
  if(year<2000U || year>2099U || month<1U || month>12U) { return 0U; }
  max_day=days[month-1U]; if(month==2U && year%4U==0U) { max_day=29U; }
  return (uint8_t)(day>=1U && day<=max_day);
}
/** @brief 五域元数据与保留值分离；present为源位且具有明确来源。 */
static uint8_t UplinkDomain(uint8_t id, uint8_t *source, uint32_t *age)
{
  const Px4Lite_SensorGnss_t *g=&s_uplink.gnss;
  uint32_t stamp=0U; uint8_t src=0U;
  switch(id) {
    case 0U: stamp=g->position_update_ms;src=g->position_source;break;
    case 1U: stamp=g->velocity_update_ms;src=g->velocity_source;break;
    case 2U: stamp=g->time_update_ms;src=g->time_source;break;
    case 3U: stamp=g->altitude_update_ms;src=g->altitude_source;break;
    default: stamp=g->status_update_ms;src=g->status_source;break;
  }
  *source=UplinkSource(src);
  *age=UplinkAge(s_uplink.encode_ms,stamp,(uint8_t)(s_uplink.gnss_available && *source!=0U));
  return (uint8_t)(s_uplink.gnss_available && *source!=0U && (g->valid_mask&(1UL<<id))!=0U);
}
/** @brief 基础位置解门禁仅看真实位置域和定位质量，ACK不参与。 */
static uint8_t UplinkNavigationValid(void)
{
  uint8_t source; uint32_t age;
  return (uint8_t)(UplinkDomain(0U,&source,&age) && age<=PX4LITE_GNSS_MAX_AGE_MS && UplinkHasFix() &&
    s_uplink.gnss.latitude_e7>=-900000000L && s_uplink.gnss.latitude_e7<=900000000L &&
    s_uplink.gnss.longitude_e7>=-1800000000L && s_uplink.gnss.longitude_e7<=1800000000L);
}
/** @brief DOP reported与可用解质量分开；有效需本句solution授权及新鲜位置，不能借别句fix。 */
static uint8_t UplinkDopFlags(uint8_t id, uint32_t *age, uint8_t *source)
{
  const Px4Lite_SensorGnss_t *g=&s_uplink.gnss;
  uint8_t flags=0U;
  *source=UplinkSource(g->dop_source[id]);
  *age=UplinkAge(s_uplink.encode_ms,g->dop_update_ms[id],(uint8_t)(s_uplink.gnss_available && *source!=0U && (g->dop_present_mask&(1U<<id))!=0U));
  if(!s_uplink.gnss_available || *source==0U || !(g->dop_present_mask&(1U<<id)) || g->dop_x100[id]==0U || g->dop_x100[id]==UINT16_MAX) { *age=UINT32_MAX;return 0U; }
  flags=1U;
  if(*age>PX4LITE_GNSS_MAX_AGE_MS) { flags|=4U; }
  else if((g->dop_solution_mask&(1U<<id))!=0U && UplinkNavigationValid()) { flags|=2U; }
  if((*source==PX4LITE_GNSS_SOURCE_UBX_PVT || *source==PX4LITE_GNSS_SOURCE_UBX_DOP) && (g->dop_itow_known_mask&(1U<<id))!=0U && g->dop_itow_ms[id]<604800000UL) { flags|=8U; }
  return flags;
}
/** @brief 为每个内层包写相同冻结代数和时刻；不随重试刷新。 */
static void UplinkHeader(uint8_t kind, uint8_t part, uint8_t count)
{
  memset(s_uplink.payload,0,sizeof(s_uplink.payload));
  s_uplink.payload[0]=1U;s_uplink.payload[1]=kind;s_uplink.payload[2]=part;s_uplink.payload[3]=count;
  UplinkPut32(&s_uplink.payload[4],s_uplink.bundle_id);UplinkPut32(&s_uplink.payload[8],s_uplink.snapshot_ms);
}
/** @brief NAV64：无效值置零，present/stale/age与真实来源单独报告。 */
static void UplinkNav(void)
{
  const Px4Lite_SensorGnss_t *g=&s_uplink.gnss;
  uint8_t i,src,present,valid=0U,stale=0U,presents=0U,dop_flags; uint16_t sources=0U; uint32_t age,ymd;
  UplinkHeader(1U,0U,1U);
  for(i=0U;i<5U;i++) {
    present=UplinkDomain(i,&src,&age); sources|=(uint16_t)((uint16_t)src<<(3U*i));
    UplinkPut16(&s_uplink.payload[48U+2U*i],UplinkAge16(age));
    if(!present) { continue; } presents|=(uint8_t)(1U<<i);
    if(age>PX4LITE_GNSS_MAX_AGE_MS) { stale|=(uint8_t)(1U<<i);continue; }
    valid|=(uint8_t)(1U<<i);
  }
  if(!UplinkNavigationValid()) { valid&=(uint8_t)~1U; }
  if(g->heading_deg100>=36000U) { valid&=(uint8_t)~2U; }
  ymd=20000000UL+g->utc_date;
  if(g->utc_date>991231UL || !UplinkDateValid(ymd) || g->utc_sec>=86400UL) { valid&=(uint8_t)~4U; }
  if((valid&1U)!=0U) { UplinkPut32(&s_uplink.payload[20],(uint32_t)g->latitude_e7);UplinkPut32(&s_uplink.payload[24],(uint32_t)g->longitude_e7); }
  if((valid&8U)!=0U) { UplinkPut32(&s_uplink.payload[28],(uint32_t)g->altitude_mm); }
  if((valid&2U)!=0U) { UplinkPut32(&s_uplink.payload[32],g->ground_speed_cms);UplinkPut16(&s_uplink.payload[36],g->heading_deg100); }
  if((valid&4U)!=0U) { UplinkPut32(&s_uplink.payload[40],ymd);UplinkPut32(&s_uplink.payload[44],g->utc_sec); }
  if((valid&16U)!=0U) { s_uplink.payload[16]=g->fix_type;s_uplink.payload[18]=g->satellites_used;s_uplink.payload[63]=g->raw_status_type; }
  if(s_uplink.gnss_available && UplinkSource(g->dimension_source)!=0U && UplinkAge(s_uplink.encode_ms,g->dimension_update_ms,1U)<=PX4LITE_GNSS_MAX_AGE_MS && (g->fix_dimension==2U || g->fix_dimension==3U)) { s_uplink.payload[17]=g->fix_dimension; }
  dop_flags=UplinkDopFlags(PX4LITE_GNSS_DOP_H,&age,&src);
  if(dop_flags&1U) { presents|=32U;s_uplink.payload[19]=src; }
  if(dop_flags&2U) { valid|=32U;UplinkPut16(&s_uplink.payload[38],g->dop_x100[PX4LITE_GNSS_DOP_H]); }
  if(dop_flags&4U) { stale|=32U; }
  UplinkPut16(&s_uplink.payload[58],UplinkAge16(age));
  UplinkPut16(&s_uplink.payload[60],UplinkAge16(UplinkAge(s_uplink.encode_ms,s_uplink.diag.last_frame_ms,s_uplink.diag.frame_seen)));
  s_uplink.payload[12]=valid;s_uplink.payload[13]=stale;UplinkPut16(&s_uplink.payload[14],sources);s_uplink.payload[62]=presents;
}
/** @brief DOP64两包按固定id发送4+3项，缺项也发送以清除主机旧缓存。 */
static void UplinkDop(uint8_t part)
{
  uint8_t row,id,offset,source,flags,count=(part==0U)?4U:3U;uint32_t age;
  UplinkHeader(2U,part,2U);s_uplink.payload[12]=count;
  for(row=0U;row<count;row++) {
    id=(uint8_t)(part*4U+row);offset=(uint8_t)(16U+row*12U);flags=UplinkDopFlags(id,&age,&source);
    s_uplink.payload[offset]=id;s_uplink.payload[offset+1U]=(flags&1U)?source:0U;s_uplink.payload[offset+2U]=flags;
    if(flags&1U) { UplinkPut16(&s_uplink.payload[offset+4U],s_uplink.gnss.dop_x100[id]); }
    UplinkPut16(&s_uplink.payload[offset+6U],UplinkAge16(age));
    if(flags&8U) { UplinkPut32(&s_uplink.payload[offset+8U],s_uplink.gnss.dop_itow_ms[id]); }
  }
}
/** @brief 分类计数32：只接受各自present/source/age，不从PVT总数伪造分类。 */
static void UplinkCounts(void)
{
  const Px4Lite_SensorGnss_t *g=&s_uplink.gnss;uint8_t i,src,value;uint32_t age;
  UplinkHeader(3U,0U,1U);
  for(i=0U;i<4U;i++) {
    src=UplinkSource(g->counts_source[i]);
    age=UplinkAge(s_uplink.encode_ms,g->counts_update_ms[i],(uint8_t)(s_uplink.gnss_available && src!=0U && (g->counts_present_mask&(1U<<i))!=0U));
    UplinkPut16(&s_uplink.payload[24U+2U*i],UplinkAge16(age));
    if(!s_uplink.gnss_available || !src || !(g->counts_present_mask&(1U<<i))) { continue; }
    s_uplink.payload[12]|=(uint8_t)(1U<<i);s_uplink.payload[20U+i]=src;
    if(age>PX4LITE_GNSS_MAX_AGE_MS) { s_uplink.payload[14]|=(uint8_t)(1U<<i);continue; }
    s_uplink.payload[13]|=(uint8_t)(1U<<i);
    value=(i==0U)?g->gps_visible:(i==1U)?g->bds_visible:(i==2U)?g->gps_used:g->bds_used;
    s_uplink.payload[16U+i]=value;
  }
}
/** @brief LINK64两包；BSP可用不代表Driver心跳新鲜，系统RTC不冒充GNSS授时。 */
static void UplinkLink(uint8_t part)
{
  const Px4Lite_GnssDiagnostics_t *d=&s_uplink.diag;const Px4Lite_TimeSnapshot_t *t=&s_uplink.time;
  uint8_t fresh=(uint8_t)(s_uplink.diag_available && UplinkAge(s_uplink.encode_ms,d->measured_ms,1U)<=PX4LITE_GNSS_MAX_AGE_MS),transport=d->transport_available;
  UplinkHeader(4U,part,2U);
  if(part==0U) {
    s_uplink.payload[12]=(uint8_t)((fresh?1U:0U)|(transport?2U:0U)|(d->rx_seen?4U:0U)|(d->frame_seen?8U:0U)|(d->version_seen?16U:0U)|(transport&&d->initialized?32U:0U)|(transport&&d->tx_busy?64U:0U));
    s_uplink.payload[13]=fresh?d->protocol:0U;s_uplink.payload[14]=fresh?d->state:255U;
    s_uplink.payload[15]=fresh?d->config_step:0U;s_uplink.payload[16]=fresh?d->config_total:0U;s_uplink.payload[17]=(uint8_t)(fresh && d->config_confirmed);
    UplinkPut16(&s_uplink.payload[18],fresh?d->protocol_version_x100:0U);
    UplinkPut32(&s_uplink.payload[20],transport?d->transport_baud_bps:(fresh?d->baud_bps:0U));
    UplinkPut32(&s_uplink.payload[24],d->rx_bytes);UplinkPut32(&s_uplink.payload[28],d->valid_nmea);UplinkPut32(&s_uplink.payload[32],d->valid_ubx);
    UplinkPut32(&s_uplink.payload[36],d->checksum_errors);UplinkPut32(&s_uplink.payload[40],d->framing_resets);
    UplinkPut32(&s_uplink.payload[44],transport?d->rx_drop_bytes:0U);UplinkPut32(&s_uplink.payload[48],transport?d->uart_errors:0U);UplinkPut32(&s_uplink.payload[52],transport?d->dma_errors:0U);
    UplinkPut32(&s_uplink.payload[56],UplinkAge(s_uplink.encode_ms,d->last_rx_ms,d->rx_seen));UplinkPut32(&s_uplink.payload[60],UplinkAge(s_uplink.encode_ms,d->last_frame_ms,d->frame_seen));
  } else {
    if(s_uplink.time_available && UplinkAge(s_uplink.encode_ms,t->header.sample_time_ms,1U)<=PX4LITE_GNSS_UPLINK_TIME_MAX_AGE_MS) { uint32_t hh=t->utc_time_hhmmss/10000U,mm=(t->utc_time_hhmmss/100U)%100U,ss=t->utc_time_hhmmss%100U;
      uint32_t age_add=(s_uplink.encode_ms-s_uplink.snapshot_ms)/1000U;
      s_uplink.payload[12]=1U;s_uplink.payload[13]=(uint8_t)t->source;s_uplink.payload[14]=(uint8_t)t->sync_state;
      UplinkPut32(&s_uplink.payload[16],(t->sync_age_s>UINT32_MAX-age_add)?UINT32_MAX:t->sync_age_s+age_add);UplinkPut32(&s_uplink.payload[56],t->utc_date_ymd);UplinkPut32(&s_uplink.payload[60],hh*3600U+mm*60U+ss); }
    UplinkPut32(&s_uplink.payload[20],d->tx_commands);UplinkPut32(&s_uplink.payload[24],d->ack_count);UplinkPut32(&s_uplink.payload[28],d->nak_count);UplinkPut32(&s_uplink.payload[32],d->timeout_count);
    UplinkPut32(&s_uplink.payload[36],transport?d->tx_packets:0U);UplinkPut32(&s_uplink.payload[40],transport?d->tx_failures:0U);UplinkPut32(&s_uplink.payload[44],transport?d->recoveries:0U);
    UplinkPut32(&s_uplink.payload[48],transport?d->transport_rx_bytes:0U);UplinkPut16(&s_uplink.payload[52],transport?d->ndtr:0U);UplinkPut16(&s_uplink.payload[54],transport?d->queued_bytes:0U);
  }
}
/** @brief 三类来源各复制一次；失败保留独立transport事实，未知仍产生六包。 */
static void UplinkFreeze(uint32_t now)
{
  Px4Lite_Result_t result,diag_result;uint32_t hh,mm,ss;
  memset(&s_uplink.gnss,0,sizeof(s_uplink.gnss));memset(&s_uplink.diag,0,sizeof(s_uplink.diag));memset(&s_uplink.time,0,sizeof(s_uplink.time));
  result=Px4Lite_CopyGnss(&s_uplink.gnss);s_uplink.gnss_available=(uint8_t)(result==PX4LITE_OK);
  diag_result=Px4Lite_GnssCopyDiagnostics(&s_uplink.diag);
  result=Px4Lite_CopyTime(&s_uplink.time);hh=s_uplink.time.utc_time_hhmmss/10000U;mm=(s_uplink.time.utc_time_hhmmss/100U)%100U;ss=s_uplink.time.utc_time_hhmmss%100U;
  now=UplinkNow(now);
  s_uplink.diag_available=(uint8_t)(diag_result==PX4LITE_OK && s_uplink.diag.available);
  s_uplink.time_available=(uint8_t)(result==PX4LITE_OK && s_uplink.time.header.valid && UplinkDateValid(s_uplink.time.utc_date_ymd) && hh<24U && mm<60U && ss<60U && (s_uplink.time.source==PX4LITE_TIME_SOURCE_RTC || s_uplink.time.source==PX4LITE_TIME_SOURCE_GNSS));
  s_uplink.snapshot_ms=now;s_uplink.next_bundle_ms=now+PX4LITE_GNSS_UPLINK_PERIOD_MS;s_uplink.retry_ms=now;
  s_uplink.bundle_id++;s_uplink.part=0U;s_uplink.attempts=0U;s_uplink.offered=0U;
}
#endif

/** @brief 仅在Comm调度初始化时清静态区，不触及外设或导航状态。 */
void Px4Lite_GnssUplinkInit(uint32_t now_ms)
{
#if PX4LITE_ENABLE_RPI_GNSS_UPLINK
  memset(&s_uplink,0,sizeof(s_uplink));s_uplink.next_bundle_ms=now_ms;s_uplink.part=PX4LITE_GNSS_UPLINK_PARTS;
#else
  (void)now_ms;
#endif
}
/** @brief 一次只提供当前一个冻结包；新秒周期主动丢弃未完成旧组。 */
Px4Lite_Result_t Px4Lite_GnssUplinkPeek(uint32_t now_ms, Px4Lite_GnssUplinkView_t *out)
{
  if(out==0) { return PX4LITE_INVALID_PARAM; }
#if PX4LITE_ENABLE_RPI_GNSS_UPLINK
  now_ms=UplinkNow(now_ms);
  if(UplinkDue(now_ms,s_uplink.next_bundle_ms)) { UplinkFreeze(now_ms);now_ms=s_uplink.snapshot_ms; }
  if(s_uplink.part>=PX4LITE_GNSS_UPLINK_PARTS || !UplinkDue(now_ms,s_uplink.retry_ms)) { return PX4LITE_IDLE; }
  s_uplink.encode_ms=UplinkNow(now_ms);
  out->length=64U;out->frame_index=s_uplink.part;
  if(s_uplink.part==0U) { UplinkNav();out->payload_type=PX4LITE_GNSS_UPLINK_NAV_TYPE; }
  else if(s_uplink.part<=2U) { UplinkDop((uint8_t)(s_uplink.part-1U));out->payload_type=PX4LITE_GNSS_UPLINK_DOP_TYPE; }
  else if(s_uplink.part==3U) { UplinkCounts();out->payload_type=PX4LITE_GNSS_UPLINK_COUNTS_TYPE;out->length=32U; }
  else { UplinkLink((uint8_t)(s_uplink.part-4U));out->payload_type=PX4LITE_GNSS_UPLINK_LINK_TYPE; }
  out->payload=s_uplink.payload;s_uplink.offered=1U;return PX4LITE_OK;
#else
  (void)now_ms;return PX4LITE_IDLE;
#endif
}
/** @brief 仅实际发送结果推进状态；失败最多两次重试，不阻塞Comm等待。 */
void Px4Lite_GnssUplinkResult(Px4Lite_Result_t result, uint32_t now_ms)
{
#if PX4LITE_ENABLE_RPI_GNSS_UPLINK
  if(!s_uplink.offered) { return; }s_uplink.offered=0U;
  s_uplink.attempts++;
  if(result==PX4LITE_OK || s_uplink.attempts>=PX4LITE_GNSS_UPLINK_ATTEMPTS) { s_uplink.part++;s_uplink.attempts=0U;s_uplink.retry_ms=now_ms; }
  else { s_uplink.retry_ms=now_ms+PX4LITE_GNSS_UPLINK_RETRY_MS; }
#else
  (void)result;(void)now_ms;
#endif
}
