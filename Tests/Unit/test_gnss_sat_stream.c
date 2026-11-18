/**
 * @file test_gnss_sat_stream.c
 * @brief 真实UART6调度下1Hz导航/64星原帧与BMI原始解析双流的共存回归。
 * @details 复用Framework事实桩，保留真实MAVLink编码、五路调度及GNSS/BMI上行实现。
 *          仅模拟115200串口每10ms可接收一个有界MAVLink帧，不代替实板时序验收。
 */
#define main GnssRawRegressionOriginalMain
#include "test_gnss_raw_uplink.c"
#undef main

#if !PX4LITE_RPI_GNSS_PARSED_TX_ENABLE || !PX4LITE_ENABLE_RPI_GNSS_UPLINK || !PX4LITE_ENABLE_RPI_GNSS_RAW_UPLINK || PX4LITE_RPI_GNSS_RAW_STATS_ENABLE
#error "Satellite stream regression requires the default parsed+RAW policy with RAW_STATS disabled"
#endif
#if !PX4LITE_ENABLE_RPI_BMI088_UPLINK
#error "Satellite coexistence regression requires the production BMI088 uplink enabled"
#endif

#define STREAM_EPOCHS 8U
#define STREAM_KINDS 3U
#define STREAM_FRAMES (STREAM_EPOCHS * STREAM_KINDS)
#define STREAM_MAX_WIRE 784U

/** @brief 测试端独立保留每个源帧和已接收字节，不依赖被测队列中的指针。 */
typedef struct {
  uint8_t source[STREAM_MAX_WIRE]; /**< 入队前的真实UBX线帧。 */
  uint8_t received[STREAM_MAX_WIRE]; /**< 通过真实MAVLink解析器重组的原线帧。 */
  uint16_t length; /**< 本源帧总长，100/26/784字节。 */
  uint16_t bytes; /**< 已重组连续字节数。 */
  uint8_t parts; /**< 已接收连续分片数。 */
  uint8_t complete; /**< 全部原字节与校验核对成功后为1。 */
} StreamFrame_t;
static StreamFrame_t stream_frames[STREAM_FRAMES];
static uint8_t stream_input[STREAM_MAX_WIRE];

/** @brief 按独立源序号保存接收端配对状态，不借用被测邮箱或编码缓冲。 */
typedef struct {
  uint32_t sample_ms; /**< 接收端已核验的源采样时刻，单位ms。 */
  uint8_t raw_seen;   /**< 已接收并核验真实0x8030为1。 */
  uint8_t parsed_seen;/**< 已接收同次0x8031为1。 */
} StreamBmiPair_t;
static StreamBmiPair_t stream_bmi_pairs[900];
static uint32_t stream_bmi_raw,stream_bmi_parsed,stream_bmi_status,stream_bmi_complete;

/** @brief 独立构造合法小端字段，不借用生产RAW编码器生成预期值。 */
static void StreamPut16(uint8_t *p, uint16_t value)
{ p[0]=(uint8_t)value; p[1]=(uint8_t)(value>>8U); }
/** @brief 写入导航历元或有符号工程值的二补码字节。 */
static void StreamPut32(uint8_t *p, uint32_t value)
{ p[0]=(uint8_t)value; p[1]=(uint8_t)(value>>8U); p[2]=(uint8_t)(value>>16U); p[3]=(uint8_t)(value>>24U); }

/** @brief 每10ms发布一组自洽六轴及寄存器事实，真实编码器负责5Hz抽样和冻结。 */
static void StreamPublishBmi(uint32_t sequence)
{
  Px4Lite_Bmi088Sample_t sample;
  uint32_t sensor_time=sequence*256U;
  memset(&sample,0,sizeof(sample));
  sample.sample_sequence=sequence;sample.sample_time_ms=clock_ms;
  sample.reinit_count=1U;sample.accel_mg[2]=1000;sample.temperature_cdeg=2300;
  sample.sample_period_us=10000U;sample.bus=1U;
  sample.accel_addr_7bit=0x18U;sample.gyro_addr_7bit=0x68U;
  sample.accel_chip_id=0x1EU;sample.gyro_chip_id=0x0FU;
  sample.accel_status=0x80U;sample.gyro_fifo_status=1U;sample.gyro_frame_count=1U;
  StreamPut16(sample.raw_accel+4U,2731U); /* 2731*12000/32768四舍五入为1000mg。 */
  sample.raw_accel[6]=(uint8_t)sensor_time;
  sample.raw_accel[7]=(uint8_t)(sensor_time>>8U);
  sample.raw_accel[8]=(uint8_t)(sensor_time>>16U);
  sample.axis_map[0]=1;sample.axis_map[1]=2;sample.axis_map[2]=3;
  Px4Lite_Bmi088UplinkPublish(&sample);
}

/** @brief 独立读取公开8030/31/32布局并核验同源配对，不调用被测编码函数。 */
static void StreamAcceptBmi(const mavlink_tunnel_t *t)
{
  const uint8_t *p=t->payload;
  uint32_t sequence=Read32(p+4U),sample_ms=Read32(p+8U),sensor_time;
  StreamBmiPair_t *pair;
  unsigned i;
  assert(t->payload_length==64U && p[0]==1U && p[3]==1U && Read32(p+12U)==clock_ms);
  if(t->payload_type==0x8032U) {
    assert(p[1]==2U && p[60]==1U && p[61]==1U && p[62]==PX4LITE_STATE_ONLINE);
    assert(Read32(p+16U)==1U && Read32(p+20U)==0U && Read32(p+24U)==0U);
    assert(p[48]==0U && p[49]==0U && p[50]==0U && p[51]==1U);
    assert(p[52]==0x1EU && p[53]==0x0FU && p[54]==0x18U && p[55]==0x68U);
    assert(Read32(p+28U)==clock_ms/10U+1U);
    if(clock_ms>1000U) { assert(Read32(p+32U)>0U); }
    stream_bmi_status++;
    return;
  }
  assert(sequence>=1U && sequence<=900U && sample_ms==(sequence-1U)*10U);
  assert((p[2]&3U)==3U && (p[2]&12U)==0U && clock_ms-sample_ms<=200U);
  assert(Read32(p+16U)==1U);
  pair=&stream_bmi_pairs[sequence-1U];
  if(t->payload_type==0x8030U) {
    assert(p[1]==0U && !pair->raw_seen && !pair->parsed_seen);
    assert(p[20]==0x18U && p[21]==0x68U && p[22]==0x1EU && p[23]==0x0FU);
    assert(p[24]==0x80U && p[25]==1U && p[26]==1U && p[27]==1U);
    assert(Read16(p+28U)==0U && Read16(p+30U)==0U && Read16(p+32U)==2731U);
    sensor_time=sequence*256U;
    assert(p[34]==(uint8_t)sensor_time && p[35]==(uint8_t)(sensor_time>>8U) && p[36]==(uint8_t)(sensor_time>>16U));
    assert(p[37]==0U && p[38]==0U);
    for(i=39U;i<64U;i++) { assert(p[i]==0U); }
    pair->sample_ms=sample_ms;pair->raw_seen=1U;stream_bmi_raw++;
  } else {
    assert(t->payload_type==0x8031U && p[1]==1U && pair->raw_seen && !pair->parsed_seen && pair->sample_ms==sample_ms);
    assert(Read32(p+20U)==0U && Read32(p+24U)==0U && Read32(p+28U)==1000U);
    assert(Read32(p+32U)==0U && Read32(p+36U)==0U && Read32(p+40U)==0U);
    assert(Read16(p+44U)==2300U && Read16(p+46U)==10000U);
    assert(Read16(p+48U)==12000U && Read16(p+50U)==1000U);
    assert(p[52]==1U && p[53]==2U && p[54]==3U && p[55]==1U);
    assert(Read32(p+56U)==0U && Read32(p+60U)==0U);
    pair->parsed_seen=1U;stream_bmi_parsed++;stream_bmi_complete++;
  }
}

/**
 * @brief 每一秒生成PVT、DOP及64星SAT，包含完整原协议校验和。
 * @param kind 0=PVT100B，1=DOP26B，2=SAT784B。
 * @param epoch 导航历元序号，0..7。
 */
static uint16_t StreamBuildUbx(uint8_t kind, uint8_t epoch)
{
  static const uint16_t payload_lengths[STREAM_KINDS]={92U,18U,776U};
  static const uint8_t ids[STREAM_KINDS]={7U,4U,0x35U};
  uint16_t i, payload_length=payload_lengths[kind], length=(uint16_t)(payload_length+8U);
  uint8_t a=0U,b=0U,*p=&stream_input[6];
  memset(stream_input,0,sizeof(stream_input));
  stream_input[0]=0xB5U; stream_input[1]=0x62U; stream_input[2]=1U; stream_input[3]=ids[kind];
  StreamPut16(&stream_input[4],payload_length); StreamPut32(p,(uint32_t)epoch*1000U);
  if(kind==0U) {
    StreamPut16(p+4U,2026U); p[6]=11U; p[7]=9U; p[8]=12U; p[9]=34U; p[10]=epoch; p[11]=7U;
    p[20]=3U; p[21]=1U; p[23]=16U;
    StreamPut32(p+24U,(uint32_t)source_gnss.longitude_e7); StreamPut32(p+28U,(uint32_t)source_gnss.latitude_e7);
    StreamPut32(p+36U,(uint32_t)source_gnss.altitude_mm); StreamPut32(p+60U,source_gnss.ground_speed_cms*10U);
    StreamPut16(p+76U,102U);
  } else if(kind==1U) {
    for(i=0U;i<7U;i++) { StreamPut16(p+4U+2U*i,(uint16_t)(101U+i)); }
  } else {
    p[4]=1U; p[5]=64U;
    for(i=0U;i<64U;i++) {
      uint8_t *sv=p+8U+12U*i;
      sv[0]=(i<32U)?0U:3U; sv[1]=(uint8_t)((i%32U)+1U); sv[2]=(uint8_t)(30U+(i%15U));
      sv[3]=(uint8_t)(i%76U); StreamPut16(sv+4U,(uint16_t)((i*11U)%360U));
      StreamPut32(sv+8U,(uint32_t)(4U+((i%4U==0U)?8U:0U)));
    }
  }
  for(i=2U;i<length-2U;i++) { a=(uint8_t)(a+stream_input[i]); b=(uint8_t)(b+a); }
  stream_input[length-2U]=a; stream_input[length-1U]=b;
  return length;
}

/** @brief Sensor每秒提供一次新事实与3个同历元完整帧；同拍入队比真实串口到达更突发。 */
static void StreamEpoch(uint8_t epoch)
{
  uint8_t kind, i;
  RefreshLocalFacts(clock_ms);
  source_gnss.fix_type=1U; source_gnss.fix_dimension=3U; source_gnss.raw_status_type=3U; source_gnss.satellites_used=16U;
  source_gnss.velocity_source=source_gnss.altitude_source=PX4LITE_GNSS_SOURCE_UBX_PVT;
  /* 原RAW有逐星列表，不将其擅自塞进目前仅授权NMEA来源的分类计数出口。 */
  source_gnss.counts_present_mask=0U;
  memset(source_gnss.counts_source,0,sizeof(source_gnss.counts_source));
  source_diag.config_step=source_diag.config_total=4U; source_diag.ack_count=4U;
  source_gnss.utc_date=261109U; source_gnss.utc_sec=45240U+epoch;
  source_time.source=PX4LITE_TIME_SOURCE_GNSS; source_time.sync_state=PX4LITE_TIME_SYNC_GNSS_SYNCED;
  source_time.utc_date_ymd=20261109U; source_time.utc_time_hhmmss=123400U+epoch; source_time.sync_age_s=0U;
  for(i=0U;i<7U;i++) { source_gnss.dop_itow_ms[i]=(uint32_t)epoch*1000U; }
  for(kind=0U;kind<STREAM_KINDS;kind++) {
    StreamFrame_t *expected=&stream_frames[(uint16_t)epoch*STREAM_KINDS+kind];
    expected->length=StreamBuildUbx(kind,epoch);
    memcpy(expected->source,stream_input,expected->length);
    Px4Lite_GnssRawEnqueue(stream_input,expected->length,2U,0U,1U,stream_input[3],1U,clock_ms);
    /* 证明生产入队接口copy语义，之后发送不能依赖这个复用输入区。 */
    memset(stream_input,0xEE,sizeof(stream_input));
  }
}

/** @brief 按公开0x8020布局重组并校验，不调用生产RAW分片函数辅助检查。 */
static uint8_t StreamAcceptRaw(const mavlink_tunnel_t *t, uint32_t *maximum_latency)
{
  uint32_t id=Read32(t->payload+4U), source_ms;
  uint16_t offset=Read16(t->payload+14U),length;
  uint8_t amount=t->payload[18], kind;
  StreamFrame_t *frame;
  assert(id>=1U && id<=STREAM_FRAMES); frame=&stream_frames[id-1U]; kind=(uint8_t)((id-1U)%STREAM_KINDS);
  assert(t->payload[0]==1U && t->payload[1]==2U && t->payload[2]==3U && t->payload[3]==24U);
  assert(t->payload[19]==0U && t->payload[20]==1U && t->payload[21]==(kind==0U?7U:kind==1U?4U:0x35U));
  source_ms=((id-1U)/STREAM_KINDS)*1000U; assert(Read32(t->payload+8U)==source_ms);
  length=Read16(t->payload+12U); assert(length==frame->length && length<=STREAM_MAX_WIRE);
  assert(!frame->complete && offset==frame->bytes && t->payload[16]==frame->parts);
  assert(t->payload[17]==(uint8_t)((length+39U)/40U) && amount>0U && amount<=40U);
  assert((uint32_t)offset+amount<=length && t->payload_length==24U+amount);
  assert(Read16(t->payload+22U)==RefCrc(frame->source,length));
  memcpy(frame->received+offset,t->payload+24U,amount); frame->bytes=(uint16_t)(frame->bytes+amount); frame->parts++;
  if(frame->bytes==length) {
    uint32_t latency=clock_ms-source_ms;
    assert(memcmp(frame->received,frame->source,length)==0);
    frame->complete=1U; if(latency>*maximum_latency) {*maximum_latency=latency;} return 1U;
  }
  return 0U;
}

/** @brief 9秒五路调度，100Hz BMI源与8轮64星流/typed/ACK/兼容遥测共同推进。 */
int main(void)
{
  uint32_t cycle,before,raw_complete=0U,raw_parts=0U,typed_groups=0U,typed_id=0U,typed_ms=0U,maximum_latency=0U;
  uint32_t acks=0U,priority_echoes=0U,legacy_gps=0U,legacy_mask=0U,heartbeats=0U,other_telemetry=0U;
  uint8_t typed_mask=0U,epoch,kind;
  Sources(0U); assert(Px4Lite_MavlinkTxInit(0U)==PX4LITE_OK); s_tx_enabled=0U;
  memset(stream_frames,0,sizeof(stream_frames));
  memset(stream_bmi_pairs,0,sizeof(stream_bmi_pairs));
  stream_bmi_raw=stream_bmi_parsed=stream_bmi_status=stream_bmi_complete=0U;
  for(cycle=0U;cycle<900U;cycle++) {
    mavlink_message_t rx; mavlink_tunnel_t t;
    clock_ms=cycle*10U;
    StreamPublishBmi(cycle+1U);
    if(cycle<800U && cycle%100U==0U) { StreamEpoch((uint8_t)(cycle/100U)); }
    if(cycle<800U && cycle%100U==35U) { Px4Lite_MavlinkQueueCommandAck(400U,0U,9U,1U,PX4LITE_MAVLINK_LINK_RPI); }
    if(cycle<800U && cycle%200U==50U) { h1_pending=1U; }
    before=uart_calls; (void)Px4Lite_MavlinkTxRunRpi(clock_ms); assert(uart_calls<=before+1U);
    if(uart_calls==before) {continue;}
    /* 最大81字节TUNNEL在115200/8N1下不足7.1ms，10ms后可再次提交。 */
    assert((uint32_t)uart_length*10000U<=115200U*10U);
    rx=DecodeWire();
    if(rx.msgid==77U) {acks++;}
    else if(rx.msgid==0U) {heartbeats++;}
    else if(rx.msgid==24U || rx.msgid==33U || rx.msgid==252U) {
      legacy_gps++;
      if(rx.msgid==24U) {legacy_mask|=1U;}
      else if(rx.msgid==33U) {legacy_mask|=2U;}
      else {
        mavlink_named_value_int_t named;
        mavlink_msg_named_value_int_decode(&rx,&named);
        if(memcmp(named.name,"GNSS_SAT",8U)==0) {legacy_mask|=4U;}
        if(memcmp(named.name,"GNSSUTC",7U)==0) {legacy_mask|=8U;}
        if(memcmp(named.name,"BAROALT",7U)==0) {legacy_mask|=16U;}
      }
    }
    else if(rx.msgid!=385U) {other_telemetry++;}
    if(rx.msgid!=385U) {continue;}
    mavlink_msg_tunnel_decode(&rx,&t); assert(t.payload_type!=0x8021U);
    if(t.payload_type==0x8020U) {raw_parts++;raw_complete+=StreamAcceptRaw(&t,&maximum_latency);}
    else if(t.payload_type==0x8003U) {priority_echoes++;}
    else if(t.payload_type>=0x8030U && t.payload_type<=0x8032U) {StreamAcceptBmi(&t);}
    else if(t.payload_type>=0x8011U && t.payload_type<=0x8014U) {
      uint32_t id=Read32(t.payload+4U); uint8_t bit;
      if(id!=typed_id) {typed_id=id;typed_ms=Read32(t.payload+8U);typed_mask=0U;}
      assert(t.payload[0]==1U && Read32(t.payload+8U)==typed_ms && t.payload_length==((t.payload_type==0x8013U)?32U:64U));
      if(t.payload_type==0x8013U) {assert(t.payload[12]==0U && t.payload[13]==0U);}
      bit=(t.payload_type==0x8011U)?0U:(t.payload_type==0x8012U)?(uint8_t)(1U+t.payload[2]):(t.payload_type==0x8013U)?3U:(uint8_t)(4U+t.payload[2]);
      assert(bit<6U); typed_mask|=(uint8_t)(1U<<bit);
      if(typed_mask==63U) {typed_groups++;typed_mask=0U;}
    }
  }
  printf("SAT_STREAM: input=%u enqueued=%lu complete=%lu parts=%lu full_drops=%lu aborted=%lu maximum_latency_ms=%lu typed_groups=%lu ack=%lu echoes=%lu legacy=%lu heartbeat=%lu other=%lu\n",
         STREAM_FRAMES,(unsigned long)s_raw.counters.enqueued_frames,(unsigned long)raw_complete,(unsigned long)raw_parts,
         (unsigned long)s_raw.counters.full_drops,(unsigned long)s_raw.counters.aborted_frames,(unsigned long)maximum_latency,
         (unsigned long)typed_groups,(unsigned long)acks,(unsigned long)priority_echoes,(unsigned long)legacy_gps,(unsigned long)heartbeats,(unsigned long)other_telemetry);
  assert(s_raw.counters.input_events==STREAM_FRAMES && s_raw.counters.enqueued_frames==STREAM_FRAMES);
  assert(raw_complete==STREAM_FRAMES && s_raw.counters.completed_frames==STREAM_FRAMES && raw_parts==STREAM_EPOCHS*24U);
  assert(s_raw.counters.full_drops==0U && s_raw.counters.aborted_frames==0U && s_raw.counters.error_attempts==0U && s_raw.depth==0U);
  assert(typed_groups>=8U && acks==8U && priority_echoes==4U && legacy_mask==31U && legacy_gps>=8U && heartbeats>=8U && other_telemetry>=8U);
  assert(maximum_latency<1500U && lora_calls==0U && raw_critical_depth==0U);
  printf("BMI_COEXIST: source=900 raw=%lu parsed=%lu pairs=%lu status=%lu\n",
         (unsigned long)stream_bmi_raw,(unsigned long)stream_bmi_parsed,(unsigned long)stream_bmi_complete,(unsigned long)stream_bmi_status);
  assert(stream_bmi_complete>=40U && stream_bmi_complete<=46U);
  assert(stream_bmi_raw==stream_bmi_complete && stream_bmi_parsed==stream_bmi_complete && stream_bmi_status>=8U);
  for(epoch=0U;epoch<STREAM_EPOCHS;epoch++) {for(kind=0U;kind<STREAM_KINDS;kind++) {assert(stream_frames[(uint16_t)epoch*STREAM_KINDS+kind].complete);}}
  puts("GNSS/BMI stream: PASS (real RunRpi 9s; 100Hz BMI source / 5Hz paired raw+parsed+status; 8 x PVT+DOP+64SV; original wire intact; ACK/legacy fair)");
  return 0;
}
