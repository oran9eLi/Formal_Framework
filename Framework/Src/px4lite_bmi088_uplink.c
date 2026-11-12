/**
 * @file px4lite_bmi088_uplink.c
 * @brief BMI088单槽最新邮箱、冻结原始/解析配对及设备状态的小端上行编码。
 * @details Sensor只复制邮箱，Comm独占冻结/编码/重试；短临界区仅复制或设置标志。
 * 不访问Driver/BSP、不发送结构体内存、不分配运行期内存，不承诺100Hz全量无损。
 */
#include "px4lite_bmi088_uplink.h"
#include "px4lite_config.h"
#include "px4lite_platform.h"
#include "px4lite_modules.h"
#include "FreeRTOS.h"
#include "task.h"
#include <limits.h>
#include <string.h>

#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
/** @brief Sensor发布邮箱；generation撤销旧配对，unread仅用于邮箱覆盖计数。 */
typedef struct {
  Px4Lite_Bmi088Sample_t sample; /**< 最近一次完整源样本。 */
  uint32_t generation;          /**< 源生命周期代数，撤销或reinit变化时前进。 */
  uint32_t published_count;     /**< 累计Publish次数，uint32自然回绕。 */
  uint32_t overwrite_count;     /**< 上次邮箱内容尚未被Comm确认取走时被覆盖的次数。 */
  uint8_t present;              /**< 曾经收到完整源样本为1。 */
  uint8_t available;            /**< 源尚未撤销为1；年龄另判。 */
  uint8_t unread;               /**< 最新邮箱尚未被Comm冻结确认为1。 */
  uint8_t invalidated;          /**< 最近已存在的源被撤销为1，新Publish清零。 */
} Bmi088Mailbox_t;

/** @brief Comm独占的冻结配对；当前pair不会被100Hz新邮箱覆盖。 */
typedef struct {
  Px4Lite_Bmi088Sample_t sample; /**< 本次raw/parsed共用的同次样本。 */
  uint32_t generation;          /**< 冻结时源代数。 */
  uint32_t last_pair_ms;        /**< 上次真正抽样时刻，ms，按无符号elapsed限速。 */
  uint32_t pair_retry_ms;       /**< 当前数据包最近失败时刻，ms。 */
  uint32_t last_status_ms;      /**< 上次状态报告结束时刻，ms。 */
  uint32_t status_retry_ms;     /**< 状态包最近失败时刻，ms。 */
  uint32_t last_sequence;       /**< 上次冻结序号，避免重复抽样旧值。 */
  uint32_t last_sample_ms;      /**< 上次冻结采样时刻，ms，0合法。 */
  uint32_t last_generation;     /**< 上次冻结源代数。 */
  uint32_t completed_pairs;     /**< 两包均UART提交成功的次数。 */
  uint32_t abandoned_pairs;     /**< 过期/撤销/失败耗尽的冻结配对数。 */
  uint32_t tx_failures;         /**< 非OK提交尝试数，不当作对端丢帧数。 */
  uint8_t payload[PX4LITE_BMI088_UPLINK_PAYLOAD_SIZE]; /**< 当前借用载荷。 */
  uint8_t active;               /**< 1有冻结数据配对。 */
  uint8_t part;                 /**< 0原始、1解析。 */
  uint8_t pair_attempts;        /**< 当前数据包已尝试次数0..3。 */
  uint8_t status_pending;       /**< 1当前状态报告等待提交/重试。 */
  uint8_t status_attempts;      /**< 当前状态报告已尝试次数0..3。 */
  uint8_t offered;              /**< 最近Peek：0无、1原始、2解析、3状态。 */
  uint8_t have_last;            /**< 标记已有抽样；0毫秒合法，Driver序号应非0。 */
  uint8_t have_status;          /**< 1已有结束的状态报告，0允许立即发送。 */
} Bmi088UplinkState_t;
static Bmi088Mailbox_t s_bmi_mailbox;
static Bmi088UplinkState_t s_bmi_uplink;

/** @brief 固定小端写16位，包括有符号量的二补码位模式。 */
static void BmiUplinkPut16(uint8_t *p,uint16_t value)
{ p[0]=(uint8_t)value;p[1]=(uint8_t)(value>>8U); }
/** @brief 固定小端写32位，不依赖C结构体布局或主机端序。 */
static void BmiUplinkPut32(uint8_t *p,uint32_t value)
{ p[0]=(uint8_t)value;p[1]=(uint8_t)(value>>8U);p[2]=(uint8_t)(value>>16U);p[3]=(uint8_t)(value>>24U); }
/** @brief 源未来时标不能授权；0ms合法，未知年龄使用FFFFFFFF。 */
static uint32_t BmiUplinkAge(uint32_t now,uint32_t sample_ms,uint8_t present)
{ uint32_t age=now-sample_ms;return (!present || age>0x7FFFFFFFUL)?UINT32_MAX:age; }
/** @brief 只复制Sensor邮箱，不执行计算或硬件IO。 */
static void BmiUplinkCopyMailbox(Bmi088Mailbox_t *out)
{ taskENTER_CRITICAL();*out=s_bmi_mailbox;taskEXIT_CRITICAL(); }
/** @brief 仅在邮箱仍是冻结样本时标记已读取，不清除并发发布的新样本。 */
static void BmiUplinkAcknowledgeMailbox(const Bmi088Mailbox_t *copy)
{
  taskENTER_CRITICAL();
  if(s_bmi_mailbox.generation==copy->generation &&
     s_bmi_mailbox.sample.sample_sequence==copy->sample.sample_sequence &&
     s_bmi_mailbox.sample.sample_time_ms==copy->sample.sample_time_ms) { s_bmi_mailbox.unread=0U; }
  taskEXIT_CRITICAL();
}
/** @brief 仅校验跨层元数据，不重新解析寄存器或从工程量反推原始字节。 */
static uint8_t BmiUplinkSampleKnown(const Px4Lite_Bmi088Sample_t *s)
{
  uint8_t i,j;int axis[3];
  if(s->sample_sequence==0U || (s->bus!=1U && s->bus!=2U) || s->accel_chip_id!=0x1EU || s->gyro_chip_id!=0x0FU ||
     s->gyro_frame_count<1U || s->gyro_frame_count>4U ||
     (s->accel_status&0x80U)==0U || (s->gyro_fifo_status&0x80U)!=0U ||
     (s->gyro_fifo_status&0x7FU)!=s->gyro_frame_count || s->raw_temperature[0]==0x80U ||
     s->sample_period_us==0U) { return 0U; }
  if(s->bus==1U) {
    if((s->accel_addr_7bit!=0x18U && s->accel_addr_7bit!=0x19U) ||
       (s->gyro_addr_7bit!=0x68U && s->gyro_addr_7bit!=0x69U)) { return 0U; }
  } else if(s->accel_addr_7bit!=0U || s->gyro_addr_7bit!=0U) { return 0U; }
  for(i=0U;i<3U;i++) {
    axis[i]=s->axis_map[i]<0?-(int)s->axis_map[i]:(int)s->axis_map[i];
    if(axis[i]<1 || axis[i]>3) { return 0U; }
    for(j=0U;j<i;j++) { if(axis[i]==axis[j]) { return 0U; } }
  }
  return (uint8_t)(((int)s->axis_map[0]*(int)s->axis_map[1]*(int)s->axis_map[2]*
      (axis[0]-axis[1])*(axis[1]-axis[2])*(axis[2]-axis[0]))>0);
}
/** @brief 清除旧配对并累计一次中止，不重发已经失败的旧序号。 */
static void BmiUplinkAbandon(void)
{
  if(s_bmi_uplink.active) { s_bmi_uplink.abandoned_pairs++; }
  s_bmi_uplink.active=0U;s_bmi_uplink.pair_attempts=0U;
  if(s_bmi_uplink.offered==1U || s_bmi_uplink.offered==2U) { s_bmi_uplink.offered=0U; }
}
/** @brief 失败/换代或超过200ms的冻结配对不再授权，年龄不由新邮箱续期。 */
static void BmiUplinkCheckPair(const Bmi088Mailbox_t *source,uint32_t now)
{
  if(s_bmi_uplink.active && (!source->available || source->generation!=s_bmi_uplink.generation ||
      BmiUplinkAge(now,s_bmi_uplink.sample.sample_time_ms,1U)>PX4LITE_RPI_BMI088_MAX_AGE_MS)) { BmiUplinkAbandon(); }
}
/** @brief 按周期冻结最新完整样本，重复旧样本不产生新配对。 */
static void BmiUplinkTryFreeze(const Bmi088Mailbox_t *source,uint32_t now)
{
  if(s_bmi_uplink.active || (s_bmi_uplink.have_last &&
     (uint32_t)(now-s_bmi_uplink.last_pair_ms)<PX4LITE_RPI_BMI088_PERIOD_MS) ||
     !source->present || !source->available || !BmiUplinkSampleKnown(&source->sample) ||
     BmiUplinkAge(now,source->sample.sample_time_ms,1U)>PX4LITE_RPI_BMI088_MAX_AGE_MS) { return; }
  if(s_bmi_uplink.have_last && source->generation==s_bmi_uplink.last_generation &&
     source->sample.sample_sequence==s_bmi_uplink.last_sequence &&
     source->sample.sample_time_ms==s_bmi_uplink.last_sample_ms) { return; }
  s_bmi_uplink.sample=source->sample;s_bmi_uplink.generation=source->generation;
  s_bmi_uplink.active=1U;s_bmi_uplink.part=0U;s_bmi_uplink.pair_attempts=0U;
  s_bmi_uplink.pair_retry_ms=now;s_bmi_uplink.last_pair_ms=now;
  s_bmi_uplink.last_sequence=source->sample.sample_sequence;s_bmi_uplink.last_sample_ms=source->sample.sample_time_ms;
  s_bmi_uplink.last_generation=source->generation;s_bmi_uplink.have_last=1U;
  BmiUplinkAcknowledgeMailbox(source);
}
/** @brief 编码三类共同头；present与usable分离，0ms不会误判未知。 */
static void BmiUplinkHeader(const Px4Lite_Bmi088Sample_t *sample,uint8_t kind,uint8_t present,
                            uint8_t available,uint8_t invalidated,uint32_t now)
{
  uint8_t flags=present?1U:0U;
  uint32_t age=BmiUplinkAge(now,sample->sample_time_ms,present);
  memset(s_bmi_uplink.payload,0,sizeof(s_bmi_uplink.payload));
  if(present && available && BmiUplinkSampleKnown(sample) && age<=PX4LITE_RPI_BMI088_MAX_AGE_MS) { flags|=2U; }
  if(present && age>PX4LITE_RPI_BMI088_MAX_AGE_MS) { flags|=4U; }
  if(invalidated) { flags|=8U; }
  s_bmi_uplink.payload[0]=1U;s_bmi_uplink.payload[1]=kind;s_bmi_uplink.payload[2]=flags;s_bmi_uplink.payload[3]=sample->bus;
  BmiUplinkPut32(&s_bmi_uplink.payload[4],present?sample->sample_sequence:0U);
  BmiUplinkPut32(&s_bmi_uplink.payload[8],present?sample->sample_time_ms:0U);
  BmiUplinkPut32(&s_bmi_uplink.payload[12],now);
  BmiUplinkPut32(&s_bmi_uplink.payload[16],sample->reinit_count);
}
/** @brief 保真复制实际读取的寄存器字节，未使用FIFO尾部保持0而不伪装有效字节。 */
static void BmiUplinkRaw(uint32_t now)
{
  const Px4Lite_Bmi088Sample_t *s=&s_bmi_uplink.sample;uint8_t *p=s_bmi_uplink.payload;
  BmiUplinkHeader(s,0U,1U,1U,0U,now);
  p[20]=s->accel_addr_7bit;p[21]=s->gyro_addr_7bit;p[22]=s->accel_chip_id;p[23]=s->gyro_chip_id;
  p[24]=s->accel_status;p[25]=s->gyro_fifo_status;p[26]=s->gyro_frame_count;p[27]=1U;
  memcpy(&p[28],s->raw_accel,9U);memcpy(&p[37],s->raw_temperature,2U);
  memcpy(&p[39],s->raw_gyro,(uint16_t)s->gyro_frame_count*6U);
}
/** @brief 输出平台统一物理单位和轴映射，保持与同次raw序号/时标一致。 */
static void BmiUplinkParsed(uint32_t now)
{
  const Px4Lite_Bmi088Sample_t *s=&s_bmi_uplink.sample;uint8_t *p=s_bmi_uplink.payload;uint8_t i;
  BmiUplinkHeader(s,1U,1U,1U,0U,now);
  for(i=0U;i<3U;i++) { BmiUplinkPut32(&p[20U+i*4U],(uint32_t)s->accel_mg[i]);BmiUplinkPut32(&p[32U+i*4U],(uint32_t)s->gyro_mdps[i]);p[52U+i]=(uint8_t)s->axis_map[i]; }
  BmiUplinkPut16(&p[44],(uint16_t)s->temperature_cdeg);BmiUplinkPut16(&p[46],s->sample_period_us);
  BmiUplinkPut16(&p[48],12000U);BmiUplinkPut16(&p[50],1000U);p[55]=1U;
  BmiUplinkPut32(&p[56],s->fifo_drop_count);BmiUplinkPut32(&p[60],s->driver_error_count);
}
/** @brief 即使无测量也报告当前设备与邮箱事实，错误ID和未就绪阶段原样保留。 */
static void BmiUplinkStatus(const Bmi088Mailbox_t *source,uint32_t now)
{
  Px4Lite_ImuDiagnostics_t diag;Px4Lite_ModuleStatus_t status;uint8_t *p=s_bmi_uplink.payload;
  uint8_t diag_available;Px4Lite_Result_t result;
  memset(&diag,0,sizeof(diag));memset(&status,0,sizeof(status));
  result=Px4Lite_ImuCopyDiagnostics(&diag);diag_available=(uint8_t)(result==PX4LITE_OK && diag.driver==PX4LITE_IMU_DRIVER_BMI088);
  BmiUplinkHeader(&source->sample,2U,source->present,source->available,source->invalidated,now);
  if(diag_available) { p[3]=diag.bus;BmiUplinkPut32(&p[16],diag.reinit_count); }
  BmiUplinkPut32(&p[20],diag_available?diag.error_count:source->sample.driver_error_count);
  BmiUplinkPut32(&p[24],diag_available?diag.fifo_drop_count:source->sample.fifo_drop_count);
  BmiUplinkPut32(&p[28],source->published_count);BmiUplinkPut32(&p[32],source->overwrite_count);
  BmiUplinkPut32(&p[36],s_bmi_uplink.completed_pairs);BmiUplinkPut32(&p[40],s_bmi_uplink.abandoned_pairs);
  BmiUplinkPut32(&p[44],s_bmi_uplink.tx_failures);
  if(diag_available) {
    p[48]=diag.last_stage;p[49]=diag.last_result;p[50]=diag.last_bsp_status;p[51]=diag.device_ready;
    p[52]=diag.accel_chip_id;p[53]=diag.gyro_chip_id;p[54]=diag.accel_addr_7bit;p[55]=diag.gyro_addr_7bit;
  }
  BmiUplinkPut32(&p[56],BmiUplinkAge(now,source->sample.sample_time_ms,source->present));
  p[60]=diag_available;p[61]=(uint8_t)(source->present && source->available);
  p[62]=(Px4Lite_GetModuleStatus(PX4LITE_MODULE_IMU,&status)==PX4LITE_OK)?(uint8_t)status.state:255U;
}
#endif

/** @brief Sensor只发布完整同次副本；新生命周期换代使旧冻结配对失去授权。 */
void Px4Lite_Bmi088UplinkPublish(const Px4Lite_Bmi088Sample_t *sample)
{
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
  uint32_t generation,published,overwritten;
  if(sample==0 || sample->sample_sequence==0U) { return; }
  /* 这三个累计量仅Sensor写；先在区外算好，Comm仅能清unread标志。 */
  generation=s_bmi_mailbox.generation;
  if(s_bmi_mailbox.present && s_bmi_mailbox.sample.reinit_count!=sample->reinit_count) { generation++; }
  published=s_bmi_mailbox.published_count+1U;overwritten=s_bmi_mailbox.overwrite_count+1U;
  taskENTER_CRITICAL();
  if(s_bmi_mailbox.available && s_bmi_mailbox.unread) { s_bmi_mailbox.overwrite_count=overwritten; }
  s_bmi_mailbox.sample=*sample;s_bmi_mailbox.present=1U;s_bmi_mailbox.available=1U;
  s_bmi_mailbox.generation=generation;s_bmi_mailbox.published_count=published;
  s_bmi_mailbox.unread=1U;s_bmi_mailbox.invalidated=0U;
  taskEXIT_CRITICAL();
#else
  (void)sample;
#endif
}
/** @brief 幂等撤销；未收到样本时不凭空制造换代或无效测量。 */
void Px4Lite_Bmi088UplinkInvalidate(void)
{
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
  uint32_t generation=s_bmi_mailbox.generation+1U;
  taskENTER_CRITICAL();
  if(s_bmi_mailbox.available) {
    s_bmi_mailbox.generation=generation;s_bmi_mailbox.available=0U;s_bmi_mailbox.unread=0U;s_bmi_mailbox.invalidated=1U;
  }
  taskEXIT_CRITICAL();
#endif
}
/** @brief 仅Comm初始化；不清除可能已经被Sensor发布的源邮箱。 */
void Px4Lite_Bmi088UplinkInit(uint32_t now_ms)
{
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
  memset(&s_bmi_uplink,0,sizeof(s_bmi_uplink));s_bmi_uplink.last_pair_ms=now_ms;s_bmi_uplink.last_status_ms=now_ms;
#else
  (void)now_ms;
#endif
}
/** @brief Comm每次只借出一个包；到期状态优先但50ms重试间隔允许数据继续。 */
Px4Lite_Result_t Px4Lite_Bmi088UplinkPeek(uint32_t now_ms,Px4Lite_Bmi088UplinkView_t *out)
{
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
  s_bmi_uplink.offered=0U;
#endif
  if(out==0) { return PX4LITE_INVALID_PARAM; }
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
  Bmi088Mailbox_t source;
  BmiUplinkCopyMailbox(&source);BmiUplinkCheckPair(&source,now_ms);BmiUplinkTryFreeze(&source,now_ms);
  if(!s_bmi_uplink.status_pending && (!s_bmi_uplink.have_status ||
     (uint32_t)(now_ms-s_bmi_uplink.last_status_ms)>=PX4LITE_RPI_BMI088_STATUS_PERIOD_MS)) {
    s_bmi_uplink.status_pending=1U;s_bmi_uplink.status_attempts=0U;s_bmi_uplink.status_retry_ms=now_ms;
  }
  if(s_bmi_uplink.status_pending && (!s_bmi_uplink.status_attempts ||
     (uint32_t)(now_ms-s_bmi_uplink.status_retry_ms)>=PX4LITE_RPI_BMI088_RETRY_MS)) {
    BmiUplinkStatus(&source,now_ms);out->payload_type=PX4LITE_BMI088_UPLINK_STATUS_TYPE;s_bmi_uplink.offered=3U;
  } else if(s_bmi_uplink.active && (!s_bmi_uplink.pair_attempts ||
            (uint32_t)(now_ms-s_bmi_uplink.pair_retry_ms)>=PX4LITE_RPI_BMI088_RETRY_MS)) {
    if(s_bmi_uplink.part==0U) { BmiUplinkRaw(now_ms);out->payload_type=PX4LITE_BMI088_UPLINK_RAW_TYPE;s_bmi_uplink.offered=1U; }
    else { BmiUplinkParsed(now_ms);out->payload_type=PX4LITE_BMI088_UPLINK_PARSED_TYPE;s_bmi_uplink.offered=2U; }
  } else { return PX4LITE_IDLE; }
  out->payload=s_bmi_uplink.payload;out->length=PX4LITE_BMI088_UPLINK_PAYLOAD_SIZE;return PX4LITE_OK;
#else
  (void)now_ms;return PX4LITE_IDLE;
#endif
}
/** @brief 根据UART实际提交推进一包；对端接收与CRC验收不由本机成功计数代替。 */
void Px4Lite_Bmi088UplinkResult(Px4Lite_Result_t result,uint32_t now_ms)
{
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
  uint8_t offered=s_bmi_uplink.offered;Bmi088Mailbox_t source;
  s_bmi_uplink.offered=0U;if(!offered) { return; }
  if(result!=PX4LITE_OK) { s_bmi_uplink.tx_failures++; }
  if(offered==3U) {
    s_bmi_uplink.status_attempts++;
    if(result==PX4LITE_OK || s_bmi_uplink.status_attempts>=PX4LITE_RPI_BMI088_MAX_ATTEMPTS) {
      s_bmi_uplink.status_pending=0U;s_bmi_uplink.last_status_ms=now_ms;s_bmi_uplink.have_status=1U;
    } else { s_bmi_uplink.status_retry_ms=now_ms; }
    return;
  }
  BmiUplinkCopyMailbox(&source);BmiUplinkCheckPair(&source,now_ms);if(!s_bmi_uplink.active) { return; }
  s_bmi_uplink.pair_attempts++;
  if(result==PX4LITE_OK) {
    s_bmi_uplink.pair_attempts=0U;s_bmi_uplink.pair_retry_ms=now_ms;
    if(s_bmi_uplink.part==0U) { s_bmi_uplink.part=1U; }
    else { s_bmi_uplink.completed_pairs++;s_bmi_uplink.active=0U; }
  } else if(s_bmi_uplink.pair_attempts>=PX4LITE_RPI_BMI088_MAX_ATTEMPTS) { BmiUplinkAbandon(); }
  else { s_bmi_uplink.pair_retry_ms=now_ms; }
#else
  (void)result;(void)now_ms;
#endif
}
