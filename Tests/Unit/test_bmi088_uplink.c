/**
 * @file test_bmi088_uplink.c
 * @brief 真实BMI编码器与MAVLink第五路的配对、有效性、重试及黄金字节回归。
 * @details 复用GNSS串口边界夹具而保留真实编码/调度，未访问硬件或修改生产采样。
 */
#define main BmiFixtureUnusedGnssMain
#include "test_gnss_raw_uplink.c"
#undef main
#ifndef TEST_GNSS_BMI088_IMPLEMENTATION_INCLUDED
#include "../../Framework/Src/px4lite_bmi088_uplink.c"
#endif

/** @brief 使用独立解析状态验证真实MAVLink2 CRC，而不是仅检查内层编码。 */
static uint8_t BmiTestDecodeWire(mavlink_tunnel_t *out)
{
  mavlink_message_t rx,out_message;
  mavlink_status_t rx_status,out_status;
  uint16_t i;uint8_t received=0U;
  memset(&rx,0,sizeof(rx));memset(&out_message,0,sizeof(out_message));
  memset(&rx_status,0,sizeof(rx_status));memset(&out_status,0,sizeof(out_status));
  for(i=0U;i<uart_length;i++) {
    uint8_t rc=mavlink_frame_char_buffer(&rx,&rx_status,uart_bytes[i],&out_message,&out_status);
    if(rc==MAVLINK_FRAMING_OK) { received++; }
  }
  if(received!=1U || out_message.msgid!=MAVLINK_MSG_ID_TUNNEL) { return 0U; }
  assert(out_message.sysid==42U && out_message.compid==PX4LITE_RPI_MAVLINK_COMPONENT_ID);
  mavlink_msg_tunnel_decode(&out_message,out);return 1U;
}

#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
/** @brief 独立生成符合量程和端序的源事实，未调用生产换算或编码函数。 */
static Px4Lite_Bmi088Sample_t BmiTestSample(uint32_t sequence,uint32_t now)
{
  Px4Lite_Bmi088Sample_t s;uint8_t i;
  memset(&s,0,sizeof(s));s.sample_sequence=sequence;s.sample_time_ms=now;s.reinit_count=2U;
  s.driver_error_count=0x01020304UL;s.fifo_drop_count=0x05060708UL;
  s.accel_mg[0]=-12000;s.accel_mg[1]=0;s.accel_mg[2]=12000;
  s.gyro_mdps[0]=-1000000;s.gyro_mdps[1]=500000;s.gyro_mdps[2]=-250000;
  s.temperature_cdeg=-1250;s.sample_period_us=10000U;s.bus=1U;s.accel_addr_7bit=0x18U;s.gyro_addr_7bit=0x68U;
  s.accel_chip_id=0x1EU;s.gyro_chip_id=0x0FU;s.accel_status=0x80U;s.gyro_fifo_status=4U;s.gyro_frame_count=4U;
  s.raw_accel[0]=0U;s.raw_accel[1]=0x80U;s.raw_accel[4]=0xFFU;s.raw_accel[5]=0x7FU;
  s.raw_accel[6]=0x33U;s.raw_accel[7]=0x22U;s.raw_accel[8]=0x11U;s.raw_temperature[0]=0xDCU;s.raw_temperature[1]=0x80U;
  for(i=0U;i<18U;i++) { s.raw_gyro[i]=i; }
  s.raw_gyro[18]=0U;s.raw_gyro[19]=0x80U;s.raw_gyro[20]=0U;s.raw_gyro[21]=0x40U;s.raw_gyro[22]=0U;s.raw_gyro[23]=0xE0U;
  s.axis_map[0]=1;s.axis_map[1]=2;s.axis_map[2]=3;return s;
}
/** @brief 模拟重启并清测试邮箱，生产Init本身不会清已经发布的源。 */
static void BmiTestReset(uint32_t now)
{
  memset(&s_bmi_mailbox,0,sizeof(s_bmi_mailbox));Sources(now);
  test_bmi_diag_result=PX4LITE_OK;test_bmi_module_state=PX4LITE_STATE_ONLINE;
  memset(&test_bmi_diag,0,sizeof(test_bmi_diag));test_bmi_diag.driver=PX4LITE_IMU_DRIVER_BMI088;
  test_bmi_diag.bus=1U;test_bmi_diag.accel_chip_id=0x1EU;test_bmi_diag.gyro_chip_id=0x0FU;
  test_bmi_diag.accel_addr_7bit=0x18U;test_bmi_diag.gyro_addr_7bit=0x68U;test_bmi_diag.device_ready=1U;
  test_bmi_diag.reinit_count=2U;test_bmi_diag.error_count=7U;test_bmi_diag.fifo_drop_count=8U;
  assert(Px4Lite_MavlinkTxInit(now)==PX4LITE_OK);uart_calls=0U;lora_calls=0U;h1_pending=b1_pending=0U;
}
/** @brief 记录状态首发，不消耗数据配对的raw/parsed顺序。 */
static void BmiTestInitialStatus(uint32_t now,uint8_t expected_present)
{
  Px4Lite_Bmi088UplinkView_t v;
  clock_ms=now;assert(Px4Lite_Bmi088UplinkPeek(now,&v)==PX4LITE_OK);
  assert(v.payload_type==0x8032U && v.length==64U && !!(v.payload[2]&1U)==expected_present);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,now);
}
/** @brief 按接口等待固定类型，所有多字节断言直接读取wire字节。 */
static Px4Lite_Bmi088UplinkView_t BmiTestPeek(uint32_t now,uint16_t type)
{
  Px4Lite_Bmi088UplinkView_t v;clock_ms=now;
  assert(Px4Lite_Bmi088UplinkPeek(now,&v)==PX4LITE_OK);assert(v.payload_type==type && v.length==64U);return v;
}
/** @brief 原始完整FIFO/解析机体系共用冻结ID，新邮箱不能覆盖活跃配对。 */
static void BmiTestPairAndBytes(void)
{
  Px4Lite_Bmi088Sample_t s=BmiTestSample(0x11223344UL,1000U),newer;
  Px4Lite_Bmi088UplinkView_t v;uint8_t raw[64];uint32_t i;
  BmiTestReset(1000U);Px4Lite_Bmi088UplinkPublish(&s);BmiTestInitialStatus(1000U,1U);
  v=BmiTestPeek(1000U,0x8030U);memcpy(raw,v.payload,64U);
  assert(memcmp(raw,"\x01\x00\x03\x01\x44\x33\x22\x11\xE8\x03\x00\x00\xE8\x03\x00\x00\x02\x00\x00\x00",20U)==0);
  assert(memcmp(&raw[20],"\x18\x68\x1E\x0F\x80\x04\x04\x01",8U)==0);
  assert(memcmp(&raw[28],s.raw_accel,9U)==0 && memcmp(&raw[37],s.raw_temperature,2U)==0 && memcmp(&raw[39],s.raw_gyro,24U)==0 && raw[63]==0U);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,1000U);
  newer=BmiTestSample(0x11223345UL,1010U);newer.accel_mg[0]=444;Px4Lite_Bmi088UplinkPublish(&newer);
  v=BmiTestPeek(1010U,0x8031U);
  assert(Read32(&v.payload[4])==s.sample_sequence && Read32(&v.payload[8])==1000U && Read32(&v.payload[12])==1010U);
  for(i=0U;i<3U;i++) { assert((int32_t)Read32(&v.payload[20U+i*4U])==s.accel_mg[i]);assert((int32_t)Read32(&v.payload[32U+i*4U])==s.gyro_mdps[i]); }
  assert((int16_t)Read16(&v.payload[44])==-1250 && Read16(&v.payload[46])==10000U);
  assert(Read16(&v.payload[48])==12000U && Read16(&v.payload[50])==1000U && v.payload[55]==1U);
  assert(memcmp(&v.payload[52],"\x01\x02\x03",3U)==0 && Read32(&v.payload[56])==s.fifo_drop_count && Read32(&v.payload[60])==s.driver_error_count);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,1010U);assert(s_bmi_uplink.completed_pairs==1U);
  assert(Px4Lite_Bmi088UplinkPeek(1199U,&v)==PX4LITE_IDLE);
  v=BmiTestPeek(1200U,0x8030U);assert(Read32(&v.payload[4])==newer.sample_sequence);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,1200U);
}
/** @brief 未使用FIFO尾部零填充，SPI地址为0，合法轴映射的负值逐字节保留。 */
static void BmiTestSpiAndPadding(void)
{
  Px4Lite_Bmi088Sample_t s=BmiTestSample(1U,0U);Px4Lite_Bmi088UplinkView_t v;uint8_t i;
  BmiTestReset(0U);s.bus=2U;s.accel_addr_7bit=s.gyro_addr_7bit=0U;s.gyro_frame_count=s.gyro_fifo_status=1U;
  s.axis_map[0]=2;s.axis_map[1]=-1;s.axis_map[2]=3;
  Px4Lite_Bmi088UplinkPublish(&s);BmiTestInitialStatus(0U,1U);v=BmiTestPeek(0U,0x8030U);
  assert(v.payload[3]==2U && v.payload[20]==0U && v.payload[21]==0U);
  assert(memcmp(&v.payload[39],s.raw_gyro,6U)==0);for(i=45U;i<64U;i++) { assert(v.payload[i]==0U); }
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,0U);v=BmiTestPeek(10U,0x8031U);assert(v.payload[53]==255U);
}
/** @brief 无源仅状态，错误ID/未就绪诊断保留；源初始化不应伪造RAW零帧。 */
static void BmiTestNoSourceAndStatus(void)
{
  Px4Lite_Bmi088UplinkView_t v;BmiTestReset(0U);Px4Lite_Bmi088UplinkInvalidate();Px4Lite_Bmi088UplinkInvalidate();
  test_bmi_diag.accel_chip_id=0xFFU;test_bmi_diag.gyro_chip_id=0xA5U;test_bmi_diag.device_ready=0U;
  test_bmi_diag.last_stage=4U;test_bmi_diag.last_result=2U;test_bmi_module_state=PX4LITE_STATE_FAILED;
  v=BmiTestPeek(0U,0x8032U);assert(v.payload[2]==0U && Read32(&v.payload[56])==UINT32_MAX);
  assert(v.payload[48]==4U && v.payload[49]==2U && v.payload[51]==0U && v.payload[52]==0xFFU && v.payload[53]==0xA5U);
  assert(v.payload[60]==1U && v.payload[61]==0U && v.payload[62]==PX4LITE_STATE_FAILED);
  assert(s_bmi_mailbox.generation==0U);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,0U);assert(Px4Lite_Bmi088UplinkPeek(500U,&v)==PX4LITE_IDLE);
  test_bmi_diag_result=PX4LITE_NOT_READY;v=BmiTestPeek(1000U,0x8032U);assert(v.payload[60]==0U && v.payload[3]==0U && v.payload[52]==0U);
}
/** @brief 200ms边界/时间回绕、源撤销和换代都不能让旧配对再次有效。 */
static void BmiTestLifetimeAndInvalidate(void)
{
  Px4Lite_Bmi088Sample_t s;Px4Lite_Bmi088UplinkView_t v;uint32_t generation;
  BmiTestReset(0U);s=BmiTestSample(7U,0U);Px4Lite_Bmi088UplinkPublish(&s);BmiTestInitialStatus(0U,1U);
  v=BmiTestPeek(200U,0x8030U);assert(v.payload[2]==3U);Px4Lite_Bmi088UplinkResult(PX4LITE_OK,200U);
  assert(Px4Lite_Bmi088UplinkPeek(201U,&v)==PX4LITE_IDLE && s_bmi_uplink.abandoned_pairs==1U);
  BmiTestReset(UINT32_MAX-100U);s=BmiTestSample(8U,UINT32_MAX-100U);Px4Lite_Bmi088UplinkPublish(&s);BmiTestInitialStatus(UINT32_MAX-100U,1U);
  v=BmiTestPeek(99U,0x8030U);assert(v.payload[2]==3U);Px4Lite_Bmi088UplinkResult(PX4LITE_OK,99U);
  assert(Px4Lite_Bmi088UplinkPeek(100U,&v)==PX4LITE_IDLE && s_bmi_uplink.abandoned_pairs==1U);
  BmiTestReset(1000U);s=BmiTestSample(9U,1000U);Px4Lite_Bmi088UplinkPublish(&s);BmiTestInitialStatus(1000U,1U);
  v=BmiTestPeek(1010U,0x8030U);Px4Lite_Bmi088UplinkResult(PX4LITE_OK,1010U);
  Px4Lite_Bmi088UplinkInvalidate();generation=s_bmi_mailbox.generation;Px4Lite_Bmi088UplinkInvalidate();assert(s_bmi_mailbox.generation==generation);
  assert(Px4Lite_Bmi088UplinkPeek(1020U,&v)==PX4LITE_IDLE && s_bmi_uplink.abandoned_pairs==1U);
  s.sample_time_ms=1200U;s.sample_sequence++;s.reinit_count++;Px4Lite_Bmi088UplinkPublish(&s);v=BmiTestPeek(1200U,0x8030U);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,1200U);s.reinit_count++;s.sample_sequence++;s.sample_time_ms=1210U;Px4Lite_Bmi088UplinkPublish(&s);
  assert(Px4Lite_Bmi088UplinkPeek(1210U,&v)==PX4LITE_IDLE && s_bmi_uplink.abandoned_pairs==2U);
}
/** @brief 重试有界且重新编码时标/年龄，最近Peek参数错误不得推进旧offer。 */
static void BmiTestRetryAndOffer(void)
{
  Px4Lite_Bmi088Sample_t s=BmiTestSample(1U,0U);Px4Lite_Bmi088UplinkView_t v;
  BmiTestReset(0U);Px4Lite_Bmi088UplinkPublish(&s);
  v=BmiTestPeek(0U,0x8032U);assert(Px4Lite_Bmi088UplinkPeek(0U,0)==PX4LITE_INVALID_PARAM);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,0U);assert(s_bmi_uplink.status_pending && !s_bmi_uplink.status_attempts);
  BmiTestInitialStatus(0U,1U);v=BmiTestPeek(0U,0x8030U);Px4Lite_Bmi088UplinkResult(PX4LITE_IO_ERROR,0U);
  assert(Px4Lite_Bmi088UplinkPeek(49U,&v)==PX4LITE_IDLE);Px4Lite_Bmi088UplinkResult(PX4LITE_OK,49U);assert(s_bmi_uplink.part==0U);
  v=BmiTestPeek(50U,0x8030U);assert(Read32(&v.payload[12])==50U);Px4Lite_Bmi088UplinkResult(PX4LITE_BUSY,50U);
  v=BmiTestPeek(100U,0x8030U);Px4Lite_Bmi088UplinkResult(PX4LITE_IO_ERROR,100U);
  assert(s_bmi_uplink.tx_failures==3U && s_bmi_uplink.abandoned_pairs==1U);
  assert(Px4Lite_Bmi088UplinkPeek(200U,&v)==PX4LITE_IDLE);
  BmiTestReset(0U);v=BmiTestPeek(0U,0x8032U);Px4Lite_Bmi088UplinkResult(PX4LITE_IO_ERROR,0U);
  assert(Px4Lite_Bmi088UplinkPeek(49U,&v)==PX4LITE_IDLE);
  v=BmiTestPeek(50U,0x8032U);assert(Read32(&v.payload[44])==1U);Px4Lite_Bmi088UplinkResult(PX4LITE_IO_ERROR,50U);
  v=BmiTestPeek(100U,0x8032U);Px4Lite_Bmi088UplinkResult(PX4LITE_IO_ERROR,100U);
  assert(Px4Lite_Bmi088UplinkPeek(1099U,&v)==PX4LITE_IDLE);v=BmiTestPeek(1100U,0x8032U);
}
/** @brief Comm重新初始化不清Sensor邮箱；覆盖计数不冒称物理丢帧。 */
static void BmiTestMailboxAndValidation(void)
{
  Px4Lite_Bmi088Sample_t s=BmiTestSample(1U,0U);Px4Lite_Bmi088UplinkView_t v;
  BmiTestReset(0U);Px4Lite_Bmi088UplinkPublish(&s);s.sample_sequence++;Px4Lite_Bmi088UplinkPublish(&s);
  assert(s_bmi_mailbox.published_count==2U && s_bmi_mailbox.overwrite_count==1U);
  Px4Lite_Bmi088UplinkInit(0U);BmiTestInitialStatus(0U,1U);assert(s_bmi_mailbox.published_count==2U && !s_bmi_mailbox.unread);
  v=BmiTestPeek(0U,0x8030U);assert(Read32(&v.payload[4])==2U);
  BmiTestReset(0U);s=BmiTestSample(1U,0U);s.gyro_frame_count=5U;Px4Lite_Bmi088UplinkPublish(&s);v=BmiTestPeek(0U,0x8032U);
  assert((v.payload[2]&2U)==0U);Px4Lite_Bmi088UplinkResult(PX4LITE_OK,0U);assert(Px4Lite_Bmi088UplinkPeek(0U,&v)==PX4LITE_IDLE);
  BmiTestReset(0U);s=BmiTestSample(1U,1U);Px4Lite_Bmi088UplinkPublish(&s);v=BmiTestPeek(0U,0x8032U);
  assert((v.payload[2]&2U)==0U && Read32(&v.payload[56])==UINT32_MAX);
  BmiTestReset(0U);s=BmiTestSample(0U,0U);Px4Lite_Bmi088UplinkPublish(&s);v=BmiTestPeek(0U,0x8032U);
  assert(v.payload[2]==0U && Read32(&v.payload[28])==0U);Px4Lite_Bmi088UplinkResult(PX4LITE_OK,0U);
  assert(Px4Lite_Bmi088UplinkPeek(0U,&v)==PX4LITE_IDLE);
}
/** @brief 长期无源或UART高优先响应超过2^31ms后，不能因旧有符号期限再等24天。 */
static void BmiTestLongIdleRecovery(void)
{
  const uint32_t later=0x80000010UL;
  Px4Lite_Bmi088Sample_t s;Px4Lite_Bmi088UplinkView_t v;
  BmiTestReset(0U);BmiTestInitialStatus(0U,0U);
  s=BmiTestSample(1U,later);Px4Lite_Bmi088UplinkPublish(&s);
  v=BmiTestPeek(later,0x8032U);Px4Lite_Bmi088UplinkResult(PX4LITE_OK,later);
  v=BmiTestPeek(later,0x8030U);assert(Read32(&v.payload[8])==later);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,later);v=BmiTestPeek(later+10U,0x8031U);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,later+10U);
  BmiTestReset(0U);s=BmiTestSample(1U,0U);Px4Lite_Bmi088UplinkPublish(&s);BmiTestInitialStatus(0U,1U);
  v=BmiTestPeek(0U,0x8030U);Px4Lite_Bmi088UplinkResult(PX4LITE_OK,0U);
  v=BmiTestPeek(10U,0x8031U);Px4Lite_Bmi088UplinkResult(PX4LITE_OK,10U);Px4Lite_Bmi088UplinkInvalidate();
  s=BmiTestSample(2U,later);Px4Lite_Bmi088UplinkPublish(&s);v=BmiTestPeek(later,0x8032U);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,later);v=BmiTestPeek(later,0x8030U);assert(Read32(&v.payload[4])==2U);
  BmiTestReset(0U);v=BmiTestPeek(0U,0x8032U);Px4Lite_Bmi088UplinkResult(PX4LITE_IO_ERROR,0U);
  v=BmiTestPeek(later,0x8032U);assert(Read32(&v.payload[12])==later);
}
/** @brief 实际MAVLink2封装/CRC/源身份与独立LoRa序号，输出跨语言黄金外帧。 */
static void BmiTestRealMavlink(void)
{
  Px4Lite_Bmi088Sample_t s=BmiTestSample(0x11223344UL,1000U);
  mavlink_tunnel_t packet;uint8_t old_lora_seq;uint16_t i;
  BmiTestReset(1000U);Px4Lite_Bmi088UplinkPublish(&s);
  assert(MavTx_SendRpiBmi088Uplink(1000U)==PX4LITE_OK && BmiTestDecodeWire(&packet));assert(packet.payload_type==0x8032U);
  old_lora_seq=mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq;
  clock_ms=1010U;assert(MavTx_SendRpiBmi088Uplink(clock_ms)==PX4LITE_OK && BmiTestDecodeWire(&packet));
  assert(packet.payload_type==0x8030U && packet.payload_length==64U && packet.target_system==0U && packet.target_component==0U);
  assert(mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq==old_lora_seq && lora_calls==0U);
  printf("BMI_GOLDEN_RAW ");for(i=0U;i<uart_length;i++) { printf("%02X",uart_bytes[i]); }puts("");
  uart_bytes[20]^=1U;assert(!BmiTestDecodeWire(&packet));uart_bytes[20]^=1U;
  clock_ms=1020U;assert(MavTx_SendRpiBmi088Uplink(clock_ms)==PX4LITE_OK && BmiTestDecodeWire(&packet));
  assert(packet.payload_type==0x8031U && Read32(&packet.payload[4])==s.sample_sequence);
  printf("BMI_GOLDEN_PARSED ");for(i=0U;i<uart_length;i++) { printf("%02X",uart_bytes[i]); }puts("");
}
/** @brief 默认真实第五路与四个旧lane共存，一轮最多一次UART提交且不依赖LoRa。 */
static void BmiTestFiveLaneScheduler(void)
{
  uint32_t now;unsigned raw=0U,parsed=0U,status=0U,gnss=0U,gnss_raw=0U,other=0U;
  mavlink_tunnel_t packet;Px4Lite_Bmi088Sample_t s;
  BmiTestReset(0U);MakeRaw(100U,0U);Px4Lite_GnssRawEnqueue(raw_wire,100U,2U,0U,1U,7U,1U,0U);
  for(now=0U;now<2200U;now+=10U) {
    unsigned before=uart_calls;s=BmiTestSample(now/10U+1U,now);Px4Lite_Bmi088UplinkPublish(&s);clock_ms=now;
    assert(Px4Lite_MavlinkTxRunRpi(now)!=PX4LITE_INVALID_PARAM);assert(uart_calls-before<=1U);
    if(uart_calls==before) { continue; }
    if(BmiTestDecodeWire(&packet)) {
      if(packet.payload_type==0x8030U) { raw++; }
      else if(packet.payload_type==0x8031U) { parsed++; }
      else if(packet.payload_type==0x8032U) { status++; }
      else if(packet.payload_type>=0x8011U && packet.payload_type<=0x8014U) { gnss++; }
      else if(packet.payload_type==0x8020U) { gnss_raw++; }
      else { other++; }
    } else { other++; }
  }
  assert(raw>=8U && parsed>=8U && status>=2U && gnss>=12U && gnss_raw>=3U && other>0U && lora_calls==0U);
  printf("BMI 5lane raw=%u parsed=%u status=%u GNSS=%u GNSS_RAW=%u other=%u\n",raw,parsed,status,gnss,gnss_raw,other);
}
#endif

/** @brief 默认及关闭配置各自验证真实接口，不启动任务或访问硬件。 */
int main(void)
{
#if PX4LITE_ENABLE_RPI_BMI088_UPLINK
  BmiTestPairAndBytes();BmiTestSpiAndPadding();BmiTestNoSourceAndStatus();BmiTestLifetimeAndInvalidate();
  BmiTestRetryAndOffer();BmiTestMailboxAndValidation();BmiTestLongIdleRecovery();BmiTestRealMavlink();BmiTestFiveLaneScheduler();
  puts("BMI088 uplink: paired raw/parsed, status, freshness, retries, generation, CRC and 5-lane fairness PASS");
#else
  Px4Lite_Bmi088Sample_t sample;Px4Lite_Bmi088UplinkView_t view;mavlink_tunnel_t tunnel;
  memset(&sample,0,sizeof(sample));Px4Lite_Bmi088UplinkPublish(&sample);Px4Lite_Bmi088UplinkInvalidate();Px4Lite_Bmi088UplinkInit(0U);
  assert(Px4Lite_Bmi088UplinkPeek(0U,&view)==PX4LITE_IDLE && Px4Lite_Bmi088UplinkPeek(0U,0)==PX4LITE_INVALID_PARAM);
  Px4Lite_Bmi088UplinkResult(PX4LITE_OK,0U);memset(&tunnel,0,sizeof(tunnel));tunnel.payload_type=0x8030U;tunnel.payload_length=64U;
  mavlink_msg_tunnel_encode(42U,193U,&s_message,&tunnel);assert(!MavTx_RpiMessageAllowed(&s_message));
  (void)BmiTestDecodeWire;puts("BMI088 uplink disabled: no source state or BMI mirror forwarding PASS");
#endif
  return 0;
}
