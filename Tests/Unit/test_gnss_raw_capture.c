/**
 * @file test_gnss_raw_capture.c
 * @brief 复用真实双协议驱动和既有BSP字节流夹具，验证原帧快照保真及生命周期。
 * @details 所有输入为主机测试向量，不向板端注入数据；采集不依赖Debug开关。
 */
#define main GnssBringupRegressionMain
#include "test_gnss_bringup.c"
#undef main

static uint8_t s_observed_ubx[GNSS_RAW_MAX_BYTES];
static uint8_t s_callback_snapshot[GNSS_RAW_MAX_BYTES];
static uint16_t s_observed_length;
static uint8_t s_observed_class,s_observed_id,s_observed_checksum;
static uint32_t s_observed_ms,s_ubx_callback_count,s_nmea_callback_count;

/** @brief 回调只复制测试快照，核对原Raw槽已发布；不得从BSP另取字节。 */
static void ObserveUbx(const uint8_t *bytes,uint16_t length,uint8_t cls,uint8_t id,uint8_t checksum_valid,uint32_t now_ms)
{
  Gnss_RawFrameInfo_t info;
  uint8_t slot=(cls==1U && id==7U)?0U:(cls==1U && id==4U)?1U:2U;
  assert(bytes!=NULL && length>=8U && length<=GNSS_RAW_MAX_BYTES);
  assert(bytes[0]==0xB5U && bytes[1]==0x62U && bytes[2]==cls && bytes[3]==id);
  assert(length==(uint16_t)(8U+(uint16_t)bytes[4]+((uint16_t)bytes[5]<<8)));
  assert(Sensor_GNSS_CopyRawFrame(slot,s_callback_snapshot,sizeof(s_callback_snapshot),&info)==GNSS_RESULT_OK);
  assert(info.length==length && info.received_ms==now_ms && info.checksum_valid==checksum_valid);
  assert(!memcmp(s_callback_snapshot,bytes,length));
  memcpy(s_observed_ubx,bytes,length);s_observed_length=length;
  s_observed_class=cls;s_observed_id=id;s_observed_checksum=checksum_valid;s_observed_ms=now_ms;
  s_ubx_callback_count++;
}

/** @brief 独立记录NMEA提交次数，验证UBX不从NMEA入口重复提交。 */
static void ObserveNmea(const uint8_t *bytes,uint16_t length,uint8_t type,uint8_t checksum_valid,uint32_t now_ms)
{
  (void)bytes;(void)length;(void)type;(void)checksum_valid;(void)now_ms;s_nmea_callback_count++;
}

/** @brief 校验槽元数据与实际投喂wire逐字节一致，读取不消费尚未处理的BSP队列。 */
static Gnss_RawFrameInfo_t ExpectRaw(uint8_t slot, const uint8_t *wire, uint16_t length,
                                    uint8_t protocol, uint8_t checksum_valid)
{
  uint8_t bytes[GNSS_RAW_MAX_BYTES];
  Gnss_RawFrameInfo_t info;
  uint16_t queued=input_size;
  assert(Sensor_GNSS_CopyRawFrame(slot,bytes,sizeof(bytes),&info)==GNSS_RESULT_OK);
  assert(input_size==queued && info.length==length && info.protocol==protocol && info.checksum_valid==checksum_valid);
  assert(memcmp(bytes,wire,length)==0);
  return info;
}

/** @brief 空槽、非法参数、容量不足均不写输出，不用序号0或时刻0判空。 */
static void TestRawParameters(void)
{
  uint8_t bytes[GNSS_RAW_MAX_BYTES],saved[GNSS_RAW_MAX_BYTES],wire[8];
  Gnss_RawFrameInfo_t info,saved_info;
  unsigned j;
  Cold();memset(bytes,0xA5,sizeof(bytes));memcpy(saved,bytes,sizeof(bytes));
  memset(&info,0x5A,sizeof(info));saved_info=info;
  for(j=0U;j<GNSS_RAW_SLOT_COUNT;j++) { assert(Sensor_GNSS_CopyRawFrame((uint8_t)j,bytes,sizeof(bytes),&info)==GNSS_RESULT_NO_DATA); }
  assert(Sensor_GNSS_CopyRawFrame(3U,bytes,sizeof(bytes),&info)==GNSS_RESULT_INVALID_PARAM);
  assert(Sensor_GNSS_CopyRawFrame(0U,NULL,sizeof(bytes),&info)==GNSS_RESULT_INVALID_PARAM);
  assert(Sensor_GNSS_CopyRawFrame(0U,bytes,sizeof(bytes),NULL)==GNSS_RESULT_INVALID_PARAM);
  assert(Sensor_GNSS_CopyRawFrame(0U,bytes,0U,&info)==GNSS_RESULT_INVALID_PARAM);
  assert(!memcmp(bytes,saved,sizeof(bytes)) && !memcmp(&info,&saved_info,sizeof(info)));
  assert(Ubx_BuildFrame(0x99U,0x88U,NULL,0U,wire,sizeof(wire))==8U);Raw(wire,sizeof(wire));Step(0U);
  assert(Sensor_GNSS_CopyRawFrame(2U,bytes,7U,&info)==GNSS_RESULT_INVALID_PARAM);
  assert(!memcmp(bytes,saved,sizeof(bytes)) && !memcmp(&info,&saved_info,sizeof(info)));
  info=ExpectRaw(2U,wire,sizeof(wire),GNSS_PROTOCOL_UBX,1U);
  assert(info.sequence==0U && info.received_ms==0U && info.class_id==0x99U && info.message_id==0x88U);
}

/** @brief 分片PVT、坏CRC以及DOP/其它帧分别采样；坏CRC原字节保留且绝不更新测量。 */
static void TestRawUbxIntegrityAndSlots(void)
{
  uint8_t payload[92],pvt[100],dop[26],other[19],bytes[GNSS_RAW_MAX_BYTES];
  Gnss_RawFrameInfo_t info;
  Gnss_Snapshot_t snapshot;
  unsigned j;
  Cold();for(j=0U;j<sizeof(payload);j++) { payload[j]=(uint8_t)(j*37U); }
  payload[20]=0U;payload[21]=0U;payload[23]=7U;
  assert(Ubx_BuildFrame(1U,7U,payload,sizeof(payload),pvt,sizeof(pvt))==sizeof(pvt));
  Raw(pvt,31U);Step(0U);
  assert(Sensor_GNSS_CopyRawFrame(0U,bytes,sizeof(bytes),&info)==GNSS_RESULT_NO_DATA);
  Raw(pvt+31U,(uint16_t)(sizeof(pvt)-31U));Step(10U);
  info=ExpectRaw(0U,pvt,sizeof(pvt),GNSS_PROTOCOL_UBX,1U);
  assert(info.sequence==0U && info.received_ms==10U && info.class_id==1U && info.message_id==7U);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_FIX && snapshot.status_update_ms==10U);
  pvt[98]^=0x55U;pvt[99]=0xB5U;Raw(pvt,sizeof(pvt));Step(20U);
  info=ExpectRaw(0U,pvt,sizeof(pvt),GNSS_PROTOCOL_UBX,0U);
  assert(info.sequence==1U && info.received_ms==20U && Diag().valid_ubx==1U && Diag().checksum_errors==1U);
  (void)Sensor_GNSS_CopySnapshot(&snapshot);assert(snapshot.status_update_ms==10U);
  assert(Ubx_BuildFrame(1U,4U,payload,18U,dop,sizeof(dop))==sizeof(dop));Raw(dop,sizeof(dop));Step(30U);
  info=ExpectRaw(1U,dop,sizeof(dop),GNSS_PROTOCOL_UBX,1U);assert(info.sequence==0U && info.message_id==4U);
  info=ExpectRaw(0U,pvt,sizeof(pvt),GNSS_PROTOCOL_UBX,0U);assert(info.sequence==1U && info.received_ms==20U);
  assert(Ubx_BuildFrame(0x99U,0x88U,payload,11U,other,sizeof(other))==sizeof(other));Raw(other,sizeof(other));Step(40U);
  info=ExpectRaw(2U,other,sizeof(other),GNSS_PROTOCOL_UBX,1U);assert(info.sequence==0U);
  Raw(other,sizeof(other)); /* getter不能代替Service消费已排队的下一帧。 */
  info=ExpectRaw(2U,other,sizeof(other),GNSS_PROTOCOL_UBX,1U);assert(input_size==sizeof(other) && info.sequence==0U);
}

/** @brief NMEA保留完整CRLF和实际坏校验文本；warm Init及自然回绕不擦原帧。 */
static void TestRawNmeaWarmAndWrap(void)
{
  static const uint8_t nmea[]="$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  uint8_t bad[sizeof(nmea)-1U],wire[8];
  Gnss_RawFrameInfo_t info;
  Cold();Raw(nmea,sizeof(nmea)-1U);Step(UINT32_MAX-9U);
  info=ExpectRaw(2U,nmea,sizeof(nmea)-1U,GNSS_PROTOCOL_NMEA,1U);
  assert(info.sequence==0U && info.received_ms==UINT32_MAX-9U && info.class_id==0U && info.message_id==0U);
  memcpy(bad,nmea,sizeof(bad));bad[sizeof(bad)-4U]='0';bad[sizeof(bad)-3U]='0';Raw(bad,sizeof(bad));Step(0U);
  info=ExpectRaw(2U,bad,sizeof(bad),GNSS_PROTOCOL_NMEA,0U);
  assert(info.sequence==1U && info.received_ms==0U && Diag().valid_nmea==1U);
  assert(Sensor_GNSS_Init()==GNSS_RESULT_OK);Sensor_GNSS_RequestReinit();Step(10U);
  info=ExpectRaw(2U,bad,sizeof(bad),GNSS_PROTOCOL_NMEA,0U);assert(info.sequence==1U && info.received_ms==0U);
  s_raw_slots[2].info.sequence=UINT32_MAX; /* 主机夹具模拟自然回绕前一帧，生产不写外部序号。 */
  assert(Ubx_BuildFrame(0x99U,1U,NULL,0U,wire,sizeof(wire))==8U);Raw(wire,sizeof(wire));Step(20U);
  info=ExpectRaw(2U,wire,sizeof(wire),GNSS_PROTOCOL_UBX,1U);assert(info.sequence==0U && info.received_ms==20U);
}

/** @brief 恰好384B载荷可导出完整392B，超长UBX或NMEA均不得截断替换已有采样。 */
static void TestRawCapacityAndOversize(void)
{
  uint8_t payload[385],wire[393],previous[GNSS_RAW_MAX_BYTES],long_line[180];
  Gnss_RawFrameInfo_t info;
  unsigned j;
  Cold();for(j=0U;j<sizeof(payload);j++) { payload[j]=(uint8_t)j; }
  assert(Ubx_BuildFrame(0x99U,2U,payload,384U,wire,sizeof(wire))==GNSS_RAW_MAX_BYTES);
  memcpy(previous,wire,sizeof(previous));Raw(wire,GNSS_RAW_MAX_BYTES);Step(0U);
  info=ExpectRaw(2U,previous,sizeof(previous),GNSS_PROTOCOL_UBX,1U);assert(info.sequence==0U);
  assert(Ubx_BuildFrame(0x99U,2U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));
  Raw(wire,sizeof(wire));Step(10U);info=ExpectRaw(2U,previous,sizeof(previous),GNSS_PROTOCOL_UBX,1U);assert(info.sequence==0U);
  wire[sizeof(wire)-1U]^=1U;Raw(wire,sizeof(wire));Step(20U);
  info=ExpectRaw(2U,previous,sizeof(previous),GNSS_PROTOCOL_UBX,1U);assert(info.sequence==0U);
  memset(long_line,'1',sizeof(long_line));long_line[0]='$';long_line[sizeof(long_line)-2U]='\r';long_line[sizeof(long_line)-1U]='\n';
  Raw(long_line,sizeof(long_line));Step(30U);
  info=ExpectRaw(2U,previous,sizeof(previous),GNSS_PROTOCOL_UBX,1U);assert(info.sequence==0U);
}

/** @brief 完整UBX回调独立于Debug和NMEA，保留坏CRC，注销及warm Init不改变正常Raw/测量。 */
static void TestUbxDiagnosticCallback(void)
{
  uint8_t payload[UBX_PAYLOAD_CAPACITY+1U],wire[UBX_PAYLOAD_CAPACITY+UBX_FRAME_OVERHEAD+1U];
  uint16_t length;
  uint32_t before;
  Gnss_Snapshot_t snapshot;
  Gnss_RawFrameInfo_t info;
  memset(payload,0,sizeof(payload));payload[23]=7U;
  length=Ubx_BuildFrame(1U,7U,payload,92U,wire,sizeof(wire));assert(length==100U);
  s_ubx_callback_count=0U;s_nmea_callback_count=0U;
  Gnss_SetUbxDiagnosticCallback(NULL);Gnss_SetDiagnosticCallback(ObserveNmea);
  Cold();Raw(wire,length);Step(0U);assert(s_ubx_callback_count==0U && s_nmea_callback_count==0U);
  info=ExpectRaw(0U,wire,length,GNSS_PROTOCOL_UBX,1U);assert(info.sequence==0U);

  Gnss_SetUbxDiagnosticCallback(ObserveUbx);Cold();Raw(wire,length);Step(0U);
  assert(s_ubx_callback_count==1U && s_observed_ms==0U && s_observed_length==length && s_observed_checksum==1U);
  assert(s_observed_class==1U && s_observed_id==7U && !memcmp(s_observed_ubx,wire,length));
  assert(Sensor_GNSS_Init()==GNSS_RESULT_OK);Raw(wire,length);Step(10U);
  assert(s_ubx_callback_count==2U); /* warm Init保留调度前注册。 */
  wire[length-2U]^=0x55U;wire[length-1U]=0xB5U;Raw(wire,length);Step(20U);
  assert(s_ubx_callback_count==3U && s_observed_checksum==0U && !memcmp(s_observed_ubx,wire,length));
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_FIX && snapshot.status_update_ms==10U);
  assert(Diag().valid_ubx==2U && Diag().checksum_errors==1U && s_nmea_callback_count==0U);

  length=Ubx_BuildFrame(1U,4U,payload,18U,wire,sizeof(wire));Raw(wire,length);Step(30U);
  assert(s_ubx_callback_count==4U && s_observed_id==4U && s_observed_checksum==1U);
  Sentence("GPGSV,1,1,08",40U);assert(s_nmea_callback_count==1U && s_ubx_callback_count==4U);
  length=Ubx_BuildFrame(0x99U,0x88U,NULL,0U,wire,sizeof(wire));Raw(wire,length);Step(50U);
  assert(s_ubx_callback_count==5U && s_observed_class==0x99U && s_observed_id==0x88U);
  length=Ubx_BuildFrame(0x99U,2U,payload,sizeof(payload),wire,sizeof(wire));Raw(wire,length);Step(60U);
  while(input_size!=0U) { Step(60U); }
  assert(s_ubx_callback_count==5U && s_nmea_callback_count==1U); /* 超长不截断提交。 */

  before=Diag().valid_ubx;Gnss_SetUbxDiagnosticCallback(NULL);
  length=Ubx_BuildFrame(1U,7U,payload,92U,wire,sizeof(wire));Raw(wire,length);Step(70U);
  assert(s_ubx_callback_count==5U && Diag().valid_ubx==before+1U);
  info=ExpectRaw(0U,wire,length,GNSS_PROTOCOL_UBX,1U);assert(info.received_ms==70U);
  (void)Sensor_GNSS_CopySnapshot(&snapshot);assert(snapshot.status_update_ms==70U);
  Gnss_SetDiagnosticCallback(NULL);
}

int main(void)
{
  assert(GnssBringupRegressionMain()==0);
  TestRawParameters();TestRawUbxIntegrityAndSlots();TestRawNmeaWarmAndWrap();TestRawCapacityAndOversize();
  TestUbxDiagnosticCallback();
  printf("GNSS raw capture passed: original wire/CRC,3slots,capacity,warm,0ms/wrap; static buffers=%luB\n",
    (unsigned long)(sizeof(s_raw_slots)+sizeof(s_raw_work)));
  return 0;
}
