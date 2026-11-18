/**
 * @file test_gnss_unified_facts.c
 * @brief 通过真实双协议驱动验证统一来源、独立DOP/计数时效和旧数据撤权边界。
 * @details 复用已有BSP字节流夹具，不改生产发送/采集路径，不使用任务或堆。
 */
#if defined(TEST_GNSS_UNIFIED_ADAPTER)
#define main LegacyGnssStatusDomainMain
#include "test_gnss_status_domain.c"
#undef main

/** @brief 使用真实Adapter逐项核对追加元数据，不以结构体强转或memcpy跨层。 */
static void TestUnifiedAdapterMapping(void)
{
  Px4Lite_SensorGnss_t out;
  unsigned i;
  Reset();s_clock=100U;s_driver_source.rx_sequence=1U;s_driver_source.last_rx_ms=100U;
  s_driver_source.valid_mask=GNSS_VALID_NAV_STATUS;s_driver_source.status_update_ms=100U;
  s_driver_source.position_source=GNSS_SOURCE_UBX_PVT;s_driver_source.velocity_source=GNSS_SOURCE_NMEA_RMC;
  s_driver_source.time_source=GNSS_SOURCE_NMEA_RMC;s_driver_source.altitude_source=GNSS_SOURCE_NMEA_GGA;
  s_driver_source.status_source=GNSS_SOURCE_NMEA_GGA;s_driver_source.raw_status_type=8U;
  s_driver_source.dimension_source=GNSS_SOURCE_NMEA_GSA;s_driver_source.dimension_update_ms=77U;
  s_driver_source.dop_present_mask=0x7FU;s_driver_source.dop_itow_known_mask=0x55U;
  s_driver_source.dop_solution_mask=0x2AU;
  for(i=0U;i<7U;i++) {
    s_driver_source.dop_source[i]=(i&1U)?GNSS_SOURCE_NMEA_GSA:GNSS_SOURCE_UBX_DOP;
    s_driver_source.dop_x100[i]=(uint16_t)(101U+i);s_driver_source.dop_update_ms[i]=10U+i;s_driver_source.dop_itow_ms[i]=1000U+i;
  }
  s_driver_source.counts_present_mask=0x0FU;
  for(i=0U;i<4U;i++) { s_driver_source.counts_source[i]=i<2U?GNSS_SOURCE_NMEA_GSV:GNSS_SOURCE_NMEA_GSA;s_driver_source.counts_update_ms[i]=40U+i; }
  assert(Px4Lite_GnssRead(&out)==PX4LITE_OK);
  assert(out.position_source==PX4LITE_GNSS_SOURCE_UBX_PVT && out.velocity_source==PX4LITE_GNSS_SOURCE_NMEA_RMC);
  assert(out.time_source==PX4LITE_GNSS_SOURCE_NMEA_RMC && out.altitude_source==PX4LITE_GNSS_SOURCE_NMEA_GGA);
  assert(out.status_source==PX4LITE_GNSS_SOURCE_NMEA_GGA && out.raw_status_type==8U && out.dimension_source==PX4LITE_GNSS_SOURCE_NMEA_GSA && out.dimension_update_ms==77U);
  assert(out.dop_present_mask==0x7FU && out.dop_itow_known_mask==0x55U && out.dop_solution_mask==0x2AU && out.counts_present_mask==0x0FU);
  for(i=0U;i<7U;i++) { assert(out.dop_source[i]==s_driver_source.dop_source[i] && out.dop_x100[i]==101U+i && out.dop_update_ms[i]==10U+i && out.dop_itow_ms[i]==1000U+i); }
  for(i=0U;i<4U;i++) { assert(out.counts_source[i]==s_driver_source.counts_source[i] && out.counts_update_ms[i]==40U+i); }
  assert(out.header.sample_time_ms==100U);
  s_driver_source.rx_sequence++;s_driver_source.last_rx_ms=200U;s_clock=200U;
  assert(Px4Lite_GnssRead(&out)==PX4LITE_OK && out.dop_update_ms[0]==10U && out.counts_update_ms[0]==40U && out.header.sample_time_ms==100U);
  s_driver_source.rx_sequence++;s_driver_source.position_source=255U;
  assert(Px4Lite_GnssRead(&out)==PX4LITE_OK && out.position_source==PX4LITE_GNSS_SOURCE_NONE);
}

int main(void)
{ assert(LegacyGnssStatusDomainMain()==0);TestUnifiedAdapterMapping();puts("GNSS unified Adapter mapping passed");return 0; }
#else
#define main LegacyGnssBringupMain
#include "test_gnss_bringup.c"
#undef main
#include "px4lite_types.h"
#include <stddef.h>

/** @brief 复制真实驱动快照；NO_FIX与OK均有设备事实，不把无定位当作无数据。 */
static Gnss_Snapshot_t Facts(void)
{
  Gnss_Snapshot_t out;
  Gnss_Result_t result=Sensor_GNSS_CopySnapshot(&out);
  assert(result==GNSS_RESULT_OK || result==GNSS_RESULT_NO_FIX);
  return out;
}

/** @brief 按12个固定ID位置生成完整GSA，避免测试向量逗号个数歧义。 */
static void Gsa(const char *talker,uint8_t dimension,const char * const ids[12],const char *pdop,const char *hdop,const char *vdop,uint32_t now)
{
  char body[120];unsigned i;int n=snprintf(body,sizeof(body),"%sGSA,A,%u",talker,(unsigned)dimension);
  assert(n>0);
  for(i=0U;i<12U;i++) { int k=snprintf(body+n,sizeof(body)-(unsigned)n,",%s",ids[i]?ids[i]:"");assert(k>0);n+=k;assert((unsigned)n<sizeof(body)); }
  n+=snprintf(body+n,sizeof(body)-(unsigned)n,",%s,%s,%s",pdop,hdop,vdop);
  assert((unsigned)n<sizeof(body));Sentence(body,now);
}

/** @brief 生成独立NAV-DOP输入，七项值按G/P/T/V/H/N/E顺序。 */
static void Dop(const uint16_t values[7],uint32_t itow,uint32_t now)
{
  uint8_t payload[18];unsigned i;memset(payload,0,sizeof(payload));Gnss_Put32(payload,itow);
  for(i=0U;i<7U;i++) { Gnss_Put16(payload+4U+2U*i,values[i]); }
  Frame(1U,4U,payload,sizeof(payload),now);
}

/** @brief 旧字段前缀不变；来源与索引两层独立命名且编码一致。 */
static void TestLayoutAndCodes(void)
{
  assert(offsetof(Gnss_Snapshot_t,position_source)==offsetof(Gnss_Snapshot_t,status_update_ms)+4U);
  assert(offsetof(Px4Lite_SensorGnss_t,position_source)==offsetof(Px4Lite_SensorGnss_t,status_update_ms)+4U);
  assert(GNSS_SOURCE_UBX_PVT==PX4LITE_GNSS_SOURCE_UBX_PVT && GNSS_SOURCE_UBX_DOP==PX4LITE_GNSS_SOURCE_UBX_DOP);
  assert(GNSS_DOP_H==4U && GNSS_COUNT_BDS_USED==3U && GNSS_DOP_COUNT==7U && GNSS_COUNTS_COUNT==4U);
}

/** @brief NMEA五域来源、质量限制、M单位与长数字拒绝，不能用旧有效值续期失败域。 */
static void TestBaseSourcesAndStrictNmea(void)
{
  Gnss_Snapshot_t s;unsigned i;
  static const char * const rejected[]={"3","6","7","8"};
  char body[128];
  Cold();Sentence("GPGGA,000000.00,2230.0000,N,11330.0000,E,1,08,1.23,25.0,M,0.0,M,,",0U);s=Facts();
  assert(s.position_source==GNSS_SOURCE_NMEA_GGA && s.altitude_source==GNSS_SOURCE_NMEA_GGA);
  assert(s.status_source==GNSS_SOURCE_NMEA_GGA && s.raw_status_type==1U && s.position_update_ms==0U);
  assert(s.dop_present_mask==(1U<<GNSS_DOP_H) && s.dop_x100[GNSS_DOP_H]==123U && s.dop_update_ms[GNSS_DOP_H]==0U);
  assert(s.dop_source[GNSS_DOP_H]==GNSS_SOURCE_NMEA_GGA && !s.dop_itow_known_mask);
  assert(s.dop_solution_mask==(1U<<GNSS_DOP_H));
  Sentence("GPRMC,000001.00,A,2230.0000,N,11330.0000,E,10.0,90.0,061126,,,A",10U);s=Facts();
  assert(s.velocity_source==GNSS_SOURCE_NMEA_RMC && s.time_source==GNSS_SOURCE_NMEA_RMC);
  assert(s.velocity_update_ms==10U && s.time_update_ms==10U && s.position_update_ms==0U);
  Sentence("GPGGA,000002.00,2230.0000,N,11330.0000,E,1,08,1.0,26.0,F,0.0,M,,",20U);s=Facts();
  assert((s.valid_mask&GNSS_VALID_POSITION) && !(s.valid_mask&GNSS_VALID_ALTITUDE));
  assert(s.altitude_update_ms==0U && s.altitude_source==GNSS_SOURCE_NMEA_GGA);
  for(i=0U;i<4U;i++) {
    (void)snprintf(body,sizeof(body),"GPGGA,000002.00,2230.0000,N,11330.0000,E,%s,08,1.0,26.0,M,0.0,M,,",rejected[i]);
    Sentence(body,30U+i);s=Facts();
    assert(!(s.valid_mask&(GNSS_VALID_POSITION|GNSS_VALID_ALTITUDE|GNSS_VALID_VELOCITY)));
    assert((s.valid_mask&GNSS_VALID_NAV_STATUS) && s.raw_status_type==(uint8_t)(rejected[i][0]-'0'));
    assert(!(s.dop_solution_mask&(1U<<GNSS_DOP_H)));
    assert(s.position_update_ms==20U);
  }
  Sentence("GPGGA,000002.00,0000000000000000000000000000000000000002230.0,N,11330.0,E,1,08,1.0,1.0,M,,M,,",40U);s=Facts();
  assert(!(s.valid_mask&GNSS_VALID_POSITION) && s.position_update_ms==20U && s.status_update_ms==40U && s.raw_status_type==1U);
  Sentence("GPRMC,000003.00,A,,,,,0000000000000000000000000000000000000001,90.0,061126,,,A",50U);s=Facts();
  assert(!(s.valid_mask&GNSS_VALID_VELOCITY) && s.velocity_update_ms==10U && (s.valid_mask&GNSS_VALID_TIME) && s.time_update_ms==50U);
  for(i=0U;i<3U;i++) {
    static const uint8_t allowed[3]={2U,4U,5U};
    (void)snprintf(body,sizeof(body),"GPGGA,000004.00,2230.0000,N,11330.0000,E,%u,08,1.0,26.0,M,0.0,M,,",(unsigned)allowed[i]);
    Sentence(body,60U+i);s=Facts();assert((s.valid_mask&GNSS_VALID_POSITION) && s.fix_quality==allowed[i]);
  }
}

/** @brief PVT给五域来源和pDOP；同历元DOP补全七项，ACK/版本不续元数据或测量时间。 */
static void TestUbxFactsAndDopEpoch(void)
{
  static const uint16_t values[7]={111U,222U,333U,444U,555U,666U,777U};
  uint16_t invalid[7]={0U,UINT16_MAX,333U,444U,555U,666U,777U};
  Gnss_Snapshot_t s;unsigned i;
  Cold();Pvt(3U,1U,0U,1234U,0U);s=Facts();
  assert(s.position_source==GNSS_SOURCE_UBX_PVT && s.velocity_source==GNSS_SOURCE_UBX_PVT && s.time_source==GNSS_SOURCE_UBX_PVT);
  assert(s.altitude_source==GNSS_SOURCE_UBX_PVT && s.status_source==GNSS_SOURCE_UBX_PVT && s.raw_status_type==3U);
  assert(s.dimension_source==GNSS_SOURCE_UBX_PVT && s.dimension_update_ms==0U);
  assert(s.dop_present_mask==(1U<<GNSS_DOP_P) && s.dop_x100[GNSS_DOP_P]==9999U && s.dop_itow_ms[GNSS_DOP_P]==1234U);
  assert(s.dop_solution_mask==(1U<<GNSS_DOP_P));
  Dop(values,1234U,10U);s=Facts();assert(s.dop_present_mask==0x7FU && s.dop_itow_known_mask==0x7FU && s.dop_solution_mask==0x7FU && s.hdop_cm==555U);
  for(i=0U;i<7U;i++) { assert(s.dop_x100[i]==values[i] && s.dop_source[i]==GNSS_SOURCE_UBX_DOP && s.dop_update_ms[i]==10U && s.dop_itow_ms[i]==1234U); }
  Ack(0x8AU,1U,20U);Version(3410U,30U);s=Facts();
  assert(s.position_update_ms==0U && s.dimension_update_ms==0U && s.dop_update_ms[GNSS_DOP_H]==10U);
  Dop(invalid,999U,40U);s=Facts();assert(!(s.dop_present_mask&3U) && !(s.dop_itow_known_mask&3U));
  assert(s.dop_update_ms[GNSS_DOP_G]==10U && s.hdop_cm==555U); /* 新历元不匹配不覆盖旧legacy HDOP。 */
}

/** @brief 独立GSA/GSV在PVT保护期仍采集，按明确星座与唯一数字ID计数；不改PVT位置/维度。 */
static void TestMixedCountsAndProtection(void)
{
  static const char * const ids[12]={"01","1","02",NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
  static const char * const empty[12]={NULL};
  static const char * const bad[12]={"A1",NULL};
  Gnss_Snapshot_t s;
  Cold();Pvt(3U,1U,0U,1000U,0U);
  Gsa("GP",2U,ids,"1.20","0.80","1.50",10U);Sentence("GBGSV,2,1,08",20U);s=Facts();
  assert(s.fix_dimension==3U && s.dimension_source==GNSS_SOURCE_UBX_PVT && s.dimension_update_ms==0U && s.position_update_ms==0U);
  assert(s.gps_used_sats==2U && s.counts_source[GNSS_COUNT_GPS_USED]==GNSS_SOURCE_NMEA_GSA && s.counts_update_ms[GNSS_COUNT_GPS_USED]==10U);
  assert(s.bds_visible_sats==8U && s.counts_update_ms[GNSS_COUNT_BDS_VISIBLE]==20U);
  assert(s.dop_source[GNSS_DOP_H]==GNSS_SOURCE_NMEA_GSA && s.dop_x100[GNSS_DOP_H]==80U && !(s.dop_itow_known_mask&(1U<<GNSS_DOP_H)));
  Sentence("GBGSV,2,2,08",30U);s=Facts();assert(s.bds_visible_sats==8U); /* 多页不相加。 */
  Sentence("GNGSV,1,1,99",40U);Gsa("GN",3U,ids,"1.0","1.0","1.0",50U);s=Facts();
  assert(s.gps_used_sats==2U && s.counts_update_ms[GNSS_COUNT_GPS_USED]==10U && s.bds_visible_sats==8U);
  Gsa("BD",1U,empty,"99.99","99.99","99.99",60U);s=Facts();
  assert((s.counts_present_mask&(1U<<GNSS_COUNT_BDS_USED)) && s.bds_used_sats==0U && (s.valid_mask&GNSS_VALID_POSITION));
  assert(s.dop_x100[GNSS_DOP_P]==9999U && (s.dop_present_mask&(1U<<GNSS_DOP_P)) && !(s.dop_solution_mask&((1U<<GNSS_DOP_P)|(1U<<GNSS_DOP_H)|(1U<<GNSS_DOP_V))));
  Gsa("GP",3U,bad,"1.0","1.0","1.0",70U);Sentence("GBGSV,1,1,256",80U);s=Facts();
  assert(!(s.counts_present_mask&(1U<<GNSS_COUNT_GPS_USED)) && s.counts_update_ms[GNSS_COUNT_GPS_USED]==10U);
  assert(!(s.counts_present_mask&(1U<<GNSS_COUNT_BDS_VISIBLE)) && s.counts_update_ms[GNSS_COUNT_BDS_VISIBLE]==30U);
  Ack(0x8AU,1U,90U);s=Facts();assert(s.counts_update_ms[GNSS_COUNT_BDS_USED]==60U && s.dimension_update_ms==0U && s.dop_update_ms[GNSS_DOP_H]==70U);
  PvtStatus(0U,0U,2U,100U);Sentence("GPGGA,123519.00,2230.0000,N,11330.0000,E,1,08,1.0,25.0,M,0.0,M,,",110U);s=Facts();
  assert(!(s.valid_mask&GNSS_VALID_POSITION) && s.raw_status_type==0U && s.status_source==GNSS_SOURCE_UBX_PVT && s.status_update_ms==100U);
  assert(s.position_update_ms==0U && s.position_source==GNSS_SOURCE_UBX_PVT);
  assert(s.dop_source[GNSS_DOP_H]==GNSS_SOURCE_NMEA_GGA && s.dop_update_ms[GNSS_DOP_H]==110U && s.hdop_cm==0U);
}

/** @brief NMEA DOP坏值不续旧时刻，0ms/iTOW0与时钟回绕均独立可解释。 */
static void TestDopInvalidAndTimeWrap(void)
{
  static const char * const ids[12]={"01",NULL};
  static const uint16_t values[7]={101U,102U,103U,104U,105U,106U,107U};
  Gnss_Snapshot_t s;
  uint8_t payload[18],wire[26];
  Cold();Gsa("GP",3U,ids,"1.23","0.77","1.56",0U);s=Facts();
  assert(s.dimension_source==GNSS_SOURCE_NMEA_GSA && s.dimension_update_ms==0U && s.dop_x100[GNSS_DOP_P]==123U);
  Gsa("GP",3U,ids,"0","655.35","bad",10U);s=Facts();
  assert(!(s.dop_present_mask&((1U<<GNSS_DOP_P)|(1U<<GNSS_DOP_H)|(1U<<GNSS_DOP_V))));
  assert(s.dop_update_ms[GNSS_DOP_P]==0U);
  Gsa("GP",3U,ids,"00000000000000000000000000000000001.25","1.0","1.0",20U);s=Facts();
  assert(!(s.dop_present_mask&(1U<<GNSS_DOP_P)) && s.dop_update_ms[GNSS_DOP_P]==0U);
  Dop(values,0U,UINT32_MAX-9U);Dop(values,0U,0U);s=Facts();
  assert(s.dop_present_mask==0x7FU && s.dop_itow_known_mask==0x7FU && s.dop_itow_ms[0]==0U && s.dop_update_ms[0]==0U);
  Sentence("GPGSV,1,1,00",10U);s=Facts();
  assert((s.counts_present_mask&1U) && s.gps_visible_sats==0U && s.counts_update_ms[0]==10U);
  memset(payload,0,sizeof(payload));Gnss_Put16(payload+12U,999U);
  assert(Ubx_BuildFrame(1U,4U,payload,sizeof(payload),wire,sizeof(wire))==sizeof(wire));wire[25]^=1U;
  Raw(wire,sizeof(wire));Step(20U);s=Facts();
  assert(s.dop_update_ms[GNSS_DOP_H]==0U && s.dop_x100[GNSS_DOP_H]==105U && s.counts_update_ms[0]==10U);
}

/** @brief DOP先到后由同历元PVT补授权，保持全部原DOP时间/来源；NOFIX或历元变更撤销授权。 */
static void TestDopSolutionPairing(void)
{
  static const uint16_t values[7]={101U,102U,103U,104U,105U,106U,107U};
  Gnss_Snapshot_t s;unsigned i;
  Cold();Dop(values,42U,0U);s=Facts();assert(s.dop_present_mask==0x7FU && s.dop_solution_mask==0U);
  Pvt(3U,1U,0U,42U,10U);s=Facts();assert(s.dop_solution_mask==0x7FU);
  for(i=0U;i<7U;i++) { assert(s.dop_source[i]==GNSS_SOURCE_UBX_DOP && s.dop_update_ms[i]==0U && s.dop_x100[i]==values[i]); }
  Pvt(0U,0U,0U,42U,20U);s=Facts();assert(!(s.valid_mask&GNSS_VALID_POSITION) && s.dop_present_mask==0x7FU && s.dop_solution_mask==0U);
  for(i=0U;i<7U;i++) { assert(s.dop_update_ms[i]==0U && s.dop_source[i]==GNSS_SOURCE_UBX_DOP); }
  Dop(values,43U,30U);s=Facts();assert(s.dop_solution_mask==0U);
  Pvt(3U,1U,0U,43U,40U);s=Facts();assert(s.dop_solution_mask==0x7FU);
  for(i=0U;i<7U;i++) { assert(s.dop_update_ms[i]==30U && s.dop_source[i]==GNSS_SOURCE_UBX_DOP); }
  Pvt(3U,1U,0U,44U,50U);s=Facts();assert(s.dop_solution_mask==(1U<<GNSS_DOP_P));
  Pvt(0U,0U,0U,44U,60U);s=Facts();assert(s.dop_solution_mask==0U);
}

int main(void)
{
  TestLayoutAndCodes();TestBaseSourcesAndStrictNmea();TestUbxFactsAndDopEpoch();TestMixedCountsAndProtection();TestDopInvalidAndTimeWrap();TestDopSolutionPairing();
  printf("GNSS unified facts passed: sources,DOP/count independence,strictNMEA,PVTpriority; Driver=%luB Framework=%luB\n",(unsigned long)sizeof(Gnss_Snapshot_t),(unsigned long)sizeof(Px4Lite_SensorGnss_t));
  return 0;
}
#endif
