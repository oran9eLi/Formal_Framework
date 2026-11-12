/**
 * @file test_gnss_satellites.c
 * @brief 验证真实GNSS驱动的NAV-SAT独立事实、合法零值、固定分页与失败不写输出。
 * @details 使用主机字节向量，不访问设备；帧校验、结构解析及发布均运行生产实现。
 */
#define main GnssBringupRegressionMain
#include "test_gnss_bringup.c"
#undef main

static uint8_t sat_payload[1016], sat_wire[1024];

/** @brief 构造标准NAV-SAT载荷，星座和同号卫星交替，用原used位区分解算参与。 */
static void PrepareSat(uint8_t count)
{
  uint16_t i;
  memset(sat_payload,0,sizeof(sat_payload));
  Gnss_Put32(sat_payload,123456U);sat_payload[4]=1U;sat_payload[5]=count;
  for(i=0U;i<count;i++) {
    uint8_t *r=sat_payload+8U+12U*i;
    r[0]=(i&1U)?3U:0U;r[1]=(uint8_t)(i/2U+1U);r[2]=(uint8_t)(30U+i%20U);
    r[3]=(uint8_t)(i%90U);Gnss_Put16(r+4U,(uint16_t)(i*5U%361U));
    Gnss_Put32(r+8U,(i&1U)?0x0DU:2U);
  }
}

/** @brief 通过真实512B每拍预算投喂完整wire，返回完整帧实际消费时刻。 */
static uint32_t SendPreparedSat(uint16_t payload_length,uint8_t bad_checksum,uint32_t now)
{
  uint16_t size=Ubx_BuildFrame(1U,0x35U,sat_payload,payload_length,sat_wire,sizeof(sat_wire));
  assert(size);if(bad_checksum){sat_wire[size-1U]^=1U;}
  Raw(sat_wire,size);
  do { Step(now);if(input_size){now+=10U;} } while(input_size);
  return now;
}

/** @brief 未发布/参数错误/超过已存页均不改输出；空卫星报告page0合法。 */
static void TestSatelliteParametersAndZero(void)
{
  Gnss_SatellitePage_t page,saved;
  uint16_t queued;
  Cold();memset(&saved,0xA5,sizeof(saved));page=saved;
  assert(sizeof(Gnss_Satellite_t)==12U && sizeof(Gnss_SatellitePage_t)<=128U);
  assert(Sensor_GNSS_CopySatellitePage(0U,NULL)==GNSS_RESULT_INVALID_PARAM);
  assert(Sensor_GNSS_CopySatellitePage(13U,&page)==GNSS_RESULT_INVALID_PARAM);
  assert(Sensor_GNSS_CopySatellitePage(0U,&page)==GNSS_RESULT_NO_DATA && !memcmp(&page,&saved,sizeof(page)));
  PrepareSat(0U);SendPreparedSat(8U,0U,0U);
  queued=input_size;
  assert(Sensor_GNSS_CopySatellitePage(0U,&page)==GNSS_RESULT_OK && input_size==queued);
  assert(page.available && page.sequence==1U && page.received_ms==0U && page.itow_ms==123456U);
  assert(page.total==0U && page.stored==0U && page.count==0U && !page.truncated && !page.used_count);
  page=saved;assert(Sensor_GNSS_CopySatellitePage(1U,&page)==GNSS_RESULT_INVALID_PARAM && !memcmp(&page,&saved,sizeof(page)));
}

/** @brief 保留同号不同星座身份、原cno零和used位；无定位也能读取真实卫星表。 */
static void TestSatelliteFactsAndAngles(void)
{
  Gnss_SatellitePage_t page;
  Gnss_Snapshot_t nav;
  PrepareSat(5U);sat_payload[10]=0U;sat_payload[11]=0U;Gnss_Put16(sat_payload+12U,0U);
  sat_payload[11U+12U]=127U;
  Gnss_Put16(sat_payload+12U+24U,361U);
  sat_payload[11U+36U]=(uint8_t)(256U-90U);Gnss_Put16(sat_payload+12U+36U,360U);
  sat_payload[11U+48U]=(uint8_t)(256U-91U);
  Cold();SendPreparedSat(68U,0U,20U);
  assert(Sensor_GNSS_CopySatellitePage(0U,&page)==GNSS_RESULT_OK);
  assert(page.count==5U && page.used_count==2U);
  assert(page.entries[0].gnss_id==0U && page.entries[1].gnss_id==3U && page.entries[0].sv_id==page.entries[1].sv_id);
  assert(page.entries[0].cno_dbhz==0U && page.entries[0].quality==2U && !page.entries[0].used);
  assert(page.entries[0].valid_mask==(GNSS_SAT_VALID_ELEVATION|GNSS_SAT_VALID_AZIMUTH));
  assert(page.entries[1].valid_mask==0U && page.entries[1].used && page.entries[1].quality==5U);
  assert(page.entries[2].valid_mask==GNSS_SAT_VALID_ELEVATION);
  assert(page.entries[3].valid_mask==(GNSS_SAT_VALID_ELEVATION|GNSS_SAT_VALID_AZIMUTH) && page.entries[3].elevation_deg==-90 && page.entries[3].azimuth_deg==360);
  assert(page.entries[4].valid_mask==0U);
  assert(Sensor_GNSS_CopySnapshot(&nav)==GNSS_RESULT_NO_FIX && nav.valid_mask==0U && !nav.fix_valid && nav.last_rx_ms==20U);
  assert(!nav.position_update_ms && !nav.time_update_ms && !nav.status_update_ms);
  assert(Diag().last_frame_ms==20U && Diag().valid_ubx==1U);
}

/** @brief 坏校验、版本和长度不匹配不得替换已发布表或续期；warm恢复保留原时刻。 */
static void TestSatelliteRejectAndLifecycle(void)
{
  Gnss_SatellitePage_t before,after;
  uint32_t snapshot_sequence;
  Cold();PrepareSat(1U);SendPreparedSat(20U,0U,100U);
  assert(Sensor_GNSS_CopySatellitePage(0U,&before)==GNSS_RESULT_OK);
  snapshot_sequence=s_snapshot.rx_sequence;
  SendPreparedSat(20U,1U,200U);
  sat_payload[4]=2U;SendPreparedSat(20U,0U,300U);
  sat_payload[4]=1U;sat_payload[5]=2U;SendPreparedSat(20U,0U,400U);
  sat_payload[5]=1U;SendPreparedSat(19U,0U,500U);
  assert(Sensor_GNSS_CopySatellitePage(0U,&after)==GNSS_RESULT_OK && !memcmp(&before,&after,sizeof(after)));
  assert(s_snapshot.rx_sequence==snapshot_sequence && Diag().checksum_errors==1U);
  assert(Sensor_GNSS_Init()==GNSS_RESULT_OK);
  assert(Sensor_GNSS_CopySatellitePage(0U,&after)==GNSS_RESULT_OK && !memcmp(&before,&after,sizeof(after)));
  after.entries[0].sv_id=255U;
  assert(Sensor_GNSS_CopySatellitePage(0U,&after)==GNSS_RESULT_OK && after.entries[0].sv_id==before.entries[0].sv_id);
  s_satellite_published.sequence=UINT32_MAX;PrepareSat(1U);SendPreparedSat(20U,0U,0U);
  assert(Sensor_GNSS_CopySatellitePage(0U,&after)==GNSS_RESULT_OK && after.sequence==0U && after.received_ms==0U && after.available);
}

/** @brief 完整64星跨512B接收预算、末页边界、超过容量截断与全报告used计数。 */
static void TestSatelliteLargeAndPaging(void)
{
  Gnss_SatellitePage_t page;
  uint8_t i;
  uint32_t last;
  Cold();PrepareSat(64U);last=SendPreparedSat(776U,0U,1000U);
  assert(last==1010U);
  for(i=0U;i<13U;i++) {
    assert(Sensor_GNSS_CopySatellitePage(i,&page)==GNSS_RESULT_OK);
    assert(page.total==64U && page.stored==64U && !page.truncated && page.received_ms==1010U && page.sequence==1U);
    assert(page.page==i && page.count==(i==12U?4U:5U) && page.used_count==32U);
    assert(page.entries[0].sv_id==(uint8_t)(i*5U/2U+1U));
    if(i==12U) { Gnss_Satellite_t zero;memset(&zero,0,sizeof(zero));assert(!memcmp(&zero,&page.entries[4],sizeof(zero))); }
  }
  PrepareSat(84U);last=SendPreparedSat(1016U,0U,2000U);
  assert(Sensor_GNSS_CopySatellitePage(12U,&page)==GNSS_RESULT_OK);
  assert(page.total==84U && page.stored==64U && page.truncated && page.used_count==42U && page.count==4U && page.received_ms==last);
}

int main(void)
{
  TestSatelliteParametersAndZero();TestSatelliteFactsAndAngles();TestSatelliteRejectAndLifecycle();TestSatelliteLargeAndPaging();
  puts("GNSS satellite driver: PASS (independent NAV-SAT facts, zero/invalid angles, 64-star pages, truncation, checksum and lifecycle)");
  return 0;
}
