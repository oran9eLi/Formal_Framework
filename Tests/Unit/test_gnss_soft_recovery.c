/**
 * @file test_gnss_soft_recovery.c
 * @brief 真实GNSS BSP/Driver的过期软恢复、新帧/半帧保护及链路流水号回归。
 * @details 只替换HAL寄存器和输入，不操作实机；未知UBX只证明链路，不生成导航测量。
 */
#define main ExistingHardwareReconnectMain
#include "test_gnss_hardware_reconnect.c"
#undef main

/** @brief 注入标准CRC编码的UBX，随后由真实BSP和Driver完整消费。 */
static void SoftUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t length,
                    uint8_t bad_crc, uint32_t now_ms)
{
  uint8_t wire[128];
  uint16_t size=Ubx_BuildFrame(cls,id,payload,length,wire,sizeof(wire));
  assert(size != 0U);
  if(bad_crc) { wire[size-1U]^=1U; }
  Wire(wire,size); assert(Run(now_ms)==GNSS_RESULT_OK);
}

/** @brief 恢复请求不得擦除同拍的新完整帧，也不把无新数据的一拍当另一条接收。 */
static void TestNewFrameWinsOverSoftRequest(void)
{
  Gnss_Snapshot_t snapshot;
  NewBoard(); Run(0U);
  Sensor_GNSS_RequestReinit(); Wire(s_good_gga,sizeof(s_good_gga)-1U); Run(100U);
  snapshot=Snapshot(); assert(snapshot.fix_valid && snapshot.rx_sequence==1U && snapshot.last_rx_ms==100U);
  Sensor_GNSS_RequestReinit(); Run(110U);
  assert(Snapshot().rx_sequence==1U && Snapshot().last_rx_ms==100U && Snapshot().fix_valid);
  assert(Info().recoveries==0U && Info().rx_drop_bytes==0U);
}

/** @brief 半帧先到、请求后到及请求与第一半同拍三种交错均保留，直到真实整句完成。 */
static void TestPartialFrameWinsOverSoftRequest(void)
{
  unsigned mode;
  const uint16_t total=(uint16_t)(sizeof(s_good_gga)-1U), half=total/2U;
  for(mode=0U;mode<3U;++mode) {
    NewBoard(); Run(0U);
    if(mode==2U) { Sensor_GNSS_RequestReinit(); }
    Wire(s_good_gga,half); Run(100U); assert(s_line_pos==half && !Snapshot().rx_sequence);
    if(mode!=2U) { Sensor_GNSS_RequestReinit(); }
    if(mode==1U) { Run(110U); assert(s_line_pos==half); }
    Wire(s_good_gga+half,(uint16_t)(total-half)); Run(120U);
    assert(Snapshot().fix_valid && Snapshot().rx_sequence==1U && Snapshot().last_rx_ms==120U);
    assert(Info().recoveries==0U && Info().rx_drop_bytes==0U);
  }
}

/** @brief 自动请求保留新PVT优先期，显式Init仍清测量并允许GGA接替，序号不会回退。 */
static void TestExplicitInitAndMonotonicSequence(void)
{
  uint8_t pvt[92]={0U};
  Gnss_Snapshot_t snapshot;
  pvt[20]=2U; pvt[21]=1U; pvt[23]=8U;
  NewBoard(); SoftUbx(1U,7U,pvt,sizeof(pvt),0U,100U);
  snapshot=Snapshot(); assert(snapshot.fix_valid && snapshot.position_source==GNSS_SOURCE_UBX_PVT && snapshot.rx_sequence==1U);
  Sensor_GNSS_RequestReinit(); Wire(s_good_gga,sizeof(s_good_gga)-1U); Run(110U);
  snapshot=Snapshot(); assert(snapshot.position_source==GNSS_SOURCE_UBX_PVT && snapshot.position_update_ms==100U && snapshot.rx_sequence==2U);
  assert(Sensor_GNSS_Init()==GNSS_RESULT_OK);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_DATA && snapshot.rx_sequence==0U && !snapshot.valid_mask);
  Wire(s_good_gga,sizeof(s_good_gga)-1U); Run(120U);
  snapshot=Snapshot(); assert(snapshot.position_source==GNSS_SOURCE_NMEA_GGA && snapshot.rx_sequence==3U && !snapshot.fix_dimension);
  assert(Sensor_GNSS_Init()==GNSS_RESULT_OK);
  s_rx_sequence_counter=UINT32_MAX-1U;
  Wire(s_good_gga,sizeof(s_good_gga)-1U); Run(130U); assert(Snapshot().rx_sequence==UINT32_MAX);
  assert(Sensor_GNSS_Init()==GNSS_RESULT_OK);
  Wire(s_good_gga,sizeof(s_good_gga)-1U); Run(140U); assert(Snapshot().rx_sequence==1U);
}

/** @brief 真正2秒输入静默才清旧快照，6秒硬件自愈和下一新帧仍独立生效。 */
static void TestRealSilenceStillRecovers(void)
{
  Gnss_Snapshot_t snapshot;
  NewBoard(); Run(0U); Wire(s_good_gga,sizeof(s_good_gga)-1U); Run(100U);
  Sensor_GNSS_RequestReinit(); Run(2099U); assert(Snapshot().rx_sequence==1U);
  Sensor_GNSS_RequestReinit(); Run(2100U);
  assert(Sensor_GNSS_CopySnapshot(&snapshot)==GNSS_RESULT_NO_DATA && !snapshot.rx_sequence);
  assert(Info().recoveries==0U);
  Run(6100U); assert(s_silence_recovery_pending && Info().recoveries==0U);
  Run(6110U); assert(Info().recoveries==1U);
  Wire(s_good_gga,sizeof(s_good_gga)-1U); Run(6200U);
  assert(Snapshot().rx_sequence==2U && Snapshot().last_rx_ms==6200U && Snapshot().fix_valid);
}

/** @brief CRC正确的未知/结构不支持UBX只刷新链路；坏CRC不续链路，已支持帧不能双计数。 */
static void TestUnknownUbxAndCrcSeparation(void)
{
  uint8_t payload[4]={1U,2U,3U,4U};
  uint32_t position_ms,time_ms,status_ms;
  Gnss_Snapshot_t snapshot;
  NewBoard(); SoftUbx(0x09U,0x99U,payload,sizeof(payload),0U,0U);
  snapshot=Snapshot(); assert(snapshot.rx_sequence==1U && snapshot.last_rx_ms==0U && !snapshot.valid_mask && !snapshot.fix_valid);
  assert(s_diag_work.frame_seen && s_diag_work.last_frame_ms==0U && s_diag_work.protocol==GNSS_PROTOCOL_UBX);
  assert(!s_diag_work.version_seen && !s_diag_work.config_confirmed);
  SoftUbx(1U,7U,payload,sizeof(payload),0U,10U);
  snapshot=Snapshot(); assert(snapshot.rx_sequence==2U && snapshot.last_rx_ms==10U && !snapshot.valid_mask);
  SoftUbx(0x09U,0x99U,payload,sizeof(payload),1U,20U);
  assert(Snapshot().rx_sequence==2U && Snapshot().last_rx_ms==10U && s_diag_work.checksum_errors==1U);
  Wire(s_good_gga,sizeof(s_good_gga)-1U); Run(30U);
  snapshot=Snapshot(); assert(snapshot.rx_sequence==3U && snapshot.fix_valid);
  position_ms=snapshot.position_update_ms; time_ms=snapshot.time_update_ms; status_ms=snapshot.status_update_ms;
  SoftUbx(0x09U,0x99U,payload,sizeof(payload),0U,40U);
  snapshot=Snapshot(); assert(snapshot.rx_sequence==4U && snapshot.position_update_ms==position_ms &&
                             snapshot.time_update_ms==time_ms && snapshot.status_update_ms==status_ms);
  assert(s_diag_work.valid_ubx==3U && s_diag_work.valid_nmea==1U);
}

/** @brief 0ms和毫秒回绕不能被当作未收数；原始坏数据也不冒充合法帧给链路续期。 */
static void TestZeroEpochWrapAndRawOnly(void)
{
  uint8_t noise=0x55U;
  NewBoard(); Wire(s_good_gga,sizeof(s_good_gga)-1U); Run(0U);
  Sensor_GNSS_RequestReinit(); Run(1999U); assert(Snapshot().rx_sequence==1U && Snapshot().last_rx_ms==0U);
  Sensor_GNSS_RequestReinit(); Run(2000U); assert(!Snapshot().rx_sequence);
  NewBoard(); Wire(s_good_gga,sizeof(s_good_gga)-1U); Run(UINT32_MAX-99U);
  Sensor_GNSS_RequestReinit(); Run(100U); assert(Snapshot().rx_sequence==1U && Snapshot().last_rx_ms==UINT32_MAX-99U);
  Wire(&noise,1U); Sensor_GNSS_RequestReinit(); Run(1900U);
  assert(Snapshot().rx_sequence==1U && Snapshot().last_rx_ms==UINT32_MAX-99U);
  Sensor_GNSS_RequestReinit(); Run(3900U); assert(!Snapshot().rx_sequence);
}

int main(void)
{
  TestNewFrameWinsOverSoftRequest(); TestPartialFrameWinsOverSoftRequest();
  TestExplicitInitAndMonotonicSequence(); TestRealSilenceStillRecovers();
  TestUnknownUbxAndCrcSeparation(); TestZeroEpochWrapAndRawOnly();
  assert(!primask);
  puts("GNSS soft recovery: new-frame/partial protection, real quiet recovery, monotonic warm RX, unknown UBX/CRC separation, zero/wrap PASS");
  return 0;
}
