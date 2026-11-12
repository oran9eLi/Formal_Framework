/**
 * @file test_gnss_link_debug.c
 * @brief 真实Debug报告器及Framework转换的GNSS异常日志门控、长度和关闭路径回归。
 * @details 只注入已缓存的Driver/BSP事实，不运行传感器采集、总线或恢复。
 */
#define main ExistingRawDebugMain
#include "test_gnss_raw_debug.c"
#undef main

#if DEBUG_GNSS_MONITOR_ENABLE && PX4LITE_ENABLE_GNSS
static unsigned s_largest_link_line;
/** @brief 模拟新鲜诊断缓存；零点/有效位遵循生产转换，测量topic仍不可用。 */
static void Live(void)
{
  Reset();s_clock=1000U;s_link_driver_result=GNSS_RESULT_OK;s_link_transport_result=BSP_STATUS_OK;
  s_link_driver_source.measured_ms=s_clock;s_link_driver_source.service_count=100U;
  s_link_driver_source.rx_bytes=900U;s_link_driver_source.valid_nmea=1U;s_link_driver_source.valid_ubx=9U;
  s_link_driver_source.last_rx_ms=s_clock;s_link_driver_source.last_frame_ms=s_clock;
  s_link_driver_source.rx_seen=1U;s_link_driver_source.frame_seen=1U;
  s_link_driver_source.state=GNSS_LINK_RECEIVING;s_link_driver_source.protocol=GNSS_PROTOCOL_UBX;
  s_link_transport_source.initialized=1U;s_link_transport_source.baud_bps=115200U;
  s_link_transport_source.rx_bytes=1000U;s_link_transport_source.ndtr=24U;
  s_expected_destination=s_gnss_raw_frame.bytes;
}
/** @brief 一次门控命中恰好两行，前缀和串口256B上限留出真实DBG前缀及CRLF。 */
static void TwoLines(void)
{
  unsigned i;
  assert(s_lines==2U);
  assert(strncmp(s_output[0],"GNSS-LINK: ",11U)==0 && strncmp(s_output[1],"GNSS-IO: ",9U)==0);
  assert(strlen(s_output[0])+strlen("[DBG] ")+2U<256U);
  assert(strlen(s_output[1])+strlen("[DBG] ")+2U<256U);
  for(i=0U;i<2U;i++) {
    unsigned bytes=(unsigned)(strlen(s_output[i])+strlen("[DBG] ")+2U);
    if(bytes>s_largest_link_line) { s_largest_link_line=bytes; }
  }
  assert(s_driver_calls==0U);
}
/** @brief 正常字节/帧/服务计数增长及TX忙切换不触发重复报告，不因无定位topic抹事实。 */
static void TestFirstAndHealthyProgress(void)
{
  Live();DebugService_ReportGnssLink();TwoLines();
  assert(strstr(s_output[0],"av=1/1") && strstr(s_output[0],"rx=1000/900"));
  assert(strstr(s_output[0],"byte=1/0 frame=1/0"));
  s_lines=0U;s_clock=2000U;s_link_driver_source.measured_ms=s_clock;
  s_link_driver_source.last_rx_ms=s_clock;s_link_driver_source.last_frame_ms=s_clock;
  s_link_driver_source.service_count+=100U;s_link_driver_source.rx_bytes+=1000U;
  s_link_driver_source.valid_nmea++;s_link_driver_source.valid_ubx++;s_link_transport_source.rx_bytes+=1000U;
  s_link_transport_source.tx_busy=1U;DebugService_ReportGnssLink();assert(s_lines==0U);
  assert(s_link_driver_copies==2U && s_link_transport_copies==2U && !s_driver_calls);
}
/** @brief 阶段/协议/初始化/诊断复制返回与各错误计数变化均报告，重复不异常时静默。 */
static void TestTransitionsAndFaultCounters(void)
{
  unsigned kind;
  for(kind=0U;kind<12U;kind++) {
    Live();DebugService_ReportGnssLink();s_lines=0U;
    switch(kind) {
      case 0U:s_link_driver_source.state=GNSS_LINK_UNCONFIRMED;break;
      case 1U:s_link_driver_source.protocol=GNSS_PROTOCOL_NMEA;break;
      case 2U:s_link_transport_source.initialized=0U;break;
      case 3U:s_link_transport_source.uart_errors++;break;
      case 4U:s_link_transport_source.dma_errors++;break;
      case 5U:s_link_transport_source.rx_drop_bytes++;break;
      case 6U:s_link_transport_source.tx_failures++;break;
      case 7U:s_link_transport_source.recoveries++;break;
      case 8U:s_link_driver_source.checksum_errors++;break;
      case 9U:s_link_driver_source.framing_resets++;break;
      case 10U:s_link_driver_result=GNSS_RESULT_NO_DATA;break;
      default:s_link_transport_result=BSP_STATUS_ERROR;break;
    }
    DebugService_ReportGnssLink();TwoLines();s_lines=0U;
    DebugService_ReportGnssLink();assert(s_lines==0U);
  }
}
/** @brief 1500/2000ms为精确边界；只读次数本身不能给sensor或原始帧年龄续期。 */
static void TestStaleAndAgeBoundaries(void)
{
  Live();DebugService_ReportGnssLink();s_lines=0U;s_clock=2500U;
  DebugService_ReportGnssLink();assert(s_lines==0U);
  s_clock=2501U;DebugService_ReportGnssLink();TwoLines();assert(strstr(s_output[0],"age=1501"));
  s_lines=0U;DebugService_ReportGnssLink();TwoLines(); /* 持续异常每既有报告周期可见。 */
  Live();DebugService_ReportGnssLink();s_lines=0U;s_clock=3000U;s_link_driver_source.measured_ms=s_clock;
  DebugService_ReportGnssLink();assert(!s_lines);
  s_clock=3001U;s_link_driver_source.measured_ms=s_clock;
  DebugService_ReportGnssLink();TwoLines();assert(strstr(s_output[0],"frame=1/2001"));
  Live();s_clock=100U;s_link_driver_source.measured_ms=UINT32_MAX-100U;
  s_link_driver_source.last_rx_ms=UINT32_MAX-200U;s_link_driver_source.last_frame_ms=UINT32_MAX-300U;
  DebugService_ReportGnssLink();TwoLines();assert(strstr(s_output[0],"age=201") && strstr(s_output[0],"byte=1/301 frame=1/401"));
  Live();s_link_driver_source.rx_seen=0U;s_link_driver_source.frame_seen=0U;
  s_link_driver_source.last_rx_ms=0U;s_link_driver_source.last_frame_ms=0U;
  DebugService_ReportGnssLink();TwoLines();assert(strstr(s_output[0],"byte=0/0 frame=0/0"));
}
/** @brief 首次仅BSP事实也能报告；完全无事实时不伪造初始化/服务数据。 */
static void TestAvailabilityAndMaximumLength(void)
{
  Reset();DebugService_ReportGnssLink();assert(!s_lines);
  s_link_transport_result=BSP_STATUS_OK;s_link_transport_source.baud_bps=115200U;
  DebugService_ReportGnssLink();TwoLines();assert(strstr(s_output[0],"av=0/1") && strstr(s_output[1],"baud=115200"));
  s_lines=0U;s_link_transport_result=BSP_STATUS_ERROR;
  DebugService_ReportGnssLink();TwoLines();assert(strstr(s_output[0],"av=0/0"));
  Live();s_link_driver_source.service_count=UINT32_MAX;s_link_driver_source.rx_bytes=UINT32_MAX;
  s_link_driver_source.valid_nmea=UINT32_MAX;s_link_driver_source.valid_ubx=UINT32_MAX;
  s_link_driver_source.checksum_errors=UINT32_MAX;s_link_driver_source.framing_resets=UINT32_MAX;
  s_link_driver_source.measured_ms=0U;s_link_driver_source.last_rx_ms=0U;s_link_driver_source.last_frame_ms=0U;s_clock=UINT32_MAX/2U;
  s_link_transport_source.baud_bps=UINT32_MAX;s_link_transport_source.rx_bytes=UINT32_MAX;
  s_link_transport_source.uart_errors=UINT32_MAX;s_link_transport_source.dma_errors=UINT32_MAX;
  s_link_transport_source.rx_drop_bytes=UINT32_MAX;s_link_transport_source.tx_failures=UINT32_MAX;s_link_transport_source.recoveries=UINT32_MAX;
  s_link_transport_source.ndtr=UINT16_MAX;s_link_transport_source.queued_bytes=UINT16_MAX;
  DebugService_ReportGnssLink();TwoLines();assert(strstr(s_output[0],"4294967295") && strstr(s_output[1],"4294967295"));
}
/** @brief 真实100ms任务循环沿用1秒门限；正常RX进展只打印首次两行，不增加采集或原帧。 */
static void TestExistingTaskCadence(void)
{
  unsigned i;volatile unsigned links=0U,io=0U;
  Live();s_clock=0U;s_link_driver_source.measured_ms=0U;s_link_driver_source.last_rx_ms=0U;s_link_driver_source.last_frame_ms=0U;
  s_link_follow_clock=1U;
  if(setjmp(s_task_exit)==0) { DebugService_Task(NULL); }
  for(i=0U;i<s_lines;i++) {
    if(strstr(s_output[i],"GNSS-LINK:")) { links++; }
    if(strstr(s_output[i],"GNSS-IO:")) { io++; }
    assert(!strstr(s_output[i],"GNSS-RAW sampled"));
  }
  assert(links==1U && io==1U && s_link_driver_copies==2U && s_link_transport_copies==2U && s_clock==2300U);
}
#endif

/** @brief 同一用例支持monitor关闭/GNSS禁用，确认不增加硬件采集或任何链路日志。 */
int main(void)
{
  assert(ExistingRawDebugMain()==0);
#if DEBUG_GNSS_MONITOR_ENABLE && PX4LITE_ENABLE_GNSS
  TestFirstAndHealthyProgress();TestTransitionsAndFaultCounters();TestStaleAndAgeBoundaries();
  TestAvailabilityAndMaximumLength();TestExistingTaskCadence();
  printf("GNSS link Debug maximum line including prefix/CRLF: %u/256 bytes\n",s_largest_link_line);
  puts("GNSS link Debug: state/error/age gate / 256B bounds / static read-only snapshot / existing cadence PASS");
#else
  Reset();assert(DebugService_CreateTask()==pdPASS);
  assert(!s_link_driver_copies && !s_link_transport_copies && !s_driver_calls && !s_lines);
  puts("GNSS link Debug disabled: no driver/hardware copies or link lines PASS");
#endif
  return 0;
}
