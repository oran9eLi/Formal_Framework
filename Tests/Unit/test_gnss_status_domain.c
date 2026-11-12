/**
 * @file test_gnss_status_domain.c
 * @brief 通过真实Adapter和App入口验证GNSS状态域与四个测量域独立。
 * @details Driver/topic使用确定性副本，禁止伪造硬件收数；不改Navigation、无线或CSV布局。
 */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } DMA_HandleTypeDef;
typedef struct { uint32_t CNT, SR; } HostTimer;
#define UART_WORDLENGTH_8B 0U
#define UART_WORDLENGTH_9B 0x1000U
#define UART_PARITY_NONE 0U
#define UART_PARITY_EVEN 0x0400U
#define UART_PARITY_ODD 0x0200U
#define UART_STOPBITS_1 0U
#define UART_STOPBITS_2 0x2000U
#define UART_HWCONTROL_NONE 0U
#define UART_HWCONTROL_RTS 0x0100U
#define UART_HWCONTROL_CTS 0x0200U
#define UART_HWCONTROL_RTS_CTS 0x0300U
static HostTimer s_host_timer;
static uint32_t s_clock;
#define TIM6 (&s_host_timer)
#define TIM_SR_UIF 1U
static uint32_t __get_PRIMASK(void) { return 0U; }
static void __disable_irq(void) { }
static void __enable_irq(void) { }
static uint32_t HAL_GetTick(void) { return s_clock; }
static uint32_t HAL_GetUIDw0(void) { return 0U; }
static uint32_t HAL_GetUIDw1(void) { return 0U; }
static uint32_t HAL_GetUIDw2(void) { return 0U; }

#include "px4lite_config.h"
#if defined(TEST_GNSS_DISABLED) && TEST_GNSS_DISABLED
#undef PX4LITE_ENABLE_GNSS
#define PX4LITE_ENABLE_GNSS 0U
#endif
#include "../../Framework/Src/px4lite_platform_f407.c"
#include "../../Business/Src/app_data_api.c"

static Gnss_Snapshot_t s_driver_source;
static Px4Lite_SensorGnss_t s_topic_source;
static Gnss_Result_t s_service_result=GNSS_RESULT_OK;
static Gnss_Result_t s_snapshot_result=GNSS_RESULT_OK;
static Px4Lite_Result_t s_topic_result=PX4LITE_OK;
static uint32_t s_service_calls, s_topic_copies;

/** @brief 真实平台时钟的确定性替身，支持0和uint32回绕。 */
uint32_t BSP_Time_GetTickMs(void) { return s_clock; }
/** @brief 仅验证Adapter拥有Service入口，不产生接收数据。 */
Gnss_Result_t Sensor_GNSS_Service(uint32_t now)
{ assert(now==s_clock);s_service_calls++;return s_service_result; }
/** @brief 逐次提供独立Driver快照。 */
Gnss_Result_t Sensor_GNSS_CopySnapshot(Gnss_Snapshot_t *out)
{ *out=s_driver_source;return s_snapshot_result; }
/** @brief 解析器恢复请求不触及硬件。 */
void Sensor_GNSS_RequestReinit(void) { }
/** @brief 只复制topic；失败时保留调用者已清零输出，模拟未发布。 */
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out)
{ s_topic_copies++;if(s_topic_result==PX4LITE_OK){*out=s_topic_source;}return s_topic_result; }

/** @brief 清测试副本和Adapter序号，不修改生产配置。 */
static void Reset(void)
{
  memset(&s_driver_source,0,sizeof(s_driver_source));memset(&s_topic_source,0,sizeof(s_topic_source));
  s_gnss_last_rx_sequence=0U;s_gnss_reinit_pending=0U;s_clock=0U;
  s_service_result=GNSS_RESULT_OK;s_snapshot_result=GNSS_RESULT_OK;s_topic_result=PX4LITE_OK;
  s_service_calls=0U;s_topic_copies=0U;
}

/** @brief 无定位/0颗卫星是合法新状态，且不授权坐标或自动授权UTC。 */
static void TestNoFixZeroSatellites(void)
{
  App_GnssDetails_t out;
  Reset();s_driver_source.rx_sequence=1U;s_driver_source.valid_mask=GNSS_VALID_NAV_STATUS;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_OK);
  assert(s_topic_source.valid_mask==PX4LITE_GNSS_VALID_NAV_STATUS && s_topic_source.status_update_ms==0U);
  assert(s_topic_source.header.sample_time_ms==0U && s_topic_source.fix_type==0U && s_topic_source.satellites_used==0U);
  assert(App_CopyGnssDetails(&out,0U)==PX4LITE_OK);
  assert(out.valid_mask==PX4LITE_GNSS_VALID_NAV_STATUS && out.fix_type==0U && out.satellites_used==0U);
  s_topic_source.valid_mask|=PX4LITE_GNSS_VALID_TIME;s_topic_source.utc_date=261106U;s_topic_source.utc_sec=0U;
  assert(App_CopyGnssDetails(&out,0U)==PX4LITE_OK && (out.valid_mask&PX4LITE_GNSS_VALID_TIME));
  assert(!(out.valid_mask&PX4LITE_GNSS_VALID_POSITION));
}

/** @brief 新状态只参与聚合源时间；缺授权位的时间不能参与选择。 */
static void TestAdapterBestAgeAndLegacy(void)
{
  Reset();s_clock=6000U;s_driver_source.rx_sequence=1U;s_driver_source.last_rx_ms=6000U;
  s_driver_source.valid_mask=GNSS_VALID_POSITION|GNSS_VALID_NAV_STATUS;
  s_driver_source.position_update_ms=100U;s_driver_source.status_update_ms=5900U;
  s_driver_source.fix_valid=1U;s_driver_source.fix_quality=2U;s_driver_source.satellites=11U;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_OK);
  assert(s_topic_source.header.sample_time_ms==5900U && s_topic_source.status_update_ms==5900U);
  assert(s_topic_source.fix_type==2U && s_topic_source.satellites_used==11U);
  assert(s_topic_source.position_update_ms==100U);
  s_driver_source.rx_sequence++;s_driver_source.valid_mask=GNSS_VALID_POSITION;
  s_driver_source.status_update_ms=6000U;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_OK);
  assert(s_topic_source.header.sample_time_ms==100U && !(s_topic_source.valid_mask&PX4LITE_GNSS_VALID_NAV_STATUS));
  assert(s_topic_source.fix_type==0U); /* 旧来源仍按位置时间限制质量。 */
  s_driver_source.rx_sequence++;s_driver_source.position_update_ms=6000U;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_OK && s_topic_source.fix_type==2U);
}

/** @brief ACK等新链路活动不续旧状态；新状态也不复活旧坐标、速度、UTC和高度。 */
static void TestIndependentExpiration(void)
{
  App_GnssDetails_t out;
  Reset();s_clock=6000U;s_topic_source.last_rx_ms=6000U;
  s_topic_source.valid_mask=PX4LITE_GNSS_VALID_POSITION|PX4LITE_GNSS_VALID_VELOCITY|
      PX4LITE_GNSS_VALID_TIME|PX4LITE_GNSS_VALID_ALTITUDE|PX4LITE_GNSS_VALID_NAV_STATUS;
  s_topic_source.position_update_ms=100U;s_topic_source.velocity_update_ms=100U;
  s_topic_source.time_update_ms=100U;s_topic_source.altitude_update_ms=100U;s_topic_source.status_update_ms=100U;
  s_topic_source.latitude_e7=123;s_topic_source.satellites_used=12U;
  assert(App_CopyGnssDetails(&out,6000U)==PX4LITE_OK && out.valid_mask==0U);
  s_topic_source.status_update_ms=6000U;
  assert(App_CopyGnssDetails(&out,6000U)==PX4LITE_OK && out.valid_mask==PX4LITE_GNSS_VALID_NAV_STATUS);
  assert(out.latitude_e7==123 && out.position_update_ms==100U); /* 数据保留但已无位置授权。 */
  s_topic_source.valid_mask=0U;
  assert(App_CopyGnssDetails(&out,6000U)==PX4LITE_OK && out.valid_mask==0U);
  s_clock=7501U;s_topic_source.valid_mask=PX4LITE_GNSS_VALID_NAV_STATUS;
  assert(App_CopyGnssDetails(&out,7501U)==PX4LITE_STALE && !(out.valid_mask&PX4LITE_GNSS_VALID_NAV_STATUS));
}

/** @brief 0ms、时钟回绕、真实复制抢占及未获时钟确认的future样本。 */
static void TestClockBoundaries(void)
{
  App_GnssDetails_t out;
  Reset();s_topic_source.valid_mask=PX4LITE_GNSS_VALID_NAV_STATUS;
  s_clock=PX4LITE_GNSS_MAX_AGE_MS;
  assert(App_CopyGnssDetails(&out,s_clock)==PX4LITE_OK && (out.valid_mask&PX4LITE_GNSS_VALID_NAV_STATUS));
  s_clock++;
  assert(App_CopyGnssDetails(&out,s_clock)==PX4LITE_STALE && out.valid_mask==0U);
  s_clock=100U;s_topic_source.status_update_ms=UINT32_MAX-200U;s_topic_source.last_rx_ms=UINT32_MAX-200U;
  assert(App_CopyGnssDetails(&out,100U)==PX4LITE_OK && (out.valid_mask&PX4LITE_GNSS_VALID_NAV_STATUS));
  s_clock=1001U;s_topic_source.status_update_ms=1001U;s_topic_source.last_rx_ms=1001U;
  assert(App_CopyGnssDetails(&out,1000U)==PX4LITE_OK && (out.valid_mask&PX4LITE_GNSS_VALID_NAV_STATUS));
  assert(out.status_update_ms==1001U && out.last_rx_ms==1001U);
  s_clock=999U;
  assert(App_CopyGnssDetails(&out,1000U)==PX4LITE_STALE && out.valid_mask==0U);
  s_clock=2601U;
  assert(App_CopyGnssDetails(&out,1000U)==PX4LITE_STALE && out.valid_mask==0U);
  s_clock=1U;s_topic_source.status_update_ms=0U;s_topic_source.last_rx_ms=0U;
  assert(App_CopyGnssDetails(&out,UINT32_MAX)==PX4LITE_OK && (out.valid_mask&PX4LITE_GNSS_VALID_NAV_STATUS));
}

/** @brief 参数、未发布与底层错误保持原结果；禁用构建不增加新的API准入语义。 */
static void TestErrorsAndUnpublished(void)
{
  App_GnssDetails_t out;
  Reset();assert(Px4Lite_GnssRead(NULL)==PX4LITE_INVALID_PARAM && s_service_calls==0U);
  assert(App_CopyGnssDetails(NULL,0U)==PX4LITE_INVALID_PARAM && s_topic_copies==0U);
  s_topic_result=PX4LITE_NOT_READY;memset(&out,0xA5,sizeof(out));
  assert(App_CopyGnssDetails(&out,0U)==PX4LITE_NOT_READY && out.valid_mask==0U && out.status_update_ms==0U);
  s_service_result=GNSS_RESULT_IO_ERROR;assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_IO_ERROR);
  s_service_result=GNSS_RESULT_OK;s_snapshot_result=GNSS_RESULT_INVALID_PARAM;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_IO_ERROR);
  s_snapshot_result=GNSS_RESULT_OK;s_driver_source.rx_sequence=0U;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_IDLE);
}

/** @brief 同轮有效RX不被TX错误遮住；旧序列不续在线心跳或定位/授时时效。 */
static void TestValidRxSurvivesServiceError(void)
{
  uint32_t old_sequence;
  Reset();s_clock=2000U;s_service_result=GNSS_RESULT_IO_ERROR;
  s_driver_source.rx_sequence=1U;s_driver_source.last_rx_ms=2000U;
  s_driver_source.valid_mask=GNSS_VALID_NAV_STATUS;s_driver_source.status_update_ms=2000U;
  s_driver_source.fix_quality=0U;s_driver_source.satellites=0U;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_OK);
  assert(s_topic_source.last_rx_ms==2000U&&s_topic_source.fix_type==0U&&s_topic_source.valid_mask==PX4LITE_GNSS_VALID_NAV_STATUS);
  old_sequence=s_gnss_last_rx_sequence;s_clock=6000U;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_IO_ERROR&&s_gnss_last_rx_sequence==old_sequence);
  assert(s_topic_source.last_rx_ms==2000U&&s_topic_source.status_update_ms==2000U);
  s_driver_source.rx_sequence=2U;s_driver_source.last_rx_ms=6000U;
  s_driver_source.valid_mask=GNSS_VALID_POSITION|GNSS_VALID_NAV_STATUS;
  s_driver_source.position_update_ms=100U;s_driver_source.status_update_ms=100U;s_driver_source.fix_quality=1U;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_OK);
  assert(s_topic_source.last_rx_ms==6000U&&s_topic_source.position_update_ms==100U&&s_topic_source.status_update_ms==100U&&s_topic_source.fix_type==0U);
  s_service_result=GNSS_RESULT_OK;assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_IDLE);
  s_snapshot_result=GNSS_RESULT_INVALID_PARAM;s_driver_source.rx_sequence=3U;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_IO_ERROR&&s_gnss_last_rx_sequence==2U);
  s_snapshot_result=GNSS_RESULT_NO_DATA;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_IDLE&&s_gnss_last_rx_sequence==2U);
  s_service_result=GNSS_RESULT_IO_ERROR;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_IO_ERROR&&s_gnss_last_rx_sequence==2U);
  s_snapshot_result=GNSS_RESULT_IO_ERROR;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_IO_ERROR&&s_gnss_last_rx_sequence==2U);
  s_snapshot_result=GNSS_RESULT_NO_FIX;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_OK&&s_gnss_last_rx_sequence==3U);
  Reset();s_service_result=GNSS_RESULT_IO_ERROR;
  assert(Px4Lite_GnssRead(&s_topic_source)==PX4LITE_IO_ERROR&&s_gnss_last_rx_sequence==0U);
}

/** @brief 执行真实跨层状态域与时效检查。 */
int main(void)
{
  TestNoFixZeroSatellites();TestAdapterBestAgeAndLegacy();TestIndependentExpiration();
  TestClockBoundaries();TestErrorsAndUnpublished();TestValidRxSurvivesServiceError();
  puts("GNSS status domain Adapter/App tests passed");return 0;
}
