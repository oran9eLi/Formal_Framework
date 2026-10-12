/** @file test_gnss_time_freshness.c
 * @brief RTC 使用独立 UTC 更新时刻，合法午夜与计时回绕不失真。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Framework/Src/px4lite_time.c"
static Px4Lite_SensorGnss_t source;
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out) { *out=source; return PX4LITE_OK; }
int main(void)
{
  Px4Lite_UtcDateTime_t utc;
  source.header.valid=1U; source.fix_type=1U; source.valid_mask=PX4LITE_GNSS_VALID_TIME;
  source.utc_date=260928U; source.utc_sec=0U;
  assert(Px4Lite_TimeCopyValidGnssTime(0U,&utc)!=0U && utc.hours==0U && utc.seconds==0U);
  source.header.sample_time_ms=2000U;
  assert(Px4Lite_TimeCopyValidGnssTime(2000U,&utc)==0U);
  source.time_update_ms=2000U;
  assert(Px4Lite_TimeCopyValidGnssTime(2000U,&utc)!=0U);
  source.valid_mask=PX4LITE_GNSS_VALID_POSITION;
  assert(Px4Lite_TimeCopyValidGnssTime(2000U,&utc)==0U);
  source.valid_mask=PX4LITE_GNSS_VALID_TIME;
  source.time_update_ms=UINT32_MAX-100U; source.header.sample_time_ms=UINT32_MAX-100U;
  assert(Px4Lite_TimeCopyValidGnssTime(50U,&utc)!=0U);
  puts("gnss time freshness tests passed"); return 0;
}
