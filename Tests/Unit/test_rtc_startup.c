/** @file test_rtc_startup.c
 * @brief 执行真实 RTC BSP 时钟选择路径；HAL/晶振由确定性替身提供。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef unsigned FlagStatus;
#define SET 1U
#define HAL_OK 0
#define RCC_FLAG_LSERDY 1U
#define RCC_FLAG_LSIRDY 2U
#define RCC_LSE_ON 1U
#define RCC_LSE_OFF 0U
#define RCC_RTCCLKSOURCE_LSE 1U
#define RCC_RTCCLKSOURCE_LSI 2U
#define RTC_HOURFORMAT_24 0U
#define RTC_OUTPUT_DISABLE 0U
#define RTC_OUTPUT_POLARITY_HIGH 0U
#define RTC_OUTPUT_TYPE_OPENDRAIN 0U
#define RTC_FORMAT_BIN 0U
#define RTC_DAYLIGHTSAVING_NONE 0U
#define RTC_STOREOPERATION_RESET 0U
#define RTC_BKP_DR0 0U
#define RTC ((void *)1)
typedef struct { uint32_t HourFormat, AsynchPrediv, SynchPrediv, OutPut, OutPutPolarity, OutPutType; } RtcInit;
typedef struct { void *Instance; RtcInit Init; } RTC_HandleTypeDef;
typedef struct { uint8_t Year, Month, Date, WeekDay; } RTC_DateTypeDef;
typedef struct { uint8_t Hours, Minutes, Seconds, DayLightSaving, StoreOperation; } RTC_TimeTypeDef;
static uint32_t tick, source, flags, allow_flags, config_count;
#define __HAL_RCC_PWR_CLK_ENABLE() ((void)0)
#define HAL_PWR_EnableBkUpAccess() ((void)0)
#define __HAL_RCC_GET_RTC_SOURCE() (source)
#define __HAL_RCC_GET_FLAG(f) ((flags & (f)) ? SET : 0U)
#define __HAL_RCC_LSE_CONFIG(x) do { if (x) { flags |= (allow_flags & 1U); } else { flags &= ~1U; } } while (0)
#define __HAL_RCC_LSI_ENABLE() do { flags |= (allow_flags & 2U); } while (0)
#define __HAL_RCC_RTC_CONFIG(x) do { source = (x); config_count++; } while (0)
#define __HAL_RCC_RTC_ENABLE() ((void)0)
uint32_t HAL_GetTick(void) { tick += 1000U; return tick; }
int HAL_RTC_Init(RTC_HandleTypeDef *h) { (void)h; return (flags & source) ? HAL_OK : 1; }
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *h, uint32_t r) { (void)h; (void)r; return 0U; }
int HAL_RTC_GetTime(RTC_HandleTypeDef *h, RTC_TimeTypeDef *t, unsigned f);
int HAL_RTC_GetDate(RTC_HandleTypeDef *h, RTC_DateTypeDef *d, unsigned f);
int HAL_RTC_SetTime(RTC_HandleTypeDef *h, RTC_TimeTypeDef *t, unsigned f);
int HAL_RTC_SetDate(RTC_HandleTypeDef *h, RTC_DateTypeDef *d, unsigned f);
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *h, uint32_t r, uint32_t v);
#include "../../Bsp/Src/bsp_rtc.c"
int main(void)
{
  source = 2U; allow_flags = 2U;
  assert(BSP_RTC_Init() == BSP_STATUS_OK);
  assert(flags == 2U && source == 2U && config_count == 0U);
  source = 1U; flags = 0U; allow_flags = 1U;
  assert(BSP_RTC_Init() == BSP_STATUS_OK);
  assert(flags == 1U && config_count == 0U);
  flags = 0U; allow_flags = 0U; tick = 0U;
  assert(BSP_RTC_Init() != BSP_STATUS_OK);
  assert(s_rtc_ready == 0U && tick <= 8000U && config_count == 0U);
  source = 0U; allow_flags = 2U; tick = 0U;
  assert(BSP_RTC_Init() == BSP_STATUS_OK);
  assert(source == 2U && config_count == 1U);
  source = 3U; flags = 0U; config_count = 0U;
  assert(BSP_RTC_Init() != BSP_STATUS_OK);
  assert(source == 3U && config_count == 0U);
  puts("RTC retained-clock startup tests passed");
  return 0;
}
