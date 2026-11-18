/**
 * @file test_power_voltage_discharge.c
 * @brief 真实ADC2合作弱泄放的GPIO所有权、恢复等待、正式测量和故障收尾回归。
 * @details HAL仅注入ADC/引脚事实；不把本模型当作真实GM模块源阻抗或准确度实测。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef struct { unsigned ClockPrescaler,Resolution,ScanConvMode,ContinuousConvMode,
  DiscontinuousConvMode,ExternalTrigConvEdge,ExternalTrigConv,DataAlign,
  NbrOfConversion,DMAContinuousRequests,EOCSelection; } ADC_InitTypeDef;
typedef struct { void *Instance; ADC_InitTypeDef Init; } ADC_HandleTypeDef;
typedef struct { unsigned Channel,Rank,SamplingTime,Offset; } ADC_ChannelConfTypeDef;
typedef struct { unsigned Pin,Mode,Pull,Speed,Alternate; } GPIO_InitTypeDef;
typedef unsigned GPIO_TypeDef;
#define ADC2 ((void *)2)
#define ADC3 ((void *)3)
#define GPIOA ((GPIO_TypeDef *)1)
#define GPIOC ((GPIO_TypeDef *)3)
#define ADC_CHANNEL_4 4U
#define ADC_CHANNEL_5 5U
#define ADC_CHANNEL_10 10U
#define ADC_CHANNEL_11 11U
#define GPIO_PIN_0 (1U<<0)
#define GPIO_PIN_1 (1U<<1)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_5 (1U<<5)
#define ADC_SAMPLETIME_480CYCLES 480U
#define ADC_CLOCK_SYNC_PCLK_DIV4 4U
#define ADC_RESOLUTION_12B 12U
#define DISABLE 0U
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0U
#define ADC_SOFTWARE_START 0U
#define ADC_DATAALIGN_RIGHT 0U
#define ADC_EOC_SINGLE_CONV 0U
#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_ANALOG 2U
#define GPIO_NOPULL 0U
#define GPIO_PULLDOWN 2U
#define GPIO_SPEED_FREQ_LOW 0U

#include "../../Bsp/Inc/bsp_config.h"

/** @brief 绑定归档物理路径，防止HAL桩与生产宏一起错误地读到另一组悬空脚。 */
static void CheckPhysicalRoute(void)
{
  assert(BSP_ADC2_INS==ADC2 && BSP_ADC2_PORT==GPIOA);
  assert(BSP_ADC_CURRENT_INS==ADC3 && BSP_ADC_CURRENT2_PORT==GPIOC);
#if BSP_POWER2_USE_EXPANSION_ROUTE
  assert(BSP_ADC2_CH==ADC_CHANNEL_5 && BSP_ADC2_PIN==GPIO_PIN_5);
  assert(BSP_ADC_CURRENT2_CH==ADC_CHANNEL_10 && BSP_ADC_CURRENT2_PIN==GPIO_PIN_0);
#else
  assert(BSP_ADC2_CH==ADC_CHANNEL_4 && BSP_ADC2_PIN==GPIO_PIN_4);
  assert(BSP_ADC_CURRENT2_CH==ADC_CHANNEL_11 && BSP_ADC_CURRENT2_PIN==GPIO_PIN_1);
#endif
}

static uint32_t s_ms,s_ghost_raw,s_normal_raw=1464U,s_pulled_raw=1400U;
static uint8_t s_powered=1U;
static unsigned s_critical,s_mode=GPIO_MODE_ANALOG,s_pull=GPIO_NOPULL;
static unsigned s_init,s_deinit,s_starts,s_dummy,s_normal,s_gpio,s_clock_reads,s_ref;
static HAL_StatusTypeDef s_init_rc=HAL_OK,s_config_rc=HAL_OK,s_start_rc=HAL_OK,s_poll_rc=HAL_OK;
static unsigned s_ref_error;
#define BSP_CRITICAL_H
/** @brief 临界区仅允许复制诊断，不允许HAL硬件调用。 */
static uint32_t BSP_Critical_Enter(void) { s_critical++;return 0U; }
/** @brief 核对快照保护配对。 */
static void BSP_Critical_Exit(uint32_t key) { (void)key;assert(s_critical);s_critical--; }
/** @brief 所有HAL边界都必须在快照锁外。 */
static inline void Outside(void) { assert(!s_critical); }
static inline HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h)
{ Outside();assert(h->Instance==ADC2);s_init++;return s_init_rc; }
static inline HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *h)
{ Outside();assert(h->Instance==ADC2);s_deinit++;return HAL_OK; }
static inline HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h,ADC_ChannelConfTypeDef *c)
{ Outside();assert(h->Instance==ADC2 && c->Channel==BSP_ADC2_CH && c->SamplingTime==480U);return s_config_rc; }
static inline HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h)
{
  Outside();assert(h->Instance==ADC2);
  assert((s_mode==GPIO_MODE_INPUT && s_pull==GPIO_PULLDOWN) ||
         (s_mode==GPIO_MODE_ANALOG && s_pull==GPIO_NOPULL));
  s_starts++;return s_start_rc;
}
static inline HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h)
{ Outside();assert(h->Instance==ADC2);return HAL_OK; }
static inline HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h,uint32_t ms)
{ Outside();assert(h->Instance==ADC2 && ms==10U);return s_poll_rc; }
static inline uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h)
{
  Outside();assert(h->Instance==ADC2);
  if(s_mode==GPIO_MODE_INPUT) { assert(s_pull==GPIO_PULLDOWN);s_dummy++;s_ghost_raw=0U;return s_powered?s_pulled_raw:0U; }
  s_normal++;return s_powered?s_normal_raw:s_ghost_raw;
}
static inline void HAL_GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *g)
{
  Outside();assert(port==GPIOA && g->Pin==BSP_ADC2_PIN);
  assert(g->Mode!=GPIO_MODE_OUTPUT_PP);
  assert((g->Mode==GPIO_MODE_INPUT && g->Pull==GPIO_PULLDOWN) ||
         (g->Mode==GPIO_MODE_ANALOG && g->Pull==GPIO_NOPULL));
  s_mode=g->Mode;s_pull=g->Pull;s_gpio++;
}

#include "../../Bsp/Src/bsp_adc2.c"

/** @brief 明确的真实毫秒注入，允许0和自然回绕。 */
uint32_t BSP_Time_GetTickMs(void) { Outside();s_clock_reads++;return s_ms; }
/** @brief 参考获取只允许在Analog恢复后，不允许把dummy当正式测量。 */
uint32_t BSP_ADC_GetVddaMv(void)
{ Outside();assert(s_mode==GPIO_MODE_ANALOG && s_pull==GPIO_NOPULL);s_ref++;return 3300U; }
/** @brief 正常VDDA为确定性测量事实，不访问真实ADC1。 */
BSP_Status_t BSP_ADC_GetReferenceDiag(BSP_ADC_ReferenceDiag_t *out)
{
  Outside();assert(s_mode==GPIO_MODE_ANALOG);if(s_ref_error) { return BSP_STATUS_ERROR; }
  memset(out,0,sizeof(*out));out->vdda_mv=3300U;out->calculated_vdda_mv=3300U;
  out->vref_raw=1500U;out->calibration_raw=1500U;out->ready=1U;return BSP_STATUS_OK;
}
/** @brief 场景初始化只通过生产DeInit清合作状态，不擦其诊断历史。 */
static void Reset(void)
{
  (void)BSP_ADC2_DeInit();
  s_ms=0U;s_mode=GPIO_MODE_ANALOG;s_pull=GPIO_NOPULL;s_powered=1U;s_ghost_raw=1464U;s_normal_raw=1464U;s_pulled_raw=1400U;
  s_init=s_deinit=s_starts=s_dummy=s_normal=s_gpio=s_clock_reads=s_ref=0U;
  s_init_rc=s_config_rc=s_start_rc=s_poll_rc=HAL_OK;s_ref_error=0U;assert(!s_critical);
}
/** @brief 对外诊断读取必须不引入额外硬件采样。 */
#if BSP_ENABLE_POWER2
static BSP_ADC2_Diag_t Diag(void)
{ BSP_ADC2_Diag_t out;unsigned reads=s_starts;assert(BSP_ADC2_GetDiag(&out)==BSP_STATUS_OK);assert(reads==s_starts && !s_critical);return out; }
#endif

#if BSP_ENABLE_POWER2 && BSP_ADC2_DISCHARGE_ENABLE
/** @brief 数次dummy后立即恢复Analog、窗口前不I/O、正式恢复后才交付原比例电压。 */
static void TestNormalAndOpenRecovery(void)
{
  uint32_t value=777U,sequence;unsigned starts,gpio,refs;BSP_ADC2_Diag_t d;
  Reset();assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY && value==777U);
  assert(s_dummy==BSP_ADC2_DISCHARGE_SAMPLE_COUNT && s_normal==0U && s_gpio==2U);
  assert(s_mode==GPIO_MODE_ANALOG && s_pull==GPIO_NOPULL && !s_ref);
  d=Diag();sequence=d.sequence;assert(d.ready && !d.sample_valid && d.discharge_pending);
  assert(d.recovery_ms==BSP_ADC2_DISCHARGE_RECOVERY_MS && d.discharge_raw==1400U);
  assert(d.discharge_enabled && d.discharge_checked);
  starts=s_starts;gpio=s_gpio;refs=s_ref;s_ms=BSP_ADC2_DISCHARGE_RECOVERY_MS-1U;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY && value==777U);
  assert(starts==s_starts && gpio==s_gpio && refs==s_ref);
  s_ms++;assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_OK && value==12009U);
  d=Diag();assert(d.sequence==sequence && d.sample_valid && !d.discharge_pending);
  assert(d.raw_average==1464U && d.pin_mv==1180U && s_normal==16U);
  assert(d.discharge_pin_mv==1128U);
  /* 开路模型保留旧电荷；只有弱输入泄放清零，不能靠以前的正读数继续发布。 */
  s_powered=0U;s_ghost_raw=1464U;s_ms++;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY && value==12009U);
  s_ms+=BSP_ADC2_DISCHARGE_RECOVERY_MS;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_OK && value==0U);
  assert(Diag().discharge_pin_mv==0U && Diag().discharge_checked);
  s_powered=1U;s_ms++;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY);
  s_ms+=BSP_ADC2_DISCHARGE_RECOVERY_MS;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_OK && value==12009U);
}
/** @brief 高阻/弱驱动模型的dummy与正式值分别上报，BSP不得自己判定电源在位。 */
static void TestRawFactsStaySeparate(void)
{
  uint32_t value=17U;BSP_ADC2_Diag_t d;Reset();s_pulled_raw=20U;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY);
  s_ms+=BSP_ADC2_DISCHARGE_RECOVERY_MS;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_OK && value==12009U);
  d=Diag();assert(d.sample_valid && d.discharge_checked && d.discharge_raw==20U);
  assert(d.discharge_pin_mv==16U && d.pin_mv==1180U);
}
/** @brief dummy与正式转换出错均不改输出；HAL_BUSY不得伪装合作等待，诊断保留原码。 */
static void TestErrorsAndRestore(void)
{
  uint32_t value=456U;BSP_ADC2_Diag_t d;
  Reset();s_start_rc=HAL_BUSY;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_ERROR && value==456U);
  d=Diag();assert(!d.discharge_pending && !d.sample_valid && d.discharge_status==BSP_STATUS_BUSY);
  assert(s_mode==GPIO_MODE_ANALOG && s_pull==GPIO_NOPULL && s_gpio==2U);
  s_start_rc=HAL_OK;s_poll_rc=HAL_TIMEOUT;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_TIMEOUT && value==456U);
  assert(s_mode==GPIO_MODE_ANALOG && !Diag().discharge_pending);
  s_poll_rc=HAL_OK;assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY);
  s_ms+=BSP_ADC2_DISCHARGE_RECOVERY_MS;s_start_rc=HAL_BUSY;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_ERROR && value==456U);
  d=Diag();assert(!d.discharge_pending && d.sample_status==BSP_STATUS_BUSY && !d.sample_valid);
  s_start_rc=HAL_OK;assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY);
  s_ms+=BSP_ADC2_DISCHARGE_RECOVERY_MS;s_ref_error=1U;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_ERROR && value==456U);
  assert(s_mode==GPIO_MODE_ANALOG && !Diag().discharge_pending);
}
/** @brief Recover取消旧deadline，时间0与跨uint32回绕都能按相对时间完成。 */
static void TestRecoverAndWrap(void)
{
  uint32_t value=9U,sequence;Reset();
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY);sequence=Diag().sequence;
  assert(BSP_ADC2_Recover()==BSP_STATUS_OK && !Diag().discharge_pending);
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY && Diag().sequence==sequence+1U);
  s_ms+=BSP_ADC2_DISCHARGE_RECOVERY_MS;assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_OK);
  Reset();s_ms=UINT32_MAX-50U;
  assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY);
  s_ms+=BSP_ADC2_DISCHARGE_RECOVERY_MS-1U;assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_BUSY);
  s_ms++;assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_OK && value==12009U);
}
#endif

/** @brief 同一真实BSP核验两种物理路由及启停组合，无输出驱动或电流GPIO访问接口。 */
int main(void)
{
  CheckPhysicalRoute();
#if !BSP_ENABLE_POWER2
  uint32_t value=55U;Reset();assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_ERROR && value==55U);
  assert(!s_starts && !s_gpio && !s_ref && !s_clock_reads);puts("ADC2 discharge disabled source: no IO PASS");
#elif !BSP_ADC2_DISCHARGE_ENABLE
  uint32_t value=55U;Reset();assert(BSP_ADC2_ReadVoltageMv(&value)==BSP_STATUS_OK && value==12009U);
  assert(s_normal==16U && !s_dummy && !s_gpio && !s_clock_reads);assert(Diag().sample_valid);
  assert(!Diag().discharge_enabled && !Diag().discharge_checked && !Diag().recovery_ms);
  puts("ADC2 discharge feature off: original single-call Analog measurement PASS");
#else
  TestNormalAndOpenRecovery();TestRawFactsStaySeparate();TestErrorsAndRestore();TestRecoverAndWrap();
  puts("ADC2 discharge: selected physical route / weak-input dummy / restored Analog wait / true measurement / open model / zero+wrap / failure cleanup / no PP or current GPIO PASS");
#endif
  return 0;
}
