/**
 * @file bsp_rgb.c
 * @brief BSP层独占PC9/AF3/TIM8_CH4及DMA2 Stream7的原始PWM发送。
 * @details 依赖HAL、MSP和板级毫秒时基；不做GRB编码或声光策略，不访问其他外设。
 *          单份静态DMA副本位于主SRAM；中断只置完成/错误标志，任务推进停止及超时。
 */
#include "bsp_rgb.h"
#include "bsp_config.h"
#include "bsp_time.h"
#include <string.h>

#if BSP_ENABLE_RGB
static TIM_HandleTypeDef s_rgb_tim; /**< TIM8私有句柄，不与电机或蜂鸣器共用。 */
static DMA_HandleTypeDef s_rgb_dma; /**< DMA2 Stream7私有句柄，普通半字发送。 */
static uint16_t s_dma_duty[BSP_RGB_MAX_DUTY_COUNT + 3U]; /**< 主SRAM静态DMA副本，含首部1槽和尾部2槽，共254byte。 */
static uint16_t s_period, s_zero, s_one; /**< 周期计数及两种建议高电平计数，单位TIM8 tick。 */
static uint32_t s_started_ms; /**< 最近接受发送的板级毫秒时刻，允许32位回绕。 */
static uint8_t s_initialized; /**< 初始化链完整时为1，故障后为0。 */
static uint8_t s_busy; /**< 静态DMA副本在发送或等待owner撤收时为1。 */
static volatile uint8_t s_dma_done; /**< DMA IRQ观察到TC时置1，不在中断推进业务。 */
static volatile uint8_t s_dma_error; /**< DMA IRQ观察到TE/DME/FE时置1。 */
static BSP_Status_t s_result = BSP_STATUS_ERROR; /**< 最近发送结果；错误锁存至Init。 */

/** @brief DMA完成中断只记录事实；尾部保护零槽保证有效载荷已完整离开PWM输出。 */
static void BspRgb_DmaComplete(DMA_HandleTypeDef *hdma)
{
  if (hdma == &s_rgb_dma) { s_dma_done = 1U; }
}

/** @brief DMA错误中断只置标志，停止/恢复必须留在sensor任务。 */
static void BspRgb_DmaError(DMA_HandleTypeDef *hdma)
{
  if (hdma == &s_rgb_dma) { s_dma_error = 1U; }
}

/**
 * @brief 任务中关闭请求和PWM，并无等待地请求DMA停用。
 * @return DMA EN尚未清除时返回BUSY；调用者必须保留副本并在后续任务周期重试。
 * @note Stream7由本模块独占，禁止为停RGB复位整个DMA2控制器。
 */
static BSP_Status_t BspRgb_Stop(void)
{
  if ((RCC->APB2ENR & RCC_APB2ENR_TIM8EN) != 0U) {
    CLEAR_BIT(TIM8->DIER, TIM_DIER_CC4DE);
    if (s_rgb_tim.Instance != 0) { (void)HAL_TIM_PWM_Stop(&s_rgb_tim, BSP_RGB_CHANNEL); }
    else {
      /* 启动加载器可能留下已开时钟；冷句柄不可传入HAL。 */
      CLEAR_BIT(TIM8->CCER, TIM_CCER_CC4E);
      CLEAR_BIT(TIM8->BDTR, TIM_BDTR_MOE);
      CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN);
    }
    TIM8->CCR4 = 0U;
    TIM8->EGR = TIM_EGR_UG;
    TIM8->CNT = 0U;
  }
  if ((RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN) != 0U) {
    CLEAR_BIT(BSP_RGB_DMA_STREAM->CR, DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    CLEAR_BIT(BSP_RGB_DMA_STREAM->FCR, DMA_SxFCR_FEIE);
    CLEAR_BIT(BSP_RGB_DMA_STREAM->CR, DMA_SxCR_EN);
    __DSB();
    if ((BSP_RGB_DMA_STREAM->CR & DMA_SxCR_EN) != 0U) { return BSP_STATUS_BUSY; }
    DMA2->HIFCR = DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7 |
                  DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;
  }
  return BSP_STATUS_OK;
}

/** @brief 根据实际APB2定时器时钟换算ns为计数；仅初始化调用，不占用周期发送路径。 */
static uint16_t BspRgb_NsToTicks(uint32_t clock_hz, uint32_t time_ns)
{
  return (uint16_t)(((uint64_t)clock_hz * time_ns + 500000000ULL) / 1000000000ULL);
}
#endif

/** @brief 建立TIM8 CH4零占空比及DMA句柄；NVIC最后开放，防止半初始化中断。 */
BSP_Status_t MX_TIM8_Init(void)
{
#if BSP_ENABLE_RGB
  TIM_OC_InitTypeDef channel = {0};
  TIM_BreakDeadTimeConfigTypeDef break_config = {0};
  uint32_t clock_hz = HAL_RCC_GetPCLK2Freq();
  uint32_t period;
  if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) { clock_hz *= 2U; }
  period = (clock_hz + BSP_RGB_BITRATE_HZ / 2U) / BSP_RGB_BITRATE_HZ;
  s_zero = BspRgb_NsToTicks(clock_hz, BSP_RGB_ZERO_HIGH_NS);
  s_one = BspRgb_NsToTicks(clock_hz, BSP_RGB_ONE_HIGH_NS);
  if ((period < 2U) || (period > 65535U) || (s_zero == 0U) ||
      (s_zero >= s_one) || ((uint32_t)s_one >= period)) { return BSP_STATUS_ERROR; }
  s_period = (uint16_t)period;
  s_rgb_tim.Instance = BSP_RGB_TIM;
  s_rgb_tim.Init.Prescaler = 0U;
  s_rgb_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_rgb_tim.Init.Period = period - 1U;
  s_rgb_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  s_rgb_tim.Init.RepetitionCounter = 0U;
  s_rgb_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&s_rgb_tim) != HAL_OK) { return BSP_STATUS_ERROR; }
  channel.OCMode = TIM_OCMODE_PWM1;
  channel.Pulse = 0U;
  channel.OCPolarity = TIM_OCPOLARITY_HIGH;
  channel.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  channel.OCFastMode = TIM_OCFAST_DISABLE;
  channel.OCIdleState = TIM_OCIDLESTATE_RESET;
  channel.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&s_rgb_tim, &channel, BSP_RGB_CHANNEL) != HAL_OK) { return BSP_STATUS_ERROR; }
  /* MOE/CC4E关闭时同样保持低电平，防止停止后复用引脚浮空。 */
  break_config.OffStateRunMode = TIM_OSSR_ENABLE;
  break_config.OffStateIDLEMode = TIM_OSSI_ENABLE;
  break_config.LockLevel = TIM_LOCKLEVEL_OFF;
  break_config.DeadTime = 0U;
  break_config.BreakState = TIM_BREAK_DISABLE;
  break_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  break_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&s_rgb_tim, &break_config) != HAL_OK) { return BSP_STATUS_ERROR; }
  /* CCDS=0表示CC4比较事件提出DMA请求，CCR4预装载仅在更新事件生效。 */
  CLEAR_BIT(TIM8->CR2, TIM_CR2_CCDS);
  SET_BIT(TIM8->CCMR2, TIM_CCMR2_OC4PE);
  TIM8->CCR4 = 0U;
  TIM8->EGR = TIM_EGR_UG;
  TIM8->SR = 0U;
  s_rgb_dma.Instance = BSP_RGB_DMA_STREAM;
  s_rgb_dma.Init.Channel = BSP_RGB_DMA_CHANNEL;
  s_rgb_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  s_rgb_dma.Init.PeriphInc = DMA_PINC_DISABLE;
  s_rgb_dma.Init.MemInc = DMA_MINC_ENABLE;
  s_rgb_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  s_rgb_dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  s_rgb_dma.Init.Mode = DMA_NORMAL;
  s_rgb_dma.Init.Priority = DMA_PRIORITY_HIGH;
  s_rgb_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&s_rgb_dma) != HAL_OK) { return BSP_STATUS_ERROR; }
  __HAL_LINKDMA(&s_rgb_tim, hdma[TIM_DMA_ID_CC4], s_rgb_dma);
  s_rgb_dma.XferCpltCallback = BspRgb_DmaComplete;
  s_rgb_dma.XferErrorCallback = BspRgb_DmaError;
  HAL_NVIC_ClearPendingIRQ(BSP_RGB_DMA_IRQn);
  HAL_NVIC_EnableIRQ(BSP_RGB_DMA_IRQn);
  return BSP_STATUS_OK;
#else
  return BSP_STATUS_ERROR;
#endif
}

/** @brief 初始化可选波形通道；DMA主SRAM范围及完整链均通过后才接受请求。 */
BSP_Status_t BSP_RGB_Init(void)
{
#if BSP_ENABLE_RGB
  uintptr_t address = (uintptr_t)s_dma_duty;
  if (s_initialized != 0U) { return (s_busy != 0U) ? BSP_STATUS_BUSY : BSP_STATUS_OK; }
  if ((address < 0x20000000UL) || ((address + sizeof(s_dma_duty)) > 0x20020000UL)) { return BSP_STATUS_ERROR; }
  if (BSP_RGB_DeInit() != BSP_STATUS_OK) { return BSP_STATUS_BUSY; }
  if (MX_TIM8_Init() != BSP_STATUS_OK) {
    BSP_RGB_EmergencyStop();
    (void)BSP_RGB_DeInit();
    s_result = BSP_STATUS_ERROR;
    return s_result;
  }
  s_dma_done = 0U;
  s_dma_error = 0U;
  s_busy = 0U;
  s_initialized = 1U;
  s_result = BSP_STATUS_OK;
  return BSP_STATUS_OK;
#else
  return BSP_STATUS_ERROR;
#endif
}

/** @brief 任务级无等待释放；只有确认DMA EN清除后才重置HAL状态并释放句柄。 */
BSP_Status_t BSP_RGB_DeInit(void)
{
#if BSP_ENABLE_RGB
  s_initialized = 0U;
  HAL_NVIC_DisableIRQ(BSP_RGB_DMA_IRQn);
  if (BspRgb_Stop() != BSP_STATUS_OK) { return BSP_STATUS_BUSY; }
  if (s_rgb_dma.Instance != 0) {
    /* 硬件已静止且IRQ关闭，允许将被超时打断的HAL状态归零，避免HAL等待循环。 */
    s_rgb_dma.State = HAL_DMA_STATE_READY;
    s_rgb_dma.Lock = HAL_UNLOCKED;
    (void)HAL_DMA_DeInit(&s_rgb_dma);
  }
  if (s_rgb_tim.Instance != 0) { (void)HAL_TIM_PWM_DeInit(&s_rgb_tim); }
  HAL_NVIC_ClearPendingIRQ(BSP_RGB_DMA_IRQn);
  s_busy = 0U;
  s_dma_done = 0U;
  s_dma_error = 0U;
  return BSP_STATUS_OK;
#else
  return BSP_STATUS_OK;
#endif
}

/**
 * @brief 复制原始波形后启动CC4 DMA，首尾保护槽消除预装载流水线边界。
 * @details 启动前CCR4活动值/预装载均为0；DMA首项再写0，随后才写首个有效比较值。
 *          因此首个比较事件只消耗低电平保护项，不会丢失或重复首个有效脉冲。
 *          DMA尾部再加2个0；TC发生时倒数第一个0已成为活动值，载荷已完整输出。
 */
BSP_Status_t BSP_RGB_Send(const uint16_t *duty, uint16_t count)
{
#if BSP_ENABLE_RGB
  uint16_t i;
  if ((duty == 0) || (count == 0U) || (count > BSP_RGB_MAX_DUTY_COUNT)) { return BSP_STATUS_ERROR; }
  if (s_initialized == 0U) { return BSP_STATUS_ERROR; }
  if (s_busy != 0U) { return BSP_STATUS_BUSY; }
  for (i = 0U; i < count; ++i) { if (duty[i] >= s_period) { return BSP_STATUS_ERROR; } }
  s_dma_duty[0] = 0U;
  memcpy(&s_dma_duty[1], duty, count * sizeof(uint16_t));
  s_dma_duty[count + 1U] = 0U;
  s_dma_duty[count + 2U] = 0U;
  s_dma_done = 0U;
  s_dma_error = 0U;
  s_busy = 1U;
  s_result = BSP_STATUS_BUSY;
  s_started_ms = BSP_Time_GetTickMs();
  CLEAR_BIT(TIM8->DIER, TIM_DIER_CC4DE);
  TIM8->CCR4 = 0U;
  TIM8->EGR = TIM_EGR_UG;
  TIM8->CNT = 0U;
  TIM8->SR = 0U;
  __DMB();
  if (HAL_DMA_Start_IT(&s_rgb_dma, (uint32_t)s_dma_duty, (uint32_t)&TIM8->CCR4, count + 3U) == HAL_OK) {
    SET_BIT(TIM8->DIER, TIM_DIER_CC4DE);
    if (HAL_TIM_PWM_Start(&s_rgb_tim, BSP_RGB_CHANNEL) == HAL_OK) { return BSP_STATUS_OK; }
  }
  (void)BspRgb_Stop();
  s_busy = 0U;
  s_initialized = 0U;
  s_result = BSP_STATUS_ERROR;
  return s_result;
#else
  (void)duty;
  (void)count;
  return BSP_STATUS_ERROR;
#endif
}

/** @brief owner在任务中检查事实/有界超时并停止输出；错误保留至上层限速重新初始化。 */
BSP_Status_t BSP_RGB_Service(uint32_t now_ms)
{
#if BSP_ENABLE_RGB
  BSP_Status_t result;
  if (s_initialized == 0U) { return s_result; }
  if (s_busy == 0U) { return s_result; }
  if (s_dma_error != 0U) { result = BSP_STATUS_ERROR; }
  else if (s_dma_done != 0U) { result = BSP_STATUS_OK; }
  else if ((uint32_t)(now_ms - s_started_ms) >= BSP_RGB_DMA_TIMEOUT_MS) { result = BSP_STATUS_TIMEOUT; }
  else { return BSP_STATUS_BUSY; }
  if (BspRgb_Stop() != BSP_STATUS_OK) {
    if (result == BSP_STATUS_OK) {
      if ((uint32_t)(now_ms - s_started_ms) < BSP_RGB_DMA_TIMEOUT_MS) { return BSP_STATUS_BUSY; }
      /* 即使TC已到达，EN迟迟不撤收也必须形成有界故障，禁止无限BUSY。 */
      result = BSP_STATUS_TIMEOUT;
    }
  }
  s_busy = 0U;
  s_result = result;
  if (result != BSP_STATUS_OK) { s_initialized = 0U; }
  return result;
#else
  (void)now_ms;
  return BSP_STATUS_ERROR;
#endif
}

/** @brief 只查询owner维护的占用，不提前允许复用尚未由Service撤收的副本。 */
uint8_t BSP_RGB_IsBusy(void)
{
#if BSP_ENABLE_RGB
  return s_busy;
#else
  return 0U;
#endif
}

/** @brief 输出以实际定时器时钟计算的周期/脉宽计数，不承诺器件在位或电气验收。 */
BSP_Status_t BSP_RGB_GetTiming(uint16_t *period, uint16_t *zero, uint16_t *one)
{
#if BSP_ENABLE_RGB
  if ((period == 0) || (zero == 0) || (one == 0) || (s_initialized == 0U)) { return BSP_STATUS_ERROR; }
  *period = s_period;
  *zero = s_zero;
  *one = s_one;
  return BSP_STATUS_OK;
#else
  (void)period;
  (void)zero;
  (void)one;
  return BSP_STATUS_ERROR;
#endif
}

/** @brief IRQ只记录原始事实；先截获错误以避开HAL DMA错误分支内的等待循环。 */
void BSP_RGB_DmaIrqHandler(void)
{
#if BSP_ENABLE_RGB
  uint32_t flags;
  if (s_rgb_dma.Instance == 0) { return; }
  flags = DMA2->HISR;
  if ((flags & (DMA_HISR_TEIF7 | DMA_HISR_DMEIF7 | DMA_HISR_FEIF7)) != 0U) {
    /* 只屏蔽/确认当前错误中断并留存事实；硬件撤收由下一次sensor服务执行。 */
    BSP_RGB_DMA_STREAM->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    BSP_RGB_DMA_STREAM->FCR &= ~DMA_SxFCR_FEIE;
    DMA2->HIFCR = DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7 |
                  DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;
    BspRgb_DmaError(&s_rgb_dma);
    return;
  }
  HAL_DMA_IRQHandler(&s_rgb_dma);
#endif
}

/** @brief 致命异常先固定PC9低，再关闭TIM8/DMA请求；不等待、不重置共享DMA控制器。 */
void BSP_RGB_EmergencyStop(void)
{
#if BSP_ENABLE_RGB
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIOC->BSRR = (uint32_t)GPIO_PIN_9 << 16U;
  GPIOC->OTYPER &= ~(uint32_t)GPIO_PIN_9;
  GPIOC->MODER = (GPIOC->MODER & ~(3UL << 18U)) | (1UL << 18U);
  if ((RCC->APB2ENR & RCC_APB2ENR_TIM8EN) != 0U) {
    TIM8->DIER &= ~TIM_DIER_CC4DE;
    TIM8->CCER &= ~TIM_CCER_CC4E;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    TIM8->CR1 &= ~TIM_CR1_CEN;
    TIM8->CCR4 = 0U;
  }
  if ((RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN) != 0U) {
    BSP_RGB_DMA_STREAM->CR &= ~(DMA_SxCR_EN | DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    BSP_RGB_DMA_STREAM->FCR &= ~DMA_SxFCR_FEIE;
  }
  __DSB();
#endif
}
