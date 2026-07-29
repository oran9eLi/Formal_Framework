/**
 * @file bsp_watchdog.c
 * @brief 实现独立看门狗(IWDG)启动、刷新与复位原因捕获。
 *
 * @details
 * IWDG 只有 KR/PR/RLR/SR 四个寄存器，直接寄存器访问即可完整覆盖，故本文件不引入
 * HAL 的 IWDG 模块，避免为一个外设改动 `stm32f4xx_hal_conf.h` 与 Keil 工程文件。
 * 本文件是全工程唯一访问 IWDG 的位置。
 */

#include "bsp_watchdog.h"

#include "bsp_config.h"

/* IWDG 键值寄存器命令字，见 RM0090 21.4.1。 */
#define BSP_IWDG_KEY_RELOAD   0x0000AAAAU /**< 重装计数器（喂狗）。 */
#define BSP_IWDG_KEY_ENABLE   0x0000CCCCU /**< 启动 IWDG，并自动使能 LSI。 */
#define BSP_IWDG_KEY_WRITE    0x00005555U /**< 解除 PR/RLR 写保护。 */

/* 等待 PR/RLR 更新完成的自旋上限。LSI 最慢 17 kHz 时单次更新约需数个 LSI 周期，
   此处取足够大的常数只为防止硬件异常时死循环，正常路径远达不到上限。 */
#define BSP_IWDG_SYNC_TIMEOUT 100000U

static uint8_t s_reset_cause = BSP_RESET_CAUSE_NONE;
static uint8_t s_started;

void BSP_Watchdog_CaptureResetCause(void)
{
  uint32_t csr;
  uint8_t cause = BSP_RESET_CAUSE_NONE;

  csr = RCC->CSR;

  if ((csr & RCC_CSR_IWDGRSTF) != 0U) { cause |= BSP_RESET_CAUSE_IWDG; }
  if ((csr & RCC_CSR_WWDGRSTF) != 0U) { cause |= BSP_RESET_CAUSE_WWDG; }
  if ((csr & RCC_CSR_SFTRSTF) != 0U)  { cause |= BSP_RESET_CAUSE_SOFT; }
  if ((csr & RCC_CSR_PORRSTF) != 0U)  { cause |= BSP_RESET_CAUSE_POR; }
  if ((csr & RCC_CSR_PINRSTF) != 0U)  { cause |= BSP_RESET_CAUSE_PIN; }
  if ((csr & RCC_CSR_BORRSTF) != 0U)  { cause |= BSP_RESET_CAUSE_BOR; }
  if ((csr & RCC_CSR_LPWRRSTF) != 0U) { cause |= BSP_RESET_CAUSE_LPWR; }

  s_reset_cause = cause;

  /* 清标志，否则下次复位时读到的是本次与历史的并集，无法判断真实原因。 */
  RCC->CSR |= RCC_CSR_RMVF;
}

uint8_t BSP_Watchdog_GetResetCause(void)
{
  return s_reset_cause;
}

/**
 * @brief 自旋等待 IWDG_SR 中指定的更新标志清零。
 *
 * @param[in] flag 待等待的标志位（`IWDG_SR_PVU` 或 `IWDG_SR_RVU`）。
 *
 * @return 1 表示标志已清零，0 表示超时。
 */
static uint8_t Bsp_WatchdogWaitSync(uint32_t flag)
{
  uint32_t guard;

  for (guard = 0U; guard < BSP_IWDG_SYNC_TIMEOUT; ++guard) {
    if ((IWDG->SR & flag) == 0U) { return 1U; }
  }
  return 0U;
}

BSP_Status_t BSP_Watchdog_Start(void)
{
  if (s_started != 0U) { return BSP_STATUS_OK; }

#if BSP_WATCHDOG_FREEZE_ON_DEBUG
  /* 调试器挂起内核时冻结 IWDG，否则断点一停就复位，无法单步。
     STM32F4 的 DBGMCU 常供时钟，无需像 F1 那样先使能 APB2 时钟。 */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#endif

  /* 启动 IWDG，该命令同时自动使能 LSI，无需另行配置 RCC。 */
  IWDG->KR = BSP_IWDG_KEY_ENABLE;

  /* 解除写保护后配置分频与重装值；每次写入前必须等待对应更新标志清零。 */
  IWDG->KR = BSP_IWDG_KEY_WRITE;

  if (Bsp_WatchdogWaitSync(IWDG_SR_PVU) == 0U) { return BSP_STATUS_TIMEOUT; }
  IWDG->PR = BSP_WATCHDOG_PRESCALER_CODE;

  if (Bsp_WatchdogWaitSync(IWDG_SR_RVU) == 0U) { return BSP_STATUS_TIMEOUT; }
  IWDG->RLR = BSP_WATCHDOG_RELOAD;

  /* 等待两个值都装载完成后再喂一次，确保首个超时窗口按新配置计时。 */
  if (Bsp_WatchdogWaitSync(IWDG_SR_PVU | IWDG_SR_RVU) == 0U) { return BSP_STATUS_TIMEOUT; }
  IWDG->KR = BSP_IWDG_KEY_RELOAD;

  s_started = 1U;
  return BSP_STATUS_OK;
}

uint8_t BSP_Watchdog_IsStarted(void)
{
  return s_started;
}

void BSP_WatchdogRefresh(void)
{
  if (s_started == 0U) { return; }
  IWDG->KR = BSP_IWDG_KEY_RELOAD;
}
