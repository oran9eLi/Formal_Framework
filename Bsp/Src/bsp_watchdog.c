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
static uint8_t s_runtime;
static uint8_t s_failed;
static uint8_t s_boot_sealed;

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

/** @brief 更新 IWDG 窗口；仅在同步成功后刷新，失败不会继续续期。 */
static BSP_Status_t Bsp_WatchdogSetWindow(uint32_t prescaler, uint32_t reload)
{
  IWDG->KR = BSP_IWDG_KEY_WRITE;
  if (Bsp_WatchdogWaitSync(IWDG_SR_PVU) == 0U) { return BSP_STATUS_TIMEOUT; }
  IWDG->PR = prescaler;
  if (Bsp_WatchdogWaitSync(IWDG_SR_RVU) == 0U) { return BSP_STATUS_TIMEOUT; }
  IWDG->RLR = reload;
  if (Bsp_WatchdogWaitSync(IWDG_SR_PVU | IWDG_SR_RVU) == 0U) { return BSP_STATUS_TIMEOUT; }
  IWDG->KR = BSP_IWDG_KEY_RELOAD;
  return BSP_STATUS_OK;
}

/** @brief main 最早期启用启动窗口；不使用 HAL tick 或 RTOS。 */
BSP_Status_t BSP_Watchdog_StartBoot(void)
{
  BSP_Status_t result;
  if (s_failed != 0U) { return BSP_STATUS_ERROR; }
  if (s_started != 0U) { return BSP_STATUS_OK; }
#if BSP_WATCHDOG_FREEZE_ON_DEBUG
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#else
  DBGMCU->APB1FZ &= ~DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#endif
  IWDG->KR = BSP_IWDG_KEY_ENABLE;
  s_started = 1U;
  s_failed = 1U;
  result = Bsp_WatchdogSetWindow(BSP_WATCHDOG_BOOT_PRESCALER_CODE, BSP_WATCHDOG_BOOT_RELOAD);
  if (result == BSP_STATUS_OK) { s_failed = 0U; }
  return result;
}

/** @brief 只允许调度前有限初始化进展点刷新启动窗口；失败后无效。 */
void BSP_Watchdog_BootProgress(void)
{
  if ((s_started != 0U) && (s_failed == 0U) && (s_boot_sealed == 0U) && (s_runtime == 0U)) {
    IWDG->KR = BSP_IWDG_KEY_RELOAD;
  }
}

/** @brief 调度器交接或任一初始化失败后，永久关闭本次启动的进展刷新。 */
void BSP_Watchdog_SealBootProgress(void)
{
  s_boot_sealed = 1U;
}

/** @brief 收齐首轮心跳后收紧窗口；失败锁存，重复成功调用不重新配置。 */
BSP_Status_t BSP_Watchdog_EnterRuntime(void)
{
  BSP_Status_t result;
  if ((s_started == 0U) || (s_failed != 0U) || (s_boot_sealed == 0U)) { return BSP_STATUS_ERROR; }
  if (s_runtime != 0U) { return BSP_STATUS_OK; }
  s_failed = 1U;
  result = Bsp_WatchdogSetWindow(BSP_WATCHDOG_PRESCALER_CODE, BSP_WATCHDOG_RELOAD);
  if (result == BSP_STATUS_OK) {
    s_runtime = 1U;
    s_failed = 0U;
  }
  return result;
}

/** @brief 查询硬件是否已发出启动命令，不能据此推断运行窗口已配置成功。 */
uint8_t BSP_Watchdog_IsStarted(void)
{
  return s_started;
}

/** @brief 仅运行配置成功后刷新；调用者必须先通过任务健康门控。 */
void BSP_WatchdogRefresh(void)
{
  if ((s_runtime != 0U) && (s_failed == 0U)) { IWDG->KR = BSP_IWDG_KEY_RELOAD; }
}
