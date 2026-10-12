/** @file test_watchdog_startup.c @brief 真实 IWDG BSP 的阶段、窗口及同步失败回归。 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef struct { volatile uint32_t KR, PR, RLR, SR; } IWDG_TypeDef;
typedef struct { uint32_t CSR; } RCC_TypeDef;
typedef struct { uint32_t APB1FZ; } DBGMCU_TypeDef;
static IWDG_TypeDef iwdg;
static RCC_TypeDef rcc;
static DBGMCU_TypeDef dbg;
#define IWDG (&iwdg)
#define RCC (&rcc)
#define DBGMCU (&dbg)
#define RCC_CSR_IWDGRSTF 1U
#define RCC_CSR_WWDGRSTF 2U
#define RCC_CSR_SFTRSTF 4U
#define RCC_CSR_PORRSTF 8U
#define RCC_CSR_PINRSTF 16U
#define RCC_CSR_BORRSTF 32U
#define RCC_CSR_LPWRRSTF 64U
#define RCC_CSR_RMVF 128U
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP 1U
#define IWDG_SR_PVU 1U
#define IWDG_SR_RVU 2U
#include "../../Bsp/Src/bsp_watchdog.c"
int main(void)
{
  assert(BSP_Watchdog_StartBoot() == BSP_STATUS_OK);
  assert(BSP_Watchdog_IsStarted());
  assert(iwdg.PR == 6U && iwdg.RLR == 4095U);
  iwdg.KR = 0U; BSP_WatchdogRefresh(); assert(iwdg.KR == 0U);
  BSP_Watchdog_BootProgress(); assert(iwdg.KR == 0xaaaaU);
  BSP_Watchdog_SealBootProgress();
  iwdg.KR = 0U; BSP_Watchdog_BootProgress(); assert(iwdg.KR == 0U);
  assert(BSP_Watchdog_EnterRuntime() == BSP_STATUS_OK);
  assert(iwdg.PR == 4U && iwdg.RLR == 1249U);
  iwdg.KR = 0U; BSP_WatchdogRefresh(); assert(iwdg.KR == 0xaaaaU);
  assert(BSP_Watchdog_StartBoot() == BSP_STATUS_OK);
  assert(iwdg.PR == 4U && iwdg.RLR == 1249U);
  /* 模拟下次冷启动的硬件同步失败：启动命令已发出，但不能喂狗或假报成功。 */
  s_started = s_runtime = s_failed = s_boot_sealed = 0U;
  iwdg.SR = IWDG_SR_PVU;
  assert(BSP_Watchdog_StartBoot() == BSP_STATUS_TIMEOUT);
  assert(BSP_Watchdog_IsStarted());
  iwdg.SR = 0U; iwdg.KR = 0U;
  BSP_Watchdog_BootProgress(); BSP_WatchdogRefresh();
  assert(iwdg.KR == 0U);
  assert(BSP_Watchdog_StartBoot() != BSP_STATUS_OK);
  /* 启动成功而运行窗口切换失败，同样永久禁止后续刷新。 */
  s_started = s_runtime = s_failed = s_boot_sealed = 0U;
  assert(BSP_Watchdog_StartBoot() == BSP_STATUS_OK);
  BSP_Watchdog_SealBootProgress(); iwdg.SR = IWDG_SR_RVU;
  assert(BSP_Watchdog_EnterRuntime() == BSP_STATUS_TIMEOUT);
  iwdg.SR = 0U; iwdg.KR = 0U;
  BSP_Watchdog_BootProgress(); BSP_WatchdogRefresh();
  assert(iwdg.KR == 0U);
  assert(BSP_Watchdog_EnterRuntime() != BSP_STATUS_OK);
  puts("Watchdog startup handoff tests passed");
  return 0;
}
