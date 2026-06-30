/**
 * @file test_lora_e22_presence.c
 * @brief 验证 E22 在位检测(s_present)不会被初始化期间的瞬时 AUX 忙永久闩锁为"未接入"。
 *
 * @details
 * 复现回归:lvgl 接入 LoRa 远程显示后，CommWorkRun 用 Lora_E22_IsPresent() 硬门控收发，
 * 一旦在位标志卡在 0，接收端整段 RX 被跳过、状态灯红(FAILED)、收不到任何远端消息。
 * 根因在 Lora_E22_Init:ResetRuntimeState 先把 s_initialized 置 1，s_present 才由
 * AuxPresent() 30ms 采样决定；若该窗口内 AUX 恰好处于瞬时忙(低)，得到
 * initialized=1 且 present=0，而 Service 的补救分支以 s_initialized==0 为前提，
 * 永远不再翻回 1。本测试在该时序下要求"模块仍判为在位"。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp_lora.h"
#include "bsp_time.h"
#include "lora_e22.h"

/* ---- 可控时钟与 AUX 桩 ---- */
static uint32_t s_now_ms;
/* IsReady 按调用序控制:落在 [low_from, low_to) 的调用返回忙(0)，其余返回就绪(1)。
   这样可以精确制造"init 等待环看到就绪、紧接着的 AuxPresent 采样窗口却忙"的时序。 */
static uint32_t s_ready_calls;
static uint32_t s_ready_low_from;
static uint32_t s_ready_low_to;

static uint8_t s_uart_tx_busy;

uint32_t BSP_Time_GetTickMs(void) { return s_now_ms++; }
void     BSP_Time_DelayMs(uint32_t delay_ms) { s_now_ms += delay_ms; }

int32_t  BSP_LoRa_Init(void) { return 0; }
UART_HandleTypeDef *BSP_LoRa_GetUartHandle(void) { return 0; }
void     BSP_LoRa_SetMode(uint8_t m) { (void)m; }

uint8_t BSP_LoRa_IsReady(void)
{
  uint32_t n = s_ready_calls++;
  if ((n >= s_ready_low_from) && (n < s_ready_low_to)) { return 0U; }
  return 1U;
}
uint8_t  BSP_LoRa_IsBusy(void) { return (uint8_t)(BSP_LoRa_IsReady() != 0U ? 0U : 1U); }
uint32_t BSP_LoRa_GetUartBaud(void) { return 9600U; }
uint32_t BSP_LoRa_GetAirBps(void) { return 2400U; }
uint32_t BSP_LoRa_GetRxOverflowCount(void) { return 0U; }

int32_t BSP_LoRa_StartSend(const uint8_t *data, uint16_t len)
{
  (void)data; (void)len;
  s_uart_tx_busy = 1U;
  return 0;
}
uint8_t BSP_LoRa_IsTxBusy(void) { return s_uart_tx_busy; }
void    BSP_LoRa_AbortTx(void) { s_uart_tx_busy = 0U; }
void    BSP_LoRa_TxDmaIrqHandler(void) { }
void    BSP_LoRa_RxIdleCallback(uint16_t dummy) { (void)dummy; }
void    BSP_LoRa_DmaIrqHandler(void) { }
void    BSP_LoRa_RecoverRx(void) { }

/* 接收始终为空:本测试只关心在位标志，不喂 RX 数据。 */
uint16_t BSP_LoRa_GetRxCount(void) { return 0U; }
uint16_t BSP_LoRa_GetRxData(uint8_t *dst, uint16_t max_len) { (void)dst; (void)max_len; return 0U; }

static int ExpectU32(const char *name, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)actual, (unsigned long)expected);
    return 1;
  }
  return 0;
}

static int ExpectResult(const char *name, Lora_Result_t actual, Lora_Result_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%d expected=%d\n", name, (int)actual, (int)expected);
    return 1;
  }
  return 0;
}

/* AUX 上电稳定就绪:在位检测必须判为在位。回归护栏，修复前后都应通过。 */
static int TestSteadyReadyIsPresent(void)
{
  int failures = 0;

  s_now_ms = 2000U;
  s_ready_calls = 0U;
  s_ready_low_from = 0U; s_ready_low_to = 0U; /* 空窗口 -> 始终就绪 */

  failures += ExpectResult("steady init ok", Lora_E22_Init(), LORA_RESULT_OK);
  failures += ExpectU32("steady present", Lora_E22_IsPresent(), 1U);
  return failures;
}

/* 真正未接入:AUX 恒低，init 超时返回 BUSY，在位判为 0。 */
static int TestNeverReadyIsAbsent(void)
{
  int failures = 0;

  s_now_ms = 3000U;
  s_ready_calls = 0U;
  s_ready_low_from = 0U; s_ready_low_to = 0xFFFFFFFFU; /* 所有调用都忙 */

  failures += ExpectResult("absent init busy", Lora_E22_Init(), LORA_RESULT_BUSY);
  failures += ExpectU32("absent present 0", Lora_E22_IsPresent(), 0U);
  return failures;
}

/* 回归核心:init 等待环已确认 AUX 就绪(模块在位)，但紧接着的采样窗口 AUX 瞬时忙。
   绝不能因此把在位模块永久判为未接入而关闭收发。 */
static int TestTransientAuxBusyAtInitKeepsPresent(void)
{
  int failures = 0;

  s_now_ms = 1000U;
  s_ready_calls = 0U;
  /* 第 0 次 IsReady(等待环)就绪 -> 退出等待;第 1..99 次(采样窗口)瞬时忙。 */
  s_ready_low_from = 1U; s_ready_low_to = 100U;

  failures += ExpectResult("transient init ok", Lora_E22_Init(), LORA_RESULT_OK);

  /* 运行期 AUX 恢复就绪,推进一个 Service 周期。 */
  s_ready_low_from = 0U; s_ready_low_to = 0U; /* 始终就绪 */
  failures += ExpectResult("transient service ok", Lora_E22_Service(s_now_ms), LORA_RESULT_OK);

  /* 在位模块经历瞬时忙后必须仍判为在位,否则收发被永久关闭。 */
  failures += ExpectU32("present after transient busy", Lora_E22_IsPresent(), 1U);
  return failures;
}

int main(void)
{
  int failures = 0;

  failures += TestSteadyReadyIsPresent();
  failures += TestNeverReadyIsAbsent();
  failures += TestTransientAuxBusyAtInitKeepsPresent();

  if (failures == 0) {
    printf("PASS test_lora_e22_presence\n");
    return 0;
  }
  printf("FAIL test_lora_e22_presence failures=%d\n", failures);
  return 1;
}
