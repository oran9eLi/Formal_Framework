/**
 * @file test_lora_e22_service_budget.c
 * @brief 验证 LoRa 驱动单次 Service 不会无界清空 RX 积压。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp_lora.h"
#include "bsp_time.h"
#include "lora_e22.h"

#define TEST_RX_TOTAL_BYTES 512U
#define TEST_RX_SERVICE_BUDGET 128U

static uint8_t s_rx_data[TEST_RX_TOTAL_BYTES];
static uint16_t s_rx_read_pos;
static uint16_t s_rx_available_len;
static uint16_t s_total_read;
static uint32_t s_now_ms;
static uint8_t s_lora_ready;
static uint8_t s_uart_tx_busy;
static uint16_t s_start_send_count;
static uint16_t s_abort_count;

int32_t BSP_LoRa_Init(void) { return 0; }
UART_HandleTypeDef *BSP_LoRa_GetUartHandle(void) { return 0; }
void BSP_LoRa_SetMode(uint8_t m) { (void)m; }
uint8_t BSP_LoRa_IsReady(void) { return s_lora_ready; }
uint8_t BSP_LoRa_IsBusy(void) { return (s_lora_ready != 0U) ? 0U : 1U; }
uint32_t BSP_LoRa_GetUartBaud(void) { return 9600U; }
uint32_t BSP_LoRa_GetAirBps(void) { return 2400U; }
uint32_t BSP_LoRa_GetRxOverflowCount(void) { return 0U; }
int32_t BSP_LoRa_StartSend(const uint8_t *data, uint16_t len)
{
  (void)data;
  (void)len;
  s_start_send_count++;
  s_uart_tx_busy = 1U;
  return 0;
}
uint8_t BSP_LoRa_IsTxBusy(void) { return s_uart_tx_busy; }
void BSP_LoRa_AbortTx(void)
{
  s_abort_count++;
  s_uart_tx_busy = 0U;
}
void BSP_LoRa_TxDmaIrqHandler(void) { }
void BSP_LoRa_RxIdleCallback(uint16_t dummy) { (void)dummy; }
void BSP_LoRa_DmaIrqHandler(void) { }
void BSP_LoRa_RecoverRx(void) { }
uint32_t BSP_Time_GetTickMs(void)
{
  return s_now_ms++;
}
void BSP_Time_DelayMs(uint32_t delay_ms) { s_now_ms += delay_ms; }

uint16_t BSP_LoRa_GetRxCount(void)
{
  return (uint16_t)(s_rx_available_len - s_rx_read_pos);
}

uint16_t BSP_LoRa_GetRxData(uint8_t *dst, uint16_t max_len)
{
  uint16_t remaining;
  uint16_t count;

  if ((dst == 0) || (max_len == 0U)) { return 0U; }
  remaining = BSP_LoRa_GetRxCount();
  count = (remaining > max_len) ? max_len : remaining;
  memcpy(dst, &s_rx_data[s_rx_read_pos], count);
  s_rx_read_pos = (uint16_t)(s_rx_read_pos + count);
  s_total_read = (uint16_t)(s_total_read + count);
  return count;
}

static int ExpectU16(const char *name, uint16_t actual, uint16_t expected)
{
  if (actual != expected) {
    printf("FAIL %s actual=%u expected=%u\n", name, actual, expected);
    return 1;
  }
  return 0;
}

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

static int TestServiceLeavesRxBacklogForNextCycle(void)
{
  memset(s_rx_data, 0x55, sizeof(s_rx_data));
  s_rx_read_pos = 0U;
  s_rx_available_len = TEST_RX_TOTAL_BYTES;
  s_total_read = 0U;
  s_now_ms = 1000U;
  s_lora_ready = 1U;
  s_uart_tx_busy = 0U;
  s_start_send_count = 0U;
  s_abort_count = 0U;

  if (Lora_E22_Init() != LORA_RESULT_OK) {
    printf("FAIL init\n");
    return 1;
  }

  if (Lora_E22_Service(1010U) != LORA_RESULT_OK) {
    printf("FAIL service\n");
    return 1;
  }

  return ExpectU16("service rx byte budget", s_total_read, TEST_RX_SERVICE_BUDGET);
}

static int TestAirRateBudgetKeepsAuxWaitDuringSlowAirTransfer(void)
{
  uint8_t frame[32];
  int failures = 0;

  memset(frame, 0xA5, sizeof(frame));
  s_rx_read_pos = 0U;
  s_rx_available_len = 0U;
  s_total_read = 0U;
  s_now_ms = 2000U;
  s_lora_ready = 1U;
  s_uart_tx_busy = 0U;
  s_start_send_count = 0U;
  s_abort_count = 0U;

  if (Lora_E22_Init() != LORA_RESULT_OK) {
    printf("FAIL init slow air\n");
    return 1;
  }

  s_lora_ready = 0U;
  failures += ExpectResult("first send waits aux", Lora_E22_Send(frame, sizeof(frame)), LORA_RESULT_OK);
  failures += ExpectU32("no uart send while aux busy", s_start_send_count, 0U);

  failures += ExpectResult("service before air timeout", Lora_E22_Service(2300U), LORA_RESULT_OK);
  failures += ExpectResult("pending frame still busy", Lora_E22_Send(frame, sizeof(frame)), LORA_RESULT_BUSY);
  failures += ExpectU32("still no uart send before aux ready", s_start_send_count, 0U);
  return failures;
}

static int TestUartDmaTimeoutUsesConfiguredBaudrate(void)
{
  uint8_t frame[LORA_E22_TX_BUF_SIZE];
  int failures = 0;

  memset(frame, 0x5A, sizeof(frame));
  s_rx_read_pos = 0U;
  s_rx_available_len = 0U;
  s_total_read = 0U;
  s_now_ms = 3000U;
  s_lora_ready = 1U;
  s_uart_tx_busy = 0U;
  s_start_send_count = 0U;
  s_abort_count = 0U;

  if (Lora_E22_Init() != LORA_RESULT_OK) {
    printf("FAIL init uart timeout\n");
    return 1;
  }

  failures += ExpectResult("start long uart frame", Lora_E22_Send(frame, sizeof(frame)), LORA_RESULT_OK);
  failures += ExpectU32("uart dma started", s_start_send_count, 1U);

  failures += ExpectResult("service before uart timeout", Lora_E22_Service(3200U), LORA_RESULT_OK);
  failures += ExpectResult("uart frame still busy", Lora_E22_Send(frame, sizeof(frame)), LORA_RESULT_BUSY);
  failures += ExpectU32("no abort before calculated uart timeout", s_abort_count, 0U);

  failures += ExpectResult("service after uart timeout", Lora_E22_Service(3400U), LORA_RESULT_OK);
  failures += ExpectU32("abort after calculated uart timeout", s_abort_count, 1U);
  return failures;
}

static int TestRuntimeReinitDoesNotWaitForMissingAux(void)
{
  int failures = 0;
  uint32_t before_ms;

  s_rx_read_pos = 0U;
  s_rx_available_len = 0U;
  s_total_read = 0U;
  s_now_ms = 4000U;
  s_lora_ready = 1U;
  s_uart_tx_busy = 0U;
  s_start_send_count = 0U;
  s_abort_count = 0U;

  if (Lora_E22_Init() != LORA_RESULT_OK) {
    printf("FAIL init runtime reinit\n");
    return 1;
  }

  s_lora_ready = 0U;
  before_ms = s_now_ms;
  Lora_E22_RequestReinit();
  failures += ExpectResult("runtime reinit service returns", Lora_E22_Service(4010U), LORA_RESULT_OK);
  failures += ExpectU32("runtime reinit no aux wait", s_now_ms, before_ms);
  failures += ExpectU32("runtime reinit no uart send", s_start_send_count, 0U);
  return failures;
}

static int TestSendBeforeSuccessfulInitDoesNotStartTx(void)
{
  uint8_t frame[8];
  int failures = 0;

  memset(frame, 0x33, sizeof(frame));
  s_rx_read_pos = 0U;
  s_rx_available_len = 0U;
  s_total_read = 0U;
  s_now_ms = 4500U;
  s_lora_ready = 0U;
  s_uart_tx_busy = 0U;
  s_start_send_count = 0U;
  s_abort_count = 0U;

  failures += ExpectResult("init without aux busy", Lora_E22_Init(), LORA_RESULT_BUSY);
  failures += ExpectResult("send before init fails", Lora_E22_Send(frame, sizeof(frame)), LORA_RESULT_IO_ERROR);
  failures += ExpectU32("send before init no uart send", s_start_send_count, 0U);
  return failures;
}

static int TestHardwareStateDoesNotDependOnTraffic(void)
{
  int failures = 0;

  s_rx_read_pos = 0U;
  s_rx_available_len = 0U;
  s_total_read = 0U;
  s_now_ms = 5000U;
  s_lora_ready = 1U;
  s_uart_tx_busy = 0U;
  s_start_send_count = 0U;
  s_abort_count = 0U;

  if (Lora_E22_Init() != LORA_RESULT_OK) {
    printf("FAIL init hardware state\n");
    return 1;
  }

  failures += ExpectU32("initialized hardware online without traffic", Lora_E22_GetState(9000U, 3000U), LORA_STATE_ONLINE);
  return failures;
}

static int TestAuxLowTimeoutMarksHardwareOfflineAndRecovers(void)
{
  int failures = 0;

  s_rx_read_pos = 0U;
  s_rx_available_len = 0U;
  s_total_read = 0U;
  s_now_ms = 6000U;
  s_lora_ready = 1U;
  s_uart_tx_busy = 0U;
  s_start_send_count = 0U;
  s_abort_count = 0U;

  if (Lora_E22_Init() != LORA_RESULT_OK) {
    printf("FAIL init aux state\n");
    return 1;
  }

  failures += ExpectResult("service with aux ready", Lora_E22_Service(6100U), LORA_RESULT_OK);
  s_lora_ready = 0U;
  failures += ExpectResult("service short aux low", Lora_E22_Service(6200U), LORA_RESULT_OK);
  failures += ExpectU32("short aux low keeps online", Lora_E22_GetState(7000U, 3000U), LORA_STATE_ONLINE);
  failures += ExpectU32("long aux low offline", Lora_E22_GetState(9301U, 3000U), LORA_STATE_OFFLINE);
  s_lora_ready = 1U;
  failures += ExpectResult("service after aux recovers", Lora_E22_Service(9400U), LORA_RESULT_OK);
  failures += ExpectU32("aux high recovers online", Lora_E22_GetState(9400U, 3000U), LORA_STATE_ONLINE);
  return failures;
}

int main(void)
{
  int failures = 0;

  failures += TestServiceLeavesRxBacklogForNextCycle();
  failures += TestAirRateBudgetKeepsAuxWaitDuringSlowAirTransfer();
  failures += TestUartDmaTimeoutUsesConfiguredBaudrate();
  failures += TestRuntimeReinitDoesNotWaitForMissingAux();
  failures += TestSendBeforeSuccessfulInitDoesNotStartTx();
  failures += TestHardwareStateDoesNotDependOnTraffic();
  failures += TestAuxLowTimeoutMarksHardwareOfflineAndRecovers();

  if (failures != 0) {
    printf("lora e22 service budget tests failed: %d\n", failures);
    return 1;
  }

  printf("lora e22 service budget tests passed\n");
  return 0;
}
