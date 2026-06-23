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
static uint16_t s_total_read;
static uint32_t s_now_ms;

int32_t BSP_LoRa_Init(void) { return 0; }
UART_HandleTypeDef *BSP_LoRa_GetUartHandle(void) { return 0; }
void BSP_LoRa_SetMode(uint8_t m) { (void)m; }
uint8_t BSP_LoRa_IsReady(void) { return 1U; }
uint8_t BSP_LoRa_IsBusy(void) { return 0U; }
uint32_t BSP_LoRa_GetRxOverflowCount(void) { return 0U; }
int32_t BSP_LoRa_StartSend(const uint8_t *data, uint16_t len)
{
  (void)data;
  (void)len;
  return 0;
}
uint8_t BSP_LoRa_IsTxBusy(void) { return 0U; }
void BSP_LoRa_AbortTx(void) { }
void BSP_LoRa_TxDmaIrqHandler(void) { }
void BSP_LoRa_RxIdleCallback(uint16_t dummy) { (void)dummy; }
void BSP_LoRa_DmaIrqHandler(void) { }
void BSP_LoRa_RecoverRx(void) { }
uint32_t BSP_Time_GetTickMs(void) { return s_now_ms; }
void BSP_Time_DelayMs(uint32_t delay_ms) { s_now_ms += delay_ms; }

uint16_t BSP_LoRa_GetRxCount(void)
{
  return (uint16_t)(TEST_RX_TOTAL_BYTES - s_rx_read_pos);
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

static int TestServiceLeavesRxBacklogForNextCycle(void)
{
  memset(s_rx_data, 0x55, sizeof(s_rx_data));
  s_rx_read_pos = 0U;
  s_total_read = 0U;
  s_now_ms = 1000U;

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

int main(void)
{
  int failures = 0;

  failures += TestServiceLeavesRxBacklogForNextCycle();

  if (failures != 0) {
    printf("lora e22 service budget tests failed: %d\n", failures);
    return 1;
  }

  printf("lora e22 service budget tests passed\n");
  return 0;
}
