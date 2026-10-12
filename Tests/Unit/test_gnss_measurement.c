/** @file test_gnss_measurement.c
 * @brief 通过真实 NMEA 字节流验证各测量更新时刻独立。
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
#define BSP_CRITICAL_H
static uint32_t BSP_Critical_Enter(void) { return 0U; }
static void BSP_Critical_Exit(uint32_t value) { (void)value; }
#include "../../Sensor/Src/sensor_nmea.c"
#include "../../Sensor/Src/sensor_gnss.c"
static uint8_t input[512];
static uint16_t input_len;
uint8_t BSP_GNSS_ConsumeRecoverRxRequest(void) { return 0U; }
BSP_Status_t BSP_GNSS_RecoverRx(void) { return BSP_STATUS_OK; }
void BSP_GNSS_RxIdleCallback(uint16_t unused) { (void)unused; }
uint16_t BSP_GNSS_GetRxCount(void) { return input_len; }
uint16_t BSP_GNSS_GetRxData(uint8_t *out, uint16_t cap)
{
  uint16_t n = input_len < cap ? input_len : cap;
  memcpy(out, input, n); memmove(input, input + n, input_len - n); input_len -= n; return n;
}
static void Feed(const char *body, uint32_t now)
{
  unsigned char sum = 0U; const char *p;
  for (p = body; *p != 0; ++p) { sum ^= (unsigned char)*p; }
  input_len = (uint16_t)sprintf((char *)input, "$%s*%02X\r\n", body, sum);
  assert(Sensor_GNSS_Service(now) == GNSS_RESULT_OK);
}
int main(void)
{
  Gnss_Snapshot_t snap;
  assert(Sensor_GNSS_Init() == GNSS_RESULT_OK);
  Feed("GPGGA,000000.00,2230.0000,N,11330.0000,E,1,08,1.0,25.0,M,0.0,M,,", 100U);
  assert(Sensor_GNSS_CopySnapshot(&snap) == GNSS_RESULT_OK);
  assert(snap.last_fix_ms == 100U);
  assert(snap.position_update_ms==100U && snap.altitude_update_ms==100U);
  Feed("GPRMC,000001.00,A,2230.0000,N,11330.0000,E,10.0,90.0,280926,,,A", 200U);
  assert(Sensor_GNSS_CopySnapshot(&snap) == GNSS_RESULT_OK);
  assert(snap.last_fix_ms == 100U);
  assert(snap.velocity_update_ms==200U && snap.time_update_ms==200U);
  Feed("GPGSV,1,1,08",1000U);
  Feed("GPGSA,A,3,01,02,03,,,,,,,,,,1.0,1.0,1.0",1100U);
  assert(Sensor_GNSS_CopySnapshot(&snap)==GNSS_RESULT_OK);
  assert(snap.last_rx_ms==1100U && snap.position_update_ms==100U && snap.velocity_update_ms==200U);
  Feed("GPGGA,000002.00,2230.0000,N,11330.0000,E,1,08,1.0,26.0,M,0.0,M,,",2000U);
  assert(Sensor_GNSS_CopySnapshot(&snap)==GNSS_RESULT_OK);
  assert(snap.position_update_ms==2000U && snap.altitude_update_ms==2000U);
  assert(snap.velocity_update_ms==200U && snap.time_update_ms==200U);
  Feed("GPRMC,000000.00,A,2230.0000,N,11330.0000,E,,,280926,,,A",2100U);
  assert(Sensor_GNSS_CopySnapshot(&snap)==GNSS_RESULT_OK);
  assert((snap.valid_mask&GNSS_VALID_VELOCITY)==0U && (snap.valid_mask&GNSS_VALID_TIME)!=0U);
  assert(snap.utc_sec==0U && snap.utc_date==260928U);
  Feed("GPRMC,000000.00,A,2230.0000,N,11330.0000,E,1.0,90.0,310226,,,A",2200U);
  assert(Sensor_GNSS_CopySnapshot(&snap)==GNSS_RESULT_OK);
  assert((snap.valid_mask&GNSS_VALID_TIME)==0U);
  Feed("GPGGA,000002.00,,N,11330.0000,E,1,08,1.0,26.0,M,0.0,M,,",2300U);
  assert(Sensor_GNSS_CopySnapshot(&snap)==GNSS_RESULT_NO_FIX);
  assert((snap.valid_mask&(GNSS_VALID_POSITION|GNSS_VALID_ALTITUDE))==0U);
  assert(snap.position_update_ms==2000U);
  Feed("GPGGA,000002.00,2230.0000,N,11330.0000,E,1,08,1.0,26.0,M,0.0,M,,",2400U);
  Feed("GPRMC,000002.00,V,,,,,,,280926,,,N",2500U);
  assert(Sensor_GNSS_CopySnapshot(&snap)==GNSS_RESULT_NO_FIX && snap.fix_valid==0U);
  /* 校验错误只消耗字节，不能延长链路或测量计时。 */
  input_len=(uint16_t)sprintf((char*)input,"$GPGGA,000002.00,2230.0000,N,11330.0000,E,1,08,1.0,26.0,M,0.0,M,,*00\r\n");
  assert(Sensor_GNSS_Service(2600U)==GNSS_RESULT_OK);
  (void)Sensor_GNSS_CopySnapshot(&snap);
  assert(snap.last_rx_ms==2500U && snap.position_update_ms==2400U);
  Feed("GPGGA,000002.00,2230.0000,N,11330.0000,E,1,08,1.0,26.0,M,0.0,M,,",2700U);
  Feed("GPGSA,A,1,,,,,,,,,,,,,1.0,1.0,1.0",2800U);
  assert(Sensor_GNSS_CopySnapshot(&snap)==GNSS_RESULT_NO_FIX);
  assert((snap.valid_mask&GNSS_VALID_POSITION)==0U && snap.position_update_ms==2700U);
  puts("gnss measurement tests passed"); return 0;
}
