/** @file test_m2_touch_recovery.c @brief 触摸初始失败仍注册 LVGL 输入。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../Display/Src/lv_port_indev.c"
#include "../../Display/Src/display_gt911.c"
static unsigned registrations, register_fail;
static lv_indev_t fake_indev;
void lv_indev_drv_init(lv_indev_drv_t *d) { memset(d,0,sizeof(*d)); }
lv_indev_t *lv_indev_drv_register(lv_indev_drv_t *d) { (void)d;registrations++;return register_fail?0:&fake_indev; }
static unsigned present, read_fail, write_fail, init_calls, reads;
static uint8_t valid_addr=0x14;
BSP_TouchPortResult_t BSP_TouchPort_Init(void) { init_calls++;return BSP_TOUCH_PORT_OK; }
void BSP_TouchPort_WriteReset(uint8_t v) { (void)v; }
void BSP_TouchPort_SetIntOutput(uint8_t v) { (void)v; }
void BSP_TouchPort_SetIntInput(void) {}
void BSP_TouchPort_Recover(void) {}
uint8_t BSP_TouchPort_GetLastError(void) { return 7; }
BSP_TouchPortResult_t BSP_TouchPort_ReadReg16(uint8_t addr,uint16_t reg,uint8_t *buf,uint8_t len) {
  reads++;memset(buf,0,len);
  if(!present || read_fail || addr!=valid_addr) return BSP_TOUCH_PORT_ERROR;
  if(reg==DISPLAY_GT911_REG_PID) memcpy(buf,"911",4);
  if(reg==DISPLAY_GT911_REG_TP_INFO) buf[0]=0x81;
  if(reg==DISPLAY_GT911_REG_TP1) { buf[0]=42;buf[2]=31; }
  return BSP_TOUCH_PORT_OK;
}
BSP_TouchPortResult_t BSP_TouchPort_WriteReg16(uint8_t a,uint16_t r,const uint8_t*b,uint8_t n) {
  (void)a;(void)r;(void)b;(void)n;return write_fail?BSP_TOUCH_PORT_ERROR:BSP_TOUCH_PORT_OK;
}
static void Advance(uint32_t from,uint32_t until) {
  uint32_t n;for(n=from;n<until;n+=10) Display_Gt911_Service(n);
}
int main(void) {
  lv_indev_data_t data;Display_Gt911Status_t status;uint32_t deadline;unsigned old_reads;
  assert(LvPortIndev_Init()==1 && registrations==1);
  assert(LvPortIndev_Init()==1 && registrations==1);
  Advance(0,400);Display_Gt911_GetStatus(&status);
  assert(!status.ready && status.recovering && s_gt911_phase==GT911_BACKOFF);
  assert(init_calls==1 && registrations==1);
  LvPortIndev_Read(0,&data);assert(data.state==LV_INDEV_STATE_REL);
  deadline=s_gt911_deadline_ms;old_reads=reads;present=1;
  Display_Gt911_Service(deadline-1);assert(reads==old_reads && init_calls==1);
  Advance(deadline,deadline+400);Display_Gt911_GetStatus(&status);
  assert(status.ready && !status.recovering && status.last_error==0 && status.recovery_count==2);
  LvPortIndev_Read(0,&data);assert(data.state==LV_INDEV_STATE_PR && data.point.x==42 && data.point.y==31);
  read_fail=1;LvPortIndev_Read(0,&data);assert(data.state==LV_INDEV_STATE_REL);
  Display_Gt911_GetStatus(&status);assert(!status.ready && status.last_error==7);
  deadline=s_gt911_deadline_ms;read_fail=0;valid_addr=0x5d;
  Advance(deadline,deadline+500);assert(s_gt911_addr==0x5d && registrations==1);
  write_fail=1;LvPortIndev_Read(0,&data);assert(data.state==LV_INDEV_STATE_REL && s_gt911_phase==GT911_BACKOFF);
  write_fail=0;s_gt911_now_ms=UINT32_MAX-99;Display_Gt911_Fail();
  Display_Gt911_Service(1899);assert(s_gt911_phase==GT911_BACKOFF);
  Display_Gt911_Service(1900);assert(s_gt911_phase==GT911_START);
  s_indev=0;s_registration_attempted=0;register_fail=1;
  assert(LvPortIndev_Init()==0 && registrations==2);
  assert(LvPortIndev_Init()==0 && registrations==2);
  puts("m2_touch_recovery: PASS (startup absent, hotplug, read/write failure, both addresses, retry/wrap)");return 0;
}
