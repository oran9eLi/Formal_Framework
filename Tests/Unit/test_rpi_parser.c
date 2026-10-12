/** @file test_rpi_parser.c @brief 真实 F407 平台 RPi 解析器初始化、切速率与断流恢复回归。 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { uint32_t CNT, SR; } HostTimer;
static HostTimer host_timer;
#define TIM6 (&host_timer)
#define TIM_SR_UIF 1U
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static uint32_t HAL_GetTick(void) { return 0; }
static uint32_t HAL_GetUIDw0(void) { return 0; }
static uint32_t HAL_GetUIDw1(void) { return 0; }
static uint32_t HAL_GetUIDw2(void) { return 0; }
#include "../../Framework/Src/px4lite_platform_f407.c"

static const uint8_t *input;
static uint16_t input_size;
static BSP_Status_t baud_result=BSP_STATUS_OK;
BSP_Status_t BSP_RpiUART_Init(void) { return BSP_STATUS_OK; }
BSP_Status_t BSP_RpiUART_SetBaudRate(uint32_t rate) { (void)rate; return baud_result; }
uint16_t BSP_RpiUART_Read(uint8_t *data,uint16_t size)
{
  if(size>input_size) { size=input_size; }
  memcpy(data,input,size); input+=size; input_size-=size; return size;
}
static void Feed(const uint8_t *frame,uint16_t len,uint32_t now)
{
  input=frame; input_size=len;
  assert(Px4Lite_RpiMavlinkService(now)==PX4LITE_OK);
  assert(input_size==0U);
}
int main(void)
{
  mavlink_message_t msg;
  uint8_t frame[300];
  uint16_t len;
  Px4Lite_CommRxFrame_t out;
  mavlink_msg_heartbeat_pack_chan(42,191,MAVLINK_COMM_1,&msg,2,8,0,0,4);
  len=mavlink_msg_to_send_buffer(frame,&msg);
  assert(Px4Lite_RpiMavlinkInit()==PX4LITE_OK);
  Feed(frame,11,0U);
  assert(Px4Lite_RpiMavlinkSetBaudRate(57600U)==PX4LITE_OK);
  Feed(frame,len,10U);
  assert(Px4Lite_RpiMavlinkCopyRxFrame(&out)==PX4LITE_OK && out.frame_len==len);
  Feed(frame,11,20U); baud_result=BSP_STATUS_ERROR;
  assert(Px4Lite_RpiMavlinkSetBaudRate(115200U)==PX4LITE_IO_ERROR);
  Feed(frame,len,30U);
  assert(Px4Lite_RpiMavlinkCopyRxFrame(&out)==PX4LITE_OK);
  Feed(frame,11,40U); assert(Px4Lite_RpiMavlinkInit()==PX4LITE_OK);
  Feed(frame,len,50U); assert(Px4Lite_RpiMavlinkCopyRxFrame(&out)==PX4LITE_OK);
  Feed(frame,11,0U); Feed(frame,len,500U);
  assert(Px4Lite_RpiMavlinkCopyRxFrame(&out)==PX4LITE_OK);
  Feed(frame,11,UINT32_MAX-20U); Feed(frame,len,400U);
  assert(Px4Lite_RpiMavlinkCopyRxFrame(&out)==PX4LITE_OK);
  frame[len-1]^=0x20; Feed(frame,len,410U);
  assert(Px4Lite_RpiMavlinkCopyRxFrame(&out)==PX4LITE_NOT_READY);
  frame[len-1]^=0x20; Feed(frame,len,420U);
  assert(Px4Lite_RpiMavlinkCopyRxFrame(&out)==PX4LITE_OK);
  puts("rpi real platform parser recovery passed");
  return 0;
}
