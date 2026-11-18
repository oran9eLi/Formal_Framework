/**
 * @file test_rpi_result_feedback_tx.c
 * @brief 主机验证真实MAVLink切速率响应发送与真实配置结果声光通知的连接。
 * @details 仅替换UART/速率查询/提示登记边界，保留真实BaudSwitch、MAVLink编码与发送分支。
 *          不启动设备、不改变生产源码；未把成功发送、接受或静态状态当实际切速率成功。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAVLINK_COMM_NUM_BUFFERS 1
#include "px4lite_config.h"
#include "px4lite_platform.h"
#include "px4lite_startup_tone.h"
#include "px4lite_rpi_h1_echo.h"

/* 两个真实源文件各有私有s_stats；本夹具重命名Baud私有标识避免同一翻译单元碰撞。 */
#define s_stats test_baud_stats
#define s_slot test_baud_slot
#include "../../Framework/Src/px4lite_rpi_baud_switch.c"
#undef s_stats
#undef s_slot
#include "../../Framework/Src/px4lite_mavlink_tx.c"

static uint32_t g_baud_bps,g_now_ms;
static Px4Lite_Result_t g_send_result,g_set_result,g_get_result;
static uint8_t g_wrong_readback;
static uint8_t g_last_frame[MAVLINK_MAX_PACKET_LEN];
static uint16_t g_last_length;
static unsigned g_send_calls,g_set_calls,g_notifications;
static uint8_t g_results[16];

/** @brief 结果出口只记录先后，不播放蜂鸣器或触碰RGB。 */
void Px4Lite_StartupToneNotifyRpiResult(uint8_t succeeded)
{
  assert(g_notifications<sizeof(g_results));
  g_results[g_notifications++]=succeeded;
}

/** @brief 旧收包提示不得从生产配置发送路径再被调用。 */
void Px4Lite_StartupToneNotifyRpiCommand(void) { assert(0); }

/** @brief 固定毫秒时钟，仅用于真实请求记录。 */
uint32_t Px4Lite_PlatformGetMs(void) { return g_now_ms; }

/** @brief 固定本机MAVLink系统号，不依赖持久化配置。 */
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void) { return 42U; }

/** @brief 单纯记录完整线上字节和返回结果；发送时必须仍在旧速率。 */
Px4Lite_Result_t Px4Lite_RpiMavlinkSend(const uint8_t *data,uint16_t length)
{
  assert(data!=0 && length<=sizeof(g_last_frame));
  assert(g_baud_bps==115200U && g_set_calls==0U);
  memcpy(g_last_frame,data,length);g_last_length=length;g_send_calls++;
  return g_send_result;
}

/** @brief 真实事务读取的已应用速率，失败或错误读回均由测试显式注入。 */
Px4Lite_Result_t Px4Lite_RpiMavlinkGetBaudRate(uint32_t *baud_bps)
{
  assert(baud_bps!=0);
  if(g_get_result!=PX4LITE_OK) { return g_get_result; }
  *baud_bps=g_baud_bps;return PX4LITE_OK;
}

/** @brief 重配桩仅在完整发送成功后允许调用，可模拟返回成功却实际读回仍旧速率。 */
Px4Lite_Result_t Px4Lite_RpiMavlinkSetBaudRate(uint32_t baud_bps)
{
  assert(baud_bps==57600U && g_send_calls>0U && g_send_result==PX4LITE_OK);
  g_set_calls++;
  if(g_set_result==PX4LITE_OK && !g_wrong_readback) { g_baud_bps=baud_bps; }
  return g_set_result;
}

/** @brief 本专项只测试波特率响应，H1无待发槽；交叉仲裁由真实RX专项覆盖。 */
Px4Lite_Result_t Px4Lite_RpiH1EchoPeekResponse(Px4Lite_RpiH1EchoResponseView_t *out)
{ (void)out;return PX4LITE_NOT_READY; }

/** @brief 对比结果分类；关闭提示时同一发送和切速率流程仍执行但不产出提示。 */
static void ExpectResults(unsigned failures,unsigned successes)
{
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  unsigned i,actual_fail=0U,actual_success=0U;
  for(i=0U;i<g_notifications;i++) {
    if(g_results[i]==1U) { actual_success++; } else { assert(g_results[i]==0U);actual_fail++; }
  }
  assert(actual_fail==failures && actual_success==successes);
#else
  (void)failures;(void)successes;assert(g_notifications==0U);
#endif
}

/** @brief 每场景恢复旧速率，重置真实事务；不把跨测试清理产生的取消通知计入用例。 */
static void Reset(void)
{
  g_baud_bps=115200U;g_now_ms=1000U;g_send_result=PX4LITE_OK;
  g_set_result=PX4LITE_OK;g_get_result=PX4LITE_OK;g_wrong_readback=0U;
  assert(Px4Lite_RpiBaudSwitchInit()==PX4LITE_OK);
  g_send_calls=g_set_calls=g_notifications=0U;g_last_length=0U;
  memset(g_results,0xFF,sizeof(g_results));
  memset(mavlink_get_channel_status(MAVLINK_COMM_0),0,sizeof(mavlink_status_t));
  s_rpi_tx_seq=0U;
}

/** @brief 通过真实完整结构校验建立待发请求，接受时不允许任何结果提示。 */
static void Enqueue(void)
{
  static const uint8_t payload[14]={1U,3U,1U,2U,3U,4U,5U,6U,7U,8U,0U,0xE1U,0U,0U};
  assert(Px4Lite_RpiBaudSwitchHandleTunnel(9U,191U,42U,193U,42U,0x8003U,payload,14U,12U)==PX4LITE_OK);
  assert(Px4Lite_RpiBaudSwitchIsPending()!=0U && g_baud_bps==115200U && g_set_calls==0U);
  ExpectResults(0U,0U);
}

/** @brief 独立CRC解析真实UART字节，验证TUNNEL接受响应，而不是只检查内部结构体。 */
static void CheckWire(void)
{
  mavlink_message_t input,decoded;
  mavlink_status_t parsing,output;
  mavlink_tunnel_t tunnel;
  uint16_t i;unsigned valid=0U;
  memset(&input,0,sizeof(input));memset(&decoded,0,sizeof(decoded));
  memset(&parsing,0,sizeof(parsing));memset(&output,0,sizeof(output));
  for(i=0U;i<g_last_length;i++) {
    if(mavlink_frame_char_buffer(&input,&parsing,g_last_frame[i],&decoded,&output)==MAVLINK_FRAMING_OK) { valid++; }
  }
  assert(valid==1U && decoded.msgid==MAVLINK_MSG_ID_TUNNEL && decoded.sysid==42U && decoded.compid==193U);
  mavlink_msg_tunnel_decode(&decoded,&tunnel);
  assert(tunnel.target_system==9U && tunnel.target_component==191U && tunnel.payload_type==0x8003U);
  assert(tunnel.payload_length==14U && tunnel.payload[0]==1U && tunnel.payload[1]==4U);
  assert(tunnel.payload[10]==0U && tunnel.payload[11]==0xE1U);
}

/** @brief 无事务静默；BUSY/IDLE等待不切速率，真正完整发送且读回57600后才成功。 */
static void TestDeferredSuccess(void)
{
  Reset();assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_NOT_READY);
  assert(g_send_calls==0U);ExpectResults(0U,0U);Enqueue();
  g_send_result=PX4LITE_BUSY;assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_BUSY);
  assert(Px4Lite_RpiBaudSwitchIsPending()!=0U && g_set_calls==0U);ExpectResults(0U,0U);CheckWire();
  g_send_result=PX4LITE_IDLE;assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_IDLE);
  assert(Px4Lite_RpiBaudSwitchIsPending()!=0U && g_set_calls==0U);ExpectResults(0U,0U);
  g_send_result=PX4LITE_OK;assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_OK);
  assert(g_baud_bps==57600U && g_set_calls==1U && !Px4Lite_RpiBaudSwitchIsPending());
  ExpectResults(0U,1U);CheckWire();
  assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_NOT_READY && g_set_calls==1U);ExpectResults(0U,1U);
}

/** @brief 发送失败只报低音一次，原事务保持可重试，随后实际应用成功再报高音。 */
static void TestRetryAfterError(void)
{
  Reset();Enqueue();g_send_result=PX4LITE_IO_ERROR;
  assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_IO_ERROR);ExpectResults(1U,0U);
  assert(Px4Lite_RpiBaudSwitchIsPending()!=0U && g_baud_bps==115200U && g_set_calls==0U);
  assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_IO_ERROR);ExpectResults(1U,0U);
  g_send_result=PX4LITE_INVALID_PARAM;assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_INVALID_PARAM);ExpectResults(1U,0U);
  g_send_result=PX4LITE_BUSY;assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_BUSY);ExpectResults(1U,0U);
  g_send_result=PX4LITE_OK;assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_OK);
  assert(g_baud_bps==57600U && g_set_calls==1U && !Px4Lite_RpiBaudSwitchIsPending());ExpectResults(1U,1U);
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  assert(g_results[0]==0U && g_results[1]==1U);
#endif
}

/** @brief 完整发送成功不等于重配成功：Set失败、Get失败和错误读回均不得报高音。 */
static void TestApplyFailures(void)
{
  Reset();Enqueue();g_set_result=PX4LITE_IO_ERROR;
  assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_OK);ExpectResults(1U,0U);
  assert(g_set_calls==1U && g_baud_bps==115200U && !Px4Lite_RpiBaudSwitchIsPending());
  Reset();Enqueue();g_wrong_readback=1U;
  assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_OK);ExpectResults(1U,0U);assert(g_baud_bps==115200U);
  Reset();Enqueue();g_get_result=PX4LITE_IO_ERROR;
  assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_OK);ExpectResults(1U,0U);assert(g_baud_bps==57600U);
  /* 发送失败已报过低音，后续重配也失败不重复低音。 */
  Reset();Enqueue();g_send_result=PX4LITE_IO_ERROR;
  assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_IO_ERROR);ExpectResults(1U,0U);
  g_send_result=PX4LITE_OK;g_set_result=PX4LITE_IO_ERROR;
  assert(MavTx_SendRpiBaudSwitchResponse()==PX4LITE_OK);ExpectResults(1U,0U);
}

/** @brief 真实初始化中止未应用事务时只产生一次未执行结果，不发伪成功。 */
static void TestReinitCancellation(void)
{
  Reset();Enqueue();assert(Px4Lite_RpiBaudSwitchInit()==PX4LITE_OK);ExpectResults(1U,0U);
  assert(!Px4Lite_RpiBaudSwitchIsPending() && g_send_calls==0U);
  assert(Px4Lite_RpiBaudSwitchInit()==PX4LITE_OK);ExpectResults(1U,0U);
}

/** @brief 执行真实发送、错误重试和配置应用结果的全部主机场景。 */
int main(void)
{
  TestDeferredSuccess();TestRetryAfterError();TestApplyFailures();TestReinitCancellation();
  printf("RPi result TX: real MAVLink CRC / pending silent / BUSY+IDLE / first send error low / retry+readback high / apply failures / init cancel PASS feedback=%u\n",(unsigned)PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK);
  return 0;
}
