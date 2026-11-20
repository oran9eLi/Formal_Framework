/**
 * @file test_pi_result_notify.c
 * @brief 真实UART6字节CRC、标准TUNNEL尾零解码与Pi终态提示端点门禁回归。
 * @details CRC/半帧/噪声走生产Platform解析；硬件输出边界仅作失败断言，
 *          Control由显式完成桩隔离；接受请求时静默，收到完成事实后才生成成功提示。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { uint32_t CNT,SR; } HostTimer;
static HostTimer s_timer;
static uint32_t s_now;
static uint8_t s_local_system=42U; /**< 当前测试本机动态系统号，0表示身份不可用。 */
#define TIM6 (&s_timer)
#define TIM_SR_UIF 1U
/* HAL串口枚举仅为编译真实平台诊断函数；本夹具不读取设备寄存器。 */
#define UART_WORDLENGTH_8B 0U
#define UART_WORDLENGTH_9B 0x1000U
#define UART_PARITY_NONE 0U
#define UART_PARITY_EVEN 0x0400U
#define UART_PARITY_ODD 0x0600U
#define UART_STOPBITS_1 0U
#define UART_STOPBITS_2 0x2000U
#define UART_HWCONTROL_NONE 0U
#define UART_HWCONTROL_RTS 0x0100U
#define UART_HWCONTROL_CTS 0x0200U
#define UART_HWCONTROL_RTS_CTS 0x0300U
static uint32_t __get_PRIMASK(void) { return 0U; }
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static uint32_t HAL_GetTick(void) { return s_now; }
static uint32_t HAL_GetUIDw0(void) { return 0U; }
static uint32_t HAL_GetUIDw1(void) { return 0U; }
static uint32_t HAL_GetUIDw2(void) { return 0U; }
#include "px4lite_config.h"
#if defined(TEST_RPI_DISABLED) && TEST_RPI_DISABLED
#undef PX4LITE_ENABLE_RPI_MAVLINK
#define PX4LITE_ENABLE_RPI_MAVLINK 0U
#endif

/* 平台真实硬件适配定义保持编译，但测试原名边界禁止通信任务直接发声/改灯。 */
#define Px4Lite_PlatformBuzzerInit Unused_PlatformBuzzerInit
#define Px4Lite_PlatformBuzzerSetFrequency Unused_PlatformBuzzerSetFrequency
#define Px4Lite_PlatformRgbInit Unused_PlatformRgbInit
#define Px4Lite_PlatformRgbDeInit Unused_PlatformRgbDeInit
#define Px4Lite_PlatformRgbSetColor Unused_PlatformRgbSetColor
#define Px4Lite_PlatformRgbService Unused_PlatformRgbService
#include "../../Framework/Src/px4lite_platform_f407.c"
#undef Px4Lite_PlatformBuzzerInit
#undef Px4Lite_PlatformBuzzerSetFrequency
#undef Px4Lite_PlatformRgbInit
#undef Px4Lite_PlatformRgbDeInit
#undef Px4Lite_PlatformRgbSetColor
#undef Px4Lite_PlatformRgbService
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void);
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t frequency);
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void);
void Px4Lite_PlatformRgbDeInit(void);
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t red,uint8_t green,uint8_t blue);
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t now_ms);
#include "../../Framework/Src/px4lite_command.c"
#include "../../Framework/Src/px4lite_rpi_baud_switch.c"
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wparentheses"
#if !PX4LITE_ENABLE_RPI_MAVLINK
/* 生产RX保留部分静态解码器声明，关闭RPi时在本夹具未引用，保持源文件不变。 */
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#endif
#include "../../Framework/Src/px4lite_mavlink_rx.c"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

static const uint8_t *s_input;
static uint16_t s_input_size;
static unsigned s_notifications,s_hardware_calls,s_command_calls,s_ack_calls,s_echo_calls,s_external_calls,s_control_calls;
static unsigned s_success_notifications,s_failure_notifications,s_control_pending;
static uint32_t s_mock_baud_bps=115200U;
static unsigned s_lora_commits;
static uint8_t s_lora_pending;
static Px4Lite_CommRxFrame_t s_lora_frame;
static Px4Lite_Result_t s_control_result;
static uint8_t s_ack_result;
static Px4Lite_MavlinkLink_t s_expected_link;
static uint8_t s_echo_pending;

/** @brief Notify只计分类结果，真正声光输出由独立sensor播放器专项验证。 */
void Px4Lite_StartupToneNotifyRpiResult(uint8_t succeeded)
{ s_notifications++;if(succeeded==1U) { s_success_notifications++; }else { s_failure_notifications++; } }
/** @brief 生产入口不得再使用纯收包通知来冒充成功。 */
void Px4Lite_StartupToneNotifyRpiCommand(void) { assert(0); }
/** @brief 原硬件边界只要被通信线程调用即失败。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerInit(void) { s_hardware_calls++;assert(0);return PX4LITE_IO_ERROR; }
/** @brief 通信线程不得改变PWM。 */
Px4Lite_Result_t Px4Lite_PlatformBuzzerSetFrequency(uint16_t f) { (void)f;s_hardware_calls++;assert(0);return PX4LITE_IO_ERROR; }
/** @brief 通信线程不得初始化RGB。 */
Px4Lite_Result_t Px4Lite_PlatformRgbInit(void) { s_hardware_calls++;assert(0);return PX4LITE_IO_ERROR; }
/** @brief 通信线程不得撤收RGB。 */
void Px4Lite_PlatformRgbDeInit(void) { s_hardware_calls++;assert(0); }
/** @brief 通信线程不得发送灯色。 */
Px4Lite_Result_t Px4Lite_PlatformRgbSetColor(uint8_t r,uint8_t g,uint8_t b) { (void)r;(void)g;(void)b;s_hardware_calls++;assert(0);return PX4LITE_IO_ERROR; }
/** @brief 通信线程不得推进RGB DMA。 */
Px4Lite_Result_t Px4Lite_PlatformRgbService(uint32_t ms) { (void)ms;s_hardware_calls++;assert(0);return PX4LITE_IO_ERROR; }
/** @brief 真实Platform只从本输入FIFO取线上原字节。 */
uint16_t BSP_RpiUART_Read(uint8_t *data,uint16_t count)
{ if(count>s_input_size) { count=s_input_size; }memcpy(data,s_input,count);s_input+=count;s_input_size=(uint16_t)(s_input_size-count);return count; }
/** @brief 冷初始化接口不访问真实设备。 */
BSP_Status_t BSP_RpiUART_Init(void) { return BSP_STATUS_OK; }
/** @brief 仅查询速率不涉及硬件重配置。 */
BSP_Status_t BSP_RpiUART_GetBaudRate(uint32_t *baud_bps) { assert(baud_bps);*baud_bps=s_mock_baud_bps;return BSP_STATUS_OK; }
/** @brief 配置应用桩写入独立速率事实，不操作真实串口。 */
BSP_Status_t BSP_RpiUART_SetBaudRate(uint32_t baud_bps) { s_mock_baud_bps=baud_bps;return BSP_STATUS_OK; }
/** @brief 平台时钟独立于msg.time字段。 */
uint32_t BSP_Time_GetTickMs(void) { return s_now; }
/** @brief 捕获旁路只记录校验过的源，不生成提示或消费接收队列。 */
void Px4Lite_CaptureRx(uint8_t source,const uint8_t *bytes,uint16_t length,uint32_t now)
{ assert(source==PX4LITE_CAPTURE_RPI_RX && bytes && length && now==s_now); }
/** @brief CRC失败可见但不能转为一次有效提示。 */
void Px4Lite_CaptureReject(uint8_t source) { assert(source==PX4LITE_CAPTURE_RPI_RX); }
/** @brief 记录真实命令执行器的ACK排队结果，提示不等于执行成功。 */
void Px4Lite_MavlinkQueueCommandAck(uint16_t command,uint8_t result,uint8_t target,uint8_t component,Px4Lite_MavlinkLink_t link)
{ (void)command;assert(target==9U && component==191U && link==s_expected_link);s_command_calls++;s_ack_result=result; }
/** @brief 真实ACK接收入口保持原消费语义。 */
void Px4Lite_MavlinkRecordCommandAck(uint16_t command,uint8_t result,uint32_t now)
{ (void)command;(void)result;assert(now==s_now);s_ack_calls++; }
/** @brief Control边界不驱动电机，只记录真实执行器是否已越过CRC及地址门禁。 */
Px4Lite_Result_t Px4Lite_ControlSetMotorThrottles(const uint8_t percent[PX4LITE_MOTOR_COUNT])
{ assert(percent);s_control_calls++;return s_control_result; }
/** @brief 精确PWM控制保留独立边界。 */
Px4Lite_Result_t Px4Lite_ControlSetMotorPulses(const uint16_t pulse[PX4LITE_MOTOR_COUNT])
{ assert(pulse);s_control_calls++;return s_control_result; }
/** @brief 模式和目标姿态配置不触及真实设备。 */
Px4Lite_Result_t Px4Lite_ControlSetModeAndAttitudeTarget(Px4Lite_ControlMode_t mode,
    int32_t roll, int32_t pitch, int32_t yaw)
{ (void)mode;(void)roll;(void)pitch;(void)yaw;s_control_calls++;return s_control_result; }
/** @brief 急停控制不触及真实设备。 */
Px4Lite_Result_t Px4Lite_ControlEmergencyStop(uint32_t now)
{ assert(now==s_now);s_control_calls++;return s_control_result; }
/** @brief 自动升速控制不触及真实设备。 */
Px4Lite_Result_t Px4Lite_ControlStartAutoTakeoff(void)
{ s_control_calls++;return s_control_result; }
/** @brief 自动缓降控制不触及真实设备。 */
Px4Lite_Result_t Px4Lite_ControlStartAutoLanding(void)
{ s_control_calls++;return s_control_result; }
/** @brief 真实执行跟踪在Control专项覆盖；本夹具仅模拟未完成的RPi目标。 */
Px4Lite_Result_t Px4Lite_ControlSetRpiMotorThrottles(const uint8_t percent[PX4LITE_MOTOR_COUNT])
{ Px4Lite_Result_t rc=Px4Lite_ControlSetMotorThrottles(percent);if(rc==PX4LITE_OK) { s_control_pending++; }return rc; }
/** @brief 精确PWM请求成功只登记待完成，不能在收包处自行成功。 */
Px4Lite_Result_t Px4Lite_ControlSetRpiMotorPulses(const uint16_t pulse[PX4LITE_MOTOR_COUNT])
{ Px4Lite_Result_t rc=Px4Lite_ControlSetMotorPulses(pulse);if(rc==PX4LITE_OK) { s_control_pending++; }return rc; }
/** @brief 自动过程启动接受尚未执行完成。 */
Px4Lite_Result_t Px4Lite_ControlStartRpiAutoTakeoff(void)
{ Px4Lite_Result_t rc=Px4Lite_ControlStartAutoTakeoff();if(rc==PX4LITE_OK) { s_control_pending++; }return rc; }
/** @brief 自动过程启动接受尚未执行完成。 */
Px4Lite_Result_t Px4Lite_ControlStartRpiAutoLanding(void)
{ Px4Lite_Result_t rc=Px4Lite_ControlStartAutoLanding();if(rc==PX4LITE_OK) { s_control_pending++; }return rc; }
/** @brief 模拟真实命名状态消费，不改变其副作用次数。 */
void Px4Lite_SetExternalModuleState(Px4Lite_ModuleId_t id,Px4Lite_State_t state,uint16_t fault,uint32_t now)
{ (void)state;(void)fault;assert(id==PX4LITE_MODULE_5G && now==s_now);s_external_calls++; }
/** @brief 本机地址仅用于保持原TUNNEL处理路径。 */
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void) { return s_local_system; }
/** @brief LoRa对端与本机地址不同。 */
uint8_t Px4Lite_IdentityGetNodeId(void) { return 42U; }
/** @brief 可控H1待发槽事实；校验与配置提示仍运行真实生产代码。 */
Px4Lite_Result_t Px4Lite_RpiH1EchoPeekResponse(Px4Lite_RpiH1EchoResponseView_t *out)
{ if(out) { memset(out,0,sizeof(*out)); }return s_echo_pending?PX4LITE_OK:PX4LITE_NOT_READY; }
/** @brief 只记录原H1请求分发，执行状态不控制提示。 */
Px4Lite_Result_t Px4Lite_RpiH1EchoHandleTunnel(uint8_t sys,uint8_t comp,uint8_t target,uint8_t component,uint8_t local,uint16_t type,const uint8_t *payload,uint8_t length,uint8_t wire_length)
{ (void)sys;(void)comp;(void)target;(void)component;(void)payload;(void)length;(void)wire_length;assert(local==s_local_system && type==PX4LITE_RPI_H1_TUNNEL_TYPE);s_echo_calls++;return PX4LITE_OK; }
/** @brief LoRa已经验证的帧事实，不借用RPi硬件解析器。 */
Px4Lite_Result_t Px4Lite_CopyCommRxFrame(Px4Lite_CommRxFrame_t *out)
{ if(!s_lora_pending) { return PX4LITE_NOT_READY; }*out=s_lora_frame;s_lora_pending=0U;return PX4LITE_OK; }
/** @brief 远端登记不触发本机声光。 */
void Px4Lite_RemoteObserve(uint8_t node,uint8_t component,uint8_t sequence,uint32_t id,const char *identity,uint32_t now)
{ (void)node;(void)component;(void)sequence;(void)id;(void)identity;(void)now; }
/** @brief 允许LoRa正常解码以验证提示入口严格限于RPi。 */
uint8_t Px4Lite_RemoteCanAccept(uint8_t node,uint32_t now) { (void)node;(void)now;return 1U; }
/** @brief 原远端增量快照保持可写。 */
Px4Lite_Result_t Px4Lite_RemoteTelemetryCopyNode(uint8_t node,Px4Lite_RemoteTelemetry_t *out) { (void)node;memset(out,0,sizeof(*out));return PX4LITE_OK; }
/** @brief LoRa解码已实际完成，仍不能引发RPi提示。 */
Px4Lite_Result_t Px4Lite_RemoteTelemetryCommitNode(uint8_t node,const Px4Lite_RemoteTelemetry_t *out,uint32_t now)
{ (void)node;(void)out;(void)now;s_lora_commits++;return PX4LITE_OK; }

#if PX4LITE_ENABLE_RPI_MAVLINK
/** @brief 每个串口场景清真实解析状态，发送状态机只用另一个MAVLink通道。 */
static void Reset(void)
{
  s_now=100U;s_local_system=42U;s_input_size=0U;s_notifications=s_hardware_calls=s_command_calls=s_ack_calls=s_echo_calls=s_external_calls=0U;
  s_success_notifications=s_failure_notifications=s_control_pending=0U;s_mock_baud_bps=115200U;
  s_lora_commits=0U;s_lora_pending=0U;s_control_calls=0U;s_control_result=PX4LITE_OK;s_ack_result=0U;
  s_expected_link=PX4LITE_MAVLINK_LINK_RPI;
  s_echo_pending=0U;
  assert(Px4Lite_RpiMavlinkInit()==PX4LITE_OK);
  assert(Px4Lite_RpiBaudSwitchInit()==PX4LITE_OK);
  /* 本地来源模式的Init可合法终结前场景旧配置，隔离后只统计本场景通知。 */
  s_notifications=s_success_notifications=s_failure_notifications=0U;
}

/** @brief Pi通知模式和提示使能门禁，关闭时不把合法帧变成声音。 */
static unsigned Expected(unsigned n)
{
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK && PX4LITE_RPI_RESULT_SOURCE_PI
  return n;
#else
  (void)n;return 0U;
#endif
}
/** @brief 整帧、半帧和噪声均经过真实Platform CRC状态机与真实RX消费入口。 */
static Px4Lite_Result_t Pump(const uint8_t *wire,uint16_t length)
{
  s_input=wire;s_input_size=length;
  while(s_input_size) {
    Px4Lite_Result_t rc=Px4Lite_RpiMavlinkService(s_now);
    assert(rc==PX4LITE_OK || rc==PX4LITE_BUSY);
    if(rc==PX4LITE_BUSY) { break; }
  }
  return Px4Lite_MavlinkRxRunRpi(s_now);
}
/** @brief 准备一条合法两字节通知；使用库内标准TUNNEL逐字段编码。 */
static void Packet(mavlink_tunnel_t *packet,uint8_t succeeded)
{
  memset(packet,0,sizeof(*packet));
  packet->target_system=s_local_system;packet->target_component=193U;
  packet->payload_type=PX4LITE_RPI_RESULT_TUNNEL_TYPE;
  packet->payload_length=2U;packet->payload[0]=1U;packet->payload[1]=succeeded;
}
/** @brief 标准编码生成线上帧，返回实际消息载荷长度以核验失败6/成功7。 */
static uint16_t Encode(const mavlink_tunnel_t *packet,uint8_t source,uint8_t component,uint8_t *wire,uint8_t *body_length)
{
  mavlink_message_t msg;
  mavlink_msg_tunnel_encode_chan(source,component,MAVLINK_COMM_1,&msg,packet);
  if(body_length) { *body_length=msg.len; }
  return mavlink_msg_to_send_buffer(wire,&msg);
}
/**
 * @brief 强制外层长度并重新计算标准MAVLink CRC，验证完整填充及越界线上帧。
 * @details 不实现解码器；只用已标准编码的133字节消息体与库内X25 CRC构造边界输入。
 */
static uint16_t EncodeForced(const mavlink_tunnel_t *packet,uint8_t body_length,uint8_t *wire)
{
  mavlink_message_t msg;uint16_t crc;char body[134]={0};
  assert(body_length<=134U);memset(&msg,0,sizeof(msg));
  mavlink_msg_tunnel_encode_chan(s_local_system,191U,MAVLINK_COMM_1,&msg,packet);
  wire[0]=0xFDU;wire[1]=body_length;wire[2]=0U;wire[3]=0U;
  wire[4]=msg.seq;wire[5]=msg.sysid;wire[6]=msg.compid;wire[7]=0x81U;wire[8]=1U;wire[9]=0U;
  /* 标准finalize会把CRC写到已裁剪payload尾部，不能把该缓存强行扩长当原数据。 */
  _mav_put_uint16_t(body,0,packet->payload_type);
  _mav_put_uint8_t(body,2,packet->target_system);
  _mav_put_uint8_t(body,3,packet->target_component);
  _mav_put_uint8_t(body,4,packet->payload_length);
  _mav_put_uint8_t_array(body,5,packet->payload,128);
  body[133]=(char)0xA5U;memcpy(wire+10U,body,body_length);
  crc=crc_calculate(wire+1U,(uint16_t)(9U+body_length));
  crc_accumulate(MAVLINK_MSG_ID_TUNNEL_CRC,&crc);
  wire[10U+body_length]=(uint8_t)crc;wire[11U+body_length]=(uint8_t)(crc>>8U);
  return (uint16_t)(12U+body_length);
}
/** @brief 新通知不得控制执行器、排ACK、回显或直接操作蜂鸣器/RGB。 */
static void NoSideEffects(void)
{
  assert(!s_hardware_calls && !s_command_calls && !s_ack_calls && !s_control_calls && !s_echo_calls);
}
/** @brief 尾零失败帧和成功帧准确分类，且连续同值通知不被错误去重。 */
static void TestSuccessFailureAndRepeatedValues(void)
{
  mavlink_tunnel_t p;uint8_t wire[300],body;uint16_t length;
  Reset();Packet(&p,1U);length=Encode(&p,s_local_system,191U,wire,&body);assert(body==7U);
  assert(Pump(wire,length)==(Expected(1U)?PX4LITE_OK:PX4LITE_IDLE));
  assert(s_success_notifications==Expected(1U) && !s_failure_notifications);
  /* 相同线上sequence也不代替Pi的请求ID去重；两条都必须被消费。 */
  (void)Pump(wire,length);assert(s_success_notifications==Expected(2U));
  Packet(&p,0U);length=Encode(&p,s_local_system,191U,wire,&body);assert(body==6U);
  (void)Pump(wire,length);(void)Pump(wire,length);
  assert(s_failure_notifications==Expected(2U) && s_notifications==Expected(4U));
  assert(Px4Lite_MavlinkRxRunRpi(s_now)==PX4LITE_IDLE && s_notifications==Expected(4U));
  NoSideEffects();
}
/** @brief CRC错误、噪声和半帧不提前触发，补全后才处理一次。 */
static void TestCrcAndFraming(void)
{
  mavlink_tunnel_t p;uint8_t wire[300],noise[]={0x11U,0x22U,0x33U};uint16_t length;
  Reset();Packet(&p,1U);length=Encode(&p,s_local_system,191U,wire,0);
  assert(Pump(noise,sizeof(noise))==PX4LITE_IDLE && !s_notifications);
  assert(Pump(wire,5U)==PX4LITE_IDLE && !s_notifications);
  s_now+=10U;(void)Pump(wire+5U,(uint16_t)(length-5U));assert(s_success_notifications==Expected(1U));
  wire[length-1U]^=0x20U;(void)Pump(wire,length);
  assert(s_notifications==Expected(1U) && s_rpi_monitor.rx_frame_errors==1U);
  wire[length-1U]^=0x20U;(void)Pump(wire,length);assert(s_success_notifications==Expected(2U));
  NoSideEffects();
}
/** @brief 当前动态ID和精确Pi/本机组件门禁，禁止源/目标广播及错误版本结果。 */
static void TestInvalidEndpointsAndPayloads(void)
{
  mavlink_tunnel_t p;uint8_t wire[300],source,component;uint16_t length;unsigned k;
  Reset();
  for(k=0U;k<13U;k++) {
    Packet(&p,1U);source=s_local_system;component=191U;
    if(k==0U) { source=9U; }
    if(k==1U) { source=0U; }
    if(k==2U) { component=193U; }
    if(k==3U) { component=0U; }
    if(k==4U) { p.target_system=0U; }
    if(k==5U) { p.target_system=99U; }
    if(k==6U) { p.target_component=191U; }
    if(k==7U) { p.target_component=0U; }
    if(k==8U) { p.payload_length=1U; }
    if(k==9U) { p.payload_length=3U; }
    if(k==10U) { p.payload[0]=2U; }
    if(k==11U) { p.payload[1]=2U; }
    if(k==12U) { p.payload_type=0x8041U; }
    length=Encode(&p,source,component,wire,0);(void)Pump(wire,length);assert(!s_notifications);
  }
  NoSideEffects();
}
/** @brief 本机号不写死为42，0表示身份尚未建立，不能接受结果。 */
static void TestDynamicIdentity(void)
{
  const uint8_t ids[]={1U,42U,218U,255U,0U};unsigned k;mavlink_tunnel_t p;uint8_t wire[300];uint16_t length;
  for(k=0U;k<sizeof(ids);k++) {
    Reset();s_local_system=ids[k];Packet(&p,1U);length=Encode(&p,s_local_system,191U,wire,0);(void)Pump(wire,length);
    assert(s_notifications==((ids[k]!=0U)?Expected(1U):0U));NoSideEffects();
  }
}
/** @brief 支持完整TUNNEL与未使用填充；只检查声明的两字节，不擅自加内层规则。 */
static void TestPaddingAndWireBounds(void)
{
  mavlink_tunnel_t p;uint8_t wire[300];uint16_t length;
  Reset();Packet(&p,1U);length=EncodeForced(&p,133U,wire);(void)Pump(wire,length);
  assert(s_success_notifications==Expected(1U));
  p.payload[2]=0xC3U;p.payload[127]=0xA5U;length=Encode(&p,s_local_system,191U,wire,0);(void)Pump(wire,length);
  assert(s_success_notifications==Expected(2U));
  Packet(&p,0U);length=EncodeForced(&p,133U,wire);(void)Pump(wire,length);assert(s_failure_notifications==Expected(1U));
  length=EncodeForced(&p,5U,wire);(void)Pump(wire,length);assert(s_notifications==Expected(3U));
  length=EncodeForced(&p,134U,wire);(void)Pump(wire,length);assert(s_notifications==Expected(3U));
  NoSideEffects();
}
/** @brief 终态通知不受H1或波特率实验待发仲裁吞掉，且不改变实验状态。 */
static void TestIndependentFromExperimentPending(void)
{
  mavlink_tunnel_t p;uint8_t wire[300],baud[14]={1U,3U,1U,0,0,0,0,0,0,0,0,0xE1U,0,0};uint16_t length;
  Reset();s_echo_pending=1U;Packet(&p,1U);length=Encode(&p,s_local_system,191U,wire,0);(void)Pump(wire,length);
  assert(s_success_notifications==Expected(1U));NoSideEffects();
  Reset();
  assert(Px4Lite_RpiBaudSwitchHandleTunnel(9U,191U,42U,193U,42U,0x8003U,baud,14U,12U)==PX4LITE_OK);
  assert(Px4Lite_RpiBaudSwitchIsPending() && !s_notifications);
  Packet(&p,0U);length=Encode(&p,s_local_system,191U,wire,0);(void)Pump(wire,length);
  assert(s_failure_notifications==Expected(1U) && Px4Lite_RpiBaudSwitchIsPending());NoSideEffects();
}
/** @brief 心跳与其他物理链路没有结果通知含义，不调用控制或返回ACK。 */
static void TestHeartbeatAndLoraScope(void)
{
  mavlink_message_t message;uint8_t wire[300],payload[128]={1U,1U};uint16_t length;
  Reset();mavlink_msg_heartbeat_pack_chan(42U,191U,MAVLINK_COMM_1,&message,2U,8U,0U,0U,4U);
  length=mavlink_msg_to_send_buffer(wire,&message);assert(Pump(wire,length)==PX4LITE_IDLE && !s_notifications);
  assert(Px4Lite_CommandHandleRpiResultTunnel(42U,191U,42U,193U,0x8040U,payload,2U,2U,PX4LITE_MAVLINK_LINK_LORA)!=PX4LITE_OK);
  assert(!s_notifications);NoSideEffects();
}
/** @brief 纯校验边界不信任手工调用的长度或解码缓存，不读取空指针或额外数组。 */
static void TestDecodedHelperBounds(void)
{
  uint8_t p[128]={1U,1U};
  Reset();
  assert(Px4Lite_CommandHandleRpiResultTunnel(42U,191U,42U,193U,0x8040U,0,2U,2U,PX4LITE_MAVLINK_LINK_RPI)!=PX4LITE_OK);
  assert(Px4Lite_CommandHandleRpiResultTunnel(42U,191U,42U,193U,0x8040U,p,2U,0U,PX4LITE_MAVLINK_LINK_RPI)!=PX4LITE_OK);
  assert(Px4Lite_CommandHandleRpiResultTunnel(42U,191U,42U,193U,0x8040U,p,2U,129U,PX4LITE_MAVLINK_LINK_RPI)!=PX4LITE_OK);
  assert(Px4Lite_CommandHandleRpiResultTunnel(42U,191U,42U,193U,0x8040U,p,2U,1U,PX4LITE_MAVLINK_LINK_RPI)!=PX4LITE_OK);
  p[0]=0U;assert(Px4Lite_CommandHandleRpiResultTunnel(42U,191U,42U,193U,0x8040U,p,2U,2U,PX4LITE_MAVLINK_LINK_RPI)!=PX4LITE_OK);
  p[0]=1U;p[1]=255U;assert(Px4Lite_CommandHandleRpiResultTunnel(42U,191U,42U,193U,0x8040U,p,2U,2U,PX4LITE_MAVLINK_LINK_RPI)!=PX4LITE_OK);
  assert(!s_notifications);NoSideEffects();
}
#endif

/** @brief 执行真实CRC→RX→结果校验链，关闭通知/旧来源模式均保持静默。 */
int main(void)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  TestSuccessFailureAndRepeatedValues();TestCrcAndFraming();TestInvalidEndpointsAndPayloads();
  TestDynamicIdentity();TestPaddingAndWireBounds();TestIndependentFromExperimentPending();TestHeartbeatAndLoraScope();TestDecodedHelperBounds();
  printf("Pi result notify RX: real CRC / trimmed6+7 / dynamic identity / strict endpoints / no ACK-control-IO / repeated values / padding / bounds / pending isolation PASS pi=%u enabled=%u\n",(unsigned)PX4LITE_RPI_RESULT_SOURCE_PI,(unsigned)PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK);
#else
  assert(Px4Lite_RpiMavlinkService(0U)==PX4LITE_IDLE && Px4Lite_MavlinkRxRunRpi(0U)==PX4LITE_IDLE);
  assert(!s_notifications && !s_hardware_calls && !s_command_calls);
  puts("Pi result notify RX disabled: no receive / no notification / no side effects PASS");
#endif
  return 0;
}
