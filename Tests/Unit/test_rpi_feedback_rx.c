/**
 * @file test_rpi_feedback_rx.c
 * @brief 真实UART6字节解析与MAVLink RX入口的控制/配置提示门禁回归。
 * @details CRC/半帧/噪声走生产Platform解析；硬件输出边界仅作失败断言，
 *          Notify只是任务间事件，不能把收包提示当作指令执行成功。
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
static unsigned s_lora_commits;
static uint8_t s_lora_pending;
static Px4Lite_CommRxFrame_t s_lora_frame;
static Px4Lite_Result_t s_control_result;
static uint8_t s_ack_result;
static Px4Lite_MavlinkLink_t s_expected_link;
static uint8_t s_echo_pending;

/** @brief Notify仅计事件，硬件操作必须由另一sensor播放器单元执行。 */
void Px4Lite_StartupToneNotifyRpiCommand(void) { s_notifications++; }
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
BSP_Status_t BSP_RpiUART_GetBaudRate(uint32_t *baud_bps) { assert(baud_bps);*baud_bps=115200U;return BSP_STATUS_OK; }
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
/** @brief 模式控制不触及真实设备。 */
Px4Lite_Result_t Px4Lite_ControlSetMode(Px4Lite_ControlMode_t mode)
{ (void)mode;s_control_calls++;return s_control_result; }
/** @brief 姿态目标控制不触及真实设备。 */
Px4Lite_Result_t Px4Lite_ControlSetAttitudeTarget(int32_t roll,int32_t pitch,int32_t yaw)
{ (void)roll;(void)pitch;(void)yaw;s_control_calls++;return s_control_result; }
/** @brief 急停控制不触及真实设备。 */
Px4Lite_Result_t Px4Lite_ControlEmergencyStop(uint32_t now)
{ assert(now==s_now);s_control_calls++;return s_control_result; }
/** @brief 自动升速控制不触及真实设备。 */
Px4Lite_Result_t Px4Lite_ControlStartAutoTakeoff(void)
{ s_control_calls++;return s_control_result; }
/** @brief 自动缓降控制不触及真实设备。 */
Px4Lite_Result_t Px4Lite_ControlStartAutoLanding(void)
{ s_control_calls++;return s_control_result; }
/** @brief 模拟真实命名状态消费，不改变其副作用次数。 */
void Px4Lite_SetExternalModuleState(Px4Lite_ModuleId_t id,Px4Lite_State_t state,uint16_t fault,uint32_t now)
{ (void)state;(void)fault;assert(id==PX4LITE_MODULE_5G && now==s_now);s_external_calls++; }
/** @brief 本机地址仅用于保持原TUNNEL处理路径。 */
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void) { return 42U; }
/** @brief LoRa对端与本机地址不同。 */
uint8_t Px4Lite_IdentityGetNodeId(void) { return 42U; }
/** @brief 可控H1待发槽事实；校验与配置提示仍运行真实生产代码。 */
Px4Lite_Result_t Px4Lite_RpiH1EchoPeekResponse(Px4Lite_RpiH1EchoResponseView_t *out)
{ if(out) { memset(out,0,sizeof(*out)); }return s_echo_pending?PX4LITE_OK:PX4LITE_NOT_READY; }
/** @brief 只记录原H1请求分发，执行状态不控制提示。 */
Px4Lite_Result_t Px4Lite_RpiH1EchoHandleTunnel(uint8_t sys,uint8_t comp,uint8_t target,uint8_t component,uint8_t local,uint16_t type,const uint8_t *payload,uint8_t length,uint8_t wire_length)
{ (void)sys;(void)comp;(void)target;(void)component;(void)payload;(void)length;(void)wire_length;assert(local==42U && type==PX4LITE_RPI_H1_TUNNEL_TYPE);s_echo_calls++;return PX4LITE_OK; }
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
  s_now=100U;s_input_size=0U;s_notifications=s_hardware_calls=s_command_calls=s_ack_calls=s_echo_calls=s_external_calls=0U;
  s_lora_commits=0U;s_lora_pending=0U;s_control_calls=0U;s_control_result=PX4LITE_OK;s_ack_result=0U;
  s_expected_link=PX4LITE_MAVLINK_LINK_RPI;
  s_echo_pending=0U;
  assert(Px4Lite_RpiMavlinkInit()==PX4LITE_OK);
  assert(Px4Lite_RpiBaudSwitchInit()==PX4LITE_OK);
}
/** @brief 每个本机控制/配置请求的收包提示计数，关闭feature时始终为0。 */
static unsigned Expected(unsigned count)
{
#if PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK
  return count;
#else
  (void)count;return 0U;
#endif
}
/** @brief 输入一段线上字节后推进实际Comm解码一次，不合并或伪造校验。 */
static Px4Lite_Result_t Pump(const uint8_t *wire,uint16_t length)
{
  s_input=wire;s_input_size=length;
  while(s_input_size) { Px4Lite_Result_t rc=Px4Lite_RpiMavlinkService(s_now);assert(rc==PX4LITE_OK || rc==PX4LITE_BUSY);if(rc==PX4LITE_BUSY) { break; } }
  return Px4Lite_MavlinkRxRunRpi(s_now);
}
/** @brief 构造真实MAVLink2完整帧，覆盖控制/配置及应保持静默的其他报文。 */
static uint16_t Build(unsigned kind,uint8_t *wire)
{
  mavlink_message_t msg;
  static const char cell_name[10]="RPICELL";
  switch(kind) {
    case 0U:mavlink_msg_heartbeat_pack_chan(9U,191U,MAVLINK_COMM_1,&msg,2U,8U,0U,0U,4U);break;
    case 1U:mavlink_msg_command_long_pack_chan(9U,191U,MAVLINK_COMM_1,&msg,42U,193U,31011U,0U,1.0f,0,0,0,0,0,0);break;
    case 2U:mavlink_msg_command_ack_pack_chan(9U,191U,MAVLINK_COMM_1,&msg,400U,0U,0U,0,42U,193U);break;
    case 3U:mavlink_msg_named_value_int_pack_chan(9U,191U,MAVLINK_COMM_1,&msg,10U,cell_name,1);break;
    case 4U: {
      mavlink_tunnel_t packet;memset(&packet,0,sizeof(packet));packet.target_system=42U;packet.target_component=193U;
      packet.payload_type=PX4LITE_RPI_H1_TUNNEL_TYPE;packet.payload_length=11U;packet.payload[0]=1U;packet.payload[1]=1U;
      mavlink_msg_tunnel_encode_chan(9U,191U,MAVLINK_COMM_1,&msg,&packet);break;
    }
    case 6U: {
      mavlink_tunnel_t packet;memset(&packet,0,sizeof(packet));packet.target_system=42U;packet.target_component=193U;
      packet.payload_type=PX4LITE_RPI_BAUD_TUNNEL_TYPE;packet.payload_length=14U;
      packet.payload[0]=1U;packet.payload[1]=3U;packet.payload[2]=1U;packet.payload[10]=0U;packet.payload[11]=0xE1U;
      mavlink_msg_tunnel_encode_chan(9U,191U,MAVLINK_COMM_1,&msg,&packet);break;
    }
    default:mavlink_msg_attitude_pack_chan(9U,191U,MAVLINK_COMM_1,&msg,10U,0.1f,0.2f,0.3f,0,0,0);break;
  }
  return mavlink_msg_to_send_buffer(wire,&msg);
}
/** @brief 心跳/坏CRC/半帧/噪声不触发，后续完整帧仍只通知一次。 */
static void TestCrcFramingAndHeartbeat(void)
{
  uint8_t wire[300],noise[]={0x11U,0x22U,0x33U};uint16_t length;
  Reset();length=Build(0U,wire);assert(Pump(wire,length)==PX4LITE_IDLE && !s_notifications);
  assert(Pump(noise,sizeof(noise))==PX4LITE_IDLE && !s_notifications);
  length=Build(1U,wire);assert(Pump(wire,7U)==PX4LITE_IDLE && !s_notifications);
  s_now+=10U;assert(Pump(wire+7U,(uint16_t)(length-7U))==PX4LITE_OK && s_notifications==Expected(1U));
  assert(Px4Lite_MavlinkRxRunRpi(s_now)==PX4LITE_IDLE && s_notifications==Expected(1U));
  length=Build(1U,wire);wire[length-1U]^=0x40U;
  assert(Pump(wire,length)==PX4LITE_IDLE && s_notifications==Expected(1U) && s_rpi_monitor.rx_frame_errors==1U);
  wire[length-1U]^=0x40U;assert(Pump(wire,length)==PX4LITE_OK && s_notifications==Expected(2U));
  assert(!s_hardware_calls && s_command_calls==2U && s_control_calls==2U);
}
/** @brief ACK/命名状态/普通回显/遥测静默，只有控制和波特率配置请求通知。 */
static void TestControlConfigurationOnly(void)
{
  uint8_t wire[300];unsigned kind;uint16_t length;mavlink_message_t msg;
  Reset();
  for(kind=0U;kind<=5U;kind++) { length=Build(kind,wire);(void)Pump(wire,length);assert(s_notifications==Expected(kind?1U:0U)); }
  assert(s_command_calls==1U && s_ack_calls==1U && s_external_calls==1U && s_echo_calls==1U);
  length=Build(6U,wire);(void)Pump(wire,length);assert(s_notifications==Expected(2U) && Px4Lite_RpiBaudSwitchIsPending());
  /* 真实执行器的业务拒绝不改变“有效控制请求已收到”的提示含义。 */
  s_control_result=PX4LITE_INVALID_PARAM;length=Build(1U,wire);
  assert(Pump(wire,length)==PX4LITE_OK && s_notifications==Expected(3U) && s_command_calls==2U);
  assert(s_ack_result==MAV_RESULT_DENIED && s_control_calls==2U);
  /* 参数非法由真实执行器拒绝，不调用Control，也不能被解释成成功。 */
  mavlink_msg_command_long_pack_chan(9U,191U,MAVLINK_COMM_1,&msg,42U,193U,31011U,0U,101.0f,0,0,0,0,0,0);
  length=mavlink_msg_to_send_buffer(wire,&msg);
  assert(Pump(wire,length)==PX4LITE_OK && s_notifications==Expected(4U));
  assert(s_ack_result==MAV_RESULT_DENIED && s_control_calls==2U && s_command_calls==3U);
  assert(!s_hardware_calls);
}

/** @brief 真实MAVLink2尾零裁剪的广播与本机目标都可提示，其他目标不提示。 */
static void TestCommandTargetsAndTrimming(void)
{
  static const uint8_t targets[][2]={{0U,0U},{42U,0U},{42U,191U},{42U,193U},{88U,193U},{42U,99U}};
  unsigned i,expected=0U;uint8_t wire[300];mavlink_message_t msg;uint16_t length;
  Reset();
  for(i=0U;i<sizeof(targets)/sizeof(targets[0]);i++) {
    mavlink_msg_command_long_pack_chan(9U,191U,MAVLINK_COMM_1,&msg,targets[i][0],targets[i][1],22U,0U,0,0,0,0,0,0,0);
    if(i==0U) { assert(msg.len==29U); }
    if(i==1U) { assert(msg.len==31U); }
    length=mavlink_msg_to_send_buffer(wire,&msg);(void)Pump(wire,length);
    if(i<4U) { expected++; }
    assert(s_notifications==Expected(expected));
  }
  assert(s_command_calls==4U && s_ack_result==MAV_RESULT_UNSUPPORTED && !s_control_calls && !s_hardware_calls);
}

/** @brief 配置必须为本机请求类型，响应/回显及错误结构不提示；末尾零裁剪仍合法。 */
static void TestConfigurationTargetsAndPayloads(void)
{
  mavlink_tunnel_t packet;mavlink_message_t msg;uint8_t wire[300],source,component;uint16_t length;unsigned variant,expected=0U;
  Px4Lite_RpiBaudSwitchStats_t stats;
  Reset();
  for(variant=0U;variant<13U;variant++) {
    source=9U;component=191U;
    memset(&packet,0,sizeof(packet));packet.target_system=42U;packet.target_component=193U;
    packet.payload_type=PX4LITE_RPI_BAUD_TUNNEL_TYPE;packet.payload_length=14U;
    packet.payload[0]=1U;packet.payload[1]=3U;packet.payload[2]=1U;packet.payload[11]=0xE1U;
    if(variant==1U) { packet.target_system=0U; }
    if(variant==2U) { packet.target_component=191U; }
    if(variant==3U) { packet.payload[1]=4U; }
    if(variant==4U) { packet.payload[0]=2U; }
    if(variant==5U) { packet.payload_length=2U; }
    if(variant==7U) { packet.payload[2]=0U; }
    if(variant==8U) { packet.payload[11]=0U; }
    if(variant==9U) { source=0U; }
    if(variant==10U) { component=0U; }
    if(variant==11U) { packet.payload[2]=2U; }
    if(variant==12U) { packet.payload_type=0x9999U; }
    /* 同nonce重发仍是收到一次配置请求，提示不等于实际再次切波特率。 */
    mavlink_msg_tunnel_encode_chan(source,component,MAVLINK_COMM_1,&msg,&packet);
    if(variant==0U || variant==6U || variant==11U) { assert(msg.len==17U);expected++; }
    length=mavlink_msg_to_send_buffer(wire,&msg);(void)Pump(wire,length);assert(s_notifications==Expected(expected));
  }
  Px4Lite_RpiBaudSwitchGetStats(&stats);
  assert(stats.accepted_count==1U && stats.duplicate_count==1U && stats.busy_count==1U);
  assert(stats.current_baud_bps==115200U && stats.state==PX4LITE_RPI_BAUD_STATE_PENDING_TX);
  assert(!s_hardware_calls);
}
/** @brief H1占用不掩盖合法配置收包提示，非法请求静默且不得抢占事务槽。 */
static void TestConfigurationWhileH1Pending(void)
{
  mavlink_tunnel_t packet;mavlink_message_t msg;uint8_t wire[300];uint16_t length;unsigned variant;
  Px4Lite_RpiBaudSwitchStats_t stats;
  Reset();s_echo_pending=1U;
  for(variant=0U;variant<7U;variant++) {
    memset(&packet,0,sizeof(packet));packet.target_system=42U;packet.target_component=193U;
    packet.payload_type=PX4LITE_RPI_BAUD_TUNNEL_TYPE;packet.payload_length=14U;
    packet.payload[0]=1U;packet.payload[1]=3U;packet.payload[2]=1U;packet.payload[11]=0xE1U;
    if(variant==1U) { packet.payload[2]=0U; }
    if(variant==2U) { packet.payload[11]=0U; }
    if(variant==3U) { packet.target_system=88U; }
    if(variant==4U) { packet.target_component=191U; }
    if(variant==5U) { packet.payload[0]=2U; }
    if(variant==6U) { packet.payload_length=2U; }
    mavlink_msg_tunnel_encode_chan(9U,191U,MAVLINK_COMM_1,&msg,&packet);
    length=mavlink_msg_to_send_buffer(wire,&msg);(void)Pump(wire,length);
    assert(s_notifications==Expected(1U) && !Px4Lite_RpiBaudSwitchIsPending());
  }
  Px4Lite_RpiBaudSwitchGetStats(&stats);
  assert(stats.accepted_count==0U && stats.busy_count==1U && stats.rejected_count==6U);
  assert(stats.state==PX4LITE_RPI_BAUD_STATE_IDLE && stats.current_baud_bps==115200U);
  /* 普通回显与等待响应没有控制/配置含义。 */
  length=Build(4U,wire);(void)Pump(wire,length);assert(s_notifications==Expected(1U));
  s_echo_pending=0U;length=Build(6U,wire);(void)Pump(wire,length);
  Px4Lite_RpiBaudSwitchGetStats(&stats);
  assert(s_notifications==Expected(2U) && stats.accepted_count==1U && Px4Lite_RpiBaudSwitchIsPending());
  assert(!s_hardware_calls && !s_control_calls && !s_command_calls);
}
/** @brief 拼接两个完整包仍一包一次，pending重复服务不会重复通知。 */
static void TestMultipleFramesAndLoraScope(void)
{
  uint8_t wire[600];uint16_t first,second;unsigned i; mavlink_message_t msg;
  Reset();first=Build(1U,wire);second=Build(0U,wire+first);
  s_input=wire;s_input_size=(uint16_t)(first+second);
  assert(Px4Lite_RpiMavlinkService(s_now)==PX4LITE_OK && s_input_size==second);
  assert(Px4Lite_RpiMavlinkService(s_now)==PX4LITE_BUSY);
  assert(Px4Lite_MavlinkRxRunRpi(s_now)==PX4LITE_OK && s_notifications==Expected(1U));
  assert(Px4Lite_RpiMavlinkService(s_now)==PX4LITE_OK && !s_input_size);
  assert(Px4Lite_MavlinkRxRunRpi(s_now)==PX4LITE_IDLE && s_notifications==Expected(1U));
  for(i=0U;i<3U;i++) { assert(Px4Lite_MavlinkRxRunRpi(s_now)==PX4LITE_IDLE); }
  mavlink_msg_attitude_pack_chan(9U,191U,MAVLINK_COMM_1,&msg,100U,0.1f,0.2f,0.3f,0,0,0);
  memset(&s_lora_frame,0,sizeof(s_lora_frame));s_lora_frame.msg_id=msg.msgid;s_lora_frame.system_id=msg.sysid;
  s_lora_frame.component_id=msg.compid;s_lora_frame.sequence=msg.seq;s_lora_frame.payload_len=msg.len;s_lora_frame.rx_time_ms=s_now;
  memcpy(s_lora_frame.payload,_MAV_PAYLOAD(&msg),msg.len);s_lora_pending=1U;
  assert(Px4Lite_MavlinkRxRun(s_now)==PX4LITE_OK && s_lora_commits==1U);
  assert(s_notifications==Expected(1U) && !s_hardware_calls);
  /* 同一命令执行器的LoRa来源继续执行业务，但绝不登记RPi提示。 */
  s_expected_link=PX4LITE_MAVLINK_LINK_LORA;
  assert(Px4Lite_CommandHandleMavlinkLong(31011U,9U,191U,42U,191U,1,2,3,4,PX4LITE_MAVLINK_LINK_LORA,s_now)==PX4LITE_OK);
  assert(s_notifications==Expected(1U) && s_command_calls==2U && s_control_calls==2U && !s_hardware_calls);
}
#endif

/** @brief 执行实际CRC到消费入口的完整门禁，关闭RPi或反馈均保持静默。 */
int main(void)
{
#if PX4LITE_ENABLE_RPI_MAVLINK
  TestCrcFramingAndHeartbeat();TestControlConfigurationOnly();TestCommandTargetsAndTrimming();
  TestConfigurationTargetsAndPayloads();TestConfigurationWhileH1Pending();TestMultipleFramesAndLoraScope();
  printf("RPi feedback RX: real CRC / partial / controls+configuration only / MAV2 trim / targets / H1-busy arbitration / consume-once / LoRa / no Comm hardware PASS enabled=%u\n",(unsigned)PX4LITE_ENABLE_RPI_COMMAND_FEEDBACK);
#else
  assert(Px4Lite_RpiMavlinkService(0U)==PX4LITE_IDLE && Px4Lite_MavlinkRxRunRpi(0U)==PX4LITE_IDLE);
  assert(!s_notifications && !s_hardware_calls && !s_command_calls);
  puts("RPi feedback RX disabled: no notification / no hardware / no command PASS");
#endif
  return 0;
}
