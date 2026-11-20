/**
 * @file px4lite_command.h
 * @brief Framework 命令执行边界接口。
 *
 * @details
 * 本模块将已解码的控制命令做目标校验和分发，并请求通信层回传COMMAND_ACK；
 * Pi终态通知只校验端点/载荷并登记声光，不执行控制、不返回ACK。
 * MAVLink RX只负责标准逐字段解码，MAVLink TX只负责排队和发送ACK。
 * 后续 5G-A、Remote ID 或其他控制面命令必须先进入本边界，不得散落在通信收发文件中。
 */

#ifndef PX4LITE_COMMAND_H
#define PX4LITE_COMMAND_H

#include <stdint.h>
#include "px4lite_types.h"

#define PX4LITE_CNS_MODULE_SWITCH_COMMAND 31015U /**< UART6 五模块运行启停与切屏命令。 */
#define PX4LITE_RPI_RESULT_TUNNEL_TYPE 0x8040U /**< Pi命令终态通知TUNNEL类型。 */
#define PX4LITE_RPI_RESULT_VERSION 1U /**< 两字节结果载荷的协议版本。 */
#define PX4LITE_RPI_RESULT_PAYLOAD_LENGTH 2U /**< 声明的内层载荷长度，单位字节。 */
#define PX4LITE_RPI_RESULT_PI_COMPONENT_ID 191U /**< Pi固定组件号，与本机LoRa身份配置独立。 */

/**
 * @brief 处理一条已解码的 MAVLink COMMAND_LONG。
 *
 * @param[in] command MAVLink 命令号。
 * @param[in] source_system 命令来源 system id。
 * @param[in] source_component 命令来源 component id。
 * @param[in] target_system 命令目标 system id，0 表示广播。
 * @param[in] target_component 命令目标 component id，0 表示广播。
 * @param[in] param1 命令参数 1，含义由 command 决定。
 * @param[in] param2 命令参数 2，含义由 command 决定。
 * @param[in] param3 命令参数 3，含义由 command 决定。
 * @param[in] param4 命令参数 4，含义由 command 决定。
 * @param[in] link 收到该命令的链路，ACK 原路返回该链路。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 处理结果。
 * @retval PX4LITE_OK 命令属于本机且已处理或已明确拒绝并排队 ACK。
 * @retval PX4LITE_IDLE 命令不属于本机。
 */
Px4Lite_Result_t Px4Lite_CommandHandleMavlinkLong(uint16_t command, uint8_t source_system, uint8_t source_component, uint8_t target_system, uint8_t target_component, float param1, float param2, float param3, float param4, Px4Lite_MavlinkLink_t link, uint32_t now_ms);

/**
 * @brief UART6完整命令边界：按CNS V0.1校验端点、confirmation和七个参数。
 * @param[in] command 命令号；六类电机动作使用31090～31094及31014，模块切屏使用31015。
 * @param[in] source_system 来源系统号，须为当前动态本机系统号。
 * @param[in] source_component 来源组件号，须为Pi的191。
 * @param[in] target_system 目标系统号，须精确为本机系统号，不接受广播。
 * @param[in] target_component 目标组件号，须为UART6的193。
 * @param[in] confirmation 请求确认序号，CNS V0.1须为0。
 * @param[in] params 七个已标准解码的float参数；不得传NULL。
 * @param[in] now_ms 请求受理时刻，单位真实ms。
 * @return OK表示已排受理结果；IDLE表示端点不符；BUSY表示应答容量不足且普通动作未执行。
 * @note 模块命令在有应答容量时登记启停，ACK 发送成功后发布页面请求和声光。
 *       电机动作原门禁保持独立，急停停车不依赖ACK。
 */
Px4Lite_Result_t Px4Lite_CommandHandleRpiLongV01(uint16_t command,
    uint8_t source_system, uint8_t source_component, uint8_t target_system,
    uint8_t target_component, uint8_t confirmation, const float params[7], uint32_t now_ms);

/**
 * @brief 处理一条已解码的 MAVLink COMMAND_ACK。
 *
 * @param[in] command ACK 对应的 MAVLink 命令号。
 * @param[in] result MAVLink ACK 结果枚举值。
 * @param[in] target_system ACK 目标 system id，0 表示广播。
 * @param[in] target_component ACK 目标 component id，0 表示广播。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 处理结果。
 */
Px4Lite_Result_t Px4Lite_CommandHandleMavlinkAck(uint16_t command, uint8_t result, uint8_t target_system, uint8_t target_component, uint32_t now_ms);

/**
 * @brief 校验一条已通过MAVLink CRC的Pi终态通知，只登记提示，不执行命令或返回ACK。
 * @param[in] source_system 来源系统号，必须精确等于当前非零本机动态系统号。
 * @param[in] source_component 来源组件号，必须为Pi的191。
 * @param[in] target_system 目标系统号，必须精确等于当前本机号，不接受广播。
 * @param[in] target_component 目标组件号，必须为本机UART6组件号，当前193。
 * @param[in] payload_type TUNNEL类型，仅0x8040属于本接口。
 * @param[in] payload 标准TUNNEL解码后的128字节数组；解码器已补齐被裁剪的尾零。
 * @param[in] payload_length 内层声明长度，必须为2字节。
 * @param[in] wire_payload_length 实际在线内层字节数，范围1～128；失败1字节、成功2字节均合法。
 * @param[in] link 物理接收链路，仅允许RPi UART6。
 * @retval PX4LITE_OK 合法通知已交给有界声光队列，队满可能丢提示，不保证实物发声。
 * @retval PX4LITE_IDLE 其他类型或Pi通知模式未启用。
 * @retval PX4LITE_INVALID_PARAM 端点、长度、版本或结果不合法，静默忽略。
 * @note result仅0失败/1成功，与MAV_RESULT不同。没有请求ID，不能按结果值去重；
 *       Pi负责请求级去重与失联旧结果丢弃；未使用的TUNNEL尾部填充不属于两字节协议。
 */
Px4Lite_Result_t Px4Lite_CommandHandleRpiResultTunnel(uint8_t source_system,
                                                     uint8_t source_component,
                                                     uint8_t target_system,
                                                     uint8_t target_component,
                                                     uint16_t payload_type,
                                                     const uint8_t *payload,
                                                     uint8_t payload_length,
                                                     uint8_t wire_payload_length,
                                                     Px4Lite_MavlinkLink_t link);

#endif
