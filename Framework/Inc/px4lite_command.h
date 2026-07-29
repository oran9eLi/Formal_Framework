/**
 * @file px4lite_command.h
 * @brief Framework 命令执行边界接口。
 *
 * @details
 * 本模块只负责将已解码的控制命令做目标校验、权限边界和分发，并请求通信层回传
 * COMMAND_ACK。MAVLink RX 只负责逐字段解码，MAVLink TX 只负责排队和发送 ACK。
 * 后续 5G-A、Remote ID 或其他控制面命令必须先进入本边界，不得散落在通信收发文件中。
 */

#ifndef PX4LITE_COMMAND_H
#define PX4LITE_COMMAND_H

#include <stdint.h>
#include "px4lite_types.h"

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

#endif
