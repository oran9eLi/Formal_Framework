/**
 * @file px4lite_gnss_uplink.h
 * @brief Framework 层协议无关 GNSS 上行编码与有界发送状态。
 * @details 仅由 Comm 调用；读取 Framework 快照，逐字段生成标准 TUNNEL 的本地载荷。
 *          不访问 BSP/Driver，不消费 UART，不增加任务或运行期内存分配。
 */
#ifndef PX4LITE_GNSS_UPLINK_H
#define PX4LITE_GNSS_UPLINK_H

#include "px4lite_types.h"

#define PX4LITE_GNSS_UPLINK_NAV_TYPE    0x8011U /**< 本项目 NAV 子协议；不是注册 MAVLink 方言。 */
#define PX4LITE_GNSS_UPLINK_DOP_TYPE    0x8012U /**< 七种 DOP，分两包。 */
#define PX4LITE_GNSS_UPLINK_COUNTS_TYPE 0x8013U /**< 四项 GPS/北斗分类计数。 */
#define PX4LITE_GNSS_UPLINK_LINK_TYPE   0x8014U /**< Driver/BSP/统一时间事实，分两包。 */
#define PX4LITE_GNSS_UPLINK_PAYLOAD_MAX 64U    /**< 单包内层容量，单位 B。 */
#define PX4LITE_GNSS_UPLINK_PARTS       6U     /**< 一个冻结 bundle 的包数。 */

/**
 * @brief 当前一个待发载荷的借用视图，仅在下一次 Peek/Result/Init 前有效。
 * @details payload 已逐字段编码为小端；调用方仅套标准 MAVLink2 TUNNEL，不修改字节。
 *          bundle_id 重启归零，不能充当全局唯一 boot_id。
 *          DOP条目的value_x100是reported_value；flags.valid清零时不能当可用DOP，
 *          消费端可保留reported值作审计，但业务value应为null。
 */
typedef struct {
  const uint8_t *payload; /**< 静态载荷；不允许调用方修改或跨下一次调用保留。 */
  uint16_t payload_type;  /**< 0x8011..0x8014。 */
  uint8_t length;         /**< 实际内层长度，32 或 64 B。 */
  uint8_t frame_index;    /**< bundle 内帧号，0..5，非 TUNNEL 分包号。 */
} Px4Lite_GnssUplinkView_t;

/** @brief Comm 初始化；首组允许立即生成，bundle 代数从 0 开始。 */
void Px4Lite_GnssUplinkInit(uint32_t now_ms);
/**
 * @brief 按 1s 节奏冻结同一次源副本并返回一个到期待发包；未知/无定位也有包。
 * @details 同组保留物理量与源时刻；每次编码/重试按当前真实毫秒重新计算年龄及有效位。
 *          头snapshot_uptime_ms是冻结分组时刻，body年龄是本次编码年龄，不能直接相减。
 *          新周期取消未完成旧组，不追赶积压历史。
 * @return OK 有包；IDLE 未到期/本组已完成；INVALID_PARAM 输出为空。
 */
Px4Lite_Result_t Px4Lite_GnssUplinkPeek(uint32_t now_ms, Px4Lite_GnssUplinkView_t *out);
/**
 * @brief 记录一次真实 UART6 发送结果；每包最多三次尝试，失败等待 50ms。
 * @details 仅与最近 OK 的 Peek 配对；成功或耗尽重试推进一包，不代表 Pi 已收到。
 */
void Px4Lite_GnssUplinkResult(Px4Lite_Result_t result, uint32_t now_ms);

#endif
