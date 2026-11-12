/**
 * @file px4lite_gnss_raw_uplink.h
 * @brief Framework独立GNSS原帧UART6上行队列，不改变typed测量出口。
 * @details Sensor经Adapter为唯一入队者，Comm为唯一发送消费者；不依赖Debug/屏幕捕获。
 *          只转交完整wire，BAD校验也保真；队满丢新帧，不承诺无损或有效定位。
 */
#ifndef PX4LITE_GNSS_RAW_UPLINK_H
#define PX4LITE_GNSS_RAW_UPLINK_H
#include "px4lite_types.h"

#define PX4LITE_GNSS_RAW_UPLINK_FRAME_TYPE 0x8020U /**< 本地RAW_FRAME子协议。 */
#define PX4LITE_GNSS_RAW_UPLINK_STATS_TYPE 0x8021U /**< 本地RAW_STATS子协议。 */
#define PX4LITE_GNSS_RAW_UPLINK_CAPACITY 4U       /**< 静态完整帧槽，含活跃帧。 */
#define PX4LITE_GNSS_RAW_UPLINK_FRAME_MAX 1024U   /**< 完整原wire最大字节数，避免截断较长NAV-SAT。 */
#define PX4LITE_GNSS_RAW_UPLINK_DATA_MAX 40U      /**< 每片最多原wire数据字节数。 */
#define PX4LITE_GNSS_RAW_UPLINK_PAYLOAD_MAX 64U   /**< 标准TUNNEL内层最大字节数。 */
/** @brief 源拒绝原因；属于软件观察事件，不能当物理空口丢帧数。 */
typedef enum {
  PX4LITE_GNSS_RAW_REJECT_OVERSIZE=1, /**< 超出源组帧器容量，长度可为已知候选总长。 */
  PX4LITE_GNSS_RAW_REJECT_FRAMING=2,  /**< 组帧失步、源重同步或未完整候选被丢弃。 */
  PX4LITE_GNSS_RAW_REJECT_METADATA=3  /**< 非法协议/长度/完整边界或元数据。 */
} Px4Lite_GnssRawRejectReason_t;
/** @brief 当前编码载荷的借用视图，至下一次Peek/Result/Init前有效；不能跨调用持有。 */
typedef struct {
  const uint8_t *payload; /**< 已逐字段编码的小端数据，不包含MAVLink外层。 */
  uint16_t payload_type;  /**< 0x8020原帧片或0x8021统计。 */
  uint8_t length;         /**< RAW_FRAME为24+实际数据字节，RAW_STATS固定64。 */
} Px4Lite_GnssRawUplinkView_t;

/** @brief 调度前初始化；整个队列生命周期状态在一个临界区复位，禁止并发重置发送流程。 */
void Px4Lite_GnssRawUplinkInit(uint32_t now_ms);
/**
 * @brief Sensor/Adapter提交一个借用的完整原wire，函数返回前完成静态复制。
 * @param protocol 1NMEA/2UBX，与typed source枚举不同。
 * @param nmea_type NMEA的0其他/1GGA/2RMC/3GSA/4GSV；UBX为0。
 * @param checksum_valid 原协议校验事实0/1；函数不修正原校验，也不授权导航。
 * @param now_ms Sensor消费完成时刻，ms，0合法，不是物理UART到达时刻。
 * @details 含缺校验短NMEA但有完整$..LF边界也接收；NULL/0或截断/超容量计源拒绝。
 *          入队、队满、源拒绝均推进独立frame_id，因此接收端可看到缺口。
 */
void Px4Lite_GnssRawEnqueue(const uint8_t *bytes, uint16_t length, uint8_t protocol,
                          uint8_t nmea_type, uint8_t class_id, uint8_t message_id,
                          uint8_t checksum_valid, uint32_t now_ms);
/** @brief 记录一个无法交付完整wire的源事件；含input_events，不入队，length不当物理丢字节数。 */
void Px4Lite_GnssRawSourceReject(uint8_t protocol, uint32_t length, uint8_t reason, uint32_t now_ms);
/** @brief Comm每轮最高优先应答前仅回收过期活跃槽；不pin/算CRC/编码/发送，未获调度无硬实时保证。 */
void Px4Lite_GnssRawUplinkPollTimeout(uint32_t now_ms);
/** @brief Comm默认只取得8020原帧片；8021仅显式Stats宏启用时输出，内部计数一直保留。 */
Px4Lite_Result_t Px4Lite_GnssRawUplinkPeek(uint32_t now_ms, Px4Lite_GnssRawUplinkView_t *out);
/** @brief Comm记录真实UART提交结果；每片/统计至多3次、失败50ms间隔；活跃帧超2s中止。 */
void Px4Lite_GnssRawUplinkResult(Px4Lite_Result_t result, uint32_t now_ms);
#endif
