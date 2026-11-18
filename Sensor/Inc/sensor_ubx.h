/**
 * @file sensor_ubx.h
 * @brief Sensor 层 UBX 有界流式组帧与校验接口。
 * @details 仅依赖标准整数类型，负责原始 UBX 帧边界和校验；不解析导航语义，
 *          不访问 HAL、RTOS、BSP 或 Framework，不分配动态内存。
 */
#ifndef SENSOR_UBX_H
#define SENSOR_UBX_H

#include <stdint.h>

/** @brief 完整RAW转发的最大UBX载荷1016B；不等于旧392B屏幕/Debug镜像容量。 */
#define UBX_PAYLOAD_CAPACITY 1016U

/** @brief UBX 帧头、长度和校验的总开销，单位：字节。 */
#define UBX_FRAME_OVERHEAD 8U

/** @brief 单字节输入产生的事件；所有完成事件均在消费 CK_B 后返回。 */
typedef enum {
  UBX_RESULT_NONE = 0,        /**< 尚无完整帧；包含同步搜索和未完成帧。 */
  UBX_RESULT_FRAME,           /**< 校验通过且载荷完整，可读取当前帧。 */
  UBX_RESULT_CHECKSUM_ERROR,  /**< 完整帧校验失败；优先于载荷超长事件。 */
  UBX_RESULT_OVERSIZE         /**< 校验通过但载荷超过容量，禁止按完整帧使用。 */
} Ubx_Result_t;

/**
 * @brief 单一字节流的解析状态，由调用者静态持有。
 * @details 只有 UBX_RESULT_FRAME 时 class_id、message_id、length、payload
 *          才构成已验证完整帧；其内容至少保持到下一次Ubx_Feed调用。
 *          FRAME或CHECKSUM_ERROR事件后，若length不超过容量，头/载荷及received_ck_a/b
 *          也可仅作为完整原帧观察；必须保留事件校验结果，坏帧不得用于测量。
 *          internal_ 字段只供编解码器使用。每条输入流必须独占一个实例。
 */
typedef struct {
  uint8_t class_id;                              /**< 消息类别，0..255。 */
  uint8_t message_id;                            /**< 类别内消息编号，0..255。 */
  uint16_t length;                               /**< 线上载荷长度，0..65535 字节。 */
  uint8_t payload[UBX_PAYLOAD_CAPACITY];          /**< 容量内完整载荷；测量只允许FRAME，坏校验帧仅供原始观察。 */
  uint8_t received_ck_a;                         /**< 完整帧实际收到的CK_A，不是重算值。 */
  uint8_t received_ck_b;                         /**< 完整帧实际收到的CK_B，不是重算值。 */
  uint16_t internal_position;                    /**< 当前已消费载荷字节数。 */
  uint8_t internal_state;                        /**< 内部组帧状态，不供调用者修改。 */
  uint8_t internal_ck_a;                         /**< 运行中的 Fletcher CK_A，模 256。 */
  uint8_t internal_ck_b;                         /**< 运行中的 Fletcher CK_B，模 256。 */
  uint8_t internal_received_ck_a;                /**< 收到的 CK_A，等待 CK_B 后统一判定。 */
} Ubx_Parser_t;

/** @brief 清空解析器并开始同步搜索；传入空指针时不执行操作。 */
void Ubx_Init(Ubx_Parser_t *parser);

/**
 * @brief 消费一个原始字节，返回组帧事件。
 * @details 超长载荷仍按线上长度消费全部载荷及两字节校验，不在载荷内部重同步。
 *          校验错误也消费完整两字节校验，避免把未完成帧提前当成成功。
 *          空指针返回 NONE；本接口无超时，丢字节后由流所有者调用 Init 重置半帧。
 */
Ubx_Result_t Ubx_Feed(Ubx_Parser_t *parser, uint8_t byte);

/**
 * @brief 只读查询是否正在接收 UBX 前导、头、载荷或校验，返回 0 或 1。
 * @details 初始化和完整帧结束后返回 0；错误 CK_B 为 B5 而保留下一前导时返回 1。
 *          空指针返回 0。调用者可在 Feed 前后查询，以隔离载荷内的其他协议伪帧头。
 */
uint8_t Ubx_IsReceiving(const Ubx_Parser_t *parser);

/**
 * @brief 构造完整 UBX 帧，返回总字节数；参数无效或容量不足返回 0 且不写输出。
 * @param class_id 消息类别。
 * @param message_id 消息编号。
 * @param payload 载荷；仅 length 为 0 时允许为空。允许与 out 指向的缓冲区重叠。
 * @param length 载荷字节数；总长必须可由 uint16_t 表示，不受接收容量限制。
 * @param out 调用者提供的输出缓冲区。
 * @param capacity 输出缓冲区容量，单位：字节。
 */
uint16_t Ubx_BuildFrame(uint8_t class_id, uint8_t message_id,
                        const uint8_t *payload, uint16_t length,
                        uint8_t *out, uint16_t capacity);

#endif
