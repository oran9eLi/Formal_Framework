/**
 * @file px4lite_lora_lab.h
 * @brief Framework 层教学发送会话与异步操作接口。
 * @details UI 只提交单槽请求；CommTask 消费，禁止 UI 直接访问驱动或保存 Flash。
 */
#ifndef PX4LITE_LORA_LAB_H
#define PX4LITE_LORA_LAB_H
#include "px4lite_types.h"
#include "px4lite_platform.h"
#include "px4lite_comm_config.h"
/** @brief 互斥的数据组，空口仅发送学生选择的一组。 */
typedef enum { PX4LITE_LAB_GNSS=0, PX4LITE_LAB_ATTITUDE, PX4LITE_LAB_POWER, PX4LITE_LAB_ENVIRONMENT } Px4Lite_LabData_t;
/** @brief 用户主动操作；EDIT 会停止连续发送，RESTORE 分别返回地址与信道结果。 */
typedef enum { PX4LITE_LAB_EDIT=0, PX4LITE_LAB_SEND, PX4LITE_LAB_START, PX4LITE_LAB_STOP,
  PX4LITE_LAB_SAVE_NODE, PX4LITE_LAB_APPLY_CHANNEL, PX4LITE_LAB_RESTORE } Px4Lite_LabOperation_t;
/** @brief 会话结果，QUEUED 只证明进入驱动发送流程。 */
typedef enum { PX4LITE_LAB_STOPPED=0, PX4LITE_LAB_RUNNING, PX4LITE_LAB_QUEUED,
  PX4LITE_LAB_NO_DATA, PX4LITE_LAB_RATE_LIMIT, PX4LITE_LAB_UNAVAILABLE, PX4LITE_LAB_CONFIG_BUSY,
  PX4LITE_LAB_PROFILE, PX4LITE_LAB_ERROR } Px4Lite_LabResult_t;
/** @brief 完整操作请求；所有参数范围在 CommTask 再次校验。 */
typedef struct {
  Px4Lite_LabOperation_t operation; /**< 操作类型。 */
  Px4Lite_LabData_t data; /**< 0..3 数据组。 */
  uint32_t interval_ms; /**< 1000/2000/5000/10000 ms。 */
  uint8_t node_id; /**< 1..250；RESTORE 使用 UID 默认。 */
  uint8_t channel; /**< 23/26/29/32。 */
} Px4Lite_LabRequest_t;
/** @brief 应用只读状态，配置和实验结果各自独立。 */
typedef struct {
  uint32_t generation; /**< 可见状态变更序号。 */
  uint32_t request_id; /**< 最近消费的用户操作序号，按启动会话递增。 */
  uint32_t queued_frames; /**< 本机会话已提交帧数，非远端确认。 */
  uint32_t rx_frames; /**< 本机实际 MAVLink 接收帧数。 */
  uint32_t crc_errors; /**< 当前硬件会话 CRC 拒绝次数。 */
  uint32_t rx_drops; /**< 当前硬件会话完整帧队列丢弃次数。 */
  uint16_t loss_rate_x10; /**< 当前硬件会话序号估算损失，百分比 * 10。 */
  uint32_t interval_ms; /**< 当前连续发送间隔，单位 ms。 */
  Px4Lite_LabData_t data; /**< 当前数据组。 */
  Px4Lite_LabResult_t result; /**< 当前实验结果。 */
  Px4Lite_LoRaConfig_t radio; /**< 寄存器读回与本机发送事实。 */
  Px4Lite_CommConfigSnapshot_t address; /**< 地址保存结果，含重启生效提示。 */
  uint8_t running; /**< 连续发送由学生启动。 */
  uint8_t pending; /**< 一组待发送；不会积累历史组。 */
  uint8_t request_pending; /**< 请求槽是否占用。 */
} Px4Lite_LabSnapshot_t;
/** @brief 启动默认 Attitude/2000 ms/STOPPED。 */
void Px4Lite_LoraLabInit(uint32_t now_ms);
/** @brief 任务上下文提交；Stop 优先且清除尚未消费请求。 */
Px4Lite_Result_t Px4Lite_LoraLabRequest(const Px4Lite_LabRequest_t *request);
/** @brief CommTask 消费请求、配置收尾与周期调度。 */
void Px4Lite_LoraLabRun(uint32_t now_ms);
/** @brief 复制已发布状态，不触碰硬件。 */
void Px4Lite_LoraLabCopy(Px4Lite_LabSnapshot_t *out);
/** @brief CommTask 查询本次待发送组。 */
uint8_t Px4Lite_LoraLabPending(Px4Lite_LabData_t *data);
/** @brief 编码器报告逐帧提交或组结束结果；不表示远端收到。 */
void Px4Lite_LoraLabTxResult(Px4Lite_Result_t result,uint8_t complete,uint32_t now_ms);
#endif
