/**
 * @file px4lite_diagnostic_capture.h
 * @brief Framework 有界诊断捕获；独立于生产解析、队列和通信状态。
 * @details Comm处理请求；MAVLink和GNSS互斥共用固定区。会话/token隔离迟到结果，
 *          协议发现为上电累计接收事实，独立于捕获会话、筛选和暂停。
 */
#ifndef PX4LITE_DIAGNOSTIC_CAPTURE_H
#define PX4LITE_DIAGNOSTIC_CAPTURE_H
#include "px4lite_types.h"
#include "px4lite_rpi_h1_echo.h"
#include "px4lite_rpi_baud_switch.h"
/** @brief Comm单写串口实验快照；本机阶段不推断Server结果。 */
typedef struct {
  Px4Lite_RpiH1EchoStats_t h1; /**< H1本机处理/响应事实。 */
  Px4Lite_RpiBaudSwitchStats_t b1; /**< B1旧速率回包与重配置事实。 */
  uint32_t measured_ms; /**< 快照更新毫秒。 */
} Px4Lite_SerialExperiment_t;
/** @brief 不访问UART的串口实验副本。 */
Px4Lite_Result_t Px4Lite_CopySerialExperiment(Px4Lite_SerialExperiment_t *out);
/** @brief 复制 Comm 发布的通用 UART6 快照；与诊断捕获是否运行无关，未发布返回 NOT_READY。 */
Px4Lite_Result_t Px4Lite_CopySerialMonitor(Px4Lite_SerialMonitor_t *out);
#define PX4LITE_CAPTURE_CAPACITY 4U /**< 完整帧槽数量。 */
#define PX4LITE_CAPTURE_SUMMARIES 16U /**< 汇总键容量。 */
#define PX4LITE_CAPTURE_FRAME_BYTES 280U /**< MAVLink2含签名的最大帧字节数。 */
#define PX4LITE_CAPTURE_NMEA_BYTES 128U /**< NMEA包含原始分隔符的最大字节数。 */
#define PX4LITE_CAPTURE_GNSS_BYTES 392U /**< 完整GNSS原帧容量，单位B；不接受截断帧。 */
/** @brief 一次仅捕获一个入口/方向；GNSS与MAVLink互斥，原NMEA编号4保持兼容。 */
typedef enum { PX4LITE_CAPTURE_LORA_RX=0, PX4LITE_CAPTURE_LORA_TX, PX4LITE_CAPTURE_RPI_RX,
  PX4LITE_CAPTURE_RPI_TX, PX4LITE_CAPTURE_GNSS=4, PX4LITE_CAPTURE_NMEA=PX4LITE_CAPTURE_GNSS } Px4Lite_CaptureSource_t;
/** @brief GNSS元数据筛选；只决定记录，不改变生产解析或常驻协议发现。 */
typedef enum {
  PX4LITE_CAPTURE_FILTER_ALL=0, /**< NMEA及UBX全部原帧。 */
  PX4LITE_CAPTURE_FILTER_GGA=1, PX4LITE_CAPTURE_FILTER_RMC=2,
  PX4LITE_CAPTURE_FILTER_GSA=3, PX4LITE_CAPTURE_FILTER_GSV=4,
  PX4LITE_CAPTURE_FILTER_NMEA=5, /**< 全部NMEA句型。 */
  PX4LITE_CAPTURE_FILTER_UBX=6,  /**< 全部UBX class/id。 */
  PX4LITE_CAPTURE_FILTER_PVT=7,  /**< UBX NAV-PVT，class1/id7。 */
  PX4LITE_CAPTURE_FILTER_DOP=8,  /**< UBX NAV-DOP，class1/id4。 */
  PX4LITE_CAPTURE_FILTER_SAT=9   /**< UBX NAV-SAT，class1/id0x35。 */
} Px4Lite_CaptureFilter_t;
/** @brief 原帧协议编码，不表示定位或测量有效。 */
typedef enum { PX4LITE_CAPTURE_PROTOCOL_NMEA=1, PX4LITE_CAPTURE_PROTOCOL_UBX=2 } Px4Lite_CaptureProtocol_t;
/** @brief 学生显式动作；Start新建会话，Clear暂停并换会话，不改生产。 */
typedef enum { PX4LITE_CAPTURE_START=0, PX4LITE_CAPTURE_PAUSE, PX4LITE_CAPTURE_CLEAR,
  PX4LITE_CAPTURE_SELECT } Px4Lite_CaptureOperation_t;
/** @brief 硬件事实阶段，均不证明对端接收。 */
typedef enum { PX4LITE_CAPTURE_SUBMITTING=0, PX4LITE_CAPTURE_QUEUED, PX4LITE_CAPTURE_UART_DONE,
  PX4LITE_CAPTURE_MODULE_IDLE, PX4LITE_CAPTURE_ERROR, PX4LITE_CAPTURE_RX_VALID,
  PX4LITE_CAPTURE_TIMEOUT } Px4Lite_CaptureStage_t;
/** @brief 非阻塞固定请求槽；GNSS的filter使用0..9，旧0..4含义保持。 */
typedef struct {
  uint8_t operation; /**< CaptureOperation。 */
  uint8_t source; /**< CaptureSource。 */
  uint8_t filter; /**< GNSS筛选，Px4Lite_CaptureFilter_t，0..9。 */
  uint8_t reserved; /**< 对齐。 */
} Px4Lite_CaptureRequest_t;
/** @brief 完整原始帧的稳定副本；CRC/签名仍为线上原字节。 */
typedef struct {
  uint32_t session_id; /**< 捕获会话代数；0不用。 */
  uint32_t token; /**< 帧代数；0不用。 */
  uint32_t time_ms; /**< RX完成/提交时间，ms。 */
  uint32_t msg_id; /**< 24位MAVLink消息号。 */
  uint16_t length; /**< 原始帧字节数，<=280。 */
  uint8_t system_id; /**< MAVLink sysid。 */
  uint8_t component_id; /**< MAVLink compid。 */
  uint8_t sequence; /**< MAVLink线上序号。 */
  uint8_t source; /**< CaptureSource。 */
  uint8_t stage; /**< CaptureStage。 */
  uint8_t flags; /**< bit0签名字节存在；bit1已接受帧已计数；不表示对端收到。 */
  uint8_t bytes[PX4LITE_CAPTURE_FRAME_BYTES]; /**< 原始STX/header/payload/CRC/signature。 */
} Px4Lite_CaptureFrame_t;
/** @brief NMEA稳定副本，原始CR/LF计入length，不额外添加分隔符。 */
typedef struct {
  uint32_t session_id; /**< 会话。 */
  uint32_t token; /**< 句子代数。 */
  uint32_t time_ms; /**< 收齐时间，ms。 */
  uint16_t length; /**< 实际字节数，<=128。 */
  uint8_t type; /**< 0其他/1GGA/2RMC/3GSA/4GSV。 */
  uint8_t checksum_valid; /**< 1校验通过，0失败或缺失。 */
  uint8_t bytes[PX4LITE_CAPTURE_NMEA_BYTES]; /**< 原始字节。 */
} Px4Lite_NmeaSentence_t;
/** @brief GNSS稳定原帧副本；含坏校验帧，长度外不作为数据，不推断有效定位。 */
typedef struct {
  uint32_t session_id; /**< 手动捕获会话代数。 */
  uint32_t token;      /**< 本会话原帧代数。 */
  uint32_t time_ms;    /**< Driver消费完整帧的服务时刻，ms，非逐字节硬件到达。 */
  uint16_t length;     /**< 完整wire字节数，1..392。 */
  uint8_t protocol;    /**< 1=NMEA，2=UBX。 */
  uint8_t nmea_type;   /**< NMEA的0其他/1GGA/2RMC/3GSA/4GSV；UBX时0。 */
  uint8_t class_id;    /**< UBX class，NMEA时0。 */
  uint8_t message_id;  /**< UBX id，NMEA时0。 */
  uint8_t checksum_valid; /**< 0坏/缺校验，1生产解析器确认校验通过；不等于定位有效。 */
  uint8_t bytes[PX4LITE_CAPTURE_GNSS_BYTES]; /**< 完整原字节，保留原校验和分隔符。 */
} Px4Lite_GnssCapturedFrame_t;
/** @brief MAV键为(source,sysid,compid,msgid)；GNSS键为(source,protocol,class,id/type)。 */
typedef struct {
  uint32_t msg_id; /**< MAV消息号；GNSS为UBX id或NMEA type。 */
  uint32_t first_ms; /**< 本会话首帧时间，ms。 */
  uint32_t last_ms; /**< 最近帧时间，ms。 */
  uint32_t count; /**< MAV有效RX/接受TX次数；GNSS原帧数含坏校验，均不代表有效定位。 */
  uint32_t latest_token; /**< 最近原始帧代数，含失败尝试；槽覆盖后可能不可复制。 */
  uint8_t system_id; /**< MAV来源系统；GNSS协议1/2。 */
  uint8_t component_id; /**< MAV来源组件；GNSS为UBX class或NMEA的0。 */
  uint8_t source; /**< 来源入口。 */
  uint8_t reserved; /**< 对齐。 */
} Px4Lite_CaptureSummary_t;
/** @brief 独立诊断状态；所有计数不等同生产RX丢帧或射频成功。 */
typedef struct {
  uint32_t session_id; /**< 当前会话。 */
  uint32_t request_id; /**< 最近接受请求ID。 */
  uint32_t completed_request_id; /**< 已执行请求ID。 */
  uint32_t count; /**< 本会话MAV有效RX/接受TX/GNSS原帧数，GNSS含坏校验。 */
  uint32_t tx_attempts; /**< 选中TX入口的原始提交尝试条数，含失败。 */
  uint32_t tx_timeouts; /**< 本会话TX超时终态次数，可能有部分字节出线。 */
  uint32_t tx_errors; /**< 本会话TX非超时错误/中止次数。 */
  uint32_t overwritten; /**< 满环覆盖旧槽次数。 */
  uint32_t capture_drops; /**< 无法存入诊断记录次数。 */
  uint32_t summary_drops; /**< 汇总表满，帧仍可入环。 */
  uint32_t rejected; /**< 选中入口的解析拒绝/超长计数。 */
  uint32_t latest_token; /**< 本会话最新token。 */
  int32_t result; /**< 最近已执行请求结果，Px4Lite_Result。 */
  uint8_t source; /**< 当前捕获入口。 */
  uint8_t filter; /**< GNSS筛选0..9。 */
  uint8_t running; /**< 1捕获，0暂停。 */
  uint8_t pending; /**< 1请求待Comm处理。 */
  uint8_t summary_count; /**< 已用汇总键。 */
  uint8_t stored_count; /**< 完整帧槽占用。 */
  uint16_t reserved; /**< 对齐。 */
  uint32_t last_nmea_ms; /**< 上电最近完整且校验通过NMEA服务时刻，ms；须检查seen bit0。 */
  uint32_t last_ubx_ms; /**< 上电最近完整且校验通过UBX服务时刻，ms；须检查seen bit1。 */
  uint8_t gnss_seen_mask; /**< 常驻RX事实：bit0 NMEA/bit1 UBX；0ms合法，不随会话/筛选/暂停清除。 */
} Px4Lite_CaptureStatus_t;
/** @brief 调度前复位，不由页面切换调用。 */
void Px4Lite_CaptureInit(void);
/** @brief 排队一个动作；槽忙返回BUSY，不覆盖。 */
Px4Lite_Result_t Px4Lite_CaptureRequest(const Px4Lite_CaptureRequest_t *request);
/** @brief 只由Comm每轮处理请求。 */
void Px4Lite_CaptureWorkRun(void);
/** @brief 原子复制状态。 */
Px4Lite_Result_t Px4Lite_CaptureCopyStatus(Px4Lite_CaptureStatus_t *out);
/** @brief 复制本会话一条汇总；会话变化拒绝。 */
Px4Lite_Result_t Px4Lite_CaptureCopySummary(uint32_t session, uint8_t index, Px4Lite_CaptureSummary_t *out);
/** @brief 按会话/token复制完整帧；覆盖后返回NOT_READY。 */
Px4Lite_Result_t Px4Lite_CaptureCopyFrame(uint32_t session, uint32_t token, Px4Lite_CaptureFrame_t *out);
/** @brief 只复制匹配槽的阶段，不改变稳定字节副本。 */
Px4Lite_Result_t Px4Lite_CaptureCopyStage(uint32_t session, uint32_t token, uint8_t *stage);
/** @brief 旧NMEA包装仅复制protocol1，UBX/过期槽返回NOT_READY且不写输出。 */
Px4Lite_Result_t Px4Lite_CaptureCopyNmea(uint32_t session, uint32_t token, Px4Lite_NmeaSentence_t *out);
/** @brief 复制完整GNSS原帧稳定副本；会话/槽不符返回NOT_READY且不改输出。 */
Px4Lite_Result_t Px4Lite_CaptureCopyGnss(uint32_t session, uint32_t token, Px4Lite_GnssCapturedFrame_t *out);
/** @brief Adapter提交已由生产解析器校验通过的原始RX帧。 */
void Px4Lite_CaptureRx(uint8_t source, const uint8_t *bytes, uint16_t length, uint32_t now_ms);
/** @brief Adapter提交解析拒绝事实，绝不改变生产状态。 */
void Px4Lite_CaptureReject(uint8_t source);
/** @brief LoRa发送前创建关联；仅在驱动空闲时调用，返回0表示未捕获。 */
uint32_t Px4Lite_CaptureTxBegin(const uint8_t *bytes, uint16_t length, uint32_t now_ms);
/** @brief 提交结果；OK仅排队，硬件完成另报。 */
void Px4Lite_CaptureTxSubmit(uint32_t token, Px4Lite_Result_t result);
/** @brief Driver任务上下文阶段事实，迟到结果按独立pending会话/token核对。 */
void Px4Lite_CaptureTxStage(uint8_t stage);
/** @brief RPi阻塞发送返回后捕获原始提交字节及UART_DONE/ERROR/TIMEOUT事实。 */
void Px4Lite_CaptureRpiTx(const uint8_t *bytes, uint16_t length, uint32_t now_ms, uint8_t stage);
/** @brief sensor提交完整原始NMEA；提交前/临界区内再次核对会话门禁。 */
void Px4Lite_CaptureNmea(const uint8_t *bytes, uint16_t length, uint8_t type, uint8_t checksum_valid, uint32_t now_ms);
/** @brief 提交UBX原帧；完整且校验正确可发现协议，超392B只舍弃屏幕存储，不截断原始上行。 */
void Px4Lite_CaptureUbx(const uint8_t *bytes, uint16_t length, uint8_t class_id, uint8_t message_id, uint8_t checksum_valid, uint32_t now_ms);
#endif
