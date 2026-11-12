/**
 * @file sensor_gnss.h
 * @brief GNSS 双协议测量、接收事实与非阻塞接入状态接口。
 *
 * @details
 * GNSS驱动固定使用115200bit/s，负责NMEA/UBX校验、解析、版本识别和配置应答事实，不自动扫速。
 * 驱动层不判定 Framework
 * OFFLINE/FAILED 状态，不直接发布 Framework topic；数据必须经 Platform Adapter
 * 转换后进入 Framework。
 */

#ifndef SENSOR_GNSS_H
#define SENSOR_GNSS_H

#include <stdint.h>

/**
 * @brief GNSS 数据事实状态。
 */
typedef enum {
  GNSS_DATA_NONE = 0, /**< 尚未解析到有效句子。 */
  GNSS_DATA_NO_FIX,   /**< 已收到 GNSS 数据，但当前无有效定位。 */
  GNSS_DATA_FIX_OK    /**< 当前定位有效。 */
} Gnss_DataState_t;

/**
 * @brief GNSS 驱动返回值。
 */
typedef enum {
  GNSS_RESULT_OK = 0,       /**< 操作成功。 */
  GNSS_RESULT_NO_DATA,      /**< 当前无新数据。 */
  GNSS_RESULT_NO_FIX,       /**< 有数据但无有效定位。 */
  GNSS_RESULT_INVALID_PARAM, /**< 参数非法。 */
  GNSS_RESULT_IO_ERROR      /**< UART/DMA 或 BSP 访问失败。 */
} Gnss_Result_t;

#define GNSS_SAT_CAPACITY 64U      /**< 本机静态卫星表最多保留64颗；原帧仍独立完整转发。 */
#define GNSS_SAT_PAGE_CAPACITY 5U  /**< 跨层只读页最多5项，避免在消费者任务栈复制整张卫星表。 */
#define GNSS_SAT_VALID_ELEVATION (1U << 0) /**< 仰角在协议允许的-90..90度范围内。 */
#define GNSS_SAT_VALID_AZIMUTH   (1U << 1) /**< 仰角已知且方位角在0..360度范围内。 */

/** @brief 单颗UBX NAV-SAT报告事实；星座编号与卫星编号共同标识身份，不授权位置有效。 */
typedef struct {
  uint8_t gnss_id;        /**< 原UBX星座编号；未知编号原样保留，不能当GPS。 */
  uint8_t sv_id;          /**< 该星座内卫星编号，范围0..255。 */
  uint8_t cno_dbhz;       /**< 模块报告的载噪比，单位dB-Hz，0仍是报告值；须结合quality解释。 */
  uint8_t quality;        /**< flags.bit0..2：0无信号、1搜索、2捕获、3不可用、4码锁定、5..7码/载波锁定。 */
  uint8_t used;           /**< 原flags.bit3，1表示报告信号用于当前导航，不等于PVT定位有效。 */
  uint8_t valid_mask;     /**< GNSS_SAT_VALID_*只控制两个角度；不借used或载噪比授权角度。 */
  int8_t elevation_deg;   /**< 原仰角，单位度；只有相应有效位为1时解释。 */
  uint8_t reserved;       /**< 保留为0。 */
  int16_t azimuth_deg;    /**< 原方位角，单位度，正北0/360、顺时针；有效位另判。 */
  uint16_t reserved2;     /**< 保留为0，保持条目12字节稳定布局。 */
} Gnss_Satellite_t;

/** @brief 同一NAV-SAT完整历元的固定5星只读页；序号和0ms均合法，以available区分空缓存。 */
typedef struct {
  uint32_t sequence;       /**< 有效NAV-SAT帧代数，uint32自然回绕。 */
  uint32_t received_ms;    /**< sensor消费完整有效帧时刻，单位ms。 */
  uint32_t itow_ms;        /**< 模块报告的GPS周内导航历元，单位ms，不与本机毫秒时钟相减。 */
  uint8_t total;           /**< 原numSvs，范围0..255；为已知可见或跟踪列表，不直接称跟踪总数。 */
  uint8_t stored;          /**< 本机保留数量，范围0..GNSS_SAT_CAPACITY。 */
  uint8_t page;            /**< 页号，从0起。 */
  uint8_t count;           /**< 本页实际条数，范围0..GNSS_SAT_PAGE_CAPACITY。 */
  uint8_t truncated;       /**< 原报告数量超过本机保留容量时为1。 */
  uint8_t available;       /**< 曾收到完整结构有效的NAV-SAT为1；未定位也可有有效卫星表。 */
  uint8_t used_count;      /**< 对完整原报告所有条目统计used标志之和，范围0..255。 */
  uint8_t reserved;        /**< 保留为0。 */
  Gnss_Satellite_t entries[GNSS_SAT_PAGE_CAPACITY]; /**< 本页条目；count之后清零，不作为真实零号卫星。 */
} Gnss_SatellitePage_t;

/**
 * @brief 原子复制最近NAV-SAT的一页元数据及固定最多5项，不读硬件、不消费数据。
 * @param[in] page 从0起的页号；报告0颗卫星时page0仍为有效空页。
 * @param[out] out 输出副本，不能为NULL；失败不改输出。
 * @return 未有完整卫星帧为NO_DATA，空指针或越界页为INVALID_PARAM，其余OK。
 * @note 驱动不执行过期策略，消费者依据received_ms判新鲜度；NMEA/GSV不伪填为UBX卫星表。
 */
Gnss_Result_t Sensor_GNSS_CopySatellitePage(uint8_t page, Gnss_SatellitePage_t *out);

#define GNSS_VALID_POSITION (1UL << 0) /**< PVT/GGA经纬度和定位质量字段有效。 */
#define GNSS_VALID_VELOCITY (1UL << 1) /**< PVT/RMC地速和航迹角字段有效。 */
#define GNSS_VALID_TIME (1UL << 2) /**< PVT/RMC完整UTC日期与时间有效。 */
#define GNSS_VALID_ALTITUDE (1UL << 3) /**< PVT/GGA海拔高度字段有效。 */
#define GNSS_VALID_NAV_STATUS (1UL << 4) /**< 完整PVT/GGA的当前可用定位状态和总使用卫星数有效，不授权坐标。 */

/** @brief 统一事实来源编码；不与UBX fixType、NMEA systemID或星座编号混用。 */
#define GNSS_SOURCE_NONE 0U
#define GNSS_SOURCE_UBX_PVT 1U
#define GNSS_SOURCE_NMEA_GGA 2U
#define GNSS_SOURCE_NMEA_RMC 3U
#define GNSS_SOURCE_NMEA_GSA 4U
#define GNSS_SOURCE_NMEA_GSV 5U
#define GNSS_SOURCE_UBX_DOP 6U
#define GNSS_DOP_COUNT 7U /**< DOP事实数量，索引依次G/P/T/V/H/N/E。 */
#define GNSS_DOP_G 0U
#define GNSS_DOP_P 1U
#define GNSS_DOP_T 2U
#define GNSS_DOP_V 3U
#define GNSS_DOP_H 4U
#define GNSS_DOP_N 5U
#define GNSS_DOP_E 6U
#define GNSS_COUNTS_COUNT 4U /**< 计数事实数量，索引依次GPS可见/北斗可见/GPS使用/北斗使用。 */
#define GNSS_COUNT_GPS_VISIBLE 0U
#define GNSS_COUNT_BDS_VISIBLE 1U
#define GNSS_COUNT_GPS_USED 2U
#define GNSS_COUNT_BDS_USED 3U

/**
 * @brief GNSS 最新解析快照；记录测量事实，年龄策略由 Framework 消费者判断。
 */
typedef struct {
  Gnss_DataState_t data_state; /**< 数据事实状态；组件离线/失败归 Framework 判定。 */
  uint32_t valid_mask;          /**< 最新各测量字段有效位，使用 GNSS_VALID_*。 */
  uint32_t position_update_ms;  /**< 最近有效PVT/GGA位置时间，单位ms。 */
  uint32_t velocity_update_ms;  /**< 最近有效PVT/RMC地速/航迹角时间，单位ms。 */
  uint32_t time_update_ms;      /**< 最近有效PVT/RMC日期/时间接收时刻，单位ms。 */
  uint32_t altitude_update_ms;  /**< 最近有效PVT/GGA海拔时间，单位ms。 */

  uint8_t fix_valid;     /**< 定位有效标志，1 表示定位有效。 */
  uint8_t fix_quality;   /**< 定位质量使用GGA质量码；PVT经显式转换，不直接填UBX fixType。 */
  uint8_t fix_dimension; /**< GSA/PVT定位维度，取0/2/3，0表示未提供可用维度。 */

  uint8_t satellites; /**< 最近完整PVT/GGA报告的总使用卫星数，0合法；须结合状态有效位和年龄。 */

  uint8_t gps_visible_sats; /**< GSV 报告的 GPS 可见卫星数。 */
  uint8_t bds_visible_sats; /**< GSV 报告的北斗可见卫星数。 */

  uint8_t gps_used_sats; /**< GSA 报告的 GPS 参与解算卫星数。 */
  uint8_t bds_used_sats; /**< GSA 报告的北斗参与解算卫星数。 */

  uint16_t hdop_cm;  /**< 水平精度因子，单位：HDOP * 100。 */
  uint32_t utc_sec;  /**< UTC 当日秒数，单位：s。 */
  uint32_t utc_date; /**< RMC 日期，压缩格式 yymmdd；无有效日期时为 0。 */

  int32_t latitude_deg_e7;  /**< 纬度，单位：degree * 1e7。 */
  int32_t longitude_deg_e7; /**< 经度，单位：degree * 1e7。 */
  int32_t altitude_mm;      /**< 海拔高度，单位：mm。 */

  uint32_t speed_cms;      /**< 地速，单位：cm/s。 */
  uint32_t heading_deg100; /**< 地面航向，单位：degree * 100。 */

  uint32_t rx_sequence; /**< 接收解析序号，每次有效解析推进。 */
  uint32_t last_rx_ms;  /**< 最近收到完整且校验通过的已识别NMEA/UBX报文时间，单位ms。 */
  uint32_t last_fix_ms; /**< 最近有效定位时间，单位：ms。 */
  uint32_t status_update_ms; /**< 最近语义有效完整PVT/GGA状态时刻，单位ms，0合法；RMC/GSA/GSV/ACK不续期。 */
  uint8_t position_source; /**< 最近有效位置的GNSS_SOURCE_*，只随真实域更新改变。 */
  uint8_t velocity_source; /**< 最近有效地速/航迹来源。 */
  uint8_t time_source; /**< 最近有效完整UTC来源。 */
  uint8_t altitude_source; /**< 最近有效海拔来源。 */
  uint8_t status_source; /**< 最近完整导航状态来源PVT/GGA。 */
  uint8_t raw_status_type; /**< status_source对应的原PVT.fixType或GGA质量码，不直接作为统一fix码。 */
  uint8_t dimension_source; /**< 定位维度来源；NONE表示未提供可解释维度。 */
  uint32_t dimension_update_ms; /**< 最近语义有效维度事实时刻，单位ms，0合法。 */
  uint8_t dop_present_mask; /**< bit0..6分别授权G/P/T/V/H/N/E报告值，时效另判；0/65535未知。 */
  uint8_t dop_source[GNSS_DOP_COUNT]; /**< 每项DOP来源GNSS_SOURCE_*。 */
  uint16_t dop_x100[GNSS_DOP_COUNT]; /**< 每项报告的DOP乘100，有效1..65534，不是米误差。 */
  uint32_t dop_update_ms[GNSS_DOP_COUNT]; /**< 每项完整DOP字段消费时刻，单位ms，不被ACK续期。 */
  uint32_t dop_itow_ms[GNSS_DOP_COUNT]; /**< 该DOP的UBX周内导航历元ms，仅known位有效时解释。 */
  uint8_t dop_itow_known_mask; /**< bit0..6标记各DOP的iTOW已提供；NMEA来源对应位为0。 */
  uint8_t counts_present_mask; /**< bit0..3授权四项已报告计数，0颗合法；时效由对应时间判断。 */
  uint8_t counts_source[GNSS_COUNTS_COUNT]; /**< 四项计数来源，未提供为NONE，不推断未知星座。 */
  uint32_t counts_update_ms[GNSS_COUNTS_COUNT]; /**< 四项计数各自的完整报告时刻，单位ms。 */
  uint8_t dop_solution_mask; /**< bit0..6表示DOP来源自己的导航解/同iTOW PVT成立；present/fresh仍须另判。 */
} Gnss_Snapshot_t;

/**
 * @brief GNSS 轻量状态。
 *
 * @details
 * 状态查询不携带位置负载，不访问慢速总线，只复制驱动内部缓存。
 */
typedef struct {
  Gnss_DataState_t data_state; /**< 数据事实状态。 */
  uint8_t fix_valid;           /**< 定位有效标志。 */
  uint8_t satellites;          /**< 当前使用卫星数。 */
  uint32_t rx_sequence;        /**< 接收解析序号。 */
  uint32_t last_rx_ms;         /**< 最近收到完整且校验通过的已识别NMEA/UBX报文时间，单位ms。 */
  uint32_t last_fix_ms;        /**< 最近有效定位时间，单位：ms。 */
} Gnss_Status_t;

/** @brief 实际识别出的线上协议；商品名称不作为芯片或协议识别依据。 */
typedef enum {
  GNSS_PROTOCOL_UNKNOWN = 0, /**< 尚未收到校验正确的已识别报文。 */
  GNSS_PROTOCOL_NMEA = 1,    /**< 已收到标准 NMEA 句子。 */
  GNSS_PROTOCOL_UBX = 2      /**< 已收到 UBX 帧。 */
} Gnss_Protocol_t;

/** @brief sensor 唯一所有者推进的接入阶段，不代替 Framework 健康状态。 */
typedef enum {
  GNSS_LINK_SCAN = 0,       /**< 在固定115200bit/s下探测有效报文，保留原阶段编号。 */
  GNSS_LINK_VERSION = 1,    /**< 查询兼容接收机的协议版本。 */
  GNSS_LINK_CONFIGURE = 2,  /**< 等待配置应答或导航输出。 */
  GNSS_LINK_RECEIVING = 3,  /**< 已有有效数据；是否配置成功另看 confirmed。 */
  GNSS_LINK_UNCONFIRMED = 4, /**< 继续接收，但主动配置没有得到完整确认。 */
  GNSS_LINK_IO_ERROR = 5    /**< 本次原始串口操作失败，等待 owner 限速恢复。 */
} Gnss_LinkState_t;

/**
 * @brief 上电累计的驱动接收与配置事实；解析器软复位不得擦除诊断或重启当前探测/配置进度。
 * @details 原始字节来自实际 BSP 消费，包含非 NMEA 数据；有效帧不等同于有效定位。
 */
typedef struct {
  uint32_t measured_ms;       /**< 最近一次 sensor service 时刻，单位 ms。 */
  uint32_t service_count;     /**< 累计 service 次数，uint32 自然回绕。 */
  uint32_t rx_bytes;          /**< 已消费原始字节数，单位 byte。 */
  uint32_t valid_nmea;        /**< 校验正确的完整 NMEA 句子数。 */
  uint32_t valid_ubx;         /**< 校验正确且有界保存的 UBX 帧数。 */
  uint32_t checksum_errors;   /**< NMEA/UBX 校验失败次数。 */
  uint32_t framing_resets;    /**< 超长、传输不连续或重新同步事件数。 */
  uint32_t tx_commands;       /**< BSP 已接受的真实协议命令数，不代表应答成功。 */
  uint32_t ack_count;         /**< 匹配当前待确认 class/id 的 ACK 次数。 */
  uint32_t nak_count;         /**< 匹配当前待确认 class/id 的 NAK 次数。 */
  uint32_t timeout_count;     /**< 版本、配置应答或导航确认超时次数。 */
  uint32_t baud_changes;      /**< BSP接口成功完成的115200速率纠正次数；正常固定运行应为0。 */
  uint32_t last_rx_ms;        /**< 最近消费原始字节时刻，单位 ms。 */
  uint32_t last_frame_ms;     /**< 最近有效协议帧时刻，单位 ms。 */
  uint32_t baud_bps;          /**< 已确认的本机 UART 波特率，单位 bit/s。 */
  uint16_t protocol_version_x100; /**< MON-VER 的 PROTVER*100；0 表示尚未识别。 */
  uint8_t rx_seen;            /**< 1 表示本次上电曾消费原始字节。 */
  uint8_t frame_seen;         /**< 1 表示曾收到有效协议帧，允许时刻为 0 ms。 */
  uint8_t protocol;          /**< Gnss_Protocol_t 的数值。 */
  uint8_t state;             /**< Gnss_LinkState_t 的数值。 */
  uint8_t config_step;       /**< 已确认配置步骤数，范围 0..config_total。 */
  uint8_t config_total;      /**< 当前版本所需配置步骤数。 */
  uint8_t config_confirmed;  /**< 全部所需 ACK 且后续确有 NAV-PVT 时才为 1。 */
  uint8_t version_seen;      /**< 收到结构合法的 MON-VER 为 1；不证明商品芯片真伪。 */
} Gnss_Diagnostics_t;

/** @brief 只复制驱动事实；首次 Service 前返回 NO_DATA，之后即使没有定位也返回 OK。 */
Gnss_Result_t Sensor_GNSS_CopyDiagnostics(Gnss_Diagnostics_t *out);

#define GNSS_RAW_MAX_BYTES 392U /**< 旧Debug采样槽容量；独立完整原帧转发容量由下一个宏定义。 */
#define GNSS_RAW_FORWARD_MAX_BYTES 1024U /**< 独立完整UBX事件转发容量，不扩大旧3槽392B镜像。 */
#define GNSS_RAW_SLOT_COUNT 3U /**< 静态采样槽数量：0 NAV-PVT，1 NAV-DOP，2其它UBX/NMEA。 */

/** @brief 最近完整接收原帧的采样元数据；不是全量接收队列，也不授权测量有效性。 */
typedef struct {
  uint32_t sequence;       /**< 每槽独立采样序号，uint32自然回绕，0合法。 */
  uint32_t received_ms;    /**< sensor消费完整帧时的真实ms，0合法；非硬件逐字节到达时刻。 */
  uint16_t length;         /**< 完整原帧字节数，范围1..GNSS_RAW_MAX_BYTES。 */
  uint8_t protocol;        /**< GNSS_PROTOCOL_NMEA或GNSS_PROTOCOL_UBX。 */
  uint8_t class_id;        /**< UBX原始class；NMEA固定为0。 */
  uint8_t message_id;      /**< UBX原始id；NMEA固定为0。 */
  uint8_t checksum_valid;  /**< 原始校验通过为1，失败为0；坏帧只供观察，不进入测量。 */
} Gnss_RawFrameInfo_t;

/**
 * @brief 短临界复制一个原帧采样槽，不读硬件、不消费UART/DMA或推进队列。
 * @details 首个槽序号为0，以独立ready标记区分空槽；warm Init保留已捕获槽和序号。
 *          OK仅表示已复制原帧，校验结果另见checksum_valid。容量不足/参数非法返回INVALID_PARAM，
 *          空槽返回NO_DATA；失败不写输出。bytes和info必须指向各自独立的调用者缓冲。
 */
Gnss_Result_t Sensor_GNSS_CopyRawFrame(uint8_t slot, uint8_t *bytes, uint16_t capacity, Gnss_RawFrameInfo_t *info);

/**
 * @brief 复位 GNSS 解析器和快照状态。
 *
 * @return 初始化结果。
 */
Gnss_Result_t Sensor_GNSS_Init(void);

/**
 * @brief 处理待解析 GNSS 输入并更新内部快照。
 * @details UBX半帧绝对寿命为500ms，到期只撤销解析片段。可独立中止的TX故障
 *          不重建RX；命令未确认和失败计数仍保留，不表示定位成功。
 *          原始字节连续静默6秒才请求下一owner周期硬件重建；坏CRC持续输入不触发。
 *          warm解析器Init不推迟静默观察，实际重建失败沿用2秒退避。
 *
 * @param[in] now_ms 当前 sensor 周期时间，单位：ms。
 *
 * @return 服务结果；即使返回IO_ERROR，仍可能已有本轮新接收快照可独立复制。
 *
 * @note 本函数由 sensor 任务调用，不在 ISR 中调用。
 */
Gnss_Result_t Sensor_GNSS_Service(uint32_t now_ms);

/**
 * @brief 复制最近一次由 `Sensor_GNSS_Service()` 生成的完整 GNSS 快照。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 */
Gnss_Result_t Sensor_GNSS_CopySnapshot(Gnss_Snapshot_t *out);

/**
 * @brief 复制不含位置负载的 GNSS 轻量状态。
 *
 * @param[out] out 输出缓冲区，不能为 NULL。
 *
 * @return 复制结果。
 *
 * @note 本函数不访问 UART/DMA，只复制缓存。
 */
Gnss_Result_t Sensor_GNSS_GetStatus(Gnss_Status_t *out);

/**
 * @brief 请求 GNSS 解析器和快照在下一次 Service 中重新初始化。
 *
 * @note 本函数只置位请求标志，适合 recovery 回调调用。
 */
void Sensor_GNSS_RequestReinit(void);

/** @brief 原始句只读回调；NULL/0报告超容量，type为0其他/1GGA/2RMC/3GSA/4GSV，不能阻塞。 */
typedef void (*Gnss_DiagnosticSentence_t)(const uint8_t *bytes, uint16_t length, uint8_t type, uint8_t checksum_valid, uint32_t now_ms);
/** @brief 调度前由Adapter登记；Driver不依赖Framework。 */
void Gnss_SetDiagnosticCallback(Gnss_DiagnosticSentence_t callback);

/**
 * @brief 最大1024B完整UBX线帧只读观察回调，含收到的CK_A/CK_B，坏校验也可报告。
 * @details 仅sensor owner调用，位于临界区外；bytes只在回调期间有效，消费者须有界复制，
 *          不得阻塞、打印、修改字节或另行消费UART/DMA；不授权测量有效性。
 */
typedef void (*Gnss_UbxDiagnostic_t)(const uint8_t *bytes, uint16_t length, uint8_t cls, uint8_t id,
                                   uint8_t checksum_valid, uint32_t now_ms);
/** @brief 调度前由Adapter登记UBX观察入口；NULL注销，不依赖Debug开关，warm Init保留注册。 */
void Gnss_SetUbxDiagnosticCallback(Gnss_UbxDiagnostic_t callback);

/** @brief 原始组帧来源拒绝原因，属于软件事件，不表示已知物理丢帧数量。 */
#define GNSS_RAW_REJECT_OVERSIZE 1U
#define GNSS_RAW_REJECT_FRAMING 2U
#define GNSS_RAW_REJECT_METADATA 3U
/**
 * @brief 无法交付完整原帧的独立通知，仅sensor上下文、临界区外调用。
 * @param protocol 1=NMEA、2=UBX。
 * @param length 已知候选字节数；完整超长UBX为声明长度加8，NMEA溢出为已观察下界，不等物理丢字节。
 * @param reason GNSS_RAW_REJECT_*，不与完整坏CRC帧重复计数。
 * @param now_ms owner报告该事件时的真实ms；无时钟的Init事件延至下一Service报告。
 */
typedef void (*Gnss_RawReject_t)(uint8_t protocol, uint32_t length, uint8_t reason, uint32_t now_ms);
/** @brief 调度前登记原始来源拒绝入口；NULL关闭通知，warm Init保留注册。 */
void Gnss_SetRawRejectCallback(Gnss_RawReject_t callback);
#endif
