/**
 * @file px4lite_gnss_diagnostics.h
 * @brief Framework 层 GNSS 接收、协议和配置的只读诊断契约。
 * @details 独立于定位测量 topic；由 Adapter 逐字段转换 Driver/BSP 事实。
 *          计数只说明本机收发及解析，不证明定位、模块配置已确认或对端收到。
 *          本头不依赖 Sensor、BSP 或 HAL，也不改变既有遥测结构。
 */
#ifndef PX4LITE_GNSS_DIAGNOSTICS_H
#define PX4LITE_GNSS_DIAGNOSTICS_H

#include <stdint.h>

/** @brief 已通过完整帧校验识别的接收协议。 */
typedef enum {
  PX4LITE_GNSS_PROTOCOL_UNKNOWN = 0, /**< 尚未识别有效协议。 */
  PX4LITE_GNSS_PROTOCOL_NMEA = 1,    /**< NMEA 文本协议。 */
  PX4LITE_GNSS_PROTOCOL_UBX = 2      /**< UBX 二进制协议。 */
} Px4Lite_GnssProtocol_t;

/** @brief GNSS 所有者的协议/配置阶段；不作为位置有效标志。 */
typedef enum {
  PX4LITE_GNSS_LINK_DETECTING = 0,   /**< 在固定115200波特率下被动接收或主动识别协议。 */
  PX4LITE_GNSS_LINK_VERSION = 1,     /**< 查询模块协议版本。 */
  PX4LITE_GNSS_LINK_CONFIGURING = 2, /**< 正在发送配置并等待结果。 */
  PX4LITE_GNSS_LINK_RECEIVING = 3,   /**< 正常收数；可能是未主动配置的被动模式。 */
  PX4LITE_GNSS_LINK_UNCONFIRMED = 4, /**< 配置未确认，但仍可继续接收导航数据。 */
  PX4LITE_GNSS_LINK_IO_ERROR = 5,    /**< 当前硬件收发失败。 */
  PX4LITE_GNSS_LINK_UNKNOWN = 255    /**< 输入阶段未知，不冒充探测或配置成功。 */
} Px4Lite_GnssLinkState_t;

/**
 * @brief GNSS 诊断副本；软件时效与定位、传输快照分别判定。
 * @details measured_ms 仅来自 sensor 的 Service，仅读 BSP 不能续期。
 *          各 uint32 累计计数允许自然回绕，不能以计数非零代替 seen 标志。
 */
typedef struct {
  uint32_t measured_ms;          /**< sensor 最近一次服务时刻，ms；0 是合法时刻。 */
  uint32_t service_count;        /**< sensor 服务累计次数。 */
  uint32_t rx_bytes;             /**< Driver 已消费的原始字节数，B，含非 NMEA/UBX 字节。 */
  uint32_t valid_nmea;           /**< 校验通过的完整 NMEA 句累计数。 */
  uint32_t valid_ubx;            /**< 校验通过的完整 UBX 帧累计数。 */
  uint32_t checksum_errors;      /**< 完整候选帧校验失败累计数。 */
  uint32_t framing_resets;       /**< 拆帧失步或超容量后重同步累计数。 */
  uint32_t tx_commands;          /**< Driver 实际提交成功的配置/查询命令累计数。 */
  uint32_t ack_count;            /**< 收到配置 ACK 的累计数。 */
  uint32_t nak_count;            /**< 收到配置 NAK 的累计数。 */
  uint32_t timeout_count;        /**< 配置/查询等待超时累计数。 */
  uint32_t baud_changes;         /**< 本机速率纠正回115200的累计数，正常运行应为0。 */
  uint32_t last_rx_ms;           /**< Driver 最近消费原始字节的时刻，ms，须检查 rx_seen。 */
  uint32_t last_frame_ms;        /**< 最近校验通过完整协议帧的时刻，ms，须检查 frame_seen。 */
  uint32_t baud_bps;             /**< Driver 当前处理速率，bit/s；0 表示未知。 */
  uint32_t transport_baud_bps;   /**< BSP 当前实际配置速率，bit/s；须检查 transport_available。 */
  uint32_t transport_rx_bytes;   /**< BSP 已观察的 DMA 接收字节累计数，B；非独立物理计数器。 */
  uint32_t rx_drop_bytes;        /**< BSP 覆盖及接收重建时舍弃的累计字节数，B。 */
  uint32_t uart_errors;          /**< UART 接收错误累计次数。 */
  uint32_t dma_errors;           /**< DMA 错误累计次数。 */
  uint32_t tx_packets;           /**< BSP 完整发送成功累计次数，不代表模块接受配置。 */
  uint32_t tx_failures;          /**< BSP 发送失败累计次数，失败时可能已有部分字节出线。 */
  uint32_t recoveries;           /**< BSP 成功重建接收链路的累计次数，普通切速率不计。 */
  uint16_t ndtr;                /**< DMA 剩余搬运量，B，只读采样。 */
  uint16_t queued_bytes;        /**< BSP 未消费队列长度，B。 */
  uint16_t protocol_version_x100; /**< 已识别的协议版本乘100，0表示PROTVER尚未知。 */
  uint8_t rx_seen;              /**< 本次上电已消费原始字节，0/1。 */
  uint8_t frame_seen;           /**< 本次上电已收到有效协议帧，0/1。 */
  uint8_t protocol;             /**< Px4Lite_GnssProtocol_t，0..2。 */
  uint8_t state;                /**< Px4Lite_GnssLinkState_t，0..5 或 255。 */
  uint8_t config_step;          /**< 当前配置步骤编号；保留 Driver 原始进度。 */
  uint8_t config_total;         /**< 当前所需配置步骤总数；0 表示未发起。 */
  uint8_t config_confirmed;     /**< 全部所需配置已获得确认才为 1，不以导航帧替代 ACK。 */
  uint8_t version_seen;         /**< 已收到合法MON-VER版本答复，0/1；PROTVER仍可未知。 */
  uint8_t initialized;          /**< BSP UART/DMA 已完成初始化，0/1。 */
  uint8_t tx_busy;              /**< BSP 原始发送正在进行，0/1。 */
  uint8_t available;            /**< Driver 已有服务副本，0/1；App 对过期副本清零。 */
  uint8_t transport_available;  /**< 本次成功复制 BSP 传输事实，0/1；不代表软件时效。 */
} Px4Lite_GnssDiagnostics_t;

/** @brief 单个原始完整线帧的最大字节数，含真实协议头和校验。 */
#define PX4LITE_GNSS_RAW_MAX_BYTES 392U
/** @brief 最新原帧槽数：0为NAV-PVT，1为NAV-DOP，2为其他UBX或NMEA。 */
#define PX4LITE_GNSS_RAW_SLOT_COUNT 3U

#define PX4LITE_GNSS_SAT_PAGE_CAPACITY 5U /**< 本地逐星视图每页固定5颗，避免跨层复制整表。 */
#define PX4LITE_GNSS_SAT_VALID_ELEVATION 1U /**< 仰角为源报告的-90..90度，0度有效。 */
#define PX4LITE_GNSS_SAT_VALID_AZIMUTH 2U /**< 方位为0..360度，且源仰角可解释。 */

/** @brief UBX NAV-SAT的一颗卫星事实，不授权位置或UTC有效。 */
typedef struct {
  uint8_t gnss_id;       /**< UBX星座标识；未知编号仍保留，不按卫星号猜星座。 */
  uint8_t sv_id;         /**< 原报告卫星编号，与gnss_id共同标识卫星。 */
  uint8_t cno_dbhz;      /**< 原报告载噪比，单位dB-Hz；0为报告值，须结合quality解释。 */
  uint8_t quality;       /**< 原flags.qualityInd，0..7；不是定位质量码。 */
  uint8_t used;          /**< 原flags.svUsed，本消息报告用于导航解算为1。 */
  uint8_t valid_mask;    /**< 角度可解释位，使用PX4LITE_GNSS_SAT_VALID_*。 */
  int8_t elevation_deg;  /**< 仰角，单位度；仅仰角位有效时解释。 */
  uint8_t reserved;      /**< 保留0。 */
  int16_t azimuth_deg;   /**< 方位角，单位度；仅方位位有效时解释。 */
  uint16_t reserved2;    /**< 保留0。 */
} Px4Lite_GnssSatellite_t;

/** @brief 同一NAV-SAT代数的一页只读事实；不替换原帧上传或既有导航topic。 */
typedef struct {
  uint32_t sequence;     /**< 完整逐星消息序号，自然回绕，0也合法。 */
  uint32_t received_ms;  /**< sensor消费完整消息的真实开机毫秒，0也合法。 */
  uint32_t itow_ms;      /**< GPS周内导航历元毫秒；不是UTC或网络接收时间。 */
  uint8_t total;         /**< 源记录数，不等于都已跟踪或用于定位。 */
  uint8_t stored;        /**< 保留表项数，范围0..64。 */
  uint8_t page;          /**< 当前页号，从0开始。 */
  uint8_t count;         /**< 本页有效表项数，范围0..5，尾部零填充不能解释。 */
  uint8_t truncated;     /**< 超过64项时为1，不把保留数称为全部记录数。 */
  uint8_t available;     /**< 已有副本为1；App时效拒绝时清零。 */
  uint8_t used_count;    /**< 本消息全部记录的svUsed计数，不冒充PVT总使用数。 */
  uint8_t reserved;      /**< 保留0。 */
  Px4Lite_GnssSatellite_t entries[PX4LITE_GNSS_SAT_PAGE_CAPACITY]; /**< 本页最多5项。 */
} Px4Lite_GnssSatellitePage_t;

/**
 * @brief 独立于测量的完整线帧镜像；每类仅保留最新样本，不是全部报文日志。
 * @details 由Adapter逐字段转换，不依赖Driver/BSP/HAL；bad CRC仍保留真实接收字节。
 */
typedef struct {
  uint32_t sequence;            /**< 原帧发布序号，uint32自然回绕，0也可有效。 */
  uint32_t received_ms;         /**< 完整线帧接收时刻，真实ms，0合法。 */
  uint16_t length;              /**< bytes中完整线帧长度，1..392字节。 */
  uint8_t protocol;             /**< Px4Lite_GnssProtocol_t，NMEA=1、UBX=2。 */
  uint8_t class_id;             /**< UBX类别；NMEA为0。 */
  uint8_t message_id;           /**< UBX消息编号；NMEA为0。 */
  uint8_t checksum_valid;       /**< 实际线帧校验通过为1，失败为0，不修正原校验。 */
  uint8_t bytes[PX4LITE_GNSS_RAW_MAX_BYTES]; /**< 完整wire镜像；只复制，不解析或重新构帧。 */
} Px4Lite_GnssRawFrame_t;

#endif
