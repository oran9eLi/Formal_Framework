/**
 * @file debug_service.c
 * @brief 在低优先级DebugTask运行独立诊断与原帧文本输出。
 * @details 通过Framework只读接口复制，不在采集任务或ISR打印，不解析Driver/HAL数据。
 */

#include "debug_service.h"

#include "debug_config.h"
#include "debug_console.h"
#include "debug_selftest.h"
#include "debug_task_monitor.h"
#include "app_data_api.h"
#include "px4lite_config.h"
#include "px4lite_modules.h"
#include "px4lite_platform.h"
#include "px4lite_time.h"
#include "px4lite_topics.h"
#include "task.h"
#include <string.h>

#if DEBUG_PERIODIC_SERVICE_ENABLE

static void DebugService_Task(void *argument);

#if DEBUG_GNSS_MONITOR_ENABLE || DEBUG_IMU_MONITOR_ENABLE || DEBUG_BARO_MONITOR_ENABLE
/**
 * @brief DebugTask独占的临时快照区；各报告器顺序使用，禁止在调用间保留成员指针。
 * @details 大快照共用静态空间，避免384word任务栈叠加GNSS/IMU诊断结构。
 *          只保存已发布副本，不发布测量、不访问硬件、不用于DMA。
 */
typedef union {
#if DEBUG_GNSS_MONITOR_ENABLE
  Px4Lite_SensorGnss_t gnss; /**< 当前GNSS测量副本，只在GNSS报告期间解释。 */
#endif
#if DEBUG_IMU_MONITOR_ENABLE
  struct {
    App_NavigationSnapshot_t nav;       /**< 本机导航/姿态副本，单位保持App契约。 */
    Px4Lite_ImuDiagnostics_t driver;     /**< IMU型号、接口、芯片ID及恢复事实副本。 */
  } imu;                               /**< IMU报告需要同时保留的两个副本。 */
#endif
#if DEBUG_BARO_MONITOR_ENABLE
  Px4Lite_SensorBaro_t baro;            /**< 气压/温度测量副本，时效由原sample_time判定。 */
#endif
} Debug_SensorScratch_t;
static Debug_SensorScratch_t s_sensor_scratch;
#endif

#if DEBUG_GNSS_MONITOR_ENABLE
/** @brief DebugTask独占静态原帧副本；禁止在384word任务栈上放392B载荷。 */
static Px4Lite_GnssRawFrame_t s_gnss_raw_frame;
static uint32_t s_gnss_raw_last_sequence[PX4LITE_GNSS_RAW_SLOT_COUNT]; /**< 每槽最近输出序号。 */
static uint8_t s_gnss_raw_seen[PX4LITE_GNSS_RAW_SLOT_COUNT]; /**< 每槽是否输出过，0序号合法。 */
static uint8_t s_gnss_raw_next_slot; /**< 下次轮转起点，0..2。 */
/** @brief 静态框架副本避免给384word调试栈增加大结构；只由DebugTask访问。 */
static Px4Lite_GnssDiagnostics_t s_gnss_link_diag;
/** @brief 只保存会触发异常日志的前次事实，不因字节/有效帧正常增长反复打印。 */
typedef struct {
  uint32_t uart_errors;     /**< UART错误累计次数。 */
  uint32_t dma_errors;      /**< DMA错误累计次数。 */
  uint32_t drops;           /**< 接收丢弃累计字节数，B。 */
  uint32_t tx_failures;     /**< TX失败累计次数。 */
  uint32_t recoveries;      /**< 接收成功重建累计次数。 */
  uint32_t checksum_errors; /**< 帧校验失败累计次数。 */
  uint32_t framing_resets;  /**< 解析重同步累计次数。 */
  Px4Lite_Result_t result;  /**< 框架复制返回码，原值保留。 */
  uint8_t seen;             /**< 至少有一次Driver或BSP副本为1。 */
  uint8_t state;            /**< 前次协议/配置阶段，0..5或255。 */
  uint8_t protocol;         /**< 前次协议编号，0..2。 */
  uint8_t initialized;      /**< 前次BSP已初始化事实，0/1。 */
  uint8_t available;        /**< 前次Driver副本可用事实，0/1。 */
  uint8_t transport_available; /**< 前次BSP副本可用事实，0/1。 */
} Debug_GnssLinkGate_t;
static Debug_GnssLinkGate_t s_gnss_link_gate;

/**
 * @brief 在既有1秒GNSS门限内仅报告首次事实、状态变化或异常，仅读Framework副本。
 * @details 普通字节/帧递增不触发；无定位仍可报告接收进展。只在DebugTask格式化，
 *          不访问BSP/HAL、不触发采集或恢复，关闭monitor不会改变生产数据路径。
 *          byte/frame以seen/age成对表示，seen=0时age=0不能解释成刚收到数据。
 *          age均为真实ms；rx为BSP/Driver累计字节，副本非同一原子时刻，差额不等同丢包。
 */
static void DebugService_ReportGnssLink(void)
{
  Px4Lite_Result_t result=Px4Lite_GnssCopyDiagnostics(&s_gnss_link_diag);
  uint32_t now_ms=Px4Lite_PlatformGetMs();
  uint32_t service_age=s_gnss_link_diag.available?Px4Lite_ElapsedMs(now_ms,s_gnss_link_diag.measured_ms):0U;
  uint32_t byte_age=s_gnss_link_diag.rx_seen?Px4Lite_ElapsedMs(now_ms,s_gnss_link_diag.last_rx_ms):0U;
  uint32_t frame_age=s_gnss_link_diag.frame_seen?Px4Lite_ElapsedMs(now_ms,s_gnss_link_diag.last_frame_ms):0U;
  uint8_t report;
  if(!s_gnss_link_gate.seen && !s_gnss_link_diag.available && !s_gnss_link_diag.transport_available) { return; }
  report=(uint8_t)(!s_gnss_link_gate.seen || result!=s_gnss_link_gate.result ||
      s_gnss_link_diag.state!=s_gnss_link_gate.state || s_gnss_link_diag.protocol!=s_gnss_link_gate.protocol ||
      s_gnss_link_diag.initialized!=s_gnss_link_gate.initialized ||
      s_gnss_link_diag.available!=s_gnss_link_gate.available ||
      s_gnss_link_diag.transport_available!=s_gnss_link_gate.transport_available ||
      s_gnss_link_diag.uart_errors!=s_gnss_link_gate.uart_errors ||
      s_gnss_link_diag.dma_errors!=s_gnss_link_gate.dma_errors ||
      s_gnss_link_diag.rx_drop_bytes!=s_gnss_link_gate.drops ||
      s_gnss_link_diag.tx_failures!=s_gnss_link_gate.tx_failures ||
      s_gnss_link_diag.recoveries!=s_gnss_link_gate.recoveries ||
      s_gnss_link_diag.checksum_errors!=s_gnss_link_gate.checksum_errors ||
      s_gnss_link_diag.framing_resets!=s_gnss_link_gate.framing_resets ||
      (s_gnss_link_diag.available && service_age>1500U) ||
      (s_gnss_link_diag.frame_seen && frame_age>2000U));
  s_gnss_link_gate.seen=1U;s_gnss_link_gate.result=result;
  s_gnss_link_gate.state=s_gnss_link_diag.state;s_gnss_link_gate.protocol=s_gnss_link_diag.protocol;
  s_gnss_link_gate.initialized=s_gnss_link_diag.initialized;
  s_gnss_link_gate.available=s_gnss_link_diag.available;
  s_gnss_link_gate.transport_available=s_gnss_link_diag.transport_available;
  s_gnss_link_gate.uart_errors=s_gnss_link_diag.uart_errors;s_gnss_link_gate.dma_errors=s_gnss_link_diag.dma_errors;
  s_gnss_link_gate.drops=s_gnss_link_diag.rx_drop_bytes;s_gnss_link_gate.tx_failures=s_gnss_link_diag.tx_failures;
  s_gnss_link_gate.recoveries=s_gnss_link_diag.recoveries;
  s_gnss_link_gate.checksum_errors=s_gnss_link_diag.checksum_errors;
  s_gnss_link_gate.framing_resets=s_gnss_link_diag.framing_resets;
  if(!report) { return; }
  DBG_PRINT("GNSS-LINK: rc=%d av=%u/%u st=%u p=%u svc=%lu age=%lu rx=%lu/%lu nmea=%lu ubx=%lu byte=%u/%lu frame=%u/%lu bad=%lu rst=%lu",
            (int)result,(unsigned int)s_gnss_link_diag.available,(unsigned int)s_gnss_link_diag.transport_available,
            (unsigned int)s_gnss_link_diag.state,(unsigned int)s_gnss_link_diag.protocol,
            (unsigned long)s_gnss_link_diag.service_count,(unsigned long)service_age,
            (unsigned long)s_gnss_link_diag.transport_rx_bytes,(unsigned long)s_gnss_link_diag.rx_bytes,
            (unsigned long)s_gnss_link_diag.valid_nmea,(unsigned long)s_gnss_link_diag.valid_ubx,
            (unsigned int)s_gnss_link_diag.rx_seen,(unsigned long)byte_age,
            (unsigned int)s_gnss_link_diag.frame_seen,(unsigned long)frame_age,
            (unsigned long)s_gnss_link_diag.checksum_errors,(unsigned long)s_gnss_link_diag.framing_resets);
  DBG_PRINT("GNSS-IO: init=%u tx=%u baud=%lu ndtr=%u q=%u uart=%lu dma=%lu drop=%lu txfail=%lu rec=%lu",
            (unsigned int)s_gnss_link_diag.initialized,(unsigned int)s_gnss_link_diag.tx_busy,
            (unsigned long)s_gnss_link_diag.transport_baud_bps,(unsigned int)s_gnss_link_diag.ndtr,
            (unsigned int)s_gnss_link_diag.queued_bytes,(unsigned long)s_gnss_link_diag.uart_errors,
            (unsigned long)s_gnss_link_diag.dma_errors,(unsigned long)s_gnss_link_diag.rx_drop_bytes,
            (unsigned long)s_gnss_link_diag.tx_failures,(unsigned long)s_gnss_link_diag.recoveries);
}

/** @brief 给已复制线帧选用名称，不解码物理量或修改原字节。 */
static const char *DebugService_GnssRawName(const Px4Lite_GnssRawFrame_t *frame)
{
  if (frame->protocol == PX4LITE_GNSS_PROTOCOL_UBX && frame->class_id == 0x01U) {
    if (frame->message_id == 0x07U) { return "NAV-PVT"; }
    if (frame->message_id == 0x04U) { return "NAV-DOP"; }
  }
  return "OTHER";
}

/**
 * @brief 每次报告最多选一个新原帧，按32字节HEX行完整输出，仅DebugTask调用。
 * @details 外层既有1000ms门限限速；三槽每类最新样本轮转，不声称覆盖所有接收帧。
 *          每行独立DBG_PRINT短锁，不持锁跨行；关闭monitor不影响任何采集路径。
 */
static void DebugService_ReportGnssRaw(void)
{
  static const char digits[] = "0123456789ABCDEF";
  uint8_t attempts;
  for (attempts = 0U; attempts < PX4LITE_GNSS_RAW_SLOT_COUNT; attempts++) {
    uint8_t slot = s_gnss_raw_next_slot;
    uint16_t offset;
    s_gnss_raw_next_slot = (uint8_t)((slot + 1U) % PX4LITE_GNSS_RAW_SLOT_COUNT);
    if (Px4Lite_GnssCopyRawFrame(slot, &s_gnss_raw_frame) != PX4LITE_OK) { continue; }
    if (!s_gnss_raw_frame.length || s_gnss_raw_frame.length > PX4LITE_GNSS_RAW_MAX_BYTES) { continue; }
    if (s_gnss_raw_seen[slot] && s_gnss_raw_last_sequence[slot] == s_gnss_raw_frame.sequence) { continue; }
    s_gnss_raw_seen[slot] = 1U;
    s_gnss_raw_last_sequence[slot] = s_gnss_raw_frame.sequence;
    DBG_PRINT("GNSS-RAW sampled %s seq=%lu class=%02X id=%02X len=%u CRC=%s ms=%lu",
              DebugService_GnssRawName(&s_gnss_raw_frame), (unsigned long)s_gnss_raw_frame.sequence,
              (unsigned int)s_gnss_raw_frame.class_id, (unsigned int)s_gnss_raw_frame.message_id,
              (unsigned int)s_gnss_raw_frame.length, s_gnss_raw_frame.checksum_valid ? "OK" : "BAD",
              (unsigned long)s_gnss_raw_frame.received_ms);
    for (offset = 0U; offset < s_gnss_raw_frame.length; offset = (uint16_t)(offset + 32U)) {
      char hexLine[97];
      uint16_t count = (uint16_t)(s_gnss_raw_frame.length - offset);
      uint16_t i, position = 0U;
      if (count > 32U) { count = 32U; }
      for (i = 0U; i < count; i++) {
        uint8_t byte = s_gnss_raw_frame.bytes[offset + i];
        hexLine[position++] = digits[byte >> 4U];
        hexLine[position++] = digits[byte & 0x0FU];
        if (i + 1U < count) { hexLine[position++] = ' '; }
      }
      hexLine[position] = '\0';
      DBG_PRINT("RAW seq=%lu +%u %s", (unsigned long)s_gnss_raw_frame.sequence, (unsigned int)offset, hexLine);
    }
    return;
  }
}

/**
 * @brief Print one coherent GNSS measurement and module-state report.
 */
static void DebugService_ReportGnss(uint32_t now_ms)
{
  Px4Lite_SensorGnss_t *gnss=&s_sensor_scratch.gnss;
  Px4Lite_ModuleStatus_t status;
  uint32_t report_ms;

  (void)now_ms;
  DebugService_ReportGnssLink();
  memset(&s_sensor_scratch,0,sizeof(s_sensor_scratch));

  if ((Px4Lite_CopyGnss(gnss) == PX4LITE_OK) && (Px4Lite_GetModuleStatus(PX4LITE_MODULE_GNSS, &status) == PX4LITE_OK)) {
    report_ms = Px4Lite_PlatformGetMs();
    DBG_PRINT("GNSS: state=%u fix=%u sats=%u gps=%u/%u bds=%u/%u "
              "lat=%ld lon=%ld utc_date=%lu utc_sec=%lu age=%lu",
              (unsigned int)status.state, (unsigned int)gnss->fix_type, (unsigned int)gnss->satellites_used, (unsigned int)gnss->gps_used, (unsigned int)gnss->gps_visible, (unsigned int)gnss->bds_used, (unsigned int)gnss->bds_visible, (long)gnss->latitude_e7, (long)gnss->longitude_e7, (unsigned long)gnss->utc_date, (unsigned long)gnss->utc_sec, (unsigned long)Px4Lite_ElapsedMs(report_ms, status.last_rx_ms));
  } else {
    DBG_PRINT("GNSS: waiting for valid GNSS data");
  }

  {
    /* 室内无 GPS 时用它判断 RTC 手表本身：src 0=NONE 1=RTC 2=GNSS；
       sync 0=INVALID 1=RTC_VALID 2=GNSS_SYNCED 3=STALE。rc!=OK 表示 RTC 从未被有效校准。 */
    Px4Lite_TimeSnapshot_t t;
    Px4Lite_Result_t trc = Px4Lite_CopyTime(&t);

    if (trc == PX4LITE_OK) {
      DBG_PRINT("TIME: rc=OK src=%u sync=%u local=%04lu-%02lu-%02lu %06lu sync_age=%lus",
                (unsigned int)t.source, (unsigned int)t.sync_state,
                (unsigned long)(t.local_date_ymd / 10000UL), (unsigned long)((t.local_date_ymd / 100UL) % 100UL), (unsigned long)(t.local_date_ymd % 100UL),
                (unsigned long)t.local_time_hhmmss, (unsigned long)t.sync_age_s);
    } else {
      DBG_PRINT("TIME: rc=%d no valid time yet (src=NONE, RTC never calibrated by GNSS)", (int)trc);
    }
  }
  /* 原帧独立于定位topic可用性：未定位或测量尚未发布也输出真实原始样本。 */
  DebugService_ReportGnssRaw();
}
#endif

#if DEBUG_IMU_MONITOR_ENABLE || DEBUG_BARO_MONITOR_ENABLE
/** @brief 明确区分未初始化、启动、降级、离线及失败；字符串只读，不推断硬件状态。 */
static const char *DebugService_ModuleState(Px4Lite_State_t state)
{
  switch(state) {
    case PX4LITE_STATE_UNINITIALIZED:return "UNINIT";
    case PX4LITE_STATE_DISABLED:return "DISABLED";
    case PX4LITE_STATE_STARTING:return "STARTING";
    case PX4LITE_STATE_ONLINE:return "ONLINE";
    case PX4LITE_STATE_DEGRADED:return "DEGRADED";
    case PX4LITE_STATE_OFFLINE:return "OFFLINE";
    case PX4LITE_STATE_FAILED:return "FAILED";
    default:return "UNKNOWN";
  }
}

/**
 * @brief 每秒复制三项真实模块状态，以实际文字/故障码区分采集离线与初始化失败。
 * @details rx为last_rx_ms原值，不将初始化成功或旧快照解释成ONLINE；三副本顺序读取。
 */
static void DebugService_ReportSensorStates(void)
{
  Px4Lite_ModuleStatus_t state[3];
  Px4Lite_Result_t result[3];
  memset(state,0,sizeof(state));
  result[0]=Px4Lite_GetModuleStatus(PX4LITE_MODULE_GNSS,&state[0]);
  result[1]=Px4Lite_GetModuleStatus(PX4LITE_MODULE_IMU,&state[1]);
  result[2]=Px4Lite_GetModuleStatus(PX4LITE_MODULE_BARO,&state[2]);
  DBG_PRINT("SENS: GNSS=%s f=%04X rx=%lu err=%lu IMU=%s f=%04X rx=%lu err=%lu Baro=%s f=%04X rx=%lu err=%lu",
    result[0]==PX4LITE_OK?DebugService_ModuleState(state[0].state):"N/A",(unsigned int)state[0].fault_code,
    (unsigned long)state[0].last_rx_ms,(unsigned long)state[0].error_count,
    result[1]==PX4LITE_OK?DebugService_ModuleState(state[1].state):"N/A",(unsigned int)state[1].fault_code,
    (unsigned long)state[1].last_rx_ms,(unsigned long)state[1].error_count,
    result[2]==PX4LITE_OK?DebugService_ModuleState(state[2].state):"N/A",(unsigned int)state[2].fault_code,
    (unsigned long)state[2].last_rx_ms,(unsigned long)state[2].error_count);
}
#endif

#if DEBUG_IMU_MONITOR_ENABLE
/**
 * @brief 读取统一IMU诊断缓存与业务姿态，避免Debug成为第二个硬件采集者。
 * @details 型号/总线/双ID/恢复阶段独立报告；无总线读写，无浮点格式化。
 */
static void DebugService_ReportImu(uint32_t now_ms)
{
  App_NavigationSnapshot_t *nav=&s_sensor_scratch.imu.nav;
  Px4Lite_ModuleStatus_t status;
  Px4Lite_ImuDiagnostics_t *driver_status=&s_sensor_scratch.imu.driver;
  Px4Lite_Result_t driver_rc;
  Px4Lite_Result_t status_rc;
  Px4Lite_Result_t nav_rc;
  uint32_t report_ms;
  uint32_t imu_age_ms;
  uint8_t imu_fresh;

  (void)now_ms;
  memset(&s_sensor_scratch,0,sizeof(s_sensor_scratch));
  memset(&status,0,sizeof(status));

  /* Debug 特例：Framework 状态用于诊断模块健康，App 快照用于查看业务可见姿态。 */
  status_rc = Px4Lite_GetModuleStatus(PX4LITE_MODULE_IMU, &status);
  driver_rc = Px4Lite_ImuCopyDiagnostics(driver_status);
  report_ms = Px4Lite_PlatformGetMs();
  nav_rc    = App_CopyNavigation(nav, report_ms);

  DBG_PRINT("IMU-DRV: rc=%d model=%u bus=%u id=0x%02X/0x%02X addr=0x%02X/0x%02X ready=%u stage=%u result=%u bsp=%u reinit=%lu fifo_drop=%lu",
            (int)driver_rc, (unsigned int)driver_status->driver, (unsigned int)driver_status->bus,
            (unsigned int)driver_status->accel_chip_id, (unsigned int)driver_status->gyro_chip_id,
            (unsigned int)driver_status->accel_addr_7bit, (unsigned int)driver_status->gyro_addr_7bit,
            (unsigned int)driver_status->device_ready, (unsigned int)driver_status->last_stage,
            (unsigned int)driver_status->last_result, (unsigned int)driver_status->last_bsp_status,
            (unsigned long)driver_status->reinit_count, (unsigned long)driver_status->fifo_drop_count);

  if (status_rc == PX4LITE_OK) {
    imu_age_ms = Px4Lite_ElapsedMs(report_ms, status.last_rx_ms);
    imu_fresh  = ((status.state == PX4LITE_STATE_ONLINE) && (imu_age_ms <= PX4LITE_IMU_MAX_AGE_MS)) ? 1U : 0U;
    if ((nav_rc == PX4LITE_OK) && ((nav->valid_mask & PX4LITE_NAV_VALID_ATTITUDE) != 0U)) {
      DBG_PRINT("IMU: state=%u err=%lu age=%lu fresh=%u "
                "roll_cdeg=%ld pitch_cdeg=%ld yaw_cdeg=%ld "
                "rate_cdps=%ld,%ld,%ld",
                (unsigned int)status.state, (unsigned long)status.error_count, (unsigned long)imu_age_ms, (unsigned int)imu_fresh,
                (long)nav->roll_deg100, (long)nav->pitch_deg100, (long)nav->yaw_deg100, (long)nav->roll_rate_dps100, (long)nav->pitch_rate_dps100, (long)nav->yaw_rate_dps100);
    } else {
      DBG_PRINT("IMU: state=%u fault=%u err=%lu age=%lu fresh=%u "
                "no valid attitude rc=%d mask=0x%08lX",
                (unsigned int)status.state, (unsigned int)status.fault_code, (unsigned long)status.error_count, (unsigned long)imu_age_ms, (unsigned int)imu_fresh,
                (int)nav_rc, (nav_rc == PX4LITE_OK) ? (unsigned long)nav->valid_mask : 0UL);
    }
  } else {
    DBG_PRINT("IMU: framework status unavailable rc=%d", (int)status_rc);
  }
}
#endif

#if DEBUG_BARO_MONITOR_ENABLE
/**
 * @brief 独立报告气压模块状态与缓存测量，即使尚无topic也保留初始化失败/离线事实。
 * @details ready/sample_valid表示存在header.valid副本，fresh按App环境年龄独立判定；不读I2C。
 */
static void DebugService_ReportBaro(uint32_t now_ms)
{
  Px4Lite_SensorBaro_t *baro=&s_sensor_scratch.baro;
  Px4Lite_ModuleStatus_t status;
  Px4Lite_Result_t sample_rc,status_rc;
  uint32_t report_ms;
  uint8_t ready;

  (void)now_ms;
  memset(&s_sensor_scratch,0,sizeof(s_sensor_scratch));memset(&status,0,sizeof(status));
  status_rc=Px4Lite_GetModuleStatus(PX4LITE_MODULE_BARO,&status);
  sample_rc=Px4Lite_CopyBaro(baro);report_ms=Px4Lite_PlatformGetMs();
  ready=(uint8_t)(sample_rc==PX4LITE_OK && baro->header.valid);
  if(!ready) {
    DBG_PRINT("BARO: state=%s fault=%04X err=%lu rec=%u rxage=%lu rc=%d/%d ready=0 sample_valid=0 fresh=0 sample=N/A age=N/A p_pa=N/A",
      status_rc==PX4LITE_OK?DebugService_ModuleState(status.state):"N/A",(unsigned int)status.fault_code,
      (unsigned long)status.error_count,(unsigned int)status.recovery_count,
      (unsigned long)(status_rc==PX4LITE_OK?Px4Lite_ElapsedMs(report_ms,status.last_rx_ms):0U),
      (int)status_rc,(int)sample_rc);
    return;
  }
  DBG_PRINT("BARO: state=%s fault=%04X err=%lu rec=%u rxage=%lu rc=%d/%d ready=1 sample_valid=1 fresh=%u sample=%lu age=%lu p_pa=%ld",
    status_rc==PX4LITE_OK?DebugService_ModuleState(status.state):"N/A",(unsigned int)status.fault_code,
    (unsigned long)status.error_count,(unsigned int)status.recovery_count,
    (unsigned long)(status_rc==PX4LITE_OK?Px4Lite_ElapsedMs(report_ms,status.last_rx_ms):0U),
    (int)status_rc,(int)sample_rc,(unsigned int)(Px4Lite_ElapsedMs(report_ms,baro->header.sample_time_ms)<=APP_ENVIRONMENT_MAX_AGE_MS),
    (unsigned long)baro->header.sample_time_ms,
    (unsigned long)Px4Lite_ElapsedMs(report_ms,baro->header.sample_time_ms),(long)baro->pressure_pa);
}
#endif

#if DEBUG_POWER_MONITOR_ENABLE
#if PX4LITE_ENABLE_BATTERY2
/** @brief 输出sensor已发布的同次诊断；不读取硬件，失败/回退不伪装为实测。 */
static void DebugService_ReportPowerAdc(uint32_t now_ms)
{
  Px4Lite_PowerDiagnostics_t diag;
  if (Px4Lite_Battery2CopyDiagnostics(&diag) != PX4LITE_OK) {
    DBG_PRINT("POWER-ADC: no sample");
    return;
  }
  DBG_PRINT("POWER-ADC: seq=%lu raw=%lu pin_mv=%lu bsp_mv=%lu final_mv=%lu current_pin_mv=%lu age_ms=%lu voltage_valid=%u",
            (unsigned long)diag.sequence, (unsigned long)diag.voltage_raw,
            (unsigned long)diag.voltage_pin_mv, (unsigned long)diag.voltage_bsp_mv,
            (unsigned long)diag.voltage_mv, (unsigned long)diag.current_pin_mv,
            (unsigned long)(now_ms - diag.sample_time_ms), (unsigned int)diag.voltage_valid);
  DBG_PRINT("POWER-REF: seq=%lu cal=0x%04lX ref_raw=%lu calc_vdda_mv=%lu used_vdda_mv=%lu status=%u current_valid=%u",
            (unsigned long)diag.sequence, (unsigned long)diag.vref_cal_raw,
            (unsigned long)diag.vref_raw, (unsigned long)diag.vdda_calculated_mv,
            (unsigned long)diag.vdda_mv, (unsigned int)diag.reference_status,
            (unsigned int)diag.current_valid);
}
#endif

/**
 * @brief 仅在低优先级 DebugTask 输出已启用输入组，不访问 ADC 或禁用组 topic。
 * @param[in] now_ms 当前毫秒时间，用于报告已有样本的年龄。
 */
static void DebugService_ReportPower(uint32_t now_ms)
{
#if PX4LITE_ENABLE_BATTERY1 || PX4LITE_ENABLE_BATTERY2
  Px4Lite_BatteryStatus_t battery;
  Px4Lite_ModuleStatus_t status;
  Px4Lite_Result_t result;
  Px4Lite_Result_t status_result = Px4Lite_GetModuleStatus(PX4LITE_MODULE_BATTERY, &status);
#if PX4LITE_ENABLE_BATTERY1
  result = Px4Lite_CopyBattery(&battery);
  DBG_PRINT("POWER: input=1 state=%u ready=%u v_mv=%lu current_valid=%u current_ma=%ld age_ms=%lu",
            (status_result == PX4LITE_OK) ? (unsigned int)status.state : (unsigned int)PX4LITE_STATE_DISABLED,
            (unsigned int)(result == PX4LITE_OK),
            (result == PX4LITE_OK) ? (unsigned long)battery.voltage_mv : 0UL,
            (result == PX4LITE_OK) ? (unsigned int)battery.current_valid : 0U,
            (result == PX4LITE_OK) ? (long)battery.current_ma : 0L,
            (result == PX4LITE_OK) ? (unsigned long)(now_ms - battery.header.sample_time_ms) : 0UL);
#endif
#if PX4LITE_ENABLE_BATTERY2
  result = Px4Lite_CopyBattery2(&battery);
  DBG_PRINT("POWER: input=2 state=%u ready=%u v_mv=%lu current_valid=%u current_ma=%ld age_ms=%lu",
            (status_result == PX4LITE_OK) ? (unsigned int)status.state : (unsigned int)PX4LITE_STATE_DISABLED,
            (unsigned int)(result == PX4LITE_OK),
            (result == PX4LITE_OK) ? (unsigned long)battery.voltage_mv : 0UL,
            (result == PX4LITE_OK) ? (unsigned int)battery.current_valid : 0U,
            (result == PX4LITE_OK) ? (long)battery.current_ma : 0L,
            (result == PX4LITE_OK) ? (unsigned long)(now_ms - battery.header.sample_time_ms) : 0UL);
  DebugService_ReportPowerAdc(now_ms);
#endif
#else
  (void)now_ms;
#endif
}
#endif

#if DEBUG_LORA_MONITOR_ENABLE
/** @brief 只打印当前目录有效事实；不以旧分消息零值推断未发送。 */
static void DebugService_ReportLoRa(uint32_t now_ms)
{
  Px4Lite_CommDebugInfo_t info;Px4Lite_ModuleStatus_t status;Px4Lite_LabSnapshot_t lab;
  (void)now_ms;
  Px4Lite_GetCommDebugInfo(&info);Px4Lite_LoraLabCopy(&lab);
  if(Px4Lite_GetModuleStatus(PX4LITE_MODULE_LORA,&status)!=PX4LITE_OK) { DBG_PRINT("LORA: status unavailable");return; }
  DBG_PRINT("LORA: state=%u queued_session=%lu uart_TC_total=%lu module_idle=%lu rx=%lu busy=%lu crc=%lu parse=%lu queue_drop=%lu dma_overflow=%lu",
    (unsigned int)status.state,(unsigned long)lab.queued_frames,(unsigned long)lab.radio.uart_done,(unsigned long)lab.radio.module_idle,
    (unsigned long)info.rx_frame_count,(unsigned long)info.tx_busy_count,(unsigned long)info.crc_error_count,(unsigned long)info.parse_error_count,
    (unsigned long)info.rx_drop_count,(unsigned long)info.rx_overflow_count);
  /* 关键诊断：rx_byte=DMA收到的原始字节数，rx_frame=解析出的完整MAVLink帧数。
     rx_byte=0        -> E22 根本没收到任何RF字节(RF不通:天线/E22参数/接线/供电/对端没发)
     rx_byte>0,rx=0   -> 收到字节但解析不出帧(空口速率/波特率致数据损坏, 或帧格式) */
  DBG_PRINT("LORA-RX: rx_byte=%lu rx_frame=%lu crc_err=%lu parse_err=%lu",
            (unsigned long)info.rx_byte_count, (unsigned long)info.rx_frame_count,
            (unsigned long)info.crc_error_count, (unsigned long)info.parse_error_count);
}
#endif

#if DEBUG_ALARM_MONITOR_ENABLE
static void DebugService_ReportAlarm(uint32_t now_ms)
{
  App_AlarmSummary_t alarm;
  Px4Lite_Result_t result;

  result = App_CopyAlarmSummary(&alarm, now_ms);
  if (result == PX4LITE_OK) {
    DBG_PRINT("ALARM: count=%u highest_src=%u fault=0x%04X sev=%u", (unsigned int)alarm.active_count, (unsigned int)alarm.highest_source_id, (unsigned int)alarm.highest_fault_code, (unsigned int)alarm.highest_severity);
  } else {
    DBG_PRINT("ALARM: no snapshot rc=%d", (int)result);
  }
}
#endif

/**
 * @brief Run each independently enabled monitor at its configured period.
 */
static void DebugService_Task(void *argument)
{
  TickType_t last_wake;
  uint32_t now_ms;
#if DEBUG_STACK_MONITOR_ENABLE
  uint32_t last_stack_report_ms = 0U;
#endif
#if DEBUG_GNSS_MONITOR_ENABLE
  uint32_t last_gnss_report_ms = 0U;
#endif
#if DEBUG_IMU_MONITOR_ENABLE
  uint32_t last_imu_report_ms = 0U;
#endif
#if DEBUG_BARO_MONITOR_ENABLE
  uint32_t last_baro_report_ms = 0U;
#endif
#if DEBUG_IMU_MONITOR_ENABLE || DEBUG_BARO_MONITOR_ENABLE
  uint32_t last_sensor_state_report_ms = 0U;
#endif
#if DEBUG_POWER_MONITOR_ENABLE
  uint32_t last_power_report_ms = 0U;
#endif
#if DEBUG_LORA_MONITOR_ENABLE
  uint32_t last_lora_report_ms = 0U;
#endif
#if DEBUG_ALARM_MONITOR_ENABLE
  uint32_t last_alarm_report_ms = 0U;
#endif
#if DEBUG_SELFTEST_ACTIVE_ENABLE
  uint32_t last_selftest_report_ms = 0U;
#endif

  (void)argument;
  last_wake = xTaskGetTickCount();
#if DEBUG_STACK_MONITOR_ENABLE || DEBUG_IMU_MONITOR_ENABLE || DEBUG_BARO_MONITOR_ENABLE
  DBG_PRINT("SENS-DIAG: sensor-stack-20261111 sensor=%uW debug=%uW %s %s",
    (unsigned int)PX4LITE_STACK_SENSOR,(unsigned int)DEBUG_SERVICE_TASK_STACK_WORDS,__DATE__,__TIME__);
#endif

  for (;;) {
    now_ms = Px4Lite_PlatformGetMs();

#if DEBUG_IMU_MONITOR_ENABLE || DEBUG_BARO_MONITOR_ENABLE
    if((uint32_t)(now_ms-last_sensor_state_report_ms)>=DEBUG_IMU_REPORT_PERIOD_MS) {
      DebugService_ReportSensorStates();last_sensor_state_report_ms=now_ms;
    }
#endif

#if DEBUG_STACK_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_stack_report_ms) >= DEBUG_STACK_REPORT_PERIOD_MS) {
      DebugTaskMonitor_Report();
      last_stack_report_ms = now_ms;
    }
#endif

#if DEBUG_GNSS_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_gnss_report_ms) >= DEBUG_GNSS_REPORT_PERIOD_MS) {
      DebugService_ReportGnss(now_ms);
      last_gnss_report_ms = now_ms;
    }
#endif

#if DEBUG_IMU_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_imu_report_ms) >= DEBUG_IMU_REPORT_PERIOD_MS) {
      DebugService_ReportImu(now_ms);
      last_imu_report_ms = now_ms;
    }
#endif

#if DEBUG_BARO_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_baro_report_ms) >= DEBUG_BARO_REPORT_PERIOD_MS) {
      DebugService_ReportBaro(now_ms);
      last_baro_report_ms = now_ms;
    }
#endif

#if DEBUG_POWER_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_power_report_ms) >= DEBUG_POWER_REPORT_PERIOD_MS) {
      DebugService_ReportPower(now_ms);
      last_power_report_ms = now_ms;
    }
#endif

#if DEBUG_LORA_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_lora_report_ms) >= DEBUG_LORA_REPORT_PERIOD_MS) {
      DebugService_ReportLoRa(now_ms);
      last_lora_report_ms = now_ms;
    }
#endif

#if DEBUG_ALARM_MONITOR_ENABLE
    if ((uint32_t)(now_ms - last_alarm_report_ms) >= DEBUG_ALARM_REPORT_PERIOD_MS) {
      DebugService_ReportAlarm(now_ms);
      last_alarm_report_ms = now_ms;
    }
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE
    if ((uint32_t)(now_ms - last_selftest_report_ms) >= DEBUG_SELFTEST_REPORT_PERIOD_MS) {
      DebugSelfTest_Run(now_ms);
      last_selftest_report_ms = now_ms;
    }
#endif

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DEBUG_SERVICE_PERIOD_MS));
  }
}

#endif

/**
 * @brief Create and register the optional low-priority DebugTask.
 */
BaseType_t DebugService_CreateTask(void)
{
#if DEBUG_PERIODIC_SERVICE_ENABLE
  TaskHandle_t task_handle = 0;

  if (xTaskCreate(DebugService_Task, "debug", DEBUG_SERVICE_TASK_STACK_WORDS, 0, DEBUG_SERVICE_TASK_PRIORITY, &task_handle) != pdPASS) { return pdFAIL; }

  (void)DebugTaskMonitor_Register(task_handle, "debug", DEBUG_SERVICE_TASK_STACK_WORDS);
#endif

  return pdPASS;
}
