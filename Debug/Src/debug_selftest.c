/**
 * @file debug_selftest.c
 * @brief 实现 DebugTask 驱动的分项自测打印。
 *
 * @details
 * 本文件属于 Debug 层，是少数允许跨层读取只读诊断快照的特例：它用于人工 bring-up、
 * 单项验证和现场定位，不参与正式业务闭环。新增测试项必须保持默认关闭、只读、低频打印，
 * 且不得在这里执行总线 reinit、协议发送、SD 写入或 UI 渲染。
 */

#include "debug_selftest.h"

#include "debug_config.h"
#include "debug_console.h"

#if DEBUG_SELFTEST_ACTIVE_ENABLE
#include "app_data_api.h"
#include "px4lite_config.h"
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_BSP_INIT_ENABLE
#include "bsp.h"
#include "bsp_uart.h"
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_DISPLAY_BUDGET_ENABLE
#include "display.h"
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_GT911_ENABLE
#include "display_gt911.h"
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_IMU_SNAPSHOT_ENABLE
#include "sensor_mpu6050.h"
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_GNSS_ENABLE
static App_NavigationSnapshot_t s_selftest_navigation;
static Px4Lite_ModuleStatus_t s_selftest_gnss_status;
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_LORA_ENABLE
static Px4Lite_CommDebugInfo_t s_selftest_comm;
static Px4Lite_ModuleStatus_t s_selftest_lora_status;
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && (DEBUG_SELFTEST_REMOTE_VIEW_ENABLE || DEBUG_SELFTEST_LORA_ENABLE)
static App_RemoteNodeView_t s_selftest_remote_nodes[PX4LITE_REMOTE_NODE_MAX];
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_IMU_SNAPSHOT_ENABLE
/**
 * @brief 将浮点量按指定比例转换为整数，避免调试串口使用浮点格式化。
 *
 * @param[in] value 原始浮点值。
 * @param[in] scale 缩放倍数。
 *
 * @return 缩放后的整数值。
 */
static int32_t DebugSelfTest_ScaleFloat(float value, float scale)
{
  return (int32_t)(value * scale);
}
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_IMU_SNAPSHOT_ENABLE
/**
 * @brief 打印 MPU6050 快照一致性自测结果。
 *
 * @details
 * 本测试直接读取 Sensor Driver 的只读诊断接口，是 Debug 特例，不得照搬到 Business 或
 * Display 层。重点确认 60 字节 IMU snapshot 复制不会出现序号倒退或状态/样本不一致。
 */
static void DebugSelfTest_ReportImuSnapshot(void)
{
  Mpu6050_Status_t status;
  Mpu6050_Snapshot_t sample;
  Mpu6050_Result_t status_rc;
  Mpu6050_Result_t sample_rc;
  uint8_t consistent = 1U;

  status_rc = Sensor_MPU6050_GetStatus(&status);
  sample_rc = Sensor_MPU6050_CopySnapshot(&sample);

  if ((status_rc == MPU6050_RESULT_OK) && (sample_rc == MPU6050_RESULT_OK)) {
    if (sample.rx_sequence > status.rx_sequence) { consistent = 0U; }
    if (sample.error_count != status.error_count) { consistent = 0U; }

    DBG_PRINT("SELFTEST IMU: rc=%d/%d ok=%u seq=%lu sample_ms=%lu err=%lu "
              "stage=%u result=%u reinit=%lu accel_mg=%ld,%ld,%ld "
              "gyro_mdps=%ld,%ld,%ld temp_cC=%ld",
              (int)status_rc, (int)sample_rc, (unsigned int)consistent, (unsigned long)sample.rx_sequence, (unsigned long)sample.sample_time_ms, (unsigned long)sample.error_count, (unsigned int)status.last_stage, (unsigned int)status.last_result, (unsigned long)status.reinit_count, (long)DebugSelfTest_ScaleFloat(sample.accel_g[0], 1000.0f), (long)DebugSelfTest_ScaleFloat(sample.accel_g[1], 1000.0f), (long)DebugSelfTest_ScaleFloat(sample.accel_g[2], 1000.0f), (long)DebugSelfTest_ScaleFloat(sample.gyro_dps[0], 1000.0f), (long)DebugSelfTest_ScaleFloat(sample.gyro_dps[1], 1000.0f), (long)DebugSelfTest_ScaleFloat(sample.gyro_dps[2], 1000.0f), (long)DebugSelfTest_ScaleFloat(sample.temperature_c, 100.0f));
  } else {
    DBG_PRINT("SELFTEST IMU: rc=%d/%d seq=%lu err=%lu stage=%u result=%u reinit=%lu chip=0x%02X bsp=%u",
              (int)status_rc, (int)sample_rc, (unsigned long)status.rx_sequence, (unsigned long)status.error_count, (unsigned int)status.last_stage, (unsigned int)status.last_result, (unsigned long)status.reinit_count, (unsigned int)status.last_chip_id, (unsigned int)status.last_bsp_status);
  }
}
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_BSP_INIT_ENABLE
/**
 * @brief 打印 BSP 启动链路和调试 UART 配置自测结果。
 */
static void DebugSelfTest_ReportBspInit(void)
{
  BSP_InitDebugInfo_t init_info;
  BSP_UART_DebugInfo_t uart_info;

  BSP_GetInitDebugInfo(&init_info);
  BSP_UART_GetDebugInfo(&uart_info);

  DBG_PRINT("SELFTEST BSP: result=%u enabled=0x%08lX attempted=0x%08lX failed=0x%08lX "
            "uart_init=%u inst=0x%08lX baud=%lu word=0x%08lX stop=0x%08lX parity=0x%08lX mode=0x%08lX",
            (unsigned int)init_info.result, (unsigned long)init_info.enabled_mask, (unsigned long)init_info.attempted_mask, (unsigned long)init_info.failed_mask, (unsigned int)uart_info.initialized, (unsigned long)uart_info.instance, (unsigned long)uart_info.baud_rate, (unsigned long)uart_info.word_length, (unsigned long)uart_info.stop_bits, (unsigned long)uart_info.parity, (unsigned long)uart_info.mode);
}
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_DISPLAY_BUDGET_ENABLE
/**
 * @brief 打印 LVGL 刷新预算执行统计。
 */
static void DebugSelfTest_ReportDisplayBudget(void)
{
  Display_DebugStats_t stats;

  Display_GetDebugStats(&stats);
  DBG_PRINT("SELFTEST DISPLAY: last_us=%lu max_us=%lu budget_busy=%lu rebuild=%lu",
            (unsigned long)stats.last_refresh_elapsed_us, (unsigned long)stats.max_refresh_elapsed_us, (unsigned long)stats.budget_busy_count, (unsigned long)stats.page_rebuild_count);
}
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_GNSS_ENABLE
/**
 * @brief 打印 GNSS 业务可见快照和模块状态。
 *
 * @param[in] now_ms 当前系统毫秒时间，用于 App 新鲜度判断。
 */
static void DebugSelfTest_ReportGnss(uint32_t now_ms)
{
  Px4Lite_Result_t nav_rc;
  Px4Lite_Result_t status_rc;
  uint32_t age_ms = 0U;

  nav_rc = App_CopyNavigation(&s_selftest_navigation, now_ms);
  status_rc = App_GetModuleStatus(PX4LITE_MODULE_GNSS, &s_selftest_gnss_status);
  if (status_rc == PX4LITE_OK) { age_ms = Px4Lite_ElapsedMs(now_ms, s_selftest_gnss_status.last_rx_ms); }

  DBG_PRINT("SELFTEST GNSS: nav_rc=%d status_rc=%d state=%u fault=%u age=%lu "
            "mask=0x%08lX fix=%u sats=%u hdop=%u quality=%u lat=%ld lon=%ld",
            (int)nav_rc, (int)status_rc, (status_rc == PX4LITE_OK) ? (unsigned int)s_selftest_gnss_status.state : 0U, (status_rc == PX4LITE_OK) ? (unsigned int)s_selftest_gnss_status.fault_code : 0U, (unsigned long)age_ms, (nav_rc == PX4LITE_OK) ? (unsigned long)s_selftest_navigation.valid_mask : 0UL, (nav_rc == PX4LITE_OK) ? (unsigned int)s_selftest_navigation.gnss_fix_type : 0U, (nav_rc == PX4LITE_OK) ? (unsigned int)s_selftest_navigation.satellites_used : 0U, (nav_rc == PX4LITE_OK) ? (unsigned int)s_selftest_navigation.hdop_x100 : 0U, (nav_rc == PX4LITE_OK) ? (unsigned int)s_selftest_navigation.navigation_quality : 0U, (nav_rc == PX4LITE_OK) ? (long)s_selftest_navigation.latitude_e7 : 0L, (nav_rc == PX4LITE_OK) ? (long)s_selftest_navigation.longitude_e7 : 0L);
}
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_LORA_ENABLE
/**
 * @brief 打印 LoRa/MAVLink 统计和远端节点摘要。
 *
 * @param[in] now_ms 当前系统毫秒时间，用于远端节点新鲜度判断。
 */
static void DebugSelfTest_ReportLoRa(uint32_t now_ms)
{
  Px4Lite_Result_t status_rc;
  Px4Lite_Result_t nodes_rc;
  uint8_t count = 0U;
  uint8_t i;

  App_GetCommStats(&s_selftest_comm);
  status_rc = App_GetModuleStatus(PX4LITE_MODULE_LORA, &s_selftest_lora_status);
  nodes_rc = App_CopyRemoteNodeStatuses(s_selftest_remote_nodes, PX4LITE_REMOTE_NODE_MAX, &count, now_ms);

  DBG_PRINT("SELFTEST LORA: status_rc=%d nodes_rc=%d state=%u fault=%u "
            "tx=%lu rx=%lu busy=%lu err=%lu ack=%lu/%lu lost=%lu loss_x10=%u last_msg=%lu nodes=%u",
            (int)status_rc, (int)nodes_rc, (status_rc == PX4LITE_OK) ? (unsigned int)s_selftest_lora_status.state : 0U, (status_rc == PX4LITE_OK) ? (unsigned int)s_selftest_lora_status.fault_code : 0U, (unsigned long)s_selftest_comm.tx_frame_count, (unsigned long)s_selftest_comm.rx_frame_count, (unsigned long)s_selftest_comm.tx_busy_count, (unsigned long)(s_selftest_comm.send_error_count + s_selftest_comm.mav_error_count), (unsigned long)s_selftest_comm.mav_command_ack_tx_count, (unsigned long)s_selftest_comm.mav_command_ack_rx_count, (unsigned long)s_selftest_comm.rx_sequence_lost_count, (unsigned int)s_selftest_comm.rx_loss_rate_x10, (unsigned long)s_selftest_comm.last_msg_id, (unsigned int)count);

  for (i = 0U; i < count; ++i) {
    DBG_PRINT("SELFTEST LORA_NODE[%u]: node=%u sys=%u type=%u sys_status=%u state=%u "
              "hb_ms=%lu data_ms=%lu rx=%lu lost=%lu loss_x10=%u",
              (unsigned int)i, (unsigned int)s_selftest_remote_nodes[i].node_id, (unsigned int)s_selftest_remote_nodes[i].system_id, (unsigned int)s_selftest_remote_nodes[i].heartbeat_type, (unsigned int)s_selftest_remote_nodes[i].heartbeat_system_status, (unsigned int)s_selftest_remote_nodes[i].state, (unsigned long)s_selftest_remote_nodes[i].last_heartbeat_ms, (unsigned long)s_selftest_remote_nodes[i].last_data_ms, (unsigned long)s_selftest_remote_nodes[i].rx_frame_count, (unsigned long)s_selftest_remote_nodes[i].rx_sequence_lost_count, (unsigned int)s_selftest_remote_nodes[i].rx_loss_rate_x10);
  }
}
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_REMOTE_VIEW_ENABLE
/**
 * @brief 打印远端节点选择和节点列表视图。
 *
 * @param[in] now_ms 当前系统毫秒时间，用于 App 远端节点新鲜度判断。
 */
static void DebugSelfTest_ReportRemoteView(uint32_t now_ms)
{
  Px4Lite_Result_t result;
  uint8_t selected_node = 0U;
  uint8_t count = 0U;
  uint8_t i;

  (void)App_GetSelectedRemoteNode(&selected_node);
  result = App_CopyRemoteNodeStatuses(s_selftest_remote_nodes, PX4LITE_REMOTE_NODE_MAX, &count, now_ms);

  DBG_PRINT("SELFTEST REMOTE: rc=%d selected=%u count=%u", (int)result, (unsigned int)selected_node, (unsigned int)count);

  for (i = 0U; i < count; ++i) {
    DBG_PRINT("SELFTEST REMOTE[%u]: node=%u sys=%u type=%u sys_status=%u state=%u hb_ms=%lu data_ms=%lu rx=%lu lost=%lu loss_x10=%u",
              (unsigned int)i, (unsigned int)s_selftest_remote_nodes[i].node_id, (unsigned int)s_selftest_remote_nodes[i].system_id, (unsigned int)s_selftest_remote_nodes[i].heartbeat_type, (unsigned int)s_selftest_remote_nodes[i].heartbeat_system_status, (unsigned int)s_selftest_remote_nodes[i].state, (unsigned long)s_selftest_remote_nodes[i].last_heartbeat_ms, (unsigned long)s_selftest_remote_nodes[i].last_data_ms, (unsigned long)s_selftest_remote_nodes[i].rx_frame_count, (unsigned long)s_selftest_remote_nodes[i].rx_sequence_lost_count, (unsigned int)s_selftest_remote_nodes[i].rx_loss_rate_x10);
  }
}
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE && DEBUG_SELFTEST_GT911_ENABLE
/**
 * @brief 打印 GT911 触摸控制器初始化和最近一次扫描诊断。
 */
static void DebugSelfTest_ReportGt911(void)
{
  Display_Gt911Debug_t debug;

  Display_Gt911_GetDebug(&debug);
  DBG_PRINT("SELFTEST GT911: addr=0x%02X result=%u err=%u init_err=%u cfg_ver=%u cfg_ok=%u "
            "int=%u scl=%u sda=%u status=0x%02X cfg=%02X%02X%02X%02X",
            (unsigned int)debug.addr, (unsigned int)debug.last_result, (unsigned int)debug.last_error, (unsigned int)debug.init_error, (unsigned int)debug.cfg_version, (unsigned int)debug.cfg_verified, (unsigned int)debug.int_level, (unsigned int)debug.scl_level, (unsigned int)debug.sda_level, (unsigned int)debug.touch_status, (unsigned int)debug.cfg_bytes[0], (unsigned int)debug.cfg_bytes[1], (unsigned int)debug.cfg_bytes[2], (unsigned int)debug.cfg_bytes[3]);
}
#endif

#if DEBUG_SELFTEST_ACTIVE_ENABLE
/**
 * @brief 执行一次已启用的 Debug 分项自测打印。
 *
 * @param[in] now_ms 当前系统毫秒时间，用于远端快照新鲜度判断。
 */
void DebugSelfTest_Run(uint32_t now_ms)
{
#if DEBUG_SELFTEST_IMU_SNAPSHOT_ENABLE
  DebugSelfTest_ReportImuSnapshot();
#endif

#if DEBUG_SELFTEST_BSP_INIT_ENABLE
  DebugSelfTest_ReportBspInit();
#endif

#if DEBUG_SELFTEST_DISPLAY_BUDGET_ENABLE
  DebugSelfTest_ReportDisplayBudget();
#endif

#if DEBUG_SELFTEST_GNSS_ENABLE
  DebugSelfTest_ReportGnss(now_ms);
#endif

#if DEBUG_SELFTEST_LORA_ENABLE
  DebugSelfTest_ReportLoRa(now_ms);
#endif

#if DEBUG_SELFTEST_REMOTE_VIEW_ENABLE
  DebugSelfTest_ReportRemoteView(now_ms);
#endif

#if DEBUG_SELFTEST_GT911_ENABLE
  DebugSelfTest_ReportGt911();
#endif

#if !(DEBUG_SELFTEST_GNSS_ENABLE || DEBUG_SELFTEST_LORA_ENABLE || DEBUG_SELFTEST_REMOTE_VIEW_ENABLE)
  (void)now_ms;
#endif
}

#else
/**
 * @brief Debug 自测关闭时的空实现。
 *
 * @param[in] now_ms 当前系统毫秒时间，关闭自测时不使用。
 */
void DebugSelfTest_Run(uint32_t now_ms)
{
  (void)now_ms;
}
#endif
