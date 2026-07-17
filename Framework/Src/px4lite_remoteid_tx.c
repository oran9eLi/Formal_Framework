/**
 * @file px4lite_remoteid_tx.c
 * @brief 将 Framework 导航和身份配置编码为 MAVLink/OpenDroneID 并发送到 ESP32-S3。
 *
 * @details
 * CommTask 是本文件唯一调用者。模块使用静态 MAVLink 消息、静态发送缓冲和静态字段缓存，
 * 不在运行期分配内存；每个调度周期最多提交一帧，避免 UART4 DMA 和 comm 任务栈被突发发送拖大。
 */

#include "px4lite_remoteid_tx.h"

#include <limits.h>
#include <string.h>

#include "px4lite_config.h"
#include "px4lite_identity.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_platform.h"
#include "px4lite_topics.h"

#if defined(__CC_ARM)
#define MAVLINK_ALIGNED_FIELDS   0
#define MAVLINK_COMM_NUM_BUFFERS 1
#pragma diag_suppress 66
#endif

#include "common/mavlink.h"

#if defined(__CC_ARM)
#pragma diag_default 66
#endif

#if PX4LITE_ENABLE_REMOTE_ID

typedef Px4Lite_Result_t (*RemoteId_EncodeFn_t)(uint32_t now_ms);

typedef struct {
  uint32_t message_id;       /**< MAVLink message id。 */
  uint32_t period_ms;        /**< 发送周期，单位 ms。 */
  uint32_t *next_ms;         /**< 下次发送时刻，单位 ms。 */
  uint32_t *success_count;   /**< 成功提交计数。 */
  RemoteId_EncodeFn_t encode;/**< 编码并提交一帧。 */
} RemoteId_Item_t;

static mavlink_message_t s_remoteid_message;
static uint8_t s_remoteid_frame[MAVLINK_MAX_PACKET_LEN];
static Px4Lite_RemoteIdTxStats_t s_remoteid_stats;
static Px4Lite_VehicleNavigation_t s_remoteid_nav;

static uint8_t s_id_or_mac[MAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_FIELD_ID_OR_MAC_LEN];
static uint8_t s_uas_id[MAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_FIELD_UAS_ID_LEN];
static char s_operator_id[MAVLINK_MSG_OPEN_DRONE_ID_OPERATOR_ID_FIELD_OPERATOR_ID_LEN];
static char s_self_id[MAVLINK_MSG_OPEN_DRONE_ID_SELF_ID_FIELD_DESCRIPTION_LEN];
static uint8_t s_home_valid;
static int32_t s_home_latitude_e7;
static int32_t s_home_longitude_e7;
static float s_home_altitude_m;

static uint32_t s_next_heartbeat_ms;
static uint32_t s_next_basic_id_ms;
static uint32_t s_next_location_ms;
static uint32_t s_next_system_ms;
static uint32_t s_next_operator_id_ms;
static uint32_t s_next_self_id_ms;
static uint8_t s_catalog_index;

/**
 * @brief 使用回绕安全差值判断一个毫秒 deadline 是否到期。
 */
static uint8_t RemoteId_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

/**
 * @brief 将 C 字符串复制到 MAVLink 固定长度字段，剩余部分补零。
 */
static void RemoteId_FillText(uint8_t *dst, uint8_t dst_len, const char *src)
{
  uint8_t i;

  if ((dst == 0) || (dst_len == 0U)) { return; }
  memset(dst, 0, dst_len);
  if (src == 0) { return; }

  for (i = 0U; (i < dst_len) && (src[i] != '\0'); ++i) {
    dst[i] = (uint8_t)src[i];
  }
}

/**
 * @brief 将 C 字符串复制到 MAVLink char 固定长度字段。
 */
static void RemoteId_FillCharText(char *dst, uint8_t dst_len, const char *src)
{
  uint8_t i;

  if ((dst == 0) || (dst_len == 0U)) { return; }
  memset(dst, 0, dst_len);
  if (src == 0) { return; }

  for (i = 0U; (i < dst_len) && (src[i] != '\0'); ++i) {
    dst[i] = src[i];
  }
}

/**
 * @brief 将 STM32 96-bit UID 填入 OpenDroneID id_or_mac 字段。
 */
static void RemoteId_FillHardwareUid(void)
{
  uint32_t uid_words[3];
  uint8_t i;
  uint8_t offset;

  memset(s_id_or_mac, 0, sizeof(s_id_or_mac));
  if (Px4Lite_PlatformGetHardwareUid(uid_words, 3U) != PX4LITE_OK) { return; }

  for (i = 0U; i < 3U; i++) {
    offset = (uint8_t)(i * 4U);
    s_id_or_mac[offset + 0U] = (uint8_t)((uid_words[i] >> 24) & 0xFFU);
    s_id_or_mac[offset + 1U] = (uint8_t)((uid_words[i] >> 16) & 0xFFU);
    s_id_or_mac[offset + 2U] = (uint8_t)((uid_words[i] >> 8) & 0xFFU);
    s_id_or_mac[offset + 3U] = (uint8_t)(uid_words[i] & 0xFFU);
  }
}

/**
 * @brief 将厘米每秒速度限幅为 OpenDroneID 水平速度字段。
 */
static uint16_t RemoteId_HorizontalSpeedCms(const Px4Lite_VehicleNavigation_t *nav)
{
  int32_t north;
  int32_t east;
  uint32_t speed_cms;

  if ((nav == 0) || ((nav->valid_mask & PX4LITE_NAV_VALID_VELOCITY) == 0U)) { return 0U; }

  north = (nav->velocity_north_cms >= 0) ? nav->velocity_north_cms : -nav->velocity_north_cms;
  east  = (nav->velocity_east_cms >= 0) ? nav->velocity_east_cms : -nav->velocity_east_cms;
  speed_cms = (uint32_t)north + (uint32_t)east;
  if (speed_cms > UINT16_MAX) { return UINT16_MAX; }
  return (uint16_t)speed_cms;
}

/**
 * @brief 将垂直速度限幅到 int16，单位 cm/s。
 */
static int16_t RemoteId_SpeedVerticalCms(const Px4Lite_VehicleNavigation_t *nav)
{
  int32_t value;

  if ((nav == 0) || ((nav->valid_mask & PX4LITE_NAV_VALID_ALTITUDE) == 0U)) { return 0; }
  value = -nav->vertical_speed_cms;
  if (value > INT16_MAX) { return INT16_MAX; }
  if (value < INT16_MIN) { return INT16_MIN; }
  return (int16_t)value;
}

/**
 * @brief 将 yaw_deg100 归一化到 OpenDroneID 航向字段，单位 degree * 100。
 */
static uint16_t RemoteId_DirectionDeg100(const Px4Lite_VehicleNavigation_t *nav)
{
  int32_t heading;

  if ((nav == 0) || ((nav->valid_mask & PX4LITE_NAV_VALID_ATTITUDE) == 0U)) { return UINT16_MAX; }

  heading = nav->yaw_deg100 % 36000;
  if (heading < 0) { heading += 36000; }
  return (uint16_t)heading;
}

static void RemoteId_UpdateHomeIfNeeded(const Px4Lite_VehicleNavigation_t *nav)
{
  if ((nav == 0) || (s_home_valid != 0U)) { return; }
  if ((nav->valid_mask & PX4LITE_NAV_VALID_POSITION) == 0U) { return; }

  s_home_latitude_e7 = nav->latitude_e7;
  s_home_longitude_e7 = nav->longitude_e7;
  s_home_altitude_m = ((float)nav->fused_altitude_mm) / 1000.0f;
  s_home_valid = 1U;
}

static uint8_t RemoteId_HorizontalAccuracy(const Px4Lite_VehicleNavigation_t *nav)
{
  uint16_t hdop;

  if ((nav == 0) || (nav->hdop_x100 == 0U)) { return MAV_ODID_HOR_ACC_UNKNOWN; }

  hdop = nav->hdop_x100;
  if (hdop <= 80U) { return MAV_ODID_HOR_ACC_3_METER; }
  if (hdop <= 150U) { return MAV_ODID_HOR_ACC_10_METER; }
  if (hdop <= 300U) { return MAV_ODID_HOR_ACC_30_METER; }
  if (hdop <= 800U) { return MAV_ODID_HOR_ACC_0_05NM; }
  return MAV_ODID_HOR_ACC_0_1NM;
}

/**
 * @brief 将已编码 MAVLink 消息提交到 ESP32-S3 UART4。
 */
static Px4Lite_Result_t RemoteId_SendPrepared(void)
{
  uint16_t length;

  length = mavlink_msg_to_send_buffer(s_remoteid_frame, &s_remoteid_message);
  if ((length == 0U) || (length > (uint16_t)sizeof(s_remoteid_frame))) { return PX4LITE_IO_ERROR; }

#if PX4LITE_ENABLE_RPI_MAVLINK
  /* D2：身份数据同时镜像到树莓派 USART6(compid 193, RPi 独立序号)，供 RPi 复用其
     现成的 OPEN_DRONE_ID_* 解码。HEARTBEAT 不镜像(RPi 已有遥测链路心跳)。镜像只临时
     改写 s_remoteid_message 并即时恢复，不影响随后发往 UART4 的 s_remoteid_frame。 */
  if (s_remoteid_message.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
    (void)Px4Lite_MavlinkTxMirrorToRpi(&s_remoteid_message);
  }
#endif

  return Px4Lite_RemoteIdSend(s_remoteid_frame, length);
}

/**
 * @brief 根据发送结果更新周期和统计。
 */
static void RemoteId_RecordResult(Px4Lite_Result_t result, uint32_t now_ms, const RemoteId_Item_t *item)
{
  if (item == 0) { return; }

  if (result == PX4LITE_OK) {
    *item->next_ms       = now_ms + item->period_ms;
    *item->success_count = *item->success_count + 1U;
    s_remoteid_stats.last_message_id = item->message_id;
    s_remoteid_stats.last_success_ms = now_ms;
  } else {
    *item->next_ms = now_ms + PX4LITE_REMOTEID_RETRY_PERIOD_MS;
    if ((result == PX4LITE_NOT_READY) || (result == PX4LITE_IDLE)) {
      s_remoteid_stats.no_data_count++;
    } else if (result == PX4LITE_STALE) {
      s_remoteid_stats.stale_count++;
    } else if (result == PX4LITE_BUSY) {
      s_remoteid_stats.busy_count++;
      s_remoteid_stats.last_busy_ms = now_ms;
    } else {
      s_remoteid_stats.error_count++;
      s_remoteid_stats.last_error_ms = now_ms;
    }
  }
}

/**
 * @brief 发送 MAVLink HEARTBEAT，帮助 ESP32 识别上游 MAVLink 源。
 */
static Px4Lite_Result_t RemoteId_SendHeartbeat(uint32_t now_ms)
{
  (void)now_ms;
  (void)mavlink_msg_heartbeat_pack_chan(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_REMOTEID_MAVLINK_COMPONENT_ID, MAVLINK_COMM_0, &s_remoteid_message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_INVALID, 0U, 0U, MAV_STATE_ACTIVE);
  return RemoteId_SendPrepared();
}

/**
 * @brief 发送 UAS 基础身份。
 */
static Px4Lite_Result_t RemoteId_SendBasicId(uint32_t now_ms)
{
  (void)now_ms;
  (void)mavlink_msg_open_drone_id_basic_id_pack(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_REMOTEID_MAVLINK_COMPONENT_ID, &s_remoteid_message, PX4LITE_REMOTEID_TARGET_SYSTEM, PX4LITE_REMOTEID_TARGET_COMPONENT, s_id_or_mac, MAV_ODID_ID_TYPE_SERIAL_NUMBER, MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR, s_uas_id);
  return RemoteId_SendPrepared();
}

/**
 * @brief 发送当前位置和速度。
 */
static Px4Lite_Result_t RemoteId_SendLocation(uint32_t now_ms)
{
  float altitude_m;
  float timestamp_s;
  Px4Lite_Result_t result;

  result = Px4Lite_CopyNavigation(&s_remoteid_nav);
  if (result != PX4LITE_OK) { return PX4LITE_NOT_READY; }
  if (Px4Lite_IsFresh(&s_remoteid_nav.header, now_ms, PX4LITE_GNSS_MAX_AGE_MS) == 0U) { return PX4LITE_STALE; }
  if ((s_remoteid_nav.valid_mask & PX4LITE_NAV_VALID_POSITION) == 0U) { return PX4LITE_NOT_READY; }
  if (s_remoteid_nav.header.sequence == s_remoteid_stats.last_location_seq) { return PX4LITE_IDLE; }

  RemoteId_UpdateHomeIfNeeded(&s_remoteid_nav);
  /* altitude_m 是绝对融合高度：同时填 altitude_barometric 与 altitude_geodetic(绝对 WGS84)；
     height 字段(height_reference=OVER_TAKEOFF)另填相对起飞点的高度差，勿与 geodetic 混淆。 */
  altitude_m = ((float)s_remoteid_nav.fused_altitude_mm) / 1000.0f;
  timestamp_s = (float)(s_remoteid_nav.gnss_utc_sec % 86400UL);

  (void)mavlink_msg_open_drone_id_location_pack(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_REMOTEID_MAVLINK_COMPONENT_ID, &s_remoteid_message, PX4LITE_REMOTEID_TARGET_SYSTEM, PX4LITE_REMOTEID_TARGET_COMPONENT, s_id_or_mac, MAV_ODID_STATUS_AIRBORNE, RemoteId_DirectionDeg100(&s_remoteid_nav), RemoteId_HorizontalSpeedCms(&s_remoteid_nav), RemoteId_SpeedVerticalCms(&s_remoteid_nav), s_remoteid_nav.latitude_e7, s_remoteid_nav.longitude_e7, altitude_m, altitude_m, MAV_ODID_HEIGHT_REF_OVER_TAKEOFF, (s_home_valid != 0U) ? (altitude_m - s_home_altitude_m) : 0.0f, RemoteId_HorizontalAccuracy(&s_remoteid_nav), MAV_ODID_VER_ACC_UNKNOWN, MAV_ODID_VER_ACC_UNKNOWN, MAV_ODID_SPEED_ACC_UNKNOWN, timestamp_s, MAV_ODID_TIME_ACC_UNKNOWN);

  result = RemoteId_SendPrepared();
  if (result == PX4LITE_OK) { s_remoteid_stats.last_location_seq = s_remoteid_nav.header.sequence; }
  return result;
}

/**
 * @brief 发送系统和操作者位置，当前以起飞点同飞机当前位置口径占位。
 */
static Px4Lite_Result_t RemoteId_SendSystem(uint32_t now_ms)
{
  float altitude_m = 0.0f;
  int32_t latitude = 0;
  int32_t longitude = 0;

  if (Px4Lite_CopyNavigation(&s_remoteid_nav) == PX4LITE_OK) {
    if ((Px4Lite_IsFresh(&s_remoteid_nav.header, now_ms, PX4LITE_GNSS_MAX_AGE_MS) != 0U) && ((s_remoteid_nav.valid_mask & PX4LITE_NAV_VALID_POSITION) != 0U)) {
      latitude = s_remoteid_nav.latitude_e7;
      longitude = s_remoteid_nav.longitude_e7;
      altitude_m = ((float)s_remoteid_nav.fused_altitude_mm) / 1000.0f;
      RemoteId_UpdateHomeIfNeeded(&s_remoteid_nav);
    }
  }
  if (s_home_valid != 0U) {
    latitude = s_home_latitude_e7;
    longitude = s_home_longitude_e7;
    altitude_m = s_home_altitude_m;
  }

  (void)mavlink_msg_open_drone_id_system_pack(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_REMOTEID_MAVLINK_COMPONENT_ID, &s_remoteid_message, PX4LITE_REMOTEID_TARGET_SYSTEM, PX4LITE_REMOTEID_TARGET_COMPONENT, s_id_or_mac, MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF, MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED, latitude, longitude, 1U, 0U, -1000.0f, -1000.0f, MAV_ODID_CATEGORY_EU_UNDECLARED, MAV_ODID_CLASS_EU_UNDECLARED, altitude_m, now_ms / 1000UL);
  return RemoteId_SendPrepared();
}

/**
 * @brief 发送操作者身份。
 */
static Px4Lite_Result_t RemoteId_SendOperatorId(uint32_t now_ms)
{
  (void)now_ms;
  (void)mavlink_msg_open_drone_id_operator_id_pack(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_REMOTEID_MAVLINK_COMPONENT_ID, &s_remoteid_message, PX4LITE_REMOTEID_TARGET_SYSTEM, PX4LITE_REMOTEID_TARGET_COMPONENT, s_id_or_mac, MAV_ODID_OPERATOR_ID_TYPE_CAA, s_operator_id);
  return RemoteId_SendPrepared();
}

/**
 * @brief 发送自描述文本。
 */
static Px4Lite_Result_t RemoteId_SendSelfId(uint32_t now_ms)
{
  (void)now_ms;
  (void)mavlink_msg_open_drone_id_self_id_pack(Px4Lite_IdentityGetMavlinkSystemId(), PX4LITE_REMOTEID_MAVLINK_COMPONENT_ID, &s_remoteid_message, PX4LITE_REMOTEID_TARGET_SYSTEM, PX4LITE_REMOTEID_TARGET_COMPONENT, s_id_or_mac, MAV_ODID_DESC_TYPE_TEXT, s_self_id);
  return RemoteId_SendPrepared();
}

static const RemoteId_Item_t s_remoteid_catalog[] = {
    {MAVLINK_MSG_ID_HEARTBEAT, PX4LITE_REMOTEID_HEARTBEAT_PERIOD_MS, &s_next_heartbeat_ms, &s_remoteid_stats.heartbeat_count, RemoteId_SendHeartbeat},
    {MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID, PX4LITE_REMOTEID_BASIC_ID_PERIOD_MS, &s_next_basic_id_ms, &s_remoteid_stats.basic_id_count, RemoteId_SendBasicId},
    {MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION, PX4LITE_REMOTEID_LOCATION_PERIOD_MS, &s_next_location_ms, &s_remoteid_stats.location_count, RemoteId_SendLocation},
    {MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM, PX4LITE_REMOTEID_SYSTEM_PERIOD_MS, &s_next_system_ms, &s_remoteid_stats.system_count, RemoteId_SendSystem},
    {MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID, PX4LITE_REMOTEID_OPERATOR_ID_PERIOD_MS, &s_next_operator_id_ms, &s_remoteid_stats.operator_id_count, RemoteId_SendOperatorId},
    {MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID, PX4LITE_REMOTEID_SELF_ID_PERIOD_MS, &s_next_self_id_ms, &s_remoteid_stats.self_id_count, RemoteId_SendSelfId},
};

#define REMOTEID_CATALOG_COUNT ((uint8_t)(sizeof(s_remoteid_catalog) / sizeof(s_remoteid_catalog[0])))

Px4Lite_Result_t Px4Lite_RemoteIdTxInit(uint32_t now_ms)
{
  memset(&s_remoteid_message, 0, sizeof(s_remoteid_message));
  memset(s_remoteid_frame, 0, sizeof(s_remoteid_frame));
  memset(&s_remoteid_stats, 0, sizeof(s_remoteid_stats));
  memset(&s_remoteid_nav, 0, sizeof(s_remoteid_nav));
  s_home_valid = 0U;
  s_home_latitude_e7 = 0;
  s_home_longitude_e7 = 0;
  s_home_altitude_m = 0.0f;
  RemoteId_FillHardwareUid();
  {
    char full_id[PX4LITE_IDENTITY_FULL_ID_MAX_LEN];
    char vendor_id[PX4LITE_IDENTITY_VENDOR_ID_MAX_LEN];
    char serial_number[PX4LITE_IDENTITY_SN_MAX_LEN];
    char self_desc[MAVLINK_MSG_OPEN_DRONE_ID_SELF_ID_FIELD_DESCRIPTION_LEN + 1U];
    uint8_t pos = 0U;
    uint8_t i;

    Px4Lite_IdentityFormatFullId(full_id, (uint8_t)sizeof(full_id));
    Px4Lite_IdentityFormatVendorProductId(vendor_id, (uint8_t)sizeof(vendor_id));
    Px4Lite_IdentityFormatSerialNumber(serial_number, (uint8_t)sizeof(serial_number));
    RemoteId_FillText(s_uas_id, (uint8_t)sizeof(s_uas_id), vendor_id);

    /* SELF_ID 仅作现场可读描述；RemoteID 权威唯一身份在 BASIC_ID.uas_id。 */
    memset(self_desc, 0, sizeof(self_desc));
    while ((full_id[pos] != '\0') && (pos < (uint8_t)(sizeof(self_desc) - 16U))) {
      self_desc[pos] = full_id[pos];
      pos++;
    }
    self_desc[pos++] = ' ';
    self_desc[pos++] = 'S';
    self_desc[pos++] = ':';
    for (i = 0U; (i < PX4LITE_IDENTITY_SN_LEN) && (pos < (uint8_t)(sizeof(self_desc) - 1U)); i++) {
      self_desc[pos++] = serial_number[i];
    }
    self_desc[pos] = '\0';

    RemoteId_FillCharText(s_self_id, (uint8_t)sizeof(s_self_id), self_desc);
  }
  RemoteId_FillCharText(s_operator_id, (uint8_t)sizeof(s_operator_id), PX4LITE_REMOTEID_OPERATOR_ID);

  s_next_heartbeat_ms   = now_ms;
  s_next_basic_id_ms    = now_ms + 50U;
  s_next_location_ms    = now_ms + 100U;
  s_next_system_ms      = now_ms + 150U;
  s_next_operator_id_ms = now_ms + 200U;
  s_next_self_id_ms     = now_ms + 250U;
  s_catalog_index       = 0U;
  return PX4LITE_OK;
}

Px4Lite_Result_t Px4Lite_RemoteIdTxRun(uint32_t now_ms)
{
  const RemoteId_Item_t *item;
  Px4Lite_Result_t result;
  uint8_t checked;

  for (checked = 0U; checked < REMOTEID_CATALOG_COUNT; ++checked) {
    item = &s_remoteid_catalog[s_catalog_index];
    s_catalog_index = (uint8_t)((s_catalog_index + 1U) % REMOTEID_CATALOG_COUNT);

    if (RemoteId_TimeReached(now_ms, *item->next_ms) == 0U) { continue; }

    result = item->encode(now_ms);
    RemoteId_RecordResult(result, now_ms, item);
    if ((result == PX4LITE_OK) || (result == PX4LITE_BUSY) || (result == PX4LITE_IO_ERROR)) { return result; }
  }

  return PX4LITE_IDLE;
}

void Px4Lite_RemoteIdTxGetStats(Px4Lite_RemoteIdTxStats_t *out)
{
  if (out != 0) { *out = s_remoteid_stats; }
}

#else

Px4Lite_Result_t Px4Lite_RemoteIdTxInit(uint32_t now_ms) { (void)now_ms; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RemoteIdTxRun(uint32_t now_ms) { (void)now_ms; return PX4LITE_IDLE; }
void Px4Lite_RemoteIdTxGetStats(Px4Lite_RemoteIdTxStats_t *out) { if (out != 0) { memset(out, 0, sizeof(*out)); } }

#endif
