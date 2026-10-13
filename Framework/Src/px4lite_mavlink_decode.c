/**
 * @file px4lite_mavlink_decode.c
 * @brief Framework课程字段解码；由Tools/generate_m5_decode.py按本仓库线上offset生成。
 * @details 静态只读目录；对稳定原始帧逐字段读小端，截断零尾补零，不解析C结构体内存。
 *          签名字节存在不表示签名认证通过；不调用Driver/Business/控制/生产解析器。
 */
#include "px4lite_mavlink_decode.h"
#include <stdio.h>
#include <string.h>
#include <float.h>
/** @brief 线上标量类型；CHAR/BYTES为有界数组。 */
typedef enum { DQ_U8=0, DQ_I8, DQ_U16, DQ_I16, DQ_U32, DQ_I32, DQ_U64, DQ_FLOAT, DQ_CHAR, DQ_BYTES } DecodeKind_t;
/** @brief 线上字段描述，不使用宿主offsetof或packed结构体。 */
typedef struct {
  const char *name; /**< 常量字段名。 */
  const char *unit; /**< MAVLink线上原始单位。 */
  uint16_t offset; /**< payload内字节偏移。 */
  uint8_t kind; /**< DecodeKind。 */
  uint8_t count; /**< 0标量，非零数组长度。 */
  uint8_t unknown; /**< 0无/1 uint16max/2 signed-1/3 int16max/4 uint8max/5零未提供/6 GPS yaw双哨兵/7零不支持。 */
} DecodeField_t;
/** @brief 课程消息目录，其他合法消息保留Raw并显示Unsupported。 */
typedef struct {
  uint32_t id; /**< 消息号。 */
  const char *name; /**< 常量消息名。 */
  const DecodeField_t *fields; /**< 静态字段数组。 */
  uint8_t count; /**< 字段数。 */
} DecodeMessage_t;
/** @brief HEARTBEAT字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_heartbeat[] = {
  {"type", "", 4U, DQ_U8, 0U, 0U},
  {"autopilot", "", 5U, DQ_U8, 0U, 0U},
  {"base_mode", "", 6U, DQ_U8, 0U, 0U},
  {"custom_mode", "", 0U, DQ_U32, 0U, 0U},
  {"system_status", "", 7U, DQ_U8, 0U, 0U},
  {"mavlink_version", "", 8U, DQ_U8, 0U, 0U},
};

/** @brief SYS_STATUS字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_sys_status[] = {
  {"onboard_control_sensors_present", "bits", 0U, DQ_U32, 0U, 0U},
  {"onboard_control_sensors_enabled", "bits", 4U, DQ_U32, 0U, 0U},
  {"onboard_control_sensors_health", "bits", 8U, DQ_U32, 0U, 0U},
  {"load", "0.1%", 12U, DQ_U16, 0U, 0U},
  {"voltage_battery", "mV", 14U, DQ_U16, 0U, 1U},
  {"current_battery", "cA", 16U, DQ_I16, 0U, 2U},
  {"battery_remaining", "%", 30U, DQ_I8, 0U, 2U},
  {"drop_rate_comm", "0.01%", 18U, DQ_U16, 0U, 0U},
  {"errors_comm", "", 20U, DQ_U16, 0U, 0U},
  {"errors_count1", "", 22U, DQ_U16, 0U, 0U},
  {"errors_count2", "", 24U, DQ_U16, 0U, 0U},
  {"errors_count3", "", 26U, DQ_U16, 0U, 0U},
  {"errors_count4", "", 28U, DQ_U16, 0U, 0U},
  {"onboard_control_sensors_present_extended", "bits", 31U, DQ_U32, 0U, 0U},
  {"onboard_control_sensors_enabled_extended", "bits", 35U, DQ_U32, 0U, 0U},
  {"onboard_control_sensors_health_extended", "bits", 39U, DQ_U32, 0U, 0U},
};

/** @brief GPS_RAW_INT字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_gps_raw_int[] = {
  {"time_usec", "us", 0U, DQ_U64, 0U, 0U},
  {"fix_type", "", 28U, DQ_U8, 0U, 0U},
  {"lat", "degE7", 8U, DQ_I32, 0U, 0U},
  {"lon", "degE7", 12U, DQ_I32, 0U, 0U},
  {"alt", "mm", 16U, DQ_I32, 0U, 0U},
  {"eph", "DOPx100", 20U, DQ_U16, 0U, 1U},
  {"epv", "DOPx100", 22U, DQ_U16, 0U, 1U},
  {"vel", "cm/s", 24U, DQ_U16, 0U, 1U},
  {"cog", "cdeg", 26U, DQ_U16, 0U, 1U},
  {"satellites_visible", "", 29U, DQ_U8, 0U, 4U},
  {"alt_ellipsoid", "mm", 30U, DQ_I32, 0U, 0U},
  {"h_acc", "mm", 34U, DQ_U32, 0U, 0U},
  {"v_acc", "mm", 38U, DQ_U32, 0U, 0U},
  {"vel_acc", "mm/s", 42U, DQ_U32, 0U, 0U},
  {"hdg_acc", "degE5", 46U, DQ_U32, 0U, 0U},
  {"yaw", "cdeg", 50U, DQ_U16, 0U, 6U},
};

/** @brief ATTITUDE字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_attitude[] = {
  {"time_boot_ms", "ms", 0U, DQ_U32, 0U, 0U},
  {"roll", "rad", 4U, DQ_FLOAT, 0U, 0U},
  {"pitch", "rad", 8U, DQ_FLOAT, 0U, 0U},
  {"yaw", "rad", 12U, DQ_FLOAT, 0U, 0U},
  {"rollspeed", "rad/s", 16U, DQ_FLOAT, 0U, 0U},
  {"pitchspeed", "rad/s", 20U, DQ_FLOAT, 0U, 0U},
  {"yawspeed", "rad/s", 24U, DQ_FLOAT, 0U, 0U},
};

/** @brief GLOBAL_POSITION_INT字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_global_position_int[] = {
  {"time_boot_ms", "ms", 0U, DQ_U32, 0U, 0U},
  {"lat", "degE7", 4U, DQ_I32, 0U, 0U},
  {"lon", "degE7", 8U, DQ_I32, 0U, 0U},
  {"alt", "mm", 12U, DQ_I32, 0U, 0U},
  {"relative_alt", "mm", 16U, DQ_I32, 0U, 0U},
  {"vx", "cm/s", 20U, DQ_I16, 0U, 0U},
  {"vy", "cm/s", 22U, DQ_I16, 0U, 0U},
  {"vz", "cm/s", 24U, DQ_I16, 0U, 0U},
  {"hdg", "cdeg", 26U, DQ_U16, 0U, 1U},
};

/** @brief SCALED_PRESSURE字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_scaled_pressure[] = {
  {"time_boot_ms", "ms", 0U, DQ_U32, 0U, 0U},
  {"press_abs", "hPa", 4U, DQ_FLOAT, 0U, 0U},
  {"press_diff", "hPa", 8U, DQ_FLOAT, 0U, 0U},
  {"temperature", "cdegC", 12U, DQ_I16, 0U, 0U},
  {"temperature_press_diff", "cdegC", 14U, DQ_I16, 0U, 5U},
};

/** @brief BATTERY_STATUS字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_battery_status[] = {
  {"id", "", 32U, DQ_U8, 0U, 0U},
  {"battery_function", "", 33U, DQ_U8, 0U, 0U},
  {"type", "", 34U, DQ_U8, 0U, 0U},
  {"temperature", "cdegC", 8U, DQ_I16, 0U, 3U},
  {"voltages", "mV", 10U, DQ_U16, 10U, 1U},
  {"current_battery", "cA", 30U, DQ_I16, 0U, 2U},
  {"current_consumed", "mAh", 0U, DQ_I32, 0U, 2U},
  {"energy_consumed", "hJ", 4U, DQ_I32, 0U, 2U},
  {"battery_remaining", "%", 35U, DQ_I8, 0U, 2U},
  {"time_remaining", "s", 36U, DQ_I32, 0U, 5U},
  {"charge_state", "", 40U, DQ_U8, 0U, 0U},
  {"voltages_ext", "mV", 41U, DQ_U16, 4U, 7U},
  {"mode", "", 49U, DQ_U8, 0U, 0U},
  {"fault_bitmask", "bits", 50U, DQ_U32, 0U, 0U},
};

/** @brief SERVO_OUTPUT_RAW字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_servo_output_raw[] = {
  {"time_usec", "us", 0U, DQ_U32, 0U, 0U},
  {"port", "", 20U, DQ_U8, 0U, 0U},
  {"servo1_raw", "us", 4U, DQ_U16, 0U, 0U},
  {"servo2_raw", "us", 6U, DQ_U16, 0U, 0U},
  {"servo3_raw", "us", 8U, DQ_U16, 0U, 0U},
  {"servo4_raw", "us", 10U, DQ_U16, 0U, 0U},
  {"servo5_raw", "us", 12U, DQ_U16, 0U, 0U},
  {"servo6_raw", "us", 14U, DQ_U16, 0U, 0U},
  {"servo7_raw", "us", 16U, DQ_U16, 0U, 0U},
  {"servo8_raw", "us", 18U, DQ_U16, 0U, 0U},
  {"servo9_raw", "us", 21U, DQ_U16, 0U, 0U},
  {"servo10_raw", "us", 23U, DQ_U16, 0U, 0U},
  {"servo11_raw", "us", 25U, DQ_U16, 0U, 0U},
  {"servo12_raw", "us", 27U, DQ_U16, 0U, 0U},
  {"servo13_raw", "us", 29U, DQ_U16, 0U, 0U},
  {"servo14_raw", "us", 31U, DQ_U16, 0U, 0U},
  {"servo15_raw", "us", 33U, DQ_U16, 0U, 0U},
  {"servo16_raw", "us", 35U, DQ_U16, 0U, 0U},
};

/** @brief NAMED_VALUE_INT字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_named_value_int[] = {
  {"time_boot_ms", "ms", 0U, DQ_U32, 0U, 0U},
  {"name", "", 8U, DQ_CHAR, 10U, 0U},
  {"value", "", 4U, DQ_I32, 0U, 0U},
};

/** @brief TUNNEL字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_tunnel[] = {
  {"target_system", "", 2U, DQ_U8, 0U, 0U},
  {"target_component", "", 3U, DQ_U8, 0U, 0U},
  {"payload_type", "", 0U, DQ_U16, 0U, 0U},
  {"payload_length", "", 4U, DQ_U8, 0U, 0U},
  {"payload", "", 5U, DQ_BYTES, 128U, 0U},
};

/** @brief COMMAND_LONG字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_command_long[] = {
  {"target_system", "", 30U, DQ_U8, 0U, 0U},
  {"target_component", "", 31U, DQ_U8, 0U, 0U},
  {"command", "", 28U, DQ_U16, 0U, 0U},
  {"confirmation", "", 32U, DQ_U8, 0U, 0U},
  {"param1", "", 0U, DQ_FLOAT, 0U, 0U},
  {"param2", "", 4U, DQ_FLOAT, 0U, 0U},
  {"param3", "", 8U, DQ_FLOAT, 0U, 0U},
  {"param4", "", 12U, DQ_FLOAT, 0U, 0U},
  {"param5", "", 16U, DQ_FLOAT, 0U, 0U},
  {"param6", "", 20U, DQ_FLOAT, 0U, 0U},
  {"param7", "", 24U, DQ_FLOAT, 0U, 0U},
};

/** @brief COMMAND_ACK字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_command_ack[] = {
  {"command", "", 0U, DQ_U16, 0U, 0U},
  {"result", "", 2U, DQ_U8, 0U, 0U},
  {"progress", "%", 3U, DQ_U8, 0U, 4U},
  {"result_param2", "", 4U, DQ_I32, 0U, 0U},
  {"target_system", "", 8U, DQ_U8, 0U, 0U},
  {"target_component", "", 9U, DQ_U8, 0U, 0U},
};

/** @brief OPEN_DRONE_ID_BASIC_ID字段，offset为线上payload字节偏移。 */
static const DecodeField_t s_fields_open_drone_id_basic_id[] = {
  {"target_system", "", 0U, DQ_U8, 0U, 0U},
  {"target_component", "", 1U, DQ_U8, 0U, 0U},
  {"id_or_mac", "", 2U, DQ_BYTES, 20U, 0U},
  {"id_type", "", 22U, DQ_U8, 0U, 0U},
  {"ua_type", "", 23U, DQ_U8, 0U, 0U},
  {"uas_id", "", 24U, DQ_BYTES, 20U, 0U},
};

static const DecodeMessage_t s_catalog[] = {
  {0U, "HEARTBEAT", s_fields_heartbeat, 6U},
  {1U, "SYS_STATUS", s_fields_sys_status, 16U},
  {24U, "GPS_RAW_INT", s_fields_gps_raw_int, 16U},
  {30U, "ATTITUDE", s_fields_attitude, 7U},
  {33U, "GLOBAL_POSITION_INT", s_fields_global_position_int, 9U},
  {29U, "SCALED_PRESSURE", s_fields_scaled_pressure, 5U},
  {147U, "BATTERY_STATUS", s_fields_battery_status, 14U},
  {36U, "SERVO_OUTPUT_RAW", s_fields_servo_output_raw, 18U},
  {252U, "NAMED_VALUE_INT", s_fields_named_value_int, 3U},
  {385U, "TUNNEL", s_fields_tunnel, 5U},
  {76U, "COMMAND_LONG", s_fields_command_long, 11U},
  {77U, "COMMAND_ACK", s_fields_command_ack, 6U},
  {12900U, "OPEN_DRONE_ID_BASIC_ID", s_fields_open_drone_id_basic_id, 6U},
};
/** @brief 零尾补齐；读取8字节以内，offset越过线上payload时返回零。 */
static uint64_t DecodeRead(const uint8_t *payload, uint16_t length, uint16_t offset, uint8_t width)
{
  uint64_t value = 0U;
  uint8_t i;
  for (i = 0U; i < width; i++) { if ((uint16_t)(offset+i) < length) { value |= (uint64_t)payload[offset+i] << (i*8U); } }
  return value;
}
/** @brief 字段宽度，byte；数组文本单独处理。 */
static uint8_t DecodeWidth(uint8_t kind)
{ return kind == DQ_U64 ? 8U : (kind >= DQ_U32 && kind <= DQ_FLOAT ? 4U : (kind == DQ_U16 || kind == DQ_I16 ? 2U : 1U)); }
/** @brief 格式化一个标量，合法零不当无效；特殊哨兵标unknown。 */
static void DecodeScalar(char *out, const DecodeField_t *field, uint8_t index, const uint8_t *payload, uint16_t length)
{
  uint8_t width = DecodeWidth(field->kind);
  uint64_t raw = DecodeRead(payload, length, (uint16_t)(field->offset+index*width), width);
  int32_t signed_value = field->kind == DQ_I8 ? (int8_t)raw : (field->kind == DQ_I16 ? (int16_t)raw : (int32_t)raw);
  char label[40];
  char value[32];
  if (field->count) { (void)snprintf(label, sizeof(label), "%s[%u]", field->name, (unsigned int)index); }
  else { (void)snprintf(label, sizeof(label), "%s", field->name); }
  if (((field->unknown == 1U || field->unknown == 6U) && raw == UINT16_MAX) || (field->unknown == 2U && signed_value == -1) || (field->unknown == 3U && raw == INT16_MAX) || (field->unknown == 4U && raw == UINT8_MAX)) {
    (void)snprintf(value, sizeof(value), "unknown");
  } else if ((field->unknown == 5U || field->unknown == 6U) && raw == 0U) {
    (void)snprintf(value, sizeof(value), "not provided");
  } else if (field->unknown == 7U && raw == 0U) {
    (void)snprintf(value, sizeof(value), "unsupported");
  } else if (field->kind == DQ_FLOAT) {
    float f;
    uint32_t bits = (uint32_t)raw;
    memcpy(&f, &bits, sizeof(f));
    if (f <= FLT_MAX && f >= -FLT_MAX) { (void)snprintf(value, sizeof(value), "%.5g", (double)f); }
    else { (void)snprintf(value, sizeof(value), "nonfinite"); }
  } else if (field->kind == DQ_I8 || field->kind == DQ_I16 || field->kind == DQ_I32) { (void)snprintf(value, sizeof(value), "%ld", (long)signed_value); }
  else if (strcmp(field->unit, "bits") == 0) { (void)snprintf(value, sizeof(value), "0x%08lX", (unsigned long)raw); }
  else if (field->kind == DQ_U64) { (void)snprintf(value, sizeof(value), "%llu", (unsigned long long)raw); }
  else { (void)snprintf(value, sizeof(value), "%lu", (unsigned long)raw); }
  (void)snprintf(out, PX4LITE_DECODE_TEXT, "%s: %s %s", label, value, field->unit);
}
/** @brief 有界文本/ID数组；不可打印字节改用HEX，完整值仍可读Raw。 */
static void DecodeArray(char *out, const DecodeField_t *field, const uint8_t *payload, uint16_t length)
{
  uint8_t i, printable = 1U, zero_seen = 0U;
  char text[41];
  uint8_t used = 0U;
  if (field->count > 20U) { (void)snprintf(out, PX4LITE_DECODE_TEXT, "%s[%u]: see Raw", field->name, (unsigned int)field->count); return; }
  for (i = 0U; i < field->count; i++) { uint8_t c = (uint8_t)DecodeRead(payload, length, (uint16_t)(field->offset+i), 1U); if (c && (c<32U || c>126U || zero_seen)) { printable=0U; } if (!c) { zero_seen=1U; } }
  for (i = 0U; i < field->count; i++) {
    uint8_t c = (uint8_t)DecodeRead(payload, length, (uint16_t)(field->offset+i), 1U);
    if (printable) { if (!c) { break; } text[used++]=(char)c; }
    else { (void)snprintf(&text[used], sizeof(text)-used, "%02X", (unsigned int)c); used+=2U; }
  }
  text[used]='\0'; (void)snprintf(out, PX4LITE_DECODE_TEXT, "%s: %s%s", field->name, printable ? "" : "HEX ", used ? text : "(empty)");
}
/** @brief 课程TUNNEL只解释0x8003的真实类型/nonce/长度，不冒称服务端成功。 */
static void DecodeTunnel(const uint8_t *p, uint16_t length, Px4Lite_DecodedPage_t *out)
{
  uint16_t payload_type = (uint16_t)DecodeRead(p,length,0U,2U);
  uint8_t n = (uint8_t)DecodeRead(p,length,4U,1U);
  uint8_t version = (uint8_t)DecodeRead(p,length,5U,1U);
  uint8_t type = (uint8_t)DecodeRead(p,length,6U,1U);
  uint8_t i;
  char nonce[17];
  (void)snprintf(out->rows[0], PX4LITE_DECODE_TEXT, "target_system: %lu", (unsigned long)DecodeRead(p,length,2U,1U));
  (void)snprintf(out->rows[1], PX4LITE_DECODE_TEXT, "target_component: %lu", (unsigned long)DecodeRead(p,length,3U,1U));
  (void)snprintf(out->rows[2], PX4LITE_DECODE_TEXT, "payload_type: 0x%04X / length: %u", (unsigned int)payload_type, (unsigned int)n);
  out->row_count=4U; out->page_count=1U;
  if (payload_type != 0x8003U) { (void)snprintf(out->rows[3], PX4LITE_DECODE_TEXT, "payload: opaque / see Raw"); return; }
  (void)snprintf(out->rows[3], PX4LITE_DECODE_TEXT, "version: %u / type: %u", (unsigned int)version, (unsigned int)type);
  if (n>128U || n<10U || version!=1U || type<1U || type>4U) { (void)snprintf(out->rows[4], PX4LITE_DECODE_TEXT, "Malformed / unsupported H1-B1"); out->row_count=5U; return; }
  for (i=0U;i<8U;i++) { (void)snprintf(&nonce[i*2U],3U,"%02X",(unsigned int)DecodeRead(p,length,(uint16_t)(7U+i),1U)); }
  (void)snprintf(out->rows[4], PX4LITE_DECODE_TEXT, "nonce: %s", nonce);
  (void)snprintf(out->rows[5], PX4LITE_DECODE_TEXT, "%s", type==1U ? "H1 request" : (type==2U ? "H1 response" : (type==3U ? "B1 request" : "B1 acceptance")));
  out->row_count=7U;
  if (type<=2U) {
    char text[41];
    uint8_t text_n=(uint8_t)DecodeRead(p,length,15U,1U);
    uint16_t expected=(uint16_t)(11U+text_n+(type==2U?4U:0U));
    if (!text_n || text_n>48U || n!=expected) {
      (void)snprintf(out->rows[6],PX4LITE_DECODE_TEXT,"Malformed H1 length / N must be 1..48"); return;
    }
    for(i=0U;i<text_n && i<40U;i++) { uint8_t c=(uint8_t)DecodeRead(p,length,(uint16_t)(16U+i),1U); text[i]=(c>=32U && c<=126U)?(char)c:'.'; } text[i]='\0';
    (void)snprintf(out->rows[6],PX4LITE_DECODE_TEXT,"text[%u]: %s%s",(unsigned int)text_n,text,text_n>40U?"...":"");
    if (type==2U) {
      (void)snprintf(out->rows[7],PX4LITE_DECODE_TEXT,"baud: %lu bit/s",(unsigned long)DecodeRead(p,length,(uint16_t)(16U+text_n),4U));
      out->row_count=8U;
    } else if (text_n>40U) { (void)snprintf(out->rows[7],PX4LITE_DECODE_TEXT,"Remaining text: see Raw"); out->row_count=8U; }
  } else {
    if (n!=14U) { (void)snprintf(out->rows[6],PX4LITE_DECODE_TEXT,"Malformed baud payload length"); return; }
    (void)snprintf(out->rows[6],PX4LITE_DECODE_TEXT,"baud: %lu bit/s",(unsigned long)DecodeRead(p,length,15U,4U));
  }
}
/** @brief 静态目录逐页解码，签名和CRC字节仍以原始页为证据。 */
Px4Lite_Result_t Px4Lite_DecodeFrame(const Px4Lite_CaptureFrame_t *frame, uint8_t page, Px4Lite_DecodedPage_t *out)
{
  const DecodeMessage_t *message=0;
  const uint8_t *payload;
  uint16_t length, total=0U, ordinal=0U, first;
  uint8_t i,j,header;
  if (!frame || !out || frame->length<8U || frame->length>PX4LITE_CAPTURE_FRAME_BYTES) { return PX4LITE_INVALID_PARAM; }
  memset(out,0,sizeof(*out));
  header=frame->bytes[0]==0xFD?10U:6U; length=frame->bytes[1];
  if ((frame->bytes[0]!=0xFD && frame->bytes[0]!=0xFE) || (uint16_t)(header+length+2U)>frame->length) { return PX4LITE_INVALID_PARAM; }
  payload=&frame->bytes[header];
  for (i=0U;i<sizeof(s_catalog)/sizeof(s_catalog[0]);i++) { if(s_catalog[i].id==frame->msg_id) { message=&s_catalog[i];break; } }
  if(!message) { (void)snprintf(out->name,sizeof(out->name),"Unsupported"); (void)snprintf(out->rows[0],PX4LITE_DECODE_TEXT,"msg %lu / see Raw",(unsigned long)frame->msg_id); out->row_count=1U;out->page_count=1U;return PX4LITE_OK; }
  (void)snprintf(out->name,sizeof(out->name),"%s",message->name);
  if(frame->msg_id==385U) { DecodeTunnel(payload,length,out);return PX4LITE_OK; }
  for(i=0U;i<message->count;i++) { const DecodeField_t *f=&message->fields[i]; total+=(f->count && f->kind!=DQ_CHAR && f->kind!=DQ_BYTES)?f->count:1U; }
  out->page_count=(uint8_t)((total+PX4LITE_DECODE_ROWS-1U)/PX4LITE_DECODE_ROWS);
  out->page=(uint8_t)(page%out->page_count);first=(uint16_t)(out->page*PX4LITE_DECODE_ROWS);
  for(i=0U;i<message->count;i++) {
    const DecodeField_t *f=&message->fields[i]; uint8_t rows=(f->count && f->kind!=DQ_CHAR && f->kind!=DQ_BYTES)?f->count:1U;
    for(j=0U;j<rows;j++,ordinal++) {
      if(ordinal<first || out->row_count>=PX4LITE_DECODE_ROWS) { continue; }
      if(f->kind==DQ_CHAR || f->kind==DQ_BYTES) { DecodeArray(out->rows[out->row_count],f,payload,length); }
      else { DecodeScalar(out->rows[out->row_count],f,j,payload,length); }
      out->row_count++;
    }
  }
  return PX4LITE_OK;
}
