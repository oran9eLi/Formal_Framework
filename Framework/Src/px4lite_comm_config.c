/**
 * @file px4lite_comm_config.c
 * @brief Framework 层 UID 绑定的 Node ID 追加记录、掉电提交和启动加载。
 * @details 依赖 Platform Adapter、Identity、Control；不包含 BSP/HAL，不在运行期擦 Flash。
 * 记录逐字段小端编码，CRC 和末尾提交字验证后才报告 Saved，UI 只复制已发布快照。
 * 正常主控供电下，保存不依赖 LoRa 在线或动力电源接入；Control 只提供停机维护互锁。
 */
#include "px4lite_comm_config.h"
#include "px4lite_identity.h"
#include "px4lite_platform.h"
#include "px4lite_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#define CONFIG_RECORD_BYTES 64U
#define CONFIG_SLOT_COUNT 2048U
#define CONFIG_MAGIC 0x314E5343UL
#define CONFIG_COMMIT 0x434D4954UL
static Px4Lite_CommConfigSnapshot_t s_state, s_published;
static uint32_t s_uid[3];
static uint8_t s_record[CONFIG_RECORD_BYTES];
static uint8_t s_saved_setting, s_requested_setting, s_word;
static uint32_t s_write_started_ms;
static uint8_t s_word_pending;

/** @brief 显式读取小端 32 位字段，不依赖 C 结构体布局。 */
static uint32_t Config_Get32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U); }
/** @brief 显式写入小端 32 位字段。 */
static void Config_Put32(uint8_t *p, uint32_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8U); p[2]=(uint8_t)(v>>16U); p[3]=(uint8_t)(v>>24U); }
/** @brief 计算 IEEE CRC32，仅覆盖记录头 32 byte。 */
static uint32_t Config_Crc(const uint8_t *p)
{
  uint32_t crc = UINT32_MAX;
  uint8_t i, b;
  for (i=0U; i<32U; i++) { crc ^= p[i]; for (b=0U; b<8U; b++) { crc=(crc>>1U)^((crc&1U)?0xEDB88320UL:0U); } }
  return ~crc;
}
/** @brief 校验本机 UID、格式、CRC 与最终提交标记，半写记录无效。 */
static uint8_t Config_Valid(const uint8_t *p)
{
  return (Config_Get32(p)==CONFIG_MAGIC && Config_Get32(p+4U)==1U &&
    Config_Get32(p+12U)==s_uid[0] && Config_Get32(p+16U)==s_uid[1] && Config_Get32(p+20U)==s_uid[2] &&
    p[24U]<=250U && p[25U]==((p[24U]==0U)?1U:0U) && Config_Get32(p+32U)==Config_Crc(p) &&
    Config_Get32(p+60U)==CONFIG_COMMIT) ? 1U : 0U;
}
/** @brief 发布短快照；临界区内不扫描或编程 Flash。 */
static void Config_Publish(void)
{ taskENTER_CRITICAL(); s_published=s_state; taskEXIT_CRITICAL(); }

/** @brief 启动扫描并固定本次所有链路使用的地址，不恢复实验会话。 */
void Px4Lite_CommConfigInit(void)
{
  uint16_t slot;
  uint8_t i, occupied, found=0U;
  memset(&s_state,0,sizeof(s_state));
  s_saved_setting=0U;
  s_state.default_node_id=Px4Lite_IdentityGetDefaultNodeId();
  s_state.current_node_id=s_state.default_node_id;
  s_state.saved_node_id=s_state.default_node_id;
  if (Px4Lite_PlatformGetHardwareUid(s_uid,3U)!=PX4LITE_OK) { s_state.result=PX4LITE_CONFIG_ERROR; Config_Publish(); return; }
  s_state.available=1U;
  for (slot=0U; slot<CONFIG_SLOT_COUNT; slot++) {
    if (Px4Lite_ConfigStoreRead((uint32_t)slot*CONFIG_RECORD_BYTES,s_record,CONFIG_RECORD_BYTES)!=PX4LITE_OK) { s_state.available=0U; s_state.result=PX4LITE_CONFIG_ERROR; break; }
    occupied=0U;
    for (i=0U;i<CONFIG_RECORD_BYTES;i++) { if(s_record[i]!=0xFFU) { occupied=1U; break; } }
    if(occupied) { s_state.used_slots=(uint16_t)(slot+1U); }
    if(Config_Valid(s_record) && (!found || (int32_t)(Config_Get32(s_record+8U)-s_state.sequence)>0)) {
      found=1U; s_state.sequence=Config_Get32(s_record+8U); s_saved_setting=s_record[24U];
    }
  }
  s_state.saved_node_id=(s_saved_setting==0U)?s_state.default_node_id:s_saved_setting;
  s_state.current_node_id=s_state.saved_node_id;
  (void)Px4Lite_IdentitySetStartupNodeId(s_state.current_node_id);
  Config_Publish();
}
/** @brief 在 CommTask 内准备一条记录，门禁拒绝不触碰 Flash。 */
Px4Lite_Result_t Px4Lite_CommConfigRequest(uint8_t node_id,uint32_t now_ms)
{
  uint8_t i;
  if(node_id>250U) { return PX4LITE_INVALID_PARAM; }
  if(s_state.busy) { return PX4LITE_BUSY; }
  if(!s_state.available) { s_state.result=PX4LITE_CONFIG_ERROR; s_state.generation++; Config_Publish(); return PX4LITE_NOT_READY; }
  if(node_id==s_saved_setting) { s_state.result=PX4LITE_CONFIG_SAVED; s_state.generation++; Config_Publish(); return PX4LITE_OK; }
  if(s_state.used_slots>=CONFIG_SLOT_COUNT) { s_state.result=PX4LITE_CONFIG_FULL; s_state.generation++; Config_Publish(); return PX4LITE_OVERFLOW; }
  if(Px4Lite_ControlBeginConfigMaintenance()!=PX4LITE_OK) { s_state.result=PX4LITE_CONFIG_INTERLOCK; s_state.generation++; Config_Publish(); return PX4LITE_NOT_READY; }
  memset(s_record,0xFF,sizeof(s_record));
  Config_Put32(s_record,CONFIG_MAGIC); Config_Put32(s_record+4U,1U); Config_Put32(s_record+8U,s_state.sequence+1U);
  for(i=0U;i<3U;i++) { Config_Put32(s_record+12U+4U*i,s_uid[i]); }
  s_record[24U]=node_id; s_record[25U]=(node_id==0U)?1U:0U;
  memset(s_record+26U,0,6U); Config_Put32(s_record+32U,Config_Crc(s_record)); Config_Put32(s_record+60U,CONFIG_COMMIT);
  s_requested_setting=node_id; s_word=0U; s_word_pending=0U; s_write_started_ms=now_ms;
  s_state.busy=1U; s_state.result=PX4LITE_CONFIG_WRITING; Config_Publish();
  return PX4LITE_OK;
}
/** @brief 结束追加，失败也跳过受损槽，释放锁后不恢复旧输出。 */
static void Config_Finish(Px4Lite_CommConfigResult_t result)
{
  s_state.used_slots++;
  s_state.busy=0U; s_state.result=result; s_state.generation++;
  Px4Lite_ControlEndConfigMaintenance(); Config_Publish();
}
/** @brief 先收取上一步硬件完成，再检查维护门禁，最后推进一个字。 */
void Px4Lite_CommConfigRun(uint32_t now_ms)
{
  uint8_t verify[CONFIG_RECORD_BYTES];
  Px4Lite_Result_t result;
  if(!s_state.busy) { return; }
  if(s_word_pending) {
    result=Px4Lite_ConfigStoreFinishWord();
    if(result==PX4LITE_BUSY) { return; }
    s_word_pending=0U;
    if(result!=PX4LITE_OK) { Config_Finish(PX4LITE_CONFIG_ERROR); return; }
    s_word++;
  }
  while(s_word<16U && Config_Get32(s_record+(uint16_t)s_word*4U)==UINT32_MAX) { s_word++; }
  if(s_word<16U) {
    if(Px4Lite_ControlCheckConfigMaintenance()!=PX4LITE_OK) { Config_Finish(PX4LITE_CONFIG_INTERLOCK); return; }
    if(Px4Lite_ElapsedMs(now_ms,s_write_started_ms)>2000U) { Config_Finish(PX4LITE_CONFIG_ERROR); return; }
    result=Px4Lite_ConfigStoreProgramWord((uint32_t)s_state.used_slots*CONFIG_RECORD_BYTES+(uint32_t)s_word*4U,Config_Get32(s_record+(uint16_t)s_word*4U));
    if(result==PX4LITE_BUSY) { s_word_pending=1U; return; }
    if(result!=PX4LITE_OK) { Config_Finish(PX4LITE_CONFIG_ERROR); return; }
    s_word++; return;
  }
  if(Px4Lite_ConfigStoreRead((uint32_t)s_state.used_slots*CONFIG_RECORD_BYTES,verify,CONFIG_RECORD_BYTES)!=PX4LITE_OK ||
     memcmp(verify,s_record,CONFIG_RECORD_BYTES)!=0 || !Config_Valid(verify)) { Config_Finish(PX4LITE_CONFIG_ERROR); return; }
  s_saved_setting=s_requested_setting; s_state.sequence=Config_Get32(verify+8U);
  s_state.saved_node_id=(s_saved_setting==0U)?s_state.default_node_id:s_saved_setting;
  s_state.reboot_pending=(s_state.saved_node_id!=s_state.current_node_id)?1U:0U;
  Config_Finish(PX4LITE_CONFIG_SAVED);
}
/** @brief 应用只复制状态，不读 Flash。 */
void Px4Lite_CommConfigCopy(Px4Lite_CommConfigSnapshot_t *out)
{ if(out) { taskENTER_CRITICAL(); *out=s_published; taskEXIT_CRITICAL(); } }
