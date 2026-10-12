/**
 * @file px4lite_comm_config.h
 * @brief Framework 层本机通信地址追加存储与启动身份入口。
 * @details Core 启动加载，CommTask 运行期唯一写者；经 Platform Adapter 访问 Flash，不依赖 SD/VBAT。
 */
#ifndef PX4LITE_COMM_CONFIG_H
#define PX4LITE_COMM_CONFIG_H
#include "px4lite_types.h"
/** @brief 地址保存的真实结果；保存成功仅下次重启生效。 */
typedef enum {
  PX4LITE_CONFIG_IDLE = 0, /**< 未请求保存。 */
  PX4LITE_CONFIG_WRITING, /**< 正在分步追加记录。 */
  PX4LITE_CONFIG_SAVED, /**< 已提交且读回验证通过。 */
  PX4LITE_CONFIG_FULL, /**< 无空白槽，运行期不擦除。 */
  PX4LITE_CONFIG_POWER, /**< 供电或电机维护门禁拒绝。 */
  PX4LITE_CONFIG_ERROR /**< 编程、读取或校验失败。 */
} Px4Lite_CommConfigResult_t;
/** @brief 本机地址配置只读快照。 */
typedef struct {
  uint32_t generation; /**< 保存结果代数，每次请求结束递增。 */
  uint32_t sequence; /**< 最近有效 Flash 记录序号，支持 uint32_t 回绕。 */
  uint16_t used_slots; /**< 已占用槽数上界，范围 0..2048，包含半写槽。 */
  uint8_t current_node_id; /**< 本次启动固定地址，范围 1..250。 */
  uint8_t default_node_id; /**< UID 派生默认地址，范围 1..250。 */
  uint8_t saved_node_id; /**< 最近已保存有效地址，范围 1..250。 */
  uint8_t reboot_pending; /**< 已保存地址与当前地址不同，1 表示需重启。 */
  uint8_t busy; /**< 1 表示持有 Control 维护锁并追加配置。 */
  uint8_t available; /**< 1 表示 UID 与保留区已加载；读取失败时为 0。 */
  Px4Lite_CommConfigResult_t result; /**< 最近请求的真实保存结果。 */
} Px4Lite_CommConfigSnapshot_t;
/** @brief 在任何链路身份初始化前加载，启动阶段可扫描 2048 个固定槽。 */
void Px4Lite_CommConfigInit(void);
/** @brief CommTask 请求保存地址，0 表示恢复 UID 默认；其他合法值为 1..250。 */
Px4Lite_Result_t Px4Lite_CommConfigRequest(uint8_t node_id, uint32_t now_ms);
/** @brief CommTask 每周期推进至多一个字编程，禁止在临界区内调用 Flash I/O。 */
void Px4Lite_CommConfigRun(uint32_t now_ms);
/** @brief 复制固定配置快照，不访问 Flash。 */
void Px4Lite_CommConfigCopy(Px4Lite_CommConfigSnapshot_t *out);
#endif
