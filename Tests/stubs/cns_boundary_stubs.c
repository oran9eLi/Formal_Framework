/**
 * @file cns_boundary_stubs.c
 * @brief 主机旧专项未合编的CNS边界隔离；真实被测源的依赖缺席时才启用专用桩。
 * @details 仅用于GCC主机测试，不进入Keil。runner逐个读取实际合编的C源码以选择缺席边界，避免Windows弱符号链接歧义。普通旧测试不启动新动作；请求若意外越界立即失败。
 */
#include <assert.h>
#include "px4lite_control.h"
#include "px4lite_startup_tone.h"
#include "px4lite_command.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_mavlink_rx.h"
#include "px4lite_modules.h"
#include "px4lite_topics.h"
#include "px4lite_platform.h"
#include "px4lite_gnss_raw_uplink.h"
#include "px4lite_gnss_uplink.h"
#include "px4lite_bmi088_uplink.h"
#include "px4lite_lora_lab.h"

/** @brief 未合编Control的专项不得伪造CNS命令执行。 */
#if defined(HOST_CNS_STUB_Px4Lite_ControlCnsRequest)
__attribute__((externally_visible)) Px4Lite_Result_t Px4Lite_ControlCnsRequest(Px4Lite_CnsAction_t action,
    uint8_t motor, uint16_t pulse, uint8_t *reason, uint32_t *token)
{ (void)action; (void)motor; (void)pulse; (void)reason; (void)token; assert(!"CNS request needs a real Control or explicit mock"); return PX4LITE_NOT_READY; }
#endif

/** @brief 旧协议专项未合编模块服务时拒绝意外的启停请求。 */
#if defined(HOST_CNS_STUB_Px4Lite_ModuleExperimentRequest)
__attribute__((externally_visible)) Px4Lite_Result_t Px4Lite_ModuleExperimentRequest(Px4Lite_ModuleId_t id,
    uint8_t enabled, uint32_t now_ms)
{ (void)id; (void)enabled; (void)now_ms; assert(!"Module switch needs a real service or explicit mock"); return PX4LITE_NOT_READY; }
#endif

/** @brief 需要观察切屏释放的专项应提供真实服务或显式替身。 */
#if defined(HOST_CNS_STUB_Px4Lite_ModuleExperimentAck)
__attribute__((externally_visible)) void Px4Lite_ModuleExperimentAck(Px4Lite_ModuleId_t id, uint8_t enabled)
{ (void)id; (void)enabled; assert(!"Module ACK needs a real service or explicit mock"); }
#endif

/** @brief 不合编模块服务的旧遥测专项继续使用自己的编译配置。 */
#if defined(HOST_CNS_STUB_Px4Lite_ModuleExperimentEnabled)
__attribute__((externally_visible)) uint8_t Px4Lite_ModuleExperimentEnabled(Px4Lite_ModuleId_t id)
{ (void)id; return 1U; }
#endif

/** @brief 旧专项不得越过缺席的模块启停边界。 */
#if defined(HOST_CNS_STUB_Px4Lite_ControlModulePauseAllowed)
__attribute__((externally_visible)) uint8_t Px4Lite_ControlModulePauseAllowed(Px4Lite_ModuleId_t id)
{ (void)id; assert(!"Module pause requires the real interlock or an explicit mock"); return 0U; }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_InvalidateSensorTopics)
__attribute__((externally_visible)) void Px4Lite_InvalidateSensorTopics(Px4Lite_ModuleId_t id)
{ (void)id; assert(!"Paused sensor topics require a real topic store or an explicit mock"); }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_Bmi088UplinkInvalidate)
__attribute__((externally_visible)) void Px4Lite_Bmi088UplinkInvalidate(void)
{ assert(!"Paused IMU uplink requires a real uplink or explicit mock"); }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_GnssRawUplinkDiscard)
__attribute__((externally_visible)) void Px4Lite_GnssRawUplinkDiscard(void)
{ assert(!"Paused GNSS raw uplink requires a real uplink or explicit mock"); }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_GnssUplinkDiscard)
__attribute__((externally_visible)) void Px4Lite_GnssUplinkDiscard(uint32_t now_ms)
{ (void)now_ms; assert(!"Paused GNSS uplink requires a real uplink or explicit mock"); }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_GnssSetExperimentEnabled)
__attribute__((externally_visible)) Px4Lite_Result_t Px4Lite_GnssSetExperimentEnabled(uint8_t enabled)
{ (void)enabled; assert(!"GNSS transition requires a real platform or explicit mock"); return PX4LITE_NOT_READY; }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_LoRaPause)
__attribute__((externally_visible)) void Px4Lite_LoRaPause(void)
{ assert(!"LoRa pause requires a real platform or explicit mock"); }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_MavlinkRxResetLoRa)
__attribute__((externally_visible)) void Px4Lite_MavlinkRxResetLoRa(void)
{ assert(!"LoRa reset requires a real receiver or explicit mock"); }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_ImuInit)
__attribute__((externally_visible)) Px4Lite_Result_t Px4Lite_ImuInit(void)
{ assert(!"IMU initialization requires a real platform or explicit mock"); return PX4LITE_NOT_READY; }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_BaroInit)
__attribute__((externally_visible)) Px4Lite_Result_t Px4Lite_BaroInit(void)
{ assert(!"Baro initialization requires a real platform or explicit mock"); return PX4LITE_NOT_READY; }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_LoraLabRequest)
__attribute__((externally_visible)) Px4Lite_Result_t Px4Lite_LoraLabRequest(const Px4Lite_LabRequest_t *request)
{ (void)request; assert(!"LoRa lab request requires a real service or explicit mock"); return PX4LITE_NOT_READY; }
#endif
#if defined(HOST_CNS_STUB_Px4Lite_StartupToneNotifyRpiResult)
__attribute__((externally_visible)) void Px4Lite_StartupToneNotifyRpiResult(uint8_t success)
{ (void)success; assert(!"RPi result sound requires a real service or explicit mock"); }
#endif
/** @brief ACK逻辑若需观察Control释放，应由专项显式提供强桩。 */
#if defined(HOST_CNS_STUB_Px4Lite_ControlCnsAckSent)
__attribute__((externally_visible)) void Px4Lite_ControlCnsAckSent(uint32_t token) { (void)token; }
#endif
/** @brief 没有Control事实的遥测专项保持锁定；真实armed由专用协议/集成测试验证。 */
#if defined(HOST_CNS_STUB_Px4Lite_ControlCnsArmed)
__attribute__((externally_visible)) uint8_t Px4Lite_ControlCnsArmed(void) { return 0U; }
#endif
/** @brief 仅有Control的专项由其自身强桩控制声光结果；旧动作不使用此新Start。 */
#if defined(HOST_CNS_STUB_Px4Lite_StartupToneCnsStart)
__attribute__((externally_visible)) void Px4Lite_StartupToneCnsStart(uint8_t action, uint32_t token)
{ (void)action; (void)token; assert(!"CNS sound needs a real player or explicit mock"); }
#endif
/** @brief 缺席播放器时取消不访问硬件；真正取消语义由真实双模块集成验证。 */
#if defined(HOST_CNS_STUB_Px4Lite_StartupToneCnsCancel)
__attribute__((externally_visible)) void Px4Lite_StartupToneCnsCancel(uint8_t all) { (void)all; }
#endif
/** @brief 不以缺席播放器伪造声音完成。 */
#if defined(HOST_CNS_STUB_Px4Lite_StartupToneCnsResult)
__attribute__((externally_visible)) Px4Lite_Result_t Px4Lite_StartupToneCnsResult(uint32_t token)
{ (void)token; return PX4LITE_IDLE; }
#endif
/** @brief 缺席发送器时不允许新动作越过ACK容量门禁。 */
#if defined(HOST_CNS_STUB_Px4Lite_MavlinkCnsAckAvailable)
__attribute__((externally_visible)) uint8_t Px4Lite_MavlinkCnsAckAvailable(void) { return 0U; }
#endif
/** @brief 未合编发送器时不伪造应答排队成功。 */
#if defined(HOST_CNS_STUB_Px4Lite_MavlinkQueueCnsAck)
__attribute__((externally_visible)) Px4Lite_Result_t Px4Lite_MavlinkQueueCnsAck(uint16_t command,
    uint8_t result, uint8_t reason, uint8_t target, uint32_t token, uint32_t now)
{ (void)command; (void)result; (void)reason; (void)target; (void)token; (void)now; return PX4LITE_NOT_READY; }
#endif
/** @brief 独立RX旧专项不借缺席执行器做CNS受理；完整新入口有专用真实测试。 */
#if defined(HOST_CNS_STUB_Px4Lite_CommandHandleRpiLongV01)
__attribute__((externally_visible)) Px4Lite_Result_t Px4Lite_CommandHandleRpiLongV01(uint16_t command,
    uint8_t source, uint8_t comp, uint8_t target, uint8_t target_comp,
    uint8_t confirmation, const float params[7], uint32_t now)
{ (void)command; (void)source; (void)comp; (void)target; (void)target_comp; (void)confirmation; (void)params; (void)now; return PX4LITE_IDLE; }
#endif
