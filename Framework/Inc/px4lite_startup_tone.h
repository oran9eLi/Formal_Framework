/**
 * @file px4lite_startup_tone.h
 * @brief Framework层CNS动作、启动旋律、渐变、命令结果及模块连接声光的单owner服务。
 * @details sensor是唯一写者；通过Platform输出，不包含BSP/HAL。
 *          调用者只登记有界结果事件；所有硬件输出仍由sensor推进，不替代物理动作验收。
 */
#ifndef PX4LITE_STARTUP_TONE_H
#define PX4LITE_STARTUP_TONE_H
#include "px4lite_types.h"

/** @brief 试听状态，只描述软件播放过程，不表示蜂鸣器在位或实际发声。 */
typedef enum {
  PX4LITE_TONE_DISABLED = 0, /**< 构建未启用。 */
  PX4LITE_TONE_WAITING, /**< 硬件就绪，等待启动延时。 */
  PX4LITE_TONE_PLAYING, /**< 正在旋律时间窗内，含音间间隙。 */
  PX4LITE_TONE_DONE, /**< 启动音已结束且不重播；后续独立结果提示可临时占用声光。 */
  PX4LITE_TONE_ERROR /**< 初始化/输出失败，已撤收且不自动重播。 */
} Px4Lite_StartupToneState_t;

/** @brief 共享声光播放器事件类型；模块连接事实不表示命令执行结果。 */
typedef enum {
  PX4LITE_FEEDBACK_COMMAND_FAILED = 0, /**< 命令未执行/失败，低音三音。 */
  PX4LITE_FEEDBACK_COMMAND_SUCCEEDED = 1, /**< 命令执行成功，高音四音。 */
  PX4LITE_FEEDBACK_MODULE_CONNECTED = 2, /**< 模块连接恢复，上扬双音。 */
  PX4LITE_FEEDBACK_MODULE_DISCONNECTED = 3, /**< 模块连接中断，下行双音。 */
  PX4LITE_FEEDBACK_NONE = 255 /**< 尚无事件，不能作为任一命令结果解读。 */
} Px4Lite_FeedbackKind_t;

/** @brief CNS动作声光编号；不依赖Control类型，避免跨模块循环包含。 */
typedef enum {
  PX4LITE_CNS_TONE_ARM = 1, /**< 解锁前低音，有限段。 */
  PX4LITE_CNS_TONE_TAKEOFF = 2, /**< 起飞前高音三声，有限段。 */
  PX4LITE_CNS_TONE_LAND = 3, /**< 降落期间高音间歇循环，直到取消。 */
  PX4LITE_CNS_TONE_DISARM = 4, /**< 上锁后高音，有限段。 */
  PX4LITE_CNS_TONE_ESTOP = 5, /**< 急停后高音两声，有限段。 */
  PX4LITE_CNS_TONE_SET_MOTOR_PWM = 6 /**< 调速前成功四音及绿青闪灯，有限段。 */
} Px4Lite_CnsToneAction_t;

/** @brief CNS声光软件状态；完成并不证明蜂鸣器或灯珠实体在位。 */
typedef enum {
  PX4LITE_CNS_TONE_IDLE = 0, /**< 无当前请求或已取消。 */
  PX4LITE_CNS_TONE_PENDING, /**< 请求已登记，等待sensor owner。 */
  PX4LITE_CNS_TONE_PLAYING, /**< owner正在推进有限段或循环。 */
  PX4LITE_CNS_TONE_DONE, /**< 当前token有限段已完成。 */
  PX4LITE_CNS_TONE_ERROR /**< 当前token真实硬件操作失败。 */
} Px4Lite_CnsToneState_t;

/** @brief 一次性播放诊断快照；时间为平台微秒，不是RTOS tick。 */
typedef struct {
  uint32_t start_us; /**< 初始化请求时刻，单位us，可回绕。 */
  uint32_t error_count; /**< 硬件初始化或输出失败累计次数。 */
  Px4Lite_Result_t result; /**< 最近硬件结果。 */
  uint16_t frequency_hz; /**< 最近软件请求的频率，0为静音，单位Hz。 */
  uint8_t state; /**< Px4Lite_StartupToneState_t。 */
  uint8_t note_index; /**< 0～16为当前音符，255为无音符。 */
  uint32_t rgb_error_count; /**< RGB初始化、DMA或超时失败累计次数，不计资源忙。 */
  uint32_t rgb_recovery_count; /**< 首次失败后尝试重新初始化的次数。 */
  Px4Lite_Result_t rgb_result; /**< 最近RGB服务结果；不改变蜂鸣器result。 */
  uint8_t rgb_ready; /**< 1表示原始输出初始化成功，不代表灯珠在位。 */
  uint8_t rgb_busy; /**< 1表示最近提交的帧尚待DMA完成确认。 */
  uint8_t rgb_red; /**< 最近成功入发送流程的红通道，0～255。 */
  uint8_t rgb_green; /**< 最近成功入发送流程的绿通道，0～255。 */
  uint8_t rgb_blue; /**< 最近成功入发送流程的蓝通道，0～255。 */
  uint32_t feedback_received; /**< 上电累计四类事件登记次数，允许自然回绕；含队满丢弃，不是命令总数。 */
  uint32_t feedback_started; /**< 已开始的四类软件旋律累计次数，不保证硬件在位。 */
  uint32_t feedback_finished; /**< 旋律及尾静音结束次数，含硬件失败，不代表实物声光验收通过。 */
  uint32_t feedback_dropped; /**< 待播队列满时丢弃的提示数，不影响指令收发/执行。 */
  uint32_t feedback_buzzer_errors; /**< 提示期间蜂鸣器初始化/输出失败次数。 */
  Px4Lite_Result_t feedback_buzzer_result; /**< 最近提示蜂鸣器结果，独立于启动音result。 */
  uint8_t feedback_pending; /**< 最近发布时待播数，0～配置上限（默认8），不含当前提示。 */
  uint8_t feedback_active; /**< 正在软件结果旋律时间窗为1，不代表已实际发声。 */
  uint8_t feedback_succeeded; /**< 仅kind为命令类型时表示1成功/0失败；模块事件恒0，须先检查kind。 */
  uint8_t feedback_kind; /**< 当前或最近事件Px4Lite_FeedbackKind_t，255表示无事件；仅软件播放诊断。 */
  uint32_t music_remaining_us; /**< 当前背景曲谱相位剩余时长，单位us；停机/提示抢占保持。 */
  uint32_t music_error_count; /**< 背景音乐蜂鸣器初始化/输出实际失败累计次数，不含退避等待。 */
  Px4Lite_Result_t music_result; /**< 背景音乐最近独立输出结果；不覆盖命令/模块提示结果。 */
  uint16_t music_phase_index; /**< 用户曲谱当前相位索引，含静音相位；恢复不跳回首音。 */
  uint8_t music_ready; /**< 1为编译期曲谱已通过频率/时长校验，不代表蜂鸣器实体在位。 */
  uint8_t music_gate; /**< 实际电机快照、控制状态与人工锁共同允许播放为1。 */
  uint8_t music_running; /**< 本轮背景时间轴取得蜂鸣器所有权为1，休止相位也为1；不是电机转速。 */
  uint32_t cns_token; /**< 最新CNS请求标识，0为无请求；旧token不能复活动作。 */
  uint32_t cns_error_count; /**< CNS期间真实输出失败的累计次数；不含等待或取消。 */
  Px4Lite_Result_t cns_result; /**< BUSY待播/播放、OK有限段完成、IO_ERROR失败、IDLE取消。 */
  uint8_t cns_action; /**< 当前CNS动作编号1～6，0表示无当前请求。 */
  uint8_t cns_state; /**< Px4Lite_CnsToneState_t，仅软件过程诊断。 */
} Px4Lite_StartupToneStatus_t;

/** @brief sensor启动时调用一次；now_us为平台微秒，不等待旋律。 */
void Px4Lite_StartupToneInit(uint32_t now_us);
/** @brief sensor每10ms推进；CNS最高优先，其他音源暂停续播，RGB按各源策略运行。 */
void Px4Lite_StartupToneRun(uint32_t now_us);
/**
 * @brief 任务上下文登记一次树莓派命令执行结果，短临界区写有界FIFO，不访问硬件。
 * @param[in] succeeded 仅1表示调用者已确认执行成功；0及其它值按未执行/失败播放。
 * @details 协议、地址和最终执行结果均由调用者判定，异步接受不能冒充完成。
 *          启动旋律期间延后；待播上限8，满时只累计dropped；禁用时无操作。
 *          不在ISR调用，Init不会清掉已登记结果；软件播放不等于实体声光有效。
 */
void Px4Lite_StartupToneNotifyRpiResult(uint8_t succeeded);
/**
 * @brief 任务上下文登记一次已确认的模块连接边沿，只入共享FIFO，不访问硬件。
 * @param[in] connected 仅1为稳定连接恢复/插入，其余值为连接中断/移除。
 * @details 调用者负责检测、防抖及启动基线；软件连接状态不证明物理拔插。
 *          独立模块开关关闭时无操作；与命令事件共用8项上限并保留入队顺序。
 */
void Px4Lite_StartupToneNotifyModuleConnection(uint8_t connected);
/** @brief 旧收包接口兼容入口：仅登记低音未执行/失败提示；生产路径必须使用结果接口。 */
void Px4Lite_StartupToneNotifyRpiCommand(void);
/**
 * @brief 仅登记一个高优先级CNS动作；不访问硬件、不等待声音结束。
 * @param[in] action Px4Lite_CnsToneAction_t，取值1～6。
 * @param[in] token 每次动作独立的非零标识；调用者负责不复用仍可能被观察的token。
 * @details 新请求替代旧CNS；普通请求暂停原启动/事件/背景时间轴。sensor推进输出。
 */
void Px4Lite_StartupToneCnsStart(uint8_t action, uint32_t token);
/**
 * @brief 仅登记取消；旧token立即不可查询为完成，硬件由下一轮sensor撤收。
 * @param[in] all 0仅取消CNS；非0同时清除generic FIFO/当前提示并终止旧启动旋律。
 * @details all清理请求具有粘性，紧接着登记ESTOP仍会先清旧播放源；不操作电机。
 */
void Px4Lite_StartupToneCnsCancel(uint8_t all);
/**
 * @brief 查询当前非零token的声光结果，不访问硬件。
 * @retval PX4LITE_BUSY 已登记或播放中，包括LAND循环。
 * @retval PX4LITE_OK 有限段及尾静音完成。
 * @retval PX4LITE_IO_ERROR 当前请求真实输出失败；不得据此继续后续电机动作。
 * @retval PX4LITE_IDLE 已取消、非当前token、token0或CNS功能未启用。
 */
Px4Lite_Result_t Px4Lite_StartupToneCnsResult(uint32_t token);
/** @brief 复制软件诊断快照，out不能为空；调用方不得据此认定电气验收通过。 */
Px4Lite_Result_t Px4Lite_StartupToneCopyStatus(Px4Lite_StartupToneStatus_t *out);
#endif
