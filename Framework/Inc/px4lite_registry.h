/**
 * @file px4lite_registry.h
 * @brief Framework 模块注册表和生命周期控制接口。
 *
 * @details
 * 注册表记录每个模块的描述符、生命周期状态和恢复次数。模块初始化、自检、
 * 启动、停止和恢复入口必须通过本文件统一接入，避免业务代码直接调用具体驱动。
 */

#ifndef PX4LITE_REGISTRY_H
#define PX4LITE_REGISTRY_H

#include "px4lite_types.h"

/**
 * @brief 模块生命周期状态。
 */
typedef enum
{
    PX4LITE_LIFECYCLE_EMPTY = 0,    /**< 槽位为空，未注册模块。 */
    PX4LITE_LIFECYCLE_REGISTERED,   /**< 模块已注册但尚未初始化。 */
    PX4LITE_LIFECYCLE_INITIALIZING, /**< 正在执行 init 回调。 */
    PX4LITE_LIFECYCLE_SELF_CHECK,   /**< 正在执行 self_check 回调。 */
    PX4LITE_LIFECYCLE_STARTING,     /**< 正在执行 start 回调。 */
    PX4LITE_LIFECYCLE_RUNNING,      /**< 模块已启动并处于运行态。 */
    PX4LITE_LIFECYCLE_RECOVERING,   /**< 正在执行 recover 回调。 */
    PX4LITE_LIFECYCLE_STOPPED,      /**< 模块已停止。 */
    PX4LITE_LIFECYCLE_FAILED,       /**< 生命周期回调失败。 */
    PX4LITE_LIFECYCLE_DISABLED      /**< 模块被配置关闭。 */
} Px4Lite_LifecycleState_t;

/**
 * @brief 模块生命周期回调函数类型。
 *
 * @return 回调执行结果。
 *
 * @note `recover` 回调只能设置请求标志，不得在 Health task 中访问慢速总线。
 */
typedef Px4Lite_Result_t (*Px4Lite_LifecycleFn_t)(void);

/**
 * @brief 模块静态描述符。
 */
typedef struct
{
    Px4Lite_ModuleId_t module_id;     /**< 模块编号，必须唯一。 */
    const char *name;                 /**< 模块名称，指向静态字符串。 */
    uint8_t enabled;                  /**< 使能标志，0 表示注册后保持 disabled。 */
    uint8_t required;                 /**< 必需模块标志，1 表示影响系统 ready 判定。 */
    Px4Lite_LifecycleFn_t init;       /**< 初始化回调，可为 NULL。 */
    Px4Lite_LifecycleFn_t self_check; /**< 自检回调，可为 NULL。 */
    Px4Lite_LifecycleFn_t start;      /**< 启动回调，可为 NULL。 */
    Px4Lite_LifecycleFn_t stop;       /**< 停止回调，可为 NULL。 */
    Px4Lite_LifecycleFn_t recover;    /**< 恢复请求回调，可为 NULL。 */
} Px4Lite_ModuleDescriptor_t;

/**
 * @brief 模块生命周期运行态。
 */
typedef struct
{
    Px4Lite_LifecycleState_t state; /**< 当前生命周期状态。 */
    Px4Lite_Result_t last_result;   /**< 最近一次生命周期回调结果。 */
    uint32_t state_since_ms;        /**< 进入当前生命周期状态的时间，单位：ms。 */
    uint16_t start_count;           /**< 启动次数。 */
    uint16_t recovery_count;        /**< 恢复请求次数。 */
} Px4Lite_ModuleRuntime_t;

/**
 * @brief 复位模块注册表和生命周期运行态。
 *
 * @return 初始化结果。
 * @retval PX4LITE_OK 初始化成功。
 *
 * @note 应在注册任何模块前调用。
 */
Px4Lite_Result_t Px4Lite_RegistryInit(void);

/**
 * @brief 在调度器启动前注册一个模块描述符。
 *
 * @param[in] descriptor 模块描述符，不能为 NULL，且 `module_id` 必须合法。
 *
 * @return 注册结果。
 * @retval PX4LITE_OK 注册成功。
 * @retval PX4LITE_INVALID_PARAM 描述符为空或模块编号非法。
 */
Px4Lite_Result_t Px4Lite_RegistryRegister(const Px4Lite_ModuleDescriptor_t *descriptor);

/**
 * @brief 按 init、self_check、start 顺序启动一个模块。
 *
 * @param[in] module_id 模块编号。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 启动结果。
 */
Px4Lite_Result_t Px4Lite_RegistryStart(Px4Lite_ModuleId_t module_id, uint32_t now_ms);

/**
 * @brief 执行一个模块注册的 stop 回调。
 *
 * @param[in] module_id 模块编号。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 停止结果。
 */
Px4Lite_Result_t Px4Lite_RegistryStop(Px4Lite_ModuleId_t module_id, uint32_t now_ms);

/**
 * @brief 执行模块 recover 回调并更新生命周期运行态。
 *
 * @param[in] module_id 模块编号。
 * @param[in] now_ms 当前系统毫秒时间。
 *
 * @return 恢复请求结果。
 *
 * @note recover 回调只应请求所属 service task 之后完成实际 re-init。
 */
Px4Lite_Result_t Px4Lite_RegistryRecover(Px4Lite_ModuleId_t module_id, uint32_t now_ms);

/**
 * @brief 复制一个模块描述符和生命周期状态。
 *
 * @param[in] module_id 模块编号。
 * @param[out] descriptor 描述符输出缓冲区，可为 NULL。
 * @param[out] runtime 运行态输出缓冲区，可为 NULL。
 *
 * @return 查询结果。
 */
Px4Lite_Result_t Px4Lite_RegistryGet(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleDescriptor_t *descriptor, Px4Lite_ModuleRuntime_t *runtime);

#endif
