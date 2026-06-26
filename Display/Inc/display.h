#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

typedef enum {
  DISPLAY_OK = 0,    /* 显示模块正常 */
  DISPLAY_NOT_READY, /* HMI 或显示链路未就绪 */
  DISPLAY_ERROR      /* 显示模块发生错误 */
} Display_Result_t;

typedef enum {
  DISPLAY_HMI_ACCESS_RO = 0, /* HMI 只读变量，由主控刷新到屏幕*/
  DISPLAY_HMI_ACCESS_RW      /* HMI 可写变量，作为参数或命令入口 */
} Display_HmiAccess_t;

typedef enum {
  DISPLAY_HMI_TYPE_U16 = 0, /* 16 位无符号变量 */
  DISPLAY_HMI_TYPE_I16,     /* 16 位有符号变量 */
  DISPLAY_HMI_TYPE_U32,     /* 32 位无符号变量 */
  DISPLAY_HMI_TYPE_I32      /* 32 位有符号变量 */
} Display_HmiDataType_t;

typedef enum {
  DISPLAY_HMI_PAGE_LOGO = 0,   /* Logo 开机页：触屏任意位置进入自检页 */
  DISPLAY_HMI_PAGE_SELF_CHECK, /* 上电自检页：进度、模块状态和故障码 */
  DISPLAY_HMI_PAGE_FLIGHT,     /* 飞行数据页：系统栏、时间/电源/温湿度气压、消息日志*/
  DISPLAY_HMI_PAGE_AIRCRAFT,   /* 飞机情况页：系统栏、姿态/LoRa 通信、消息日志*/
  DISPLAY_HMI_PAGE_DATA,       /* 定位数据页：系统、GNSS 定位、消息日志*/
  DISPLAY_HMI_PAGE_MOTOR,      /* 电机控制页：四路油门滑条和急停入口 */
  DISPLAY_HMI_PAGE_ALARM,      /* 告警页：运行期告警码和原因表*/
  DISPLAY_HMI_PAGE_HIDDEN,     /* 通信连接页：触屏进入，用于远端节点选择和查看 */
  DISPLAY_HMI_PAGE_COUNT       /* 页面数量 */
} Display_HmiPage_t;

typedef enum {
  DISPLAY_DATA_SOURCE_LOCAL = 0, /* 当前页面显示本机 App 数据视图 */
  DISPLAY_DATA_SOURCE_REMOTE     /* 当前页面显示对端 App 数据视图 */
} Display_DataSource_t;

#define DISPLAY_MOTOR_TRACK_W       30U
#define DISPLAY_MOTOR_TRACK_H       166U
#define DISPLAY_MOTOR_TRACK_TOP_Y   144U
#define DISPLAY_MOTOR_TRACK1_X      270U
#define DISPLAY_MOTOR_TRACK2_X      355U
#define DISPLAY_MOTOR_TRACK3_X      440U
#define DISPLAY_MOTOR_TRACK4_X      525U
#define DISPLAY_MOTOR_HANDLE_HALF_W 22U
#define DISPLAY_MOTOR_HANDLE_HALF_H 7U
#define DISPLAY_MOTOR_ESTOP_X       250U
#define DISPLAY_MOTOR_ESTOP_Y       344U
#define DISPLAY_MOTOR_ESTOP_W       328U
#define DISPLAY_MOTOR_ESTOP_H       52U

typedef enum {
  DISPLAY_HMI_VAR_SELF_CHECK_5GA = 0,    /* 5G-A 连接管理状态灯；不表示 MCU 承载 5G/Remote ID 数据流 */
  DISPLAY_HMI_VAR_SELF_CHECK_MPU6050,    /* MPU6050 姿态传感器自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_BME280,     /* BME280 环境传感器自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_SD,         /* SD 卡存储自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_MOTOR,      /* 电机 1 驱动自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_2,    /* 电机 2 驱动自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_3,    /* 电机 3 驱动自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_MOTOR_4,    /* 电机 4 驱动自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_POWER,      /* 电源采样自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_BUZZER,     /* 蜂鸣器自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_KEY,        /* 按键自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_DEBUG,      /* 调试接口自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_GNSS,       /* GNSS 自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_LORA,       /* LoRa 自检状态*/
  DISPLAY_HMI_VAR_SELF_CHECK_ERROR_CODE, /* 上电自检故障码*/
  DISPLAY_HMI_VAR_DATA_SELFCHECK_RESULT, /* 自检结果位图 (uint32) */
  DISPLAY_HMI_VAR_SYSTEM_STATUS,         /* 系统总状态，来源 CNS_State.system */
  DISPLAY_HMI_VAR_UPTIME_MS,             /* 系统运行时间，单位 ms */
  DISPLAY_HMI_VAR_BATTERY_VOLTAGE,       /* 电池电压，单位 0.01V */
  DISPLAY_HMI_VAR_BATTERY_PERCENT,       /* 电量百分比，单位 % */
  DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE,     /* 电机电池电压，单位 0.01V；未接入独立采样时跟随主电池 */
  DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT,     /* 电机电池电量，单位 %；未接入独立采样时跟随主电池 */
  DISPLAY_HMI_VAR_GNSS_FIX,              /* GNSS 定位状态*/
  DISPLAY_HMI_VAR_GNSS_SAT_COUNT,        /* GNSS 使用卫星数量 */
  DISPLAY_HMI_VAR_GNSS_HDOP,             /* GNSS HDOP，单位 0.01 */
  DISPLAY_HMI_VAR_LATITUDE,              /* 纬度，单位 1e-7 度*/
  DISPLAY_HMI_VAR_LONGITUDE,             /* 经度，单位 1e-7 度*/
  DISPLAY_HMI_VAR_ALTITUDE,              /* 高度，单位 mm */
  DISPLAY_HMI_VAR_GNSS_SPEED,            /* GNSS 地速，单位 0.01m/s */
  DISPLAY_HMI_VAR_GNSS_TIME,             /* 本地显示时间，格式 HHMMSS */
  DISPLAY_HMI_VAR_ROLL,                  /* 横滚角，单位 0.1 度*/
  DISPLAY_HMI_VAR_PITCH,                 /* 俯仰角，单位 0.1 度*/
  DISPLAY_HMI_VAR_YAW,                   /* 偏航角，单位 0.1 度*/
  DISPLAY_HMI_VAR_TEMPERATURE,           /* 温度，单位 0.1 摄氏度*/
  DISPLAY_HMI_VAR_HUMIDITY,              /* 湿度，单位 0.1%RH */
  DISPLAY_HMI_VAR_PRESSURE,              /* 气压，单位 Pa */
  DISPLAY_HMI_VAR_LORA_STATUS,           /* LoRa 在线或链路状态*/
  DISPLAY_HMI_VAR_LORA_TX_COUNT,         /* LoRa 发送计数*/
  DISPLAY_HMI_VAR_LORA_RX_COUNT,         /* LoRa 接收计数 */
  DISPLAY_HMI_VAR_LORA_HEARTBEAT,        /* LoRa 心跳状态*/
  DISPLAY_HMI_VAR_LORA_ACK_COUNT,        /* LoRa ACK 计数 */
  DISPLAY_HMI_VAR_LORA_LOSS_RATE,        /* LoRa 丢包率，单位 0.1% */
  DISPLAY_HMI_VAR_ALARM_CODE,            /* 当前告警码*/
  DISPLAY_HMI_VAR_MOTOR_PWM_1,           /* 电机 1 PWM 命令入口，单位 % */
  DISPLAY_HMI_VAR_MOTOR_PWM_2,           /* 电机 2 PWM 命令入口，单位 % */
  DISPLAY_HMI_VAR_MOTOR_PWM_3,           /* 电机 3 PWM 命令入口，单位 % */
  DISPLAY_HMI_VAR_MOTOR_PWM_4,           /* 电机 4 PWM 命令入口，单位 % */
  DISPLAY_HMI_VAR_ALARM_ACTIVE_MASK,     /* 当前激活告警位图*/
  DISPLAY_HMI_VAR_ALARM_ROW1_CODE,       /* 告警表第 1 行告警码 */
  DISPLAY_HMI_VAR_ALARM_ROW2_CODE,       /* 告警表第 2 行告警码 */
  DISPLAY_HMI_VAR_ALARM_ROW3_CODE,       /* 告警表第 3 行告警码 */
  DISPLAY_HMI_VAR_ALARM_ROW4_CODE,       /* 告警表第 4 行告警码 */
  DISPLAY_HMI_VAR_ALARM_ROW5_CODE,       /* 告警表第 5 行告警码 */
  DISPLAY_HMI_VAR_CLOCK_TIME,            /* 本地显示时间，编码 HHMMSS，由业务层喂值*/
  DISPLAY_HMI_VAR_DATE,                  /* 本地显示日期，编码 YYYYMMDD，由业务层喂值*/
  DISPLAY_HMI_VAR_FLIGHT_TIME_S,         /* 飞行时间(上电后运行)，单位 s */
  DISPLAY_HMI_VAR_MESSAGE_LOG,           /* 消息日志缓冲版本号，变化即重绘日志区 */
  DISPLAY_HMI_VAR_VIEW_NODE_ID,          /* 当前显示对象 node_id，Remote ID 接入前用于临时身份显示 */
  DISPLAY_HMI_VAR_COUNT                  /* HMI 变量数量 */
} Display_HmiVariableId_t;

typedef struct {
  Display_HmiPage_t page; /* Display 内部页面枚举 */
  uint16_t page_id;       /* 固件内部页面 ID */
  const char *name;       /* 页面名称，供调试和文档生成使用*/
  uint16_t refresh_ms;    /* 页面默认刷新周期，0 表示不周期刷新*/
} Display_HmiPageConfig_t;

typedef struct {
  Display_HmiVariableId_t id;      /* Display 内部变量 ID */
  Display_HmiPage_t page;          /* 变量所属页面*/
  uint16_t var_addr;               /* 固件内部变量路由 ID，不是屏幕硬件地址 */
  Display_HmiDataType_t data_type; /* 变量数据类型 */
  Display_HmiAccess_t access;      /* 变量读写属性*/
  uint16_t refresh_ms;             /* 变量刷新周期，0 表示事件触发 */
  uint16_t x;                      /* 横屏显示区域左上角 X */
  uint16_t y;                      /* 横屏显示区域左上角 Y */
  uint16_t width;                  /* 横屏显示区域宽度 */
  uint16_t height;                 /* 横屏显示区域高度 */
  const char *name;                /* 变量名称，供 LVGL、调试和文档生成使用 */
  const char *unit;                /* 显示单位或比例系数说明*/
  const char *source;              /* 数据来源或命令入口说明*/
} Display_HmiVariableConfig_t;

typedef struct {
  Display_HmiPage_t page;     /* 触摸区域所属页面*/
  Display_HmiVariableId_t id; /* 命中的变量 ID */
  uint16_t var_addr;          /* 命中的固件内部变量路由 ID */
  uint16_t x1;                /* 触摸区域左上角 X */
  uint16_t y1;                /* 触摸区域左上角 Y */
  uint16_t x2;                /* 触摸区域右下角 X */
  uint16_t y2;                /* 触摸区域右下角 Y */
  int16_t delta;              /* 相对增量，非绝对写入时使用*/
  uint16_t absolute_value;    /* 绝对写入值*/
  uint8_t set_absolute;       /* 0 表示按相对增量调整，1 表示写入绝对值*/
  const char *name;           /* 触摸区域名称 */
} Display_HmiTouchRegionConfig_t;

/**
 * @brief       设置 uint16 显示变量缓存
 * @param       id: 显示变量 ID
 * @param       value: 写入的 16 位无符号值
 * @retval      Display_Result_t: 设置结果
 */
Display_Result_t Display_SetHmiValueU16(Display_HmiVariableId_t id, uint16_t value);

/**
 * @brief       设置 int16 显示变量缓存
 * @param       id: 显示变量 ID
 * @param       value: 写入的 16 位有符号值
 * @retval      Display_Result_t: 设置结果
 */
Display_Result_t Display_SetHmiValueI16(Display_HmiVariableId_t id, int16_t value);

/**
 * @brief       设置 uint32 显示变量缓存
 * @param       id: 显示变量 ID
 * @param       value: 写入的 32 位无符号值
 * @retval      Display_Result_t: 设置结果
 */
Display_Result_t Display_SetHmiValueU32(Display_HmiVariableId_t id, uint32_t value);

/**
 * @brief       设置 int32 显示变量缓存
 * @param       id: 显示变量 ID
 * @param       value: 写入的 32 位有符号值
 * @retval      Display_Result_t: 设置结果
 */
Display_Result_t Display_SetHmiValueI32(Display_HmiVariableId_t id, int32_t value);

/**
 * @brief       读取当前显示变量缓存原始值
 * @param       id: 显示变量 ID
 * @param       value: 原始值输出指针
 * @retval      Display_Result_t: 读取结果
 */
Display_Result_t Display_GetHmiRawValue(Display_HmiVariableId_t id, uint32_t *value);

/**
 * @brief       请求读取显示变量
 * @param       id: 显示变量 ID
 * @retval      Display_Result_t: 请求结果
 */
Display_Result_t Display_RequestHmiRead(Display_HmiVariableId_t id);

/**
 * @brief       初始化 Display 层元数据和 LVGL 显示入口
 * @param       无
 * @retval      Display_Result_t: 初始化结果
 */
Display_Result_t Display_Init(void);

/**
 * @brief       显示模块自检
 * @param       error_code: 错误码输出指针，可传入空指针
 * @retval      Display_Result_t: 自检结果
 */
Display_Result_t Display_SelfCheck(uint16_t *error_code);

/**
 * @brief Request display hardware reinitialization from the display owner task.
 */
void Display_RequestRecover(void);

/**
 * @brief       周期刷新当前显示页面
 * @param       now_ms: 当前系统时间，单位 ms
 * @retval      Display_Result_t: 刷新结果
 */
Display_Result_t Display_Refresh(uint32_t now_ms);

/**
 * @brief       从应用只读快照准备一版完整显示缓存
 * @param       now_ms: 当前系统时间，单位 ms
 * @retval      Display_Result_t: 准备结果
 */
Display_Result_t Display_PrepareSnapshot(uint32_t now_ms);

/**
 * @brief       设置当前显示数据源
 * @param       source: 数据源
 * @retval      Display_Result_t: 设置结果
 */
Display_Result_t Display_SetDataSource(Display_DataSource_t source);

/**
 * @brief       获取当前显示数据源
 * @param       无
 * @retval      Display_DataSource_t: 当前数据源
 */
Display_DataSource_t Display_GetDataSource(void);

/**
 * @brief       在本机和对端数据源之间切换
 * @param       无
 * @retval      Display_Result_t: 切换结果
 */
Display_Result_t Display_ToggleDataSource(void);

Display_Result_t Display_RequestMotorThrottle(Display_HmiVariableId_t id, uint16_t throttle_percent);

Display_Result_t Display_RequestMotorEmergencyStop(void);

/**
 * @brief Display 层 LVGL 刷新预算诊断统计。
 *
 * @details
 * 本结构体由 Display facade 对 Debug 模块只读暴露，用于确认 `Display_RefreshStep()`
 * 是否遵守单步预算。业务层不得根据这些调试计数改变页面逻辑。
 */
typedef struct {
  uint32_t last_refresh_elapsed_us; /**< 最近一次 LVGL 刷新步耗时，单位 us。 */
  uint32_t max_refresh_elapsed_us;  /**< 启动以来 LVGL 刷新步最大耗时，单位 us。 */
  uint32_t budget_busy_count;       /**< 因超过预算返回 `DISPLAY_NOT_READY` 的次数。 */
  uint32_t page_rebuild_count;      /**< LVGL 页面重建次数。 */
} Display_DebugStats_t;

/**
 * @brief 复制 Display/LVGL 刷新预算诊断统计。
 *
 * @param[out] out 输出缓冲区，允许为 NULL；为 NULL 时函数不执行任何操作。
 */
void Display_GetDebugStats(Display_DebugStats_t *out);

/**
 * @brief       执行一次有预算约束的显示刷新步骤
 * @param       now_ms: 当前系统时间，单位 ms
 * @param       budget_us: 本次刷新预算，单位 us
 * @retval      Display_Result_t: 刷新结果；仍有工作时返回 DISPLAY_NOT_READY
 */
Display_Result_t Display_RefreshStep(uint32_t now_ms, uint32_t budget_us);

/**
 * @brief       查询 LVGL 是否需要持续调度刷新
 * @param       无
 * @retval      uint8_t: 非 0 表示显示任务需要立即继续刷新
 */
uint8_t Display_HasPendingRedraw(void);

/**
 * @brief       切换当前显示页面
 * @param       page: 目标页面 ID
 * @retval      Display_Result_t: 切换结果
 */
Display_Result_t Display_SetHmiPage(Display_HmiPage_t page);

/**
 * @brief       获取当前显示页面
 * @param       无
 * @retval      Display_HmiPage_t: 当前页面 ID
 */
Display_HmiPage_t Display_GetCurrentHmiPage(void);

/**
 * @brief       兼容旧业务触摸轮询入口，LVGL 触摸由输入设备端口处理
 * @param       无
 * @retval      Display_Result_t: 处理结果
 */
Display_Result_t Display_PollTouch(void);

/**
 * @brief       在屏幕底部显示启动阶段码
 * @param       code: 阶段码
 * @retval      无
 */
void Display_ShowBootCode(uint8_t code);

/**
 * @brief       兼容旧触摸坐标入口，当前交互由 LVGL widget 回调处理
 * @param       x: 触摸。X 坐标
 * @param       y: 触摸。Y 坐标
 * @param       id: 命中的显示变量 ID 输出指针，可传入空指针
 * @param       value: 更新后的变量值输出指针，可传入空指针
 * @retval      Display_Result_t: 处理结果
 */
Display_Result_t Display_HandleTouch(uint16_t x, uint16_t y, Display_HmiVariableId_t *id, uint32_t *value);

/**
 * @brief       获取页面配置表数量
 * @param       无
 * @retval      uint16_t: 页面数量
 */
uint16_t Display_GetHmiPageCount(void);

/**
 * @brief       获取页面配置项
 * @param       无
 * @retval      const Display_HmiPageConfig_t *: 页面配置表指针
 */
const Display_HmiPageConfig_t *Display_GetHmiPageTable(void);

/**
 * @brief       获取显示变量配置表数量
 * @param       无
 * @retval      uint16_t: 显示变量数量
 */
uint16_t Display_GetHmiVariableCount(void);

/**
 * @brief       获取显示变量配置项
 * @param       无
 * @retval      const Display_HmiVariableConfig_t *: 显示变量配置表指针
 */
const Display_HmiVariableConfig_t *Display_GetHmiVariableTable(void);

/**
 * @brief       获取触摸区域配置表数量
 * @param       无
 * @retval      uint16_t: 触摸区域数量
 */
uint16_t Display_GetHmiTouchRegionCount(void);

/**
 * @brief       获取触摸区域配置项
 * @param       无
 * @retval      const Display_HmiTouchRegionConfig_t *: 触摸区域配置表指针
 */
const Display_HmiTouchRegionConfig_t *Display_GetHmiTouchRegionTable(void);

#endif
