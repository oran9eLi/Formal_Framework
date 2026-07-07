# LVGL 移植方案与接入记录

## 1. 接入原则

本次移植不改变既有架构。LVGL 只作为 Display 模块内部渲染后端，禁止
Business、Framework、Sensor、BSP 以外路径直接调用 LVGL 或写 FSMC LCD。

保持的正式链路：

```text
Framework topic
  -> app_data_api.h
  -> Business_DisplayServiceTask
  -> Display_PrepareSnapshot() / Display_SetHmiValue*
  -> display_lvgl.c
  -> lv_port_disp.c / lv_port_indev.c
  -> display_ssd1963.c / display_gt911.c
  -> BSP FSMC / Touch Port
```

强制边界：

- `Business_DisplayServiceTask` 仍是显示任务，不新增 LVGL 独立任务。
- DisplayTask 仍是 FSMC LCD 唯一写入者。
- 上层仍只通过 `app_data_api.h` 和 `Display_SetHmiValue*` 传递显示数据。
- LVGL 头文件只在 Display 模块内部使用。
- 不分配 800 x 480 全屏帧缓冲。

## 2. 内存方案

硬件资源：

```text
Flash: 0x08000000, 1 MB
SRAM : 0x20000000, 128 KB
CCM  : 0x10000000, 64 KB
LCD  : SSD1963 内部 GRAM
```

LVGL 配置：

| 项目 | 当前值 | 说明 |
|---|---:|---|
| `LV_COLOR_DEPTH` | 16 | RGB565 |
| `LV_MEM_SIZE` | 40 KB | LVGL 内部堆 |
| `LV_MEM_ADR` | `0x10000000` | 放入 CCM，减轻主 SRAM 压力 |
| draw buffer | `800 x 32 x 2 = 50 KB` | 静态局部刷新缓冲（原 800x10=16KB，2026-06-25 放大） |
| 全屏 framebuffer | 不使用 | 800 x 480 x 2 需要约 768 KB |

主 SRAM 实测占用（map）：链接区 RW_IRAM1 约 74 KB（含 40 KB FreeRTOS 堆 + 原 16 KB
draw buffer），MSP 栈仅 1 KB 在区顶，其上约 54 KB 空闲（ARM 库堆已移除，LVGL 堆在 CCM）。
draw buffer 放大到 50 KB 后主 SRAM 约用 108 KB、余约 20 KB，CCM 仍余 24 KB——在合理裕量下
尽量放大缓冲，整屏重绘条带 flush 由 48 次降到 15 次，缩短切页扫描感。flush 不走 DMA，
缓冲放主 SRAM 即可。若日后主 SRAM 紧张，可把 buffer 挪入 CCM（最多约 24 KB / 15 行）。

注意事项：

- 当前 scatter 文件没有把普通 RW/ZI 自动放入 CCM。LVGL heap 通过 `LV_MEM_ADR`
  使用固定 CCM 地址，不占主 SRAM 链接空间。
- draw buffer 当前是 `lv_port_disp.c` 内静态数组，位于主 SRAM。
- 若后续主 SRAM 紧张，可给 draw buffer 增加专用段并挪入 CCM；由于当前刷新不走 DMA，
  CCM 可用于 CPU 读写。

## 3. 文件变更

新增：

```text
Third_Party/LVGL/
Third_Party/LVGL/lv_conf.h
Display/Inc/display_lvgl.h
Display/Src/display_lvgl.c
Display/Inc/lv_port_disp.h
Display/Src/lv_port_disp.c
Display/Inc/lv_port_indev.h
Display/Src/lv_port_indev.c
```

改造：

```text
Display/Src/display.c
Display/Inc/display_ssd1963.h
Display/Src/display_ssd1963.c
Business/Inc/business_template_config.h
MDK-ARM/formal_framework.uvprojx
```

暂不删除：

```text
Display/Src/display_pages.c
Display/Src/display_gfx.c
Display/Src/display_text.c
Display/Src/display_logo.c
```

旧点阵渲染文件先保留，用于硬件调试回退和页面迁移对照。全部 LVGL 页面稳定后，
再从 Keil 工程中移除旧渲染文件。

## 4. 当前实现状态

已完成：

- LVGL 8.3 源码复制到工程内部 `Third_Party/LVGL`。
- `lv_conf.h` 按 STM32F407 + SSD1963 小内存方案配置。
- SSD1963 新增 `Display_Ssd1963_FlushPixels()`，支持 LVGL 脏区连续写 GRAM。
- 新增 LVGL 显示 port：`flush_cb -> Display_Ssd1963_FlushPixels()`。
- 新增 LVGL 触摸 port：`read_cb -> Display_Gt911_Scan()`。
- `display.c` 保持对外 API 不变，内部通过 `DISPLAY_USE_LVGL_BACKEND` 切换到 LVGL。
- `Display_RefreshStep()` 驱动 `lv_tick_inc()` 和 `lv_timer_handler()`。
- 显示任务栈从 768 word 调整为 1280 word，后续需用 high-water mark 实测确认。
- Keil 工程加入 LVGL include path、Display 新文件和 LVGL 必需源码组。

当前 LVGL UI 是第一阶段状态总览页，用于验证刷屏、触摸、数据入口和内存配置。
后续正式页面应继续通过 `Display_SetHmiValue*` 和 `Display_PrepareSnapshot()` 更新控件。

## 5. 后续页面迁移规则

迁移时按页进行，不一次性删除旧代码：

1. 总览页确认 LVGL 刷屏和触摸稳定。
2. 迁移姿态/飞行页。
3. 迁移定位/数据页。
4. 迁移电机控制页。
5. 迁移告警页。
6. 移除旧点阵渲染文件和对应 Keil source list。

每一页迁移必须满足：

- 不新增任务。
- 不在任务栈中放大数组。
- 控件更新只消费 Display 已有变量缓存或 App 只读快照。
- 触摸命令仍在 Display 层转成业务命令，不在绘图函数里改业务状态。
- 刷新路径必须可被 `BUSINESS_DISPLAY_REFRESH_BUDGET_US` 预算约束。

## 6. 验证门禁

合入前必须确认：

- Keil ARMCC5 构建 `0 Error(s), 0 Warning(s)`。
- 实机显示 LVGL 总览页。
- GT911 触摸在 LVGL indev 下可读。
- DisplayTask 栈最小剩余大于 25%，且不少于 100 word。
- FreeRTOS heap 峰值使用率不超过 75%。
- LVGL heap 没有进入 malloc assert。
- 屏幕恢复仍通过 `Display_RequestRecover()` 置位，由 DisplayTask 路径执行。
