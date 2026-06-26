# Display LVGL 边界清理

## 1. 背景

Display 已迁移为 LVGL 页面交互，但工程中仍保留旧手绘页面栈源码和 Keil source list 项，容易让后续协作者继续向 `display_pages/display_gfx/display_text/display_logo` 添加功能，造成显示边界再次模糊。

## 2. 本次清理

1. 删除旧手绘显示栈源码和头文件：`display_pages.*`、`display_gfx.*`、`display_text.*`、`display_logo.*`、`display_logo_data.c`。
2. Keil Display group 移除旧手绘文件，新增 `Display/Src/display_log.c`。
3. `Display/Src/display.c` 仅保留 App 数据视图缓存、页面切换、数据源切换和兼容 facade；旧脏矩形/手绘刷新状态已移除。
4. 消息日志从旧页面层拆出为 `display_log` 轻量环形缓存，LVGL 页面只读取日志快照。
5. 删除 `Business_DisplaySourceKeyPressed()`，KEY0 不再作为本机/对端数据源切换入口；产品交互统一由 LVGL 页面控件承载。
6. `px4lite_mavlink_tx.c` 清理远端查看租约的未使用状态，120 s 远端查看开始时间只在从机角色编译。

## 3. 边界要求

1. 后续新增显示页面、按钮、选择框、触摸交互必须接入 LVGL。
2. `Display/Src/display.c` 不得重新承担页面绘图职责，只能作为 Display facade。
3. Display 页面不得直接读取 Framework topic、LoRa RX 帧、BSP DMA buffer 或 Driver 私有状态。
4. 本机/对端数据显示、远端节点选择和 120 s 查看租约继续通过 `app_data_api.h` 和 MAVLink stream 命令闭环。

## 4. 验证

Keil Rebuild All 已通过：

```text
".\Objects\formal_framework.axf" - 0 Error(s), 0 Warning(s).
```
