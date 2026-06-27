# 2026-06-26 44 Command 与 5G/Remote ID 接入边界收口

## 背景

进入 5G-A 和 Remote ID 开发前，需要把 LoRa 主从流控、MAVLink 命令和 ACK 处理从 RX/TX 文件中收口，避免后续控制命令散落在通信收发路径中。

## 变更内容

- 新增 `Framework/Inc/px4lite_command.h` 和 `Framework/Src/px4lite_command.c`，作为 COMMAND_LONG / COMMAND_ACK 的唯一执行边界。
- MAVLink RX 只负责逐字段解码后调用 Command 执行器，不直接执行命令。
- MAVLink TX 只保留 ACK 排队、ACK 统计和流控应用接口，不承载命令支持列表和分发策略。
- 不支持的 COMMAND_LONG 统一排队 `MAV_RESULT_UNSUPPORTED`，并视为已处理命令帧。
- 更新模块接入手册、架构边界和完成度清单，明确 5G-A、Remote ID 和 watchdog 的后续接入口径。
- 接入 Display 页眉 Logo：新增只读 RGB565 资源文件，LVGL 页眉显示 Logo 与品牌文字；未引入外部旧页面栈或 Framework 直接依赖。
- 修正远端显示断链收口：远端快照不可用时 Display 自动关闭远端查看、切回本机数据源并重建当前页；远端节点列表改为静态 scratch，避免增加 biz_display 栈占用。
- 优化远端显示交互：选择页新增返回本机按钮；远端数据页面的页眉返回入口直接回本机同一页面，不再要求重新选择从机。

## 边界结论

- 5G-A：按通信类 Driver 进入 `Driver/Comm/`，AT 控制面归所属 service/driver，Business 不接触原始 AT。
- Remote ID：当前不从本 STM32 板直通业务数据，只保留身份显示来源、模块状态和后续接入位置。
- Watchdog：BSP 初始化/刷新入口落地前保持关闭，不为了收口提前打开。

## 验证

- 已执行 Keil Rebuild All：`MDK-ARM/formal_framework.uvprojx` / `Target 1`。
- 构建工具：ARMCC 5.06 update 7 build 960。
- 构建结果：`formal_framework.axf - 0 Error(s), 0 Warning(s)`。
- 输出产物已更新：`MDK-ARM/Objects/formal_framework.axf`、`MDK-ARM/Objects/formal_framework.hex`。
