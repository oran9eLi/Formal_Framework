# 2026-06-24 变更记录 31：Telemetry Catalog 外发目录

## 1. 变更范围

- 将 `px4lite_mavlink_tx.c` 的固定 slot switch 调度收敛为 Telemetry Catalog。
- 每个外发项独立描述 enable、周期、MAVLink message id、deadline、统计计数、编码函数和成功回调。
- 保留原生 MAVLink 编码函数，单周期仍最多提交一帧。
- `MODSTAT0/1` 模块状态分片通过成功回调推进，发送失败时保持当前分片等待重试。

## 2. 后续外设接入规则

- 统一的是 App/Framework 数据入口，不是统一大包。
- 新增外设状态先进入模块状态和 Health，自动通过 `MODSTAT0/1` 外发。
- 新增外设数值必须先形成 topic 或 App 快照，再新增 Telemetry Catalog 条目和编码函数。
- 禁止在 `comm` 任务栈上拼接全量显示数据或全量外设数据。
- 大 payload、分段重传和主从从机表必须使用静态缓存或固定队列。

## 3. 验证

- Keil build verification：`formal_framework.axf - 0 Error(s), 0 Warning(s)`。
