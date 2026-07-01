# 2026-06-30 LoRa / RemoteID 统一身份与广播过滤收口

## 变更摘要

- 新增 `px4lite_identity`，统一派生 RemoteID、LoRa node_id 和 MAVLink system_id。
- 配置规则改为 `DCDW-%03u`：主机 `DCDW-001 / id=1`，从机从 `DCDW-002 / id=2` 开始。
- LoRa 心跳周期调整为 1000 ms，LoRa offline 判定窗口调整为 8000 ms，降低从机黄绿反复闪烁。
- 新增 `LORASUM` MAVLink `NAMED_VALUE_INT` 摘要，广播 active viewer、lease id 和 120 s 倒计时。
- 完整数据接收增加门控：未处于远程查看目标的节点只处理 heartbeat、command、ack 和 summary，不写入完整远端快照。
- 主机收到从机 stream start 后按“最后一个查看者为准”记录 active viewer；同一查看者续约不延长 120 s 上限。
- Display 远端节点列表改为 `DCDW-%03u`，并显示完整流占用者和剩余秒数。

## 架构和栈口径

- 未新增任务，未扩大 `PX4LITE_STACK_COMM`。
- 新增状态均为文件级静态变量、远端遥测快照标量字段或 App 视图标量字段。
- Command 执行仍收口在 `Framework/Src/px4lite_command.c`；MAVLink RX 只负责解码和过滤，TX 只负责调度和控制帧发送。
- RemoteID UAS ID 不再硬编码配置宏，由 `px4lite_identity` 根据 `PX4LITE_UNIT_ID` 生成。

## 验证

- Keil Rebuild All：`MDK-ARM/build_lora_remoteid_identity.log`
- Program Size: `Code=267828 RO-data=91540 RW-data=1360 ZI-data=128488`
- 结果：`0 Error(s), 0 Warning(s)`
