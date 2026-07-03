# 2026-07-02 LoRa 与 Storage 状态防抖修复

## 背景

台架测试发现 LoRa 拔掉后模块状态在黄色和红色之间反复跳变；LoRa 插上后 SD 卡模块状态出现红绿反复。复查确认 LoRa recovery 的重新初始化窗口会短暂返回 `BUSY`，导致已经确认的本机离线事实被显示为降级；Storage 在写入或 sync 失败后，只要重新 ready 就立即恢复 ONLINE，缺少状态保持和稳定恢复窗口。

## 修改

- LoRa E22 驱动新增本机硬件失败锁存：AUX 初始化超时或长期不可用后保持本机离线状态，后续 recovery 的 `BUSY` 阶段不再把红灯拉回黄色；只有 AUX 恢复并初始化成功后才清除锁存。
- Storage 状态新增故障保持窗口和恢复稳定窗口：写入失败、队列满或 `f_sync()` 失败后，公开状态至少保持降级；重新 ready 后需连续稳定一段时间再回 ONLINE，避免红绿反复。
- 同步更新 `Development_Guide/02_架构边界与数据流.md` 和 `Development_Guide/05_完成度与后续清单.md`。

## 验证

- Keil MDK-ARM Rebuild All 通过：`.\Objects\formal_framework.axf - 0 Error(s), 0 Warning(s)`。
- 构建大小：`Code=265936 RO-data=91648 RW-data=1328 ZI-data=97680`。
