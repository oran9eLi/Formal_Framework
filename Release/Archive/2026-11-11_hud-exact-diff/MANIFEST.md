# 姿态HUD精确差分写屏候选

日期：2026-11-11。V1-HUD-T01 / V1-E65。软件检查通过，实屏待验证。基于E64，针对圆圈条带重刷割裂，按最终RGB565仅提交变化像素；其他页面、传感器/通信/控制/RGB设置保留。

- [可烧录HEX](cns-f407-hud-exact-diff-20261111.hex)
- [开发日志与边界](../../../Development_Guide/Change_History/2026-11-11_姿态HUD差分写屏与割裂修正.md)
- [生产源码](production_source.zip)、[测试源码](host_tests_source.zip)、[开发记录快照](development_records.zip)
- [双配置最终构建](build_matrix.json)、[14组host汇总](host_summary.json)、[两轮构建/最终测试/反汇编](verification.zip)
- [1045项输入](production_inputs.json)、[5项生产变化](changed_production.json)、[内存与固件完整性](final_integrity.json)、[SHA256清单](SHA256SUMS.txt)

HEX SHA256：`f43c8fa763d8d535fb5caecc47c28bd9695826542837acea5dbeb76e7fefb29b`。

最终2026-11-11 10:29:29—10:30:04（UTC+8），ARMCC5.06/C99、唯一工程Target 1，关闭差分/默认启用两配置Rebuild All均0 Error(s), 0 Warning(s)，各15s。14组选定host通过，1045项生产输入前后/源码ZIP一致。

缓存25057B＝CCM尾24KB＋主SRAM481B；CCM64KB明确UNINIT保留，前40KB仍给LVGL，模块不清前区。主SRAM125792B余5280B（+496B）；ROM488872B（+2904B）。HEX只含Flash地址数据。已知色精确比较，未知抗锯齿色始终写；新模块局部优化和热点内联，不改全工程设置。

保留50ms HUD请求、10ms显示任务、2ms软预算和16行缓冲；50ms不等于实测20FPS。模拟缓慢1px步进写入量减少约95.1%，最终GRAM与原全量路径一致；此数字不是硬件耗时或帧率。未烧录/实屏，没有TE同步，不承诺完全无撕裂。下一步烧录后连续转动确认圆盘、Freeze/Resume、切页与触摸效果；既有资源/电流计/M5/M6待验项不关闭。

回退编译开关：`DISPLAY_HUD_DIFF_ENABLE=0`，对应HEX在verification.zip的`keil/run2/hud_diff_off/`；启用版本为本目录主HEX。旧归档与Latest保持。
