# 姿态HUD完整帧呈现候选

日期：2026-11-11。V1-HUD-T01 / V1-E66。软件验证通过，实屏待验证。基于用户烧录后仍反馈分块的E65，改为先准备完整圆圈，再集中写屏，保留原像素外观。

- [可烧录HEX](cns-f407-hud-complete-frame-20261111.hex)
- [开发日志及边界](../../../Development_Guide/Change_History/2026-11-11_姿态HUD完整帧呈现与圆mask缓存.md)
- [生产源码](production_source.zip)、[测试源码](host_tests_source.zip)、[开发记录快照](development_records.zip)
- [两配置Keil](build_matrix.json)、[15组host及真实绘图汇总](host_summary.json)、[完整证据及原始像素](verification.zip)
- [1047项输入](production_inputs.json)、[8项生产变化](changed_production.json)、[内存与HEX完整性](final_integrity.json)、[SHA256清单](SHA256SUMS.txt)

HEX SHA256：`5d042012c4617bb04815ac9b962ed3c05fd18b58488d5ac2c662168620d1a5d1`。

最终Keil唯一工程Target 1/ARMCC5.06/C99，关闭后备帧及默认启用两配置Rebuild All均0错误0警告，14s/13s；15组host通过。1193组真实LVGL姿态下，整帧/16行/49行像素相同，Capture不写GRAM，Commit无损；9份E65完整原图及28组mask/18组圆环原生像素比较通过。

CCM尾24KB存逐行RGB565无损RLE，最大测试15016B；前40KB仍供LVGL。主SRAM126008B余5064B（比E65+216B），ROM490804B（+1932B）。只有HUD用同缓冲可容纳的49行准备，其他区域仍16行；HUD专属固定圆表避免反复重算，未改全局LVGL/任务/传感器/协议/控制等设置。

准备仍用原2ms软预算，最终完整HUD提交是一个可抢占的有界同步单元，可能超2ms，耗时进入flush统计；不在中途等待下个10ms节拍。尚未烧录/测帧率与触摸时延，未接TE同步，不承诺完全无物理撕裂。需实屏确认连续转动及Freeze/Resume/切页效果，原资源/电流计/M5/M6待验项保持。

`DISPLAY_HUD_DIFF_ENABLE=0`为兼容的后备帧关闭开关，其固件位于verification.zip内`keil/run1/hud_frame_off/`，普通区域仍原批量路径；主HEX为默认启用版。旧归档与Latest保留。
