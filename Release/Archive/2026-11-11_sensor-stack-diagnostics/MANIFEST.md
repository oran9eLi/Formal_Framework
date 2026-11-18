# 三传感器公共采集栈收紧与诊断候选

日期：2026-11-11。V1-SENS-R01 / V1-E73，关联GNSS-R02及IMU-T01。用户反馈自检GNSS/IMU/Baro共红；本包修正公共栈压力并补只读证据，实板根因尚未确认。

- [本次诊断HEX](cns-f407-sensor-stack-diag-20261111.hex)
- [开发日志与改动责任核对](../../../Development_Guide/Change_History/2026-11-11_三传感器共红复核与采集栈收紧诊断.md)
- [生产源码](production_source.zip)、[测试源码](host_tests_source.zip)、[开发记录](development_records.zip)
- [双配置Keil](build_matrix.json)、[58组选定回归](host_summary.json)、[实际栈图对比](stack_comparison.json)、[完整证据](verification.zip)
- [1047输入](production_inputs.json)、[6项生产变化](changed_production.json)、[资源与身份](final_integrity.json)、[SHA256清单](SHA256SUMS.txt)

HEX SHA256：`15483420057bb319b3ab858d76afc215100f9307abe38f78dd92d64f8253663a`。

sensor独占静态暂存取代嵌套大栈对象，已知调用链1528→992B，配置sensor仍2048B、RTOS heap仍40KB；发布按值复制，失败/有效性/采集周期/硬件I/O/驱动/引脚/协议未改。静态图不是实际溢出或根因证明。

诊断版启动标签为`SENS-DIAG: sensor-stack-20261111`。每秒输出SENS三模块实际状态/故障、IMU双ID/阶段、Baro真实样本有效性，每5秒打印既有MEM/已测STACK；不新增传感器采集，GNSS原帧/双流及其他功能保留。诊断额外负载尚未实板测时。

Quiet/诊断两配置ARMCC5/C99 Rebuild All均0错误0警告、各20s，58组选定host通过；1047诊断输入前后/ZIP一致。主SRAM127080B余3992B（比E70+896B），ROM494636B（+2268B），GNSS DMA0x200092E4/1024B主SRAM，CCM保持。旧3个历史显示夹具失败及新夹具修正均见日志，不宣称全库测试全通过。

烧录本主HEX后，先在CubeProgrammer断开并让箱子独立运行20秒，再提供`SENS:`、`IMU-DRV:`、`BARO:`、`STACK: sensor`及启动标签。尚未烧录/未捕获实际水位/未确认共红根因；原资源、GNSS、电流计及M5/M6待验项保持。

Quiet仅关闭本次STACK/IMU/Baro额外打印，保留相同采集减栈；固件与对应debug_config.h位于verification.zip的`keil/run1/quiet/`。主源码ZIP对应诊断配置，Quiet复现需使用其配置副本。旧归档及Latest不覆盖。
