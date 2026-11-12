# 树莓派控制、配置命令声光提示候选

2026-11-12，V1-RPI-T01 / V1-E75。软件验证通过，实板待验。

- [可烧录HEX](cns-f407-rpi-command-feedback-20261112.hex)
- [开发记录与测试边界](../../../Development_Guide/Change_History/2026-11-12_树莓派控制配置命令双鸣与RGB提示.md)
- [三配置Keil结果](build_matrix.json)、[27组回归摘要](host_summary.json)、[完整验证证据](verification.zip)
- [生产源码](production_source.zip)、[测试源码](host_tests_source.zip)、[开发记录快照](development_records.zip)
- [1047项输入](production_inputs.json)、[6项生产变动](changed_production.json)、[资源与身份](final_integrity.json)、[校验清单](SHA256SUMS.txt)

只对UART6本机/广播控制请求及完整合法配置请求提示，两声短鸣、白灯同步双闪；心跳、网络状态、普通遥测、ACK、H1回显和LoRa静默。每槽100ms、共四槽，由sensor推进；启动旋律优先，待播8项，满时只丢提示。结束恢复当前渐变。收包提示不表示执行成功，原命令应答与配置仲裁保留。

最终关闭提示、关闭启动声光、默认三配置ARMCC5/C99 Rebuild All均0错误0警告，27组最终相关host通过；旧两个过时夹具失败另有记录，不宣称全库测试通过。1047项构建前后/源码ZIP一致，6项生产变动、1041项与E73一致。

HEX SHA256：`44504ff9fa769645b4d4ecf0026cf3aa9b57c1cc14756871249c12f5a099e3d9`。

主SRAM 127168B，余3904B；ROM 495468B。保持原传感器诊断负载与其他功能；本功能不修复此前GNSS输入零字节或电流计偏差。尚未烧录/实测声光、资源时序与长稳，不关闭既有待验项。旧Archive和Latest不覆盖。
