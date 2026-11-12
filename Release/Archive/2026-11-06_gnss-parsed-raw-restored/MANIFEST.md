# GPS解析与原始帧同时上报恢复候选

默认恢复parsed policy1/typed1，RAW保持1，RAW_STATS仍0。UART6解析数据8011..14及原始8020分开发，旧GPS兼容遥测与RID镜像同时恢复；本地解析、屏幕、LoRa及非GPS遥测保持。

40组选定回归通过；真实六秒模拟取得6组完整解析快照及30个完整原帧，无8021。Keil Rebuild All0错误0警告、33s，1034输入与当前源码/源码ZIP/固件匹配。HEX SHA：`dd947416c6e07a7af4e5083316b25611954fe8c1960e5427185bcf083bb438ce`。

接口只维护随包GPS原始帧_树莓派MAVLink接口.md（标题为解析与原始帧）。新候选未烧录/真实Pi/资源长稳验收，原heap预算超目标问题保留；旧归档与Latest未覆盖。
