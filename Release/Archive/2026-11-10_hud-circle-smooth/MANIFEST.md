# 姿态页HUD圆圈刷新优化候选

日期：2026-11-10。证据V1-E62，V1-HUD-T01。仅优化姿态页圆圈流畅度，其他功能及显示节拍保持原配置。软件验证通过，未烧录；实际帧率/触摸响应仍待本机屏幕观察。

固件：[cns-f407-hud-smooth-20261110.hex](cns-f407-hud-smooth-20261110.hex)

HEX SHA256：`3f9c0864b68c0c45e667790bcd20c154f1d682db4596ff83e52421998a5c8072`。

仅1项生产文件变化：`Display/Src/display_lvgl.c`。HUD从200ms整页快照中独立出来，以50ms请求最新显示快照；复用App显示参考，保留Freeze与失效灰盘。完整未绘制脏区存在时固定当前帧，结束后直接取最新值；不反复重新失效整圆，不排队旧角度。横滚/俯仰变化仅失效203×203内盘，航向/有效性变化失效全盘。三角量每帧缓存，刻度用固定10°递推，不在每个条带重复sin/cos。

唯一Keil工程、ARMCC5.06/C99 Rebuild All：0错误0警告，日志19s；1041项生产输入前后不变，与当前源码和源码ZIP逐项一致。13组选定host通过，HUD刻度递推212个航向×36刻度最大误差0.000173px。触摸优先、10ms显示任务、200ms常规快照、2ms软预算及16行缓冲不改；50ms是请求间隔，不承诺LCD实际20FPS。

Code370848、RO111868、RW2044、ZI122852B；主SRAM124896B，余6176B，较RGB父候选增加24B；压缩后ROM482848B。无新任务、DMA或运行期堆分配，既有资源长稳问题保留。

目录包含同次HEX/AXF/MAP、构建日志、build_result、production_inputs、production_source.zip及13组host证据/测试源码。旧候选和Latest不改。下一步按[开发日志](../../../Development_Guide/Change_History/2026-11-10_姿态页HUD圆圈独立刷新优化.md)检查倾斜/水平转动、Freeze/Resume、ZERO及断流灰盘；其他页面与按钮行为继续核对，不将软件回归当作实屏验收。
