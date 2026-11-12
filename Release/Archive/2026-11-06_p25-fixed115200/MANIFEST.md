# 2026-11-06 P25通信确认与固定115200界面整理

本地归档候选，执行台账R51、V1-E47。用户已确认接收链路有数据，清理前照片显示UBX接收及版本/配置握手；**最终固定115200新包尚未烧录或取得用户验收**。V1-GNSS-R01仅按用户首轮接收确认关闭，T07定位及其他必需验证保持待验证，M5/M6不整体关闭。

- [本次HEX](cns-f407-p25-fixed115200-20261106.hex)、[构建证据](build_evidence.json)、[SHA清单](SHA256SUMS.txt)。
- [开发日志](../../../Development_Guide/Change_History/2026-11-06_P25通信确认与固定115200界面整理.md)。
- [用户确认](verification/user_confirmation.json)、[清理前照片记录](verification/field_receiving.json)、[最终host结果](verification/host_results.json)。
- [源码ZIP](production_source.zip)与输入清单、同次AXF/MAP/HTM/日志供版本复核；当前无.git，不声明提交或分支。

## 接收证据与验收范围

用户明确反馈“现在已经接通了有数据了”。清理前截图Uptime00:07:56，UBX/115200，Local及UART均484682 B，NMEA0、UBX5242、Receiving、Config3/3 confirmed、PROTVER34.10、ACK3/NAK0/TO0、DMA NDTR694/q0；测量字段显示N/A。

该反馈按会话关联清理前本轮固件，未从设备读回核对HEX SHA。它支持接收链路和截图所示版本/配置握手已运行；不能由N/A推断实际卫星数、fix标志或定位结果，也不能把UBX帧计数当导航更新率、计数相等当零丢帧。历史6/0及独立扫速缺陷的根因关系未单独证实，不再继续旧转换器/USB排查。

## 当前实现

相对[V1-E46开发跟进包](../../../build/20261106_p25_rx_followup/delivery/MANIFEST.md)仅8项GNSS相关生产变更，见[清单](verification/changed_production.json)。

- 本机USART2固定115200、8N1，1024 B循环RX DMA及96 B静态TX保持。删除六档扫速和驻留状态；正常不切速、不停止DMA，传输参数异常时只纠正回115200。三次MON-VER查询后退避10s，在相同速率继续探测，原UBX/NMEA、版本门控、ACK/PVT确认及owner恢复保留；不写模块保存波特率、BBR或Flash。
- 删除主GNSS三行临时提示、Details六行接收诊断、NMEA临时UBX说明及相关定时刷新。主GNSS恢复16个测量标签、2个原按钮，正常数据、NMEA捕获、告警和日志保留。
- PVT/GGA提供独立NAV_STATUS bit4和status_update_ms，授权当前可用解状态及真实总used satellites。无fix/0颗卫星也可有效；坏字段为未知，RMC/ACK/MON-VER/GSA/GSV不续期该域。Adapter第五域择新，App独立裁期；Display通过既有App_CopyGnssDetails读取正常状态，不再绑POSITION或调用诊断API，旧值/未知显示N/A，不伪造坐标。
- GNSS的UTC显示仅来自有效TIME及真实utc_sec，主Clock/Date仍为系统本地时间；Freeze只冻结测量。新增尾字段仅属于内部GNSS快照/topic，Navigation结构、wire及CSV布局保持。

第一组供电停用、第二组PA4/PC1、统一V/A/W、门禁默认0及原阈值、人工锁与电机页面均保留；未改标定。没有回环、新任务或USB设备命令。

## 实际验证与固件身份

27组选定host全部PASS；额外旧`display_lvgl_invalidation`夹具编译失败另有[记录](verification/ui_cleanup_review.json)，因缺任务类型且引用已删除的旧标签数组，未修改或计为通过。不能称全库全部通过。

唯一入口`MDK-ARM/formal_framework.uvprojx`、Target 1，ARMCC5.06 update7 build960/C99，实际Rebuild All：2026-11-06 **10:22:01—10:22:28 UTC+8**（UTC02:22:01—02:22:28），日志21s，退出0、**0 Error(s), 0 Warning(s)**。1030项输入在该次普通版构建前后、构建时工作树及对应源码ZIP逐项一致。后续E48原帧版另存子目录，当前工作树不再等于此普通版快照。

| 项目 | 结果 |
|---|---|
| Code / RO / RW / ZI | 347872 / 111016 / 1964 / 111356 B |
| 主SRAM使用 / 余量 | 113320 / 17752 B，与E46相同；Code减少2608 B |
| RX DMA / TX槽 | 0x20006450 / 1024 B；0x20006850 / 96 B，均在主SRAM |
| Flash实际装载 / HEX范围 | 459020 B；[0x08000000, 0x0807010C) |
| HEX检查 | 记录校验通过、无重叠、sector11未写 |
| HEX SHA-256 | cf9371f146c5f3a1971c391dff48dc2d51810d6e2d61977e8fb6194c60129efa |
| 源码ZIP SHA-256 | 562de876771b46a6c3738ae3ed12791f76d24ecb2aaaaa0582fd51c34126473f |

## 后续与归档

本轮用户明确要求归档，新增第21个Release/Archive目录；旧20目录及E46 build包保持历史可回退，Latest未更新。最终新包尚未烧录；位置有效性/精度、状态和UTC实屏时效、断流热插拔、最重负载资源与长稳仍待同候选验证。接收首轮通过不等于这些项目通过。

文档收尾不改生产输入，不重复构建；最终SHA清单由归档流程纳入本MANIFEST后统一重算。未执行新的USB交互或探针命令，不重试上轮被自动审批拒绝的操作。

## 后续原帧观察包

用户追加的debug串口原帧功能单独保存在[raw-debug/MANIFEST](raw-debug/MANIFEST.md)；父目录的普通HEX、源码ZIP和构建证据保持原身份，不混入观察版源码。父目录SHA清单仅覆盖普通包，不递归包含raw-debug子包；子包自带独立清单。
