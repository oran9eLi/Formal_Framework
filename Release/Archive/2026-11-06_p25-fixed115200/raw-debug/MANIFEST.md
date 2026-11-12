# P25固定115200原帧观察子包

本包用于通过电脑调试串口短时查看GPS实际接收原帧，执行台账R52、证据V1-E48，配置为`p25_fixed115200_raw_debug`。它是[普通固定115200清屏版](../MANIFEST.md)的观察子包，**原帧版尚未烧录实测**；既有接收首轮用户确认不等于本观察版或定位、精度、长稳验收通过。

- [观察版HEX](cns-f407-p25-fixed115200-raw-20261106.hex)、[串口原帧查看说明](串口原帧查看说明.md)。
- [构建证据](build_evidence.json)、[同次SHA清单](SHA256SUMS.txt)。
- [Sensor原帧审查](verification/sensor_review.json)、[Framework与Debug验证](verification/framework_debug_delivery.json)、[host结果](verification/host_results.json)、[资源与回归复核](verification/resource_and_regression_review.json)。

## 输出范围

GPS仍接USART2 PA2/PA3、115200 8N1；电脑文本输出使用独立的**USART1 PA9 TX / PA10 RX，115200 8N1，无流控**。本包开启既有`DEBUG_GNSS_MONITOR_ENABLE`，复用100ms、idle+1、384word的DebugTask，不新建传感器任务。

Sensor保留三类各最新一个完整原帧：NAV-PVT、NAV-DOP、其他UBX/NMEA。Debug每1000ms轮转查找，最多输出一个新样本；每行32个字节的文本HEX，含序号和字节偏移。**这是采样镜像，不是无损、全量UART转发**。每槽序号独立，0和自然回绕合法；序号跳变也可能只是最新槽被覆盖，不能据此计算通信丢包。

最大完整wire帧392 B，即UBX载荷最多384 B加8 B头/校验。符合容量的完整坏校验帧也原样保留，CRC BAD时输出实际收到的校验字节，绝不重算成正确值；超长帧不截断输出为完整帧。NMEA镜像保留原始行和CR/LF。`ms`为sensor消费完该完整帧的服务时刻，不是硬件到线时间、GPS测量时间或PPS时间。

原帧只读复制经Framework接口进入Debug，不在Sensor/ISR打印、不由Debug消费DMA，不解码或修改定位测量。没有定位topic时仍可输出已有原帧；关闭监视不改变采集路径。普通屏幕清理及原电源、电机行为保持。

## 实际构建与验证

唯一工程`MDK-ARM/formal_framework.uvprojx`、Target 1，ARMCC5.06 update7 build960/C99，实际Rebuild All：2026-11-06 **10:52:03—10:52:25 UTC+8**（UTC02:52:03—02:52:25），日志20s，退出0、**0 Error(s), 0 Warning(s)**。1030项输入构建前后不变，同次产物和源码快照随包保存。

主机检查为原27组选定回归加4项原帧专项，两批合计31项全部通过；不是全库全部测试。专项覆盖原字节/坏校验、392 B边界、不截断超长、槽轮转、序号0/回绕、无定位不阻原帧、每秒限速及关闭监视/GNSS禁用配置。原普通版不重复构建，其父目录HEX仍为`cf9371f146c5f3a1971c391dff48dc2d51810d6e2d61977e8fb6194c60129efa`。

| 项目 | 原帧观察版 |
|---|---|
| Code / RO / RW / ZI | 349432 / 111344 / 1968 / 113400 B |
| 主SRAM静态使用 / 余量 | 115368 / 15704 B；较普通版增加2048 B |
| RX DMA / TX槽 | 0x20006AB0 / 1024 B；0x20006EB0 / 96 B，均在主SRAM |
| Flash装载 / HEX范围 | 460908 B；[0x08000000, 0x0807086C) |
| HEX检查 | 记录校验通过、无重叠、sector11未写 |
| HEX SHA-256 | 22e9f080346d5c5bb1ac86fa150dff43ac3e7372b720a2136772e1d14910fccc |
| 源码ZIP SHA-256 | 8d7fe069e87fcf20e34d9a12a5214f271a977f469e948d31be41508e42965312 |

独立包复核确认1030项源码与ZIP一致，相对普通版为9项预期生产差异。启用既有DebugTask后的FreeRTOS堆使用**估算79.22%**，高于75%目标，尚无本版运行峰值、栈水位和实时性实测。因此本包仅用于短时原帧观察，不作为长稳或资源达标候选；正常使用及后续验收选择父目录普通版。

本观察版未烧录或取得用户实测，Latest未更新。文档收尾不改生产输入，不重复构建；本MANIFEST及说明由主代理统一纳入最终SHA清单。
