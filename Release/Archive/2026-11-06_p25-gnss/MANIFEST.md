# 2026-11-06 P25 Pro GNSS 双协议接入候选

状态：本地软件候选，执行台账R48、V1-E44、V1-GNSS-T07。用户重新授权接入P25定位功能；代码、24组针对性host回归及Keil全量构建已完成，**实板待验证**。尚未烧录本候选或确认P25收数、定位和配置成功，V1-GNSS-R01及M5/M6不关闭。

- [本次HEX](cns-f407-p25-gps-20261106.hex)、[构建证据](build_evidence.json)、[SHA清单](SHA256SUMS.txt)。
- [接入与验证说明](../../../Development_Guide/Change_History/2026-11-06_P25_GNSS双协议接入.md)。
- [源码ZIP](production_source.zip)及目录内输入清单、同次AXF/MAP/HTM/日志保存本候选身份；当前无.git，不声明提交或分支身份。

## 基线与实现范围

基线为[2026-11-05原始GNSS恢复候选](../2026-11-05_gnss-original-restored/MANIFEST.md)。本轮18项生产变更为15个已有文件及3个新增文件：UBX编解码器头/源与Framework GNSS诊断类型头。完整清单见[changed_production.json](../../../build/20261106_p25_gnss/changed_production.json)；1030项生产输入在构建前后不变，无清单外生产改动。旧候选、旧MANIFEST和原SHA证据保留。

USART2仍为PA2 TX / PA3 RX、8N1，初始115200 bit/s；RX循环DMA为1024 B，静态复制TX槽96 B。sensor仍是唯一硬件所有者，每10ms服务、单次最多消费512 B；ISR维护原始传输事实并请求owner恢复，不解析协议。未新增任务或开启周期Debug任务，无回环。

接收同时支持有界NMEA和UBX；UBX内部文本不会进入NMEA解析。MON-VER轮询按115200、9600、57600、38400、19200、230400顺序探测，每档3500ms。只有结构合法MON-VER且取得有效PROTVER后才选择配置路径：协议版本至少27使用CFG-VALSET RAM层，较旧版本使用legacy CFG；启用UART1 NAV-PVT、NAV-DOP并设置测量周期，速率至少38400时目标5Hz，低速1Hz。保留NMEA输出，不设置模块保存波特率，不写BBR或Flash。

每项配置等待匹配class/id的ACK，1200ms超时，最多提交3次；步骤间至少守候1200ms。全部所需ACK之后仍须收到新PVT，才置config_confirmed；NAK或超时进入未确认但继续接收，每10s重查。没有有效帧超过6s重新探测；已接受TX卡住2s请求sensor owner恢复。ACK不含事务序号，守候基于正常应答时间界限，不能当成逐键读回或实板配置验证。

PVT与DOP按iTOW配对，pDOP不冒充HDOP；未知HDOP显示N/A。84B旧PVT保留字段不作为新标志；92B invalidLlh按当前已识别协议至少27的支持策略门控，此阈值不宣称为原厂首发版本。NMEA页仍仅捕获完整文本，UBX接收/配置事实查看GNSS页新增诊断。Driver/BSP诊断经Adapter和App API上屏，原始搬运、软件消费、有效帧、ACK及定位状态分别表达。

电源第一组停用、第二组PA4/PC1、统一V/A/W、电压门禁默认0及原算法阈值、人工锁和电机页布局保留；未改采样标定。罗盘、SBAS配置、飞控动态模型和RTK功能未新增，不凭商品名称断定芯片为M10。

## 本次实际构建

唯一入口`MDK-ARM/formal_framework.uvprojx`，Target 1，Rebuild All；实际ARMCC5.06 update7 build960、C99，构建配置标识`p25_autodetect_nmea_ubx`。2026-11-06 **08:59:28—08:59:52 UTC+8**（UTC 00:59:28—00:59:52），日志18s，退出0，**0 Error(s), 0 Warning(s)**。

| 项目 | 结果 |
|---|---|
| Code / RO / RW / ZI | 348872 / 111160 / 1968 / 111344 B |
| 主SRAM静态使用 / 余量 | 113312 / 17760 B；较恢复版增加1320 B |
| RX DMA地址 / 大小 | 0x20006448 / 1024 B |
| 静态TX槽地址 / 大小 | 0x20006848 / 96 B |
| DMA与TX所在区域 | 0x20000000起主SRAM，非CCM |
| Flash实际装载 / HEX范围 | 460164 B；[0x08000000, 0x08070584) |
| HEX检查 | 记录校验通过、无重叠、sector11未写 |
| HEX SHA-256 | 42189b44de011e759876536df33491b1b1cf9cf818d7928305fa87a5e0cedd65 |
| 源码ZIP SHA-256 | 210af7c6c7d4a84f54dc268360dcbc08c5d814b56c26523ddda3a21e68238c9e |

## 验证与交付边界

[最终host目录](../../../build/20261106_p25_gnss/host_final)24组针对性回归全部通过，涵盖UBX边界/校验、双协议与接入状态机、真实BSP传输与恢复替身、诊断映射/显示、测量新鲜度，以及单路供电、电机人工锁、CSV/MAVLink和fatal撤收保护。不是全库所有测试或真实硬件通过结论；恢复版曾记录的旧UART6夹具失败属于历史，本轮未将其改写为通过。

T07仍待同候选烧录与现场验证：记录实际baud、协议版本、硬件搬运/软件消费/有效帧、配置应答与PVT确认，再验证室外定位、断流恢复及原电源/电机页面。模块本批芯片、当前配置与兼容性以现场应答为准。资源静态余量不等于运行堆峰值、实时性或长稳达标。

Latest未更新，未提交、推送、合并或新增标签；无本轮烧录、实板成功或长稳结论。文档收尾不改变固件输入，因此不重复Keil构建；最终SHA清单由归档流程纳入本MANIFEST后统一生成。
