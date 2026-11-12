# 固件发布目录

## 先看这里

本目录同时保存发布计划、验证记录和历史候选，并非里面所有文件都属于当前固件。开发过程的原因和代码改动写在[开发日志目录](../Development_Guide/Change_History/README.md)，电源、电机等当前保留功能见[已落地功能汇总](../Development_Guide/Change_History/2026-11-06_已落地功能与验证状态汇总.md)。

| 要找什么 | 当前入口 |
|---|---|
| 当前源码Git提交与main集成记录 | [提交与合入日志](../Development_Guide/Change_History/2026-11-12_当前版本提交与main合入.md)，独立检出37组host/Keil全量0/0；最终远端结果见该记录 |
| 当前树莓派控制/配置声光提示候选（继承E73传感器诊断） | [固件及说明](Archive/2026-11-12_rpi-command-feedback/MANIFEST.md)，27组相关回归、最终三配置Keil全量0/0；实际双鸣双闪待验，GNSS等原问题保持 |
| 声光提示前的三传感器共红诊断父候选（公共栈收紧＋只读状态/ID/水位） | [固件及说明](Archive/2026-11-11_sensor-stack-diagnostics/MANIFEST.md)，58组选定host/Quiet与诊断Keil全量0/0，尚未认定现场根因；本包有额外Debug负载 |
| 三模块共红排查前GNSS重连父候选（继承HUD/BMI双流/GPS/RGB） | [固件及说明](Archive/2026-11-11_gnss-reconnect/MANIFEST.md)，重开状态校验、6s字节静默自愈及异常Debug；45组host/Keil全量0/0，实板待复测 |
| 重连跟进前GNSS半帧/TX隔离父候选，用户确认已刷仍不稳 | [固件及说明](Archive/2026-11-11_gnss-dropout-fix/MANIFEST.md)，500ms半帧期限与TX/RX隔离；39组host/Keil全量0/0，实际掉线待烧录复测 |
| GNSS修复前HUD完整帧父候选（继承BMI双流/GPS/RGB） | [固件及说明](Archive/2026-11-11_hud-complete-frame/MANIFEST.md)，先离屏准备完整圆圈再集中写屏；15组host/1193真实绘图/Keil双配置0/0，实屏待烧录 |
| 完整帧前HUD差分父候选，用户反馈仍有分块 | [固件及说明](Archive/2026-11-11_hud-exact-diff/MANIFEST.md)，14组host/双配置Keil0/0；针对条带割裂，实屏待烧录 |
| HUD差分前RGB慢渐变父候选（继承BMI双流/HUD/GPS） | [固件及说明](Archive/2026-11-11_rgb-ambient/MANIFEST.md)，2秒/色、14秒/轮，4组最终测试及双配置Keil0/0；未烧录实板 |
| 慢渐变前BMI088双流父候选，保留HUD/GPS/RGB等 | [固件与Pi对接包](Archive/2026-11-10_bmi088-mavlink-dual-stream/MANIFEST.md)，原始/解析5Hz配对＋1Hz状态，32C/17Py/三配置Keil0/0；未烧录实Pi |
| BMI双流前HUD圆圈流畅度父候选 | [HUD固件与说明](Archive/2026-11-10_hud-circle-smooth/MANIFEST.md)，13组选定host/Keil全量0/0；独立50ms请求，实屏效果待烧录确认 |
| HUD优化前PC9七彩启动联动父候选（默认BMI088 I²C） | [RGB节拍候选与HEX](Archive/2026-11-10_rgb-startup-rhythm/MANIFEST.md)，14组软件测试、双配置Keil全量0/0；未烧录，灯色/节奏/波形待实板 |
| 当前BMI088接入候选：默认I²C，用户已确认接通 | [BMI088候选与三配置说明](Archive/2026-11-10_bmi088-dual-bus/MANIFEST.md)，E58软件验证通过；[最新接通/教学设计记录](../Development_Guide/Change_History/2026-11-10_IMU接通反馈与两项箱体实验设计.md)，完整实板待验 |
| 替换前回退：箱体GNSS教学，原MPU6050 | [cns-f407-gnss-box-teaching-20261109.hex](Archive/2026-11-09_gnss-box-teaching/cns-f407-gnss-box-teaching-20261109.hex)，保留ADC诊断、GPS双流与Time/Sat；不含BMI088驱动 |
| 主回路ADC诊断父候选（历史） | [cns-f407-power-adc-diag-20261108.hex](Archive/2026-11-08_power-adc-diagnostics/cns-f407-power-adc-diag-20261108.hex)，22组回归/Keil0/0；现场偏差待日志定位 |
| GPS解析＋原帧父候选（历史） | [cns-f407-gnss-parsed-raw-20261106.hex](Archive/2026-11-06_gnss-parsed-raw-restored/cns-f407-gnss-parsed-raw-20261106.hex)，40组回归/Keil0/0 |
| 前版GPS仅原帧固件（历史） | [cns-f407-gnss-raw-only-20261106.hex](Archive/2026-11-06_gnss-raw-only/cns-f407-gnss-raw-only-20261106.hex)，40组回归/Keil0/0 |
| 当前唯一Pi接口文档 | [GPS原始帧MAVLink接口](../Development_Guide/GPS原始帧_树莓派MAVLink接口.md) |
| 前版typed＋RAW候选（历史） | [统一出口＋独立原始帧](Archive/2026-11-06_gnss-unified-raw-uplink/MANIFEST.md)，软件验证通过，待实板 |
| 前版typed＋RAW固件 | [cns-f407-gnss-unified-raw-20261106.hex](Archive/2026-11-06_gnss-unified-raw-uplink/cns-f407-gnss-unified-raw-20261106.hex) |
| 前版完整交付包（历史） | [gnss-mavlink-delivery-20261106.zip](Archive/2026-11-06_gnss-unified-raw-uplink/gnss-mavlink-delivery-20261106.zip) |
| 前版屏幕适配观察候选 | [GNSS Frames自适应NMEA/UBX](Archive/2026-11-06_gnss-adaptive-frames/MANIFEST.md)，[操作说明](Archive/2026-11-06_gnss-adaptive-frames/USER_GUIDE.md)；保留debug，未实屏/资源验收 |
| GNSS报文页适配历史固件 | [cns-f407-gnss-adaptive-frames-20261106.hex](Archive/2026-11-06_gnss-adaptive-frames/cns-f407-gnss-adaptive-frames-20261106.hex)，32组host/Keil0/0；观察配置 |
| 前版原帧串口观察候选 | [raw-debug/MANIFEST](Archive/2026-11-06_p25-fixed115200/raw-debug/MANIFEST.md)，[查看说明](Archive/2026-11-06_p25-fixed115200/raw-debug/串口原帧查看说明.md)；每秒采样一帧，仅短时观察，未烧录验收 |
| 最新普通归档 | [P25通信确认与固定115200界面整理](Archive/2026-11-06_p25-fixed115200/MANIFEST.md)，最终新包待烧录验收 |
| 历史开发诊断包、可回退 | [V1-E46扫速恢复/六行诊断](../build/20261106_p25_rx_followup/delivery/MANIFEST.md)，当前已移除临时屏幕提示 |
| 上一归档、可回退基线 | [V1-E44 P25 GNSS候选](Archive/2026-11-06_p25-gnss/MANIFEST.md)，旧归档不改 |
| 原帧观察HEX | [cns-f407-p25-fixed115200-raw-20261106.hex](Archive/2026-11-06_p25-fixed115200/raw-debug/cns-f407-p25-fixed115200-raw-20261106.hex)，USART1 debug 115200/8N1文本接收 |
| 普通版可烧录文件 | [cns-f407-p25-fixed115200-20261106.hex](Archive/2026-11-06_p25-fixed115200/cns-f407-p25-fixed115200-20261106.hex)，固定115200、正常GNSS状态与UTC显示 |
| 当前完成度、问题和验收证据 | [V1.0_执行台账.md](V1.0_执行台账.md)，进度唯一维护入口 |
| 每次修改的详细开发日志 | [Development_Guide/Change_History/](../Development_Guide/Change_History/README.md) |
| 较早发布的历史说明 | [Latest/MANIFEST.md](Latest/MANIFEST.md)：本工作区目前只有11月3日说明，没有HEX/AXF/MAP，不是本次最新固件下载入口 |

以上以2026-11-06本地实际文件为准。Latest目录名不证明文件是最新源码或已通过整机验收；选择固件要看对应MANIFEST、实际文件及SHA。

## 为什么有这么多文件

V1-E45历史盘点时，Release顶层有34份Markdown文档，Archive为20个目录、742个文件、约244.46MiB；这不是新增归档后的当前文件总数。V1-E46跟进包仅放build下，V1-E47现按用户明确要求新增固定115200归档，E47时Archive共21个目录；E49再新增GNSS Frames候选，当前23个目录（E51新增统一出口与原始帧最终候选），原21个目录不改不删。历史GNSS试接、修复、诊断、回滚和当前版本分别保留追溯用途，不是同时运行的23套功能。

E45盘点的文件数量主要来自522份小型测试日志；空间主要由源码ZIP和带调试信息的AXF占用。较早的11个Archive目录在本工作区仅保留MANIFEST，不能仅凭目录名假定固件齐全。完整盘点见[本地统计](../build/20261106_development_log_review/release_inventory.json)。

| 位置或文件 | 用途 |
|---|---|
| `Archive/日期_主题/` | 某次候选或历史版本；用于追溯、比较、必要时回退 |
| `Latest/` | 既有默认发布入口；当前本地只留旧清单，本轮候选没有自动覆盖这里 |
| `V1.0_里程碑计划.md`、`*_准备*`、`*_实施计划*` | 范围、顺序和执行方案，不代表功能已完成 |
| `V1.0_执行台账.md` | 当前任务、范围决定、问题关闭与发布状态 |
| `*_验证*`、`*_VALIDATION*`、`*_ACCEPTANCE*` | 检查步骤、软件/实板结果和未完成项 |
| `*_合入*`、`*_归档记录*` | 当时版本身份及集成/存档过程 |

一个候选目录内常见文件的含义：

| 文件 | 用途 |
|---|---|
| `.hex` | 实际烧录固件；只使用选定候选对应的那一份 |
| `.axf` | 带调试信息的程序，供调试器定位函数和变量 |
| `.map` | Flash/RAM及函数、变量的分配报告 |
| `rebuild.log`、`.build_log.htm`、`verification/` | 同次编译与测试证据 |
| `production_source.zip`、`host_tests_source.zip` | 该次生产源码及主机测试快照，不是另一套正在运行的程序 |
| `inputs_*.json`、`production_inputs.json`、`SHA256SUMS.txt` | 文件清单和校验值，确认源码/固件是否对应 |
| `MANIFEST.md` | 版本用途、构建结果、固件身份与尚未验证项，先读它再选固件 |

开发日志整理保留历史归档及原固件身份；本轮最终固定115200版本另按用户要求归档。旧试验被撤回后，其记录仍可追溯，不作为当前能力或验收通过证明。

## 当前发布计划

- [2026-11-06 GNSS Frames自适应报文页](Archive/2026-11-06_gnss-adaptive-frames/MANIFEST.md)：当前软件观察候选。Auto RX识别NMEA/UBX/Mixed，支持双协议原帧/筛选/分页，源数值4和旧NMEA API兼容。32组选定host、run2 Keil全量0/0、19s；主SRAM余15144B，原堆预算超目标保留，新页与资源实板待验，旧21及Latest不改。

- [2026-11-06 GPS原帧debug观察版](Archive/2026-11-06_p25-fixed115200/raw-debug/MANIFEST.md)：当前源码附加USART1原帧采样，固定115200和清理界面保留；27+4两批31组选定host/Keil0/0，日志20s。堆预算约79.22%超过75%目标，仅短时取帧，资源/长稳未通过，尚未烧录；普通版cf937…保持独立，Archive仍21目录，Latest不改。

- [2026-11-06 P25通信确认与固定115200界面整理](Archive/2026-11-06_p25-fixed115200/MANIFEST.md)：当前本地归档。用户首轮确认接收，清理前图见UBX/115200、PROT34.10、Config3/3及ACK3；最终新包固定115200、清临时提示，独立状态/卫星时效和真实GPS UTC正常显示。27组选定host PASS、Keil全量0/0、21s，1030输入一致；最终新包未烧录，T07定位/其他矩阵仍待验，Latest不改。

- [2026-11-06 GNSS开发诊断跟进包（历史E46，可回退）](../build/20261106_p25_rx_followup/delivery/MANIFEST.md)：该时点修扫速恢复驻留并加六行接收诊断，24组host/Keil0/0保留证据。用户后续确认接收已通；当前E47固定115200并移除临时提示，六行诊断不再是当前界面。

- [2026-11-06 P25 GNSS双协议接入（历史E44，可回退）](Archive/2026-11-06_p25-gnss/MANIFEST.md)：早期六档探测及诊断归档，24组host/Keil0/0证据保留。当前E47已固定115200并清临时界面，不能按该旧清单描述当前扫速行为。

- [2026-11-05原始GNSS复核与ATGM现场反馈](../Development_Guide/Change_History/2026-11-05_原始GNSS复核与ATGM实板反馈.md)：只读复核，原始身份与电源/电机保留确认，18项回归及补充主机数据流通过；用户已烧录、接回ATGM336H后仍无有效显示，ST-Link只读连接未成功，实板可用未确认。未修改固件、未新建候选或更新Latest。

- [2026-11-05恢复P25适配前原始GNSS](Archive/2026-11-05_gnss-original-restored/MANIFEST.md)：历史回滚候选，R48已按用户新授权实现P25双协议；该次1027生产输入精确恢复motor基线，9600/512B及原NMEA行为；Keil全量0/0、13s，HEX与原motor相同。19组尝试为18主回归通过、1旧UART6夹具链接失败；未烧录或关闭GPS实板故障。P25及后续修复/诊断候选均转历史，Latest保留。

- [2026-11-05移除GNSS回环（历史R45）](Archive/2026-11-05_gnss-no-loopback/MANIFEST.md)：该轮删除回环的构建/HEX证据保留；R46已进一步恢复首次P25适配前原始GNSS，原R43接收/配置诊断不再是当前实现。

- [2026-11-05 GNSS运行配置读回（历史，已撤回实现）](Archive/2026-11-05_gnss-rx-config/MANIFEST.md)：保留37组host、两版Keil及现场读回证据；R46按用户要求撤回CFG/DMA/REG接口与报告，不声明GPS已调通。

- [2026-11-05 GNSS接收恢复修复（历史，已撤回实现）](Archive/2026-11-05_gnss-rx-fix/MANIFEST.md)：保留当时36组host、普通/诊断Keil0/0及候选身份；R46恢复原接收/恢复行为，不能将旧修复描述为当前能力。

- [2026-11-04北征P25 Pro定位试接（历史，用户已取消本轮实现）](Archive/2026-11-04_p25-gnss-trial/MANIFEST.md)：原115200/1024B及34组host、两版Keil证据保留；R46已恢复首次P25之前的9600/512B原始GNSS，罗盘未接入。

- [2026-11-04电机人工锁候选](Archive/2026-11-04_motor-manual-lock/MANIFEST.md)：第四个Lock/Unlock按钮，原电压门禁默认停用、代码保留；26组host及Keil门禁1/0全量0/0。实板待验，用户报告的PA4约12V须先排查，Latest保留。

- [2026-11-04单路主回路监测候选](Archive/2026-11-04_single-supply/MANIFEST.md)：默认停用第一组，仅第二组采样，统一Voltage/Current/Power；20组host及Keil四配置全量0/0通过。同次固件与源码证据本地归档，实板待验，Latest保留。

- [2026-11-03项目归档候选](Archive/2026-11-03_project-archive/MANIFEST.md)：最新功能代码与固件完整归档，常规候选重新Keil全量0/0、41 s；GT911诊断另包保存。当前触摸故障未定位，M5/M6和长稳状态保持；本次不合入develop，不作为稳定版发布。

- [2026-10-27 PE5经典启动音试听](../Development_Guide/Change_History/2026-10-27_PE5经典启动音试听.md)：独立本地候选，Keil0/0；仅一次启动音，接线与实听待验，不覆盖Latest，不作为正式发布。

- [2026-10-23 Serial通用监视候选](Archive/2026-10-23_serial-monitor-candidate/MANIFEST.md)：codex/serial-monitor独立分支；完整UART6配置和端口统计，标题Serial、协议及实验保持；Keil0/0，同次产物更新Latest，实板待验证。

- [2026-10-23数字字距修复合入](V1.0_数字字距修复_合入记录.md)：用户确认第二版字距显示正常，完整候选已本地快进合入develop；合入后Keil0/0且HEX与已验收候选相同，推送目标origin/develop。未新增tag/Release附件，M5/M6其他验收保留。以下日期候选保留历史身份。

- [2026-10-22运行性能候选](Archive/2026-10-22_runtime-performance-candidate/MANIFEST.md)：codex/fix-runtime-performance，Latest已更新为同次本地候选，Keil0/0、SRAM+8 B/Flash+192 B；未合入develop或推送，实板待验，灰显日期归档保留。

- [2026-10-22姿态仪灰显合入](V1.0_姿态仪灰显_合入记录.md)：已本地合入develop，合入后Keil0/0且HEX不变；后续性能问题另开分支，实板验收继续待补。

- [2026-10-21姿态仪无数据灰显候选](Archive/2026-10-21_attitude-no-data-candidate/MANIFEST.md)：独立修复分支，本地Keil0/0，SRAM不变、Flash+624 B；该同次候选保留在日期归档，尚未推送/发布，实板待验。原优化预发布与标签保留。

- [2026-10-15 SRAM与显示精简记录](../Development_Guide/2026-10-15_SRAM与显示刷新精简说明.md)：已合入develop，[优化预发布](https://github.com/oran9eLi/Formal_Framework/releases/tag/v1.0.0-pre.opt.20261015)及固件已归档，合入后Keil0/0，主SRAM空余19384 B；四项净释放14768 B。该优化候选与M5历史产物均保留在日期归档；实板与计时待验，见[合入归档记录](V1.0_SRAM与显示精简_合入与归档记录.md)。
- [V1-M5实施计划](V1.0_M5_实施计划.md)、[验证记录](V1.0_M5_验证记录.md)与[原M5候选身份](Archive/2026-10-14_v1.0.0-pre.m5.20261014/MANIFEST.md)：六步软件及审查修正完成，主SRAM余4616 B；Details/Inspector/NMEA/Serial/Observe/Drift/Record和电脑LoRa工具已交付。2026-10-14重新Keil全量0/0、41 s，M5标签/四个固件附件已推送并回下载一致，见[GitHub归档记录](V1.0_M5_GitHub归档记录.md)。M5已合入develop，已合并分支已清理，交互/显示首轮观察良好，见[合入记录](V1.0_M5_合入记录.md)；功能完整性与数据正确性等C01—C16待联合验收，问题单开修复分支。[课堂说明](../Development_Guide/2026-10-13_M5教学观察与记录说明.md)、[PC LoRa说明](../Tools/LoRa测试工具说明.md)。

- [M6准备方案](V1.0_M6_准备与联合验收方案.md)、[M5/M6联合矩阵](V1.0_M5_M6_联合验收矩阵.md)：按用户安排同候选联合验收，108点具体操作/预期行为覆盖页面、数据、通信、记录、保护、资源和课堂；现有硬件组装完整一台，正式Word后续交由文档输出代理，M7保留至少24小时长稳。

- [V1.0 单台组装与展示方案 R3](V1.0_硬件升级与外观设计方案.md)：沿用现有F407硬件、模块与连接方式，组装一台能够完整使用与展示的设备；完成必要装配、接线修复和全机验收，任务与证据统一记录在执行台账。
- [V2.0 硬件升级与外观设计方案 R2](V2.0_硬件升级与外观设计方案.md)：原R2升级整体转V2，保留统一外壳/底座、可取放模块、新插座/线束、P25 Pro候选、条件罗盘、蜂鸣器和灯孔/条件装灯设计；数量与排期重新评估。H743等V1完成后单独评估，未承诺纳入V2。
- [2026-10-12 V1-M3 GitHub 开发存档](V1.0_M3_GitHub归档记录.md)：源码标签 `v1.0.0-pre.m3.20261012`，HEX/构建附件已上传并回下载核对；本次全量构建 0 错误/0 警告，保持预发布和实板待验状态。
- [2026-10-13 V1-M4 修正标签归档](V1.0_M4_标签归档记录.md)：Node ID保存修正06e6a21已推送develop并建立`v1.0.0-pre.m4.20261013`；用户确认该修正通过，重建0/0，同次固件本地归档；未创建M4 GitHub Release附件，其余必需验收继续待验证。
- [V1-M4 实施计划](V1.0_M4_实施计划.md)与[验证记录](V1.0_M4_验证记录.md)：按已确认[设计方案](V1.0_M4_准备与设计方案.md)实施手动轻量收发、地址保存、CH读回和一个活动对端；代码已接入，构建/审查与实板证据分别记录。
- [V1-M3 实施计划](V1.0_M3_实施计划.md)与[验证记录](V1.0_M3_验证记录.md)：页面、导航和事件收敛；D01—D03 已确认，实际构建与实板证据分开记录。
- [V1-M2 实施计划](V1.0_M2_实施计划.md)与[验证记录](V1.0_M2_验证记录.md)：五项软件与审查修复完成，41 组回归、默认 Keil 全量构建通过；实板待验，静态 SRAM 增加 640 B。
- [V1.0 里程碑计划](V1.0_里程碑计划.md)：独立的 V1-M0—V1-M8，按交付结果安排阶段、依赖和验收条件。
- [V1.0 执行台账](V1.0_执行台账.md)：本轮任务、范围决策、问题关闭、证据与候选状态的唯一维护入口。
- [V1-M1 保护与启动准备及验收方案](V1.0_M1_准备与验收方案.md)：改动前调用链、XRotor 20A 异常撤收、参数冻结点与实板验收条件。
- [V1-M1 实施计划](V1.0_M1_实施计划.md)：电源门禁、fatal、RTC、心跳和启动看门狗五步软件已实施；实板步骤待执行。
- [V1-M1 实施与验证记录](V1.0_M1_验证记录.md)：审查修复、17 组回归、Keil 全量构建、资源和开发产物身份；实板与长稳未执行。
- [V1.0 稳定版预发布改动文档（R5）](../Development_Guide/2026-09-28_V1.0稳定版预发布改动文档.md)：学生主动操作、精简屏幕、远端单页、轻量LoRa和手动配置；日志布局、M4精简及M5增强范围已确认，具体设计、实现和验收分别记录。
- [2026-09-28 全库审查](../Development_Guide/2026-09-28_稳定版固件全库审查.md)：本轮 A01—A17 的根因与验证线索。

本轮从新台账建立基线，不继承旧模块清单或旧里程碑的完成状态。预发布文档定义范围，里程碑计划定义顺序，执行台账记录实际结果。

## 历史资料

- [旧 v1.0 发布里程碑与分批执行清单](v1.0_MILESTONES.md)：9 月 16 日 M1—M5 的历史批次与证据。
- [v1.0 发布前审查](../Development_Guide/2026-09-15_v1.0发布前审查.md)：问题证据、复现结果和教学交互建议。
- [M1 软件验证与资源预算](v1.0_M1_VALIDATION.md)：实际回归、构建、内存增量和实板待验项。
- [M2 测量与日志验证](v1.0_M2_VALIDATION.md)：主动校准、显示基准、CSV v2 与实板待验项。
- [M3 教学交互验证](v1.0_M3_VALIDATION.md)：姿态操作、电机目标/输出、模式选择、实时通信统计与资源变化。
- [v1.0 课堂实验台操作说明](../Development_Guide/2026-09-16_v1.0课堂实验台操作说明.md)：教师和学生的操作步骤、预期反馈及记录要求。
- [M4 v1.0-rc1 实板验收记录](v1.0_M4_ACCEPTANCE.md)：绑定当前源码/固件身份的实板矩阵与运行水位记录表；全部填写前不构成 rc1。

这些记录与课堂说明保留历史实现/验收上下文，不表示 Node ID、信道配置或新的页面布局已经交付，也不代替新候选验收。

## 产物归档规则

- `Latest/`：选定的默认发布入口；内容以实际文件及MANIFEST为准，本地软件候选不会自动覆盖。当前目录仅有历史说明，实际最新本地候选见本页开头。
- `Archive/`：需要长期保留的版本，按日期和版本号建立子目录。

只有满足以下条件才允许更新 `Latest/`：

1. Keil ARMCC5 全量重建完成。
2. 编译结果为 `0 Error / 0 Warning`。
3. `hex`、`axf`、`map` 和构建日志来自同一次构建。
4. `MANIFEST.md` 已更新。

开发过程中的中间产物仍保留在 `MDK-ARM/Objects` 和
`MDK-ARM/Listings`，不得当作发布版本分发。

- [2026-11-08主回路ADC诊断候选](Archive/2026-11-08_power-adc-diagnostics/MANIFEST.md)：参考合法性检查、明确回退与低优先级串口诊断；22组host和Keil全量0/0、18s。用户PA4更正1.18V，旧12V判断撤销；实际偏差根因与标定待日志，Latest保持。
