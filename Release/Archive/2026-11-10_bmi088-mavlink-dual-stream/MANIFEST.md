# BMI088原始与解析MAVLink双流候选

2026-11-10，V1-E63、V1-IMU-T03。默认BMI088 I²C，保留现有GPS双流、HUD50ms独立请求、RGB及电源/电机设置。未烧录或实Pi接收验收。

固件：[cns-f407-bmi088-dual-stream-20261110.hex](cns-f407-bmi088-dual-stream-20261110.hex)

HEX SHA256：`7df99dae989b17d5fcb15cb73f7d6445c93b4994c507c5553ecf150951bc6f11`。

UART6保持115200 8N1，MAVLink2 TUNNEL385：0x8030原始寄存器采样、0x8031同次解析六轴、0x8032设备/发送状态，内层均64B。默认最多5Hz完整配对抽样、状态约1Hz；标称100Hz采集不等于100Hz全量无损上报。原始含完整本轮陀螺FIFO而非float反算，不是I²C/SPI波形抓包。

对接文档见本目录同名MD或[维护入口](../../../Development_Guide/BMI088原始与解析数据_树莓派MAVLink接口.md)。[BMI088_Pi_MAVLink_v1.zip](BMI088_Pi_MAVLink_v1.zip)含README、标准库解码器、机器字段表/真实C黄金帧及Python自测，可在既有唯一串口读取器中按payload_type接入。

32组选定C回归、17项Python测试通过；真实C生成的两条MAVLink外帧由Python验CRC、配对及复算六轴/负温度。9s共存测试发布900个100Hz源，BMI45个完整配对＋9状态；原24条GPS原帧全部完成、192片，队满/中止0、最大450ms，typed/ACK/echo/legacy/HB保留。

Keil唯一工程ARMCC5.06/C99 Rebuild All：关闭BMI上报20s、MPU兼容19s、默认双流19s，均0错误0警告，1043项输入各轮前后不变。默认源码/源码ZIP/同次固件一致。默认Code373624、RO111868、RW2044、ZI123236B；主SRAM125280B余5792B（较HUD父候选+384B），压缩后ROM485624B，无新任务/运行期堆分配。

build_matrix/verification记录覆盖宏和实际产物；production_inputs/production_source.zip为默认源码；changed_production列8项生产变更；host_summary/host_logs/host_tests_source为同次回归。备用关闭/MPU HEX仅在verification对应配置中，不与默认混用。旧归档及Latest保持。

下一步加载默认固件后核对Pi收到的三类帧、同源配对、原始复算、100Hz源/5Hz抽样计数、断流失效及GPS共存，并观察HUD/触摸、栈和长稳。软件通过不代替实板时序或精度验收。详见[开发日志](../../../Development_Guide/Change_History/2026-11-10_BMI088原始与解析MAVLink双流上报.md)。
