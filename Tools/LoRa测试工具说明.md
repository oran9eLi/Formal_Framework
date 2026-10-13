# CNS M5 电脑 LoRa 对端工具

## 准备

- Python 3.10+；连接真实串口时安装：`python -m pip install pyserial`。
- 电脑 → USB-TTL → **第二块 E22** → 无线 → 箱内 E22。普通串口助手可以发送生成的 HEX，但直接接箱子 UART 只验证串口。
- USB-TTL TX 接第二块 E22 RX，RX 接 TX，GND 共地。模块电源、天线、逻辑电平和正常工作模式按实物说明接好；电脑工具不配置 E22 寄存器。
- 两端读回课堂预设：UART 9600 8N1、空速 9600 bit/s、透明传输；地址/NETID/分包/RSSI与当前箱子配置一致，Channel 同为 23/26/29/32 中所选值。厂商默认配置不等于课堂预设。箱子显示 Profile mismatch 时先人工修正。
- 工具默认 sysid=77、compid=191、身份 DCDWCNS1PC0000000001；与箱子 Node ID 冲突时手动换 sysid。多组实验手动分配不同信道或错开发送。

在仓库根目录运行。以下 COM7 仅为示例，换成设备管理器中的实际串口；同一串口同时只由一个程序打开。`--help`可查看所有参数。

## 1. 先观察箱子发给电脑的数据

```powershell
python Tools/lora_peer.py --port COM7 --mode listen --duration 60
```

在箱子 Comm 页手动 Send 或 Start。工具默认只监听，不主动开启实验。监听会保存原始 RX 字节及解析结果；箱子的 Queued/UART done/Module idle 只表示本机阶段，用电脑的 RX 记录证明远端接收。

## 2. 电脑发送一帧，再发送目录

```powershell
python Tools/lora_peer.py --port COM7 --mode send --message HEARTBEAT --count 1
python Tools/lora_peer.py --port COM7 --mode send --message catalog --count 13 --interval 2 --duration 35
```

箱子进入 Comm → Inspect，选 LoRa RX，按 Start。目录包含 HEARTBEAT、SYS_STATUS、GPS_RAW_INT、ATTITUDE、GLOBAL_POSITION_INT、SCALED_PRESSURE、BATTERY_STATUS、SERVO_OUTPUT_RAW、NAMED_VALUE_INT、TUNNEL、COMMAND_LONG、COMMAND_ACK、OPEN_DRONE_ID_BASIC_ID。

Messages 的计数/Avg Hz/Age按本捕获会话更新。Next选汇总行，Select复制该键最近帧，再看 Raw/Decoded。固定4槽可能覆盖其他键的最近帧，此时显示 unavailable；重新发该消息再Select。选帧字节不会被后续覆盖，TX槽仍在时阶段可更新，已覆盖会标注。Start新建会话并清旧统计，Clear会暂停并换会话，不清生产数据。TX Count为接口已接受帧；Attempt/Timeout/Error独立，失败Raw可选，不能用Count证明远端收到。

消息是用于观察的样例数据；COMMAND_LONG使用未支持的命令65535，没有电机控制动作。目录中协议字段按仓库内MAVLink元数据编码，浮点值、合法零和unknown哨兵均保留。CSV/实际传感器结果不能用这些样例替代。

## 3. 作为一个远端设备发送

```powershell
python Tools/lora_peer.py --port COM7 --mode peer --sysid 77 --count 0 --interval 2 --duration 60
```

按顺序循环 Heartbeat、Basic ID、Attitude、GNSS；默认每8秒一轮，低于当前20秒心跳过期门限。箱子在 Remote 只读页选择该对端。此工具模拟教学所需数据，不能代替另一箱子的传感器/电调保护或全部设备交互验收。count=0仍受duration上限，Ctrl+C可以结束。工具不会替学生启动箱子的连续发送。

## 4. 异常输入与完整帧边界

```powershell
python Tools/lora_peer.py --port COM7 --mode send --message ATTITUDE --case bad-crc
python Tools/lora_peer.py --port COM7 --mode send --message ATTITUDE --case truncated
python Tools/lora_peer.py --port COM7 --mode send --case noise
python Tools/lora_peer.py --port COM7 --mode send --message ATTITUDE
python Tools/lora_peer.py --port COM7 --mode send --message ATTITUDE --mav-version 1
python Tools/lora_peer.py --port COM7 --mode send --message ENCAPSULATED_DATA --case signed-placeholder
```

每次由学生手动发起。坏CRC/截断/噪声不应成为合法Decoded业务记录；后续正常帧应恢复。Rejected统计仅统计所选入口完成的CRC/签名拒绝，未收齐截断帧未必立即增加Rejected，生产解析器的错误/溢出计数另看。

最后一条生成完整280 B的MAVLink2帧，载荷含FE/FD，附13 B占位签名；用于Raw边界、分包及签名字节存在检查。**没有密钥，工具和屏幕均不认证签名**。ENCAPSULATED_DATA不在13项解码目录，预期显示 Unsupported并保留Raw。2秒默认间隔适合首轮低速试验，压测间隔由人工选择并记录，工具不宣称空口吞吐达标。

## 5. 无串口时生成HEX，供串口助手复制

```powershell
python Tools/lora_peer.py --mode generate --message catalog --count 13 --interval 0.05 --duration 5 --output build/lora_peer/catalog
```

不需要pyserial。`frames.hex`一行一帧，在串口助手用HEX发送；使用整行，不加ASCII行尾。0.05秒仅为离线生成间隔。已生成的样例可见 [LoRa目录样例帧](LoRa目录样例帧.hex)。

## 证据与当前限制

默认输出位于 `build/lora_peer/时间戳/`：

| 文件 | 内容 |
|---|---|
| rx.bin / tx.bin | 完整原始字节流，包括噪声和未收齐片段 |
| events.jsonl | UTC与相对时间、参数、帧HEX、msg/sys/comp/seq、CRC/签名字节状态及错误 |
| frames.hex | generate模式生成的HEX |

提交验收时记录箱子HEX SHA、板号、双方E22型号/读回参数、Node ID、Channel、接线、时长与电脑证据目录。串口write成功只证明PC端接受字节，不能证明射频发送或箱子接收。脚本离线生成/语法检查不等于实测；M5 C03—C06及M4无线必需项仍待实板。
