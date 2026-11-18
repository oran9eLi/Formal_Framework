# GPS原始帧独立上报候选

GPS到Pi仅MAVLink2 TUNNEL385 / payload_type0x8020；解析继续供屏幕和本地。Pi旧GPS24/33/GNSS_SAT/GNSSUTC/BAROALT、typed8011..14、8021及RID位置/时间镜像停发；非GPS遥测、LoRa和硬件RemoteID广播保留。

40组选定回归与Keil全量0错误0警告、20s通过；1034项当前输入、源码ZIP及固件一致。HEX SHA：`5d2e85ea347dd62952ebc8c7913548a378844b89a05d08e3cb43a6f9e38f30e4`。本候选尚未烧录/实际Pi接收及长稳，既有debug堆预算超75%问题保留。

当前接口只维护随包GPS原始帧_树莓派MAVLink接口.md；旧归档和Latest保持。
