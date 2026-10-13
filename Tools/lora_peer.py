#!/usr/bin/env python3
"""CNS M5 手动 LoRa 对端。协议元数据取自本仓库，默认只监听。

电脑经 USB-TTL 和第二块 E22 接入射频链路。串口写入完成不证明箱子收到。
只需 pyserial；generate 不需要串口依赖。示例消息不会发电机控制命令。
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import struct
import sys
import time

CATALOG = (
    "HEARTBEAT", "SYS_STATUS", "GPS_RAW_INT", "ATTITUDE", "GLOBAL_POSITION_INT",
    "SCALED_PRESSURE", "BATTERY_STATUS", "SERVO_OUTPUT_RAW", "NAMED_VALUE_INT",
    "TUNNEL", "COMMAND_LONG", "COMMAND_ACK", "OPEN_DRONE_ID_BASIC_ID",
)
FORMATS = {"CHAR": "c", "UINT8_T": "B", "INT8_T": "b", "UINT16_T": "H",
           "INT16_T": "h", "UINT32_T": "I", "INT32_T": "i", "UINT64_T": "Q",
           "INT64_T": "q", "FLOAT": "f", "DOUBLE": "d"}


def x25(data: bytes, initial: int = 0xFFFF) -> int:
    """MAVLink X.25 CRC，额外CRC由调用方作为最后一个字节加入。"""
    crc = initial
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class Dialect:
    """读取工程内生成的common方言，不使用另一个版本的协议目录。"""
    def __init__(self, root: Path):
        self.root = root / "Third_Party" / "mavlink" / "common"
        common = (self.root / "common.h").read_text(encoding="utf-8")
        table = re.search(r"#define MAVLINK_MESSAGE_CRCS (.*)", common).group(1)
        self.crcs = {int(a): int(b) for a, b in re.findall(r"\{(\d+),\s*(\d+),", table)}
        self.cache = {}

    def message(self, name: str):
        if name in self.cache:
            return self.cache[name]
        path = self.root / ("mavlink_msg_" + name.lower() + ".h")
        if not path.is_file():
            candidates = list(self.root.parent.rglob(path.name))
            if len(candidates) != 1: raise ValueError("Missing/ambiguous inherited message: "+name)
            path = candidates[0]
        text = path.read_text(encoding="utf-8")
        number = int(re.search(r"#define MAVLINK_MSG_ID_" + name + r" (\d+)", text).group(1))
        maximum = int(re.search(r"#define MAVLINK_MSG_ID_" + name + r"_LEN (\d+)", text).group(1))
        minimum = int(re.search(r"#define MAVLINK_MSG_ID_" + name + r"_MIN_LEN (\d+)", text).group(1))
        fields = {}
        for field, kind, count, offset in re.findall(
                r'\{\s*"([^"]+)",\s*(?:NULL|"[^"]*"),\s*MAVLINK_TYPE_(\w+),\s*(\d+),\s*(\d+),\s*offsetof', text):
            fields[field] = (FORMATS[kind], int(count), int(offset))
        result = number, minimum, maximum, fields
        self.cache[name] = result
        return result

    def payload(self, name: str, values: dict) -> bytes:
        """逐字段小端编码；不发送宿主C/Python对象的内存布局。"""
        _, _, maximum, fields = self.message(name)
        result = bytearray(maximum)
        for name, value in values.items():
            fmt, count, offset = fields[name]
            if count:
                if isinstance(value, (bytes, bytearray, str)):
                    value = value.encode("ascii") if isinstance(value, str) else bytes(value)
                    if len(value) > count or fmt not in ("c", "B"):
                        raise ValueError("invalid byte array: " + name)
                    result[offset:offset+count] = value.ljust(count, b"\0")
                else:
                    if len(value) != count:
                        raise ValueError("array length mismatch: " + name)
                    struct.pack_into("<" + fmt * count, result, offset, *value)
            else:
                struct.pack_into("<" + fmt, result, offset, value)
        return bytes(result)


def example(name: str, elapsed_ms: int, peer_id: str) -> dict:
    """课程用确定性示例；COMMAND_LONG使用未知命令65535，禁止操纵电机。"""
    t = elapsed_ms / 1000.0
    examples = {
        "HEARTBEAT": dict(type=2, autopilot=8, base_mode=0, custom_mode=0,
                          system_status=3, mavlink_version=3),
        "SYS_STATUS": dict(onboard_control_sensors_present=7, onboard_control_sensors_enabled=7,
                           onboard_control_sensors_health=7, load=120, voltage_battery=12000,
                           current_battery=0, battery_remaining=80),
        "GPS_RAW_INT": dict(time_usec=elapsed_ms*1000, fix_type=3, lat=225430000,
                            lon=1140570000, alt=50000, eph=100, epv=150, vel=0,
                            cog=65535, satellites_visible=10),
        "ATTITUDE": dict(time_boot_ms=elapsed_ms, roll=0.2*math.sin(t), pitch=0.1*math.cos(t),
                         yaw=0.3*t, rollspeed=0.2*math.cos(t), pitchspeed=-0.1*math.sin(t), yawspeed=0.3),
        "GLOBAL_POSITION_INT": dict(time_boot_ms=elapsed_ms, lat=225430000,
                                    lon=1140570000, alt=50000, relative_alt=0,
                                    vx=0, vy=0, vz=0, hdg=65535),
        "SCALED_PRESSURE": dict(time_boot_ms=elapsed_ms, press_abs=1013.25,
                                press_diff=0.0, temperature=2500),
        "BATTERY_STATUS": dict(id=0, battery_function=1, type=3, temperature=32767,
                               voltages=[12000]+[65535]*9, current_battery=0,
                               current_consumed=-1, energy_consumed=-1, battery_remaining=80),
        "SERVO_OUTPUT_RAW": dict(time_usec=elapsed_ms*1000, port=0, servo1_raw=1000,
                                 servo2_raw=1000, servo3_raw=1000, servo4_raw=1000),
        "NAMED_VALUE_INT": dict(time_boot_ms=elapsed_ms, name="PC_TEST", value=int(100*math.sin(t))),
        "TUNNEL": dict(target_system=0, target_component=0, payload_type=0x8000,
                       payload_length=12, payload=b"CNS-PC-\xfe\xfd-01"),
        "COMMAND_LONG": dict(target_system=0, target_component=0, command=65535, confirmation=0),
        "COMMAND_ACK": dict(command=65535, result=3),
        "OPEN_DRONE_ID_BASIC_ID": dict(target_system=0, target_component=0,
                                       id_or_mac=peer_id, id_type=1, ua_type=2, uas_id=peer_id),
        "ENCAPSULATED_DATA": dict(seqnr=0, data=bytes([0xFE, 0xFD])+bytes(range(1, 251))+b"\x01"),
    }
    return examples[name]


def frame(dialect: Dialect, name: str, values: dict, seq: int, sysid: int, compid: int,
          version: int = 2, signed_placeholder: bool = False) -> bytes:
    """签名占位只覆盖原字节观察，不表示签名验证通过。"""
    msgid, minimum, _, _ = dialect.message(name)
    payload = dialect.payload(name, values)
    if version == 1:
        if msgid > 255 or signed_placeholder:
            raise ValueError("MAVLink 1 cannot carry this message/signature")
        payload = payload[:minimum]
        header = bytes([len(payload), seq & 255, sysid, compid, msgid])
        prefix = b"\xfe"
    else:
        payload = payload.rstrip(b"\0") or b"\0"
        header = bytes([len(payload), int(signed_placeholder), 0, seq & 255, sysid, compid]) + msgid.to_bytes(3, "little")
        prefix = b"\xfd"
    crc = x25(header + payload + bytes([dialect.crcs[msgid]]))
    result = prefix + header + payload + struct.pack("<H", crc)
    if signed_placeholder:
        result += bytes([1]) + (1).to_bytes(6, "little") + b"\0"*6
    return result


class Parser:
    """有界流重组；原始接收字节由Log独立保存，解析丢弃不抹去证据。"""
    def __init__(self, dialect, callback):
        self.dialect, self.callback = dialect, callback
        self.buffer = bytearray()
        self.last_byte = 0.0

    def expire(self, now):
        if self.buffer and now - self.last_byte > 2.0:
            self.callback("rx_partial", hex=self.buffer.hex(" "), reason="idle timeout")
            self.buffer.clear()

    def feed(self, data, now):
        self.expire(now)
        self.last_byte = now
        self.buffer.extend(data)
        if len(self.buffer) > 4096:
            self.callback("parser_overflow", bytes=len(self.buffer))
            del self.buffer[:-280]
        while self.buffer:
            if self.buffer[0] not in (0xFD, 0xFE):
                del self.buffer[0]
                continue
            version = 2 if self.buffer[0] == 0xFD else 1
            header_length = 10 if version == 2 else 6
            if len(self.buffer) < header_length:
                return
            signature = version == 2 and bool(self.buffer[2] & 1)
            payload_length = self.buffer[1]
            length = header_length + payload_length + 2 + (13 if signature else 0)
            if len(self.buffer) < length:
                return
            candidate = bytes(self.buffer[:length])
            msgid = int.from_bytes(candidate[7:10], "little") if version == 2 else candidate[5]
            seq, sysid, compid = candidate[4:7] if version == 2 else candidate[2:5]
            crc_extra = self.dialect.crcs.get(msgid)
            received_crc = int.from_bytes(candidate[header_length+payload_length:header_length+payload_length+2], "little")
            crc_valid = None if crc_extra is None else received_crc == x25(candidate[1:header_length+payload_length]+bytes([crc_extra]))
            self.callback("rx_frame", msgid=msgid, seq=seq, sysid=sysid, compid=compid,
                          mavlink_version=version, bytes=length, crc_valid=crc_valid,
                          signature_present=signature, signature_verified=False, hex=candidate.hex(" "))
            del self.buffer[:length if crc_valid is not False else 1]


class Log:
    """JSONL事件与RX/TX完整二进制证据；按执行时长限量，不在RAM累计历史。"""
    def __init__(self, directory):
        directory.mkdir(parents=True, exist_ok=False)
        self.events = (directory/"events.jsonl").open("w", encoding="utf-8")
        self.rx = (directory/"rx.bin").open("wb")
        self.tx = (directory/"tx.bin").open("wb")
        self.start = time.monotonic()

    def event(self, kind, **fields):
        record = dict(event=kind, utc=datetime.now(timezone.utc).isoformat(),
                      elapsed_ms=round((time.monotonic()-self.start)*1000), **fields)
        self.events.write(json.dumps(record, ensure_ascii=False)+"\n")
        self.events.flush()
        if kind in ("rx_frame", "tx_frame", "error", "rx_partial", "summary"):
            brief = {k: v for k, v in record.items() if k not in ("hex", "utc")}
            print(json.dumps(brief, ensure_ascii=False), flush=True)

    def raw(self, direction, data):
        handle = self.rx if direction == "rx" else self.tx
        handle.write(data)
        handle.flush()
        self.event(direction+"_chunk", bytes=len(data), hex=data.hex(" "))

    def close(self):
        self.events.close(); self.rx.close(); self.tx.close()


def run(args):
    dialect = Dialect(Path(__file__).resolve().parents[1])
    if not 1 <= args.sysid <= 250 or not 1 <= args.compid <= 255:
        raise ValueError("sysid 1..250; compid 1..255")
    if not 1 <= len(args.peer_id.encode("ascii")) <= 20:
        raise ValueError("peer-id must be 1..20 ASCII bytes")
    if not math.isfinite(args.duration) or not math.isfinite(args.interval) or args.duration <= 0 or args.count < 0 or args.interval < 0.05:
        raise ValueError("duration > 0; count >= 0; interval >= 0.05s")
    if args.case == "signed-placeholder" and args.mav_version != 2:
        raise ValueError("signed-placeholder requires MAVLink 2")
    names = CATALOG if args.message == "catalog" else (args.message,)
    if args.mode == "peer":
        names = ("HEARTBEAT", "OPEN_DRONE_ID_BASIC_ID", "ATTITUDE", "GPS_RAW_INT")
    for name in names:
        dialect.message(name)
        if args.mav_version == 1 and dialect.message(name)[0] > 255:
            raise ValueError("MAVLink 1: select a message ID below 256")
    log = Log(args.output)
    serial_port = None
    generated = None
    parser = Parser(dialect, log.event)
    sent, received = 0, 0
    started = time.monotonic()
    try:
        log.event("configuration", arguments={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                  dialect="repository common.h", tx_meaning="PC UART accepted; RF reception unconfirmed")
        if args.mode == "generate":
            generated = (args.output/"frames.hex").open("w", encoding="ascii")
        else:
            if not args.port:
                raise ValueError("--port is required except for generate")
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("Install dependency: python -m pip install pyserial") from exc
            serial_port = serial.Serial(args.port, args.baud, timeout=0, write_timeout=2,
                                        bytesize=8, parity="N", stopbits=1)
        next_send = started
        while time.monotonic()-started < args.duration:
            now = time.monotonic()
            if args.mode != "listen" and (args.count == 0 or sent < args.count) and now >= next_send:
                name = names[sent % len(names)]
                elapsed = int((now-started)*1000)
                raw = frame(dialect, name, example(name, elapsed, args.peer_id),
                            sent, args.sysid, args.compid, args.mav_version, args.case == "signed-placeholder")
                if args.case == "bad-crc":
                    raw = raw[:-1]+bytes([raw[-1] ^ 0x80])
                elif args.case == "truncated":
                    raw = raw[:max(3, len(raw)//2)]
                elif args.case == "noise":
                    raw = b"CNS LoRa ASCII noise\r\n"
                if serial_port:
                    count = serial_port.write(raw)
                    log.raw("tx", raw[:count])
                    if count != len(raw):
                        raise RuntimeError("partial PC UART write")
                else:
                    generated.write(raw.hex(" ").upper()+"\n"); generated.flush()
                    log.raw("tx", raw)
                log.event("tx_frame", name=name, seq=sent & 255, bytes=len(raw), case=args.case,
                          signature_verified=False, hex=raw.hex(" "))
                sent += 1
                next_send = now + args.interval
                if args.mode == "generate" and args.count and sent >= args.count:
                    break
            if serial_port:
                raw = serial_port.read(min(max(serial_port.in_waiting, 1), 1024))
                if raw:
                    received += len(raw); log.raw("rx", raw); parser.feed(raw, now)
                parser.expire(now)
            time.sleep(0.005)
    except KeyboardInterrupt:
        log.event("stopped", reason="Ctrl+C")
    except Exception as exc:
        log.event("error", message=str(exc))
        raise
    finally:
        if serial_port: serial_port.close()
        if generated: generated.close()
        log.event("summary", transmitted_frames=sent, received_bytes=received,
                  acceptance="pending board inspection and matching evidence")
        log.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", help="例如COM7；USB-TTL连接第二块E22")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--mode", choices=("listen", "send", "peer", "generate"), default="listen")
    p.add_argument("--message", choices=CATALOG+("catalog", "ENCAPSULATED_DATA"), default="HEARTBEAT")
    p.add_argument("--case", choices=("normal", "bad-crc", "truncated", "noise", "signed-placeholder"), default="normal")
    p.add_argument("--mav-version", type=int, choices=(1, 2), default=2)
    p.add_argument("--sysid", type=int, default=77)
    p.add_argument("--compid", type=int, default=191)
    p.add_argument("--peer-id", default="DCDWCNS1PC0000000001")
    p.add_argument("--count", type=int, default=1, help="0表示持续发送，仍受duration限制")
    p.add_argument("--interval", type=float, default=2.0, help="手动选取发送间隔，秒")
    p.add_argument("--duration", type=float, default=60.0, help="最多执行秒数，Ctrl+C可停止")
    p.add_argument("--output", type=Path, default=Path("build")/"lora_peer"/datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
    try:
        run(p.parse_args())
    except (ValueError, RuntimeError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
