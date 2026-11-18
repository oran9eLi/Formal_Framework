#!/usr/bin/env python3
"""BMI088 UART6 MAVLink2参考解码器；只接收，不发送配置命令。

标准TUNNEL385承载0x8030原始寄存器采样、0x8031解析六轴及0x8032状态。
离线模式仅用Python标准库；--port显式启用时才需要可选pyserial。
原始字节用于审计，解析value仅在授权及接收端已知年龄范围内可用。
CRC不是身份认证；没有密钥时不验证MAVLink签名。
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import struct
import sys
import time
from collections import OrderedDict
from pathlib import Path

RAW_TYPE, PARSED_TYPE, STATUS_TYPE = 0x8030, 0x8031, 0x8032
KINDS = {RAW_TYPE: 0, PARSED_TYPE: 1, STATUS_TYPE: 2}
FRESH_MAX_MS = 200
CRC_EXTRAS = {0: 50, 1: 124, 24: 24, 29: 115, 30: 39, 33: 104,
              36: 222, 77: 143, 147: 154, 252: 44, 385: 147,
              12900: 114, 12901: 254, 12903: 249, 12904: 77, 12905: 49}


class ProtocolError(ValueError):
    """校验、结构、枚举或原始/解析对应关系不符合本地v1协议。"""


def require(condition, message):
    if not condition:
        raise ProtocolError(message)


def x25_crc(data, extra):
    """MAVLink X.25 CRC，包含CRC-extra，不含0xFD。"""
    crc = 0xFFFF
    for byte in bytes(data) + bytes((extra,)):
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def _u32(data, offset):
    return struct.unpack_from("<I", data, offset)[0]


def _axis_map_valid(axes):
    if sorted(abs(x) for x in axes) != [1, 2, 3]:
        return False
    absolute = list(map(abs, axes))
    sign = axes[0] * axes[1] * axes[2]
    sign *= (absolute[0] - absolute[1]) * (absolute[1] - absolute[2]) * (absolute[2] - absolute[0])
    return sign > 0


def _round_away(value):
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)


def _header(data, kind):
    require(len(data) == 64, "BMI088 inner payload must be exactly 64 bytes")
    version, actual_kind, flags, bus = data[:4]
    require(version == 1 and actual_kind == kind, "unsupported version or kind/type mismatch")
    require(not flags & 0xF0, "reserved BMI088 flag bit set")
    present, usable, stale, invalidated = [bool(flags & (1 << i)) for i in range(4)]
    seq, sample_ms, encode_ms, reinit = struct.unpack_from("<IIII", data, 4)
    delta = (encode_ms - sample_ms) & 0xFFFFFFFF
    age = delta if present and delta <= 0x7FFFFFFF else None
    require(not usable or (present and not stale and not invalidated and age is not None and age <= FRESH_MAX_MS), "invalid usable authorization")
    require(not stale or (present and (age is None or age > FRESH_MAX_MS)), "stale flag without an old/unknown-age sample")
    require(not (present and (age is None or age > FRESH_MAX_MS) and not stale), "old sample missing stale flag")
    require(not present or seq != 0, "present sample has zero sequence")
    require(present or (seq == 0 and sample_ms == 0 and not usable and not stale), "unknown sample has nonzero placeholder/flags")
    require(bus in ((0, 1, 2) if kind == 2 else (1, 2)), "unknown bus")
    require(kind == 2 or present, "raw/parsed packet cannot represent an invented absent sample")
    return {"version": version, "kind": kind, "flags": flags, "bus": bus,
            "present": present, "usable": usable, "stale": stale, "invalidated": invalidated,
            "sample_sequence": seq, "sample_time_ms": sample_ms, "encode_time_ms": encode_ms,
            "reinit_count": reinit, "encoded_age_ms": age}


def _addresses(bus, accel, gyro):
    require((bus == 1 and accel in (0x18, 0x19) and gyro in (0x68, 0x69)) or
            (bus == 2 and accel == gyro == 0), "address does not match selected bus")


def _raw(data, header):
    accel_addr, gyro_addr, accel_id, gyro_id, accel_status, fifo_status, frames, profile = data[20:28]
    _addresses(header["bus"], accel_addr, gyro_addr)
    require((accel_id, gyro_id) == (0x1E, 0x0F), "raw packet chip IDs do not match BMI088")
    require(profile == 1, "unknown BMI088 range/rate profile")
    require(accel_status & 0x80, "raw sample lacks accelerometer DRDY fact")
    require(1 <= frames <= 4 and fifo_status == frames, "FIFO count/overrun does not match complete raw burst")
    require(data[63] == 0 and not any(data[39 + frames * 6:63]), "nonzero reserved/unused gyro tail")
    accel_bytes, temp_bytes, gyro_bytes = data[28:37], data[37:39], data[39:63]
    require(temp_bytes[0] != 0x80, "invalid BMI088 temperature sample")
    accel = list(struct.unpack_from("<hhh", accel_bytes))
    gyro = [list(struct.unpack_from("<hhh", gyro_bytes, i * 6)) for i in range(frames)]
    temperature_raw = (temp_bytes[0] << 3) | (temp_bytes[1] >> 5)
    if temperature_raw >= 1024:
        temperature_raw -= 2048
    return {"profile": profile, "accel_addr_7bit": accel_addr, "gyro_addr_7bit": gyro_addr,
            "accel_chip_id": accel_id, "gyro_chip_id": gyro_id,
            "accel_status": accel_status, "gyro_fifo_status": fifo_status, "gyro_frame_count": frames,
            "accel_hex": accel_bytes.hex(), "temperature_hex": temp_bytes.hex(),
            "gyro_burst_hex": gyro_bytes[:frames * 6].hex(), "gyro_padded_hex": gyro_bytes.hex(),
            "audit_counts": {"accel_xyz": accel, "gyro_fifo_xyz": gyro, "temperature_11bit": temperature_raw,
                             "accel_sensor_time": int.from_bytes(accel_bytes[6:9], "little")},
            "audit_chip_values": {"accel_mg": [x * 12000 / 32768 for x in accel],
                                  "gyro_mdps": [x * 1000000 / 32768 for x in gyro[-1]],
                                  "temperature_cdeg": temperature_raw * 12.5 + 2300},
            "business_value": None, "audit_only": True}


def _parsed(data, header):
    accel = list(struct.unpack_from("<iii", data, 20))
    gyro = list(struct.unpack_from("<iii", data, 32))
    temperature, period, accel_range, gyro_range = struct.unpack_from("<hHHH", data, 44)
    axes = list(struct.unpack_from("<bbb", data, 52))
    require((accel_range, gyro_range) == (12000, 1000), "unsupported full-scale profile")
    require(_axis_map_valid(axes) and data[55] == 1, "invalid axis map/body frame")
    require(period != 0, "zero sample period")
    require(all(abs(x) <= accel_range for x in accel), "acceleration outside configured range")
    require(all(abs(x) <= gyro_range * 1000 for x in gyro), "angular rate outside configured range")
    require(-10488 <= temperature <= 15088, "temperature outside register conversion range")
    reported = {"accel_mg": accel, "gyro_mdps": gyro, "temperature_cdeg": temperature,
                "accel_g": [x / 1000 for x in accel], "gyro_dps": [x / 1000 for x in gyro],
                "temperature_c": temperature / 100}
    return {"reported": reported, "value": copy.deepcopy(reported) if header["usable"] else None,
            "sample_period_us": period, "accel_fullscale_mg": accel_range,
            "gyro_fullscale_dps": gyro_range, "axis_map": axes, "axis_frame": "body",
            "fifo_drop_count": _u32(data, 56), "driver_error_count": _u32(data, 60),
            "gyro_bias_applied": False, "display_zero_applied": False}


def _status(data, header):
    require(data[60] in (0, 1) and data[61] in (0, 1) and data[63] == 0, "invalid status availability/reserved byte")
    require(data[62] in (0, 1, 2, 3, 4, 5, 6, 255), "unknown Framework IMU state")
    diagnostics_available, source_available = bool(data[60]), bool(data[61])
    require(not source_available or header["present"], "source available without sample")
    if diagnostics_available:
        require(data[48] <= 9 and data[49] <= 6 and data[50] <= 3 and data[51] in (0, 1), "unknown driver/BSP enum")
        if data[51]:
            require((data[52], data[53]) == (0x1E, 0x0F), "ready status without both BMI088 IDs")
            _addresses(header["bus"], data[54], data[55])
    age = _u32(data, 56)
    require((header["encoded_age_ms"] is None and age == 0xFFFFFFFF) or
            (header["present"] and age == header["encoded_age_ms"]), "status source age mismatch")
    reported_counters = dict(zip(("driver_error_count", "fifo_drop_count", "source_published_count", "source_overwrites",
                         "completed_pairs", "abandoned_pairs", "tx_failures"), struct.unpack_from("<IIIIIII", data, 20)))
    counters = copy.deepcopy(reported_counters)
    if not diagnostics_available:
        counters.update(driver_error_count=None, fifo_drop_count=None)
    diagnostics = dict(zip(("driver_stage", "driver_result", "bsp_result", "device_ready", "accel_chip_id", "gyro_chip_id",
                            "accel_addr_7bit", "gyro_addr_7bit"), data[48:56])) if diagnostics_available else None
    return {"counters": counters, "reported_counters": reported_counters, "diagnostics_available": diagnostics_available,
            "diagnostics": diagnostics, "source_available": source_available,
            "source_age_ms": None if age == 0xFFFFFFFF else age,
            "imu_state": None if data[62] == 255 else data[62]}


def decode_payload(payload_type, payload):
    """解码TUNNEL.payload内层64B；调用者必须先校验MAVLink外层。"""
    require(payload_type in KINDS, "not a BMI088 payload type")
    data = bytes(payload)
    header = _header(data, KINDS[payload_type])
    body = (_raw, _parsed, _status)[header["kind"]](data, header)
    return {"event": ("bmi088_raw", "bmi088_parsed", "bmi088_status")[header["kind"]],
            "payload_type": payload_type, "header": header, "data": body,
            "payload_hex": data.hex()}


def decode_tunnel_payload(payload, sysid=1, compid=193, crc_verified=False):
    """解码MAVLink外层的TUNNEL字段布局，恢复MAVLink2省略的尾零。"""
    body = bytes(payload)
    require(len(body) <= 133, "invalid TUNNEL field length")
    body = body.ljust(133, b"\0")
    payload_type, target_system, target_component, length = struct.unpack_from("<HBBB", body)
    if payload_type not in KINDS:
        return None
    require(target_system == target_component == 0, "unexpected TUNNEL target")
    require(length == 64 and not any(body[69:]), "invalid BMI088 length/nonzero TUNNEL unused tail")
    event = decode_payload(payload_type, body[5:69])
    require(1 <= sysid <= 250 and compid == 193, "unexpected BMI088 source identity")
    event["mavlink"] = {"sysid": sysid, "compid": compid, "crc_valid": bool(crc_verified), "signature_verified": False}
    return event


def decode_tunnel(message, crc_verified=False):
    """接pymavlink的TUNNEL对象；仅在库已验CRC后显式传crc_verified=True。"""
    require(message.get_type() == "TUNNEL", "pymavlink message is not TUNNEL")
    payload = bytes(message.payload)
    require(len(payload) == 128, "pymavlink TUNNEL payload array must have 128 bytes")
    body = struct.pack("<HBBB", message.payload_type, message.target_system, message.target_component, message.payload_length) + payload
    return decode_tunnel_payload(body, message.get_srcSystem(), message.get_srcComponent(), crc_verified)


def decode_mavlink_frame(frame):
    """验证一个MAVLink2帧；未知消息不提升为BMI测量，签名仅识别长度。"""
    data = bytes(frame)
    require(len(data) >= 12 and data[0] == 0xFD, "not a complete MAVLink2 frame")
    require(not data[2] & ~1, "unsupported MAVLink incompatibility flags")
    length = data[1]
    require(len(data) == length + 12 + (13 if data[2] & 1 else 0), "MAVLink frame length mismatch")
    msgid = int.from_bytes(data[7:10], "little")
    extra = CRC_EXTRAS.get(msgid)
    verified = extra is not None
    if verified:
        require(x25_crc(data[1:10 + length], extra) == struct.unpack_from("<H", data, 10 + length)[0], "MAVLink CRC mismatch")
    if msgid == 385:
        event = decode_tunnel_payload(data[10:10 + length], data[5], data[6], True)
        if event is not None:
            event["mavlink"].update(sequence=data[4], signed=bool(data[2] & 1))
            return event
    return {"event": "ignored_message", "msgid": msgid, "mavlink": {"sysid": data[5], "compid": data[6],
            "sequence": data[4], "crc_valid": verified if extra is not None else None, "signature_verified": False}}


class Mavlink2StreamDecoder:
    """接收任意分片的MAVLink2流；坏帧后重新搜索0xFD，不执行串口写入。"""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data, final=False):
        self.buffer.extend(data)
        events = []
        while self.buffer:
            try:
                index = self.buffer.index(0xFD)
            except ValueError:
                self.buffer.clear()
                break
            del self.buffer[:index]
            if len(self.buffer) < 10:
                if final:
                    events.append({"event": "error", "error": "truncated MAVLink2 header"})
                    self.buffer.clear()
                break
            total = self.buffer[1] + 12 + (13 if self.buffer[2] & 1 else 0)
            if len(self.buffer) < total:
                if final:
                    events.append({"event": "error", "error": "truncated MAVLink2 frame"})
                    del self.buffer[0]
                    continue
                break
            frame = bytes(self.buffer[:total])
            try:
                event = decode_mavlink_frame(frame)
            except ProtocolError as exc:
                events.append({"event": "error", "error": str(exc)})
                msgid = int.from_bytes(frame[7:10], "little")
                extra = CRC_EXTRAS.get(msgid)
                crc_valid = extra is not None and not frame[2] & ~1 and x25_crc(frame[1:10 + frame[1]], extra) == struct.unpack_from("<H", frame, 10 + frame[1])[0]
                # 已验CRC的语义坏包保持其确定边界，不把寄存器里的0xFD当成新帧头。
                del self.buffer[:total if crc_valid else 1]
            else:
                events.append(event)
                del self.buffer[:total]
        return events


class PairAssembler:
    """有界原始/解析配对；两包同源同样本、内容相符才产生一组，不重复续期。"""
    def __init__(self, timeout_ms=1000, max_pending=16, max_recent=64):
        require(timeout_ms > 0 and max_pending > 0 and max_recent > 0, "invalid assembler limits")
        self.timeout_ms, self.max_pending, self.max_recent = timeout_ms, max_pending, max_recent
        self.pending, self.recent, self.sources, self._latest = OrderedDict(), OrderedDict(), {}, {}

    def reset_source(self, sysid, compid, reason="explicit session reset"):
        """收到连接重建/设备重启通知时调用；v1没有保证唯一的boot ID。"""
        source = (sysid, compid)
        for mapping in (self.pending, self.recent):
            for key in list(mapping):
                if key[:2] == source:
                    del mapping[key]
        self.sources.pop(source, None)
        self._latest.pop(source, None)
        return {"event": "bmi088_source_reset", "sysid": sysid, "compid": compid, "reason": reason}

    def _remember(self, key, value):
        self.recent[key] = value
        self.recent.move_to_end(key)
        while len(self.recent) > self.max_recent:
            self.recent.popitem(last=False)

    def expire(self, now_ms):
        events = []
        for key, state in list(self.pending.items()):
            if now_ms - state["first_ms"] > self.timeout_ms:
                del self.pending[key]
                self._remember(key, None)
                events.append({"event": "bmi088_pair_timeout", "identity": list(key), "received_kinds": sorted(state["parts"])})
        return events

    @staticmethod
    def _canonical(packet):
        data = bytes.fromhex(packet["payload_hex"])
        return data[:2] + data[3:12] + data[16:]  # flags/encode_now可随真实重试变更。

    def feed(self, packet, now_ms):
        events = self.expire(now_ms)
        if packet.get("event") not in ("bmi088_raw", "bmi088_parsed", "bmi088_status"):
            return events
        meta, header = packet.get("mavlink", {}), packet["header"]
        require(meta.get("crc_valid") is True, "pairing requires CRC-verified MAVLink input")
        source = (meta["sysid"], meta["compid"])
        previous = self.sources.get(source)
        if previous and (header["reinit_count"] < previous["reinit_count"] or
                         (header["encode_time_ms"] < previous["encode_time_ms"] and previous["encode_time_ms"] - header["encode_time_ms"] > self.timeout_ms) or
                         (header["present"] and previous.get("sample_sequence", 0) > 0xFFFF0000 and header["sample_sequence"] < 0x10000)):
            events.append(self.reset_source(*source, reason="uptime backstep or source counter reset/wrap"))
        self.sources[source] = {"reinit_count": header["reinit_count"], "encode_time_ms": header["encode_time_ms"],
                                "sample_sequence": header["sample_sequence"] if header["present"] else 0}
        if header["kind"] == 2:
            if not packet["data"]["source_available"] and source in self._latest:
                self._latest[source]["event"]["usable"] = False
                self._latest[source]["event"]["value"] = None
            return events
        key = source + (header["reinit_count"], header["sample_sequence"], header["sample_time_ms"])
        kind = header["kind"]
        if key in self.recent:
            prior = self.recent[key]
            if prior is not None and prior.get(kind) != self._canonical(packet):
                self._remember(key, None)
                if source in self._latest and tuple(self._latest[source]["event"]["identity"]) == key:
                    self._latest[source]["event"].update(usable=False, value=None)
                raise ProtocolError("conflicting completed sample identity")
            if not header["usable"] and source in self._latest and tuple(self._latest[source]["event"]["identity"]) == key:
                self._latest[source]["event"].update(usable=False, value=None)
            return events
        state = self.pending.get(key)
        if state is None:
            if len(self.pending) >= self.max_pending:
                old_key, _ = self.pending.popitem(last=False)
                self._remember(old_key, None)
                events.append({"event": "bmi088_pair_evicted", "identity": list(old_key)})
            state = {"first_ms": now_ms, "parts": {}}
            self.pending[key] = state
        if kind in state["parts"]:
            old = state["parts"][kind]
            if self._canonical(old["packet"]) != self._canonical(packet):
                del self.pending[key]
                self._remember(key, None)
                raise ProtocolError("conflicting duplicate sample part")
            old["usable"] = old["usable"] and header["usable"]
            old["age_ms"] = None if old["age_ms"] is None or header["encoded_age_ms"] is None else max(old["age_ms"], header["encoded_age_ms"])
        else:
            state["parts"][kind] = {"packet": copy.deepcopy(packet), "received_ms": now_ms,
                                    "usable": header["usable"], "age_ms": header["encoded_age_ms"]}
        if len(state["parts"]) != 2:
            return events
        del self.pending[key]
        self._remember(key, None)
        raw, parsed = (state["parts"][i]["packet"] for i in (0, 1))
        require(raw["header"]["bus"] == parsed["header"]["bus"], "sample bus mismatch")
        audit, report = raw["data"]["audit_chip_values"], parsed["data"]["reported"]
        axes = parsed["data"]["axis_map"]
        for field in ("accel_mg", "gyro_mdps"):
            expected = [_round_away(audit[field][abs(axis) - 1] * (1 if axis > 0 else -1)) for axis in axes]
            require(all(abs(a - b) <= 1 for a, b in zip(expected, report[field])), "raw/parsed six-axis values disagree")
        require(abs(_round_away(audit["temperature_cdeg"]) - report["temperature_cdeg"]) <= 1, "raw/parsed temperatures disagree")
        age = None if any(part["age_ms"] is None for part in state["parts"].values()) else max(part["age_ms"] + max(0, now_ms - part["received_ms"]) for part in state["parts"].values())
        usable = all(part["usable"] for part in state["parts"].values()) and age is not None and age <= FRESH_MAX_MS
        event = {"event": "bmi088_pair", "identity": list(key), "header": copy.deepcopy(parsed["header"]),
                 "usable": usable, "source_effective_age_ms": age,
                 "value": copy.deepcopy(report) if usable else None,
                 "raw": copy.deepcopy(raw["data"]), "parsed_reported": copy.deepcopy(report),
                 "parsed_metadata": {k: copy.deepcopy(v) for k, v in parsed["data"].items() if k not in ("value", "reported")},
                 "mavlink_crc_verified": True, "signature_verified": False,
                 "unique_boot_id_guaranteed": False, "hardware_synchronized": False}
        self._remember(key, {i: self._canonical(state["parts"][i]["packet"]) for i in (0, 1)})
        self._latest[source] = {"event": copy.deepcopy(event), "completed_ms": now_ms}
        events.append(event)
        return events

    def current(self, sysid, compid, now_ms):
        """取得本地最新组并按主机经过时间失效；无样本返回None，过期value为None。"""
        state = self._latest.get((sysid, compid))
        if state is None:
            return None
        event = copy.deepcopy(state["event"])
        if event["source_effective_age_ms"] is not None:
            event["source_effective_age_ms"] += max(0, now_ms - state["completed_ms"])
        if event["source_effective_age_ms"] is None or event["source_effective_age_ms"] > FRESH_MAX_MS:
            event.update(usable=False, value=None)
        return event


class Decoder:
    """字节流入口；输出单包及完整配对事件，状态包不混入六轴值。"""
    def __init__(self, emit_packets=True, pair_timeout_ms=1000):
        self.stream = Mavlink2StreamDecoder()
        self.assembler = PairAssembler(pair_timeout_ms)
        self.emit_packets = emit_packets

    def feed(self, data, now_ms=None, final=False):
        if now_ms is None:
            now_ms = time.monotonic_ns() // 1000000
        output = []
        for event in self.stream.feed(data, final):
            if event.get("event", "").startswith("bmi088_"):
                if self.emit_packets or event["event"] == "bmi088_status":
                    output.append(event)
                try:
                    output.extend(self.assembler.feed(event, now_ms))
                except ProtocolError as exc:
                    output.append({"event": "error", "error": str(exc), "channel": "bmi088"})
            elif event.get("event") == "error":
                output.append(event)
        output.extend(self.assembler.expire(now_ms))
        return output


def _main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--input", type=Path, help="离线MAVLink2二进制文件；默认stdin")
    source.add_argument("--hex", help="纯HEX文本，或@HEX文件")
    source.add_argument("--port", help="可选pyserial接收端口，本工具不写串口")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--pairs-only", action="store_true")
    args = parser.parse_args(argv)
    decoder = Decoder(not args.pairs_only)
    failed = False
    def emit(events):
        nonlocal failed
        for event in events:
            failed |= event.get("event") == "error"
            print(json.dumps(event, ensure_ascii=False, allow_nan=False), flush=True)
    if args.port:
        try:
            import serial  # 可选，仅显式串口模式导入。
        except ImportError:
            parser.error("串口模式需要pyserial，离线/嵌入式解码仅用标准库")
        with serial.Serial(args.port, args.baud, timeout=0.1, rtscts=False, dsrdtr=False) as port:
            try:
                while True:
                    emit(decoder.feed(port.read(4096)))
            except KeyboardInterrupt:
                return 0
    else:
        if args.hex is not None:
            text = Path(args.hex[1:]).read_text(encoding="utf-8-sig") if args.hex.startswith("@") else args.hex
            raw = bytes.fromhex(text)
            emit(decoder.feed(raw, now_ms=0, final=True))
        else:
            stream = args.input.open("rb") if args.input else sys.stdin.buffer
            try:
                while True:
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    emit(decoder.feed(chunk))
                emit(decoder.feed(b"", final=True))
            finally:
                if args.input:
                    stream.close()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(_main())
