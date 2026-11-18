#!/usr/bin/env python3
"""UART6统一GNSS参考解码器；默认离线，串口模式仅接收、不发送命令。

冻结协议v1：标准MAVLink2 TUNNEL(385/CRC-extra147)封装typed0x8011..0x8014，
独立RAW_FRAME0x8020/RAW_STATS0x8021；RAW事件不进入typed导航缓存。
标准库支持二进制文件或HEX；只有显式--port才导入可选pyserial。
签名字节参与帧长度，但无密钥时不验证签名；CRC通过不等于认证通过。
所有工程量受present/valid/stale/source/age授权，无效为None/JSON null。
"""
from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import struct
import sys
import time
from collections import OrderedDict
from pathlib import Path

VERSION = 1
FRESH_MAX_MS = 1500
AGE_UNKNOWN = 0xFFFF
AGE_SATURATED = 0xFFFE
SOURCES = {0: "NONE", 1: "UBX_PVT", 2: "NMEA_GGA", 3: "NMEA_RMC",
           4: "NMEA_GSA", 5: "NMEA_GSV", 6: "UBX_DOP"}
DOMAIN_NAMES = ("position", "velocity", "time", "altitude", "navigation_status", "hdop")
DOP_NAMES = ("GDOP", "PDOP", "TDOP", "VDOP", "HDOP", "NDOP", "EDOP")
COUNT_NAMES = ("gps_visible", "bds_visible", "gps_used", "bds_used")
NAV_SOURCES = ({0, 1, 2}, {0, 1, 3}, {0, 1, 3}, {0, 1, 2}, {0, 1, 2}, {0, 2, 4, 6})
DOP_SOURCES = ({0, 6}, {0, 1, 4, 6}, {0, 6}, {0, 4, 6}, {0, 2, 4, 6}, {0, 6}, {0, 6})
LOCAL_TYPES = {0x8011: (1, "GNSS_NAV", 64, 1), 0x8012: (2, "GNSS_DOP", 64, 2),
               0x8013: (3, "GNSS_COUNTS", 32, 1), 0x8014: (4, "GNSS_LINK", 64, 2)}
RAW_FRAME_TYPE, RAW_STATS_TYPE = 0x8020, 0x8021
RAW_PROTOCOLS = {1: "NMEA", 2: "UBX"}  # 与typed SOURCES不同的独立命名空间。
RAW_MAX_FRAME, RAW_MAX_CHUNK = 1024, 40
# 兼容旧目录仅保留/识别，不把旧GNSS帧提升成新权威快照。
LEGACY = {0: ("HEARTBEAT", 50), 1: ("SYS_STATUS", 124), 24: ("GPS_RAW_INT", 24),
          29: ("SCALED_PRESSURE", 115), 30: ("ATTITUDE", 39), 33: ("GLOBAL_POSITION_INT", 104),
          36: ("SERVO_OUTPUT_RAW", 222), 77: ("COMMAND_ACK", 143), 147: ("BATTERY_STATUS", 154),
          252: ("NAMED_VALUE_INT", 44), 385: ("TUNNEL", 147),
          12900: ("OPEN_DRONE_ID_BASIC_ID", 114), 12901: ("OPEN_DRONE_ID_LOCATION", 254),
          12903: ("OPEN_DRONE_ID_SELF_ID", 249), 12904: ("OPEN_DRONE_ID_SYSTEM", 77),
          12905: ("OPEN_DRONE_ID_OPERATOR_ID", 49)}


class ProtocolError(ValueError):
    """结构、校验、版本、标志或授权范围不符合冻结协议。"""


def require(condition, message):
    if not condition:
        raise ProtocolError(message)


def x25_crc(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for byte in bytes(data) + bytes((extra,)):
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def crc16_ccitt_false(data: bytes) -> int:
    """完整GPS wire封装校验；poly1021/initFFFF/refin=false/xorout0。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def _u16(data, offset):
    return struct.unpack_from("<H", data, offset)[0]


def _u32(data, offset):
    return struct.unpack_from("<I", data, offset)[0]


def _i32(data, offset):
    return struct.unpack_from("<i", data, offset)[0]


def _source(value):
    require(value in SOURCES, "unknown source enum")
    return SOURCES[value] if value else None


def _utc(date, seconds):
    require(2000 <= date // 10000 <= 2099, "UTC year outside v1 range")
    require(0 <= seconds < 86400, "UTC second outside day")
    try:
        day = dt.date(date // 10000, date // 100 % 100, date % 100)
    except ValueError as exc:
        raise ProtocolError("invalid UTC calendar date") from exc
    return {"date_yyyymmdd": date, "second_of_day": seconds,
            "iso8601": f"{day.isoformat()}T{seconds//3600:02}:{seconds//60%60:02}:{seconds%60:02}Z"}


def _mask_check(present, valid, stale, allowed):
    require(not ((present | valid | stale) & ~allowed), "reserved mask bit set")
    require(not (valid & ~present) and not (stale & ~present), "valid/stale without present")
    require(not (valid & stale), "valid and stale overlap")


def _domain(present, valid, stale, source, age, value):
    name = _source(source)
    require(not present or source != 0, "present domain without source")
    require(not valid or age <= FRESH_MAX_MS, "valid domain has unknown/old age")
    require(not stale or age == AGE_UNKNOWN or age > FRESH_MAX_MS, "stale domain has fresh age")
    require(not (present and age > FRESH_MAX_MS and not stale), "old present domain missing stale flag")
    state = "UNKNOWN" if not present else "STALE" if stale else "VALID" if valid else "INVALID"
    return {"present": bool(present), "valid": bool(valid), "stale": bool(stale), "state": state,
            "source_id": source, "source": name, "encoded_age_ms": None if age == AGE_UNKNOWN else age,
            "age_ms": None if age == AGE_UNKNOWN else age, "age_saturated": age == AGE_SATURATED,
            "value": value if valid else None}


def _nav(data):
    valid, stale, present = data[12], data[13], data[62]
    _mask_check(present, valid, stale, 0x3F)
    packed = _u16(data, 14)
    require(not packed & 0x8000, "reserved source-pack bit set")
    sources = [(packed >> (3 * i)) & 7 for i in range(5)] + [data[19]]
    for index, source in enumerate(sources):
        _source(source)
        require(source in NAV_SOURCES[index], "source not allowed for NAV domain")
    quality, dimension, satellites, raw_type = data[16], data[17], data[18], data[63]
    require(quality <= 8 and dimension in (0, 2, 3), "invalid fix quality/dimension")
    lat, lon, altitude = _i32(data, 20), _i32(data, 24), _i32(data, 28)
    speed, course, hdop = _u32(data, 32), _u16(data, 36), _u16(data, 38)
    date, second = _u32(data, 40), _u32(data, 44)
    if valid & 1:
        require(-900000000 <= lat <= 900000000 and -1800000000 <= lon <= 1800000000, "coordinate outside range")
    if valid & 2:
        require(course < 36000, "course outside 0..35999")
    time_value = _utc(date, second) if valid & 4 else None
    if valid & 16:
        if sources[4] == 1:
            require(raw_type <= 5, "invalid PVT fixType")
        elif sources[4] == 2:
            require(raw_type <= 8, "invalid GGA raw status")
        require(not (quality == 0 and valid & 1), "NoFix status contradicts valid position")
    if valid & 32:
        require(1 <= hdop <= 65534, "usable HDOP outside range")
    values = [{"latitude_e7": lat, "longitude_e7": lon, "latitude_deg": lat / 1e7, "longitude_deg": lon / 1e7},
              {"ground_speed_cms": speed, "ground_speed_mps": speed / 100, "course_cdeg": course, "course_deg": course / 100},
              time_value, {"altitude_msl_mm": altitude, "altitude_msl_m": altitude / 1000},
              {"fix_quality_gga": quality, "fix_dimension": dimension if quality and dimension else None,
               "satellites_used": satellites, "raw_status_type": raw_type}, {"x100": hdop, "factor": hdop / 100}]
    result = {}
    for i, name in enumerate(DOMAIN_NAMES):
        domain = _domain(present & (1 << i), valid & (1 << i), stale & (1 << i), sources[i], _u16(data, 48 + 2*i), values[i])
        if name == "navigation_status" and domain["valid"] and quality == 0:
            domain["state"] = "NO_FIX"
        result[name] = domain
    age = _u16(data, 60)
    result["link_age_ms"] = None if age == AGE_UNKNOWN else age
    result["link_age_saturated"] = age == AGE_SATURATED
    return result


def _dop(data, part):
    count = (4, 3)[part]
    require(data[12] == count, "wrong DOP entry count")
    require(not any(data[13:16]), "nonzero DOP reserved header")
    require(not any(data[16 + 12*count:64]), "nonzero DOP unused tail")
    entries = []
    for row in range(count):
        offset = 16 + 12*row
        ident, source, flags = data[offset:offset+3]
        require(ident == part*4 + row, "wrong DOP id/order")
        _source(source)
        require(source in DOP_SOURCES[ident], "source not allowed for DOP id")
        require(not flags & ~15 and data[offset+3] == 0, "reserved DOP flags/byte")
        present, valid, stale, known = bool(flags & 1), bool(flags & 2), bool(flags & 4), bool(flags & 8)
        _mask_check(int(present), int(valid), int(stale), 1)
        reported, age, itow = _u16(data, offset+4), _u16(data, offset+6), _u32(data, offset+8)
        require(not present or 1 <= reported <= 65534, "reported DOP outside 1..65534")
        require(not known or (present and source in (1, 6) and itow < 604800000), "invalid DOP iTOW/source")
        require(known or itow == 0, "unknown iTOW must use zero wire placeholder")
        entry = _domain(present, valid, stale, source, age, reported/100)
        if present and not valid and not stale:
            entry["state"] = "REPORTED_ONLY"
        entry.update({"id": ident, "name": DOP_NAMES[ident], "reported_x100": reported if present else None,
                      "reported_factor": reported/100 if present else None,
                      "itow_known": known, "itow_ms": itow if known else None})
        entries.append(entry)
    return {"entries": entries}


def _counts(data):
    present, valid, stale = data[12:15]
    _mask_check(present, valid, stale, 15)
    require(data[15] == 0, "nonzero COUNTS reserved byte")
    result = {}
    for i, name in enumerate(COUNT_NAMES):
        value, source, age = data[16+i], data[20+i], _u16(data, 24+2*i)
        require(source in ({0, 5} if i < 2 else {0, 4}), "source not allowed for COUNTS field")
        result[name] = _domain(present & (1 << i), valid & (1 << i), stale & (1 << i), source, age, value)
    return result


def _link(data, part):
    if part == 0:
        flags = data[12]
        require(not flags & 0x80, "reserved LINK diagnostic flag")
        require(data[13] <= 2 and data[14] in (0, 1, 2, 3, 4, 5, 255), "invalid receiver protocol/state")
        require(data[17] <= 1 and data[15] <= data[16], "invalid config progress/bool")
        if data[17]:
            require(flags & 1 and flags & 16 and data[16] > 0 and data[15] == data[16], "config confirmed without complete version handshake")
        names = ("rx_bytes", "valid_nmea", "valid_ubx", "checksum_errors", "framing_resets", "rx_drop_bytes", "uart_errors", "dma_errors")
        result = {name: _u32(data, 24+4*i) for i, name in enumerate(names)}
        result.update({"diagnostic_flags": flags, "receiver_protocol": data[13], "receiver_state": data[14],
                       "config_step": data[15], "config_total": data[16], "config_confirmed": bool(data[17]),
                       "protocol_version_x100": _u16(data, 18), "gnss_baud_bps": _u32(data, 20),
                       "last_byte_age_ms": _u32(data, 56), "last_frame_age_ms": _u32(data, 60)})
        return result
    require(data[12] in (0, 1) and data[13] <= 2 and data[14] <= 3 and data[15] == 0, "invalid system time enum/reserved")
    names = ("time_sync_age_s", "tx_commands", "ack_count", "nak_count", "config_timeout_count", "tx_packets", "tx_failures", "recoveries", "transport_rx_bytes")
    result = {name: _u32(data, 16+4*i) for i, name in enumerate(names)}
    result.update({"time_available": bool(data[12]), "system_time_source": data[13], "system_time_sync_state": data[14],
                   "dma_ndtr_bytes": _u16(data, 52), "queued_bytes": _u16(data, 54),
                   "system_utc_date_yyyymmdd": _u32(data, 56), "system_utc_second_of_day": _u32(data, 60)})
    if result["time_available"]:
        require(data[13] in (1, 2), "available system time has NONE source")
        _utc(result["system_utc_date_yyyymmdd"], result["system_utc_second_of_day"])
    return result


def _raw_fragment(data):
    """RAW独立24B头，不按typed12B头或typed来源枚举解释。"""
    require(len(data) >= 25, "RAW_FRAME shorter than header+data")
    require(data[0] == 1 and data[3] == 24, "wrong RAW version/header length")
    protocol, flags = data[1], data[2]
    require(protocol in RAW_PROTOCOLS, "unknown RAW protocol namespace")
    require(flags & 2 and not flags & ~3, "RAW CRC16 flag missing/reserved flags")
    total, offset = _u16(data, 12), _u16(data, 14)
    index, count, amount = data[16:19]
    require(0 < total <= (128 if protocol == 1 else RAW_MAX_FRAME), "RAW frame length outside protocol capacity")
    require(protocol != 2 or total >= 8, "UBX wire shorter than8")
    require(count == (total+RAW_MAX_CHUNK-1)//RAW_MAX_CHUNK and index < count, "wrong RAW fragment index/count")
    require(offset == RAW_MAX_CHUNK*index and offset < total, "RAW overlapping/gapped fragment offset")
    require(amount == min(RAW_MAX_CHUNK, total-offset) and len(data) == 24+amount, "wrong RAW data length")
    nmea_type, ubx_class, ubx_id = data[19:22]
    if protocol == 1:
        require(nmea_type <= 4 and ubx_class == ubx_id == 0, "NMEA RAW has UBX metadata/invalid type")
    else:
        require(nmea_type == 0, "UBX RAW must not use NMEA type")
    return {"event": "gnss_raw_fragment", "header": {"version": 1, "protocol_id": protocol,
        "protocol": RAW_PROTOCOLS[protocol], "flags": flags, "frame_id": _u32(data, 4),
        "received_uptime_ms": _u32(data, 8), "frame_length": total, "fragment_offset": offset,
        "fragment_index": index, "fragment_count": count, "data_length": amount,
        "nmea_type": nmea_type, "ubx_class": ubx_class, "ubx_id": ubx_id,
        "wire_crc16": _u16(data, 22), "source_checksum_valid_reported": bool(flags & 1)},
        "fragment_hex": data[24:].hex(), "authoritative_navigation": False}


def _raw_stats(data):
    require(len(data) == 64 and data[:4] == b"\x01\x21\x00\x01", "wrong RAW_STATS version/kind/part/length")
    depth, capacity, protocol, flags = data[44:48]
    require(capacity == 4 and depth <= capacity and protocol in (0, 1, 2), "wrong RAW queue depth/capacity/protocol")
    require(not flags & ~3 and data[63] == 0, "reserved RAW_STATS flags/byte")
    active = bool(flags & 2)
    require((active and protocol in (1, 2) and flags & 1) or (not active and protocol == 0), "RAW active protocol/flags mismatch")
    require(_u16(data, 60) == RAW_MAX_FRAME and data[62] == RAW_MAX_CHUNK, "wrong RAW maximum constants")
    names = ("input_events", "enqueued_frames", "uart_completed_frames", "queue_full_drop_frames",
             "source_reject_events", "aborted_frames", "uart_completed_parts", "tx_error_attempts")
    ages = {name: (_u32(data, offset) if _u32(data, offset) != 0xFFFFFFFF else None) for name, offset in
            (("last_input_age_ms", 48), ("last_completed_age_ms", 52), ("oldest_pending_age_ms", 56))}
    return {"event": "gnss_raw_stats", "version": 1, "stats_sequence": _u32(data, 4),
            "current_uptime_ms": _u32(data, 8), "counters": {name: _u32(data, 12+4*i) for i, name in enumerate(names)},
            "queue": {"depth": depth, "capacity": capacity, "active_protocol_id": protocol,
                      "active_protocol": RAW_PROTOCOLS.get(protocol), "enabled": bool(flags & 1), "active": active},
            "ages": ages, "max_frame_bytes": RAW_MAX_FRAME, "max_chunk_bytes": RAW_MAX_CHUNK,
            "authoritative_navigation": False}


def decode_tunnel_payload(payload: bytes, sysid=1, compid=193):
    """MAVLink已验CRC的TUNNEL有效载荷；先补零到标准133B再按声明内层长度读取。"""
    require(len(payload) <= 133, "TUNNEL payload exceeds 133 bytes")
    padded = bytes(payload).ljust(133, b"\0")
    local_type = _u16(padded, 0)
    length = padded[4]
    require(length <= 128, "TUNNEL inner length exceeds 128")
    inner = padded[5:5+length]
    envelope = {"payload_type": local_type, "payload_type_hex": f"0x{local_type:04X}",
                "target_system": padded[2], "target_component": padded[3], "payload_length": length}
    # 必须先按外层type分流；RAW_FRAME的byte1是protocol而不是typed kind。
    if local_type in (RAW_FRAME_TYPE, RAW_STATS_TYPE):
        require(1 <= sysid <= 250 and compid == 193, "RAW local message has wrong source identity")
        require(not any(padded[5+length:]), "nonzero unused RAW TUNNEL tail")
        event = _raw_fragment(inner) if local_type == RAW_FRAME_TYPE else _raw_stats(inner)
        return {**event, **envelope}
    if local_type not in LOCAL_TYPES:
        return {"event": "unknown_tunnel", **envelope, "unknown": True, "inner_hex": inner.hex(),
                "raw_tunnel_payload_hex": bytes(payload).hex(), "legacy_local_type": local_type in (0x8001, 0x8002, 0x8003)}
    require(1 <= sysid <= 250 and compid == 193, "GNSS local message has wrong source identity")
    kind, name, expected_length, parts = LOCAL_TYPES[local_type]
    require(length == expected_length, "wrong fixed inner length")
    require(not any(padded[5+length:]), "nonzero unused TUNNEL payload tail")
    require(inner[0] == VERSION, "unsupported GNSS wire version")
    require(inner[1] == kind and inner[3] == parts and inner[2] < parts, "wrong kind/part index/count")
    part = inner[2]
    header = {"version": inner[0], "kind": kind, "part_index": part, "part_count": parts,
              "bundle_id": _u32(inner, 4), "snapshot_uptime_ms": _u32(inner, 8)}
    if kind == 1:
        data = _nav(inner)
    elif kind == 2:
        data = _dop(inner, part)
    elif kind == 3:
        data = _counts(inner)
    else:
        data = _link(inner, part)
    return {"event": "gnss_packet", **envelope, "name": name, "header": header, "data": data, "inner_hex": inner.hex()}


def decode_mavlink_frame(frame: bytes):
    """严格解一个完整MAVLink2帧；签名只保留，不声称验证认证。"""
    require(len(frame) >= 12 and frame[0] == 0xFD, "not a complete MAVLink2 frame")
    size, incompat = frame[1], frame[2]
    require(not incompat & ~1, "unsupported MAVLink incompatibility flags")
    signed = bool(incompat & 1)
    require(len(frame) == size + 12 + (13 if signed else 0), "wrong MAVLink frame/signature length")
    msgid = frame[7] | frame[8] << 8 | frame[9] << 16
    body, received_crc = frame[10:10+size], _u16(frame, 10+size)
    descriptor = LEGACY.get(msgid)
    crc_valid = None
    if descriptor:
        require(x25_crc(frame[1:10+size], descriptor[1]) == received_crc, "MAVLink CRC mismatch")
        crc_valid = True
    meta = {"version": 2, "sequence": frame[4], "sysid": frame[5], "compid": frame[6], "msgid": msgid,
            "crc_valid": crc_valid, "signed": signed, "signature_verified": False,
            "signature_hex": frame[-13:].hex() if signed else None}
    if msgid == 385:
        event = decode_tunnel_payload(body, frame[5], frame[6])
    else:
        event = {"event": "legacy_message" if descriptor else "unknown_message", "name": descriptor[0] if descriptor else None,
                 "payload_hex": body.hex(), "authoritative_gnss": False}
        if msgid == 252:
            named = body.ljust(18, b"\0")
            event["named_value"] = {"time_boot_ms_wire": _u32(named, 0), "value": _i32(named, 4),
                                    "name": named[8:18].rstrip(b"\0").decode("ascii", "replace")}
    event["mavlink"] = meta
    return event


class Mavlink2StreamDecoder:
    """有界分帧，支持噪声/任意分片/签名长度，坏CRC后重新搜索。"""
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes, final=False):
        self.buffer.extend(data)
        events = []
        while self.buffer:
            try:
                start = self.buffer.index(0xFD)
            except ValueError:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 10:
                if final:
                    events.append({"event": "error", "error": "truncated MAVLink header", "raw_hex": self.buffer.hex()})
                    self.buffer.clear()
                break
            length = self.buffer[1] + 12 + (13 if self.buffer[2] & 1 else 0)
            if len(self.buffer) < length:
                if not final:
                    break
                events.append({"event": "error", "error": "truncated MAVLink frame", "raw_hex": self.buffer.hex()})
                del self.buffer[0]
                continue
            frame = bytes(self.buffer[:length])
            try:
                event = decode_mavlink_frame(frame)
            except ProtocolError as exc:
                events.append({"event": "error", "error": str(exc), "raw_hex": frame.hex()})
                del self.buffer[0]
            else:
                events.append(event)
                del self.buffer[:length]
        return events


def _complete_link(parts):
    first, second = parts[0]["data"], parts[1]["data"]
    flags = first["diagnostic_flags"]
    driver, transport = bool(flags & 1), bool(flags & 2)
    result = {"driver_fresh": driver, "transport_available": transport,
              "initialized": bool(flags & 32) if transport else None, "tx_busy": bool(flags & 64) if transport else None,
              "rx_seen": bool(flags & 4), "frame_seen": bool(flags & 8), "version_seen": bool(flags & 16),
              "receiver_protocol": ("UNKNOWN", "NMEA", "UBX")[first["receiver_protocol"]] if driver else None,
              "receiver_state": first["receiver_state"] if driver and first["receiver_state"] != 255 else None,
              "config": {k: first[k] if driver else None for k in ("config_step", "config_total", "config_confirmed")},
              "protocol_version_x100": first["protocol_version_x100"] if driver and flags & 16 and first["protocol_version_x100"] else None,
              "baud_bps": first["gnss_baud_bps"] if (driver or transport) and first["gnss_baud_bps"] else None}
    result["driver_counters"] = {k: (first if k in first else second)[k] if driver else None for k in
        ("rx_bytes", "valid_nmea", "valid_ubx", "checksum_errors", "framing_resets", "tx_commands", "ack_count", "nak_count", "config_timeout_count")}
    result["transport_counters"] = {k: (first if k in first else second)[k] if transport else None for k in
        ("rx_drop_bytes", "uart_errors", "dma_errors", "tx_packets", "tx_failures", "recoveries", "transport_rx_bytes")}
    for name, seen in (("last_byte_age_ms", flags & 4), ("last_frame_age_ms", flags & 8)):
        value = first[name]
        result[name] = value if seen and value != 0xFFFFFFFF else None
    require(not transport or (second["dma_ndtr_bytes"] <= 1024 and second["queued_bytes"] <= 1024), "transport count outside RX buffer")
    result["dma_ndtr_bytes"] = second["dma_ndtr_bytes"] if transport else None
    result["queued_bytes"] = second["queued_bytes"] if transport else None
    available = second["time_available"]
    sync_known = available and (second["system_time_source"] == 2 or second["time_sync_age_s"] > 0)
    result["system_time"] = {"available": available,
        "source": ("NONE", "RTC", "GNSS")[second["system_time_source"]] if available else None,
        "sync_state": ("INVALID", "RTC_VALID", "GNSS_SYNCED", "STALE")[second["system_time_sync_state"]] if available else None,
        "reported_sync_age_s": second["time_sync_age_s"] if available else None,
        "sync_age_known": bool(sync_known),
        "sync_age_s": second["time_sync_age_s"] if sync_known else None,
        "value": _utc(second["system_utc_date_yyyymmdd"], second["system_utc_second_of_day"]) if available else None}
    # 保留观测供审计，但不拿RTC或旧软件副本填NAV.time。
    result["reported"] = {"part0": first, "part1": second}
    return result


def age_bundle(bundle, elapsed_ms):
    """按主机已知经过时间失效工程量；不同主机/MCU时钟绝不直接相减。"""
    result = copy.deepcopy(bundle)
    elapsed_ms = max(0, int(elapsed_ms))
    result["host_age_ms"] = result.get("host_age_ms", 0) + elapsed_ms
    result["fresh"] = result["host_age_ms"] <= FRESH_MAX_MS
    def walk(node):
        if isinstance(node, dict):
            if "encoded_age_ms" in node:
                if node["age_ms"] is not None:
                    node["age_ms"] += elapsed_ms
                if node["present"] and (node["age_ms"] is None or node["age_ms"] > FRESH_MAX_MS):
                    node.update(valid=False, stale=True, state="STALE", value=None)
            for value in node.values():
                walk(value)
        elif isinstance(node, list):
            for value in node:
                walk(value)
    walk(result.get("groups", {}))
    link = result.get("groups", {}).get("link")
    if isinstance(link, dict):
        for name in ("last_byte_age_ms", "last_frame_age_ms"):
            if link.get(name) is not None:
                link[name] += elapsed_ms
        link["snapshot_fresh"] = result["fresh"]
        if not result["fresh"]:
            link.update(driver_fresh=False, transport_available=False, initialized=None, tx_busy=None,
                        receiver_protocol=None, receiver_state=None, protocol_version_x100=None,
                        baud_bps=None, dma_ndtr_bytes=None, queued_bytes=None)
            link["config"] = {key: None for key in link["config"]}
            link["driver_counters"] = {key: None for key in link["driver_counters"]}
            link["transport_counters"] = {key: None for key in link["transport_counters"]}
            link["system_time"] = {"available": False, "source": None, "sync_state": None,
                                   "reported_sync_age_s": None, "sync_age_known": False,
                                   "sync_age_s": None, "value": None}
    return result


def _age_data(data, elapsed_ms):
    """为单片增加本地等待时长，绝不从snapshot_uptime反推源时刻。"""
    return age_bundle({"host_age_ms": 0, "groups": data}, elapsed_ms)["groups"]


def _same_observation(old, new):
    """重试可改年龄/有效位/无效占位；已授权的冻结事实不得矛盾。"""
    if old["header"] != new["header"] or old["payload_type"] != new["payload_type"]:
        return False
    kind = old["header"]["kind"]
    left, right = old["data"], new["data"]
    if kind in (1, 3):
        names = DOMAIN_NAMES if kind == 1 else COUNT_NAMES
        for name in names:
            a, b = left[name], right[name]
            if (a["present"], a["source_id"]) != (b["present"], b["source_id"]):
                return False
            if a["valid"] and b["valid"]:
                av, bv = copy.deepcopy(a["value"]), copy.deepcopy(b["value"])
                if name == "navigation_status":
                    # 独立维度也可能随自身年龄失效成0/未知，已有非零值不能互相矛盾。
                    ad, bd = av.pop("fix_dimension"), bv.pop("fix_dimension")
                    if ad is not None and bd is not None and ad != bd:
                        return False
                if av != bv:
                    return False
        return True
    if kind == 2:
        stable = ("id", "source_id", "present", "reported_x100", "itow_known", "itow_ms")
        return all(all(a[k] == b[k] for k in stable) for a, b in zip(left["entries"], right["entries"]))
    if old["header"]["part_index"] == 0:
        if (left["diagnostic_flags"] & 0x7E) != (right["diagnostic_flags"] & 0x7E):
            return False
        for key in ("rx_bytes", "valid_nmea", "valid_ubx", "checksum_errors", "framing_resets", "rx_drop_bytes", "uart_errors", "dma_errors"):
            if left[key] != right[key]:
                return False
        if left["diagnostic_flags"] & right["diagnostic_flags"] & 2:
            if left["gnss_baud_bps"] != right["gnss_baud_bps"]:
                return False
        if left["diagnostic_flags"] & right["diagnostic_flags"] & 1:
            if left["gnss_baud_bps"] != right["gnss_baud_bps"]:
                return False
            return all(left[k] == right[k] for k in ("receiver_protocol", "receiver_state", "config_step", "config_total", "config_confirmed", "protocol_version_x100"))
        return True
    stable = ("tx_commands", "ack_count", "nak_count", "config_timeout_count", "tx_packets", "tx_failures", "recoveries", "transport_rx_bytes", "dma_ndtr_bytes", "queued_bytes")
    if not all(left[k] == right[k] for k in stable):
        return False
    if left["time_available"] and right["time_available"]:
        return all(left[k] == right[k] for k in ("system_time_source", "system_time_sync_state", "system_utc_date_yyyymmdd", "system_utc_second_of_day"))
    return True


def _merge_retry(previous, incoming, waited_ms):
    """乱序重试不得把年龄倒退；保持最新编码元数据，并保守累计已知本地时间。"""
    result = copy.deepcopy(incoming)
    kind = result["header"]["kind"]
    if kind in (1, 2, 3):
        old_data = _age_data(previous["data"], waited_ms)
        def merge(old, new):
            if isinstance(new, dict):
                if "encoded_age_ms" in new:
                    if new["age_ms"] is not None and old["age_ms"] is not None:
                        new["age_ms"] = max(new["age_ms"], old["age_ms"])
                    if new["present"] and (new["age_ms"] is None or new["age_ms"] > FRESH_MAX_MS):
                        new.update(valid=False, stale=True, state="STALE", value=None)
                for key in new:
                    if key in old and isinstance(new[key], (dict, list)):
                        merge(old[key], new[key])
            elif isinstance(new, list):
                for a, b in zip(old, new):
                    merge(a, b)
        merge(old_data, result["data"])
    elif result["header"]["part_index"] == 0:
        if not previous["data"]["diagnostic_flags"] & 1:
            result["data"]["diagnostic_flags"] &= ~1
        for name in ("last_byte_age_ms", "last_frame_age_ms"):
            old = previous["data"][name]
            new = result["data"][name]
            if old != 0xFFFFFFFF and new != 0xFFFFFFFF:
                result["data"][name] = min(0xFFFFFFFE, max(old + waited_ms, new))
    elif not previous["data"]["time_available"]:
        result["data"]["time_available"] = False
    return result


def _business_signature(bundle):
    """只比较业务授权变化，年龄数字每次增长不制造无限重复通知。"""
    signature = [bundle.get("fresh")]
    def walk(node):
        if isinstance(node, dict):
            if "encoded_age_ms" in node:
                signature.append((node["present"], node["valid"], node["stale"], node["state"]))
            for value in node.values():
                walk(value)
        elif isinstance(node, list):
            for value in node:
                walk(value)
    walk(bundle.get("groups", {}))
    link = bundle.get("groups", {}).get("link", {})
    signature.extend((link.get("driver_fresh"), link.get("transport_available"), link.get("system_time", {}).get("available")))
    return tuple(signature)


class BundleAssembler:
    """只发布齐全6包；每源最多一组。回退清缓存但不伪造唯一boot ID。"""
    def __init__(self, timeout_ms=1500, max_sources=32):
        require(timeout_ms > 0 and max_sources > 0, "invalid assembler limits")
        self.timeout_ms, self.max_sources = timeout_ms, max_sources
        self.sources = {}
        self.completed = {}

    def expire(self, now_ms):
        events = []
        for source, state in list(self.sources.items()):
            if state.get("parts") and not state.get("published") and now_ms-state["first_ms"] > self.timeout_ms:
                state["parts"] = {}
                state["expired"] = True
                events.append({"event": "bundle_expired", "sysid": source[0], "compid": source[1], "key": list(state["key"])})
        for source, completed in self.completed.items():
            before = completed["bundle"]
            after = age_bundle(before, now_ms-completed["aged_at_ms"])
            completed["bundle"], completed["aged_at_ms"] = after, now_ms
            if _business_signature(before) != _business_signature(after):
                event = copy.deepcopy(after)
                event.update(event="gnss_invalidated", reason="local age exceeded validity without newer complete data")
                events.append(event)
        return events

    def feed(self, packet, now_ms):
        events = self.expire(now_ms)
        if packet.get("event") != "gnss_packet":
            return events
        meta, header = packet["mavlink"], packet["header"]
        source = (meta["sysid"], meta["compid"])
        key = (header["bundle_id"], header["snapshot_uptime_ms"])
        state = self.sources.get(source)
        if state is None or state["key"] != key:
            generation = state["generation"] if state else 0
            if state:
                old_id, old_ms = state["key"]
                rollback = ((key[1]-old_ms) & 0xFFFFFFFF) >= 0x80000000 or ((key[0]-old_id) & 0xFFFFFFFF) >= 0x80000000
                if rollback:
                    generation += 1
                    self.completed.pop(source, None)
                    events.append({"event": "epoch_reset", "sysid": source[0], "compid": source[1],
                                   "reason": "uptime/bundle rollback: reboot or delayed old packets; identity is ambiguous"})
                elif state["parts"] and not state.get("published"):
                    events.append({"event": "bundle_replaced", "sysid": source[0], "compid": source[1], "old_key": list(state["key"])})
            if state is None and len(self.sources) >= self.max_sources:
                oldest = min(self.sources, key=lambda src: self.sources[src]["last_ms"])
                del self.sources[oldest]
                self.completed.pop(oldest, None)
                events.append({"event": "source_evicted", "sysid": oldest[0], "compid": oldest[1]})
            state = {"key": key, "generation": generation, "first_ms": now_ms, "last_ms": now_ms,
                     "parts": {}, "arrival": {}, "history": {}, "published": False, "expired": False}
            self.sources[source] = state
        state["last_ms"] = now_ms
        if state.get("expired"):
            return events
        part_key = (packet["payload_type"], header["part_index"])
        old = state["parts"].get(part_key)
        if old:
            if old["inner_hex"] == packet["inner_hex"]:
                return events  # 真正重复不能重置本地接收年龄。
            if not all(_same_observation(history, packet) for history in state["history"][part_key]):
                state["parts"] = {}
                state["expired"] = True
                events.append({"event": "error", "error": "conflicting duplicate part; bundle quarantined"})
                return events
            if len(state["history"][part_key]) >= 3:
                events.append({"event": "error", "error": "too many distinct retry versions for one part"})
                return events
            state["history"][part_key].append(copy.deepcopy(packet))
            packet = _merge_retry(old, packet, now_ms-state["arrival"][part_key])
        else:
            state["history"][part_key] = [copy.deepcopy(packet)]
        state["parts"][part_key] = packet
        state["arrival"][part_key] = now_ms
        if len(state["parts"]) != 6:
            return events
        parts = state["parts"]
        required = {(0x8011, 0), (0x8012, 0), (0x8012, 1), (0x8013, 0), (0x8014, 0), (0x8014, 1)}
        require(set(parts) == required, "unexpected bundle parts")
        def aged(key):
            return _age_data(parts[key]["data"], now_ms-state["arrival"][key])
        nav = aged((0x8011, 0))
        dop = aged((0x8012, 0))["entries"] + aged((0x8012, 1))["entries"]
        if not nav["position"]["valid"]:
            for entry in dop + [nav["hdop"]]:
                if entry["valid"]:
                    entry.update(valid=False, value=None, state="INVALID", unusable_reason="navigation_position_unavailable")
        link = _complete_link({0: parts[(0x8014, 0)], 1: parts[(0x8014, 1)]})
        bundle = {"event": "gnss_bundle", "sysid": source[0], "compid": source[1], "bundle_id": key[0],
                  "snapshot_uptime_ms": key[1], "connection_generation": state["generation"],
                  "unique_boot_id_guaranteed": False, "host_age_ms": now_ms-state["first_ms"],
                  "fresh": now_ms-state["first_ms"] <= FRESH_MAX_MS,
                  "part_wait_ms": {f"0x{key[0]:04X}/{key[1]}": now_ms-stamp for key, stamp in state["arrival"].items()},
                  "groups": {"nav": nav, "dop": {entry["name"]: entry for entry in dop},
                             "counts": aged((0x8013, 0)), "link": link}}
        for name in ("last_byte_age_ms", "last_frame_age_ms"):
            if bundle["groups"]["link"][name] is not None:
                bundle["groups"]["link"][name] += now_ms-state["arrival"][(0x8014, 0)]
        bundle = age_bundle(bundle, 0)
        was_published = state["published"]
        state["published"] = True
        previous = self.completed.get(source)
        self.completed[source] = {"bundle": bundle, "aged_at_ms": now_ms}
        if not was_published:
            events.append(bundle)
        elif previous and _business_signature(previous["bundle"]) != _business_signature(bundle):
            event = copy.deepcopy(bundle)
            event["event"] = "gnss_metadata_update"
            events.append(event)
        return events

    def current(self, now_ms):
        """消费者再次读取时显式计算年龄，未齐分片从不替换完整组。"""
        self.expire(now_ms)
        return [copy.deepcopy(state["bundle"]) for state in self.completed.values()]


def _gps_source_checksum(wire, header):
    """独立核对原GPS校验，不修字节，不解析定位业务。"""
    if header["protocol_id"] == 1:
        require(wire.startswith(b"$") and wire.endswith(b"\n"), "RAW NMEA wire boundary mismatch")
        content = wire[:-1]
        if content.endswith(b"\r"):
            content = content[:-1]
        star = content.find(b"*")
        if star < 1 or len(content)-star != 3 or any(byte not in b"0123456789ABCDEFabcdef" for byte in content[star+1:]):
            return {"algorithm": "NMEA_XOR", "recomputed_valid": None, "detail": "missing/malformed original checksum"}
        checksum = 0
        for byte in content[1:star]:
            checksum ^= byte
        return {"algorithm": "NMEA_XOR", "recomputed_valid": checksum == int(content[star+1:], 16)}
    require(len(wire) >= 8 and wire[:2] == b"\xB5\x62" and _u16(wire, 4)+8 == len(wire), "RAW UBX wire boundary/length mismatch")
    require(wire[2] == header["ubx_class"] and wire[3] == header["ubx_id"], "RAW UBX metadata does not match wire class/id")
    a = b = 0
    for byte in wire[2:-2]:
        a = (a+byte) & 0xFF
        b = (b+a) & 0xFF
    return {"algorithm": "UBX_FLETCHER8", "recomputed_valid": wire[-2:] == bytes((a, b))}


class RawFrameAssembler:
    """独立RAW有界组装；5s首片窗口，完整CRC16后才发布，绝不碰typed缓存。"""
    _META = ("version", "protocol_id", "flags", "frame_id", "received_uptime_ms", "frame_length",
             "fragment_count", "nmea_type", "ubx_class", "ubx_id", "wire_crc16")

    def __init__(self, timeout_ms=5000, max_pending=16, max_recent=64):
        require(timeout_ms > 0 and max_pending > 0 and max_recent > 0, "invalid RAW assembler limits")
        self.timeout_ms, self.max_pending, self.max_recent = timeout_ms, max_pending, max_recent
        self.pending, self.recent = OrderedDict(), OrderedDict()

    def _remember(self, identity, state, status, wire=None):
        self.recent[identity] = {"metadata": state["metadata"], "status": status, "wire": wire}
        self.recent.move_to_end(identity)
        while len(self.recent) > self.max_recent:
            self.recent.popitem(last=False)

    @staticmethod
    def _identity_event(identity):
        return dict(zip(("sysid", "compid", "frame_id", "received_uptime_ms"), identity))

    def expire(self, now_ms):
        events = []
        for identity, state in list(self.pending.items()):
            if now_ms-state["first_ms"] > self.timeout_ms:
                self._remember(identity, state, "timeout")
                del self.pending[identity]
                events.append({"event": "gnss_raw_timeout", **self._identity_event(identity),
                               "received_fragments": len(state["chunks"]), "expected_fragments": state["header"]["fragment_count"]})
        return events

    def feed(self, packet, now_ms):
        events = self.expire(now_ms)
        if packet.get("event") != "gnss_raw_fragment":
            return events
        meta, header = packet.get("mavlink", {}), packet["header"]
        require(meta.get("crc_valid") is True, "RAW assembly requires verified MAVLink CRC")
        identity = (meta["sysid"], meta["compid"], header["frame_id"], header["received_uptime_ms"])
        metadata = tuple(header[key] for key in self._META)
        chunk = bytes.fromhex(packet["fragment_hex"])
        index, offset = header["fragment_index"], header["fragment_offset"]
        previous = self.recent.get(identity)
        if previous:
            if previous["status"] != "complete":
                return events
            if previous["metadata"] != metadata or previous["wire"][offset:offset+len(chunk)] != chunk:
                previous.update(status="quarantined", wire=None)
                events.append({"event": "error", "error": "conflicting completed RAW identity/fragment", **self._identity_event(identity)})
            return events
        state = self.pending.get(identity)
        if state is None:
            if len(self.pending) >= self.max_pending:
                evicted, old = self.pending.popitem(last=False)
                self._remember(evicted, old, "evicted")
                events.append({"event": "gnss_raw_evicted", **self._identity_event(evicted), "reason": "bounded Pi reassembly capacity"})
            state = {"header": copy.deepcopy(header), "metadata": metadata, "chunks": {}, "first_ms": now_ms}
            self.pending[identity] = state
        if state["metadata"] != metadata or (index in state["chunks"] and state["chunks"][index] != chunk):
            self._remember(identity, state, "quarantined")
            del self.pending[identity]
            events.append({"event": "error", "error": "conflicting RAW metadata/overlap", **self._identity_event(identity)})
            return events
        if index in state["chunks"]:
            return events  # 相同分片重试不重置首片超时，也不重复发布。
        state["chunks"][index] = chunk
        if len(state["chunks"]) != header["fragment_count"]:
            return events
        wire = b"".join(state["chunks"][i] for i in range(header["fragment_count"]))
        del self.pending[identity]
        self._remember(identity, state, "quarantined")
        require(len(wire) == header["frame_length"], "RAW reassembly has a hole/length mismatch")
        require(crc16_ccitt_false(wire) == header["wire_crc16"], "RAW frame CRC16-CCITT-FALSE mismatch")
        check = _gps_source_checksum(wire, header)
        reported = header["source_checksum_valid_reported"]
        recomputed = check["recomputed_valid"]
        check.update(reported_valid=reported, consistent=(reported == recomputed) if recomputed is not None else None)
        self._remember(identity, state, "complete", wire)
        names = {(1, 7): "NAV-PVT", (1, 4): "NAV-DOP", (1, 0x35): "NAV-SAT",
                 (0x0A, 4): "MON-VER", (5, 1): "ACK-ACK", (5, 0): "ACK-NAK"}
        name = ("OTHER", "GGA", "RMC", "GSA", "GSV")[header["nmea_type"]] if header["protocol_id"] == 1 else names.get((header["ubx_class"], header["ubx_id"]), "OTHER")
        event = {"event": "gnss_raw_frame", **self._identity_event(identity), "protocol_id": header["protocol_id"],
                 "protocol": header["protocol"], "message_name": name, "length": len(wire),
                 "nmea_type": header["nmea_type"], "ubx_class": header["ubx_class"], "ubx_id": header["ubx_id"],
                 "wire_crc16": header["wire_crc16"], "wire_crc16_verified": True, "mavlink_crc_verified": True,
                 "signature_verified": False, "bytes_hex": wire.hex(), "source_checksum": check,
                 "source_checksum_confirmed": bool(reported and recomputed is True),
                 "usable_as_navigation_update": False, "navigation_validity": "not_evaluated",
                 "authoritative_navigation": False, "host_assembly_ms": max(0, now_ms-state["first_ms"]),
                 "unique_boot_id_guaranteed": False}
        if header["protocol_id"] == 1:
            event["ascii"] = wire.decode("ascii", "backslashreplace")
        events.append(event)
        return events


class Decoder:
    """帧流和齐包组装入口；旧帧/未知类型作为独立事件保留。"""
    def __init__(self, emit_packets=False, timeout_ms=1500, raw_timeout_ms=5000):
        self.stream = Mavlink2StreamDecoder()
        self.assembler = BundleAssembler(timeout_ms)
        self.raw_assembler = RawFrameAssembler(raw_timeout_ms)
        self.emit_packets = emit_packets

    def feed(self, data, now_ms=None, final=False):
        if now_ms is None:
            now_ms = time.monotonic_ns() // 1000000
        output = []
        for event in self.stream.feed(data, final):
            if event.get("event") == "gnss_packet":
                if self.emit_packets:
                    output.append(event)
                try:
                    output.extend(self.assembler.feed(event, now_ms))
                except ProtocolError as exc:
                    output.append({"event": "error", "error": str(exc)})
            elif event.get("event") == "gnss_raw_fragment":
                if self.emit_packets:
                    output.append(event)
                try:
                    output.extend(self.raw_assembler.feed(event, now_ms))
                except ProtocolError as exc:
                    output.append({"event": "error", "error": str(exc), "channel": "raw"})
            else:
                output.append(event)
        output.extend(self.assembler.expire(now_ms))
        output.extend(self.raw_assembler.expire(now_ms))
        return output


def _main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--input", type=Path, help="离线MAVLink二进制文件；未指定来源则读stdin")
    source.add_argument("--hex", help="离线纯HEX字符串，或@HEX文本文件")
    source.add_argument("--port", help="可选pyserial只读接收端口；本工具不发送命令")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--packets", action="store_true", help="额外输出分片事件；完整GNSS状态仍只取gnss_bundle")
    parser.add_argument("--bundle-timeout-ms", type=int, default=1500)
    parser.add_argument("--raw-timeout-ms", type=int, default=5000, help="独立RAW首片组装窗口，不是GPS源年龄")
    args = parser.parse_args(argv)
    if args.bundle_timeout_ms <= 0 or args.raw_timeout_ms <= 0 or args.baud <= 0:
        parser.error("timeout和baud必须为正数")
    decoder = Decoder(args.packets, args.bundle_timeout_ms, args.raw_timeout_ms)
    errors = False
    def emit(events):
        nonlocal errors
        for event in events:
            errors |= event.get("event") == "error"
            print(json.dumps(event, ensure_ascii=False, allow_nan=False), flush=True)
    if args.port:
        try:
            import serial  # type: ignore  # 可选依赖，只在显式串口模式导入。
        except ImportError:
            parser.error("串口模式需要可选pyserial；离线模式仅用标准库")
        try:
            with serial.Serial(args.port, args.baud, timeout=0.2, rtscts=False, dsrdtr=False) as port:
                while True:
                    emit(decoder.feed(port.read(4096)))
        except KeyboardInterrupt:
            return 0
    else:
        if args.hex is not None:
            value = Path(args.hex[1:]).read_text(encoding="utf-8-sig") if args.hex.startswith("@") else args.hex
            try:
                raw = bytes.fromhex(value)
            except ValueError:
                parser.error("--hex需要纯十六进制字节及空白分隔")
        else:
            raw = args.input.read_bytes() if args.input else sys.stdin.buffer.read()
        emit(decoder.feed(raw, now_ms=0, final=True))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(_main())
