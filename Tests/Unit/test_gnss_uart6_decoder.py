"""真实Pi解码器回归；独立bitwise CRC构帧及C生成wire黄金向量，无串口访问。"""
import importlib.util
import binascii
import json
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("gnss_uart6_decoder", ROOT / "Tools/gnss_uart6_decoder.py")
DEC = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DEC)

# 来源：真实C test_gnss_unified_uplink.c的golden_frames.json；纯host向量，不是实测。
GOLDEN_WIRE_HEX = (
    'FD450000002AC181010011800000400101000101000000E80300003F00591404030C04EB32A4F8B168DE3A2EFBFFFFC801000039306900F2283501F0B0000000000000000000000000000000003F047B43',
    'FD440000012AC181010012800000400102000201000000E80300000400000000060B0065000A0040E2010001060B0066000A0040E2010002060B0067000A0040E2010003060B0068000A0040E201E322',
    'FD380000022AC181010012800000400102010201000000E80300000300000004040300690014000000000005060B006A00140040E2010006060B006B00140040E20196D3',
    'FD240000032AC181010013800000200103000101000000E80300000F0F0000000E0008050504041E001E001E001EA64C',
    'FD420000042AC181010014800000400104000201000000E80300007F0203030301520D00C2010039300000070000005900000002000000030000000400000005000000060000002800000028FC27',
    'FD430000052AC181010014800000400104010201000000E8030000010101002A0000000A0000000300000001000000020000000B0000000C0000000D0000003D300000B6020000F2283501F0B0275E',
)

# 独立RAW C生产者+真实MAVLink编码器黄金；BAD GPS校验必须仍原样输出。
RAW_C_GOLDEN = (
    'FD450000002AC181010020800000400102021801000000E803000064000000000328000107D10BB56201075C0066778899AABBCCDDEEFF102132435465768798A9BACBDCEDFE0F203142536475869732C9',
    'FD450000012AC181010020800000400102021801000000E803000064002800010328000107D10BA8B9CADBECFD0E1F30415263748596A7B8C9DAEBFC0D1E2F405162738495A6B7C8D9EAFB0C1D2E3FE053',
    'FD310000022AC1810100208000002C0102021801000000E803000064005000020314000107D10B5061728394A5B6C7D8E9FA0B1C2D3E4F6071060BA426',
    'FD440000032AC181010021800000400121000101000000D0070000010000000100000001000000000000000000000000000000030000000000000000040001E8030000D4030000FFFFFFFF0004286553',
)
RAW_C_ORIGINAL = 'B56201075C0066778899AABBCCDDEEFF102132435465768798A9BACBDCEDFE0F2031425364758697A8B9CADBECFD0E1F30415263748596A7B8C9DAEBFC0D1E2F405162738495A6B7C8D9EAFB0C1D2E3F5061728394A5B6C7D8E9FA0B1C2D3E4F6071060B'


def crc_reference(data, extra):
    """逐bit参考算法，避免与被测nibble实现共享同一代码错误。"""
    crc = 0xFFFF
    for byte in bytes(data) + bytes((extra,)):
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc & 0xFFFF


def mav_frame(payload, msgid=385, extra=147, sysid=1, compid=193, signed=False, trim=True):
    payload = bytes(payload)
    if trim:
        payload = payload.rstrip(b"\0") or b"\0"
    header = bytes((len(payload), int(signed), 0, 0, sysid, compid)) + msgid.to_bytes(3, "little")
    packet = b"\xfd" + header + payload + struct.pack("<H", crc_reference(header + payload, extra))
    return packet + bytes(range(13)) if signed else packet


def inner(kind, part=0, bundle=0, snapshot=0):
    out = bytearray(32 if kind == 3 else 64)
    struct.pack_into("<BBBBII", out, 0, 1, kind, part, 2 if kind in (2, 4) else 1, bundle, snapshot)
    return out


def nav(fix=False, utc=False, age=0, bundle=0, snapshot=0):
    out = inner(1, bundle=bundle, snapshot=snapshot)
    for offset in range(48, 62, 2):
        struct.pack_into("<H", out, offset, 0xFFFF)
    out[12] = out[62] = 16 | (1 if fix else 0) | (4 if utc else 0)
    struct.pack_into("<H", out, 14, (1 << 12) | (1 if fix else 0) | (3 << 6 if utc else 0))
    out[16], out[17], out[18], out[63] = (1, 3, 10, 3) if fix else (0, 0, 0, 0)
    struct.pack_into("<H", out, 56, age)
    if fix:
        struct.pack_into("<H", out, 48, age)
    if utc:
        struct.pack_into("<II", out, 40, 20261106, 0)
        struct.pack_into("<H", out, 52, age)
    return out


def dop(part=0, usable=False, nmea=False, bundle=0, snapshot=0):
    out = inner(2, part, bundle, snapshot)
    count = 4 if part == 0 else 3
    out[12] = count
    for row in range(count):
        ident = 4*part + row
        is_nmea = nmea and ident in (1, 3, 4)
        struct.pack_into("<BBBBHHI", out, 16+12*row, ident, 4 if is_nmea else 6,
                         1 | (2 if usable else 0) | (0 if is_nmea else 8), 0, 101+ident, 0, 0 if is_nmea else 123456)
    return out


def counts(known_zero=False, bundle=0, snapshot=0):
    out = inner(3, bundle=bundle, snapshot=snapshot)
    for offset in range(24, 32, 2):
        struct.pack_into("<H", out, offset, 0xFFFF)
    if known_zero:
        out[12] = out[13] = 1
        out[20] = 5
        struct.pack_into("<H", out, 24, 0)
    return out


def link(part=0, bundle=0, snapshot=0):
    out = inner(4, part, bundle, snapshot)
    if part == 0:
        out[12:18] = bytes((63, 2, 3, 3, 3, 1))
        struct.pack_into("<HI", out, 18, 3410, 115200)
    else:
        out[12:15] = bytes((1, 1, 1))  # 有效RTC，不是GNSS UTC。
        struct.pack_into("<HHII", out, 52, 1024, 0, 20261106, 0)
    return out


def packet(data, sysid=1):
    kind = data[1]
    outer = struct.pack("<HBBB", 0x8010+kind, 0, 0, len(data)) + data + bytes(128-len(data))
    return DEC.decode_mavlink_frame(mav_frame(outer, sysid=sysid))


def six(bundle=0, snapshot=0, fix=False, utc=False):
    return [packet(nav(fix, utc, bundle=bundle, snapshot=snapshot)),
            packet(dop(0, fix, bundle=bundle, snapshot=snapshot)),
            packet(dop(1, fix, bundle=bundle, snapshot=snapshot)),
            packet(counts(bundle=bundle, snapshot=snapshot)),
            packet(link(0, bundle, snapshot)), packet(link(1, bundle, snapshot))]


def ubx_wire(payload=b"", cls=1, ident=7):
    data = bytes((cls, ident)) + struct.pack("<H", len(payload)) + bytes(payload)
    a = b = 0
    for byte in data:
        a = (a + byte) & 255
        b = (b + a) & 255
    return b"\xb5\x62" + data + bytes((a, b))


def nmea_wire(body=b"GPGSV,1,1,00"):
    checksum = 0
    for byte in body:
        checksum ^= byte
    return b"$" + body + b"*" + f"{checksum:02X}".encode("ascii") + b"\r\n"


def raw_parts(wire, protocol=2, frame_id=0, received=0, reported=True, crc=None, nmea_type=0):
    """独立标准库CRC16与显式偏移构造RAW片，不复用被测编码器。"""
    wire = bytes(wire)
    checksum = binascii.crc_hqx(wire, 0xFFFF) if crc is None else crc
    result = []
    for index, offset in enumerate(range(0, len(wire), 40)):
        chunk = wire[offset:offset+40]
        out = bytearray(24 + len(chunk))
        struct.pack_into("<BBBBIIHHBBBBBBH", out, 0, 1, protocol, 2 | int(reported), 24,
                         frame_id, received, len(wire), offset, index, (len(wire)+39)//40,
                         len(chunk), nmea_type, wire[2] if protocol == 2 else 0,
                         wire[3] if protocol == 2 else 0, checksum)
        out[24:] = chunk
        result.append(out)
    return result


def raw_mav(data, local_type=0x8020, **kwargs):
    outer = struct.pack("<HBBB", local_type, 0, 0, len(data)) + data + bytes(128-len(data))
    return mav_frame(outer, **kwargs)


def raw_packet(data, local_type=0x8020, **kwargs):
    return DEC.decode_mavlink_frame(raw_mav(data, local_type, **kwargs))


def raw_stats():
    out = bytearray(64)
    struct.pack_into("<BBBBII", out, 0, 1, 0x21, 0, 1, 0, 0)
    struct.pack_into("<8I", out, 12, 12, 7, 5, 4, 1, 2, 31, 0xFFFFFFFF)
    out[44:48] = bytes((4, 4, 2, 3))
    struct.pack_into("<IIIHB", out, 48, 0, 0xFFFFFFFF, 4000, 1024, 40)
    return out


class DecoderTests(unittest.TestCase):
    def test_zero_uptime_midnight_no_fix_and_valid_zero_coordinates(self):
        data = packet(nav(False, True))["data"]
        self.assertEqual(data["navigation_status"]["state"], "NO_FIX")
        self.assertEqual(data["navigation_status"]["value"]["satellites_used"], 0)
        self.assertEqual(data["time"]["value"]["second_of_day"], 0)
        self.assertEqual(data["time"]["age_ms"], 0)
        self.assertIsNone(data["position"]["value"])
        zero_position = packet(nav(True))["data"]["position"]
        self.assertEqual(zero_position["value"]["latitude_deg"], 0.0)
        self.assertEqual(zero_position["value"]["longitude_deg"], 0.0)

    def test_dop_reported_is_not_usable_and_nmea_has_no_fake_itow(self):
        entry = packet(dop(1, usable=False, nmea=True))["data"]["entries"][0]
        self.assertEqual(entry["reported_x100"], 105)
        self.assertIsNone(entry["value"])
        self.assertIsNone(entry["itow_ms"])
        ubx = dop(0)
        struct.pack_into("<I", ubx, 24, 0)  # UBX known的0毫秒也合法。
        self.assertEqual(packet(ubx)["data"]["entries"][0]["itow_ms"], 0)

    def test_counts_unknown_differs_from_known_zero(self):
        self.assertIsNone(packet(counts())["data"]["gps_visible"]["value"])
        self.assertEqual(packet(counts(True))["data"]["gps_visible"]["value"], 0)

    def test_kind_part_version_len_flags_and_ranges(self):
        for offset, value in ((0, 2), (1, 4), (2, 1), (3, 2), (12, 128)):
            malformed = nav()
            malformed[offset] = value
            with self.subTest(offset=offset), self.assertRaises(DEC.ProtocolError):
                packet(malformed)
        malformed = nav()[:-1]
        with self.assertRaises(DEC.ProtocolError):
            packet(malformed)

        malformed = nav(True)
        struct.pack_into("<i", malformed, 20, 900000001)
        with self.assertRaises(DEC.ProtocolError):
            packet(malformed)
        malformed = nav(utc=True)
        struct.pack_into("<I", malformed, 40, 20260229)
        with self.assertRaises(DEC.ProtocolError):
            packet(malformed)
        malformed = dop()
        malformed[18] |= 128
        with self.assertRaises(DEC.ProtocolError):
            packet(malformed)
        malformed = dop(nmea=True)
        malformed[30] |= 8  # id1是NMEA_GSA，不能有iTOWknown。
        with self.assertRaises(DEC.ProtocolError):
            packet(malformed)

    def test_source_combination_is_domain_specific(self):
        malformed = nav(True)
        struct.pack_into("<H", malformed, 14, (1 << 12) | 3)  # RMC不能授权位置。
        with self.assertRaises(DEC.ProtocolError):
            packet(malformed)
        malformed = dop()
        malformed[17] = 4  # GSA不报告GDOP。
        with self.assertRaises(DEC.ProtocolError):
            packet(malformed)
        malformed = counts(True)
        malformed[20] = 1  # PVT总数不能假拆GPS visible。
        with self.assertRaises(DEC.ProtocolError):
            packet(malformed)

    def test_valid_age_must_be_fresh_and_unknown_age_not_zero(self):
        with self.assertRaises(DEC.ProtocolError):
            packet(nav(age=1501))
        value = nav(age=0xFFFF)
        value[12], value[13] = 0, 16
        domain = packet(value)["data"]["navigation_status"]
        self.assertIsNone(domain["age_ms"])
        self.assertIsNone(domain["value"])
        self.assertEqual(domain["state"], "STALE")

    def test_mavlink_crc_signed_and_zero_tail(self):
        value = nav()
        outer = struct.pack("<HBBB", 0x8011, 0, 0, 64) + value + bytes(64)
        frame = mav_frame(outer, signed=True)
        self.assertLess(frame[1], 133)
        decoded = DEC.decode_mavlink_frame(frame)
        self.assertTrue(decoded["mavlink"]["signed"])
        self.assertFalse(decoded["mavlink"]["signature_verified"])
        with self.assertRaises(DEC.ProtocolError):
            DEC.decode_mavlink_frame(frame[:-1])
        bad = bytearray(frame)
        bad[20] ^= 1
        with self.assertRaises(DEC.ProtocolError):
            DEC.decode_mavlink_frame(bad)
        stream = DEC.Mavlink2StreamDecoder()
        output = []
        for byte in b"noise" + frame:
            output += stream.feed(bytes((byte,)))
        self.assertEqual([v["event"] for v in output], ["gnss_packet"])

    def test_unknown_local_and_old_messages_are_retained_not_commands(self):
        raw = b"\x01\x04\x00\x00"
        outer = struct.pack("<HBBB", 0x8003, 1, 193, len(raw)) + raw
        result = DEC.decode_mavlink_frame(mav_frame(outer))
        self.assertEqual(result["event"], "unknown_tunnel")
        self.assertEqual(result["inner_hex"], raw.hex())
        old = DEC.decode_mavlink_frame(mav_frame(bytes(9), msgid=0, extra=50))
        self.assertEqual(old["event"], "legacy_message")
        self.assertFalse(old["authoritative_gnss"])

    def test_parts_out_of_order_complete_once_and_atomic_replace(self):
        assembly = DEC.BundleAssembler()
        values = six()
        for index in (5, 2, 0, 4, 1):
            self.assertFalse(any(e["event"] == "gnss_bundle" for e in assembly.feed(values[index], 0)))
            self.assertEqual(assembly.current(0), [])
        events = assembly.feed(values[3], 10)
        self.assertEqual([e["event"] for e in events], ["gnss_bundle"])
        self.assertEqual(assembly.feed(values[3], 50), [])
        new = six(bundle=1, snapshot=1000)
        assembly.feed(new[0], 1000)
        self.assertEqual(assembly.current(1000)[0]["bundle_id"], 0)
        for value in new[1:]:
            assembly.feed(value, 1000)
        self.assertEqual(assembly.current(1000)[0]["bundle_id"], 1)

    def test_rtc_does_not_fill_nav_gnss_utc(self):
        assembly = DEC.BundleAssembler()
        events = []
        for value in six():
            events += assembly.feed(value, 0)
        bundle = events[-1]
        self.assertIsNone(bundle["groups"]["nav"]["time"]["value"])
        system = bundle["groups"]["link"]["system_time"]
        self.assertEqual(system["source"], "RTC")
        self.assertEqual(system["value"]["second_of_day"], 0)
        self.assertIsNone(system["sync_age_s"])
        self.assertEqual(system["reported_sync_age_s"], 0)
        self.assertFalse(system["sync_age_known"])

    def test_rtc_sync_zero_ambiguity_and_gnss_zero(self):
        for source, reported, known in ((1, 0, False), (1, 12, True), (2, 0, True)):
            values = six()
            second = link(1)
            second[13:15] = bytes((source, 2))
            struct.pack_into("<I", second, 16, reported)
            values[5] = packet(second)
            assembly = DEC.BundleAssembler()
            for value in values:
                assembly.feed(value, 0)
            system = assembly.current(0)[0]["groups"]["link"]["system_time"]
            self.assertEqual(system["reported_sync_age_s"], reported)
            self.assertEqual(system["sync_age_known"], known)
            self.assertEqual(system["sync_age_s"], reported if known else None)

    def test_incomplete_dop_cannot_mix_bundle_keys(self):
        assembly = DEC.BundleAssembler()
        old, new = six(bundle=1, snapshot=1000), six(bundle=2, snapshot=2000)
        assembly.feed(old[1], 0)
        for index in (0, 2, 3, 4, 5):
            assembly.feed(new[index], 10)
        self.assertEqual(assembly.current(10), [])
        events = assembly.feed(new[1], 20)
        self.assertEqual(events[-1]["bundle_id"], 2)

    def test_local_wait_and_current_read_age_out_values(self):
        assembly = DEC.BundleAssembler()
        values = six()
        values[0] = packet(nav(age=1490))
        assembly.feed(values[0], 0)
        for value in values[1:]:
            assembly.feed(value, 20)
        bundle = assembly.current(20)[0]
        status = bundle["groups"]["nav"]["navigation_status"]
        self.assertEqual(status["age_ms"], 1510)
        self.assertIsNone(status["value"])
        self.assertEqual(bundle["part_wait_ms"]["0x8012/1"], 0)
        self.assertFalse(assembly.current(2000)[0]["fresh"])

    def test_disconnect_empty_feed_emits_bounded_invalidation(self):
        decoder = DEC.Decoder()
        events = []
        for raw in GOLDEN_WIRE_HEX:
            events += decoder.feed(bytes.fromhex(raw), now_ms=0)
        self.assertEqual(events[-1]["event"], "gnss_bundle")
        events = decoder.feed(b"", now_ms=1600)
        self.assertEqual([event["event"] for event in events], ["gnss_invalidated"])
        current = decoder.assembler.completed[(42, 193)]["bundle"]
        self.assertFalse(current["fresh"])
        self.assertIsNone(current["groups"]["nav"]["position"]["value"])
        self.assertFalse(current["groups"]["link"]["system_time"]["available"])
        self.assertIsNone(current["groups"]["link"]["receiver_state"])
        self.assertTrue(all(value is None for value in current["groups"]["link"]["transport_counters"].values()))
        for now in range(1700, 10000, 100):
            self.assertEqual(decoder.feed(b"", now_ms=now), [])

    def test_unavailable_link_never_turns_default_zero_into_live_state(self):
        first, second = link(0), link(1)
        first[12:18] = bytes(6)
        second[12] = 0
        result = DEC._complete_link({0: packet(first), 1: packet(second)})
        self.assertIsNone(result["receiver_state"])
        self.assertIsNone(result["protocol_version_x100"])
        self.assertTrue(all(value is None for value in result["driver_counters"].values()))
        self.assertTrue(all(value is None for value in result["transport_counters"].values()))
        self.assertIsNone(result["system_time"]["value"])
        self.assertIsNone(result["system_time"]["source"])
        self.assertEqual(result["reported"]["part0"]["receiver_state"], 0)
        first[12], first[14] = 1, 255
        self.assertIsNone(DEC._complete_link({0: packet(first), 1: packet(second)})["receiver_state"])
        first[14] = 0
        self.assertEqual(DEC._complete_link({0: packet(first), 1: packet(second)})["receiver_state"], 0)

    def test_retry_metadata_refresh_allowed_but_cannot_deage(self):
        assembly = DEC.BundleAssembler()
        assembly.feed(packet(nav(age=500)), 0)
        self.assertEqual(assembly.feed(packet(nav(age=100)), 200), [])
        for value in six()[1:]:
            assembly.feed(value, 250)
        status = assembly.current(250)[0]["groups"]["nav"]["navigation_status"]
        self.assertEqual(status["encoded_age_ms"], 100)
        self.assertEqual(status["age_ms"], 750)
        conflict = nav(True)
        other = DEC.BundleAssembler()
        other.feed(packet(conflict), 0)
        struct.pack_into("<i", conflict, 20, 1)
        error = other.feed(packet(conflict), 20)
        self.assertIn("conflicting", error[-1]["error"])

    def test_expiry_rollback_wrap_and_device_isolation(self):
        assembly = DEC.BundleAssembler(timeout_ms=100)
        values = six(bundle=8, snapshot=8000)
        for value in values[:-1]:
            assembly.feed(value, 0)
        self.assertEqual(assembly.expire(101)[0]["event"], "bundle_expired")
        self.assertEqual(assembly.feed(values[-1], 102), [])
        events = []
        for value in six(bundle=0, snapshot=0):
            events += assembly.feed(value, 200)
        self.assertIn("epoch_reset", [event["event"] for event in events])
        self.assertFalse(events[-1]["unique_boot_id_guaranteed"])
        wrap = DEC.BundleAssembler()
        for value in six(bundle=0xFFFFFFFF, snapshot=0xFFFFFF00):
            wrap.feed(value, 0)
        events = []
        for value in six(bundle=0, snapshot=100):
            events += wrap.feed(value, 10)
        self.assertNotIn("epoch_reset", [event["event"] for event in events])
        separate = DEC.BundleAssembler()
        for value in six()[:3]:
            separate.feed(value, 0)
        for value in six()[3:]:
            value["mavlink"]["sysid"] = 2
            separate.feed(value, 0)
        self.assertEqual(separate.current(0), [])

    def test_real_c_wire_golden(self):
        self.assertEqual(len(GOLDEN_WIRE_HEX), 6)
        decoder = DEC.Decoder()
        events = []
        for index, raw in enumerate(GOLDEN_WIRE_HEX):
            events += decoder.feed(bytes.fromhex(raw), now_ms=1000 + 10*index)
        self.assertEqual([e["event"] for e in events], ["gnss_bundle"])
        bundle = events[0]
        self.assertEqual((bundle["sysid"], bundle["compid"], bundle["bundle_id"], bundle["snapshot_uptime_ms"]), (42, 193, 1, 1000))
        self.assertEqual(bundle["groups"]["nav"]["position"]["value"]["latitude_e7"], -123456789)
        self.assertEqual(bundle["groups"]["nav"]["position"]["age_ms"], 50)
        self.assertEqual(bundle["groups"]["dop"]["HDOP"]["reported_x100"], 105)
        self.assertIsNone(bundle["groups"]["dop"]["HDOP"]["itow_ms"])
        self.assertEqual([entry["value"] for entry in bundle["groups"]["counts"].values()], [0, 14, 0, 8])

    def test_cli_hex_and_binary_input_without_serial(self):
        raw = b"".join(bytes.fromhex(value) for value in GOLDEN_WIRE_HEX)
        tool = str(ROOT / "Tools/gnss_uart6_decoder.py")
        text = subprocess.check_output([sys.executable, tool, "--hex", raw.hex()], text=True)
        self.assertEqual(json.loads(text)["event"], "gnss_bundle")
        temporary_root = ROOT / "build/gnss_uart6_decoder_tests"
        temporary_root.mkdir(parents=True, exist_ok=True)
        self.assertTrue(temporary_root.resolve().is_relative_to(ROOT.resolve()))
        with tempfile.TemporaryDirectory(dir=temporary_root) as directory:
            self.assertTrue(Path(directory).resolve().is_relative_to(temporary_root.resolve()))
            path = Path(directory) / "frames.bin"
            path.write_bytes(raw)
            text = subprocess.check_output([sys.executable, tool, "--input", str(path)], text=True)
            self.assertEqual(json.loads(text)["bundle_id"], 1)
        bad = bytearray(raw)
        bad[15] ^= 1
        result = subprocess.run([sys.executable, tool, "--hex", bad.hex()], text=True, capture_output=True)
        self.assertEqual(result.returncode, 1)
        self.assertIn('"event": "error"', result.stdout)


class RawDecoderTests(unittest.TestCase):
    def assemble(self, wire, protocol=2, **kwargs):
        decoder = DEC.Decoder()
        events = []
        for part in raw_parts(wire, protocol, **kwargs):
            events += decoder.feed(raw_mav(part), now_ms=0)
        self.assertEqual([e["event"] for e in events], ["gnss_raw_frame"])
        return events[0]

    def test_ccitt_false_reference_and_original_nmea_zero_epoch(self):
        self.assertEqual(DEC.crc16_ccitt_false(b"123456789"), 0x29B1)
        wire = nmea_wire()
        event = self.assemble(wire, 1, nmea_type=4)
        self.assertEqual(event["bytes_hex"], wire.hex())
        self.assertEqual(event["ascii"].encode("ascii"), wire)
        self.assertEqual((event["frame_id"], event["received_uptime_ms"]), (0, 0))
        self.assertEqual(event["protocol"], "NMEA")
        self.assertTrue(event["source_checksum"]["recomputed_valid"])
        self.assertFalse(event["authoritative_navigation"])

    def test_bad_source_checksum_is_delivered_without_repair(self):
        for protocol, wire in ((1, nmea_wire()), (2, ubx_wire(bytes(range(92))))):
            bad = bytearray(wire)
            bad[-4 if protocol == 1 else -1] ^= 1
            event = self.assemble(bad, protocol, reported=False)
            self.assertEqual(bytes.fromhex(event["bytes_hex"]), bad)
            self.assertTrue(event["wire_crc16_verified"])
            self.assertFalse(event["source_checksum"]["recomputed_valid"])
            self.assertTrue(event["source_checksum"]["consistent"])
            self.assertFalse(event["usable_as_navigation_update"])
            self.assertFalse(event["source_checksum_confirmed"])
        mismatch = self.assemble(ubx_wire(), reported=False)
        self.assertFalse(mismatch["source_checksum"]["consistent"])
        self.assertFalse(mismatch["usable_as_navigation_update"])
        self.assertFalse(mismatch["source_checksum_confirmed"])

    def test_navsat_64_satellites_and_maximum_1024_26_parts(self):
        # NAV-SAT: 8B epoch header + 64*12B satellite records + 8B UBX envelope.
        payload = struct.pack("<IBBH", 0, 1, 64, 0) + bytes((i * 17) & 255 for i in range(64*12))
        sat = ubx_wire(payload, ident=0x35)
        self.assertEqual(len(sat), 784)
        self.assertEqual(self.assemble(sat)["bytes_hex"], sat.hex())
        maximum = ubx_wire(bytes((i * 29) & 255 for i in range(1016)), cls=0x0A, ident=4)
        parts = raw_parts(maximum)
        self.assertEqual((len(maximum), len(parts), len(parts[-1])-24), (1024, 26, 24))
        assembly = DEC.RawFrameAssembler()
        events = []
        for index in reversed(range(len(parts))):
            events += assembly.feed(raw_packet(parts[index], signed=index == 0), 0)
        self.assertEqual(events[-1]["bytes_hex"], maximum.hex())
        self.assertTrue(events[-1]["source_checksum"]["recomputed_valid"])
        self.assertTrue(events[-1]["source_checksum_confirmed"])
        self.assertFalse(events[-1]["usable_as_navigation_update"])
        oversized = raw_parts(ubx_wire(bytes(1017)))[0]
        with self.assertRaises(DEC.ProtocolError):
            raw_packet(oversized)
        oversized_nmea = raw_parts(nmea_wire(b"GPGSV," + b"0"*125), 1)[0]
        with self.assertRaises(DEC.ProtocolError):
            raw_packet(oversized_nmea)

    def test_raw_protocol_namespace_and_navigation_aging_are_independent(self):
        self.assertEqual(raw_packet(raw_parts(nmea_wire(), 1)[0])["header"]["protocol"], "NMEA")
        self.assertEqual(packet(nav())["data"]["navigation_status"]["source"], "UBX_PVT")
        old = DEC.decode_mavlink_frame(mav_frame(bytes(30), msgid=24, extra=24))
        self.assertEqual((old["event"], old["name"]), ("legacy_message", "GPS_RAW_INT"))
        decoder = DEC.Decoder()
        for wire in GOLDEN_WIRE_HEX:
            decoder.feed(bytes.fromhex(wire), now_ms=0)
        events = decoder.feed(raw_mav(raw_parts(nmea_wire(), 1)[0]), now_ms=1600)
        self.assertIn("gnss_raw_frame", [e["event"] for e in events])
        self.assertIn("gnss_invalidated", [e["event"] for e in events])
        self.assertIsNone(decoder.assembler.current(1600)[0]["groups"]["nav"]["position"]["value"])

    def test_fragment_bounds_flags_and_namespace_rejected(self):
        first = raw_parts(ubx_wire(bytes(92)))[0]
        for offset, value in ((0, 2), (1, 3), (2, 1), (2, 6), (3, 12), (14, 1), (16, 3), (17, 2), (18, 39), (19, 1)):
            bad = bytearray(first)
            bad[offset] = value
            with self.subTest(offset=offset, value=value), self.assertRaises(DEC.ProtocolError):
                raw_packet(bad)
        for offset in (20, 21):
            bad = raw_parts(nmea_wire(), 1)[0]
            bad[offset] = 1
            with self.assertRaises(DEC.ProtocolError):
                raw_packet(bad)
        with self.assertRaises(DEC.ProtocolError):
            raw_packet(first[:-1])

    def test_duplicate_out_of_order_and_timeout_never_publish_partial(self):
        parts = raw_parts(ubx_wire(bytes(range(92))))
        assembly = DEC.RawFrameAssembler(timeout_ms=100)
        self.assertEqual(assembly.feed(raw_packet(parts[2]), 0), [])
        self.assertEqual(assembly.feed(raw_packet(parts[2]), 50), [])
        self.assertEqual(assembly.feed(raw_packet(parts[0]), 60), [])
        done = assembly.feed(raw_packet(parts[1]), 70)
        self.assertEqual(done[0]["event"], "gnss_raw_frame")
        for part in parts:
            self.assertEqual(assembly.feed(raw_packet(part), 80), [])
        lost = DEC.RawFrameAssembler(timeout_ms=100)
        lost.feed(raw_packet(parts[0]), 0)
        self.assertEqual(lost.expire(101)[0]["event"], "gnss_raw_timeout")
        for part in parts:
            self.assertEqual(lost.feed(raw_packet(part), 102), [])

    def test_conflicting_metadata_and_chunk_quarantine_identity(self):
        parts = raw_parts(ubx_wire(bytes(range(92))))
        for metadata in (True, False):
            assembly = DEC.RawFrameAssembler()
            assembly.feed(raw_packet(parts[0]), 0)
            bad = bytearray(parts[1] if metadata else parts[0])
            bad[22 if metadata else 24] ^= 1
            events = assembly.feed(raw_packet(bad), 1)
            self.assertEqual(events[-1]["event"], "error")
            for part in parts:
                self.assertEqual(assembly.feed(raw_packet(part), 2), [])

    def test_wrong_full_crc16_and_mav_crc_are_separate_errors(self):
        decoder = DEC.Decoder()
        wire = ubx_wire(bytes(92))
        events = []
        for part in raw_parts(wire, crc=binascii.crc_hqx(wire, 0xFFFF) ^ 1):
            events += decoder.feed(raw_mav(part), now_ms=0)
        self.assertEqual(events[-1]["event"], "error")
        self.assertIn("CRC16", events[-1]["error"])
        bad = bytearray(raw_mav(raw_parts(wire)[0]))
        bad[-1] ^= 1
        events = DEC.Decoder().feed(bad, now_ms=0, final=True)
        self.assertTrue(any(e["event"] == "error" for e in events))
        self.assertFalse(any(e["event"] == "gnss_raw_frame" for e in events))

    def test_stats_queue_full_counters_and_unknown_age(self):
        event = raw_packet(raw_stats(), 0x8021)
        self.assertEqual(event["event"], "gnss_raw_stats")
        self.assertEqual(event["counters"]["queue_full_drop_frames"], 4)
        self.assertEqual(event["counters"]["tx_error_attempts"], 0xFFFFFFFF)
        self.assertEqual(event["queue"]["depth"], 4)
        self.assertEqual(event["ages"]["last_input_age_ms"], 0)
        self.assertIsNone(event["ages"]["last_completed_age_ms"])
        self.assertEqual(event["max_frame_bytes"], 1024)
        for offset, value in ((1, 1), (2, 1), (3, 2), (44, 5), (45, 8), (46, 0), (47, 7), (60, 1), (62, 39), (63, 1)):
            bad = raw_stats()
            bad[offset] = value
            with self.subTest(offset=offset), self.assertRaises(DEC.ProtocolError):
                raw_packet(bad, 0x8021)
        idle = raw_stats()
        idle[44:48] = bytes((0, 4, 0, 1))
        self.assertFalse(raw_packet(idle, 0x8021)["queue"]["active"])

    def test_bounded_pending_and_different_sources_do_not_mix(self):
        assembly = DEC.RawFrameAssembler(max_pending=1, max_recent=2)
        wire = ubx_wire(bytes(92))
        first, second = raw_parts(wire, frame_id=1), raw_parts(wire, frame_id=2)
        assembly.feed(raw_packet(first[0]), 0)
        self.assertEqual(assembly.feed(raw_packet(second[0]), 1)[0]["event"], "gnss_raw_evicted")
        self.assertEqual(len(assembly.pending), 1)
        for ident in range(3, 10):
            assembly.feed(raw_packet(raw_parts(wire, frame_id=ident)[0]), ident)
        self.assertLessEqual(len(assembly.recent), 2)
        separate = DEC.RawFrameAssembler()
        separate.feed(raw_packet(first[0]), 0)
        events = []
        for part in first[1:]:
            events += separate.feed(raw_packet(part, sysid=2), 1)
        self.assertFalse(any(e["event"] == "gnss_raw_frame" for e in events))

    def test_real_c_raw_golden_and_stats(self):
        decoder = DEC.Decoder()
        events = []
        # 真正的3片乱序后组装；stats不走typed组装器。
        for ordinal, index in enumerate((2, 0, 1, 3)):
            events += decoder.feed(bytes.fromhex(RAW_C_GOLDEN[index]), now_ms=2000+ordinal)
        self.assertEqual([e["event"] for e in events], ["gnss_raw_frame", "gnss_raw_stats"])
        raw, stats = events
        self.assertEqual((raw["sysid"], raw["compid"], raw["frame_id"], raw["received_uptime_ms"]), (42, 193, 1, 1000))
        self.assertEqual(raw["bytes_hex"].upper(), RAW_C_ORIGINAL)
        self.assertEqual(raw["wire_crc16"], 0x0BD1)
        self.assertFalse(raw["usable_as_navigation_update"])
        self.assertFalse(raw["source_checksum_confirmed"])
        self.assertFalse(raw["source_checksum"]["recomputed_valid"])
        self.assertEqual(stats["counters"], dict(zip(
            ("input_events", "enqueued_frames", "uart_completed_frames", "queue_full_drop_frames",
             "source_reject_events", "aborted_frames", "uart_completed_parts", "tx_error_attempts"),
            (1, 1, 1, 0, 0, 0, 3, 0))))
        self.assertEqual(stats["queue"]["capacity"], 4)
        self.assertEqual(stats["ages"], {"last_input_age_ms": 1000, "last_completed_age_ms": 980, "oldest_pending_age_ms": None})
        self.assertEqual(decoder.assembler.current(2003), [])

    def test_raw_cli_remains_independent_json_events(self):
        wire = b"".join(bytes.fromhex(value) for value in RAW_C_GOLDEN)
        text = subprocess.check_output([sys.executable, str(ROOT / "Tools/gnss_uart6_decoder.py"),
                                        "--hex", wire.hex()], text=True)
        events = [json.loads(line) for line in text.splitlines()]
        self.assertEqual([e["event"] for e in events], ["gnss_raw_frame", "gnss_raw_stats"])
        self.assertEqual(events[0]["bytes_hex"].upper(), RAW_C_ORIGINAL)


if __name__ == "__main__":
    unittest.main()
