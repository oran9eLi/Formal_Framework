"""BMI088 Pi解码器离线回归，全部为独立主机向量，不访问串口或真实硬件。"""
import copy
import importlib.util
import struct
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("bmi088_uart6_decoder", ROOT / "Tools/bmi088_uart6_decoder.py")
D = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D)

# 真实C编码器与MAVLink库产生，来源test_bmi088_uplink.c；是主机向量，不是实测。
C_GOLDEN_RAW = "FD440000012AC181010030800000400100030144332211E8030000F20300000200000018681E0F8004040100800000FF7F332211DC80000102030405060708090A0B0C0D0E0F10110080004000E0206F"
C_GOLDEN_PARSED = "FD450000022AC181010031800000400101030144332211E8030000FC0300000200000020D1FFFF00000000E02E0000C0BDF0FF20A10700702FFCFF1EFB1027E02EE8030102030108070605040302018E5A"


def crc_reference(data, extra):
    """逐bit参考CRC，与解码器nibble形式独立。"""
    crc = 0xFFFF
    for byte in bytes(data) + bytes((extra,)):
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc & 0xFFFF


def frame(inner, payload_type=D.RAW_TYPE, seq=3, signed=False, trim=True, sysid=42):
    body = struct.pack("<HBBB", payload_type, 0, 0, 64) + bytes(inner) + bytes(64)
    return message(body, 385, 147, seq, signed, trim, sysid)


def message(body, msgid, extra, seq=3, signed=False, trim=True, sysid=42):
    body = bytes(body)
    if trim:
        body = body.rstrip(b"\0") or b"\0"
    header = bytes((len(body), int(signed), 0, seq, sysid, 193)) + msgid.to_bytes(3, "little")
    wire = b"\xFD" + header + body + struct.pack("<H", crc_reference(header + body, extra))
    return wire + (bytes(range(13)) if signed else b"")


def base(kind, seq=7, sample=1000, now=1020, flags=3, bus=1, reinit=2):
    out = bytearray(64)
    struct.pack_into("<BBBBIIII", out, 0, 1, kind, flags, bus, seq, sample, now, reinit)
    return out


def raw(**kwargs):
    out = base(0, **kwargs)
    out[20:28] = bytes((0x18, 0x68, 0x1E, 0x0F, 0x80, 2, 2, 1))
    if out[3] == 2:
        out[20:22] = bytes(2)
    struct.pack_into("<hhh", out, 28, 100, -200, 2731)
    out[34:37] = bytes((0xFE, 0xFF, 0xFF))
    # 温度raw=-200，低5bit故意非零；工程温度-2℃。
    out[37:39] = bytes((0xE7, 0x1B))
    struct.pack_into("<hhhhhh", out, 39, 9, 8, 7, -32768, 12345, -23456)
    return out


def parsed(**kwargs):
    out = base(1, **kwargs)
    axes = (2, -1, 3)
    chip_acc = [x * 12000 / 32768 for x in (100, -200, 2731)]
    chip_gyro = [x * 1000000 / 32768 for x in (-32768, 12345, -23456)]
    def nearest(value):
        return int(value + 0.5) if value >= 0 else int(value - 0.5)
    def mapped(values):
        return [nearest(values[abs(axis) - 1] * (1 if axis > 0 else -1)) for axis in axes]
    struct.pack_into("<iii", out, 20, *mapped(chip_acc))
    struct.pack_into("<iii", out, 32, *mapped(chip_gyro))
    struct.pack_into("<hHHHbbbBII", out, 44, -200, 10000, 12000, 1000, *axes, 1, 3, 4)
    return out


def status(present=False, available=False, diagnostics=False, **kwargs):
    out = base(2, seq=7 if present else 0, sample=1000 if present else 0,
               flags=3 if present and available else 1 if present else 0,
               bus=1 if diagnostics or present else 0, **kwargs)
    struct.pack_into("<IIIIIII", out, 20, 4, 3, 120, 100, 6, 1, 2)
    out[48:56] = bytes((3, 2, 0, 0, 0x42, 0x99, 0x18, 0x69))
    struct.pack_into("<I", out, 56, ((int.from_bytes(out[12:16], "little") - 1000) & 0xFFFFFFFF) if present else 0xFFFFFFFF)
    out[60:64] = bytes((int(diagnostics), int(available), 5 if diagnostics else 255, 0))
    return out


def packet(data, payload_type):
    return D.decode_mavlink_frame(frame(data, payload_type))


class PayloadTests(unittest.TestCase):
    def test_raw_bytes_fifo_temperature_and_tail(self):
        result = D.decode_payload(D.RAW_TYPE, raw())
        self.assertEqual(result["data"]["gyro_frame_count"], 2)
        self.assertEqual(result["data"]["audit_counts"]["gyro_fifo_xyz"], [[9, 8, 7], [-32768, 12345, -23456]])
        self.assertEqual(result["data"]["temperature_hex"], "e71b")
        self.assertEqual(result["data"]["audit_chip_values"]["temperature_cdeg"], -200)
        self.assertEqual(result["data"]["audit_counts"]["accel_sensor_time"], 0xFFFFFE)
        self.assertIsNone(result["data"]["business_value"])
        data = raw(); data[61] = 1
        with self.assertRaises(D.ProtocolError):
            D.decode_payload(D.RAW_TYPE, data)
        data = raw(); data[37] = 0x80
        with self.assertRaises(D.ProtocolError):
            D.decode_payload(D.RAW_TYPE, data)

    def test_spi_and_independent_address_selection(self):
        self.assertEqual(D.decode_payload(D.RAW_TYPE, raw(bus=2))["header"]["bus"], 2)
        data = raw(); data[20:22] = bytes((0x19, 0x69))
        self.assertEqual(D.decode_payload(D.RAW_TYPE, data)["data"]["accel_addr_7bit"], 0x19)
        for offset, value in ((20, 0x76), (22, 0x42), (24, 0), (25, 0x82), (26, 5), (27, 2), (63, 1)):
            bad = raw(); bad[offset] = value
            with self.subTest(offset=offset), self.assertRaises(D.ProtocolError):
                D.decode_payload(D.RAW_TYPE, bad)

    def test_parsed_units_and_mapping(self):
        body = D.decode_payload(D.PARSED_TYPE, parsed())["data"]
        self.assertEqual(body["axis_map"], [2, -1, 3])
        self.assertEqual(body["value"]["temperature_c"], -2)
        self.assertEqual(body["value"]["gyro_mdps"][1], 1000000)
        self.assertFalse(body["gyro_bias_applied"])
        self.assertEqual(body["sample_period_us"], 10000)
        for mapping in ((1, 1, 3), (-1, 2, 3), (1, 2, 4)):
            data = parsed(); struct.pack_into("<bbb", data, 52, *mapping)
            with self.subTest(mapping=mapping), self.assertRaises(D.ProtocolError):
                D.decode_payload(D.PARSED_TYPE, data)
        for offset, value in ((48, 8000), (50, 2000), (46, 0)):
            data = parsed(); struct.pack_into("<H", data, offset, value)
            with self.subTest(offset=offset), self.assertRaises(D.ProtocolError):
                D.decode_payload(D.PARSED_TYPE, data)

    def test_header_lengths_version_kind_and_authorization(self):
        for data in (raw()[:-1], raw() + b"\0"):
            with self.assertRaises(D.ProtocolError):
                D.decode_payload(D.RAW_TYPE, data)
        for offset, value in ((0, 2), (1, 1), (2, 2), (2, 7), (2, 11), (2, 0x13), (3, 0)):
            data = raw(); data[offset] = value
            with self.subTest(offset=offset, value=value), self.assertRaises(D.ProtocolError):
                D.decode_payload(D.RAW_TYPE, data)
        old = D.decode_payload(D.PARSED_TYPE, parsed(now=1201, flags=5))
        self.assertIsNone(old["data"]["value"])
        self.assertTrue(old["header"]["stale"])
        self.assertEqual(D.decode_payload(D.RAW_TYPE, raw(sample=0xFFFFFFF0, now=20))["header"]["encoded_age_ms"], 36)

    def test_status_unknown_fields_and_bad_id_are_not_fake_measurements(self):
        missing = D.decode_payload(D.STATUS_TYPE, status())["data"]
        self.assertIsNone(missing["diagnostics"])
        self.assertIsNone(missing["counters"]["driver_error_count"])
        self.assertEqual(missing["counters"]["completed_pairs"], 6)
        bad_id = D.decode_payload(D.STATUS_TYPE, status(diagnostics=True))["data"]
        self.assertEqual(bad_id["diagnostics"]["accel_chip_id"], 0x42)
        ready = status(diagnostics=True); ready[51] = 1
        with self.assertRaises(D.ProtocolError):
            D.decode_payload(D.STATUS_TYPE, ready)
        unknown_age = status(present=True, diagnostics=True, now=900)
        unknown_age[2] = 5
        struct.pack_into("<I", unknown_age, 56, 0xFFFFFFFF)
        decoded = D.decode_payload(D.STATUS_TYPE, unknown_age)
        self.assertIsNone(decoded["header"]["encoded_age_ms"])
        self.assertIsNone(decoded["data"]["source_age_ms"])
        self.assertFalse(decoded["header"]["usable"])


class MavlinkTests(unittest.TestCase):
    def test_real_c_wire_golden_pair(self):
        raw_packet = D.decode_mavlink_frame(bytes.fromhex(C_GOLDEN_RAW))
        parsed_packet = D.decode_mavlink_frame(bytes.fromhex(C_GOLDEN_PARSED))
        self.assertEqual(raw_packet["header"]["sample_sequence"], 0x11223344)
        self.assertEqual(raw_packet["data"]["gyro_frame_count"], 4)
        self.assertEqual(raw_packet["data"]["audit_counts"]["gyro_fifo_xyz"][-1], [-32768, 16384, -8192])
        self.assertEqual(parsed_packet["data"]["reported"]["accel_mg"], [-12000, 0, 12000])
        self.assertEqual(parsed_packet["data"]["reported"]["gyro_mdps"], [-1000000, 500000, -250000])
        self.assertEqual(parsed_packet["data"]["reported"]["temperature_cdeg"], -1250)
        self.assertEqual(parsed_packet["data"]["driver_error_count"], 0x01020304)
        self.assertEqual(parsed_packet["data"]["fifo_drop_count"], 0x05060708)
        pair = D.PairAssembler()
        self.assertEqual(pair.feed(raw_packet, 0), [])
        result = pair.feed(parsed_packet, 10)[-1]
        self.assertTrue(result["usable"])
        self.assertEqual(result["identity"], [42, 193, 2, 0x11223344, 1000])
        self.assertEqual(result["value"]["temperature_c"], -12.5)

    def test_crc_and_trailing_zero_reconstruction(self):
        wire = frame(raw())
        result = D.decode_mavlink_frame(wire)
        self.assertLess(wire[1], 133)
        self.assertEqual(bytes.fromhex(result["payload_hex"]), bytes(raw()))
        self.assertTrue(result["mavlink"]["crc_valid"])
        damaged = bytearray(wire); damaged[39] ^= 1
        with self.assertRaises(D.ProtocolError):
            D.decode_mavlink_frame(damaged)

    def test_noise_every_split_and_bad_frame_resync(self):
        wire = frame(raw())
        for split in range(len(wire) + 1):
            stream = D.Mavlink2StreamDecoder()
            result = stream.feed(b"noise" + wire[:split]) + stream.feed(wire[split:], final=True)
            self.assertEqual([x["event"] for x in result], ["bmi088_raw"])
        bad = bytearray(wire); bad[-1] ^= 0x80
        out = D.Mavlink2StreamDecoder().feed(bad + wire, final=True)
        self.assertEqual(out[-1]["event"], "bmi088_raw")
        self.assertTrue(any(x["event"] == "error" for x in out))
        self.assertTrue(D.Mavlink2StreamDecoder().feed(wire[:-2], final=True))
        malformed = raw(); malformed[27] = 2; malformed[28:30] = b"\xFD\xFF"
        out = D.Mavlink2StreamDecoder().feed(frame(malformed) + wire)
        self.assertEqual(out[-1]["event"], "bmi088_raw")

    def test_signed_frame_identified_without_false_signature_claim(self):
        result = D.decode_mavlink_frame(frame(raw(), signed=True))
        self.assertTrue(result["mavlink"]["signed"])
        self.assertFalse(result["mavlink"]["signature_verified"])
        bad = bytearray(frame(raw())); bad[2] = 2
        with self.assertRaises(D.ProtocolError):
            D.decode_mavlink_frame(bad)

    def test_unknown_other_types_ignored_and_outer_fields_checked(self):
        self.assertEqual(D.decode_mavlink_frame(message(bytes(9), 0, 50))["event"], "ignored_message")
        self.assertEqual(D.decode_mavlink_frame(frame(raw(), 0x8020))["event"], "ignored_message")
        self.assertEqual(D.decode_mavlink_frame(message(bytes(133), 385, 147))["event"], "ignored_message")
        self.assertEqual(D.Decoder().feed(frame(raw(), 0x8020), final=True), [])
        body = struct.pack("<HBBB", D.RAW_TYPE, 0, 0, 63) + raw() + bytes(64)
        with self.assertRaises(D.ProtocolError):
            D.decode_mavlink_frame(message(body, 385, 147))
        body = struct.pack("<HBBB", D.RAW_TYPE, 1, 0, 64) + raw() + bytes(64)
        with self.assertRaises(D.ProtocolError):
            D.decode_mavlink_frame(message(body, 385, 147))

    def test_pymavlink_adapter_requires_caller_crc_contract(self):
        class Fake:
            payload_type, target_system, target_component, payload_length = D.RAW_TYPE, 0, 0, 64
            payload = list(raw()) + [0] * 64
            def get_type(self): return "TUNNEL"
            def get_srcSystem(self): return 42
            def get_srcComponent(self): return 193
        decoded = D.decode_tunnel(Fake())
        with self.assertRaises(D.ProtocolError):
            D.PairAssembler().feed(decoded, 0)
        self.assertFalse(D.decode_tunnel(Fake(), True)["mavlink"]["signature_verified"])


class PairTests(unittest.TestCase):
    def test_raw_parsed_pair_and_latest_expiration(self):
        assembler = D.PairAssembler()
        self.assertEqual(assembler.feed(packet(raw(), D.RAW_TYPE), 0), [])
        events = assembler.feed(packet(parsed(now=1070), D.PARSED_TYPE), 50)
        pair = events[-1]
        self.assertTrue(pair["usable"])
        self.assertEqual(pair["source_effective_age_ms"], 70)
        self.assertEqual(pair["value"]["temperature_c"], -2)
        self.assertIsNotNone(assembler.current(42, 193, 180)["value"])
        self.assertIsNone(assembler.current(42, 193, 181)["value"])

    def test_reverse_arrival_and_duplicate_do_not_repeat(self):
        assembler = D.PairAssembler()
        parsed_packet = packet(parsed(now=1070), D.PARSED_TYPE)
        assembler.feed(parsed_packet, 0)
        events = assembler.feed(packet(raw(), D.RAW_TYPE), 10)
        self.assertEqual(events[-1]["event"], "bmi088_pair")
        self.assertEqual(assembler.feed(parsed_packet, 100), [])
        self.assertIsNone(assembler.current(42, 193, 201)["value"])
        invalidated = packet(raw(flags=9), D.RAW_TYPE)
        assembler.feed(invalidated, 110)
        self.assertIsNone(assembler.current(42, 193, 111)["value"])

    def test_conflicts_and_value_mismatch_quarantined(self):
        assembler = D.PairAssembler()
        assembler.feed(packet(raw(), D.RAW_TYPE), 0)
        bad = raw(); bad[28] += 1
        with self.assertRaises(D.ProtocolError):
            assembler.feed(packet(bad, D.RAW_TYPE), 1)
        self.assertEqual(assembler.feed(packet(parsed(), D.PARSED_TYPE), 2), [])
        assembler = D.PairAssembler(); assembler.feed(packet(raw(), D.RAW_TYPE), 0)
        bad = parsed(); struct.pack_into("<i", bad, 20, -500)
        with self.assertRaises(D.ProtocolError):
            assembler.feed(packet(bad, D.PARSED_TYPE), 1)
        self.assertIsNone(assembler.current(42, 193, 2))

    def test_timeout_capacity_and_invalidation(self):
        assembler = D.PairAssembler(timeout_ms=1000, max_pending=2)
        assembler.feed(packet(raw(seq=1), D.RAW_TYPE), 0)
        assembler.feed(packet(raw(seq=2), D.RAW_TYPE), 1)
        self.assertEqual(assembler.feed(packet(raw(seq=3), D.RAW_TYPE), 2)[0]["event"], "bmi088_pair_evicted")
        self.assertEqual(len(assembler.expire(1003)), 2)
        assembler = D.PairAssembler()
        assembler.feed(packet(raw(), D.RAW_TYPE), 0)
        assembler.feed(packet(parsed(), D.PARSED_TYPE), 10)
        assembler.feed(packet(status(present=True, available=False, diagnostics=True), D.STATUS_TYPE), 20)
        self.assertIsNone(assembler.current(42, 193, 21)["value"])

    def test_restart_wrap_and_explicit_session_reset(self):
        assembler = D.PairAssembler()
        assembler.feed(packet(raw(sample=0xFFFFFFD0, now=0xFFFFFFF0), D.RAW_TYPE), 0)
        events = assembler.feed(packet(raw(seq=8, sample=0, now=20), D.RAW_TYPE), 50)
        self.assertEqual(events[0]["event"], "bmi088_source_reset")
        self.assertEqual(len(assembler.pending), 1)
        assembler.feed(packet(parsed(seq=8, sample=0, now=30), D.PARSED_TYPE), 60)
        self.assertIsNotNone(assembler.current(42, 193, 60))
        assembler.reset_source(42, 193)
        self.assertIsNone(assembler.current(42, 193, 60))
        self.assertEqual(len(assembler.pending), 0)

    def test_stale_audit_pair_and_decoder_pipeline(self):
        decoder = D.Decoder(False)
        events = decoder.feed(frame(raw(now=1201, flags=5)) + frame(parsed(now=1201, flags=5), D.PARSED_TYPE), now_ms=0, final=True)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["event"], "bmi088_pair")
        self.assertFalse(events[0]["usable"])
        self.assertIsNone(events[0]["value"])


if __name__ == "__main__":
    unittest.main()
