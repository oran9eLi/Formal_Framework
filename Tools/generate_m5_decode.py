"""由仓库MAVLink头中的线上offset生成M5只读字段目录；不运行固件或测试。"""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
NAMES = ("HEARTBEAT", "SYS_STATUS", "GPS_RAW_INT", "ATTITUDE", "GLOBAL_POSITION_INT",
         "SCALED_PRESSURE", "BATTERY_STATUS", "SERVO_OUTPUT_RAW", "NAMED_VALUE_INT",
         "TUNNEL", "COMMAND_LONG", "COMMAND_ACK", "OPEN_DRONE_ID_BASIC_ID")
KINDS = {"UINT8_T": "DQ_U8", "INT8_T": "DQ_I8", "UINT16_T": "DQ_U16", "INT16_T": "DQ_I16",
         "UINT32_T": "DQ_U32", "INT32_T": "DQ_I32", "UINT64_T": "DQ_U64", "FLOAT": "DQ_FLOAT", "CHAR": "DQ_CHAR"}
UNITS = dict(time_boot_ms="ms", time_usec="us", lat="degE7", lon="degE7", alt="mm", relative_alt="mm",
             eph="cm", epv="cm", vel="cm/s", cog="cdeg", hdg="cdeg", vx="cm/s", vy="cm/s", vz="cm/s",
             roll="rad", pitch="rad", yaw="rad", rollspeed="rad/s", pitchspeed="rad/s", yawspeed="rad/s",
             voltage_battery="mV", current_battery="cA", battery_remaining="%", temperature="cdegC",
             voltages="mV", voltages_ext="mV", current_consumed="mAh", energy_consumed="hJ",
             press_abs="hPa", press_diff="hPa", load="0.1%", drop_rate_comm="0.01%", time_remaining="s",
             temperature_press_diff="cdegC", h_acc="mm", v_acc="mm", vel_acc="mm/s", hdg_acc="degE5", yaw_accuracy="cdeg")
# 同名字段不保证同一单位/哨兵；以消息和字段为键，依据本仓库协议注释。
UNIT_OVERRIDES = {("GPS_RAW_INT", "eph"): "DOPx100", ("GPS_RAW_INT", "epv"): "DOPx100",
                  ("GPS_RAW_INT", "yaw"): "cdeg", ("GPS_RAW_INT", "alt_ellipsoid"): "mm",
                  ("COMMAND_ACK", "progress"): "%"}
UNKNOWN = {
    **{("GPS_RAW_INT", f): 1 for f in ("eph", "epv", "vel", "cog")},
    ("GPS_RAW_INT", "satellites_visible"): 4, ("GPS_RAW_INT", "yaw"): 6,
    ("GLOBAL_POSITION_INT", "hdg"): 1,
    ("SYS_STATUS", "voltage_battery"): 1, ("SYS_STATUS", "current_battery"): 2,
    ("SYS_STATUS", "battery_remaining"): 2,
    **{("BATTERY_STATUS", f): 2 for f in ("current_battery", "battery_remaining", "current_consumed", "energy_consumed")},
    ("BATTERY_STATUS", "temperature"): 3, ("BATTERY_STATUS", "voltages"): 1,
    ("BATTERY_STATUS", "voltages_ext"): 7, ("BATTERY_STATUS", "time_remaining"): 5,
    ("SCALED_PRESSURE", "temperature_press_diff"): 5, ("COMMAND_ACK", "progress"): 4,
}


def main():
    chunks = []
    catalog = []
    for name in NAMES:
        path = ROOT/"Third_Party/mavlink/common"/("mavlink_msg_"+name.lower()+".h")
        if not path.is_file():
            candidates = list((ROOT/"Third_Party/mavlink").rglob(path.name))
            if len(candidates) != 1: raise ValueError("Missing/ambiguous inherited message: "+name)
            path = candidates[0]
        text = path.read_text(encoding="utf-8")
        msgid = int(re.search(r"#define MAVLINK_MSG_ID_"+name+r" (\d+)", text).group(1))
        fields = dict()
        for field, kind, count, offset in re.findall(r'\{\s*"([^"]+)",\s*(?:NULL|"[^"]*"),\s*MAVLINK_TYPE_(\w+),\s*(\d+),\s*(\d+),\s*offsetof', text):
            fields[field] = (kind, int(count), int(offset))
        lines = []
        for field, (kind, count, offset) in fields.items():
            dtype = "DQ_BYTES" if kind == "UINT8_T" and count else KINDS[kind]
            unit = "us" if field.startswith("servo") and field.endswith("_raw") else UNITS.get(field, "")
            unit = UNIT_OVERRIDES.get((name, field), unit)
            if field.startswith("onboard_control_sensors_") or field.endswith("bitmask"):
                unit = "bits"
            unknown = UNKNOWN.get((name, field), 0)
            lines.append(f'  {{"{field}", "{unit}", {offset}U, {dtype}, {count}U, {unknown}U}},')
        chunks.append(f'/** @brief {name}字段，offset为线上payload字节偏移。 */\nstatic const DecodeField_t s_fields_{name.lower()}[] = {{\n'+"\n".join(lines)+"\n};\n")
        catalog.append(f'  {{{msgid}U, "{name}", s_fields_{name.lower()}, {len(fields)}U}},')
    template = (ROOT/"Tools/m5_decode_template.txt").read_text(encoding="utf-8")
    output = template.replace("/* GENERATED_FIELDS */", "\n".join(chunks)).replace("/* GENERATED_CATALOG */", "\n".join(catalog))
    (ROOT/"Framework/Src/px4lite_mavlink_decode.c").write_text(output, encoding="utf-8")


if __name__ == "__main__": main()
