"""从本工程 MAVLink 头文件提取协议表，生成不依赖 Python 的 Windows 桌面程序。"""
from pathlib import Path
import os
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent))
from lora_peer import Dialect


def build():
    output = ROOT / "build" / "MavlinkMonitor"
    output.mkdir(parents=True, exist_ok=True)
    dialect = Dialect(ROOT)
    messages = {}
    for directory in ("minimal", "standard", "common"):
        for header in (dialect.root.parent / directory).glob("mavlink_msg_*.h"):
            name = header.stem.removeprefix("mavlink_msg_").upper()
            number, minimum, maximum, fields = dialect.message(name)
            if number not in dialect.crcs:
                continue
            if not fields:
                raise ValueError("缺少字段定义 " + name)
            layout = ";".join(f"{key},{fmt},{count},{offset}"
                              for key, (fmt, count, offset) in fields.items())
            messages[number] = f"{number}|{name}|{dialect.crcs[number]}|{minimum}|{maximum}|{layout}"
    if set(messages) != set(dialect.crcs):
        raise ValueError("未完整解析 common 方言消息表")
    table = output / "dialect.txt"
    table.write_text("\n".join(messages[i] for i in sorted(messages)), encoding="ascii")
    framework = Path(os.environ["SystemRoot"]) / "Microsoft.NET" / "Framework" / "v4.0.30319"
    binary_dir = output / "dist"
    binary_dir.mkdir(exist_ok=True)
    exe = binary_dir / "MAVLink中文串口助手.exe"
    subprocess.run([str(framework / "csc.exe"), "/nologo", "/warnaserror+", "/optimize+",
                    "/target:winexe", "/platform:anycpu", "/codepage:65001",
                    "/reference:System.Windows.Forms.dll", "/reference:System.Drawing.dll",
                    f"/resource:{table},dialect", f"/resource:{HERE / 'eastway.png'},eastway.logo",
                    f"/win32icon:{HERE / 'eastway.ico'}", f"/out:{exe}",
                    *map(str, sorted(HERE.glob("*.cs")))], check=True)
    print(f"{len(messages)} messages -> {exe}")


if __name__ == "__main__":
    build()
