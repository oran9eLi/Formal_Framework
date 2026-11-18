"""构建东创大为 E22 配置助手；可选 --test 同时运行寄存器、事务与窗口回归。"""
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ASSETS = HERE.parent / "MavlinkMonitor"
OUT = ROOT / "build" / "E22Configurator"
CSC = Path(os.environ["SystemRoot"]) / "Microsoft.NET/Framework/v4.0.30319/csc.exe"


def build():
    dist = OUT / "dist"
    dist.mkdir(parents=True, exist_ok=True)
    exe = dist / "东创大为LoRa配置助手.exe"
    args = [str(CSC), "/nologo", "/warnaserror+", "/optimize+", "/platform:anycpu",
            "/codepage:65001", "/reference:System.Windows.Forms.dll", "/reference:System.Drawing.dll",
            f"/win32icon:{ASSETS / 'eastway.ico'}", f"/resource:{ASSETS / 'eastway.png'},eastway.logo"]
    sources = [str(HERE / "E22Protocol.cs"), str(HERE / "ConfiguratorWindow.cs")]
    subprocess.run([*args, "/target:winexe", f"/out:{exe}", *sources], check=True)
    if "--test" in sys.argv:
        tester = OUT / "check_e22.exe"
        subprocess.run([*args, "/target:exe", "/main:Eastway.Radio.Checks", f"/out:{tester}",
                        *sources, str(HERE / "tests/Checks.cs")], check=True)
        subprocess.run([str(tester)], cwd=OUT, check=True)
    print(exe)


if __name__ == "__main__":
    build()
