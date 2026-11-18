/** @file E22Protocol.cs @brief E22 本地配置协议、参数编码与校验。此层不依赖界面。 */
using System;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;

namespace Eastway.Radio
{
    /** @brief 精确型号来自设备文本；旧版 PID 只能确认频段与功率时保留设备专用位。 */
    internal enum Model { Usb22, Ttl30, Common22, Common30 }

    /** @brief 在同一次串口会话中获取的参数与设备能力。 */
    internal sealed class DeviceSnapshot
    {
        internal readonly Model Model;
        internal readonly byte[] Data;
        internal DeviceSnapshot(Model model, byte[] data) { Model=model; Data=data; }
    }

    /** @brief 0x00 至 0x06 的可读参数。所有未显示位都从最近一次设备读取中保留。 */
    internal sealed class Settings
    {
        internal int Address, Network, Baud, Parity, Air, Packet, Power, Channel, Period;
        internal bool Noise, SoftwareMode, AppendRssi, Fixed, Relay, Lbt, WorTx;

        /** @brief 解码七个可读寄存器，并拒绝超出 400 系列范围的信道。 */
        internal static Settings Decode(byte[] data, Model model)
        {
            if (data == null || data.Length != 7 || data[5] > 83)
                throw new InvalidDataException("模块参数长度或信道超出支持范围，请检查设备并重新读取。");
            int uart = data[3], rf = data[4], mode = data[6];
            return new Settings {
                Address = data[0] * 256 + data[1], Network = data[2], Baud = uart >> 5,
                Parity = (uart >> 3) & 3, Air = uart & 7, Packet = rf >> 6, Power = rf & 3,
                Noise = (rf & 32) != 0, SoftwareMode = (rf & SoftwareMask(model)) != 0,
                Channel = data[5], AppendRssi = (mode & 128) != 0, Fixed = (mode & 64) != 0,
                Relay = (mode & 32) != 0, Lbt = (mode & 16) != 0,
                WorTx = (mode & 8) != 0, Period = mode & 7
            };
        }

        internal static bool HighPower(Model model) { return model==Model.Ttl30 || model==Model.Common30; }
        internal static int SoftwareMask(Model model)
        {
            if(model==Model.Usb22) return 4;
            if(model==Model.Ttl30) return 16;
            return 0;
        }
        private static int Bit(bool enabled, int mask) { return enabled ? mask : 0; }

        /** @brief 编码可编辑位；USB 款没有 WOR 模式，低四位沿用设备读值。 */
        internal byte[] Encode(byte[] original, Model model)
        {
            int soft = SoftwareMask(model);
            int rf = (original[4] & (0x1C & ~soft)) | (Packet << 6) | Bit(Noise, 32) |
                Bit(SoftwareMode, soft) | Power;
            int low = model == Model.Ttl30 ? Bit(WorTx, 8) | Period : original[6] & 15;
            return new byte[] { (byte)(Address >> 8), (byte)Address, (byte)Network,
                (byte)((Baud << 5) | (Parity << 3) | Air), (byte)rf, (byte)Channel,
                (byte)(Bit(AppendRssi,128) | Bit(Fixed,64) | Bit(Relay,32) | Bit(Lbt,16) | low) };
        }

        /** @brief 实验箱配套模板，只描述可读字段，密钥需单独显式输入。 */
        internal static Settings Kit(Model model)
        {
            return Decode(new byte[] { 0, 0, 0, 0x64, 0, 23, 3 }, model);
        }
    }

    /** @brief 公司配置文件使用固定版本、型号标识与七个公开寄存器，密钥不落盘。 */
    internal static class ProfileFile
    {
        internal static string Name(Model model)
        {
            switch(model) {
                case Model.Usb22: return "E22-400T22U";
                case Model.Ttl30: return "E22-400T30D";
                case Model.Common22: return "E22-400-22dBm";
                default: return "E22-400-30dBm";
            }
        }
        internal static string Format(Model model, byte[] data)
        {
            Settings.Decode(data,model);
            return "EASTWAY-E22/1\n"+Name(model)+"\n"+BitConverter.ToString(data)+"\n";
        }
        internal static Settings Parse(Model model, string content, byte[] baseline)
        {
            string[] lines=content.Trim().Replace("\r","").Split('\n');
            if(lines.Length!=3 || lines[0]!="EASTWAY-E22/1") throw new InvalidDataException("配置文件格式无效。");
            Model source;
            if (lines[1]==Name(model)) source=model;
            else {
                Model exact=Settings.HighPower(model)?Model.Ttl30:Model.Usb22;
                Model common=Settings.HighPower(model)?Model.Common30:Model.Common22;
                if(lines[1]!=Name(exact) && lines[1]!=Name(common))
                    throw new InvalidDataException("文件参数与已连接设备不匹配。");
                source=common;
            }
            string[] fields=lines[2].Split('-');
            if(fields.Length!=7 || fields.Any(f=>f.Length!=2 || !f.All(Uri.IsHexDigit)))
                throw new InvalidDataException("配置文件参数无效。");
            var loaded=Settings.Decode(fields.Select(f=>Convert.ToByte(f,16)).ToArray(),source);
            return Settings.Decode(loaded.Encode(baseline,source),model);
        }
    }

    /** @brief 每次由调用方独占一个配置端口，请求和响应按同一事务处理。 */
    internal interface IExchange {
        byte[] Exchange(byte[] command);
        string QueryDeviceType();
    }

    /** @brief 本地串口始终使用模块配置模式的 9600 8N1；退出事务即释放 COM。 */
    internal sealed class SerialLink : IDisposable, IExchange
    {
        private readonly SerialPort port;
        internal SerialLink(string name)
        {
            port = new SerialPort(name, 9600, Parity.None, 8, StopBits.One) { WriteTimeout = 750 };
            try { port.Open(); port.DiscardInBuffer(); }
            catch { port.Dispose(); throw; }
        }
        public void Dispose() { port.Dispose(); }

        /** @brief 发只读设备名查询。按静默间隔收尾，兼容无换行及分段返回。 */
        public string QueryDeviceType()
        {
            port.Write("AT+DEVTYPE=?");
            byte[] buffer=new byte[128]; int size=0;
            var timer=Stopwatch.StartNew();
            while(size<buffer.Length && timer.ElapsedMilliseconds<1000) {
                port.ReadTimeout=Math.Max(1,Math.Min(size==0?1000:120,1000-(int)timer.ElapsedMilliseconds));
                try { size+=port.Read(buffer,size,buffer.Length-size); }
                catch(TimeoutException) { break; }
            }
            return Encoding.ASCII.GetString(buffer,0,size).Trim('\0','\r','\n',' ');
        }

        /** @brief 收齐固定长度回包，超时按绝对期限计算，避免拆包或持续噪声造成卡顿。 */
        public byte[] Exchange(byte[] command)
        {
            port.Write(command, 0, command.Length);
            byte[] answer = new byte[3 + command[2]];
            var timer = Stopwatch.StartNew();
            int received = 0;
            while (received < answer.Length)
            {
                int remaining = 1600 - (int)timer.ElapsedMilliseconds;
                if (remaining <= 0) throw new TimeoutException();
                port.ReadTimeout = remaining;
                int count = port.Read(answer, received, (received < 3 ? 3 : answer.Length) - received);
                received += count;
                if (received >= 3)
                    Radio.CheckHeader(command, answer);
            }
            return answer;
        }
    }

    /** @brief 读、持久化写和独立回读；密钥只写，绝不从回读的零值推断密钥。 */
    internal static class Radio
    {
        /** @brief 确认参数应答后查设备名；老版本使用产品信息区，无法判定时禁止写入。 */
        internal static DeviceSnapshot Inspect(IExchange link)
        {
            byte[] data=Payload(link,new byte[]{0xc1,0,7});
            string reply=link.QueryDeviceType();
            string line=reply.Split(new[]{'\r','\n','\0'},StringSplitOptions.RemoveEmptyEntries)
                .Select(s=>s.Trim()).FirstOrDefault(s=>s.StartsWith("DEVTYPE=",StringComparison.OrdinalIgnoreCase));
            Model model;
            if(line!=null) {
                string name=line.Substring(8).Trim().ToUpperInvariant();
                switch(name) {
                    case "E22-400T22U": case "E22-400T22U/U": model=Model.Usb22; break;
                    case "E22-400T30D": case "E22-400T30D/D": case "E22-400T30S/D": model=Model.Ttl30; break;
                    default: throw new InvalidDataException("当前助手暂不支持此设备，已停止配置。");
                }
            } else {
                var pid=Payload(link,new byte[]{0xc1,0x80,7});
                if(pid[0]!=0 || pid[1]!=0x22 || (pid[4]!=4 && pid[4]!=10) || (pid[3]!=22 && pid[3]!=30))
                    throw new InvalidDataException("未识别到配套 E22 设备，已停止配置。");
                model=pid[3]==30?Model.Common30:Model.Common22;
            }
            Settings.Decode(data,model);
            return new DeviceSnapshot(model,data);
        }
        internal static void CheckHeader(byte[] request, byte[] reply)
        {
            if (reply == null || reply.Length < 3 || reply[0] != 0xC1 ||
                reply[1] != request[1] || reply[2] != request[2])
                throw new InvalidDataException("模块返回了非预期的配置应答，请确认模块已进入配置模式。");
        }

        private static byte[] Payload(IExchange link, byte[] request)
        {
            byte[] reply = link.Exchange(request);
            CheckHeader(request, reply);
            if (reply.Length != 3 + request[2]) throw new InvalidDataException("模块应答长度不完整。");
            return reply.Skip(3).ToArray();
        }

        internal static byte[] Read(IExchange link, Model model)
        {
            byte[] data = Payload(link, new byte[] { 0xC1, 0, 7 });
            Settings.Decode(data, model);
            return data;
        }

        /** @brief 先核对设备基线，再写入并单独回读。按需在同一条写命令中设置只写密钥。 */
        internal static byte[] WriteVerified(IExchange link, Model model, byte[] baseline, byte[] target, int? key)
        {
            Settings.Decode(target, model);
            var device=Inspect(link);
            if (device.Model!=model || !device.Data.SequenceEqual(baseline))
                throw new InvalidDataException("设备参数在上次读取后发生了变化。请重新读取，再确认修改。");
            if (target.SequenceEqual(baseline) && !key.HasValue) return (byte[])baseline.Clone();
            byte[] request = new byte[3 + (key.HasValue ? 9 : 7)];
            request[0] = 0xC0; request[2] = (byte)(request.Length - 3);
            Array.Copy(target, 0, request, 3, 7);
            if (key.HasValue)
            {
                if (key.Value < 0 || key.Value > 65535) throw new ArgumentOutOfRangeException("key");
                request[10] = (byte)(key.Value >> 8); request[11] = (byte)key.Value;
            }
            byte[] ack = Payload(link, request);
            if (!ack.Take(7).SequenceEqual(target)) throw new InvalidDataException("写入应答与目标值不一致，请重新读取模块。");
            Thread.Sleep(120);
            byte[] actual = Read(link, model);
            if (!actual.SequenceEqual(target)) throw new InvalidDataException("写入后回读不一致，请重新读取并核对参数。");
            return actual;
        }
    }
}
