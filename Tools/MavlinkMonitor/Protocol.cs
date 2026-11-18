using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;

namespace Cns.Monitor
{
    /// <summary>从构建时嵌入的工程方言表获取 CRC、字段类型和实际线序。</summary>
    internal sealed class MessageDefinition
    {
        internal int Id, CrcExtra, MinLength, MaxLength;
        internal string Name;
        internal readonly List<Field> Fields = new List<Field>();
        internal readonly Dictionary<string, Field> ByName = new Dictionary<string, Field>();
        internal static readonly Dictionary<int, MessageDefinition> Messages = Load();

        private static Dictionary<int, MessageDefinition> Load()
        {
            var messages = new Dictionary<int, MessageDefinition>();
            using (var reader = new StreamReader(Assembly.GetExecutingAssembly().GetManifestResourceStream("dialect")))
            {
                string line;
                while ((line = reader.ReadLine()) != null)
                {
                    string[] cols = line.Split('|');
                    var definition = new MessageDefinition {
                        Id = int.Parse(cols[0]), Name = cols[1], CrcExtra = int.Parse(cols[2]),
                        MinLength = int.Parse(cols[3]), MaxLength = int.Parse(cols[4])
                    };
                    foreach (string field in cols[5].Split(';'))
                    {
                        string[] f = field.Split(',');
                        var entry = new Field { Name = f[0], Format = f[1][0], Count = int.Parse(f[2]), Offset = int.Parse(f[3]) };
                        definition.Fields.Add(entry);
                        definition.ByName.Add(entry.Name, entry);
                    }
                    messages.Add(definition.Id, definition);
                }
            }
            return messages;
        }
    }

    internal sealed class Field
    {
        internal string Name;
        internal char Format;
        internal int Count, Offset;

        internal object ReadNumber(byte[] p, int pos)
        {
            switch (Format)
            {
                case 'b': return (sbyte)p[pos];
                case 'B': case 'c': return p[pos];
                case 'h': return BitConverter.ToInt16(p, pos);
                case 'H': return BitConverter.ToUInt16(p, pos);
                case 'i': return BitConverter.ToInt32(p, pos);
                case 'I': return BitConverter.ToUInt32(p, pos);
                case 'q': return BitConverter.ToInt64(p, pos);
                case 'Q': return BitConverter.ToUInt64(p, pos);
                case 'f': return BitConverter.ToSingle(p, pos);
                case 'd': return BitConverter.ToDouble(p, pos);
                default: throw new InvalidDataException("未知字段类型 " + Format);
            }
        }

        internal string Read(byte[] payload)
        {
            if (Count == 0) return Convert.ToString(ReadNumber(payload, Offset), CultureInfo.InvariantCulture);
            if (Format == 'c') return Packet.Text(payload, Offset, Count);
            int size = Buffer.ByteLength(Array.CreateInstance(ReadNumber(payload, Offset).GetType(), 1));
            var values = new string[Count];
            for (int i = 0; i < Count; i++)
                values[i] = Convert.ToString(ReadNumber(payload, Offset + i * size), CultureInfo.InvariantCulture);
            return string.Join(", ", values);
        }
    }

    /// <summary>已知消息通过 CRC 后才发布。补零载荷仅用于解释 MAVLink 2 尾零裁剪。</summary>
    internal sealed class Packet
    {
        internal MessageDefinition Definition;
        internal byte[] Wire, Payload;
        internal int Version, Node, Component, Sequence;
        internal bool Signed;
        internal DateTime Received;

        internal double Number(string key)
        {
            var field = Definition.ByName[key];
            return Convert.ToDouble(field.ReadNumber(Payload, field.Offset));
        }

        internal string StringField(string key)
        {
            var field = Definition.ByName[key];
            return Text(Payload, field.Offset, field.Count);
        }

        internal static string Text(byte[] buffer, int start, int length)
        {
            int end = Array.IndexOf(buffer, (byte)0, start, length);
            string value = Encoding.UTF8.GetString(buffer, start, end < 0 ? length : end - start);
            var clean = new StringBuilder(value.Length);
            foreach (char c in value) clean.Append(char.IsControl(c) ? ' ' : c);
            return clean.ToString();
        }
    }

    /// <summary>单线程串口流拆帧，跨读保留半帧，错误后逐字节寻找下一帧。</summary>
    internal sealed class StreamDecoder
    {
        private readonly List<byte> buffer = new List<byte>(4096);
        private long lastByteMs;
        internal long Valid, CrcErrors, Unknown, FormatErrors;
        private readonly Action<Packet> publish;

        internal StreamDecoder(Action<Packet> publish) { this.publish = publish; }

        internal void Feed(byte[] data, int length, long nowMs)
        {
            Expire(nowMs);
            lastByteMs = nowMs;
            for (int i = 0; i < length; i++) buffer.Add(data[i]);
            Parse(false);
        }

        internal void Expire(long nowMs)
        {
            if (buffer.Count > 0 && nowMs - lastByteMs >= 2000) Parse(true);
        }

        private static int Accumulate(int crc, int b)
        {
            int tmp = (b ^ crc) & 255;
            tmp ^= (tmp << 4) & 255;
            return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 65535;
        }

        private void Parse(bool expired)
        {
            int cursor = 0;
            while (cursor < buffer.Count)
            {
                byte start = buffer[cursor];
                if (start != 253 && start != 254) { cursor++; continue; }
                bool v2 = start == 253;
                int header = v2 ? 10 : 6;
                int remaining = buffer.Count - cursor;
                if (remaining < header)
                {
                    if (expired) { FormatErrors++; cursor++; continue; }
                    break;
                }
                int len = buffer[cursor + 1];
                byte flags = v2 ? buffer[cursor + 2] : (byte)0;
                if ((flags & ~1) != 0) { FormatErrors++; cursor++; continue; }
                int id = v2 ? buffer[cursor+7] | buffer[cursor+8] << 8 | buffer[cursor+9] << 16 : buffer[cursor+5];
                MessageDefinition def;
                if (!MessageDefinition.Messages.TryGetValue(id, out def)) { Unknown++; cursor++; continue; }
                if ((!v2 && len != def.MinLength) || (v2 && len == 0 && def.MaxLength != 0))
                { FormatErrors++; cursor++; continue; }
                int total = header + len + 2 + ((flags & 1) != 0 ? 13 : 0);
                if (remaining < total)
                {
                    if (expired) { FormatErrors++; cursor++; continue; }
                    break;
                }
                int checksumPos = cursor + header + len;
                int crc = 65535;
                for (int i = cursor + 1; i < checksumPos; i++) crc = Accumulate(crc, buffer[i]);
                crc = Accumulate(crc, def.CrcExtra);
                if (crc != (buffer[checksumPos] | buffer[checksumPos+1] << 8))
                { CrcErrors++; cursor++; continue; }

                byte[] raw = buffer.GetRange(cursor, total).ToArray();
                byte[] payload = new byte[Math.Max(len, def.MaxLength)];
                Array.Copy(raw, header, payload, 0, len);
                Valid++;
                publish(new Packet { Definition = def, Payload = payload, Wire = raw,
                    Version = v2 ? 2 : 1, Sequence = raw[v2 ? 4 : 2], Node = raw[v2 ? 5 : 3],
                    Component = raw[v2 ? 6 : 4], Signed = (flags & 1) != 0, Received = DateTime.Now });
                cursor += total;
            }
            if (cursor > 0) buffer.RemoveRange(0, cursor);
        }
    }
}
