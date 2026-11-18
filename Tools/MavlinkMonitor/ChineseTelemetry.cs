using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Cns.Monitor
{
    /// <summary>把协议字段换算为实验箱使用的工程单位，协议未知值保留为“未提供”。</summary>
    internal sealed class Reading
    {
        internal string Title;
        internal readonly List<KeyValuePair<string,string>> Items = new List<KeyValuePair<string,string>>();
        internal void Add(string name, string value) { Items.Add(new KeyValuePair<string,string>(name, value)); }
        internal string Summary()
        {
            var values = new List<string>();
            foreach (var item in Items) values.Add(item.Key + " " + item.Value);
            return string.Join("　·　", values.ToArray());
        }
    }

    internal static class ChineseTelemetry
    {
        private const string Missing = "未提供";
        private static readonly string[] Fix = { "无接收器", "未定位", "二维定位", "三维定位", "差分定位", "RTK 浮点", "RTK 固定", "静态定位", "精密单点定位" };
        private static readonly string[] States = { "未初始化", "启动中", "校准中", "待机", "运行中", "异常", "紧急", "已关机", "强制终止" };
        private static string EnumValue(string[] values, int index)
        { return index >= 0 && index < values.Length ? values[index] : "代码 " + index; }

        private static string Units(double value, string format, string unit)
        {
            if (double.IsNaN(value) || double.IsInfinity(value)) return Missing;
            return value.ToString(format, CultureInfo.InvariantCulture) + unit;
        }
        private static string Scaled(Packet p, string key, double unknown, double scale, string fmt, string unit)
        { double val = p.Number(key); return val == unknown ? Missing : Units(val*scale, fmt, unit); }

        private static string Sensor(Packet p, int bit)
        {
            if (((long)p.Number("onboard_control_sensors_present") & bit) == 0) return "未报告";
            if (((long)p.Number("onboard_control_sensors_enabled") & bit) == 0) return "未启用";
            return ((long)p.Number("onboard_control_sensors_health") & bit) != 0 ? "健康标志正常" : "健康异常";
        }

        private static void Battery(Reading r, Packet p)
        {
            r.Title = "供电状态";
            r.Add("输入", "第 " + ((int)p.Number("id") + 1) + " 路");
            int sum = 0, cells = 0;
            int pos = p.Definition.ByName["voltages"].Offset;
            for (int i = 0; i < 10; i++) {
                int voltage = BitConverter.ToUInt16(p.Payload, pos+i*2);
                if (voltage != 65535) { sum += voltage; cells++; }
            }
            int ext = p.Definition.ByName["voltages_ext"].Offset;
            for (int i = 0; i < 4; i++) {
                int voltage = BitConverter.ToUInt16(p.Payload, ext+i*2);
                if (voltage != 0 && voltage != 65535) { sum += voltage; cells++; }
            }
            r.Add("电压", cells > 0 ? Units(sum/1000.0,"0.000"," V") : Missing);
            r.Add("电流", Scaled(p,"current_battery",-1,.01,"0.00"," A"));
            r.Add("剩余电量", Scaled(p,"battery_remaining",-1,1,"0"," %"));
        }

        internal static Reading Decode(Packet p)
        {
            var r = new Reading();
            int fix;
            switch (p.Definition.Name)
            {
                case "HEARTBEAT":
                    r.Title = "设备心跳";
                    r.Add("节点", p.Node.ToString());
                    int type = (int)p.Number("type");
                    r.Add("设备", type == 18 ? "机载控制器" : type == 2 ? "四旋翼" : type == 6 ? "地面站" : "类型 " + type);
                    r.Add("状态", EnumValue(States,(int)p.Number("system_status")));
                    break;
                case "OPEN_DRONE_ID_BASIC_ID":
                    r.Title = "设备身份";
                    r.Add("标识", p.StringField("uas_id"));
                    r.Add("标识类型", p.Number("id_type")==1 ? "序列号" : "代码 " + (int)p.Number("id_type"));
                    break;
                case "SYS_STATUS":
                    r.Title = "系统状态";
                    r.Add("定位", Sensor(p,32)); r.Add("陀螺仪", Sensor(p,1));
                    r.Add("加速度计", Sensor(p,2)); r.Add("气压计", Sensor(p,8));
                    break;
                case "ATTITUDE":
                    r.Title = "姿态";
                    double deg = 180.0/Math.PI;
                    r.Add("横滚", Units(p.Number("roll")*deg,"0.00","°"));
                    r.Add("俯仰", Units(p.Number("pitch")*deg,"0.00","°"));
                    r.Add("偏航", Units(p.Number("yaw")*deg,"0.00","°"));
                    r.Add("横滚角速度", Units(p.Number("rollspeed")*deg,"0.00"," °/s"));
                    r.Add("俯仰角速度", Units(p.Number("pitchspeed")*deg,"0.00"," °/s"));
                    r.Add("偏航角速度", Units(p.Number("yawspeed")*deg,"0.00"," °/s"));
                    break;
                case "GPS_RAW_INT":
                    r.Title = "卫星定位"; fix = (int)p.Number("fix_type");
                    r.Add("定位", EnumValue(Fix,fix));
                    r.Add("卫星", Scaled(p,"satellites_visible",255,1,"0"," 颗"));
                    r.Add("纬度", fix>=2 ? Units(p.Number("lat")/1e7,"0.0000000","°") : Missing);
                    r.Add("经度", fix>=2 ? Units(p.Number("lon")/1e7,"0.0000000","°") : Missing);
                    r.Add("海拔", fix>=3 ? Units(p.Number("alt")/1000,"0.0"," m") : Missing);
                    r.Add("地速", Scaled(p,"vel",65535,.01,"0.00"," m/s"));
                    r.Add("航迹角", Scaled(p,"cog",65535,.01,"0.00","°"));
                    r.Add("水平精度因子", Scaled(p,"eph",65535,.01,"0.00",""));
                    break;
                case "SCALED_PRESSURE":
                    r.Title = "气压与温度";
                    r.Add("气压", Units(p.Number("press_abs"),"0.00"," hPa"));
                    r.Add("温度", Units(p.Number("temperature")/100,"0.00"," ℃"));
                    break;
                case "BATTERY_STATUS": Battery(r,p); break;
                case "NAMED_VALUE_INT":
                    string key = p.StringField("name");
                    if (key == "HUMIDITY") { r.Title = "环境湿度"; r.Add("相对湿度", Units(p.Number("value")/10,"0.0"," %RH")); }
                    else { r.Title = "命名整数"; r.Add("名称",key); r.Add("数值", Units(p.Number("value"),"0","")); }
                    break;
                case "GLOBAL_POSITION_INT":
                    r.Title = "融合位置"; r.Add("纬度",Units(p.Number("lat")/1e7,"0.0000000","°"));
                    r.Add("经度",Units(p.Number("lon")/1e7,"0.0000000","°"));
                    r.Add("海拔",Units(p.Number("alt")/1000,"0.0"," m"));
                    r.Add("相对高度",Units(p.Number("relative_alt")/1000,"0.0"," m")); break;
                case "SERVO_OUTPUT_RAW":
                    r.Title="舵机输出"; r.Add("端口",p.Number("port").ToString());
                    for(int i=1;i<=4;i++) r.Add("通道 "+i, Units(p.Number("servo"+i+"_raw"),"0"," μs"));
                    break;
                case "TUNNEL":
                    r.Title="透传数据"; r.Add("载荷类型","0x"+((int)p.Number("payload_type")).ToString("X4"));
                    r.Add("载荷长度",Units(p.Number("payload_length"),"0"," 字节")); break;
                case "COMMAND_LONG": case "COMMAND_ACK":
                    bool ack=p.Definition.Name=="COMMAND_ACK";
                    r.Title=ack?"指令应答":"长指令"; r.Add("指令号",p.Number("command").ToString());
                    if(ack) r.Add("结果",EnumValue(new[]{"已接受","暂时拒绝","已拒绝","不支持","失败","执行中","已取消"},(int)p.Number("result")));
                    break;
                case "STATUSTEXT": r.Title="状态文本"; r.Add("消息",p.StringField("text")); break;
                default:
                    r.Title = p.Definition.Name;
                    foreach (var field in p.Definition.Fields) {
                        if (r.Items.Count >= 3) break;
                        r.Add(field.Name, field.Read(p.Payload));
                    }
                    break;
            }
            return r;
        }

        internal static string Detail(Packet p, Reading r)
        {
            var s = new StringBuilder();
            s.AppendLine(r.Title + "   /   " + p.Definition.Name + "   #" + p.Definition.Id);
            s.AppendLine(string.Format("接收 {0:HH:mm:ss.fff}    节点 {1}    组件 {2}    帧序号 {3}    MAVLink {4}    CRC 通过{5}",
                p.Received,p.Node,p.Component,p.Sequence,p.Version,p.Signed ? "    携带签名（未认证）" : ""));
            foreach (var item in r.Items) s.AppendLine(item.Key + "    " + item.Value);
            s.AppendLine(); s.AppendLine("协议字段（原始值）");
            foreach (var field in p.Definition.Fields) {
                if (p.Version == 1 && field.Offset >= p.Definition.MinLength) continue;
                s.AppendLine(field.Name + "    " + field.Read(p.Payload));
            }
            s.AppendLine(); s.AppendLine("原始帧 HEX"); s.Append(BitConverter.ToString(p.Wire).Replace('-', ' '));
            return s.ToString();
        }
    }
}
