/** @file Checks.cs @brief 通过模拟配置应答与离屏窗口验证寄存器、提交状态和界面。 */
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Windows.Forms;

namespace Eastway.Radio
{
    internal sealed class SimulatedRadio : IExchange
    {
        internal byte[] Data = { 0,0,0,0x64,0,23,3 };
        internal int Key = 49152;
        internal string Fault;
        internal string TypeReply = "DEVTYPE=E22-400T22U\r\n";
        internal byte[] ProductInfo = { 0,0x22,1,22,4,0,0 };
        public string QueryDeviceType() { return TypeReply; }
        internal readonly List<byte[]> Requests = new List<byte[]>();
        public byte[] Exchange(byte[] command)
        {
            Requests.Add((byte[])command.Clone());
            if(Fault=="timeout") throw new TimeoutException();
            if(Fault=="header") return new byte[]{0xff,0xff,0xff};
            if(Fault=="address") return new byte[]{0xc1,3,7,0,0,0,0,0,0,0};
            if(Fault=="length") return new byte[]{0xc1,0,7};
            int size=command[2]; var reply=new byte[size+3]; reply[0]=0xc1; reply[1]=command[1]; reply[2]=(byte)size;
            if(command.SequenceEqual(new byte[]{0xc1,0x80,7})) {
                Array.Copy(ProductInfo,0,reply,3,7); return reply;
            }
            if(command[0]==0xc0) {
                Data=command.Skip(3).Take(7).ToArray();
                if(size==9) Key=command[10]*256+command[11];
            } else if(command[0]!=0xc1 || size!=7 || command.Length!=3) throw new Exception("unexpected request");
            Array.Copy(Data,0,reply,3,7);
            if(command[0]==0xc0 && Fault=="ack") reply[4]^=1;
            if(command[0]==0xc1 && Fault=="verify" && Requests.Count>=3) reply[8]^=1;
            return reply;
        }
    }

    internal static class Checks
    {
        private static int passed;
        private static void Check(bool ok,string name)
        {
            if(!ok) throw new Exception(name);
            passed++; Console.WriteLine("PASS "+name);
        }
        private static void Rejected<T>(Action action,string name) where T:Exception
        {
            try { action(); } catch(T) { Check(true,name); return; }
            throw new Exception("accepted "+name);
        }
        private static T Get<T>(ConfiguratorWindow form,string field) where T:Control
        {
            return (T)typeof(ConfiguratorWindow).GetField(field,BindingFlags.Instance|BindingFlags.NonPublic).GetValue(form);
        }
        private static IEnumerable<Control> Children(Control parent)
        {
            foreach(Control c in parent.Controls) { yield return c; foreach(Control sub in Children(c)) yield return sub; }
        }
        [STAThread] internal static void Main()
        {
            byte[] sample={0,0,0,0x64,0,23,3};
            var rng=new Random(20261113);
            foreach(Model model in Enum.GetValues(typeof(Model))) {
                var kit=Settings.Kit(model);
                Check(kit.Encode(sample,model).SequenceEqual(sample),"kit "+model);
                bool exact=true;
                for(int i=0;i<12000;i++) {
                    var raw=new byte[7]; rng.NextBytes(raw); raw[5]=(byte)(i%84);
                    exact &= Settings.Decode(raw,model).Encode(raw,model).SequenceEqual(raw);
                }
                Check(exact,"12000 register roundtrips "+model);
                byte[] drive={0xff,0xef,0xfc,0x9f,0x1c,83,0xff};
                var parsed=Settings.Decode(drive,model);
                parsed.SoftwareMode=false; parsed.WorTx=false; parsed.Period=3;
                byte[] result=parsed.Encode(drive,model);
                int expectedBits=model==Model.Usb22?0x18:(model==Model.Ttl30?0x0c:0x1c);
                Check(result[4]==expectedBits && result[6]==(model==Model.Ttl30?0xf3:0xff),"model bit ownership "+model);
                string stored=ProfileFile.Format(model,drive);
                Check(ProfileFile.Parse(model,stored,drive).Encode(drive,model).SequenceEqual(drive),"profile roundtrip "+model);
                Rejected<InvalidDataException>(()=>ProfileFile.Parse(Settings.HighPower(model)?Model.Usb22:Model.Ttl30,stored,drive),"model mismatch "+model);
                Rejected<InvalidDataException>(()=>ProfileFile.Parse(model,stored.Replace("FF-EF","?0-EF"),drive),"invalid profile "+model);
            }
            var legacySource=(byte[])sample.Clone(); legacySource[4]=0x10; legacySource[6]=0x0f;
            foreach(Model target in new[]{Model.Ttl30,Model.Common30}) {
                string profile=ProfileFile.Format(target==Model.Ttl30?Model.Common30:Model.Ttl30,legacySource);
                Check(ProfileFile.Parse(target,profile,sample).Encode(sample,target).SequenceEqual(sample),"shared profile merge "+target);
            }
            Check(Settings.Decode(sample,Model.Usb22).Air==4,"screenshot air 9.6k index");
            byte[] invalid=(byte[])sample.Clone(); invalid[5]=84;
            Rejected<InvalidDataException>(()=>Settings.Decode(invalid,Model.Usb22),"channel range");
            var sim=new SimulatedRadio(); var newValues=(byte[])sample.Clone(); newValues[5]=36;
            Check(Radio.WriteVerified(sim,Model.Usb22,sample,newValues,null).SequenceEqual(newValues)
                && sim.Key==49152 && sim.Requests[1].SequenceEqual(new byte[]{0xc0,0,7,0,0,0,0x64,0,36,3})
                && sim.Requests[2].SequenceEqual(new byte[]{0xc1,0,7}),"persistent write / independent verify / key retained");
            sim.Requests.Clear();
            Radio.WriteVerified(sim,Model.Usb22,newValues,newValues,0xa1b2);
            Check(sim.Key==0xa1b2 && sim.Requests[1][2]==9,"write-only key with zero ack bytes");
            sim.Requests.Clear(); Radio.WriteVerified(sim,Model.Usb22,newValues,newValues,null);
            Check(sim.Requests.Count==1 && sim.Requests[0][0]==0xc1,"unchanged uses read only");
            sim.Requests.Clear();
            Rejected<InvalidDataException>(()=>Radio.WriteVerified(sim,Model.Usb22,sample,newValues,null),"stale baseline");
            Check(sim.Requests.Count==1,"stale baseline cannot write");
            sim=new SimulatedRadio{TypeReply="DEVTYPE=E22-400T30D/D\r\n"};
            Check(Radio.Inspect(sim).Model==Model.Ttl30,"TTL exact identification");
            sim.TypeReply="DEVTYPE=E22-400T22U";
            Rejected<InvalidDataException>(()=>Radio.WriteVerified(sim,Model.Ttl30,sample,newValues,null),"device swapped same registers");
            Check(!sim.Requests.Any(c=>c[0]==0xc0),"no write to swapped module");
            sim.TypeReply="DEVTYPE=E32-433T20D";
            Rejected<InvalidDataException>(()=>Radio.Inspect(sim),"other device rejected");
            sim.TypeReply="";
            Check(Radio.Inspect(sim).Model==Model.Common22,"legacy 22 dBm capability");
            sim.ProductInfo[3]=30; sim.ProductInfo[4]=10;
            Check(Radio.Inspect(sim).Model==Model.Common30,"legacy 30 dBm capability");
            sim.ProductInfo[4]=11;
            Rejected<InvalidDataException>(()=>Radio.Inspect(sim),"other band rejected");
            sim.ProductInfo[4]=4; sim.ProductInfo[1]=0x32;
            Rejected<InvalidDataException>(()=>Radio.Inspect(sim),"unsupported PID rejected");
            foreach(string fault in new[]{"header","address","length","ack","verify"}) {
                sim=new SimulatedRadio { Fault=fault };
                Rejected<InvalidDataException>(()=>Radio.WriteVerified(sim,Model.Usb22,sample,newValues,null),"fault "+fault);
            }
            sim=new SimulatedRadio { Fault="timeout" };
            Rejected<TimeoutException>(()=>Radio.Read(sim,Model.Usb22),"timeout");
            Application.EnableVisualStyles();
            using(var form=new ConfiguratorWindow()) {
                form.ShowInTaskbar=false; form.StartPosition=FormStartPosition.Manual; form.Location=new Point(-12000,0);
                form.Show(); Application.DoEvents();
                Check(!Get<Button>(form,"write").Enabled && !Get<TextBox>(form,"key").Enabled,"startup gating");
                Check(!Children(form).Any(c=>c.Text.Contains("模块型号")) &&
                    !Children(form).OfType<ComboBox>().Any(c=>c.Items.Cast<object>().Any(i=>i.ToString().Contains("E22-"))),"no model selector or label");
                Check(Get<ComboBox>(form,"power").SelectedIndex==-1 && !Get<CheckBox>(form,"software").Enabled,"unknown device has no power rating");
                var apply=typeof(ConfiguratorWindow).GetMethod("ApplySnapshot",BindingFlags.Instance|BindingFlags.NonPublic);
                apply.Invoke(form,new object[]{new DeviceSnapshot(Model.Usb22,sample)});
                Check(Get<ComboBox>(form,"power").Text=="22" && !Get<ComboBox>(form,"wor").Enabled,"auto USB controls");
                apply.Invoke(form,new object[]{new DeviceSnapshot(Model.Common30,sample)});
                Check(!Get<ComboBox>(form,"wor").Enabled && !Get<CheckBox>(form,"software").Enabled,"legacy only shared fields");
                apply.Invoke(form,new object[]{new DeviceSnapshot(Model.Ttl30,sample)});
                Check(Get<ComboBox>(form,"power").Text=="30","auto TTL controls");
                Check(Get<ComboBox>(form,"wor").Enabled && Get<Button>(form,"kit").Enabled,"read enables edit");
                Get<NumericUpDown>(form,"channel").Value=36; Get<Button>(form,"kit").PerformClick();
                Check(Get<NumericUpDown>(form,"channel").Value==23 && Get<CheckBox>(form,"changeKey").Checked==false,"kit only populates editor");
                Check(Children(form).OfType<ComboBox>().All(c=>c.Width>70 && c.Height>=20),"input control layout");
                using(var bitmap=new Bitmap(form.ClientSize.Width,form.ClientSize.Height)) {
                    form.Controls[0].DrawToBitmap(bitmap,form.Controls[0].Bounds); bitmap.Save("ui-check.png");
                }
                form.Close();
            }
            Console.WriteLine("Total checks: "+passed);
        }
    }
}
