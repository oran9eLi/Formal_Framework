/** @file ConfiguratorWindow.cs @brief 东创大为 E22 本地配置窗口；串口事务在后台运行。 */
using System;
using System.ComponentModel;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Reflection;
using System.Windows.Forms;

[assembly: AssemblyTitle("东创大为 LoRa 配置助手")]
[assembly: AssemblyCompany("江苏东创大为智能科技有限公司")]
[assembly: AssemblyProduct("东创大为 LoRa 配置助手")]
[assembly: AssemblyVersion("1.1.0.0")]
[assembly: AssemblyFileVersion("1.1.0.0")]

namespace Eastway.Radio
{
    /** @brief 本地参数编辑器。更换目标串口后重新读取设备能力和参数。 */
    internal sealed class ConfiguratorWindow : Form
    {
        private static readonly Color Blue = Color.FromArgb(19,43,136), Green = Color.FromArgb(27,126,65);
        private static readonly Color Ink = Color.FromArgb(30,40,61), Muted = Color.FromArgb(77,93,116);
        private static readonly Color Canvas = Color.FromArgb(239,243,249);
        private readonly ComboBox ports = Choice();
        private readonly ComboBox baud = Choice("1200","2400","4800","9600","19200","38400","57600","115200");
        private readonly ComboBox parity = Choice("8N1","8O1","8E1","8N1（编码 3）");
        private readonly ComboBox air = Choice("2.4（编码 0）","2.4（编码 1）","2.4","4.8","9.6","19.2","38.4","62.5");
        private readonly ComboBox packet = Choice("240","128","64","32"), power = Choice();
        private readonly ComboBox mode = Choice("透明传输","定点传输"), wor = Choice("接收方","发送方");
        private readonly ComboBox period = Choice("500","1000","1500","2000","2500","3000","3500","4000");
        private readonly NumericUpDown channel = Number(83), address = Number(65535), network = Number(255);
        private readonly CheckBox relay = Option("启用中继"), lbt = Option("发送前监听（LBT）");
        private readonly CheckBox noise = Option("读取信道噪声（RSSI）"), rssi = Option("数据附加 RSSI");
        private readonly CheckBox software = Option("允许串口指令切换模式"), changeKey = Option("同时写入通信密钥");
        private readonly TextBox key = new TextBox { MaxLength=5, UseSystemPasswordChar=true };
        private readonly Label hint = Caption(""), status = Caption(""), raw = Caption(""), frequency = Caption("");
        private readonly Button refresh = ActionButton("刷新串口",false), read = ActionButton("读取参数",false);
        private readonly Button kit = ActionButton("填入实验箱预设",false), write = ActionButton("写入并校验",true);
        private readonly Button profiles = ActionButton("配置文件 ▾",false);
        private readonly ContextMenuStrip profileMenu = new ContextMenuStrip();
        private readonly TableLayoutPanel cards = Grid(3,1), connection = Grid(4,2);
        private readonly PictureBox logo = new PictureBox();
        private DeviceSnapshot device;
        private bool busy;

        /** @brief 初始化品牌页头、配置项和异步操作入口；启动时仅枚举 COM 端口。 */
        internal ConfiguratorWindow()
        {
            Text = "LoRa 配置助手 · 东创大为";
            Font = new Font("Microsoft YaHei UI",9.5f);
            Icon = Icon.ExtractAssociatedIcon(Assembly.GetExecutingAssembly().Location);
            ForeColor = Ink; BackColor = Canvas; DoubleBuffered = true;
            AutoScaleDimensions = new SizeF(96,96); AutoScaleMode = AutoScaleMode.Dpi;
            ClientSize = new Size(1070,668); MinimumSize = new Size(680,490);
            StartPosition = FormStartPosition.CenterScreen;
            AutoScroll = true;
            var root = Grid(1,5);
            root.Dock = DockStyle.Top; root.Height = 659; root.MinimumSize = new Size(940,659);
            root.Padding = new Padding(16,8,16,4); root.BackColor = Canvas;
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,100));
            foreach (int size in new[] { 83,100,338,61,65 }) root.RowStyles.Add(new RowStyle(SizeType.Absolute,size));
            Controls.Add(root);

            var brand = Grid(2,2); brand.BackColor = Color.White; brand.Margin = new Padding(0,0,0,8);
            brand.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute,125)); brand.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,100));
            brand.RowStyles.Add(new RowStyle(SizeType.Percent,60)); brand.RowStyles.Add(new RowStyle(SizeType.Percent,40));
            using (Stream s = Assembly.GetExecutingAssembly().GetManifestResourceStream("eastway.logo"))
            using (Image original = Image.FromStream(s)) logo.Image = new Bitmap(original);
            logo.Dock = DockStyle.Fill; logo.Margin = new Padding(10,5,12,5); logo.SizeMode = PictureBoxSizeMode.Zoom;
            brand.Controls.Add(logo,0,0); brand.SetRowSpan(logo,2);
            var title = Caption("LoRa 配置助手"); title.Font = new Font(Font.FontFamily,17,FontStyle.Bold);
            title.TextAlign = ContentAlignment.BottomLeft; title.ForeColor = Blue;
            brand.Controls.Add(title,1,0); var company = Caption("江苏东创大为智能科技有限公司");
            company.TextAlign = ContentAlignment.TopLeft; brand.Controls.Add(company,1,1); root.Controls.Add(brand,0,0);

            connection.BackColor=Color.White; connection.Padding=new Padding(14,5,12,2); connection.Margin = new Padding(0,0,0,8);
            connection.RowStyles.Add(new RowStyle(SizeType.Absolute,43)); connection.RowStyles.Add(new RowStyle(SizeType.Percent,100));
            foreach(int w in new[]{60,170,126}) connection.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute,w));
            connection.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,100));
            connection.Controls.Add(Caption("串口"),0,0); connection.Controls.Add(ports,1,0); connection.Controls.Add(refresh,2,0);
            refresh.Anchor=AnchorStyles.Left; refresh.Width=108;
            connection.Controls.Add(read,3,0); read.Anchor=AnchorStyles.Left; read.Width=128;
            connection.Controls.Add(hint,0,1); connection.SetColumnSpan(hint,4); root.Controls.Add(connection,0,1);

            foreach(int pct in new[]{34,33,33}) cards.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,pct));
            cards.RowStyles.Add(new RowStyle(SizeType.Percent,100)); root.Controls.Add(cards,0,2);
            TableLayoutPanel radio = Card("无线与串口",0), addressing = Card("网络与寻址",1), advanced = Card("高级参数",2);
            Field(radio,1,"串口速率（bps）",baud); Field(radio,2,"串口格式",parity);
            Field(radio,3,"空中速率（kbps）",air); Field(radio,4,"分包长度（字节）",packet);
            Field(radio,5,"发射功率（dBm）",power); Wide(radio,6,software);
            Field(addressing,1,"模块地址",address); Field(addressing,2,"网络 ID",network);
            Field(addressing,3,"频率信道",channel); Field(addressing,4,"中心频率",frequency);
            Field(addressing,5,"传输方式",mode); Field(addressing,6,"WOR 角色",wor);
            Field(addressing,7,"WOR 周期（ms）",period);
            Wide(advanced,1,relay); Wide(advanced,2,lbt); Wide(advanced,3,noise); Wide(advanced,4,rssi);
            Wide(advanced,5,changeKey); Field(advanced,6,"新密钥（十进制）",key);
            var keyNote = Caption("密钥只写，范围 0–65535。"); Wide(advanced,7,keyNote);

            var actions = new FlowLayoutPanel { Dock=DockStyle.Fill, FlowDirection=FlowDirection.LeftToRight,
                WrapContents=false, BackColor=Color.White, Padding=new Padding(12,9,8,0), Margin=new Padding(0,5,0,0) };
            foreach(Button b in new[]{kit,write,profiles}) { b.Width=150; actions.Controls.Add(b); }
            profileMenu.Items.Add("导出当前编辑参数…",null,delegate { TransferProfile(true); });
            profileMenu.Items.Add("从文件填入参数…",null,delegate { TransferProfile(false); });
            profiles.Click += delegate { profileMenu.Show(profiles,0,profiles.Height); };
            root.Controls.Add(actions,0,3);
            var foot=Grid(1,2); foot.Padding=new Padding(12,1,8,2); foot.BackColor=Color.White;
            foot.RowStyles.Add(new RowStyle(SizeType.Absolute,29)); foot.RowStyles.Add(new RowStyle(SizeType.Percent,100));
            foot.Controls.Add(status,0,0); foot.Controls.Add(raw,0,1); root.Controls.Add(foot,0,4);

            channel.ValueChanged += delegate { UpdateFrequency(); };
            changeKey.CheckedChanged += delegate { key.Enabled=changeKey.Checked; if (!changeKey.Checked) key.Clear(); };
            ports.SelectedIndexChanged += delegate { ResetTarget(); };
            refresh.Click += delegate { ScanPorts(); }; read.Click += delegate { StartTransaction(false); };
            kit.Click += delegate { ShowSettings(Settings.Kit(device.Model)); ClearKey();
                SetStatus("实验箱预设已填入；点击「写入并校验」后生效。原通信密钥会保留。",Blue); };
            write.Click += delegate { StartTransaction(true); };
            FormClosing += delegate(object sender, FormClosingEventArgs e) {
                if (busy) { e.Cancel=true; SetStatus("正在与模块通信，完成后即可关闭窗口。",Muted); } };
            ResetTarget(); ScanPorts();
        }

        private static TableLayoutPanel Grid(int cols,int rows)
        {
            return new TableLayoutPanel { ColumnCount=cols, RowCount=rows, Dock=DockStyle.Fill, Margin=Padding.Empty };
        }
        private static Label Caption(string text)
        {
            return new Label { Text=text, Dock=DockStyle.Fill, TextAlign=ContentAlignment.MiddleLeft,
                ForeColor=Muted, Margin=new Padding(3,0,3,0), AutoEllipsis=true };
        }
        private static ComboBox Choice(params string[] items)
        {
            var box=new ComboBox { DropDownStyle=ComboBoxStyle.DropDownList, Anchor=AnchorStyles.Left|AnchorStyles.Right };
            box.Items.AddRange(items); if(items.Length>0) box.SelectedIndex=0;
            return box;
        }
        private static CheckBox Option(string text)
        {
            return new CheckBox { Text=text, Dock=DockStyle.Fill, Margin=new Padding(3,0,1,0) };
        }
        private static NumericUpDown Number(int max)
        {
            return new NumericUpDown { Minimum=0, Maximum=max, Anchor=AnchorStyles.Left|AnchorStyles.Right };
        }
        private static Button ActionButton(string text,bool primary)
        {
            var b=new Button { Text=text, Height=34, FlatStyle=FlatStyle.Flat, Cursor=Cursors.Hand,
                BackColor=primary?Blue:Color.White, ForeColor=primary?Color.White:Blue, UseVisualStyleBackColor=false };
            b.FlatAppearance.BorderColor=primary?Blue:Color.FromArgb(177,190,214);
            return b;
        }
        private TableLayoutPanel Card(string text,int column)
        {
            var card=Grid(2,9); card.BackColor=Color.White;
            card.Margin=new Padding(column==0?0:4,0,column==2?0:4,0); card.Padding=new Padding(12,6,12,4);
            card.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute,119)); card.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,100));
            card.RowStyles.Add(new RowStyle(SizeType.Absolute,39));
            for(int i=1;i<=7;i++) card.RowStyles.Add(new RowStyle(SizeType.Absolute,38));
            card.RowStyles.Add(new RowStyle(SizeType.Percent,100));
            var title=Caption(text); title.Font=new Font(Font.FontFamily,12,FontStyle.Bold); title.ForeColor=Blue;
            Wide(card,0,title); cards.Controls.Add(card,column,0); return card;
        }
        private static void Wide(TableLayoutPanel panel,int row,Control control)
        {
            panel.Controls.Add(control,0,row); panel.SetColumnSpan(control,2);
        }
        private static void Field(TableLayoutPanel panel,int row,string name,Control input)
        {
            panel.Controls.Add(Caption(name),0,row); panel.Controls.Add(input,1,row);
            if(input is TextBox) input.Anchor=AnchorStyles.Left|AnchorStyles.Right;
        }
        private void SetStatus(string text,Color color) { status.ForeColor=color; status.Text=text; }
        private void UpdateFrequency()
        {
            frequency.Text=(410.125m+channel.Value).ToString("F3",CultureInfo.InvariantCulture)+" MHz";
        }
        private void ClearKey() { changeKey.Checked=false; key.Clear(); key.Enabled=false; }

        /** @brief 仅按用户操作刷新端口。优先保留已有选择，避免反复重绘或抢占其他程序。 */
        private void ScanPorts()
        {
            string previous=ports.Text;
            try {
                string[] names=SerialPort.GetPortNames().OrderBy(s=>s.Length).ThenBy(s=>s).ToArray();
                if (names.SequenceEqual(ports.Items.Cast<string>())) return;
                ports.Items.Clear(); ports.Items.AddRange(names);
                if(names.Length>0) ports.SelectedIndex=Math.Max(0,Array.IndexOf(names,previous));
                ResetTarget();
            } catch(Exception e) { SetStatus("串口枚举失败："+e.Message,Color.Firebrick); }
        }
        private void ResetTarget()
        {
            device=null; raw.Text="设备回读　—"; power.Items.Clear();
            hint.Text="进入配置模式后读取参数。白色 USB 模块长按按键超过 1.5 秒，松开至红灯常亮；绿色模块直连 USB 转 TTL，设 M1=1、M0=0。";
            ShowSettings(Settings.Kit(Model.Common22)); ClearKey();
            SetStatus(ports.SelectedIndex<0?"未选择串口。插入模块后点击「刷新串口」。":"当前显示实验箱参考值。点击「读取参数」查看模块的实际配置。",Muted);
            UpdateEnabled();
        }
        private void UpdateEnabled()
        {
            connection.Enabled=!busy; read.Enabled=!busy && ports.SelectedIndex>=0;
            bool edit=!busy && device!=null;
            cards.Enabled=edit; kit.Enabled=edit; write.Enabled=edit; profiles.Enabled=edit;
        }
        private void ShowSettings(Settings p)
        {
            baud.SelectedIndex=p.Baud; parity.SelectedIndex=p.Parity; air.SelectedIndex=p.Air;
            packet.SelectedIndex=p.Packet; power.SelectedIndex=device==null?-1:p.Power; channel.Value=p.Channel;
            address.Value=p.Address; network.Value=p.Network; mode.SelectedIndex=p.Fixed?1:0;
            bool hasWor=device!=null && device.Model==Model.Ttl30;
            wor.SelectedIndex=hasWor?(p.WorTx?1:0):-1;
            period.SelectedIndex=hasWor?p.Period:-1;
            wor.Enabled=period.Enabled=hasWor;
            software.Enabled=device!=null && Settings.SoftwareMask(device.Model)!=0;
            relay.Checked=p.Relay; lbt.Checked=p.Lbt; rssi.Checked=p.AppendRssi; noise.Checked=p.Noise;
            software.Checked=p.SoftwareMode; UpdateFrequency();
        }
        private Settings EditorSettings()
        {
            return new Settings { Baud=baud.SelectedIndex, Parity=parity.SelectedIndex, Air=air.SelectedIndex,
                Packet=packet.SelectedIndex, Power=power.SelectedIndex, Channel=(int)channel.Value,
                Address=(int)address.Value, Network=(int)network.Value, Fixed=mode.SelectedIndex==1,
                WorTx=wor.SelectedIndex==1, Period=period.SelectedIndex, Relay=relay.Checked,
                Lbt=lbt.Checked, AppendRssi=rssi.Checked, Noise=noise.Checked, SoftwareMode=software.Checked };
        }

        /** @brief 使用设备身份检查配置文件，填入后仍需用户执行写入。 */
        private void TransferProfile(bool saving)
        {
            if(busy || device==null) return;
            using(FileDialog dialog=saving?(FileDialog)new SaveFileDialog():new OpenFileDialog())
            {
                dialog.Filter="东创大为 E22 配置 (*.e22cfg)|*.e22cfg"; dialog.DefaultExt="e22cfg";
                if(saving) dialog.FileName=ProfileFile.Name(device.Model)+".e22cfg";
                if(dialog.ShowDialog(this)!=DialogResult.OK) return;
                try {
                    if(saving) {
                        string data=ProfileFile.Format(device.Model,EditorSettings().Encode(device.Data,device.Model));
                        File.WriteAllText(dialog.FileName,data);
                        SetStatus("已导出编辑区参数。通信密钥不存入文件。",Green);
                    } else {
                        if(new FileInfo(dialog.FileName).Length>1024) throw new InvalidDataException("配置文件过大。");
                        Settings loaded=ProfileFile.Parse(device.Model,File.ReadAllText(dialog.FileName),device.Data);
                        ShowSettings(loaded); ClearKey();
                        SetStatus("文件参数已填入。点击「写入并校验」后生效。",Blue);
                    }
                } catch(Exception e) { SetStatus("配置文件操作失败："+e.Message,Color.Firebrick); }
            }
        }

        /** @brief 捕获编辑快照后在后台独占端口；仅成功回读后显示已核对。 */
        private void StartTransaction(bool writing)
        {
            if(busy || ports.SelectedIndex<0 || (writing && device==null)) return;
            DeviceSnapshot initial=device; string name=ports.Text;
            int? secret=null;
            if(writing && changeKey.Checked) {
                ushort parsed;
                if(!ushort.TryParse(key.Text, NumberStyles.None, CultureInfo.InvariantCulture, out parsed)) {
                    SetStatus("请输入 0–65535 的十进制通信密钥。",Color.Firebrick); key.Focus(); return;
                }
                secret=parsed;
            }
            byte[] target=writing?EditorSettings().Encode(initial.Data,initial.Model):null;
            bool unchanged=writing && !secret.HasValue && initial.Data.SequenceEqual(target);
            busy=true; UpdateEnabled(); SetStatus(writing?"正在核对设备并处理写入…":"正在读取设备参数…",Blue);
            var worker=new BackgroundWorker();
            worker.DoWork += delegate(object sender,DoWorkEventArgs e) {
                using(var link=new SerialLink(name)) e.Result=writing
                    ? new DeviceSnapshot(initial.Model,Radio.WriteVerified(link,initial.Model,initial.Data,target,secret)) : Radio.Inspect(link);
            };
            worker.RunWorkerCompleted += delegate(object sender,RunWorkerCompletedEventArgs e) {
                busy=false; worker.Dispose(); ClearKey();
                if(e.Error!=null) {
                    ResetTarget();
                    string message=e.Error is TimeoutException ? "模块应答超时，请确认配置模式和所选串口。" :
                        e.Error is UnauthorizedAccessException ? "无法打开串口，请先关闭占用它的串口助手或亿佰特软件。" : e.Error.Message;
                    SetStatus(message+(writing?"　请重新读取设备确认当前值。":""),Color.Firebrick);
                } else {
                    ApplySnapshot((DeviceSnapshot)e.Result);
                    raw.Text="设备回读　"+BitConverter.ToString(device.Data).Replace('-',' ')+"     ·     "+name+
                        "     ·     "+DateTime.Now.ToString("HH:mm:ss");
                    string result=!writing?"已读取模块参数，可编辑或填入实验箱预设。":
                        unchanged?"设备参数与目标一致，已核对。":"参数已写入，7 个可读寄存器已回读一致。";
                    if(secret.HasValue) result+="　密钥写入已应答，仅支持写入。";
                    if(!writing && Settings.SoftwareMask(device.Model)==0) result+="　专用参数保留原值。";
                    SetStatus(result,Green);
                }
                UpdateEnabled();
            };
            worker.RunWorkerAsync();
        }

        /** @brief 收到设备回复后再填入相应功率档和专用字段。 */
        private void ApplySnapshot(DeviceSnapshot result)
        {
            device=result; power.Items.Clear();
            power.Items.AddRange(Settings.HighPower(result.Model)?new[]{"30","27","24","21"}:new[]{"22","17","13","10"});
            ShowSettings(Settings.Decode(result.Data,result.Model));
            UpdateEnabled();
        }

        protected override void Dispose(bool disposing)
        {
            if(disposing && logo.Image!=null) { logo.Image.Dispose(); logo.Image=null; }
            if(disposing) profileMenu.Dispose();
            base.Dispose(disposing);
        }
        [STAThread] internal static void Main()
        {
            Application.EnableVisualStyles(); Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new ConfiguratorWindow());
        }
    }
}
