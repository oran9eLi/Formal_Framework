using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows.Forms;

[assembly: System.Reflection.AssemblyTitle("MAVLink 中文串口助手")]
[assembly: System.Reflection.AssemblyCompany("江苏东创大为智能科技有限公司")]
[assembly: System.Reflection.AssemblyProduct("东创大为 MAVLink 中文串口助手")]
[assembly: System.Reflection.AssemblyVersion("1.2.0.0")]
[assembly: System.Reflection.AssemblyFileVersion("1.2.0.0")]

namespace Cns.Monitor
{
    /// <summary>收包与界面在同一 Windows 消息线程分批处理，断开后释放串口所有权。</summary>
    internal sealed class MonitorWindow : Form
    {
        private readonly ComboBox ports = new ComboBox(), speeds = new ComboBox();
        private readonly Button connect = new Button(), refresh = new Button();
        private readonly Label status = new Label(), counters = new Label(), detailsLabel = new Label();
        private readonly ListView frames = new BufferedFrameList();
        private readonly QuietDetailBox detail = new QuietDetailBox();
        private readonly CheckBox follow = new CheckBox(), wrap = new CheckBox();
        private readonly System.Windows.Forms.Timer timer = new System.Windows.Forms.Timer();
        private readonly Stopwatch clock = Stopwatch.StartNew();
        private readonly byte[] buffer = new byte[8192];
        private static readonly Color ink = Color.FromArgb(28,39,58), accent = Color.FromArgb(19,43,136);
        private static readonly Color canvas = Color.FromArgb(243,246,250), muted = Color.FromArgb(74,87,106);
        private readonly List<Packet> pending = new List<Packet>();
        private const int PresentationPeriodMs = 200;
        private long lastPresentationMs;
        private ListViewItem displayed;
        private readonly PictureBox logo = new PictureBox();
        private readonly Label linkState = new Label();
        private StreamDecoder decoder;
        private SerialPort serial;
        private long byteCount;
        private bool presenting;
        private int summaryWidth, titleWidth;
        private const int Retained = 500;

        internal MonitorWindow()
        {
            Text = "MAVLink 中文串口助手 · 东创大为";
            Icon = Icon.ExtractAssociatedIcon(typeof(MonitorWindow).Assembly.Location);
            Font = new Font("Microsoft YaHei UI", 9.5F);
            DoubleBuffered = true;
            AutoScaleMode = AutoScaleMode.Dpi;
            StartPosition = FormStartPosition.CenterScreen;
            MinimumSize = new Size(860,600);
            ClientSize = new Size(1130,750);
            BackColor = canvas; ForeColor = ink;
            var layout = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount=1, RowCount=6, Margin=Padding.Empty };
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,100));
            foreach(int h in new[]{100,63,42}) layout.RowStyles.Add(new RowStyle(SizeType.Absolute,h));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent,100));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute,32));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute,3));
            Controls.Add(layout);

            var top = new TableLayoutPanel { Dock=DockStyle.Fill, BackColor=Color.White, Margin=Padding.Empty, Padding=new Padding(20,10,22,10), ColumnCount=4, RowCount=1 };
            top.RowStyles.Add(new RowStyle(SizeType.Percent,100));
            top.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute,156)); top.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute,1));
            top.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,100)); top.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute,110));
            using (var resource=typeof(MonitorWindow).Assembly.GetManifestResourceStream("eastway.logo"))
            using (var source=Image.FromStream(resource)) logo.Image=new Bitmap(source);
            logo.SizeMode=PictureBoxSizeMode.Zoom; logo.Size=new Size(132,72); logo.Dock=DockStyle.Fill; logo.Margin=new Padding(0,2,20,2);
            logo.AccessibleName="东创大为公司标志"; top.Controls.Add(logo,0,0);
            top.Controls.Add(new Panel { Dock=DockStyle.Fill,Margin=new Padding(0,8,0,8),BackColor=Color.FromArgb(215,224,236)},1,0);
            var headings = new TableLayoutPanel { Dock=DockStyle.Fill, Margin=new Padding(19,2,0,0), ColumnCount=1, RowCount=2 };
            headings.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,100));
            headings.RowStyles.Add(new RowStyle(SizeType.Percent,64)); headings.RowStyles.Add(new RowStyle(SizeType.Percent,36));
            headings.Controls.Add(new Label {Text="MAVLink 中文串口助手",Dock=DockStyle.Fill,TextAlign=ContentAlignment.BottomLeft,
                ForeColor=accent, Font=new Font(Font.FontFamily,19F,FontStyle.Bold),Margin=Padding.Empty},0,0);
            headings.Controls.Add(new Label {Text="江苏东创大为智能科技有限公司",Dock=DockStyle.Fill,TextAlign=ContentAlignment.MiddleLeft,
                ForeColor=muted,Margin=Padding.Empty},0,1);
            top.Controls.Add(headings,2,0);
            linkState.Anchor=AnchorStyles.Right; linkState.TextAlign=ContentAlignment.MiddleCenter; linkState.Size=new Size(94,30);
            SetConnectionState(false); top.Controls.Add(linkState,3,0); layout.Controls.Add(top,0,0);

            var bar = new FlowLayoutPanel {Dock=DockStyle.Fill,BackColor=Color.White, Padding=new Padding(18,13,0,0), WrapContents=false, Margin=new Padding(0,2,0,0)};
            bar.Controls.Add(ToolLabel("串口")); ConfigureCombo(ports,100); bar.Controls.Add(ports);
            ConfigureButton(refresh,"刷新",60); refresh.Click += delegate { RefreshPorts(); }; bar.Controls.Add(refresh);
            bar.Controls.Add(ToolLabel("波特率")); ConfigureCombo(speeds,95);
            speeds.Items.AddRange(new object[]{"1200","2400","4800","9600","19200","38400","57600","115200","230400","460800","921600"});
            speeds.SelectedItem = "9600"; bar.Controls.Add(speeds);
            bar.Controls.Add(ToolLabel("8N1"));
            ConfigureButton(connect,"打开串口",96); connect.BackColor=accent; connect.ForeColor=Color.White; connect.FlatStyle=FlatStyle.Flat;
            connect.Click += delegate { if(serial==null) OpenPort(); else ClosePort("串口已关闭"); };
            bar.Controls.Add(connect); layout.Controls.Add(bar,0,1);

            var frameHead = new Panel { Dock=DockStyle.Fill, Margin=Padding.Empty };
            frameHead.Controls.Add(new Label { Text="接收记录", AutoSize=true, ForeColor=accent, Location=new Point(21,11), Font=new Font(Font,FontStyle.Bold) });
            follow.Text="跟随最新帧"; follow.Checked=true; follow.AutoSize=true; follow.Dock=DockStyle.Right; follow.Padding=new Padding(0,0,17,0);
            follow.CheckedChanged += delegate { if(follow.Checked && frames.Items.Count>0) ShowLatest(); };
            frameHead.Controls.Add(follow); layout.Controls.Add(frameHead,0,2);

            var split = new SplitContainer { Dock=DockStyle.Fill, Orientation=Orientation.Horizontal,
                Margin=new Padding(18,0,18,0), BackColor=canvas, SplitterWidth=9,
                Size=new Size(1040,520), SplitterDistance=283, Panel1MinSize=80, Panel2MinSize=80 };
            frames.View=View.Details; frames.FullRowSelect=true; frames.HideSelection=false;
            frames.MultiSelect=false; frames.Dock=DockStyle.Fill; frames.BorderStyle=BorderStyle.None; frames.ForeColor=ink;
            frames.Columns.Add("接收时间",112); frames.Columns.Add("节点",56); frames.Columns.Add("消息名称",128); frames.Columns.Add("解析摘要",720);
            frames.Scrollable=true;
            frames.SizeChanged += delegate { FitFrameColumns(); };
            frames.MouseDown += delegate(object sender,MouseEventArgs e) {
                if(frames.GetItemAt(e.X,e.Y)!=null) follow.Checked=false;
            };
            frames.SelectedIndexChanged += delegate {
                if(!presenting && frames.SelectedItems.Count==1) {
                    follow.Checked=false; ShowRow(frames.SelectedItems[0]);
                }
            };
            split.Panel1.Controls.Add(frames);
            var detailLayout = new TableLayoutPanel{Dock=DockStyle.Fill, BackColor=Color.White,Padding=new Padding(10,2,10,4),Margin=Padding.Empty,ColumnCount=1,RowCount=2};
            detailLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,100));
            detailLayout.RowStyles.Add(new RowStyle(SizeType.Absolute,30)); detailLayout.RowStyles.Add(new RowStyle(SizeType.Percent,100));
            detailsLabel.Text="帧详情"; detailsLabel.ForeColor=accent; detailsLabel.TextAlign=ContentAlignment.MiddleLeft; detailsLabel.Dock=DockStyle.Fill; detailsLabel.Font=new Font(Font,FontStyle.Bold);
            var detailHead=new Panel { Dock=DockStyle.Fill, Margin=Padding.Empty };
            wrap.Text="自动换行"; wrap.AutoSize=true; wrap.Checked=true; wrap.Dock=DockStyle.Right;
            wrap.AccessibleName="帧详情自动换行";
            detailHead.Controls.Add(detailsLabel); detailHead.Controls.Add(wrap);
            detailLayout.Controls.Add(detailHead,0,0); detail.Multiline=true; detail.ReadOnly=true;
            detail.WordWrap=true; detail.ScrollBars=ScrollBars.Both;
            wrap.CheckedChanged += delegate { detail.WordWrap=wrap.Checked; };
            detail.ReadingFrame += delegate { if(displayed!=null && !presenting) follow.Checked=false; };
            detail.BackColor=Color.White; detail.ForeColor=ink; detail.Dock=DockStyle.Fill;
            detail.Margin=new Padding(3,0,0,0); detail.BorderStyle=BorderStyle.None; detail.Text="打开 LoRa 模块对应的串口后，在接收记录中选择一帧即可查看详情。";
            detailLayout.Controls.Add(detail,0,1); split.Panel2.Controls.Add(detailLayout); layout.Controls.Add(split,0,3);

            var footer = new TableLayoutPanel{ Dock=DockStyle.Fill,ColumnCount=2,RowCount=1,Margin=new Padding(12,0,12,0)};
            footer.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,42)); footer.ColumnStyles.Add(new ColumnStyle(SizeType.Percent,58));
            status.TextAlign=ContentAlignment.MiddleLeft; status.Dock=DockStyle.Fill; status.AutoEllipsis=true; status.ForeColor=muted;
            counters.TextAlign=ContentAlignment.MiddleRight; counters.Dock=DockStyle.Fill; counters.ForeColor=muted;
            footer.Controls.Add(status,0,0); footer.Controls.Add(counters,1,0); layout.Controls.Add(footer,0,4);
            layout.Controls.Add(new Panel {Dock=DockStyle.Fill,BackColor=accent,Margin=Padding.Empty},0,5);
            decoder = new StreamDecoder(pending.Add);
            status.Text="等待打开串口"; RefreshPorts(); UpdateCounts();
            timer.Interval=70; timer.Tick += delegate { Receive(); };
            FormClosed += delegate { ClosePort("串口已关闭"); timer.Dispose(); logo.Image.Dispose(); };
        }

        private Label ToolLabel(string value)
        { return new Label {Text=value, AutoSize=true, Margin=new Padding(5,7,5,0)}; }
        private void ConfigureCombo(ComboBox box,int width)
        { box.DropDownStyle=ComboBoxStyle.DropDownList; box.Width=width; box.Margin=new Padding(2,3,7,0); }
        private void ConfigureButton(Button b,string text,int width)
        {
            b.Text=text; b.Size=new Size(width,32); b.Margin=new Padding(2,0,8,0);
            b.FlatStyle=FlatStyle.Flat; b.FlatAppearance.BorderColor=Color.FromArgb(198,210,227);
            b.BackColor=Color.White; b.ForeColor=accent; b.Cursor=Cursors.Hand;
        }

        /// <summary>连接状态仅在打开或关闭端口时更新，不参与遥测刷新。</summary>
        private void SetConnectionState(bool connected)
        {
            linkState.Text=connected ? "●  已连接" : "○  未连接";
            linkState.BackColor=connected ? Color.FromArgb(232,247,233) : Color.FromArgb(239,243,249);
            linkState.ForeColor=connected ? Color.FromArgb(25,113,48) : muted;
        }

        private void RefreshPorts()
        {
            try {
                string previous=ports.Text;
                string[] names=SerialPort.GetPortNames().Distinct(StringComparer.OrdinalIgnoreCase).OrderBy(n=>n.Length).ThenBy(n=>n).ToArray();
                ports.Items.Clear(); ports.Items.AddRange(names);
                if(names.Length>0) ports.SelectedItem=names.Contains(previous)?previous:(names.Contains("COM13")?"COM13":names[0]);
                connect.Enabled=names.Length>0;
                if(names.Length==0) status.Text="未发现串口，请插入 USB 模块后点击刷新";
            } catch(Exception ex) { status.Text="串口枚举失败："+ex.Message; }
        }

        /// <summary>按接收会话中的最长摘要分配列宽，窗口变窄时由原生列表显示横向滚动条。</summary>
        private void FitFrameColumns()
        {
            int nameWidth=Math.Max(128,titleWidth);
            if(frames.Columns[2].Width!=nameWidth) frames.Columns[2].Width=nameWidth;
            int available=frames.ClientSize.Width-frames.Columns[0].Width-frames.Columns[1].Width-nameWidth-4;
            int width=Math.Max(Math.Max(260,available),summaryWidth);
            if(frames.Columns[3].Width!=width) frames.Columns[3].Width=width;
        }

        private void OpenPort()
        {
            if(ports.SelectedItem==null) return;
            var candidate=new SerialPort(ports.Text,int.Parse(speeds.Text),Parity.None,8,StopBits.One);
            try {
                candidate.Handshake=Handshake.None; candidate.DtrEnable=false; candidate.RtsEnable=false;
                candidate.ReadTimeout=25; candidate.ReadBufferSize=32768; candidate.Open();
            } catch(Exception ex) {
                candidate.Dispose(); status.Text="打开失败："+ex.Message;
                MessageBox.Show(this,status.Text+"\r\n请关闭占用同一 COM 口的串口助手或 E22 配置软件。","串口连接",MessageBoxButtons.OK,MessageBoxIcon.Information);
                return;
            }
            serial=candidate; decoder=new StreamDecoder(pending.Add); byteCount=0;
            pending.Clear(); displayed=null; lastPresentationMs=clock.ElapsedMilliseconds-PresentationPeriodMs;
            summaryWidth=titleWidth=0;
            frames.Items.Clear(); FitFrameColumns(); detail.Text="正在等待第一帧…"; follow.Checked=true;
            ports.Enabled=speeds.Enabled=refresh.Enabled=false; connect.Text="关闭串口";
            SetConnectionState(true); status.Text="正在监听 "+serial.PortName+"  ·  "+serial.BaudRate+" / 8N1"; UpdateCounts(); timer.Start();
        }

        private void ClosePort(string message)
        {
            timer.Stop();
            if(serial!=null) {
                try { serial.Dispose(); } catch(IOException) { } catch(InvalidOperationException) { }
                serial=null;
            }
            PresentPending(clock.ElapsedMilliseconds,true);
            SetConnectionState(false);
            ports.Enabled=speeds.Enabled=refresh.Enabled=true; connect.Text="打开串口"; status.Text=message;
        }

        private void Receive()
        {
            if(serial==null) return;
            try {
                int waiting=serial.BytesToRead;
                int count=waiting>0 ? serial.Read(buffer,0,Math.Min(buffer.Length,waiting)) : 0;
                ProcessInput(buffer,count,clock.ElapsedMilliseconds);
            } catch(TimeoutException) { }
            catch(Exception ex) { ClosePort("连接已中断："+ex.Message); }
        }

        /// <summary>每轮完整处理输入和半帧超时，界面有变化才发布。保留窗口外的历史不排队。</summary>
        private void ProcessInput(byte[] bytes,int count,long now)
        {
            if(count>0) { byteCount+=count; decoder.Feed(bytes,count,now); }
            else decoder.Expire(now);
            if(pending.Count>Retained) pending.RemoveRange(0,pending.Count-Retained);
            PresentPending(now,false);
        }

        /// <summary>同批新帧合并一次列表变更、一次滚动和一次详情替换。空轮询不触碰列表。</summary>
        private void PresentPending(long now,bool force)
        {
            if(!force && now-lastPresentationMs<PresentationPeriodMs) return;
            lastPresentationMs=now;
            if(pending.Count>0) {
                presenting=true;
                frames.BeginUpdate();
                try {
                    int excess=frames.Items.Count+pending.Count-Retained;
                    for(int i=0;i<excess;i++) frames.Items.RemoveAt(0);
                    foreach(var packet in pending) AddFrame(packet);
                    FitFrameColumns();
                } finally {
                    frames.EndUpdate(); presenting=false; pending.Clear();
                }
                if(follow.Checked) ShowLatest();
            }
            UpdateCounts();
        }

        private void UpdateCounts()
        {
            string value=string.Format("接收 {0} B    有效帧 {1}    CRC 错 {2}    未识别 {3}    格式错 {4}",
                byteCount,decoder.Valid,decoder.CrcErrors,decoder.Unknown,decoder.FormatErrors);
            if(counters.Text!=value) counters.Text=value;
        }
        private void AddFrame(Packet packet)
        {
            var reading=ChineseTelemetry.Decode(packet);
            string summary=reading.Summary();
            summaryWidth=Math.Max(summaryWidth,BufferedFrameList.TextWidth(summary,frames.Font));
            titleWidth=Math.Max(titleWidth,BufferedFrameList.TextWidth(reading.Title,frames.Font));
            var row=new ListViewItem(new[]{packet.Received.ToString("HH:mm:ss.fff"),packet.Node.ToString(),reading.Title,summary});
            row.Tag=Tuple.Create(packet,reading);
            frames.Items.Add(row);
        }

        private void ShowLatest()
        {
            if(frames.Items.Count==0) return;
            presenting=true;
            try {
                var last=frames.Items[frames.Items.Count-1];
                if(!last.Selected) last.Selected=true;
                last.EnsureVisible(); ShowRow(last);
            } finally { presenting=false; }
        }
        private void ShowRow(ListViewItem row)
        {
            if(displayed==row) return;
            var data=(Tuple<Packet,Reading>)row.Tag;
            detail.ShowFrame(ChineseTelemetry.Detail(data.Item1,data.Item2));
            displayed=row;
        }
    }

    /// <summary>启用 Windows 原生列表的双缓冲绘制，批量追加期间不擦除可见列表。</summary>
    internal sealed class BufferedFrameList : ListView
    {
        private const TextFormatFlags CellFormat=TextFormatFlags.NoPrefix|TextFormatFlags.SingleLine|
            TextFormatFlags.NoPadding|TextFormatFlags.VerticalCenter|TextFormatFlags.PreserveGraphicsClipping;

        internal BufferedFrameList()
        {
            DoubleBuffered=true; OwnerDraw=true;
        }

        internal static int TextWidth(string text,Font font)
        {
            return TextRenderer.MeasureText(text,font,new Size(int.MaxValue,int.MaxValue),CellFormat).Width+16;
        }

        protected override void OnDrawColumnHeader(DrawListViewColumnHeaderEventArgs e) { e.DrawDefault=true; }

        /// <summary>长摘要从托管行读取全文绘制，避开原生列表对超长单元格文本的显示限制。</summary>
        protected override void OnDrawSubItem(DrawListViewSubItemEventArgs e)
        {
            if(e.ColumnIndex!=3 || e.SubItem.Text.Length<240) { e.DrawDefault=true; return; }
            bool selected=e.Item.Selected;
            Color background=selected?(Focused?SystemColors.Highlight:SystemColors.Control):BackColor;
            Color foreground=selected && Focused?SystemColors.HighlightText:ForeColor;
            var bounds=e.Bounds;
            using(var brush=new SolidBrush(background)) e.Graphics.FillRectangle(brush,bounds);
            bounds.Inflate(-6,0);
            TextRenderer.DrawText(e.Graphics,e.SubItem.Text,Font,bounds,foreground,CellFormat);
        }
    }

    /// <summary>将多行文本替换及定位合为一次可见绘制，控件仍支持滚动、选择及复制。</summary>
    internal sealed class QuietDetailBox : TextBox
    {
        private const int SetRedraw=0x000B;
        internal event EventHandler ReadingFrame;
        [DllImport("user32.dll", CharSet=CharSet.Auto)]
        private static extern IntPtr SendMessage(IntPtr window,int message,IntPtr wParam,IntPtr lParam);

        /// <summary>用户点击详情、滚轮或拖动滚动条时锁定当前帧，持续收包时可以阅读末尾。</summary>
        protected override void WndProc(ref Message message)
        {
            const int LeftDown=0x0201, MouseWheel=0x020A, MouseHWheel=0x020E, HScroll=0x0114, VScroll=0x0115;
            switch(message.Msg) {
                case LeftDown: case MouseWheel: case MouseHWheel: case HScroll: case VScroll:
                    if(ReadingFrame!=null) ReadingFrame(this,EventArgs.Empty);
                    break;
            }
            base.WndProc(ref message);
        }

        internal void ShowFrame(string value)
        {
            bool suspend=IsHandleCreated && Visible;
            if(suspend) SendMessage(Handle,SetRedraw,IntPtr.Zero,IntPtr.Zero);
            try { Text=value; Select(0,0); ScrollToCaret(); }
            finally {
                if(suspend) {
                    SendMessage(Handle,SetRedraw,new IntPtr(1),IntPtr.Zero);
                    Invalidate();
                }
            }
        }
    }

    internal static class Program
    {
        [STAThread]
        private static void Main()
        {
            Application.EnableVisualStyles(); Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MonitorWindow());
        }
    }
}
