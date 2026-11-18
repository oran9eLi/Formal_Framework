# Windows PowerShell -STA，通过最终 EXE 接收路径验证长数组末项和原始帧尾。
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class FieldScrollProbe {
    [DllImport("user32.dll")] public static extern IntPtr SendMessage(IntPtr h,int m,IntPtr w,IntPtr l);
    [DllImport("user32.dll")] public static extern int GetScrollPos(IntPtr h,int bar);
    public static byte[] Frame() {
        byte[] bytes=new byte[267];
        bytes[0]=253; bytes[1]=255; bytes[5]=42; bytes[6]=1; bytes[7]=131;
        for(int i=0;i<253;i++) bytes[12+i]=(byte)i;
        int crc=65535;
        for(int i=1;i<266;i++){
            int value=i==265 ? 223 : bytes[i];
            int t=(value^crc)&255; t^=(t<<4)&255;
            crc=((crc>>8)^(t<<8)^(t<<3)^(t>>4))&65535;
        }
        bytes[265]=(byte)crc; bytes[266]=(byte)(crc>>8); return bytes;
    }
}
'@
$root=(Get-Item $PSScriptRoot).Parent.Parent.FullName
$exe=Join-Path $root 'build/MavlinkMonitor/dist/MAVLink中文串口助手.exe'
$assembly=[Reflection.Assembly]::LoadFrom($exe)
$ui=$assembly.GetType('Cns.Monitor.MonitorWindow',$true)
$binding=[Reflection.BindingFlags]'Instance,Public,NonPublic'
[Windows.Forms.Application]::EnableVisualStyles()
$window=[Activator]::CreateInstance($ui,$true)
$pump=$ui.GetMethod('ProcessInput',$binding)
$list=$ui.GetField('frames',$binding).GetValue($window)
$detail=$ui.GetField('detail',$binding).GetValue($window)
$wrap=$ui.GetField('wrap',$binding).GetValue($window)
$follow=$ui.GetField('follow',$binding).GetValue($window)
try {
    $window.StartPosition='Manual'
    $window.Location=[Drawing.Point]::new(-20000,-20000)
    $window.ShowInTaskbar=$false
    $window.Show()
    $window.Size=$window.MinimumSize
    [byte[]]$sample=[FieldScrollProbe]::Frame()
    $pump.Invoke($window,@($sample,$sample.Length,[long]1000))
    [Windows.Forms.Application]::DoEvents()
    $item=$list.Items[0]
    $summary=$item.SubItems[3].Text
    if($summary.Length -lt 1000 -or !$summary.EndsWith('250, 251, 252')){throw 'Array preview is incomplete'}
    $wire=[BitConverter]::ToString($sample).Replace('-',' ')
    if(!$detail.Text.EndsWith($wire) -or !$detail.Text.Contains('250, 251, 252')) {throw 'Array or HEX is incomplete'}
    $nativeLines={ [FieldScrollProbe]::SendMessage($detail.Handle,0xBA,[IntPtr]::Zero,[IntPtr]::Zero).ToInt32() }
    $wrapped=& $nativeLines
    if(!$wrap.Checked -or !$detail.WordWrap -or $wrapped -le $detail.Lines.Length){throw 'Detail does not visually wrap'}
    $original=$detail.Text
    $wrap.Checked=$false
    [Windows.Forms.Application]::DoEvents()
    if($detail.Text -cne $original -or (& $nativeLines) -ge $wrapped){throw 'Changing wrap altered content or kept it wrapped'}
    $pos=$detail.Text.IndexOf('250, 251, 252')+12
    $detail.Select($pos,0)
    $detail.ScrollToCaret()
    $cursor=$detail.GetPositionFromCharIndex($pos)
    if($cursor.X -lt 0 -or $cursor.X -gt $detail.ClientSize.Width -or [FieldScrollProbe]::GetScrollPos($detail.Handle,0) -le 0){throw 'Cannot scroll to the last array element'}
    $wrap.Checked=$true
    [Windows.Forms.Application]::DoEvents()
    [void][FieldScrollProbe]::SendMessage($detail.Handle,0x115,[IntPtr]7,[IntPtr]::Zero)
    $lastChar=$detail.GetPositionFromCharIndex($detail.TextLength-1)
    if($lastChar.Y -lt 0 -or $lastChar.Y -ge $detail.ClientSize.Height -or $follow.Checked){throw 'Cannot inspect final HEX byte while pinned'}
    $pump.Invoke($window,@($sample,$sample.Length,[long]1300))
    if($list.Items.Count -ne 2 -or $list.SelectedIndices[0] -ne 0 -or $detail.Text -cne $original){throw 'New frame displaced pinned text'}
    [void][FieldScrollProbe]::SendMessage($list.Handle,0x1014,[IntPtr]30000,[IntPtr]::Zero)
    [Windows.Forms.Application]::DoEvents()
    $cell=$item.SubItems[3].Bounds
    if([FieldScrollProbe]::GetScrollPos($list.Handle,0) -le 0 -or $cell.Right -gt ($list.ClientSize.Width+2) -or $cell.Right -le 0){throw 'List does not reach right edge of long summary'}
    if($env:MAVLINK_LONGFIELD_PREVIEW) {
        $bmp=[Drawing.Bitmap]::new($window.ClientSize.Width,$window.ClientSize.Height)
        try {$window.DrawToBitmap($bmp,[Drawing.Rectangle]::new([Drawing.Point]::Empty,$bmp.Size)); $bmp.Save($env:MAVLINK_LONGFIELD_PREVIEW)} finally {$bmp.Dispose()}
    }
    "PASS long fields: $($summary.Length) summary characters, 253 array elements, 267 bytes, $wrapped wrapped lines, list and detail end reachable."
} finally {
    $window.Close()
    $window.Dispose()
}
