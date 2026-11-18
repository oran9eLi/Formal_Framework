# Windows PowerShell -STA。用已打包的 EXE 检查无数据时的重绘、突发帧与历史阅读。
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type -ReferencedAssemblies System.Windows.Forms -TypeDefinition @'
using System.Windows.Forms;
public sealed class MonitorPaintProbe : NativeWindow {
    public int Paints, RedrawCommands;
    public MonitorPaintProbe(Control control) { AssignHandle(control.Handle); }
    protected override void WndProc(ref Message m) {
        if (m.Msg == 15) Paints++;
        if (m.Msg == 11) RedrawCommands++;
        base.WndProc(ref m);
    }
}
'@
$root = (Get-Item $PSScriptRoot).Parent.Parent.FullName
$exe = Join-Path $root 'build/MavlinkMonitor/dist/MAVLink中文串口助手.exe'
$assembly = [Reflection.Assembly]::LoadFrom($exe)
$ui = $assembly.GetType('Cns.Monitor.MonitorWindow', $true)
$binding = [Reflection.BindingFlags]'Instance,Public,NonPublic'
[Windows.Forms.Application]::EnableVisualStyles()
$window = [Activator]::CreateInstance($ui, $true)
$pump = $ui.GetMethod('ProcessInput', $binding)
$list = $ui.GetField('frames', $binding).GetValue($window)
$detail = $ui.GetField('detail', $binding).GetValue($window)
$follow = $ui.GetField('follow', $binding).GetValue($window)
$lines = Get-Content -Encoding UTF8 -LiteralPath (Join-Path $root 'Tools/LoRa目录样例帧.hex')
[byte[]]$sample = foreach ($hex in ($lines -join ' ').Trim().Split(' ')) {
    if ($hex) { [Convert]::ToByte($hex, 16) }
}
$window.StartPosition = 'Manual'
$window.Location = [Drawing.Point]::new(-24000, -24000)
$window.ShowInTaskbar = $false
$listProbe = $null
$detailProbe = $null
try {
    $window.Show()
    $pump.Invoke($window, @($sample, $sample.Length, [long]1000))
    [Windows.Forms.Application]::DoEvents()
    if ($list.Items.Count -ne 13) { throw '样例批次未完整呈现' }
    $listProbe = [MonitorPaintProbe]::new($list)
    $detailProbe = [MonitorPaintProbe]::new($detail)
    [long]$clock = 1000
    for ($i=0; $i -lt 100; $i++) {
        $clock += 70
        $pump.Invoke($window, @([byte[]]@(), [int]0, $clock))
        [Windows.Forms.Application]::DoEvents()
    }
    if ($listProbe.Paints -or $detailProbe.Paints -or $listProbe.RedrawCommands) {
        throw '空闲轮询触发了额外绘制'
    }
    $list.Items[0].Selected = $true
    [Windows.Forms.Application]::DoEvents()
    $pinned = $detail.Text
    for ($i=0; $i -lt 100; $i++) {
        $clock += 70
        $pump.Invoke($window, @($sample, $sample.Length, $clock))
    }
    $pump.Invoke($window, @([byte[]]@(), [int]0, [long]($clock+300)))
    if ($list.Items.Count -ne 500 -or $follow.Checked -or $detail.Text -ne $pinned) {
        throw '连续接收时历史帧详情发生跳转'
    }
    $follow.Checked = $true
    if ($list.SelectedIndices[0] -ne 499) { throw '恢复跟随后未显示最新帧' }
    'PASS: 100 次空闲轮询无重绘，连续 1,300 帧保留 500 行、选中详情稳定，可恢复跟随。'
} finally {
    if ($null -ne $listProbe) { $listProbe.ReleaseHandle() }
    if ($null -ne $detailProbe) { $detailProbe.ReleaseHandle() }
    $window.Close()
    $window.Dispose()
}
