param(
  [Parameter(Mandatory=$true)][string[]]$LogPath,
  [Parameter(Mandatory=$true)][string[]]$Label,
  [Parameter(Mandatory=$true)][string]$OutputPath,
  [string]$PythonPath='python'
)
$ErrorActionPreference='Stop'
if ($LogPath.Count -ne $Label.Count -or $LogPath.Count -lt 1 -or $LogPath.Count -gt 4) {
  throw 'Provide one label for each of one to four complete logs'
}
Add-Type -AssemblyName System.Windows.Forms.DataVisualization
$chart=New-Object System.Windows.Forms.DataVisualization.Charting.Chart
$chart.Width=1500
$chart.Height=180+300*$LogPath.Count
$chart.BackColor=[System.Drawing.Color]::White
$chart.AntiAliasing='All'
$chart.TextAntiAliasingQuality='High'
$title=$chart.Titles.Add('ABI velocity in commanded direction, |target| = 1 rad/s (10 ms window)')
$title.Font=New-Object System.Drawing.Font('Segoe UI',17)
$title.Docking='Top'
$note=$chart.Titles.Add('11.1 V / 0.8 A; dashed black = 1 rad/s target, orange = 0.2 rad/s slow threshold; same ABI signal, no independent speed measurement')
$note.Font=New-Object System.Drawing.Font('Segoe UI',10)
$note.Docking='Bottom'
try {
  $minimumVelocity=-.2;$maximumVelocity=3.0
  for ($index=0;$index -lt $LogPath.Count;$index++) {
    $lines=Get-Content -LiteralPath $LogPath[$index] -Encoding utf8
    if (-not ($lines -contains 'HOST final_stop_acknowledged=True')) { throw 'Incomplete stop acknowledgement' }
    $start=0..($lines.Count-1) | Where-Object {$lines[$_].StartsWith('trace_us,')} | Select-Object -First 1
    $end=($start+1)..($lines.Count-1) | Where-Object {$lines[$_] -eq 'TRACE_END'} | Select-Object -First 1
    if ($null -eq $start -or $null -eq $end) { throw 'Complete trace required' }
    $rows=$lines[$start..($end-1)] | ConvertFrom-Csv
    $reference=$lines | Where-Object {$_.StartsWith('RUN_REFERENCE ')} | Select-Object -First 1
    if ($reference -notmatch 'sensor_direction=(-?[0-9.]+)') { throw 'Aligned sensor direction required' }
    $direction=[double]$Matches[1]
    $analysis=(& $PythonPath (Join-Path $PSScriptRoot 'analyze_trace.py') $LogPath[$index] | Out-String) | ConvertFrom-Json
    if ($LASTEXITCODE -ne 0) { throw 'Trace analysis failed' }
    $metrics=$analysis.covered_after_0_8s_metrics
    $commandSign=$metrics.command_direction
    $settledTarget=$rows | Where-Object {[double]$_.trace_us -ge 800000} | Select-Object -First 1
    if ([Math]::Abs([Math]::Abs([double]$settledTarget.target_rad_s)-1) -gt .000001) { throw 'Plot requires a unit speed command' }
    $name='area'+$index
    $area=New-Object System.Windows.Forms.DataVisualization.Charting.ChartArea($name)
    $area.Position=New-Object System.Windows.Forms.DataVisualization.Charting.ElementPosition(3,(7+$index*(85/$LogPath.Count)),94,(82/$LogPath.Count))
    $area.AxisX.Minimum=.8;$area.AxisX.Maximum=3;$area.AxisX.Interval=.2
    $area.AxisX.Title='Time after run start (s)'
    $area.AxisY.Minimum=-.2;$area.AxisY.Maximum=3;$area.AxisY.Interval=.5
    $area.AxisY.Title='Velocity in command direction (rad/s)'
    $area.AxisX.MajorGrid.LineColor=[System.Drawing.Color]::Gainsboro
    $area.AxisY.MajorGrid.LineColor=[System.Drawing.Color]::Gainsboro
    $area.AxisX.LabelStyle.Font=New-Object System.Drawing.Font('Segoe UI',10)
    $area.AxisY.LabelStyle.Font=New-Object System.Drawing.Font('Segoe UI',10)
    $chart.ChartAreas.Add($area)
    $panel=$chart.Titles.Add(('{0}    mean {1:F3} rad/s, RMS error {2:F3} rad/s, slow {3:F1}%' -f $Label[$index],$metrics.mean_speed_in_command_direction_rad_s,$metrics.window_tracking_rmse_rad_s,(100*$metrics.window_speed_below_0_2_fraction)))
    $panel.DockedToChartArea=$name;$panel.IsDockedInsideChartArea=$false
    $panel.Alignment='MiddleLeft';$panel.Font=New-Object System.Drawing.Font('Segoe UI',12)
    $series=New-Object System.Windows.Forms.DataVisualization.Charting.Series('velocity'+$index)
    $series.ChartType='Line';$series.ChartArea=$name;$series.BorderWidth=2
    $series.Color=[System.Drawing.Color]::SteelBlue;$series.IsVisibleInLegend=$false
    foreach ($row in $rows) {
      $time=[double]$row.trace_us/1000000
      if ($time -ge .8) {
        $velocity=$commandSign*$direction*[double]$row.window10ms_rad_s
        $minimumVelocity=[Math]::Min($minimumVelocity,1.05*$velocity)
        $maximumVelocity=[Math]::Max($maximumVelocity,1.05*$velocity)
        [void]$series.Points.AddXY($time,$velocity)
      }
    }
    $chart.Series.Add($series)
    foreach ($level in 1.0,.2) {
      $line=New-Object System.Windows.Forms.DataVisualization.Charting.Series(('level'+$index+'_'+$level))
      $line.ChartArea=$name;$line.ChartType='Line';$line.BorderDashStyle='Dash';$line.IsVisibleInLegend=$false
      $line.Color=if ($level -eq 1) {[System.Drawing.Color]::Black} else {[System.Drawing.Color]::DarkOrange}
      [void]$line.Points.AddXY(.8,$level);[void]$line.Points.AddXY(3,$level)
      $chart.Series.Add($line)
    }
  }
  foreach ($area in $chart.ChartAreas) {
    $area.AxisY.Minimum=[Math]::Floor(2*$minimumVelocity)/2
    $area.AxisY.Maximum=[Math]::Ceiling(2*$maximumVelocity)/2
  }
  $chart.SaveImage([System.IO.Path]::GetFullPath($OutputPath),[System.Windows.Forms.DataVisualization.Charting.ChartImageFormat]::Png)
  Write-Output ('Saved '+$OutputPath)
} finally { $chart.Dispose() }
