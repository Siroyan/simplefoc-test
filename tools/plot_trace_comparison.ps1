param(
  [Parameter(Mandatory=$true)][string[]]$LogPath,
  [Parameter(Mandatory=$true)][string[]]$Label,
  [Parameter(Mandatory=$true)][string]$OutputPath,
  [string]$PythonPath='python',
  [long]$StartUs=0
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
$title=$chart.Titles.Add('ABI velocity in commanded direction (10 ms window)')
$title.Font=New-Object System.Drawing.Font('Segoe UI',17)
$title.Docking='Top'
$note=$chart.Titles.Add('11.1 V / 0.8 A; dashed black = target, orange = 0.2 rad/s slow threshold; same ABI signal, no independent speed measurement')
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
    $analysisArgs=@((Join-Path $PSScriptRoot 'analyze_trace.py'),$LogPath[$index])
    if ($StartUs -gt 0) {$analysisArgs+=@('--start-us',$StartUs)}
    $analysis=(& $PythonPath @analysisArgs | Out-String) | ConvertFrom-Json
    if ($LASTEXITCODE -ne 0) { throw 'Trace analysis failed' }
    $metrics=$analysis.covered_after_startup_metrics
    $analysisStart=[double]$analysis.analysis_start_us
    $commandSign=$metrics.command_direction
    $settledTarget=$rows | Where-Object {[double]$_.trace_us -ge $analysisStart} | Select-Object -First 1
    $targetMagnitude=[Math]::Abs([double]$settledTarget.target_rad_s)
    if ($targetMagnitude -le 0 -or @($rows | Where-Object {[double]$_.trace_us -ge $analysisStart -and [Math]::Abs([Math]::Abs([double]$_.target_rad_s)-$targetMagnitude) -gt .000001}).Count -gt 0) { throw 'Plot requires a constant nonzero target after the reference ramp' }
    $maximumVelocity=[Math]::Max($maximumVelocity,1.1*$targetMagnitude)
    $name='area'+$index
    $area=New-Object System.Windows.Forms.DataVisualization.Charting.ChartArea($name)
    $area.Position=New-Object System.Windows.Forms.DataVisualization.Charting.ElementPosition(3,(7+$index*(85/$LogPath.Count)),94,(82/$LogPath.Count))
    $area.AxisX.Minimum=$analysisStart/1000000;$area.AxisX.Maximum=3;$area.AxisX.Interval=.2
    $area.AxisX.Title='Time after run start (s)'
    $area.AxisY.Minimum=-.2;$area.AxisY.Maximum=3;$area.AxisY.Interval=.5
    $area.AxisY.Title='Velocity in command direction (rad/s)'
    $area.AxisX.MajorGrid.LineColor=[System.Drawing.Color]::Gainsboro
    $area.AxisY.MajorGrid.LineColor=[System.Drawing.Color]::Gainsboro
    $area.AxisX.LabelStyle.Font=New-Object System.Drawing.Font('Segoe UI',10)
    $area.AxisY.LabelStyle.Font=New-Object System.Drawing.Font('Segoe UI',10)
    $chart.ChartAreas.Add($area)
    $panel=$chart.Titles.Add(('{0}    target {4:F1}, mean {1:F3} rad/s, RMS {2:F3} rad/s, slow {3:F1}%' -f $Label[$index],$metrics.mean_speed_in_command_direction_rad_s,$metrics.window_tracking_rmse_rad_s,(100*$metrics.window_speed_below_0_2_fraction),$targetMagnitude))
    $panel.DockedToChartArea=$name;$panel.IsDockedInsideChartArea=$false
    $panel.Alignment='MiddleLeft';$panel.Font=New-Object System.Drawing.Font('Segoe UI',12)
    $series=New-Object System.Windows.Forms.DataVisualization.Charting.Series('velocity'+$index)
    $series.ChartType='Line';$series.ChartArea=$name;$series.BorderWidth=2
    $series.Color=[System.Drawing.Color]::SteelBlue;$series.IsVisibleInLegend=$false
    foreach ($row in $rows) {
      $time=[double]$row.trace_us/1000000
      if ($time -ge ($analysisStart/1000000)) {
        $velocity=$commandSign*$direction*[double]$row.window10ms_rad_s
        $minimumVelocity=[Math]::Min($minimumVelocity,1.05*$velocity)
        $maximumVelocity=[Math]::Max($maximumVelocity,1.05*$velocity)
        [void]$series.Points.AddXY($time,$velocity)
      }
    }
    $chart.Series.Add($series)
    foreach ($level in $targetMagnitude,.2) {
      $line=New-Object System.Windows.Forms.DataVisualization.Charting.Series(('level'+$index+'_'+$level))
      $line.ChartArea=$name;$line.ChartType='Line';$line.BorderDashStyle='Dash';$line.IsVisibleInLegend=$false
      $line.Color=if ($level -eq $targetMagnitude) {[System.Drawing.Color]::Black} else {[System.Drawing.Color]::DarkOrange}
      [void]$line.Points.AddXY(($analysisStart/1000000),$level);[void]$line.Points.AddXY(3,$level)
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
