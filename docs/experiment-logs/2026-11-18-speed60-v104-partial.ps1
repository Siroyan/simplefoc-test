$ErrorActionPreference='Stop'
Add-Type -AssemblyName System.Windows.Forms.DataVisualization
$lines=Get-Content docs/experiment-logs/2026-11-18-current-velocity-speed60-P150-I120-v104-01.log
$start=0..($lines.Count-1) | Where-Object {$lines[$_].StartsWith('trace_us,')} | Select-Object -First 1
$end=($start+1)..($lines.Count-1) | Where-Object {$lines[$_] -eq 'TRACE_END'} | Select-Object -First 1
$rows=@($lines[$start..($end-1)] | ConvertFrom-Csv | Where-Object {[double]$_.trace_us -ge 2000000})
$chart=New-Object System.Windows.Forms.DataVisualization.Charting.Chart
$chart.Width=1400;$chart.Height=850;$chart.BackColor='White';$chart.AntiAliasing='All'
$title=$chart.Titles.Add('60 rpm attempt, v104: partial post-ramp record (protection stop at 2.300 s)')
$title.Font=New-Object System.Drawing.Font('Segoe UI',16)
$note=$chart.Titles.Add('Same ABI for FOC and speed; interval differentiation includes quantization. This 0.30 s record does not prove 3 s operation.')
$note.Docking='Bottom';$note.Font=New-Object System.Drawing.Font('Segoe UI',10)
try {
 foreach($n in 0,1) {
  $area=New-Object System.Windows.Forms.DataVisualization.Charting.ChartArea(('area'+$n))
  $area.Position=New-Object System.Windows.Forms.DataVisualization.Charting.ElementPosition(3,(9+42*$n),94,39)
  $area.AxisX.Minimum=2;$area.AxisX.Maximum=2.3;$area.AxisX.Interval=.05;$area.AxisX.Title='Time after run start (s)'
  $area.AxisY.Title=if($n -eq 0){'Velocity (rad/s)'}else{'Nominal current (A)'}
  $area.AxisY.Minimum=if($n -eq 0){3}else{-.05};$area.AxisY.Maximum=if($n -eq 0){9}else{.5}
  $area.AxisY.Interval=if($n -eq 0){1}else{.1}
  $area.AxisX.MajorGrid.LineColor='Gainsboro';$area.AxisY.MajorGrid.LineColor='Gainsboro'
  $chart.ChartAreas.Add($area)
  $caption=$chart.Titles.Add($(if($n -eq 0){'Blue: ABI angle differences (~2 ms); dashed black: 6.283185 rad/s target'}else{'Orange: applied Iq target; green: filtered Iq (nominal conversion)'}))
  $caption.DockedToChartArea=$area.Name;$caption.IsDockedInsideChartArea=$false;$caption.Font=New-Object System.Drawing.Font('Segoe UI',11)
 }
 foreach($name in 'interval','target','iq_target','iq_filtered') {
  $series=New-Object System.Windows.Forms.DataVisualization.Charting.Series($name)
  $series.ChartType='Line';$series.BorderWidth=2;$series.IsVisibleInLegend=$false
  $series.ChartArea=if($name -in 'interval','target'){'area0'}else{'area1'}
  $series.Color=switch($name){'interval'{'SteelBlue'} 'target'{'Black'} 'iq_target'{'DarkOrange'} 'iq_filtered'{'SeaGreen'}}
  if($name -eq 'target'){$series.BorderDashStyle='Dash'}
  $chart.Series.Add($series)
 }
 for($i=0;$i -lt $rows.Count;$i++) {
  $time=[double]$rows[$i].trace_us/1e6
  [void]$chart.Series['target'].Points.AddXY($time,6.283185)
  [void]$chart.Series['iq_target'].Points.AddXY($time,[double]$rows[$i].iq_target_nominal_A)
  [void]$chart.Series['iq_filtered'].Points.AddXY($time,[double]$rows[$i].iq_filtered_nominal_A)
  if($i -gt 0){
   $previous=[double]$rows[$i-1].trace_us/1e6
   $velocity=([double]$rows[$i].angle_rad-[double]$rows[$i-1].angle_rad)/($time-$previous)
   [void]$chart.Series['interval'].Points.AddXY(($time+$previous)/2,$velocity)
  }
 }
 $chart.SaveImage([IO.Path]::GetFullPath('docs/experiment-logs/2026-11-18-speed60-v104-partial.png'),'Png')
}finally{$chart.Dispose()}
