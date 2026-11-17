"""Known signed velocities verify target scaling without motor access."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from capture_test import verify_reference_profile,verify_active_filters,candidate_velocity_profile,verify_adaptive_profile,candidate_reference_duration,candidate_current_lpf,ADAPTIVE_COMMANDS,HIGH_RANGE_COMMANDS,verify_rolling_profile

expected={'@':(.12,1.2,.0005,6.283185),'_':(.12,1.2,.0005,6.283185),'{':(.075,1.2,.0005,6.283185),']':(.12,1.2,.0005,6.283185),'[':(.12,1.2,.0005,6.283185),'~':(.12,1.2,.0005,6.283185),'<':(.15,1.2,.0005,6.283185),'>':(.15,1.2,.0005,6.283185),'=':(.075,1.2,.0005,6.283185),'4':(.025,1.2,.0005,6.283185),'3':(.025,.3,.0005,6.283185),'1':(.15,7.2,.001,6.283185),'G':(.15,7.2,.001,6.283185),'0':(.15,7.2,.001,2.5),'9':(.15,7.2,.001,3),'7':(.15,7.2,.001,2.5),'8':(.15,7.2,.001,2.5),'6':(.15,7.2,.001,2),'5':(.15,7.2,.001,2.5),'2':(.15,7.2,.0005,2.5),'P':(.15,7.2,.001,3),'O':(.15,7.2,.001,2.5),'Y':(.15,7.2,.001,2),'W':(.15,7.2,.002,2),
          'J':(.15,4.8,.002,2),'K':(.1,4.8,.002,2),'X':(.1,4.8,.002,3),
          'I':(.15,7.2,.002,1),'R':(.15,7.2,.002,-1),
          'V':(.2,7.2,.002,1),'L':(.15,4.8,.001,1),
          'E':(.1,4.8,.002,1),'B':(.15,9.6,.002,1),'S':(.15,9.6,.002,-1)}
for command,profile in expected.items():assert candidate_velocity_profile(command)==profile
short='ESTIMATOR controller=rolling_LS2ms min_sample_us=75 warmup_span_us=1500 guard=original_window2ms_LPF2ms guard_limit_rad_s=8.5 guard_profile=reference_envelope'
verify_rolling_profile(short,'3')
verify_rolling_profile(short,'4')
verify_rolling_profile(short,'=')
verify_rolling_profile(short,'>')
verify_rolling_profile(short,'<')
verify_rolling_profile(short,'~')
assert candidate_current_lpf('~')==.001 and candidate_reference_duration('~')==2000000
assert candidate_current_lpf('<')==.0005 and candidate_reference_duration('<')==2000000
assert candidate_current_lpf('>')==.001 and candidate_reference_duration('>')==2000000
assert candidate_current_lpf('=')==.001 and candidate_reference_duration('=')==2000000
assert candidate_current_lpf('4')==.001 and candidate_reference_duration('4')==2000000
for wrong in (short.replace('LS2ms','LS4ms'),short.replace('min_sample_us=75','min_sample_us=150'),short.replace('guard_limit_rad_s=8.5','guard_limit_rad_s=9')):
    try:verify_rolling_profile(wrong,'3')
    except RuntimeError:pass
    else:raise AssertionError('Wrong control-window/guard profile accepted')
verify_rolling_profile('ESTIMATOR controller=rolling_LS4ms min_sample_us=150 warmup_span_us=3000 guard=original_window2ms_LPF2ms guard_limit_rad_s=5','q')
assert candidate_reference_duration('G')==candidate_reference_duration('1')==2000000
assert HIGH_RANGE_COMMANDS==frozenset('G134=><~[]{_@') and candidate_current_lpf('1')==.001 and candidate_current_lpf('G')==.002
assert not (HIGH_RANGE_COMMANDS & ADAPTIVE_COMMANDS) and all(command in ADAPTIVE_COMMANDS for command in '7890')
assert candidate_reference_duration('0')==1500000 and candidate_reference_duration('8')==800000
adaptive='ADAPTIVE_CONFIG order=84 learning_rate=.2 correction_bound_nominal_A=.03 response_lag_rad=.785398 gate_us=800000 blend_us=200000 total_limit_nominal_A=.45 per_run_reset=1 learning_reference_rad_s=2.5 learning_min_velocity_rad_s=.75 learning_max_velocity_rad_s=4.25'
verify_adaptive_profile(adaptive,'7')
speed3=adaptive.replace('reference_rad_s=2.5','reference_rad_s=3').replace('min_velocity_rad_s=.75','min_velocity_rad_s=.9').replace('max_velocity_rad_s=4.25','max_velocity_rad_s=5.1')
verify_adaptive_profile(speed3,'9')
try:verify_adaptive_profile(adaptive,'9')
except RuntimeError:pass
else:raise AssertionError('3rad/s adaptive profile accepted a 2.5rad/s learning gate')
verify_adaptive_profile(adaptive.replace('learning_rate=.2','learning_rate=0'),'8')
verify_adaptive_profile(adaptive.replace('learning_rate=.2','learning_rate=0'),'0')
for wrong in (adaptive.replace('bound_nominal_A=.03','bound_nominal_A=.04'),adaptive.replace('reference_rad_s=2.5','reference_rad_s=1'),adaptive.replace('max_velocity_rad_s=4.25','max_velocity_rad_s=5'),adaptive.replace('learning_reference_rad_s','missing_reference_rad_s')):
    try:verify_adaptive_profile(wrong,'7')
    except RuntimeError:pass
    else:raise AssertionError('Incorrect adaptive reference/bound accepted')
legacy=adaptive.split(' learning_reference_rad_s')[0]
verify_adaptive_profile(legacy,'A')
verify_adaptive_profile(legacy.replace('learning_rate=.2','learning_rate=0'),'Z')
verify_adaptive_profile(legacy.replace('response_lag_rad=.785398','response_lag_rad=0'),'F')
verify_adaptive_profile(legacy+' learning_reference_rad_s=1 learning_min_velocity_rad_s=.3 learning_max_velocity_rad_s=1.7','A')
try:verify_adaptive_profile(adaptive,'A')
except RuntimeError:pass
else:raise AssertionError('Legacy 1rad/s profile accepted a 2.5rad/s learning reference')
faster='CURRENT_ACTIVE q_LPF_s=0.0020 d_LPF_s=0.0020 velocity_LPF_s=0.0010 q_P=0.6000 q_I=200.0000 velocity_P=0.1500 velocity_I=7.2000'
verify_active_filters(faster,.002,.001,.6,200,.15,7.2)
half=faster.replace('velocity_LPF_s=0.0010','velocity_LPF_s=0.0005')
verify_active_filters(half,.002,.0005,.6,200,.15,7.2)
try:verify_active_filters(faster,.002,.0005,.6,200,.15,7.2)
except RuntimeError:pass
else:raise AssertionError('1ms actual filter accepted for half-ms candidate')
for wrong in (faster.replace('velocity_LPF_s=0.0010','velocity_LPF_s=0.0020'),faster.replace('velocity_I=7.2000','velocity_I=4.8000')):
    try:verify_active_filters(wrong,.002,.001,.6,200,.15,7.2)
    except RuntimeError:pass
    else:raise AssertionError('Incorrect Y filter/gain accepted')

active='CURRENT_ACTIVE q_LPF_s=0.0020 d_LPF_s=0.0020 velocity_LPF_s=0.0020 q_P=0.6000 q_I=200.0000 velocity_P=0.2000 velocity_I=7.2000'
verify_active_filters(active,.002,.002,.6,200,.2,7.2)
try:verify_active_filters(active.replace('velocity_P=0.2000','velocity_P=0.1500'),.002,.002,.6,200,.2,7.2)
except RuntimeError:pass
else:raise AssertionError('Unexpected actual P accepted')

reference='REFERENCE_RAMP duration_us=800000 target_rad_s=2.000 trace_target=final_goal origin=trace_start'
verify_reference_profile(reference)
verify_reference_profile(reference.replace('2.000','2.500').replace('800000','1500000'),2.5,1500000)
verify_reference_profile(reference.replace('2.000','3.000'),3)
verify_reference_profile(reference.replace('2.000','2.500'),2.5)
try:verify_reference_profile(reference,3)
except RuntimeError:pass
else:raise AssertionError('2rad/s metadata accepted for a requested 3rad/s profile')
lower='CURRENT_ACTIVE q_LPF_s=0.0020 d_LPF_s=0.0020 velocity_LPF_s=0.0020 q_P=0.6000 q_I=200.0000 velocity_P=0.1000 velocity_I=4.8000'
verify_active_filters(lower,.002,.002,.6,200,.1,4.8)
for wrong in (lower.replace('velocity_P=0.1000','velocity_P=0.1500'),lower.replace('velocity_I=4.8000','velocity_I=7.2000')):
    try:verify_active_filters(wrong,.002,.002,.6,200,.1,4.8)
    except RuntimeError:pass
    else:raise AssertionError('Unexpected lower-P profile gains accepted')
for bad in (reference.replace('800000','0'),reference.replace('2.000','3.000'),reference.replace('final_goal','instantaneous')):
    try:verify_reference_profile(bad)
    except RuntimeError:pass
    else:raise AssertionError('Incorrect ramp metadata accepted')
with tempfile.TemporaryDirectory() as directory:
    path=Path(directory)/'known.log'
    for goal in (1,2,2.5,3,6.283185):
        for encoder_sign in (-1,1):
            for command_sign in (-1,1):
                actual=command_sign*(goal+.2)
                lines=['FIRMWARE diag_version=86','DRIVE current_velocity=1',
                       f'RUN_REFERENCE start_angle_rad=0 zero_electric_angle_rad=0 sensor_direction={encoder_sign}',
                       'trace_us,loop_us,A_edges,B_edges,angle_rad,raw_rad_s,filtered_rad_s,window10ms_rad_s,target_rad_s,Uq_V']
                for us in range(10000,3000001,10000):
                    lines.append(f'{us},80,0,0,{encoder_sign*actual*us/1e6},0,{actual},{encoder_sign*actual},{command_sign*goal},.1')
                lines+=['TRACE_END','STOP: 3-second test complete','HOST final_stop_acknowledged=True']
                path.write_text('\n'.join(lines)+'\n',encoding='utf-8')
                result=json.loads(subprocess.check_output([sys.executable,'tools/analyze_trace.py',str(path)],text=True))
                m=result['covered_after_0_8s_metrics']
                assert abs(m['window_tracking_rmse_rad_s']-.2)<1e-10
                assert abs(m['relative_window_tracking_rmse']-.2/goal)<1e-10
                assert abs(m['mean_speed_in_command_direction_rad_s']-(goal+.2))<1e-10
                assert m['target_magnitude_rad_s']==goal
    # The complete analyzer must require snapshots for new current-velocity firmware,
    # while legacy captures above remain readable with unchanged metrics.
    new_lines=[line.replace('diag_version=86','diag_version=97') for line in lines]+['RUN CURRENT VELOCITY: synthetic']
    path.write_text('\n'.join(new_lines)+'\n',encoding='utf-8')
    rejected=subprocess.run([sys.executable,'tools/analyze_trace.py',str(path)],text=True,capture_output=True)
    assert rejected.returncode!=0 and 'guard-peak configuration required' in rejected.stderr
    from guard_peak import CONFIG
    peak=' valid=1 elapsed_us=900000 sample_age_us=10 adc_age_us=80 guard_rad_s=3.2 control_rad_s=3.2 reference_rad_s=3 iq_applied_nominal_A=.2 iq_next_nominal_A=-.1 iq_filtered_nominal_A=.18 Uq_V=.1 Ud_V=.01 correction_nominal_A=0'
    new_lines += [CONFIG,'GUARD_PEAK scope=whole'+peak,'GUARD_PEAK scope=after_ramp'+peak,
                  'RUN_SUMMARY elapsed_us,displacement_rad: 3000000,9.6','STOP_GUARD run_abs_peak_rad_s=3.2']
    path.write_text('\n'.join(new_lines)+'\n',encoding='utf-8')
    captured=json.loads(subprocess.check_output([sys.executable,'tools/analyze_trace.py',str(path)],text=True))
    assert captured['guard_peak_snapshots']['whole']['iq_next_nominal_A']==-.1
    assert captured['covered_after_0_8s_metrics']==m
    slow=['FIRMWARE diag_version=98','DRIVE current_velocity=1','RUN CURRENT VELOCITY: synthetic',
          'RUN_REFERENCE start_angle_rad=0 zero_electric_angle_rad=0 sensor_direction=1',
          'REFERENCE_RAMP duration_us=1500000 target_rad_s=3.000 trace_target=final_goal origin=trace_start',
          CONFIG.replace('800000','1500000'),
          'GUARD_PEAK scope=whole'+peak.replace('900000','1900000'),
          'GUARD_PEAK scope=after_ramp'+peak.replace('900000','1900000'),
          'trace_us,loop_us,A_edges,B_edges,angle_rad,raw_rad_s,filtered_rad_s,window10ms_rad_s,target_rad_s,Uq_V']
    def position(us):
        t=us/1e6
        return t*t if t<=1.5 else 2.25+3.2*(t-1.5)
    for us in range(10000,3000001,10000):
        window=(position(us)-position(us-10000))/.01
        slow.append(f'{us},80,0,0,{position(us)},0,{window},{window},3,.1')
    slow+=['TRACE_END','STOP: 3-second test complete','HOST final_stop_acknowledged=True',
           'RUN_SUMMARY elapsed_us,displacement_rad: 3000000,7.05','STOP_GUARD run_abs_peak_rad_s=3.2']
    path.write_text('\n'.join(slow)+'\n',encoding='utf-8')
    automatic=json.loads(subprocess.check_output([sys.executable,'tools/analyze_trace.py',str(path)],text=True))
    assert automatic['analysis_start_us']==1500000 and automatic['covered_after_startup_metrics']['covered_start_us']==1500000
    common=json.loads(subprocess.check_output([sys.executable,'tools/analyze_trace.py',str(path),'--start-us','1510000'],text=True))
    assert abs(common['covered_after_startup_metrics']['window_tracking_rmse_rad_s']-.2)<1e-10
    rejected=subprocess.run([sys.executable,'tools/analyze_trace.py',str(path),'--start-us','800000'],text=True,capture_output=True)
    assert rejected.returncode!=0 and 'Comparison start must follow' in rejected.stderr
print('PASS: signed target scaling and legacy metrics; wrong gain/ramp/adaptive metadata and missing v97 peaks rejected; valid snapshots retained')
