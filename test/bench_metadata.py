from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from bench_metadata import PROFILE_SUFFIX,verify_bench_profile,verify_bench_current_loop,verify_bench_joint
for command,kind,amplitude in (('/','current_pulse','0.050'),('&','current_pulse','0.100'),('*','alignment','0.000')):
    line=f'BENCH_PROFILE kind={kind} amplitude_nominal_A={amplitude}'+PROFILE_SUFFIX
    verify_bench_profile(line,command)
    for bad in (line.replace('vector_limit_V=.5','vector_limit_V=.6'),line.replace('phase_frames=3','phase_frames=4'),line.replace('trace_period_us=1000','trace_period_us=2000')):
        try:verify_bench_profile(bad,command)
        except RuntimeError:pass
        else:raise AssertionError('Incorrect bench bound or trace cadence accepted')
try:verify_bench_current_loop('CURRENT_LOOP q_I=200')
except RuntimeError:pass
else:raise AssertionError('Wrong actual current loop accepted')
print('PASS: exact standalone-motor profiles and current-loop/voltage/filter/current/ADC protection metadata')

for command,direction in ((",",1),(".",-1)):
    suffix=PROFILE_SUFFIX.replace("iq_limit_nominal_A=.15","iq_limit_nominal_A=.30").replace("pulse_duration_us=300000 trace_period_us=1000","pulse_duration_us=0 trace_period_us=10000")
    verify_bench_profile("BENCH_PROFILE kind=position amplitude_nominal_A=0.000"+suffix,command)
    line=f"BENCH_JOINT direction={direction} amplitude_deg=.5 ramp_us=100000 duration_us=2700000 trace_period_us=10000 position_P=30 velocity_P=.05 velocity_I=3.0 velocity_feedforward=1 velocity_command_limit_deg_s=5 position_limit_deg=3 guard_window_us=20000 guard_limit_deg_s=15 error_limit_deg=1 error_delay_us=100000 control_gap_us=5000 nominal_iq_limit_A=.30 trace_layout=joint_ab_union integral_policy=clear_opposing_position hold_policy=velocity_damping enter_deg=.05 exit_deg=.075"
    verify_bench_joint(line,command)
    try:verify_bench_joint(line.replace("position_limit_deg=3","position_limit_deg=4"),command)
    except RuntimeError:pass
    else:raise AssertionError("Incorrect joint limit accepted")
print("PASS: actual joint direction, position/speed/error/gap guards and memory-layout metadata")
from bench_metadata import verify_bench_brake
for command,active in (('#',0),('-',1)):
    suffix=PROFILE_SUFFIX.replace('iq_limit_nominal_A=.15','iq_limit_nominal_A=.22').replace('pulse_duration_us=300000 trace_period_us=1000','pulse_duration_us=0 trace_period_us=2000')
    verify_bench_profile('BENCH_PROFILE kind=brake amplitude_nominal_A=0.000'+suffix,command)
    line=f'BENCH_BRAKE active={active} acceleration_iq_A=.22 acceleration_ramp_us=10000 zero_us=40000 trigger_velocity_rad_s=2 trigger_guard_rad_s=2 trigger_earliest_us=50000 acceleration_deadline_us=400000 brake_iq_A=-.20 brake_release_rad_s=.1 brake_duration_us=150000 trace_period_us=2000 position_limit_motor_rad=.418879 control_gap_us=5000 trace_layout=brake_ab_union transition_trace=forced transition_voltage=after_disable reference_update=loop_direct'
    verify_bench_brake(line,command)
    try:verify_bench_brake(line.replace('trigger_guard_rad_s=2','trigger_guard_rad_s=1'),command)
    except RuntimeError:pass
    else:raise AssertionError('Brake without matching independent speed prerequisite accepted')
print('PASS: coast/active braking mode, signed bounds, trigger, deadlines and trace layout')
