"""Strict metadata for the standalone-motor bench; no hardware access."""
PROFILE_SUFFIX=' iq_limit_nominal_A=.15 vector_limit_V=.5 pulse_duration_us=300000 trace_period_us=1000 run_phase_limit_A=.60 align_phase_limit_A=2.2 phase_frames=3 dq_limit_A=.50 dq_delay_us=2000 align_deadline_us=18000000 motor_only=1 ratio_for_analysis=8 startup_clock=first_foc trace_clock=first_foc'

def verify_bench_profile(line,command):
    if command not in ('/','&','*',',','.','#','-'):raise ValueError('Unknown bench command')
    position=command in (',','.')
    brake=command in ('#','-')
    kind='brake' if brake else 'position' if position else 'alignment' if command=='*' else 'current_pulse'
    amplitude='0.000' if command=='*' or position or brake else '0.100' if command=='&' else '0.050'
    suffix=PROFILE_SUFFIX.replace('iq_limit_nominal_A=.15','iq_limit_nominal_A=.30').replace('pulse_duration_us=300000 trace_period_us=1000','pulse_duration_us=0 trace_period_us=10000') if position else PROFILE_SUFFIX
    if brake:suffix=PROFILE_SUFFIX.replace('iq_limit_nominal_A=.15','iq_limit_nominal_A=.22').replace('pulse_duration_us=300000 trace_period_us=1000','pulse_duration_us=0 trace_period_us=2000')
    if line!=f'BENCH_PROFILE kind={kind} amplitude_nominal_A={amplitude}'+suffix:
        raise RuntimeError('Bench limits/profile mismatch; stopping')

def verify_bench_joint(line,command):
    direction=-1 if command=='.' else 1
    expected=f'BENCH_JOINT direction={direction} amplitude_deg=.5 ramp_us=100000 duration_us=2700000 trace_period_us=10000 position_P=30 velocity_P=.05 velocity_I=3.0 velocity_feedforward=1 velocity_command_limit_deg_s=5 position_limit_deg=3 guard_window_us=20000 guard_limit_deg_s=15 error_limit_deg=1 error_delay_us=100000 control_gap_us=5000 nominal_iq_limit_A=.30 trace_layout=joint_ab_union integral_policy=clear_opposing_position hold_policy=velocity_damping enter_deg=.05 exit_deg=.075'
    if line!=expected:raise RuntimeError('Bench joint settings mismatch; stopping')

def verify_bench_current_loop(line,vector_limit=.5,q_lpf=.001):
    if vector_limit not in (.5,.7,1.):raise ValueError('Unreviewed voltage ceiling')
    if q_lpf not in (.001,.0005):raise ValueError('Unreviewed q current filter')
    expected='CURRENT_LOOP q_P=0.60 q_I=400.0 d_P=.20 d_I=50 D=0 q_voltage_build_up_V_s=100.0 current_LPF_s=0.0010 vector_voltage_limit_V=.5 release=immediate reversal=via_zero; nominal current scaling'
    if vector_limit==.7:expected=expected.replace('vector_voltage_limit_V=.5','vector_voltage_limit_V=.7')
    if vector_limit==1.:expected=expected.replace('vector_voltage_limit_V=.5','vector_voltage_limit_V=1.0')
    if q_lpf==.0005:expected=expected.replace('current_LPF_s=0.0010','current_LPF_s=0.0005')
    if line!=expected:raise RuntimeError('Bench current gains/filter/actuator mismatch; stopping')

def verify_bench_brake(line,command):
    active=1 if command=='-' else 0
    expected=f'BENCH_BRAKE active={active} acceleration_iq_A=.22 acceleration_ramp_us=10000 zero_us=40000 trigger_velocity_rad_s=2 trigger_guard_rad_s=2 trigger_earliest_us=50000 acceleration_deadline_us=400000 brake_iq_A=-.20 brake_release_rad_s=.1 brake_duration_us=150000 trace_period_us=2000 position_limit_motor_rad=.418879 control_gap_us=5000 trace_layout=brake_ab_union transition_trace=forced transition_voltage=after_disable reference_update=loop_direct'
    if line!=expected:raise RuntimeError('Bench braking bounds/mode mismatch; stopping')
