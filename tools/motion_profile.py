"""Declared standalone-motor motion profile; no serial or device access."""
import math,re

def fields(line):
    return {k:float(v) for k,v in re.findall(r'([A-Za-z_][A-Za-z_0-9]*)=(-?\d+(?:\.\d*)?|-?\.\d+)',line)}

def parameters(rpm,shape):
    if rpm not in (10,20,40,60) or shape not in (0,1):raise ValueError('Unqualified motion profile')
    move=9375000//rpm if shape==0 else 5000000//rpm+62500
    return move,100000+move+400000,100000+2*move+900000,max(5,rpm*math.pi/30+2)

def reference(t,direction,rpm,shape):
    move,ret,_,_=parameters(rpm,shape);amp=math.radians(3.75)
    if t<100000:return 0,0,0
    backward=t>=ret;local=t-(ret if backward else 100000)
    if local>=move:return (0 if backward else direction*amp),0,(4 if backward else 2)
    if shape==0:
        u=local/move;x=u**3*(10-15*u+6*u*u);v=amp*30*u*u*(1-u)**2*1e6/move
    else:
        time=local*1e-6;total=move*1e-6;vmax=rpm*math.pi/30/8
        if time<.1:v=vmax*time/.1;distance=.5*v*time
        elif time<total-.025:v=vmax;distance=vmax*(time-.05)
        else:left=total-time;v=vmax*left/.025;distance=amp-.5*v*left
        x=distance/amp
    return direction*amp*(1-x if backward else x),direction*v*(-1 if backward else 1),(3 if backward else 1)

def verify_motion(line,rpm,shape,direction,velocity_p=None,velocity_i=None,velocity_limit=None):
    move,_,duration,limit=parameters(rpm,shape)
    actual=fields(line)
    if velocity_p is None:velocity_p=actual.get('velocity_P')
    if velocity_p not in (.05,.12):raise RuntimeError('Unvalidated motion velocity P')
    if velocity_i is None:velocity_i=actual.get('velocity_I')
    if velocity_i not in (1.2,3.0):raise RuntimeError('Unvalidated motion velocity I')
    ptext='.05' if velocity_p==.05 else '.12'
    expected=(f'BENCH_MOTION direction={direction} rpm={rpm} shape={shape} move_us={move} duration_us={duration} speed_limit_rad_s={limit:.6f}'
       ' amplitude_motor_deg=30 initial_us=100000 hold_us=400000 final_us=500000 position_limit_joint_deg=6 error_limit_joint_deg=1 error_delay_us=100000 control_gap_us=5000 position_P=30 velocity_P=.05 velocity_I=3.0 velocity_limit=1.25peak_plus_.5motor hold_enter_deg=.05 hold_exit_deg=.075 reference_update=loop_direct trace=moving96_hold10ms stats=all_FOC_samples')
    expected=expected.replace('velocity_P=.05',f'velocity_P={ptext}')
    expected=expected.replace('velocity_I=3.0',f'velocity_I={velocity_i:.1f}')
    if velocity_limit is None:
        match=re.search(r' velocity_limit=(\S+)',line);velocity_limit=match[1] if match else None
    if velocity_limit not in ('1.25peak_plus_.5motor','peak_motor'):raise RuntimeError('Unvalidated motion velocity clamp')
    expected=expected.replace('velocity_limit=1.25peak_plus_.5motor',f'velocity_limit={velocity_limit}')
    # Float constant rounding on MCU can differ by one least printed digit.
    if abs(actual.get('speed_limit_rad_s',0)-limit)>2e-6:raise RuntimeError('Motion independent speed limit mismatch')
    if re.sub(r'speed_limit_rad_s=\S+','speed_limit_rad_s=CHECKED',line)!=re.sub(r'speed_limit_rad_s=\S+','speed_limit_rad_s=CHECKED',expected):raise RuntimeError('Motion trajectory/controller/guard mismatch')

PROFILE=('BENCH_PROFILE kind=motion amplitude_nominal_A=0.000 iq_limit_nominal_A=.45 vector_limit_V=.5 pulse_duration_us=0 trace_period_us=10000 run_phase_limit_A=.60 align_phase_limit_A=2.2 phase_frames=3 dq_limit_A=.50 dq_delay_us=2000 align_deadline_us=18000000 motor_only=1 ratio_for_analysis=8 startup_clock=first_foc trace_clock=first_foc')
