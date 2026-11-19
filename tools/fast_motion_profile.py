"""Explicit free-motor high-speed trajectory, separate from old small-angle trials."""
import math,re
from motion_profile import fields,PROFILE
STAGES=(120,240,360,600,1200,2400,3733)

def parameters(rpm,shape=2):
    if rpm not in STAGES or shape!=2:raise ValueError('Unqualified high-speed stage/trajectory')
    move=50000 if rpm==3733 else (186666667+rpm//2)//rpm
    amplitude=math.radians(70);peak=2*amplitude*8*1e6/move;final=amplitude+peak/8*.125
    return {'move_us':move,'initial_us':100000,'brake_us':250000,'final_us':400000,
        'duration_us':100000+move+250000+400000,'peak_motor_rad_s':peak,'speed_limit_rad_s':1.2*peak+2,
        'final_joint_deg':math.degrees(final),'position_limit_joint_deg':math.degrees(final)+5}

def reference(t,direction,rpm,shape=2):
    if direction not in (-1,1):raise ValueError('Unqualified direction')
    p=parameters(rpm,shape);amplitude=math.radians(70);ret=100000+p['move_us']
    if t<100000:return 0,0,0
    if t<ret:
        u=(t-100000)/p['move_us']
        return direction*amplitude*(2*u**3-u**4),direction*amplitude*(6*u*u-4*u**3)*1e6/p['move_us'],1
    if t<ret+250000:
        u=(t-ret)/250000;v=p['peak_motor_rad_s']/8
        return direction*(amplitude+v*.25*(u-u**3+.5*u**4)),direction*v*(1-3*u*u+2*u**3),3
    return direction*math.radians(p['final_joint_deg']),0,4

def electrical_profile(vector_limit=.5,iq_limit=.45):
    if vector_limit not in (.5,.7,1.) or iq_limit not in (.4,.45):raise ValueError('Unreviewed high-speed voltage/current command ceiling')
    text=PROFILE.replace('vector_limit_V=.5','vector_limit_V='+{.5:'.5',.7:'.7',1.:'1.0'}[vector_limit])
    return text.replace('iq_limit_nominal_A=.45','iq_limit_nominal_A=.40') if iq_limit==.4 else text

def verify_candidate(rpm,shape,velocity_p=.12,velocity_i=1.2,velocity_lpf=.001,estimator='edge',velocity_limit='peak_motor',vector_limit=.5,iq_limit=.45,q_lpf=.001):
    parameters(rpm,shape)
    electrical_profile(vector_limit,iq_limit)
    if (velocity_p,velocity_i,velocity_lpf,estimator)!=(.12,1.2,.001,'edge') or velocity_limit not in ('peak_motor','peak_motor_plus_10pct'):
        raise ValueError('Unqualified high-speed controller configuration')
    if q_lpf not in (.001,.0005):raise ValueError('Unreviewed q current filter')
    if rpm==240 and q_lpf==.001 and (iq_limit==.45 or iq_limit==.4 and vector_limit==.7):raise RuntimeError('Known protective-stop v170/v171/v172 240rpm/1ms-current-filter condition: identical retry refused before serial access')
    if rpm==360 and vector_limit==.7 and iq_limit==.4 and q_lpf==.0005:raise RuntimeError('Known protective-stop v174 360rpm/.7V condition: identical retry refused before serial access')
    if rpm==360 and vector_limit==1. and iq_limit==.4 and q_lpf==.0005 and velocity_limit=='peak_motor':raise RuntimeError('Known protective-stop v175 peak-only command condition: identical retry refused before serial access')

def verify_motion(line,rpm,shape,direction,velocity_p=None,velocity_i=None,velocity_limit=None):
    actual=fields(line);p=parameters(rpm,shape)
    expected=dict(p,direction=direction,rpm=rpm,shape=2,amplitude_motor_deg=560,absolute_error_limit_joint_deg=3,
        error_limit_joint_deg=1,error_delay_us=20000,control_period_us=250,control_gap_us=1000,
        position_P=30,velocity_P=.12,velocity_I=1.2,hold_enter_deg=.05,hold_exit_deg=.075)
    mode=re.search(r'\bvelocity_limit=(\S+)',line)
    if not mode or mode[1] not in ('peak_motor','peak_motor_plus_10pct'):raise ValueError('Unknown high-speed velocity command limit')
    mode=mode[1]
    if mode=='peak_motor_plus_10pct':expected.update(velocity_command_factor=1.1,velocity_command_limit_rad_s=p['peak_motor_rad_s']*1.1)
    if set(actual)!=set(expected):raise ValueError('Missing/extra high-speed profile fields')
    for name,value in expected.items():
        tolerance=.0002 if name in ('peak_motor_rad_s','speed_limit_rad_s','final_joint_deg','position_limit_joint_deg','velocity_command_limit_rad_s') else 1e-7
        if abs(actual[name]-value)>tolerance:raise ValueError('High-speed profile mismatch: '+name)
    for token in ('velocity_limit='+mode,'reference_update=loop_direct','trajectory=quartic_accel_smooth_brake','trace=moving160_80_hold10ms','stats=all_FOC_samples'):
        if token not in line.split():raise ValueError('Missing high-speed mode token: '+token)
    if velocity_p is not None and velocity_p!=.12 or velocity_i is not None and velocity_i!=1.2 or velocity_limit is not None and velocity_limit!=mode:
        raise ValueError('Unexpected requested high-speed gains/velocity cap')

def metadata(rpm,direction,velocity_limit='peak_motor'):
    p=parameters(rpm);head=f'BENCH_MOTION direction={direction} rpm={rpm} shape=2 move_us={p["move_us"]} duration_us={p["duration_us"]}'
    for name in ('speed_limit_rad_s','peak_motor_rad_s','final_joint_deg','position_limit_joint_deg'):head+=f' {name}={p[name]:.6f}'
    if velocity_limit=='peak_motor_plus_10pct':head+=f' velocity_command_factor=1.1 velocity_command_limit_rad_s={1.1*p["peak_motor_rad_s"]:.6f}'
    elif velocity_limit!='peak_motor':raise ValueError('Unknown velocity command limit')
    return head+' amplitude_motor_deg=560 initial_us=100000 brake_us=250000 final_us=400000 absolute_error_limit_joint_deg=3 error_limit_joint_deg=1 error_delay_us=20000 control_period_us=250 control_gap_us=1000 position_P=30 velocity_P=.12 velocity_I=1.2 velocity_limit='+velocity_limit+' hold_enter_deg=.05 hold_exit_deg=.075 reference_update=loop_direct trajectory=quartic_accel_smooth_brake trace=moving160_80_hold10ms stats=all_FOC_samples'
