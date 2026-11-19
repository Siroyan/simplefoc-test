"""Offline candidate, waveform and hostile metadata checks. No device access."""
import math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from fast_motion_profile import parameters,reference,verify_motion,metadata,verify_candidate,STAGES
for rpm in STAGES:
    p=parameters(rpm);assert p['duration_us']<3000000
    for sign in (-1,1):
        verify_motion(metadata(rpm,sign),rpm,2,sign,.12,1.2,'peak_motor')
        q,v,phase=reference(100000+p['move_us'],sign,rpm)
        assert abs(q-sign*math.radians(70))<1e-12 and abs(v*8-sign*p['peak_motor_rad_s'])<1e-10 and phase==3
        q,v,phase=reference(p['duration_us'],sign,rpm)
        assert abs(q-sign*math.radians(p['final_joint_deg']))<1e-12 and v==0 and phase==4
    text=metadata(rpm,1)
    new=metadata(rpm,1,'peak_motor_plus_10pct');verify_motion(new,rpm,2,1,.12,1.2,'peak_motor_plus_10pct')
    assert parameters(rpm)['peak_motor_rad_s']*1.1<parameters(rpm)['speed_limit_rad_s']
    for bad in (new.replace('velocity_command_factor=1.1','velocity_command_factor=1.2'),new.replace('speed_limit_rad_s=','speed_limit_wrong='),new.replace(' velocity_command_factor=1.1','')):
        try:verify_motion(bad,rpm,2,1)
        except ValueError:pass
        else:raise AssertionError('Bad recovery cap/changed independent guard accepted')
    for corrupted in (text.replace('control_gap_us=1000','control_gap_us=5000'),text.replace('amplitude_motor_deg=560','amplitude_motor_deg=5600'),text.replace('trajectory=quartic_accel_smooth_brake','trajectory=old'),text.replace(' error_delay_us=20000','')):
        try:verify_motion(corrupted,rpm,2,1)
        except ValueError:pass
        else:raise AssertionError('Corrupted high-speed bounds accepted')
for args in ((60,2),(120,1),(120,2,.05)):
    try:verify_candidate(*args)
    except ValueError:pass
    else:raise AssertionError('Unknown stage/controller accepted')
print('PASS: all seven fast-stage profiles, takeoff/brake endpoints and rejection of changed/missing bounds or unknown candidates')
