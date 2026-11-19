"""One-volt candidate checks; imports existing virtual protocols, never COM."""
import contextlib,io,re,sys,tempfile,types
from pathlib import Path
from unittest.mock import patch
import fast_voltage_proposal as previous
import capture_motion
import fast_motion_profile as fast
legacy=previous.legacy
active='FAST_VOLTAGE_ACTIVE vector_limit_V=1.0000 q_integrator_limit_V=1.0000 d_integrator_limit_V=1.0000 q_slew_limit_V=1.0000 d_slew_limit_V=1.0000'
for scenario in ('normal','missing_voltage','wrong_slew','wrong_declared_voltage','wrong_applied_current','wrong_filter','wrong_command','wrong_speed_guard'):
    prefix=[]
    for s in previous.original:
        if s.startswith('BENCH_MOTION '):
            s=fast.metadata(360,1,'peak_motor_plus_10pct')
            if scenario=='wrong_command':s=s.replace('velocity_command_factor=1.1','velocity_command_factor=1.2')
            if scenario=='wrong_speed_guard':s=re.sub(r'speed_limit_rad_s=\S+','speed_limit_rad_s=999',s)
        if s.startswith('BENCH_PROFILE '):
            s=s.replace('iq_limit_nominal_A=.45','iq_limit_nominal_A=.40')
            if scenario!='wrong_declared_voltage':s=s.replace('vector_limit_V=.5','vector_limit_V=1.0')
        if s.startswith('CURRENT_LOOP '):s=s.replace('vector_voltage_limit_V=.5','vector_voltage_limit_V=1.0').replace('current_LPF_s=0.0010','current_LPF_s=0.0005')
        if s.startswith('CURRENT_ACTIVE ') and scenario!='wrong_filter':s=re.sub(r'q_LPF_s=\S+','q_LPF_s=0.0005',s)
        if s.startswith('RUN BENCH MOTION:'):
            if scenario!='missing_voltage':prefix.append(active.replace('q_slew_limit_V=1.0000','q_slew_limit_V=.7000') if scenario=='wrong_slew' else active)
            prefix.append('FAST_CURRENT_ACTIVE iq_limit_nominal_A='+('.4500' if scenario=='wrong_applied_current' else '.4000'))
        prefix.append(s)
    legacy.prefix=prefix;port=legacy.VirtualPort('normal');append=port.append
    port.append=lambda t:append(t.replace('diag_version=153','diag_version=176'))
    with tempfile.TemporaryDirectory() as temp:
        output=Path(temp)/'capture.log';args=['capture_motion.py','--fast','--rpm','360','--shape','2','--direction','1','--firmware-version','176','--vector-limit','1.0','--iq-limit','.4','--q-lpf','.0005','--velocity-limit','peak_motor_plus_10pct','--output',str(output)]
        error=None
        with patch.object(sys,'argv',args),patch.object(capture_motion.serial,'Serial',return_value=port),patch.object(capture_motion.list_ports,'comports',return_value=[types.SimpleNamespace(device='COM10',vid=0x0483,pid=0x374b)]),patch.object(capture_motion.time,'monotonic',side_effect=lambda:port.clock),contextlib.redirect_stdout(io.StringIO()):
            try:capture_motion.main()
            except (RuntimeError,ValueError) as exc:error=str(exc)
        assert (error is None)==(scenario=='normal'),(scenario,error)
        assert 'HOST final_stop_acknowledged=True' in output.read_text()
    print('PASS one-volt virtual:',scenario)
legacy.prefix=previous.original
try:fast.verify_candidate(360,2,vector_limit=.7,iq_limit=.4,q_lpf=.0005)
except RuntimeError as exc:assert 'v174' in str(exc)
else:raise AssertionError('Failed .7V candidate accepted')
print('PASS: one-volt declared/applied vector, PI, slew, current and filter checked; failed .7V candidate refused; no hardware access')
with patch.object(capture_motion.serial,'Serial',side_effect=AssertionError('must refuse before COM')):
    try:fast.verify_candidate(360,2,vector_limit=1.,iq_limit=.4,q_lpf=.0005)
    except RuntimeError as exc:assert 'v175' in str(exc)
    else:raise AssertionError('Failed peak-only command accepted')
print('PASS: v175 identical condition refused; command headroom allowed only with unchanged independent speed guard')
