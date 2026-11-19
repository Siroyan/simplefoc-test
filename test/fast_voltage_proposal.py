"""Virtual .7V protocol and old failed .5V refusal; no real board access."""
import contextlib,io,re,sys,tempfile,types
from pathlib import Path
from unittest.mock import patch
import motion_capture_protocol as legacy
import capture_motion
import fast_motion_profile as fast
original=list(legacy.prefix)
active='FAST_VOLTAGE_ACTIVE vector_limit_V=.7000 q_integrator_limit_V=.7000 d_integrator_limit_V=.7000 q_slew_limit_V=.7000 d_slew_limit_V=.7000'
current='FAST_CURRENT_ACTIVE iq_limit_nominal_A=.4000'
for scenario in ('normal','missing_active','wrong_integrator','wrong_slew','wrong_profile','wrong_loop','missing_current','wrong_current','wrong_filter'):
    prefix=[]
    for s in original:
        if s.startswith('BENCH_MOTION '):s=fast.metadata(240,1)
        if s.startswith('BENCH_PROFILE '):s=s.replace('iq_limit_nominal_A=.45','iq_limit_nominal_A=.40')
        if s.startswith('BENCH_PROFILE ') and scenario!='wrong_profile':s=s.replace('vector_limit_V=.5','vector_limit_V=.7')
        if s.startswith('CURRENT_LOOP ') and scenario!='wrong_loop':s=s.replace('vector_voltage_limit_V=.5','vector_voltage_limit_V=.7')
        if s.startswith('CURRENT_LOOP '):s=s.replace('current_LPF_s=0.0010','current_LPF_s=0.0005')
        if s.startswith('CURRENT_ACTIVE ') and scenario!='wrong_filter':s=re.sub(r'q_LPF_s=\S+','q_LPF_s=0.0005',s)
        if s.startswith('RUN BENCH MOTION:') and scenario!='missing_active':prefix.append(active.replace('q_integrator_limit_V=.7000','q_integrator_limit_V=.5000') if scenario=='wrong_integrator' else active.replace('d_slew_limit_V=.7000','d_slew_limit_V=.5000') if scenario=='wrong_slew' else active)
        if s.startswith('RUN BENCH MOTION:') and scenario!='missing_current':prefix.append(current.replace('.4000','.4500') if scenario=='wrong_current' else current)
        prefix.append(s)
    legacy.prefix=prefix;port=legacy.VirtualPort('normal');append=port.append
    port.append=lambda t:append(t.replace('diag_version=153','diag_version=173'))
    with tempfile.TemporaryDirectory() as temp:
        output=Path(temp)/'capture.log';args=['capture_motion.py','--fast','--rpm','240','--shape','2','--direction','1','--firmware-version','173','--vector-limit','.7','--iq-limit','.4','--q-lpf','.0005','--output',str(output)]
        error=None
        with patch.object(sys,'argv',args),patch.object(capture_motion.serial,'Serial',return_value=port),patch.object(capture_motion.list_ports,'comports',return_value=[types.SimpleNamespace(device='COM10',vid=0x0483,pid=0x374b)]),patch.object(capture_motion.time,'monotonic',side_effect=lambda:port.clock),contextlib.redirect_stdout(io.StringIO()):
            try:capture_motion.main()
            except (ValueError,RuntimeError) as exc:error=str(exc)
        assert (error is None)==(scenario=='normal'),(scenario,error)
        assert 'HOST final_stop_acknowledged=True' in output.read_text()
    print('PASS voltage proposal virtual serial:',scenario)
legacy.prefix=original
for direction in (-1,1):
    for voltage in ('.5','.7'):
        args=['capture_motion.py','--fast','--rpm','240','--shape','2','--direction',str(direction),'--vector-limit',voltage,'--output','must-not-create.log']
        with patch.object(sys,'argv',args),patch.object(capture_motion.serial,'Serial') as serial,patch.object(capture_motion.list_ports,'comports') as ports:
            try:capture_motion.main()
            except RuntimeError as exc:assert 'Known protective-stop v170' in str(exc)
            else:raise AssertionError('Failed 240rpm/Iq.45 condition accepted')
            ports.assert_not_called();serial.assert_not_called()
print('PASS: declared/applied .7V voltage and .4A command ceilings required; both failed Iq.45 conditions refused before serial; all virtual')
args=['capture_motion.py','--fast','--rpm','240','--shape','2','--direction','1','--vector-limit','.7','--iq-limit','.4','--output','must-not-create.log']
with patch.object(sys,'argv',args),patch.object(capture_motion.list_ports,'comports') as ports:
    try:capture_motion.main()
    except RuntimeError:pass
    else:raise AssertionError('Failed v172 current-filter condition accepted')
    ports.assert_not_called()
