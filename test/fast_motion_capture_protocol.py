"""Run old and fast capture protocols entirely with virtual COM/time."""
import contextlib,io,sys,tempfile,types
from pathlib import Path
from unittest.mock import patch
import motion_capture_protocol as legacy
import capture_motion
import fast_motion_profile as fast

original=list(legacy.prefix)
base=[fast.metadata(120,1) if s.startswith('BENCH_MOTION ') else s for s in original]
for scenario in ('normal','live','bad_profile','bad_bounds','bad_firmware'):
    legacy.prefix=[s.replace('control_gap_us=1000','control_gap_us=5000') if scenario=='bad_bounds' else s for s in base]
    port=legacy.VirtualPort('bad_metadata' if scenario=='bad_profile' else 'live' if scenario=='live' else 'normal')
    append=port.append
    port.append=lambda text:append(text.replace('diag_version=153','diag_version=168' if scenario=='bad_firmware' else 'diag_version=169'))
    def terminal(reason):
        elapsed=1500000 if scenario=='live' else fast.parameters(120)['duration_us'];speed=12.5 if scenario=='live' else 0
        port.append(f'{reason}\nRUN_SUMMARY elapsed_us,displacement_rad: {elapsed},.1\nSTOP_DETAIL elapsed_us,filtered_rad_s,raw_rad_s,window10ms_rad_s: {elapsed},{speed},0,0\nTRACE_END\n'+legacy.idle)
    port.terminal=terminal
    with tempfile.TemporaryDirectory() as temporary:
        output=Path(temporary)/'capture.log';args=['capture_motion.py','--fast','--rpm','120','--shape','2','--direction','1','--firmware-version','169','--output',str(output)]
        if scenario=='live':args+=['--stop-after-ms','1400']
        error=None
        with patch.object(sys,'argv',args),patch.object(capture_motion.serial,'Serial',return_value=port),patch.object(capture_motion.list_ports,'comports',return_value=[types.SimpleNamespace(device='COM10',vid=0x0483,pid=0x374b)]),patch.object(capture_motion.time,'monotonic',side_effect=lambda:port.clock),contextlib.redirect_stdout(io.StringIO()):
            try:capture_motion.main()
            except (ValueError,RuntimeError) as exc:error=str(exc)
        text=output.read_text()
        if scenario in ('normal','live'):assert error is None,(scenario,error)
        else:assert error,(scenario,'invalid profile/firmware accepted')
        if scenario=='bad_firmware':assert port.drives==0 and 'HOST sending one command=' not in text
        assert 'HOST final_stop_acknowledged=True' in text and port.sent[-1]==b's'
    print('PASS fast virtual serial:',scenario)
legacy.prefix=original
for extra in (['--fast','--rpm','60','--shape','2'],['--rpm','120','--shape','0']):
    args=['capture_motion.py',*extra,'--direction','1','--output','must-not-create.log']
    with patch.object(sys,'argv',args),patch.object(capture_motion.list_ports,'comports') as ports,patch.object(capture_motion.serial,'Serial') as serial:
        try:capture_motion.main()
        except ValueError:pass
        else:raise AssertionError('Unknown trajectory accepted')
        ports.assert_not_called();serial.assert_not_called()
print('PASS: fast firmware/profile/bounds and trajectory family checked, stop acknowledgement retained, unknown stages rejected before serial; no real device access')
