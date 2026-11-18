"""Virtual serial/clock only. Never opens a real COM port or resets a board."""
import argparse,contextlib,io,re,sys,tempfile,types
from pathlib import Path
from unittest.mock import patch
root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--tools-dir',type=Path,default=root/'tools');configuration=parser.parse_args()
sys.path.insert(0,str(configuration.tools_dir.resolve()))
import capture_motion

saved=(root/'docs/experiment-logs/2026-11-19-motion-sharp60-positive-v153-01.log').read_text(encoding='utf-8').splitlines()
begin=next(i for i,s in enumerate(saved) if s.startswith('EDGE_CLOCK_CHECK '))
end=next(i for i,s in enumerate(saved) if s.startswith('RUN BENCH MOTION:'))
prefix=[s for s in saved[begin:end+1] if not s.startswith('HOST ')]
prefix=[capture_motion.PROFILE if s.startswith('BENCH_PROFILE ') else s for s in prefix]
idle='100,0,0,0,0,0.0,0.0,0.0\n'*3
pwm=next(s for s in saved if s.startswith('PWM '))

class VirtualPort:
    def __init__(self,scenario):self.scenario=scenario;self.queue=bytearray(idle.encode());self.clock=0;self.sent=[];self.drives=0;self.stops=0;self.queries=0
    def __enter__(self):return self
    def __exit__(self,*args):pass
    @property
    def in_waiting(self):return len(self.queue)
    def append(self,text):self.queue.extend(text.encode('ascii'))
    def read(self,n):self.clock+=.1;result=bytes(self.queue[:n]);del self.queue[:n];return result
    def flush(self):pass
    def terminal(self,reason):
        elapsed=500000 if self.scenario=='not_moving' else 170000
        speed=0 if self.scenario=='not_moving' else 3.5
        self.append(f'{reason}\nRUN_SUMMARY elapsed_us,displacement_rad: {elapsed},.1\nSTOP_DETAIL elapsed_us,filtered_rad_s,raw_rad_s,window10ms_rad_s: {elapsed},{speed},0,0\nTRACE_END\n'+idle)
    def write(self,data):
        self.sent.append(data)
        if data==b'c':
            self.queries+=1;reply=pwm.replace('gate_inputs_low=1','gate_inputs_low=0') if self.scenario=='bad_gate' and self.queries>1 else pwm
            self.append(reply+'\nFIRMWARE diag_version=153\n')
        elif data==b'"':
            self.drives+=1
            if self.drives==1:
                text='\n'.join(prefix)+'\n'
                if self.scenario in ('bad_metadata','alignment_abort'):text=re.sub(r'dq_limit_A=\S+','dq_limit_A=.99',text)
                self.append(text)
                if self.scenario in ('normal','missing_ack'):self.terminal('STOP: bench motion sequence complete')
                if self.scenario=='protect_before_timer':self.terminal('STOP: motion tracking error exceeded 1deg for 100ms')
            else:self.append('RUN BENCH MOTION: unexpected restart\n' if self.scenario=='bad_lock' else 'Rejected: unavailable or already attempted; reset for another test.\n')
        elif data==b's':
            self.stops+=1
            if self.scenario=='missing_ack':return
            if self.stops==1 and self.scenario=='alignment_abort':self.append('STOP: slow alignment aborted or pole-pair check failed\n');return
            if self.stops==1 and self.scenario in ('live','not_moving','bad_lock','bad_gate','protect_race'):
                self.terminal('STOP: motion tracking error exceeded 1deg for 100ms' if self.scenario=='protect_race' else 'STOP: serial stop')
            else:self.append('STOP: serial stop\n')

for scenario in ('normal','live','bad_metadata','alignment_abort','timeout','missing_ack','protect_before_timer','protect_race','not_moving','bad_lock','bad_gate'):
    port=VirtualPort(scenario)
    with tempfile.TemporaryDirectory() as directory:
        out=Path(directory)/'capture.log'
        argv=['capture_motion.py','--rpm','60','--shape','1','--direction','1','--output',str(out)]
        if scenario not in ('normal','bad_metadata','alignment_abort','timeout','missing_ack'):argv+=['--stop-after-ms','170']
        error=None
        with patch.object(sys,'argv',argv),patch.object(capture_motion.serial,'Serial',return_value=port),patch.object(capture_motion.list_ports,'comports',return_value=[types.SimpleNamespace(device='COM10',vid=0x0483,pid=0x374b)]),patch.object(capture_motion.time,'monotonic',side_effect=lambda:port.clock),contextlib.redirect_stdout(io.StringIO()):
            try:capture_motion.main()
            except RuntimeError as exc:error=str(exc)
        text=out.read_text(encoding='utf-8')
        if scenario in ('normal','live'):assert error is None,(scenario,error)
        else:
            expected={'bad_metadata':'Motion current/protection profile mismatch','alignment_abort':'Motion current/protection profile mismatch','timeout':'Missing STOP/whole trace','missing_ack':'Stop acknowledgement missing',
                      'protect_before_timer':'Protection stopped before','protect_race':'Protection stopped before','not_moving':'did not occur during verified fast motion',
                      'bad_lock':'Same-boot drive lock failed','bad_gate':'Missing gate OFF confirmation'}[scenario]
            assert error and expected in error,(scenario,error)
        assert b's' in port.sent and port.drives<=2
        if scenario=='live':assert all(s in text for s in ('HOST stop_during_motion_verified=True','HOST drive_lock_verified=True','HOST gate_off_verified=True','HOST verification_final_stop_acknowledged=True'))
        if scenario in ('protect_before_timer','protect_race','not_moving'):assert 'HOST stop_during_motion_verified=True' not in text and port.drives==1
        if scenario in ('bad_lock','bad_gate'):assert port.sent[-1]==b's' and 'HOST verification_final_stop_acknowledged=True' in text
        if scenario=='missing_ack':assert 'HOST final_stop_acknowledged=False' in text
        if scenario=='alignment_abort':assert port.stops==2 and 'HOST final_stop_acknowledged=True' in text
    print('PASS virtual serial:',scenario)
print('PASS: no real serial/reset/upload calls; protection-stop races cannot masquerade as live-stop proof; verification faults end with acknowledged stop')
with tempfile.TemporaryDirectory() as directory:
    argv=['capture_motion.py','--rpm','60','--shape','1','--direction','1','--output',str(Path(directory)/'stale.log')]
    with patch.object(sys,'argv',argv),patch.object(capture_motion,'PROFILE','STALE PROFILE'),patch.object(capture_motion.serial,'Serial') as opener:
        try:capture_motion.main()
        except RuntimeError as exc:assert 'source/cache profile mismatch' in str(exc)
        else:raise AssertionError('Stale imported profile accepted')
        opener.assert_not_called()
print('PASS: stale host profile rejected before opening serial')
