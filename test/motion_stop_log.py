import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from verify_motion_stop import verify_stop
root=Path(__file__).resolve().parents[1];path=root/'docs/experiment-logs/2026-11-20-motion-fast-stop-v161-01.log'
assert verify_stop(path)['pass'];text=path.read_text(encoding='utf-8')
with tempfile.TemporaryDirectory() as directory:
    p=Path(directory)/'bad.log'
    for bad in (text.replace('STOP: serial stop','STOP: protective stop',1),text.replace('HOST verification_final_stop_acknowledged=True','HOST verification_final_stop_acknowledged=False'),text[:text.index('HOST drive_lock_verified=True')]+text[text.index('HOST drive_lock_verified=True'):].replace('gate_inputs_low=1','gate_inputs_low=0'),text.replace('181370,6.379995','181370,0.000000')):
        p.write_text(bad,encoding='utf-8')
        try:verify_stop(p)
        except ValueError:pass
        else:raise AssertionError('Protective/slow/unacknowledged/energized stop accepted')
print('PASS: actual fast serial stop verified; first-stop races, missing final ACK, nonmoving shaft and gate inconsistency rejected')
