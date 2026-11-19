"""Portable archive and explicit rejected-preflight audit, no device access."""
import json,shutil,sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from summarize_motion import recovered_preflight,verify_condition
root=Path(__file__).resolve().parents[1]
registry=json.loads((root/'docs/experiment-logs/2026-11-19-motion-preflight-aborts.json').read_text(encoding='utf-8'))['records'][0]
original=root/registry['log'];result=recovered_preflight(root,original,registry)
assert not result['final_stop_acknowledged'] and result['recovery_stop_acknowledged'] and not result['position_motion_started']
with tempfile.TemporaryDirectory() as directory:
    temporary=Path(directory);target=temporary/'docs/experiment-logs';target.mkdir(parents=True)
    archive='2026-11-19-motion-condition-archives.zip';shutil.copyfile(root/'docs/experiment-logs'/archive,target/archive)
    condition=json.loads((root/'docs/experiment-logs/2026-11-19-motion-v161-condition.json').read_text(encoding='utf-8'))
    verify_condition(temporary,condition) # No .pio exists here: clean-checkout path.
    current_archive='2026-11-20-motion-condition-archives.zip'
    shutil.copyfile(root/'docs/experiment-logs'/current_archive,target/current_archive)
    new_conditions=list((root/'docs/experiment-logs').glob('2026-11-20-motion-v*-condition.json'))
    assert len(new_conditions)==7
    for source in new_conditions:verify_condition(temporary,json.loads(source.read_text(encoding='utf-8')))
    for separator in ('/','\\'):
        foreign=dict(condition);foreign['snapshot_directory']=condition['snapshot_directory'].replace('\\','/').replace('/',separator)
        verify_condition(temporary,foreign)
    damaged=dict(condition);damaged['sha256']=dict(condition['sha256']);damaged['sha256']['src/main.cpp']='0'*64
    try:verify_condition(temporary,damaged)
    except ValueError:pass
    else:raise AssertionError('Portable archive hash tampering accepted')
    entry=dict(registry);entry['recovery_log']='recovery.log';recovery=(root/registry['recovery_log']).read_text(encoding='utf-8')
    fake=temporary/'attempt.log';fake.write_text(original.read_text(encoding='utf-8'),encoding='utf-8')
    for bad in (recovery.replace('STOP: serial stop','STOP: missing'),recovery.replace('gate_inputs_low=1','gate_inputs_low=0')):
        (temporary/'recovery.log').write_text(bad,encoding='utf-8')
        try:recovered_preflight(temporary,fake,entry)
        except ValueError:pass
        else:raise AssertionError('Unacknowledged/energized recovery accepted')
    (temporary/'recovery.log').write_text(recovery,encoding='utf-8')
    fake.write_text(original.read_text(encoding='utf-8')+'\nRUN BENCH MOTION: actually started\n',encoding='utf-8')
    try:recovered_preflight(temporary,fake,entry)
    except ValueError:pass
    else:raise AssertionError('A real position trial excluded as preflight-only')
print('PASS: both dated archives and all seven new conditions verified without .pio; corrupted hashes/stop/gates and misclassified real runs rejected')
