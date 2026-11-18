import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from analyze_bench import analyze
base=Path(__file__).resolve().parents[1]/'docs/experiment-logs'
good=base/'2026-11-19-bench-pulse10-v123-01.log'
assert analyze(good)['pass']
assert analyze(base/'2026-11-19-bench-pulse05-v123-01.log')['pass']
assert not analyze(base/'2026-11-19-bench-pulse05-v121-01.log')['pass']
assert analyze(base/'2026-11-19-bench-joint-positive-v133-01.log')['pass']
assert not analyze(base/'2026-11-19-bench-joint-positive-v132-01.log')['pass']
assert not analyze(base/'2026-11-19-bench-joint-positive-v131-01.log')['pass']
with tempfile.TemporaryDirectory() as directory:
    p=Path(directory)/'copy.log'
    text=good.read_text(encoding='utf-8')
    for bad in (text.replace('HOST final_stop_acknowledged=True','HOST final_stop_acknowledged=False'),
                text.replace('ABI_QUALITY_RUN callbacks=34 invalid=0','ABI_QUALITY_RUN callbacks=34 invalid=1'),
                text.replace('TRACE_END','REMOVED')):
        p.write_text(bad,encoding='utf-8')
        try:analyze(p)
        except ValueError:pass
        else:raise AssertionError('Corrupt or incomplete evidence accepted')
print('PASS: saved full-range positive/negative response, protection stop excluded, missing stop/trace and corrupt ABI rejected')
