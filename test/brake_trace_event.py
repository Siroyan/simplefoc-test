import sys,tempfile
from pathlib import Path
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'tools'))
from analyze_bench import analyze
old=root/'docs/experiment-logs/2026-11-19-bench-coast-v135-01.log'
with tempfile.TemporaryDirectory() as folder:
    fake=Path(folder)/'fake.log'
    fake.write_text(old.read_text(encoding='utf-8').replace('trace_layout=brake_ab_union','trace_layout=brake_ab_union transition_trace=forced'),encoding='utf-8')
    try:analyze(fake)
    except ValueError as e:assert 'transition-triggered' in str(e)
    else:raise AssertionError('Delayed first row accepted as transition-triggered capture')
try:analyze(root/'docs/experiment-logs/2026-11-19-brake-transition-coast-v140-01.log')
except ValueError as e:assert 'gates/voltage' in str(e)
else:raise AssertionError('Mixed pre-/post-disable voltage capture accepted')
print('PASS: delayed transition sample and stale pre-disable voltage payload rejected')
