import copy,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from run_motion_repetitions import seed_counts
from analyze_motion import analyze_motion
root=Path(__file__).resolve().parents[1]
positive=analyze_motion(root/'docs/experiment-logs/2026-11-19-motion-sharp60-positive-v161-02.log')
negative=analyze_motion(root/'docs/experiment-logs/2026-11-20-motion-sharp60-negative-v161-01.log')
assert seed_counts([positive,negative])=={1:1,-1:1}
bad=copy.deepcopy(negative);bad['profile']['iq_limit_nominal_A']=.3
fault=copy.deepcopy(negative);fault['stop']='STOP: protective stop'
for rows in ([positive,positive],[positive,bad],[positive,fault]):
    try:seed_counts(rows)
    except ValueError:pass
    else:raise AssertionError('Duplicate/mixed-range/faulted seed accepted')
print('PASS: valid explicit normal-run seeds counted once; rejected preflight never inferred from file numbering; duplicate/range/fault contamination rejected')
