"""An edge-control run must preserve its guard and expose its actual estimator."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from edge_estimator import ESTIMATOR,CONFIG,CONFIG_DWT,parse_edge_estimator,verify_edge_estimator_config,verify_edge_clock_check
from capture_test import verify_rolling_profile,candidate_velocity_profile,candidate_current_lpf,candidate_reference_duration

summary='EDGE_ESTIMATOR_SUMMARY invalid=0 samples=5 signed_count=8012 span_us=999'
lines=[ESTIMATOR,CONFIG,summary,'STOP: 3-second test complete']
assert parse_edge_estimator(lines)['samples']==5
assert parse_edge_estimator([]) is None
verify_rolling_profile(ESTIMATOR,'[')
assert candidate_velocity_profile('[')==(.12,1.2,.0005,6.283185)
assert candidate_current_lpf('[')==.001 and candidate_reference_duration('[')==2000000
assert candidate_velocity_profile('_')==(.12,1.2,.0005,6.283185)
assert candidate_current_lpf('_')==.001 and candidate_reference_duration('_')==800000
verify_rolling_profile(ESTIMATOR,'_')
for bad in (lines[1:],lines+[summary],
            [line.replace('phase_gain_s_inv=200','phase_gain_s_inv=400') for line in lines],
            [line.replace('guard_limit_rad_s=8.5','guard_limit_rad_s=9') for line in lines],
            [line.replace('sample_gap_limit_us=5000','sample_gap_limit_us=20000') for line in lines],
            [line.replace('invalid=0','invalid=1') for line in lines],
            [line.replace('span_us=999','span_us=0') for line in lines]):
    try:parse_edge_estimator(bad)
    except (ValueError,RuntimeError):pass
    else:raise AssertionError('Missing/incorrect edge-estimator configuration accepted')
for bad in (ESTIMATOR.replace('edge_period4_phase','rolling_LS2ms'),ESTIMATOR.replace('guard_limit_rad_s=8.5','guard_limit_rad_s=9')):
    try:verify_rolling_profile(bad,'[')
    except RuntimeError:pass
    else:raise AssertionError('Wrong estimator or guard accepted by capture preflight')
try:verify_edge_estimator_config(CONFIG.replace('prediction_count_bound=1','prediction_count_bound=2'))
except RuntimeError:pass
else:raise AssertionError('Wrong extrapolation bound accepted')
assert parse_edge_estimator([ESTIMATOR,CONFIG,summary.replace('invalid=0','invalid=1'),'STOP: invalid edge velocity estimate'])['invalid']==1
check='EDGE_CLOCK_CHECK advancing=1 gates_off=1 cycles_per_us=170 core_regressions=2 maximum_core_backstep_us=999'
fault='EDGE_CLOCK_FAULT reason=0 now_us=2999900 edge_us=2999800 age_us=100 update_gap_us=400'
dwt=[ESTIMATOR,CONFIG_DWT,summary,check,fault,'STOP: 3-second test complete']
assert parse_edge_estimator(dwt)['clock_check']==check
verify_rolling_profile(ESTIMATOR,']')
assert candidate_velocity_profile(']')==(.12,1.2,.0005,6.283185) and candidate_current_lpf(']')==.001
verify_edge_estimator_config(CONFIG_DWT,True)
for bad in (dwt[:-3]+dwt[-2:], [l.replace('reason=0','reason=1') for l in dwt], [l.replace('cycles_per_us=170','cycles_per_us=160') for l in dwt]):
    try:parse_edge_estimator(bad)
    except (ValueError,RuntimeError):pass
    else:raise AssertionError('Missing/bad cycle clock or completed clock fault accepted')
try:verify_edge_clock_check(check.replace('advancing=1','advancing=0'))
except RuntimeError:pass
else:raise AssertionError('Stopped hardware clock accepted')
print('PASS: edge estimator mode, gains, count/sample bounds, independent guard, missing metadata and fault/completion distinction')
