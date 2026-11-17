"""Missing edge data, inconsistent counters and altered guards must be rejected."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from run_edge_log import CONFIG,ACTIVE,BEGIN,OPEN,ALIGN,HIGH_ACTIVE,HIGH_OPEN,HIGH_ESTIMATOR,HIGH_REFERENCE,HIGH_DRIVE,FAST_REFERENCE,FAST_DRIVE,parse_run_edges
from speed_range import GUARD_CONFIG,STATS_CONFIG

observer='CURRENT_CONFIG observer_only=1 max_age_us=2000 deviation_delay_us=20000'
check='EDGE_CLOCK_CHECK advancing=1 gates_off=1 cycles_per_us=170 core_regressions=9 maximum_core_backstep_us=999'

def fixture(count=8,reverse=False):
    direction=-1 if reverse else 1
    cycle=[0,1,3,2] if reverse else [0,2,3,1]
    rows=[f'{n*250},{direction*n},{cycle[n%4]}' for n in range(max(1,count-255),count+1)]
    return [CONFIG,ACTIVE,observer,check,ALIGN,OPEN,'STOP: 3-second test complete',
            f'RUN_EDGE_SUMMARY total_edges={count} retained={min(256,count)} origin_count=0 origin_AB=0 end_count={direction*count} end_us=3000000',
            BEGIN,'edge_us,signed_count,AB_state',*rows,'RUN_EDGES_END',
            f'ABI_QUALITY_RUN callbacks={count} invalid=0 duplicate=0 reversals=0 count_delta={direction*count} encoder_count_delta={direction*count}']

for reverse in (False,True):
    for count in (8,300):
        result=parse_run_edges(fixture(count,reverse))
        assert len(result['rows'])==min(count,256)
        assert result['tail_invalid_transitions']==result['tail_duplicate_callbacks']==0
        assert abs(result['tail_speed']['mean_encoder_rad_s']-(-1 if reverse else 1)*6.283185307179586)<1e-10
        assert result['tail_speed']['one_edge_RMS_about_mean_rad_s']<1e-10

lines=fixture()
assert parse_run_edges([]) is None
high=[l.replace(ACTIVE,HIGH_ACTIVE).replace(OPEN,HIGH_OPEN) for l in lines]+[HIGH_ESTIMATOR,HIGH_REFERENCE,HIGH_DRIVE,GUARD_CONFIG,STATS_CONFIG]
assert parse_run_edges(high)['target_rad_s']==6.283185
fast=[l.replace(HIGH_REFERENCE,FAST_REFERENCE).replace(HIGH_DRIVE,FAST_DRIVE).replace(STATS_CONFIG,STATS_CONFIG.replace('2000000','800000')) for l in high]
assert parse_run_edges(fast)['target_rad_s']==6.283185
for wrong in (high[:-1], [l.replace('hard_abs_rad_s=8.5','hard_abs_rad_s=9') for l in high], [l.replace('current_PI=off','current_PI=on') for l in high]):
    try:parse_run_edges(wrong)
    except (ValueError,RuntimeError):pass
    else:raise AssertionError('Unverified higher-range raw drive accepted')
for bad in (lines[1:],lines+[CONFIG],lines[:-2]+lines[-1:],
            [l.replace('guard_limit_rad_s=5','guard_limit_rad_s=8.5') for l in lines],
            [l.replace('retained=8','retained=7') for l in lines],
            [l.replace('callbacks=8','callbacks=9') for l in lines],
            [l.replace('end_count=8','end_count=9') for l in lines],
            [l.replace('500,2,3','250,2,3') for l in lines],
            [l.replace('500,2,3','500,7,3') for l in lines],
            [l.replace('observer_only=1','observer_only=0') for l in lines],
            [l.replace('voltage_limit_V=0.500','voltage_limit_V=0.700') for l in lines]):
    try:parse_run_edges(bad)
    except (ValueError,RuntimeError):pass
    else:raise AssertionError('Missing or inconsistent raw capture was accepted')
try:parse_run_edges([l for l in lines if not l.startswith('STOP:')])
except ValueError:pass
else:raise AssertionError('A running dump was accepted')
print('PASS: raw edge signed timing, ring tail, frozen dump, guard/ADC/clock metadata, independent count agreement, corruption/truncation rejection')
