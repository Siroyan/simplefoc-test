"""Snapshot completeness and timing checks, without hardware access."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from guard_peak import CONFIG,parse_guard_peaks,verify_guard_peak_config

sample=' valid=1 elapsed_us=900000 sample_age_us=15 adc_age_us=140 guard_rad_s=-4 control_rad_s=-3.8 reference_rad_s=-2.5 iq_applied_nominal_A=.15 iq_next_nominal_A=-.1 iq_filtered_nominal_A=.18 Uq_V=.1 Ud_V=.01 correction_nominal_A=.02'
lines=[CONFIG,'GUARD_PEAK scope=whole'+sample,'GUARD_PEAK scope=after_ramp'+sample,
       'RUN_SUMMARY elapsed_us,displacement_rad: 2999900,6.48','STOP_GUARD run_abs_peak_rad_s=4', 'STOP: 3-second test complete']
r=parse_guard_peaks(lines,required=True)
assert r['whole']['iq_applied_nominal_A']==.15 and r['whole']['iq_next_nominal_A']==-.1
assert parse_guard_peaks([]) is None
for bad in ([],lines[1:],lines+[lines[1]],
            [line.replace('elapsed_us=900000','elapsed_us=3000000') for line in lines],
            [line.replace('elapsed_us=900000','elapsed_us=700000') for line in lines],
            [line.replace('iq_next_nominal_A=-.1','iq_next_nominal_A=nan') for line in lines],
            [line.replace('run_abs_peak_rad_s=4','run_abs_peak_rad_s=4.1') for line in lines]):
    try:parse_guard_peaks(bad,required=True)
    except (RuntimeError,ValueError):pass
    else:raise AssertionError('Invalid/missing peak telemetry accepted')
for bad in (CONFIG.replace('800000','400000'),CONFIG.replace('post_move','pre_move'),CONFIG.replace('after_stop','while_running')):
    try:verify_guard_peak_config(bad)
    except RuntimeError:pass
    else:raise AssertionError('Wrong peak timing accepted')
early=[CONFIG,lines[1].replace('900000','200000'),'GUARD_PEAK scope=after_ramp valid=0',
       'RUN_SUMMARY elapsed_us,displacement_rad: 300000,.2','STOP: unexpected velocity']
assert parse_guard_peaks(early,required=True)['after_ramp'] is None
slow=[line.replace('steady_start_us=800000','steady_start_us=1500000').replace('elapsed_us=900000','elapsed_us=1900000') for line in lines]
slow.append('REFERENCE_RAMP duration_us=1500000 target_rad_s=-2.500 trace_target=final_goal origin=trace_start')
assert parse_guard_peaks(slow,required=True)['steady_start_us']==1500000
try:parse_guard_peaks([line.replace('steady_start_us=1500000','steady_start_us=800000') for line in slow],required=True)
except RuntimeError:pass
else:raise AssertionError('Slow-ramp peak window accepted the old startup boundary')
print('PASS: whole/post-ramp scopes, prior/next Iq distinction, timing/configuration, missing/nonfinite/duplicate/mismatched payload and early fault')
