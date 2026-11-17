from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from capture_test import candidate_velocity_profile,candidate_current_lpf,candidate_reference_duration,verify_adaptive_profile,verify_rolling_profile,HIGH_RANGE_COMMANDS,DWT_EDGE_COMMANDS,ADAPTIVE_COMMANDS
from edge_estimator import ESTIMATOR
assert '?' in HIGH_RANGE_COMMANDS & DWT_EDGE_COMMANDS & ADAPTIVE_COMMANDS
assert candidate_velocity_profile('?')==(.12,1.2,.0005,6.283185)
assert candidate_current_lpf('?')==.001 and candidate_reference_duration('?')==800000
verify_rolling_profile(ESTIMATOR,'?')
config='ADAPTIVE_CONFIG order=14 learning_rate=0.2000 correction_bound_nominal_A=0.010000 response_lag_rad=0.785398 gate_us=800000 blend_us=200000 total_limit_nominal_A=0.4500 learning_reference_rad_s=6.2832 learning_min_velocity_rad_s=1.8850 learning_max_velocity_rad_s=10.6814 anchor=run_start per_run_reset=1'
verify_adaptive_profile(config,'?')
assert candidate_velocity_profile('^')==(.12,1.2,.0005,6.283185)
assert candidate_current_lpf('^')==.001 and candidate_reference_duration('^')==800000
assert '^' in HIGH_RANGE_COMMANDS & DWT_EDGE_COMMANDS & ADAPTIVE_COMMANDS
verify_adaptive_profile(config+' trig=phase_rotation','^')
try:verify_adaptive_profile(config,'^')
except RuntimeError:pass
else:raise AssertionError('Unverified phase-calculation mode accepted')
for bad in (config.replace('order=14','order=84'),config.replace('0.010000','0.030000'),config.replace('800000','400000'),config.replace('0.4500','0.4600'),config.replace('6.2832','3.0000')):
    try:verify_adaptive_profile(bad,'?')
    except RuntimeError:pass
    else:raise AssertionError('Wrong high-speed adaptive configuration accepted')
print('PASS: explicit 14th order, .01A correction cap, 60rpm reference, actual baseline profile and unchanged limits/gating')
