from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from capture_test import candidate_velocity_profile,candidate_current_lpf,candidate_reference_duration,verify_response_probe,verify_active_filters,verify_rolling_profile,HIGH_RANGE_COMMANDS,DWT_EDGE_COMMANDS
from edge_estimator import ESTIMATOR

config='CURRENT_PROBE frequency_Hz=17.50 phase_rad=0.000000 amplitude_nominal_A=0.0200 start_us=800000 blend_us=200000 total_bound_A=0.4500 stationary_attempt=0 excitation_only=1'
active='CURRENT_ACTIVE q_LPF_s=0.0010 d_LPF_s=0.0010 velocity_LPF_s=0.0005 q_P=0.6000 q_I=400.0000 velocity_P=0.1200 velocity_I=1.2000'
for command in ('(',')'):
    assert command in HIGH_RANGE_COMMANDS and command in DWT_EDGE_COMMANDS
    assert candidate_velocity_profile(command)==(.12,1.2,.0005,6.283185)
    assert candidate_current_lpf(command)==.001 and candidate_reference_duration(command)==800000
    verify_rolling_profile(ESTIMATOR,command)
    verify_active_filters(active,.001,.0005,.6,400,.12,1.2)
    expected=config if command=='(' else config.replace('phase_rad=0.000000','phase_rad=3.141593')
    verify_response_probe(expected,command)
    for bad in (expected.replace('17.50','13.50'),expected.replace('0.0200','0.0300'),expected.replace('800000','400000'),expected.replace('0.4500','0.4600'),expected.replace('excitation_only=1','excitation_only=0')):
        try:verify_response_probe(bad,command)
        except RuntimeError:pass
        else:raise AssertionError('Wrong excitation profile accepted')
try:verify_response_probe(config,')')
except RuntimeError:pass
else:raise AssertionError('Wrong excitation phase accepted')
verify_response_probe(config.replace('17.50','13.50'),'T')
print('PASS: 60rpm probe modes, actual current/velocity gains, filters, ramp, estimator/guards, frequency/phase/amplitude/activation/total-cap metadata')
