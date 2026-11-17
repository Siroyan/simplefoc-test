from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from current_tracking import CONFIG,ACTIVE,parse_current_tracking,verify_current_tracking
from capture_test import candidate_velocity_profile,candidate_current_lpf,candidate_reference_duration,verify_active_filters,verify_rolling_profile,DWT_EDGE_COMMANDS,HIGH_RANGE_COMMANDS
from edge_estimator import ESTIMATOR

assert parse_current_tracking([CONFIG,ACTIVE])['active']==ACTIVE
assert parse_current_tracking([]) is None
assert candidate_velocity_profile('@')==(.12,1.2,.0005,6.283185)
assert candidate_current_lpf('@')==.001 and candidate_reference_duration('@')==800000
assert candidate_velocity_profile('%')==(.075,1.2,.0005,6.283185)
assert candidate_current_lpf('%')==.001 and candidate_reference_duration('%')==800000
assert '%' in DWT_EDGE_COMMANDS and '%' in HIGH_RANGE_COMMANDS
verify_rolling_profile(ESTIMATOR,'%')
assert candidate_velocity_profile(':')==(.12,1.2,.0005,6.283185)
assert candidate_current_lpf(':')==.001 and candidate_reference_duration(':')==800000
assert ':' in DWT_EDGE_COMMANDS and ':' in HIGH_RANGE_COMMANDS
verify_rolling_profile(ESTIMATOR,':')
assert candidate_velocity_profile(';')==(.12,1.2,.0005,6.283185)
assert candidate_current_lpf(';')==.001 and candidate_reference_duration(';')==800000
assert ';' in DWT_EDGE_COMMANDS and ';' in HIGH_RANGE_COMMANDS
verify_rolling_profile(ESTIMATOR,';')
active400='CURRENT_ACTIVE q_LPF_s=0.0010 d_LPF_s=0.0010 velocity_LPF_s=0.0005 q_P=0.6000 q_I=400.0000 velocity_P=0.1200 velocity_I=1.2000'
verify_active_filters(active400,.001,.0005,.6,400,.12,1.2)
try:verify_active_filters(active400.replace('q_I=400.0000','q_I=200.0000'),.001,.0005,.6,400,.12,1.2)
except RuntimeError:pass
else:raise AssertionError('Old current I accepted in the faster current-loop profile')
config5=CONFIG.replace('0.0020','0.0050');active5=ACTIVE.replace('0.0020','0.0050')
assert parse_current_tracking([config5,active5])['active']==active5
for config,active in ((config5,ACTIVE),(CONFIG,active5)):
    try:parse_current_tracking([config,active])
    except RuntimeError:pass
    else:raise AssertionError('Requested and actual tracking times disagree')
try:verify_current_tracking(ACTIVE,True,.005)
except RuntimeError:pass
else:raise AssertionError('Host accepted 2ms when requesting 5ms')
active_filters='CURRENT_ACTIVE q_LPF_s=0.0010 d_LPF_s=0.0010 velocity_LPF_s=0.0005 q_P=0.6000 q_I=200.0000 velocity_P=0.0750 velocity_I=1.2000'
verify_active_filters(active_filters,.001,.0005,.6,200,.075,1.2)
for wrong in (active_filters.replace('velocity_P=0.0750','velocity_P=0.1200'),active_filters.replace('q_LPF_s=0.0010','q_LPF_s=0.0005')):
    try:verify_active_filters(wrong,.001,.0005,.6,200,.075,1.2)
    except RuntimeError:pass
    else:raise AssertionError('Incorrect lower-P profile accepted after initialization')
for wrong in ([CONFIG],[ACTIVE],[CONFIG,ACTIVE,ACTIVE],[CONFIG,ACTIVE.replace('0.0020','0.0000')],[CONFIG.replace('0.0020','nan'),ACTIVE]):
    try:parse_current_tracking(wrong)
    except (ValueError,RuntimeError):pass
    else:raise AssertionError('Missing or incorrect active tracking time accepted')
print('PASS: explicit and actual q tracking time, unchanged d mode, required config pair, high-speed current profile')
