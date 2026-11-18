import copy,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from compare_bench_brake import compare
c={'brake':{'active':0,'brake_iq_A':-.1},'profile':{'current':.2},'pass':True,'log':'coast',
   'brake_status':{'start_velocity_rad_s':2.,'start_guard_rad_s':2.1},'max_forward_excursion_20ms_rad':.02,'opposition_samples':0}
a=copy.deepcopy(c);a.update(log='active',max_forward_excursion_20ms_rad=.01,opposition_samples=3);a['brake']['active']=1
assert compare(c,a)['pass']
bad=copy.deepcopy(a);bad['brake_status']['start_velocity_rad_s']=2.5
assert not compare(c,bad)['pass']
bad=copy.deepcopy(a);bad['max_forward_excursion_20ms_rad']=.018
assert not compare(c,bad)['pass']
bad=copy.deepcopy(a);bad['brake']['brake_iq_A']=-.2
try:compare(c,bad)
except ValueError:pass
else:raise AssertionError('Multiple changed factors accepted')
bad=copy.deepcopy(c);bad['max_forward_excursion_20ms_rad']=0
assert not compare(bad,a)['pass']
bad=copy.deepcopy(a);bad['brake_status']['start_velocity_rad_s']=0;bad['pass']=False
assert not compare(c,bad)['pass']
print('PASS: matched initial speeds, normalized forward travel, zero/unqualified baselines and multiple changed factors')

from analyze_bench import max_forward_excursion
assert max_forward_excursion([{'trace_us':0,'angle_rad':1},{'trace_us':10000,'angle_rad':1.2},{'trace_us':20000,'angle_rad':.9}])>.199999
assert max_forward_excursion([{'trace_us':0,'angle_rad':1},{'trace_us':10000,'angle_rad':.8}]) is None
print('PASS: forward stopping distance uses the maximum; later reversal cannot cancel it; incomplete windows rejected')
