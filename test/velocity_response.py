"""Known closed-loop response verifies phase inversion and velocity-window timing."""
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from analyze_current_probe import velocity_response
for frequency,lag,phase,direction in ((f,l,p,d) for f in (13.5,17.5) for l in (-1.8,-.4,.4,2.3) for p in (0,math.pi) for d in (-1,1)):
    rows=[]
    for n in range(190):
        us=1100000+n*10000+(n%3)*37
        age=200+(n*719)%9800
        midpoint=(us-age-5000)*1e-6
        t=us*1e-6
        rows.append({'trace_us':us,'window_age_us':age,
                     'filtered_rad_s':1+.03*t+.16*math.sin(math.tau*frequency*t+phase-lag),
                     'window10ms_rad_s':direction*(1+.03*midpoint+.16*math.sin(math.tau*frequency*midpoint+phase-lag))})
    result=velocity_response(rows,frequency,.02,phase,direction)
    for row in result.values():
        assert abs(row['gain_rad_s_per_nominal_A']-8)<1e-9
        assert abs(math.remainder(math.radians(row['phase_deg'])+lag,math.tau))<1e-9
        assert row['coherent_velocity_component']['sine_R2_after_removing_linear_trend']>1-1e-10
try:velocity_response(rows,13.5,0,0)
except ValueError:pass
else:raise AssertionError('No identification from zero excitation')
print('PASS: known gain 8rad/s/A, leads/lags across phase wrap, 13.5/17.5Hz, inverse probes, both encoder signs and variable window timing; zero excitation refused')
