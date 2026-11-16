"""A known sinusoid sampled at variable window ages must retain phase/amplitude."""
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from analyze_ripple import fit_sinusoid,window_midpoint_us

frequency=20
rows=[]
values=[]
for n in range(220):
    logged=800000+n*10000
    age=200+((n*307)%9800)
    row={'trace_us':logged,'window_age_us':age}
    value=.3*math.cos(math.tau*frequency*(logged-age-5000)/1e6-.4)
    rows.append(row);values.append(value)
phases=[math.tau*frequency*window_midpoint_us(r)/1e6 for r in rows]
fit=fit_sinusoid(phases,values,[1]*len(rows))
assert abs(fit['amplitude_rad_s']-.3)<1e-10
assert abs(fit['phase_rad']-.4)<1e-10
assert fit['explained_variance']>1-1e-10
old=fit_sinusoid([math.tau*frequency*r['trace_us']/1e6 for r in rows],values,[1]*len(rows))
assert old['explained_variance']<.95
assert window_midpoint_us({'trace_us':10000})==5000
try:window_midpoint_us({'trace_us':100000,'window_age_us':65535})
except ValueError:pass
else:raise AssertionError('Saturated timing must not be used for phase fitting')
print('PASS: variable-age midpoint timing recovers known 20Hz amplitude .3/phase .4rad; old log-time fit is biased; old-format fallback and saturated age handled')
