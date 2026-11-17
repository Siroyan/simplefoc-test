from pathlib import Path
import math
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from analyze_index_harmonics import index_angle,series,compare

data=[]
for n,(origin,direction) in enumerate(((.03,1),(1.7,1),(-3,-1))):
    rows=[]
    for i in range(321):
        t=i*.002
        theta=math.tau*t+.5/(14*math.tau)*math.sin(14*math.tau*t)
        rows.append({'trace_us':i*2000,'angle_rad':origin+direction*theta})
    data.append({'log':str(n),'anchor':origin,**series(rows,origin,direction)})
model=compare(data,[14])
assert all(f['held_out_R2']>.98 for f in model['leave_one_run_out'])
assert all(abs(r['harmonics'][0]['amplitude_rad_s']-.5)<.01 for r in model['per_run'])
# A half-cycle error in one Index anchor must spoil prediction of that run.
wrong=[dict(d) for d in data]
wrong[0]={**data[0],'angles':[a+math.pi/14 for a in data[0]['angles']]}
assert compare(wrong,[14])['leave_one_run_out'][0]['held_out_R2']<0
assert index_angle(['INDEX_REFERENCE pulse=1 encoder_angle_rad=0.010000',
                    f'INDEX_REFERENCE pulse=2 encoder_angle_rad={.01+math.tau:.6f}'])==.01
for bad in ([],['INDEX_REFERENCE pulse=1 encoder_angle_rad=0.01','INDEX_REFERENCE pulse=2 encoder_angle_rad=0.02']):
    try:index_angle(bad)
    except ValueError:pass
    else:raise AssertionError('Missing or inconsistent Index accepted')
for bad in ([rows[0],rows[0]], [rows[0],dict(rows[1],trace_us=6000)]):
    try:series(bad,0,1)
    except ValueError:pass
    else:raise AssertionError('Missing or duplicate interval accepted')
print('PASS: cross-boot origin shifts, encoder direction, known harmonic amplitude, true held-out phase rejection, Index/cadence bounds')
