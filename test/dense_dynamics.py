"""Known high-frequency motion/current data verify units, lag sign and reversal."""
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from analyze_dense_dynamics import spectrum


def synthetic(sign):
    rows=[]
    elapsed=2350000
    for n in range(320):
        elapsed+=2000+(n%3-1)*30
        t=elapsed/1e6
        phase=math.tau*58*t
        rows.append({'trace_us':elapsed,'angle_rad':sign*(t+.2*math.sin(phase)/(math.tau*58)),
                     'iq_target_nominal_A':sign*(.15+.03*math.sin(phase)),
                     'iq_filtered_nominal_A':sign*(.15+.018*math.sin(phase-math.pi/6)),
                     'target_rad_s':sign})
    return rows


outputs=[spectrum(synthetic(sign)) for sign in (1,-1)]
for sign,result in zip((1,-1),outputs):
    peak=result['peak_components'][0]
    assert abs(peak['frequency_Hz']-58)<=.5
    assert abs(peak['interval_velocity_amplitude_rad_s']-.2)<.01
    assert abs(peak['closed_loop_current_component_ratio']-.6)<.005
    assert abs(peak['closed_loop_current_phase_deg']+30)<.5
    assert abs(result['signed_mean_velocity_rad_s']-sign)<.01
    assert .13<result['interval_velocity_tracking_rms_rad_s']<.15
    assert 1900<result['interval_us_range'][0]<=result['interval_us_range'][1]<2100
assert abs(outputs[0]['interval_velocity_tracking_rms_rad_s']-outputs[1]['interval_velocity_tracking_rms_rad_s'])<1e-12
print('PASS: known 58Hz motion and current gain .6/lag -30deg, irregular timing, rad/s units, sign symmetry and interval RMS')
