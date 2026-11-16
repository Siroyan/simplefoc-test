"""Inspect high-rate RAM tails without treating closed-loop spectra as plant identification."""
import argparse
import cmath
import csv
import io
import json
import math
from pathlib import Path
import statistics
from analyze_ripple import solve_weighted, explained_variance


def harmonic(times, values, weights, frequency):
    origin=times[0]
    basis=[[1,math.cos(math.tau*frequency*(t-origin)),math.sin(math.tau*frequency*(t-origin))] for t in times]
    coefficients=solve_weighted(basis,values,weights)
    if coefficients is None:
        raise ValueError("Degenerate harmonic fit")
    return complex(coefficients[1],-coefficients[2]), explained_variance(basis,values,weights,coefficients)


def interval_series(rows, angles=None):
    if angles is None: angles=[r['angle_rad'] for r in rows]
    return [(b['trace_us']+a['trace_us'])/2e6 for a,b in zip(rows,rows[1:])], [
        (right-left)*1e6/(b['trace_us']-a['trace_us'])
        for a,b,left,right in zip(rows,rows[1:],angles,angles[1:])]


def spectrum(rows):
    times,velocity=interval_series(rows)
    iq=[(a['iq_filtered_nominal_A']+b['iq_filtered_nominal_A'])/2 for a,b in zip(rows,rows[1:])]
    target=[(a['iq_target_nominal_A']+b['iq_target_nominal_A'])/2 for a,b in zip(rows,rows[1:])]
    dt=[(b['trace_us']-a['trace_us'])/1e6 for a,b in zip(rows,rows[1:])]
    span=times[-1]-times[0]
    weights=[step*math.sin(math.pi*(t-times[0])/span)**2 for t,step in zip(times,dt)]
    frequencies=[n/2 for n in range(6,361) if n/2<.4/max(dt)]
    fits=[]
    for frequency in frequencies:
        v,r2=harmonic(times,velocity,weights,frequency)
        measured,_=harmonic(times,iq,weights,frequency)
        requested,_=harmonic(times,target,weights,frequency)
        ratio=measured/requested if abs(requested)>=.005 else None
        fits.append({'frequency_Hz':frequency,'velocity_single_frequency_R2':r2,
                     'interval_velocity_amplitude_rad_s':abs(v),
                     'approx_angle_amplitude_rad':abs(v)/(math.tau*frequency),
                     'iq_target_amplitude_nominal_A':abs(requested),
                     'iq_filtered_amplitude_nominal_A':abs(measured),
                     'closed_loop_current_component_ratio':abs(ratio) if ratio is not None else None,
                     'closed_loop_current_phase_deg':math.degrees(cmath.phase(ratio)) if ratio is not None else None})
    peaks=[]
    for item in sorted(fits,key=lambda f:f['velocity_single_frequency_R2'],reverse=True):
        if all(abs(item['frequency_Hz']-other['frequency_Hz'])>=2/span for other in peaks):
            peaks.append(dict(item))
        if len(peaks)==8: break
    # A deliberately narrow null reference: steady rotation quantized at the
    # actual sample times. It cannot exclude aliasing during nonuniform motion.
    speed=(rows[-1]['angle_rad']-rows[0]['angle_rad'])/(sum(dt))
    tick=math.tau/4000
    for peak in peaks:
        null_amplitudes=[]
        for phase in range(8):
            angles=[round((speed*(r['trace_us']-rows[0]['trace_us'])/1e6+phase*tick/8)/tick)*tick for r in rows]
            _,null_velocity=interval_series(rows,angles)
            value,_=harmonic(times,null_velocity,weights,peak['frequency_Hz'])
            null_amplitudes.append(abs(value))
        peak['constant_speed_quantization_null_amplitude_max_rad_s']=max(null_amplitudes)
    target_speed=statistics.mean(r['target_rad_s'] for r in rows)
    return {'covered_start_us':rows[0]['trace_us'],'covered_end_us':rows[-1]['trace_us'],
            'intervals':len(dt),'interval_us_range':[min(dt)*1e6,max(dt)*1e6],
            'elapsed_s':sum(dt),'signed_mean_velocity_rad_s':speed,
            'interval_velocity_tracking_rms_rad_s':math.sqrt(sum(step*(v-target_speed)**2 for step,v in zip(dt,velocity))/sum(dt)),
            'frequency_resolution_scale_Hz':1/span,'peak_components':peaks,
            'dominant_components_in_bands':[
                {'band_Hz':[low,high],**max((f for f in fits if low<=f['frequency_Hz']<=high),key=lambda f:f['velocity_single_frequency_R2'])}
                for low,high in ((3,20),(40,100))],
            'selected_components':[f for f in fits if f['frequency_Hz'] in (7.5,13,20,40,58,80,100)]}


def load(path):
    lines=path.read_text(encoding='utf-8').splitlines()
    if 'HOST final_stop_acknowledged=True' not in lines:
        raise ValueError('Completed capture with final stop acknowledgement required')
    first=next(n for n,line in enumerate(lines) if line.startswith('trace_us,'))
    last=lines.index('TRACE_END',first)
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(io.StringIO('\n'.join(lines[first:last])))]
    if len(rows)<100 or statistics.median(b['trace_us']-a['trace_us'] for a,b in zip(rows,rows[1:]))>2500:
        raise ValueError('At least 100 high-rate samples with median cadence <=2.5ms required')
    if any(b['trace_us']<=a['trace_us'] for a,b in zip(rows,rows[1:])):
        raise ValueError('Trace clock must increase')
    return rows,lines


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    rows,lines=load(args.log)
    result={'log':args.log.name,'samples':len(rows),
            'control':next((l for l in lines if l.startswith('CONTROL ')),None),
            'trace_configuration':[l for l in lines if l.startswith(('TRACE_CONFIG ','TRACE_SCOPE '))],
            'whole_run_guard':next((l for l in lines if l.startswith('STOP_GUARD ')),None),
            'whole_run_actuator':next((l for l in lines if l.startswith('CURRENT_ACTUATOR_STATS ')),None),
            **spectrum(rows),
            'note':'Only the retained RAM tail is analyzed. Interval differentiation increases ABI quantization noise. Fits use Hann time weights and irregular sample times; peaks are exploratory and selected after searching. Current ratios are coherent closed-loop components, not an independent current-loop transfer measurement or proof of cause. The constant-speed quantization null cannot exclude aliasing in nonuniform motion. No compensation is exported.'}
    output=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(output,encoding='utf-8')
    print(output)


if __name__=='__main__': main()
