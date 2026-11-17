"""Check whether ABI ripple harmonics predict other completed runs in the Index frame."""
import argparse
import json
import math
from pathlib import Path
import re
from analyze_dense_dynamics import load
from analyze_ripple import solve_weighted, explained_variance
from speed_range import parse_interval_velocity_stats


def index_angle(lines):
    angles=[float(re.search(r'encoder_angle_rad=(-?[0-9.]+)',line)[1])
            for line in lines if line.startswith('INDEX_REFERENCE pulse=')]
    if not angles or not all(math.isfinite(a) for a in angles):
        raise ValueError('Recorded Index angles required')
    anchor=angles[0]
    if any(abs(math.remainder(a-anchor,math.tau))>2*math.tau/4000 for a in angles):
        raise ValueError('Index phases disagree by more than two ABI counts')
    return anchor


def series(rows,anchor,direction):
    if direction not in (-1,1) or not math.isfinite(anchor):
        raise ValueError('Finite Index anchor and measured encoder direction required')
    angles=[];values=[];weights=[]
    for a,b in zip(rows,rows[1:]):
        dt=(b['trace_us']-a['trace_us'])/1e6
        if not 0<dt<=.005:
            raise ValueError('Missing or nonmonotone high-rate sample')
        angles.append(direction*((a['angle_rad']+b['angle_rad'])/2-anchor))
        values.append(direction*(b['angle_rad']-a['angle_rad'])/dt)
        weights.append(dt)
    if len(values)<30 or not all(math.isfinite(v) for v in angles+values):
        raise ValueError('Insufficient finite interval data')
    return {'angles':angles,'values':values,'weights':[w/sum(weights) for w in weights]}


def basis(data,orders):
    return [[1]+[v for order in orders for v in (math.cos(order*a),math.sin(order*a))]
            for a in data['angles']]


def compare(datasets,orders):
    matrices=[basis(d,orders) for d in datasets]
    folds=[];fits=[]
    for which,data in enumerate(datasets):
        coefficients=solve_weighted(matrices[which],data['values'],data['weights'])
        if coefficients is None:raise ValueError('Degenerate per-run fit')
        fits.append({'log':data['log'],'Index_angle_rad':data['anchor'],
                     'within_run_R2':explained_variance(matrices[which],data['values'],data['weights'],coefficients),
                     'harmonics':[{'mechanical_order':order,'amplitude_rad_s':math.hypot(*coefficients[1+2*n:3+2*n]),
                                   'phase_rad':math.atan2(coefficients[2+2*n],coefficients[1+2*n])}
                                  for n,order in enumerate(orders)]})
        train=[n for n in range(len(datasets)) if n!=which]
        coefficients=solve_weighted([x for n in train for x in matrices[n]],
                                    [v for n in train for v in datasets[n]['values']],
                                    [w for n in train for w in datasets[n]['weights']])
        if coefficients is None:raise ValueError('Degenerate cross-run fit')
        folds.append({'held_out_log':data['log'],'training_logs':[datasets[n]['log'] for n in train],
                      'held_out_R2':explained_variance(matrices[which],data['values'],data['weights'],coefficients)})
    return {'mechanical_orders':orders,'per_run':fits,'leave_one_run_out':folds}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if len(args.logs)<3 or len(set(p.resolve() for p in args.logs))!=len(args.logs):
        raise ValueError('At least three distinct completed captures required')
    datasets=[];profile=None
    prefixes=('CONTROL ','CURRENT_LOOP ','CURRENT_ACTIVE ','CURRENT_TRACKING_ACTIVE ',
              'REFERENCE_RAMP ','ESTIMATOR ','EDGE_ESTIMATOR_CONFIG ')
    for path in args.logs:
        rows,lines=load(path)
        stats=parse_interval_velocity_stats(lines)
        if not stats or not stats['complete_run']:raise ValueError('Completed post-ramp statistics required')
        settings=[]
        for prefix in prefixes:
            matches=[l for l in lines if l.startswith(prefix)]
            if len(matches)!=1:raise ValueError('Unique actual control profile required')
            settings.append(matches[0])
        if profile is not None and settings!=profile:raise ValueError('Cross-run control profiles differ')
        profile=settings
        if rows[0]['trace_us']<stats['first_elapsed_us']:raise ValueError('Trace includes startup')
        reference=next(l for l in lines if l.startswith('RUN_REFERENCE '))
        direction=int(re.search(r'sensor_direction=(-?\d+)',reference)[1])
        anchor=index_angle(lines)
        datasets.append({'log':path.name,'anchor':anchor,**series(rows,anchor,direction)})
    result={'reference':'Recorded ABI Index, normalized modulo one mechanical revolution',
            'profile':profile,'models':[compare(datasets,orders) for orders in ([14],[28],[84],[14,28,84])],
            'note':'Exploratory fits of retained post-ramp tails. Training excludes the held-out capture. Index and speed use the same ABI sensor. Cross-run predictability does not identify sensor angle error, motor torque, ADC/drive distortion, or causation. No drive compensation is exported.'}
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    for model in result['models']:
        print(model['mechanical_orders'],[round(f['held_out_R2'],4) for f in model['leave_one_run_out']])


if __name__=='__main__':main()
