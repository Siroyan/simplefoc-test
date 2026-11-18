"""Compare qualified low-energy coast and active-brake records offline."""
import argparse,json
from analyze_bench import analyze

def compare(coast,active):
    if coast['brake']['active']!=0 or active['brake']['active']!=1:raise ValueError('Expected coast then active')
    left={k:v for k,v in coast['brake'].items() if k!='active'}
    right={k:v for k,v in active['brake'].items() if k!='active'}
    if left!=right or coast['profile']!=active['profile']:raise ValueError('Multiple changed test factors')
    cv=coast['brake_status']['start_velocity_rad_s'];av=active['brake_status']['start_velocity_rad_s']
    cg=coast['brake_status']['start_guard_rad_s'];ag=active['brake_status']['start_guard_rad_s']
    speed_mismatch=abs(av-cv)/cv if cv>0 and av>0 else None
    guard_mismatch=abs(ag-cg)/cg if cg>0 and ag>0 else None
    c=coast.get('max_forward_excursion_20ms_rad');a=active.get('max_forward_excursion_20ms_rad')
    normalized_ratio=(a/av)/(c/cv) if c is not None and a is not None and c>0 and av>0 and cv>0 else None
    return {'coast':coast['log'],'active':active['log'],'speed_relative_difference':speed_mismatch,
            'guard_relative_difference':guard_mismatch,'normalized_forward_20ms_ratio':normalized_ratio,
            'opposition_samples':active['opposition_samples'],
            'pass':bool(coast['pass'] and active['pass'] and speed_mismatch is not None and guard_mismatch is not None and speed_mismatch<=.2 and guard_mismatch<=.2 and
                        normalized_ratio is not None and normalized_ratio<=.8),
            'scope':'Bare-motor low-energy comparison only; not a calibrated torque or loaded landing/regen qualification.'}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('coast');p.add_argument('active');p.add_argument('--output')
    args=p.parse_args();result=compare(analyze(args.coast),analyze(args.active));text=json.dumps(result,indent=2)
    if args.output:
        with open(args.output,'x',encoding='utf-8') as f:f.write(text+'\n')
    print(text)
