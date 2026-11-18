"""Offline motion audit. Every declared outbound/return/hold is retained."""
import argparse,csv,io,json,math,re,statistics
from pathlib import Path
from analyze_bench import analyze
from motion_profile import fields,parameters,reference,verify_motion,PROFILE

def analyze_motion(path):
    path=Path(path);lines=path.read_text(encoding='utf-8').splitlines()
    def one(prefix):
        found=[s for s in lines if s.startswith(prefix)]
        if len(found)!=1:raise ValueError(f'Expected one {prefix}, got {len(found)}')
        return found[0]
    result=analyze(path);meta=one('BENCH_MOTION ');cfg=fields(meta)
    rpm=int(cfg['rpm']);shape=int(cfg['shape']);direction=int(cfg['direction'])
    verify_motion(meta,rpm,shape,direction)
    active=fields(one('CURRENT_ACTIVE '))
    if active.get('velocity_LPF_s') not in (.001,.005) or any(abs(active.get(k,100)-v)>1e-7 for k,v in {'q_LPF_s':.001,'d_LPF_s':.001,'q_P':.6,'q_I':400}.items()):raise ValueError('Unexpected applied current/filter gains')
    iq_limit=result['profile']['iq_limit_nominal_A'];dq_limit=result['profile']['dq_limit_A']
    if (iq_limit,dq_limit) not in ((.30,.35),(.30,.50),(.45,.50)):raise ValueError('Unknown historical motion current range')
    expected=re.sub(r'iq_limit_nominal_A=\S+',f'iq_limit_nominal_A={iq_limit:.2f}'.replace('=0.','=.'),PROFILE)
    expected=re.sub(r'dq_limit_A=\S+',f'dq_limit_A={dq_limit:.2f}'.replace('=0.','=.'),expected)
    if one('BENCH_PROFILE ')!=expected:raise ValueError('Motion bounds disagree')
    move,ret,duration,limit=parameters(rpm,shape)
    first=next(i for i,s in enumerate(lines) if s.startswith('trace_us,'));last=lines.index('TRACE_END',first)
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(io.StringIO('\n'.join(lines[first:last])))]
    ref=fields(one('RUN_REFERENCE '));degree=math.pi/180
    command_tolerance=rpm*math.pi/30/8*.0005+2e-6
    if any(abs(v['joint_measured_rad']-ref['sensor_direction']*(v['angle_rad']-ref['start_angle_rad'])/8)>1e-6 for v in rows):raise ValueError('Joint measurement scaling disagrees')
    if any(abs(v['joint_target_rad']-reference(v['trace_us'],direction,rpm,shape)[0])>command_tolerance for v in rows):raise ValueError('Recorded reference differs from declared trajectory')
    if max(abs(v['joint_measured_rad'])/degree for v in rows)>6.0001:raise ValueError('Position hard boundary exceeded')
    whole=(rows[0]['trace_us']<1000 and duration<=rows[-1]['trace_us']<duration+2000 and
           max(b['trace_us']-a['trace_us'] for a,b in zip(rows,rows[1:]))<11000)
    stats=[fields(s) for s in lines if s.startswith('MOTION_STATS ')]
    if len(stats)!=2 or [s['phase'] for s in stats]!=[1,3]:raise ValueError('Missing/duplicate all-loop moving phase statistics')
    moving=[]
    for phase in (1,3):
        s=stats[0 if phase==1 else 1];n=s['samples']
        if n<=0:
            if whole:raise ValueError('Missing moving FOC samples in a complete run')
            moving.append({'phase':phase,'FOC_samples':0,'pass':False,'reason':'Not reached before protection stop'})
            continue
        observed=[r for r in rows if reference(r['trace_us'],direction,rpm,shape)[2]==phase]
        if len(observed)<85 and whole:raise ValueError('Moving trace truncated or too sparse')
        if any(s[k]<0 or s[k]>n for k in ('vector_limited','slew_limited','near_iq_limit')):raise ValueError('Invalid full-loop counts')
        if s['max_vector_V']>.500001 or s['error_square']<0 or s['iq_error_square']<0:raise ValueError('All-loop voltage/statistics invalid')
        m={'phase':phase,'FOC_samples':int(n),'trace_samples':len(observed),
           'max_error_joint_deg':s['max_error_joint_rad']/degree,'rms_error_joint_deg':math.sqrt(s['error_square']/n)/degree,
           'rms_iq_error_nominal_A':math.sqrt(s['iq_error_square']/n),
           'peak_motor_rpm':s['peak_velocity_motor_rad_s']*30/math.pi,
           'vector_limited_fraction':s['vector_limited']/n,'slew_limited_fraction':s['slew_limited']/n,'near_iq_limit_fraction':s['near_iq_limit']/n,
           'max_vector_V':s['max_vector_V'],'vbus_min_V':s['vbus_min_V'],'vbus_max_V':s['vbus_max_V']}
        m['pass']=(m['max_error_joint_deg']<=.5 and m['rms_error_joint_deg']<=.25 and
                   m['rms_iq_error_nominal_A']<=.05 and .8*rpm<=m['peak_motor_rpm']<=1.2*rpm)
        moving.append(m)
    holds=[]
    for start,end,goal,travel_sign in ((100000+move,ret,direction*math.radians(3.75),direction),(ret+move,duration+2000,0,-direction)):
        segment=[v for v in rows if start<=v['trace_us']<end]
        tail=[v for v in segment if v['trace_us']>=end-152000]
        h={'start_us':start,'samples':len(segment),'tail_samples':len(tail),'pass':False}
        if segment and tail:
            errors=[abs(v['joint_measured_rad']-goal)/degree for v in tail];positions=[v['joint_measured_rad']/degree for v in tail]
            overshoot=max(0,max(travel_sign*(v['joint_measured_rad']-goal)/degree for v in segment))
            settled=next((v['trace_us']-start for i,v in enumerate(segment) if all(abs(w['joint_measured_rad']-goal)/degree<=.2 for w in segment[i:])),None)
            fraction=statistics.mean(abs(v['iq_target_nominal_A'])>=iq_limit*.993 for v in tail)
            h.update(tail_max_error_joint_deg=max(errors),tail_peak_to_peak_joint_deg=max(positions)-min(positions),
                     overshoot_joint_deg=overshoot,settling_us=settled,tail_near_iq_limit_fraction=fraction)
            h['pass']=bool(len(tail)>=14 and max(errors)<=.2 and h['tail_peak_to_peak_joint_deg']<=.2 and
                           overshoot<=.3 and settled is not None and settled<=250000 and fraction<=.1)
        holds.append(h)
    result.update(motion=cfg,active_config=active,estimator=one('ESTIMATOR '),whole_trace=whole,all_loop_stats=stats,moving=moving,holds=holds,
                  scope='Same-ABI motor/8 tracking and repeatability; no external absolute-angle or loaded validation.')
    result['pass']=bool(whole and result['alignment_check_pass'] and result['guard']['fault']==0 and
                       result['stop']=='STOP: bench motion sequence complete' and all(m['pass'] for m in moving) and all(h['pass'] for h in holds))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('log');p.add_argument('--output',required=True);args=p.parse_args()
    result=analyze_motion(args.log)
    with Path(args.output).open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:result[k] for k in ('stop','pass','whole_trace','moving','holds')},indent=2))
