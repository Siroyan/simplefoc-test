"""Audit every reached phase of a high-speed free-motor capture, including failures."""
import argparse,csv,io,json,math,statistics
from pathlib import Path
from analyze_bench import analyze
from fast_motion_profile import fields,parameters,reference,verify_motion,electrical_profile
from motion_profile import PROFILE
from bench_metadata import verify_bench_current_loop

def analyze_fast(path):
    path=Path(path);lines=path.read_text(encoding='utf-8').splitlines()
    def one(prefix):
        rows=[v for v in lines if v.startswith(prefix)]
        if len(rows)!=1:raise ValueError('Expected exactly one '+prefix)
        return rows[0]
    vector_limit=fields(one('BENCH_PROFILE '))['vector_limit_V']
    if vector_limit not in (.5,.7,1.):raise ValueError('Unknown fast voltage ceiling')
    iq_limit=fields(one('BENCH_PROFILE '))['iq_limit_nominal_A']
    expected_profile=electrical_profile(vector_limit,iq_limit)
    if one('BENCH_PROFILE ')!=expected_profile:raise ValueError('Electrical limits changed')
    if iq_limit==.4 and fields(one('FAST_CURRENT_ACTIVE '))!={'iq_limit_nominal_A':.4}:raise ValueError('Missing/inconsistent applied current command limit')
    if vector_limit>.5:
        active_voltage=fields(one('FAST_VOLTAGE_ACTIVE '))
        if set(active_voltage)!={'vector_limit_V','q_integrator_limit_V','d_integrator_limit_V','q_slew_limit_V','d_slew_limit_V'} or any(abs(v-vector_limit)>1e-7 for v in active_voltage.values()):raise ValueError('Missing/inconsistent applied voltage ceilings')
    result=analyze(path,vector_limit)
    cfg=fields(one('BENCH_MOTION '));rpm=int(cfg['rpm']);direction=int(cfg['direction']);p=parameters(rpm)
    verify_motion(one('BENCH_MOTION '),rpm,2,direction)
    active=fields(one('CURRENT_ACTIVE '))
    q_lpf=active.get('q_LPF_s')
    if q_lpf not in (.001,.0005):raise ValueError('Unreviewed q current filter')
    verify_bench_current_loop(one('CURRENT_LOOP '),vector_limit,q_lpf)
    for k,v in {'q_LPF_s':q_lpf,'d_LPF_s':.001,'velocity_LPF_s':.001,'q_P':.6,'q_I':400}.items():
        if abs(active.get(k,100)-v)>1e-7:raise ValueError('Unexpected applied controller: '+k)
    if one('CURRENT_TRACKING_ACTIVE ')!='CURRENT_TRACKING_ACTIVE q_backcalc_s=0.0050 d_backcalc_s=0.0000':raise ValueError('Current tracking changed')
    if one('ESTIMATOR ')!='ESTIMATOR controller=edge_period4_phase guard=original_window2ms_LPF2ms guard_limit=BENCH_MOTION':raise ValueError('Unexpected estimator')
    status=fields(one('FAST_MOTION_STATUS '));start=fields(one('RUN_REFERENCE '))
    first=next(i for i,s in enumerate(lines) if s.startswith('trace_us,'));last=lines.index('TRACE_END',first)
    rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(io.StringIO('\n'.join(lines[first:last])))]
    tolerance=p['peak_motor_rad_s']/8*.00025+2e-6
    inside_position=True
    for r in rows:
        command_limit=cfg.get('velocity_command_limit_rad_s',p['peak_motor_rad_s'])/8
        if abs(r['joint_velocity_target_rad_s'])>command_limit+2e-6:raise ValueError('Saved velocity command exceeds declared cap')
        if abs(r['joint_measured_rad']-start['sensor_direction']*(r['angle_rad']-start['start_angle_rad'])/8)>2e-6:raise ValueError('Angle/ABI scaling mismatch')
        if abs(r['joint_target_rad']-reference(r['trace_us'],direction,rpm)[0])>tolerance:raise ValueError('Reference/timestamp mismatch')
        inside_position &= -5.001<=direction*math.degrees(r['joint_measured_rad'])<=p['position_limit_joint_deg']+.001
    if not status['fault'] and status['max_control_gap_us']>1000:raise ValueError('Unreported controller timing fault')
    whole=rows[0]['trace_us']<1000 and p['duration_us']<=rows[-1]['trace_us']<p['duration_us']+2000
    if any(b['trace_us']-a['trace_us']>max(10000,p['move_us']//160)+1100 for a,b in zip(rows,rows[1:])):raise ValueError('Trace sampling gap')
    moving=[]
    for s in (fields(v) for v in lines if v.startswith('MOTION_STATS ')):
        n=int(s['samples']);m={'phase':int(s['phase']),'FOC_samples':n}
        if n:m.update(max_error_joint_deg=math.degrees(s['max_error_joint_rad']),rms_error_joint_deg=math.degrees(math.sqrt(s['error_square']/n)),
            rms_iq_error_nominal_A=math.sqrt(s['iq_error_square']/n),peak_motor_rpm=s['peak_velocity_motor_rad_s']*30/math.pi,
            vector_limited_fraction=s['vector_limited']/n,slew_limited_fraction=s['slew_limited']/n,near_iq_limit_fraction=s['near_iq_limit']/n,
            max_vector_V=s['max_vector_V'],vbus_min_V=s['vbus_min_V'],vbus_max_V=s['vbus_max_V'])
        moving.append(m)
    if [m['phase'] for m in moving]!=[1,3]:raise ValueError('Missing phase statistics')
    takeoff=100000+p['move_us'];window=max(10000,min(40000,3*(p['move_us']//160)))
    near=[r for r in rows if takeoff-window<=r['trace_us']<takeoff]
    takeoff_result={'samples':len(near),'window_us':window,'reached':bool(near)}
    if near:
        actual=statistics.mean(r['joint_velocity_rad_s']*8*direction for r in near)
        expected=statistics.mean(reference(r['trace_us'],direction,rpm)[1]*8*direction for r in near)
        takeoff_result.update(mean_motor_rpm=actual*30/math.pi,reference_mean_motor_rpm=expected*30/math.pi,
            achieved_fraction=actual/expected,max_error_joint_deg=max(abs(r['joint_target_rad']-r['joint_measured_rad'])*180/math.pi for r in near))
    end=p['duration_us'];tail=[r for r in rows if end-150000<=r['trace_us']<=end+2000]
    hold={'samples':len(tail),'reached':bool(tail)}
    if tail:
        q=[math.degrees(r['joint_measured_rad'])*direction for r in tail]
        hold.update(max_error_joint_deg=max(abs(v-p['final_joint_deg']) for v in q),peak_to_peak_joint_deg=max(q)-min(q),
            max_abs_motor_rpm=max(abs(r['joint_velocity_rad_s'])*8*30/math.pi for r in tail))
    complete=bool(whole and inside_position and result['stop']=='STOP: bench motion sequence complete' and result['guard']['fault']==0 and status['fault']==0 and result['alignment_check_pass'])
    advance=bool(complete and status['max_control_gap_us']<=800 and takeoff_result.get('achieved_fraction',0)>=.9 and
        takeoff_result.get('achieved_fraction',2)<=1.1 and takeoff_result.get('max_error_joint_deg',100)<=1 and
        all(m['FOC_samples'] and m['vector_limited_fraction']<=.05 for m in moving) and
        hold.get('max_error_joint_deg',100)<=.5 and hold.get('peak_to_peak_joint_deg',100)<=.3)
    result.update(kind='fast_motion',motion=cfg,voltage_limit_V=vector_limit,whole_trace=whole,normal_complete=complete,advance_candidate=advance,moving=moving,takeoff=takeoff_result,hold=hold,control=status,inside_position_envelope=inside_position,
        scope='Bare motor acceleration/braking with same-ABI motor/8 angle and nominal currents; not loaded takeoff or jump validation.')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('log');p.add_argument('--output',required=True);args=p.parse_args();result=analyze_fast(args.log)
    with Path(args.output).open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:result[k] for k in ('stop','normal_complete','advance_candidate','moving','takeoff','hold','control')},indent=2))
