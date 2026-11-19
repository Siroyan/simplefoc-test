"""Evidence summary of all motion attempts; no device access or success filtering."""
import argparse,csv,hashlib,json,re,zipfile
from pathlib import Path
from analyze_motion import analyze_motion
from analyze_bench import analyze
from verify_motion_stop import verify_stop

def recovered_preflight(root,path,entry):
    text=path.read_text(encoding='utf-8');recovery=root/entry['recovery_log'];off=recovery.read_text(encoding='utf-8')
    if ('RUN BENCH MOTION:' in text or 'BENCH_STARTUP first_pi_elapsed_us=0' not in text or
        'initial_pull_complete=0' not in text or 'BENCH_CURRENT_STATUS fault=0 ' not in text):raise ValueError('Preflight exception does not prove absence of a position run')
    rows=[s.split(',') for s in off.splitlines() if re.match(r'^\d+,0,',s)]
    if (len(rows)<5 or any(abs(float(row[-1]))>1e-8 for row in rows) or 'STOP: serial stop' not in off.splitlines() or
        'gate_inputs_low=1' not in off or 'HOST verified: STOP acknowledged, drive locked until reset, five output-OFF rows' not in off):raise ValueError('Preflight stop recovery not verified')
    return {'kind':'host_preflight_abort','pass':False,'whole_trace':False,'stop':'host metadata rejected before position run',
            'reason':entry['reason'],'position_motion_started':False,'final_stop_acknowledged':False,
            'recovery_stop_acknowledged':True,'recovery_log':entry['recovery_log'],'recovery_sha256':digest(recovery)}

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def verify_condition(root,data):
    relative=Path(data['snapshot_directory'].replace('\\','/'))
    directory=root/relative
    archive=root/'docs/experiment-logs/2026-11-19-motion-condition-archives.zip'
    if directory.exists():
        for name,h in data['sha256'].items():
            if digest(directory/Path(name).name)!=h:raise ValueError('Archived condition hash mismatch')
    else:
        candidates=sorted((root/'docs/experiment-logs').glob('20??-??-??-motion-condition-archives.zip'))
        for archive in candidates:
            with zipfile.ZipFile(archive) as z:
                keys={name:str(relative.relative_to('.pio')/Path(name.replace('\\','/')).name).replace('\\','/') for name in data['sha256']}
                if not all(key in z.namelist() for key in keys.values()):continue
                for name,h in data['sha256'].items():
                    if hashlib.sha256(z.read(keys[name])).hexdigest()!=h:raise ValueError('Portable condition archive hash mismatch')
                return
        raise ValueError('Missing condition snapshot and portable archive')

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--csv',required=True);p.add_argument('--date');args=p.parse_args()
    root=Path(__file__).resolve().parents[1];records=[]
    exceptions={}
    for registry in (root/'docs/experiment-logs').glob('20??-??-??-motion-preflight-aborts.json'):
        exceptions.update({v['log']:v for v in json.loads(registry.read_text(encoding='utf-8'))['records']})
    for path in sorted((root/'docs/experiment-logs').glob(f'{args.date or "20??-??-??"}-motion-*.log')):
        text=path.read_text(encoding='utf-8')
        if 'HOST sending one command=' not in text:continue
        key=str(path.relative_to(root)).replace('\\','/')
        if key in exceptions:result=recovered_preflight(root,path,exceptions[key])
        else:
            if 'HOST final_stop_acknowledged=True' not in text.splitlines():raise ValueError(f'Unfinished motion capture: {path.name}')
            pulse='BENCH_PROFILE kind=current_pulse ' in text
            stop_test='HOST live serial stop sent' in text
            result=verify_stop(path) if stop_test else analyze(path) if pulse else analyze_motion(path)
            result['kind']='serial_stop_test' if stop_test else 'current_pulse' if pulse else 'motion';result['final_stop_acknowledged']=True
        result['log']=str(path.relative_to(root));result['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
        result['firmware_version']=int(re.search(r'FIRMWARE diag_version=(\d+)',text)[1])
        if result['kind']=='motion':result['profile_text']=next(s for s in text.splitlines() if s.startswith('BENCH_MOTION '))
        conditions=list((root/'docs/experiment-logs').glob(f"20??-??-??-motion-v{result['firmware_version']}-condition.json"))
        if len(conditions)!=1:raise ValueError('Missing/ambiguous firmware condition')
        condition=conditions[0]
        condition_data=json.loads(condition.read_text(encoding='utf-8'));verify_condition(root,condition_data)
        result['condition']=str(condition.relative_to(root))
        records.append(result)
    groups={}
    for result in records:
        if result['kind']!='motion':continue
        signature=re.sub(r'direction=-?1 ', 'direction=both ',result['profile_text'])+'|'+json.dumps(result['active_config'],sort_keys=True)+'|'+result['estimator']
        group=groups.setdefault(signature,{'normal_complete':{'1':0,'-1':0},'performance_pass':{'1':0,'-1':0},'protective_stop':{'1':0,'-1':0},'logs':[]})
        direction=str(int(result['motion']['direction']));normal=result['stop']=='STOP: bench motion sequence complete'
        group['normal_complete'][direction]+=int(normal);group['performance_pass'][direction]+=int(result['pass'])
        group['protective_stop'][direction]+=int(not normal and result['stop']!='STOP: serial stop');group['logs'].append(result['log'])
    data={'scope':'All saved standalone motor motion attempts, including failures; no goal-completion inference.',
          'supply_V':11.1,'input_current_limit_A':.8,'records':records,'groups':groups}
    with Path(args.output).open('x',encoding='utf-8') as f:json.dump(data,f,indent=2)
    columns=['firmware','kind','rpm','shape','direction','stop','whole_trace','performance_pass','max_moving_error_joint_deg','max_moving_rms_joint_deg','max_moving_iq_rms_nominal_A','max_hold_error_joint_deg','max_peak_motor_rpm','final_stop_acknowledged','recovery_stop_acknowledged','log']
    with Path(args.csv).open('x',encoding='utf-8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=columns);w.writeheader()
        for r in records:
            moving=[v for v in r.get('moving',[]) if v['FOC_samples']];holds=[v for v in r.get('holds',[]) if v['tail_samples']];motion=r.get('motion',{})
            w.writerow(dict(firmware=r['firmware_version'],kind=r['kind'],rpm=motion.get('rpm'),shape=motion.get('shape'),direction=motion.get('direction'),stop=r['stop'],whole_trace=r.get('whole_trace'),performance_pass=r['pass'],final_stop_acknowledged=r['final_stop_acknowledged'],recovery_stop_acknowledged=r.get('recovery_stop_acknowledged'),
                max_moving_error_joint_deg=max((v['max_error_joint_deg'] for v in moving),default=None),max_moving_rms_joint_deg=max((v['rms_error_joint_deg'] for v in moving),default=None),
                max_moving_iq_rms_nominal_A=max((v['rms_iq_error_nominal_A'] for v in moving),default=None),max_hold_error_joint_deg=max((v['tail_max_error_joint_deg'] for v in holds),default=None),
                max_peak_motor_rpm=max((v['peak_motor_rpm'] for v in moving),default=None),log=r['log']))
    print(json.dumps({'records':len(records),'normal_motion_complete':sum(v['stop']=='STOP: bench motion sequence complete' for v in records),'motion_performance_pass':sum(v['pass'] for v in records if v['kind']=='motion'),'pulse_pass':sum(v['pass'] for v in records if v['kind']=='current_pulse'),'recovered_preflight_aborts':sum(v['kind']=='host_preflight_abort' for v in records),'all_final_or_recovery_stop_acknowledged':all(v['final_stop_acknowledged'] or v.get('recovery_stop_acknowledged') for v in records)}))

if __name__=='__main__':main()
