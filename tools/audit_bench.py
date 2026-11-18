"""Create a final, evidence-linked bench audit. No device access."""
import argparse,hashlib,json,re
from pathlib import Path
from analyze_bench import analyze
from compare_bench_brake import compare

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True);p.add_argument('--idle-log',required=True)
    p.add_argument('--coast-log',required=True);p.add_argument('--brake-log',required=True)
    p.add_argument('--stop-log',required=True);args=p.parse_args()
    root=Path(__file__).resolve().parents[1];records=[];joints=[];alignments=[]
    logdir=root/'docs/experiment-logs'
    paths=set(logdir.glob('2026-11-19-bench-*.log'))|set(logdir.glob('2026-11-19-range-*.log'))|set(logdir.glob('2026-11-19-brake-transition-*.log'))
    for path in sorted(paths):
        text=path.read_text(encoding='utf-8');lines=text.splitlines()
        if 'HOST sending one command=' not in text:continue
        if 'HOST final_stop_acknowledged=True' not in lines:raise ValueError(f'Unfinished capture: {path.name}')
        row={'log':str(path.relative_to(root)),'sha256':digest(path),'final_stop_acknowledged':True}
        row['firmware_version']=int(re.search(r'FIRMWARE diag_version=(\d+)',text)[1])
        if 'BENCH_PROFILE ' not in text:
            if 'SENSE_BEGIN: gates OFF' not in text:raise ValueError('Unknown non-bench capture')
            row.update(kind='route_error_no_drive',pass_=False)
        elif row['firmware_version']==124:
            row.update(kind='invalid_joint_current_telemetry',pass_=False,reason='Current/voltage union payload overwritten; source fixed in v125')
        elif row['firmware_version']==140 and 'BENCH_BRAKE active=0 ' in text:
            row.update(kind='invalid_brake_transition_voltage',pass_=False,reason='First transition row retained pre-disable Ud/target; actual gates were OFF; fixed in v141')
        else:
            result=analyze(path);row.update(result);row['log']=str(path.relative_to(root))
            kind=re.search(r'BENCH_PROFILE kind=(\w+)',text)[1];row['kind']=kind
            if result['alignment_check_pass']:alignments.append(row)
            if kind=='position' and 'HOST live serial stop sent' not in text:
                signature=next(v for v in lines if v.startswith('BENCH_JOINT '))
                # Direction is the intended one-factor change within a profile.
                signature=re.sub(r'direction=-?1 ', 'direction=both ',signature)
                joints.append((row['firmware_version'],signature,row))
        records.append(row)
    pulse_results=[analyze(root/f'docs/experiment-logs/2026-11-19-bench-pulse{amp}-v123-01.log')['pass'] for amp in ('05','10')]
    latest_signature=max(joints,key=lambda v:v[0])[1] if joints else None
    final_joints=[v[2] for v in joints if v[1]==latest_signature]
    counts={str(direction):sum(v['pass'] and int(v['joint']['direction'])==direction for v in final_joints) for direction in (-1,1)}
    joint_pass=bool(final_joints and all(v['pass'] for v in final_joints) and all(n>=3 for n in counts.values()))
    pair=compare(analyze(args.coast_log),analyze(args.brake_log))
    stop=Path(args.stop_log).read_text(encoding='utf-8')
    stop_pass=all(v in stop for v in ('HOST live serial stop sent','HOST final_stop_acknowledged=True','HOST drive_lock_verified=True','HOST gate_off_verified=True','TRACE_END'))
    stop_pass=stop_pass and next(v for v in stop.splitlines() if v.startswith('STOP: '))=='STOP: serial stop'
    idle=Path(args.idle_log).read_text(encoding='utf-8')
    if 'HOST verified: STOP acknowledged, drive locked until reset, five output-OFF rows' not in idle:raise ValueError('Final drive lock not verified')
    source=(root/'src/main.cpp').read_text(encoding='utf-8')
    version=int(re.search(r'kDiagVersion = (\d+)',source)[1])
    if f'FIRMWARE diag_version={version}' not in idle:raise ValueError('Source and final hardware version differ')
    goals={'G1_alignment':len(alignments)>=2,'G2_signed_current_response':all(pulse_results),
           'G3_joint_position_reproducibility':joint_pass,'G4_braking':pair['pass'],'G5_serial_stop_and_lock':stop_pass}
    files=['src/main.cpp','include/BenchJoint.h','include/BenchBrake.h','include/BenchCurrentPulse.h',
           'include/BenchCurrentGuard.h','include/TraceSample.h','include/TrackingCurrentPI.h','include/BoundedBLDCMotor.h',
           'tools/capture_test.py','tools/bench_metadata.py','tools/analyze_bench.py','tools/compare_bench_brake.py',
           '.pio/observer-build/disco_b_g431b_esc1/firmware.elf',args.idle_log,args.stop_log]
    result={'scope':'Standalone bare motor only; all current units are nominal, not calibrated torque.',
            'firmware_version':version,'supply_setting_V':11.1,'input_current_limit_A':.8,
            'pwm_Hz':12500,'cv_and_motor_temperature_measured':False,'goals':goals,
            'all_goals_pass':all(goals.values()),'final_position_signature':latest_signature,
            'final_position_pass_counts':counts,'braking_comparison':pair,'records':records,
            'final_files_sha256':{name:digest(root/name) for name in files}}
    with Path(args.output).open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps({k:result[k] for k in ('firmware_version','goals','all_goals_pass','final_position_pass_counts')},ensure_ascii=False))

if __name__=='__main__':main()
