"""Three specified runs per direction, counting performance failures without retrying faults."""
import argparse,json,re,subprocess,sys
from pathlib import Path
from analyze_motion import analyze_motion
from motion_profile import verify_live_candidate

def seed_counts(records):
    names=[str(Path(r['log']).resolve()) for r in records]
    if len(names)!=len(set(names)):raise ValueError('Duplicate seed capture')
    signatures=[];counts={1:0,-1:0}
    for r in records:
        if r['stop']!='STOP: bench motion sequence complete' or not r['whole_trace']:raise ValueError('Seed is not a complete normal position run')
        direction=int(r['motion']['direction']);counts[direction]+=1
        cfg={k:v for k,v in r['motion'].items() if k!='direction'}
        signatures.append((cfg,r['profile'],r['active_config'],r['estimator']))
    if signatures and any(v!=signatures[0] for v in signatures):raise ValueError('Seed profiles differ beyond direction')
    if any(n>3 for n in counts.values()):raise ValueError('More than three seeds per direction')
    return counts

def main():
    p=argparse.ArgumentParser();p.add_argument('--version',type=int,required=True);p.add_argument('--rpm',type=int,required=True)
    p.add_argument('--shape',type=int,choices=(0,1),required=True);p.add_argument('--positive-done',type=int,default=0);p.add_argument('--negative-done',type=int,default=0)
    p.add_argument('--output-prefix',required=True);p.add_argument('--velocity-p',type=float,default=.12)
    p.add_argument('--seed-log',type=Path,action='append',default=[]);args=p.parse_args()
    if not all(0<=n<=3 for n in (args.positive_done,args.negative_done)):raise ValueError('At most three per direction')
    verify_live_candidate(args.rpm,args.shape,args.velocity_p)
    root=Path(__file__).resolve().parents[1];rows=[];failure=None
    def path(direction,n):return root/f'docs/experiment-logs/{args.output_prefix}-{"positive" if direction==1 else "negative"}-v{args.version}-{n:02d}.log'
    def inspect(log):
        result=analyze_motion(log);text=log.read_text(encoding='utf-8')
        if int(re.search(r'FIRMWARE diag_version=(\d+)',text)[1])!=args.version or int(result['motion']['rpm'])!=args.rpm or int(result['motion']['shape'])!=args.shape:raise ValueError('Unexpected firmware/trajectory')
        out=log.with_suffix('.json')
        if not out.exists():
            with out.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
        rows.append(result)
        print(json.dumps({'log':log.name,'stop':result['stop'],'performance_pass':result['pass'],'max_moving_error_joint_deg':max((m.get('max_error_joint_deg',0) for m in result['moving']),default=0)}),flush=True)
        if result['stop']!='STOP: bench motion sequence complete':raise RuntimeError('Protective stop: cancel remaining identical repetitions')
    try:
        if args.seed_log:
            if args.positive_done or args.negative_done:raise ValueError('Use explicit seeds or numbered completed counts, not both')
            for log in args.seed_log:inspect(log)
            counts=seed_counts(rows);args.positive_done=counts[1];args.negative_done=counts[-1]
        else:
            for direction,done in ((1,args.positive_done),(-1,args.negative_done)):
                for n in range(1,done+1):inspect(path(direction,n))
        # Alternate directions; completed records are never overwritten/retried.
        for n in range(1,4):
            for direction,done in ((1,args.positive_done),(-1,args.negative_done)):
                if n<=done:continue
                log=path(direction,n)
                if log.exists():raise FileExistsError('Next log already exists; no reset or retry')
                reset=root/f'.pio/reset-{args.output_prefix}-v{args.version}-{direction}-{n:02d}.log'
                subprocess.run([sys.executable,str(root/'tools/reset_board.py'),'--output',str(reset)],check=True,capture_output=True,text=True)
                with (root/'.pio/motion-repetition-latest.txt').open('w',encoding='utf-8') as output:
                    result=subprocess.run([sys.executable,str(root/'tools/capture_motion.py'),'--rpm',str(args.rpm),'--shape',str(args.shape),'--direction',str(direction),'--velocity-p',str(args.velocity_p),'--output',str(log)],stdout=output,stderr=subprocess.STDOUT)
                if result.returncode!=0:raise RuntimeError('Capture failed: inspect the exact log/process; no reset/retry')
                inspect(log)
    except Exception as exc:failure=str(exc);print(f'CANCELLED remaining repetitions: {failure}',flush=True)
    finally:
        idle=root/f'docs/experiment-logs/{args.output_prefix}-v{args.version}-repetitions-idle-01.log'
        subprocess.run([sys.executable,str(root/'tools/lock_idle.py'),'--port','COM10','--output',str(idle)],check=True,capture_output=True,text=True)
        summary=root/f'docs/experiment-logs/{args.output_prefix}-v{args.version}-repetitions.json'
        data={'version':args.version,'rpm':args.rpm,'shape':args.shape,'records':rows,'failure':failure,'all_six_normally_completed':len(rows)==6 and failure is None,
              'all_six_performance_pass':len(rows)==6 and failure is None and all(r['pass'] for r in rows),'final_idle_log':str(idle.relative_to(root))}
        with summary.open('x',encoding='utf-8') as f:json.dump(data,f,indent=2)
        print(json.dumps({k:data[k] for k in ('all_six_normally_completed','all_six_performance_pass','failure')}),flush=True)
    if failure:sys.exit(1)

if __name__=='__main__':main()
