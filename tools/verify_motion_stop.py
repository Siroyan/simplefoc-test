"""Independently audit the physical live-stop log; does not drive hardware."""
import argparse,json,math
from pathlib import Path
from analyze_motion import analyze_motion
from motion_profile import fields,reference

def verify_stop(path):
    result=analyze_motion(path);lines=Path(path).read_text(encoding='utf-8').splitlines()
    required=('HOST live serial stop sent','HOST final_stop_acknowledged=True','HOST drive_lock_verified=True','HOST gate_off_verified=True','HOST verification_final_stop_acknowledged=True','Rejected: unavailable or already attempted; reset for another test.')
    if not all(v in lines for v in required) or result['stop']!='STOP: serial stop':raise ValueError('Missing real first serial stop, final acknowledgement or lock proof')
    elapsed=float(next(s for s in lines if s.startswith('RUN_SUMMARY elapsed_us,')).split(': ',1)[1].split(',')[0])
    detail=list(map(float,next(s for s in lines if s.startswith('STOP_DETAIL elapsed_us,')).split(': ',1)[1].split(',')))
    cfg=result['motion'];_,velocity,phase=reference(elapsed,int(cfg['direction']),int(cfg['rpm']),int(cfg['shape']))
    minimum=.5*cfg['rpm']*math.pi/30
    if phase not in (1,3) or abs(velocity*8)<minimum or abs(detail[1])<minimum:raise ValueError('Stop was outside verified fast motion')
    index=lines.index('HOST drive_lock_verified=True')
    pwm=fields(next(s for s in lines[index+1:] if s.startswith('PWM ')))
    if pwm.get('gate_inputs_low')!=1 or result['guard']['fault']!=0 or not result['alignment_check_pass']:raise ValueError('Gate/current/alignment evidence failed')
    result.update(kind='serial_stop_test',pass_scope='Live serial stop during fast motion, same-boot drive rejection and gates OFF only; not motion-performance qualification.',
                  pass_=True,stop_elapsed_us=elapsed,stop_feedback_motor_rpm=detail[1]*30/math.pi,stop_window_motor_rpm=detail[3]*30/math.pi)
    result['motion_performance_pass']=result['pass'];result['pass']=True
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('log');p.add_argument('--output',required=True);args=p.parse_args();result=verify_stop(args.log)
    with Path(args.output).open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:result[k] for k in ('kind','pass','stop_elapsed_us','stop_feedback_motor_rpm','stop_window_motor_rpm')}))
