"""One bounded bare-motor motion capture. No automatic reset, upload or retry."""
import argparse,ast,hashlib,re,sys,time
from datetime import datetime
from pathlib import Path
# Content-address local helper bytecode: copy2 can preserve same-second mtimes
# and equal file sizes across reviewed range changes. Never trust that cache.
_HOST_SOURCE_HASH=hashlib.sha256(b''.join(p.read_bytes() for p in sorted(Path(__file__).parent.glob('*.py')))).hexdigest()
sys.pycache_prefix=str(Path(__file__).resolve().parents[1]/'.pio/python-content-cache'/_HOST_SOURCE_HASH)
import serial
from serial.tools import list_ports
import motion_profile
from bench_metadata import verify_bench_current_loop
from motion_profile import PROFILE,verify_motion,fields,reference

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default='COM10');p.add_argument('--rpm',type=int,choices=(10,20,40,60),required=True)
    p.add_argument('--shape',type=int,choices=(0,1),default=0);p.add_argument('--direction',type=int,choices=(-1,1),required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--stop-after-ms',type=int)
    p.add_argument('--velocity-p',type=float,choices=(.05,.12),default=.12)
    p.add_argument('--velocity-i',type=float,choices=(1.2,3.0),default=1.2)
    p.add_argument('--velocity-lpf',type=float,choices=(.001,.005),default=.001)
    p.add_argument('--estimator',choices=('edge','rolling'),default='edge')
    p.add_argument('--velocity-limit',choices=('1.25peak_plus_.5motor','peak_motor'),default='peak_motor')
    args=p.parse_args();command=b'"' if args.direction==1 else b'|'
    source=ast.parse(Path(motion_profile.__file__).read_text(encoding='utf-8'))
    declared=next(ast.literal_eval(n.value) for n in source.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='PROFILE' for t in n.targets))
    if PROFILE!=declared:raise RuntimeError('Host source/cache profile mismatch; serial port not opened')
    ports=[v for v in list_ports.comports() if v.device==args.port and v.vid==0x0483 and v.pid==0x374b]
    if len(ports)!=1:raise RuntimeError('Expected ST-LINK VCP missing; no reset/retry')
    with args.output.open('x',encoding='utf-8',buffering=1) as log:
        def record(s):
            log.write(s+'\n')
            if not re.match(r'^\d',s) and not s.startswith('MOTION_STATS '):print(s,flush=True)
        with serial.Serial(args.port,115200,timeout=.05,write_timeout=1) as link:
            buffer=b''
            def read():
                nonlocal buffer
                buffer+=link.read(max(1,link.in_waiting));out=[]
                while b'\n' in buffer:
                    row,buffer=buffer.split(b'\n',1);s=row.decode('ascii',errors='strict').strip('\r')
                    if s:record(s);out.append(s)
                return out
            def send(b):link.write(b);link.flush()
            def confirmed_stop(label,message):
                send(b's');record(message);ack=False;reissued=False;end=time.monotonic()+2
                while time.monotonic()<end:
                    replies=read()
                    if 'STOP: serial stop' in replies:ack=True;break
                    if not reissued and any(s.startswith('STOP: ') for s in replies):
                        send(b's');reissued=True;record('HOST repeated s after an abort STOP for explicit serial acknowledgement')
                record(f'HOST {label}={ack}')
                if not ack:raise RuntimeError('Stop acknowledgement missing')
            record(f'HOST start={datetime.now().astimezone().isoformat()} port={args.port}')
            record(f'HOST python_source_fingerprint={_HOST_SOURCE_HASH}')
            record('HOST supply=11.1V current_limit=0.8A; nominal phase/Iq limits are separate; CV and temperature unmeasured')
            sent=False
            try:
                # Confirm live output-OFF rows, then physical gate and PWM registers.
                end=time.monotonic()+3;off=0
                while time.monotonic()<end and off<3:
                    for s in read():
                        if re.match(r'^\d+,0,',s) and abs(float(s.split(',')[-1]))<1e-8:off+=1
                if off<3:raise RuntimeError('No live disabled telemetry; drive not started')
                send(b'c');end=time.monotonic()+2;pwm=False
                while time.monotonic()<end and not pwm:
                    for s in read():
                        if s.startswith('PWM '):
                            v=fields(s)
                            if any(v.get(k)!=x for k,x in {'hz':12500,'system_clock_Hz':170000000,'tim1_psc':0,'tim1_arr':6799,'gate_inputs_low':1,'rcc_cfgr':15}.items()) or int(v.get('tim1_bdtr',0))&255!=0x84 or int(v.get('tim1_cr1',0))&0x360!=0x40:raise RuntimeError('PWM/gate invariant mismatch')
                            pwm=True
                if not pwm:raise RuntimeError('Missing PWM confirmation')
                record(f'HOST sending one command={command.decode()} rpm={args.rpm} shape={args.shape} direction={args.direction}')
                send(command);sent=True
                verified=set();end=time.monotonic()+32;trace=False;stopped=False;off=0;stop_deadline=None;live_stop=False
                first_stop_reason=None;stop_elapsed=None;stop_velocity=None
                while time.monotonic()<end:
                    if stop_deadline is not None and not live_stop and not stopped and time.monotonic()>=stop_deadline:
                        send(b's');live_stop=True;record('HOST live serial stop sent')
                    for s in read():
                        if s.startswith('BENCH_PROFILE '):
                            if s!=PROFILE:raise RuntimeError('Motion current/protection profile mismatch')
                            verified.add('profile')
                        if s.startswith('CURRENT_CONFIG '):
                            v=fields(s)
                            if any(v.get(k)!=x for k,x in {'observer_only':0,'deviation_delay_us':20000,'max_age_us':2000,'nominal_ADC_ref_V':3.3,'shunt_ohm':.003}.items()) or not 10<v.get('bus_low_V',0)<10.9 or not 11.5<v.get('bus_high_V',0)<12.5:raise RuntimeError('ADC/Vbus guard mismatch')
                            verified.add('observer')
                        if s.startswith('BENCH_MOTION '):verify_motion(s,args.rpm,args.shape,args.direction,args.velocity_p,args.velocity_i,args.velocity_limit);verified.add('motion')
                        if s.startswith('ESTIMATOR '):
                            expected='ESTIMATOR controller=edge_period4_phase guard=original_window2ms_LPF2ms guard_limit=BENCH_MOTION' if args.estimator=='edge' else 'ESTIMATOR controller=rolling_LS4ms min_sample_us=150 warmup_span_us=3000 guard=original_window2ms_LPF2ms guard_limit=BENCH_MOTION'
                            if s!=expected:raise RuntimeError('Independent/controller velocity estimators mismatch')
                            verified.add('estimator')
                        if s.startswith('EDGE_CLOCK_CHECK '):
                            v=fields(s)
                            if any(v.get(k)!=x for k,x in {'advancing':1,'gates_off':1,'cycles_per_us':170}.items()):raise RuntimeError('DWT disabled clock check failed')
                            verified.add('edgeclock')
                        if s.startswith('EDGE_ESTIMATOR_CONFIG '):
                            if s!='EDGE_ESTIMATOR_CONFIG counts_per_turn=4000 period_intervals=4 phase_gain_s_inv=200 prediction_count_bound=1 sample_gap_limit_us=5000 no_edge_decay=gap_to_2gap isr=integer_only clock=DWT_elapsed cycles_per_us=170':raise RuntimeError('Edge clock/estimator configuration mismatch')
                            verified.add('edgeconfig')
                        if s.startswith('CURRENT_LOOP '):verify_bench_current_loop(s);verified.add('loop')
                        if s.startswith('CURRENT_TRACKING_ACTIVE '):
                            if s!='CURRENT_TRACKING_ACTIVE q_backcalc_s=0.0050 d_backcalc_s=0.0000':raise RuntimeError('Current tracking mismatch')
                            verified.add('tracking')
                        if s.startswith('CURRENT_ACTIVE '):
                            v=fields(s)
                            if any(abs(v.get(k,100)-x)>1e-7 for k,x in {'q_LPF_s':.001,'d_LPF_s':.001,'velocity_LPF_s':args.velocity_lpf,'q_P':.6,'q_I':400}.items()):raise RuntimeError('Applied current/velocity configuration mismatch')
                            verified.add('active')
                        if s.startswith('RUN BENCH MOTION:'):
                            required={'profile','motion','loop','tracking','active','observer','estimator'}|({'edgeclock','edgeconfig'} if args.estimator=='edge' else set())
                            if verified!=required:raise RuntimeError('Motion started without complete verified metadata')
                            if args.stop_after_ms is not None:stop_deadline=time.monotonic()+args.stop_after_ms/1000
                        if s.startswith('STOP: '):
                            stopped=True
                            if first_stop_reason is None:first_stop_reason=s
                        if s.startswith('RUN_SUMMARY elapsed_us,displacement_rad:') and stop_elapsed is None:
                            stop_elapsed=float(s.split(': ',1)[1].split(',')[0])
                        if s.startswith('STOP_DETAIL elapsed_us,filtered_rad_s,raw_rad_s,window10ms_rad_s:') and stop_velocity is None:
                            stop_velocity=float(s.split(': ',1)[1].split(',')[1])
                        if s=='TRACE_END':trace=True
                        if stopped and re.match(r'^\d+,0,',s) and abs(float(s.split(',')[-1]))<1e-8:off+=1
                    if stopped and trace and off>=3:break
                else:raise RuntimeError('Missing STOP/whole trace/disabled telemetry; no retry')
                record('HOST completed: STOP + TRACE_END + disabled telemetry confirmed')
            finally:
                confirmed_stop('final_stop_acknowledged',f'HOST sent final s; test_command_sent={sent}')
            if args.stop_after_ms is not None:
                try:
                    if not live_stop or first_stop_reason!='STOP: serial stop':raise RuntimeError('Protection stopped before the requested live serial stop')
                    if stop_elapsed is None or stop_velocity is None:raise RuntimeError('Missing live-stop time/velocity')
                    _,v,phase=reference(stop_elapsed,args.direction,args.rpm,args.shape)
                    if phase not in (1,3) or abs(v*8)<.5*args.rpm*3.141592653589793/30 or abs(stop_velocity)<.5*args.rpm*3.141592653589793/30:
                        raise RuntimeError('Serial stop did not occur during verified fast motion')
                    record(f'HOST stop_during_motion_verified=True elapsed_us={stop_elapsed:g} feedback_motor_rad_s={stop_velocity:g}')
                    send(command);end=time.monotonic()+2;rejected=False
                    while time.monotonic()<end and not rejected:
                        for s in read():
                            if s=='Rejected: unavailable or already attempted; reset for another test.':rejected=True
                            if s.startswith(('ALIGNING:','RUN BENCH')):raise RuntimeError('Same-boot drive lock failed')
                    record(f'HOST drive_lock_verified={rejected}')
                    if not rejected:raise RuntimeError('Missing drive lock rejection')
                    send(b'c');end=time.monotonic()+2;gate=False
                    while time.monotonic()<end and not gate:
                        for s in read():
                            if s.startswith('PWM '):gate=fields(s).get('gate_inputs_low')==1
                    record(f'HOST gate_off_verified={gate}')
                    if not gate:raise RuntimeError('Missing gate OFF confirmation')
                finally:
                    confirmed_stop('verification_final_stop_acknowledged','HOST sent final verification s')

if __name__=='__main__':main()
