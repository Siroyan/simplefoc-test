"""Offline validation of standalone-motor alignment and short current captures."""
import argparse,csv,io,json,math,re,statistics
from pathlib import Path

def fields(line):
    return {k:float(v) for k,v in re.findall(r'([A-Za-z_][A-Za-z_0-9]*)=(-?[0-9.]+)',line)}

def max_forward_excursion(rows):
    if not rows:return None
    base=rows[0];end=base['trace_us']+20000
    if rows[-1]['trace_us']<end:return None
    return max(0,max(v['angle_rad'] for v in rows if v['trace_us']<=end)-base['angle_rad'])

def analyze(path):
    lines=Path(path).read_text(encoding='utf-8').splitlines()
    def one(prefix):
        found=[v for v in lines if v.startswith(prefix)]
        if len(found)!=1:raise ValueError(f'Expected one {prefix}, got {len(found)}')
        return found[0]
    if 'HOST final_stop_acknowledged=True' not in lines or 'TRACE_END' not in lines:
        raise ValueError('Incomplete trace or final stop acknowledgement')
    if any(v=='FIRMWARE diag_version=124' for v in lines):raise ValueError('v124 current trace overwritten by joint payload; invalid telemetry')
    profile=fields(one('BENCH_PROFILE '));alignment=fields(one('BENCH_ALIGNMENT '))
    guard=fields(one('BENCH_CURRENT_STATUS '))
    stopped=next(v for v in lines if v.startswith('STOP: '))
    forward,reverse=map(float,one('ALIGN measured radians forward/reverse: ').split(': ')[1].split('/'))
    quality=fields(one('ABI_QUALITY '))
    if quality['invalid'] or quality['duplicate']:raise ValueError('ABI quality failure')
    g1=(alignment['initial_pull_complete']==1 and
        all(abs(abs(a)*7-6*math.pi)<.5 for a in (forward,reverse)) and forward*reverse>0)
    result={'analysis_version':2,'log':str(path),'stop':stopped,'profile':profile,'alignment':alignment,
            'alignment_joint_degrees':{k:alignment[k]*180/math.pi/8 for k in
                ('initial_pull_motor_rad','minimum_delta_motor_rad','maximum_delta_motor_rad')},
            'alignment_check_pass':g1,'guard':guard,'abi':quality}
    if 'kind=alignment' in one('BENCH_PROFILE '):
        begin=next(i for i,v in enumerate(lines) if v.startswith('align_us,'))
        end=lines.index('TRACE_END',begin)
        rows=list(csv.DictReader(io.StringIO('\n'.join(lines[begin:end]))))
        expected=int(fields(one('ALIGN_TRACE_CONFIG '))['expected_samples'])
        if len(rows)!=expected:raise ValueError('Alignment trace truncated')
        result['alignment_samples']=len(rows)
        result['pass']=g1 and guard['fault']==0 and stopped=='STOP: alignment survey complete'
        return result
    begin=next(i for i,v in enumerate(lines) if v.startswith('trace_us,'))
    end=lines.index('TRACE_END',begin)
    rows=[{k:float(v) for k,v in row.items()} for row in csv.DictReader(io.StringIO('\n'.join(lines[begin:end])))]
    if not rows or any(not math.isfinite(v) for row in rows for v in row.values()):
        raise ValueError('Missing/nonfinite trace')
    times=[v['trace_us'] for v in rows]
    if any(b<=a for a,b in zip(times,times[1:])):raise ValueError('Trace clock not increasing')
    amp=profile['amplitude_nominal_A']
    scale=3.3/4096*187/18
    quality_run=fields(one('ABI_QUALITY_RUN '))
    if quality_run['invalid'] or quality_run['duplicate'] or abs(quality_run['count_delta']-quality_run['encoder_count_delta'])>.01:
        raise ValueError('Run ABI/count inconsistency')
    if any(v['adc_age_us']>2000 or abs(v['adc1_frames']-v['adc2_frames'])>1 for v in rows):
        raise ValueError('ADC stale/incoherent')
    if any(math.hypot(v['Uq_V'],v['Ud_V'])>.5001 or abs(v['iq_target_nominal_A'])>profile['iq_limit_nominal_A']+.000001 for v in rows):
        raise ValueError('Applied command bound violated')
    result.update(samples=len(rows),duration_us=times[-1],first_sample_us=times[0],
        vbus_min_V=min(v['vbus_adc']*scale for v in rows),vbus_max_V=max(v['vbus_adc']*scale for v in rows),
        max_vector_V=max(math.hypot(v['Uq_V'],v['Ud_V']) for v in rows),
        max_abs_iq_A=max(abs(v['iq_filtered_nominal_A']) for v in rows),
        max_abs_id_A=max(abs(v['id_filtered_nominal_A']) for v in rows),
        max_adc_age_us=max(v['adc_age_us'] for v in rows))
    if 'kind=motion' in one('BENCH_PROFILE '):
        # Dedicated analyzer adds trajectory, whole-loop statistics and holds.
        return result
    if 'kind=brake' in one('BENCH_PROFILE '):
        brake=fields(one('BENCH_BRAKE '));status=fields(one('BENCH_BRAKE_STATUS '))
        expected='STOP: bench active brake complete' if brake['active'] else 'STOP: bench coast complete'
        complete=(stopped==expected and guard['fault']==0 and status['invalid']==0 and status['phase']==3 and
                  brake.get('trigger_earliest_us',60000)<=status['transition_us']<400000 and
                  status['transition_us']+150000<=times[-1]<status['transition_us']+152000 and
                  times[0]<1000 and max(b-a for a,b in zip(times,times[1:]))<3000)
        transition=status['transition_us']
        braking=[v for v in rows if v['trace_us']>=transition and v['brake_phase']>=2]
        opposition=[v for v in braking if v['filtered_rad_s']>.2 and v['iq_filtered_nominal_A']<-.02]
        if not brake['active'] and any(not(int(v['bench_flags'])&4) or v['Uq_V']!=0 or v['Ud_V']!=0 for v in braking):
            raise ValueError('Coasting phase contains driven gates/voltage')
        if not brake['active'] and 'transition_voltage=after_disable' in one('BENCH_BRAKE ') and any(v['iq_target_nominal_A']!=0 for v in braking):
            raise ValueError('Coasting target was not cleared after disable')
        result.update(brake=brake,brake_status=status,complete=complete,opposition_samples=len(opposition),
                      max_motor_displacement_rad=max(abs(v['angle_rad']-fields(one('RUN_REFERENCE '))['start_angle_rad']) for v in rows))
        if braking:
            # Use adjacent 2ms samples consistently; no fictitious exact trigger angle.
            base=braking[0]
            if 'transition_trace=forced' in one('BENCH_BRAKE ') and not 0<=base['trace_us']-transition<=250:
                raise ValueError('Missing transition-triggered brake sample')
            result['brake_first_sample_us']=base['trace_us']
            result['max_forward_excursion_20ms_rad']=max_forward_excursion(braking)
        result['pass']=bool(g1 and complete and (not brake['active'] or len(opposition)>=2))
        return result
    if 'kind=position' in one('BENCH_PROFILE '):
        joint=fields(one('BENCH_JOINT '));direction=int(joint['direction']);degree=math.pi/180
        ramp=joint.get('ramp_us',500000)
        def reference(t):
            shape=0 if t<100000 else (t-100000)/ramp if t<100000+ramp else 1 if t<1300000 else (1300000+ramp-t)/ramp if t<1300000+ramp else 0
            return direction*.5*degree*shape
        # Target is sampled at the last <=1ms outer-controller update.
        command_time_tolerance=.5*degree*1100/ramp+1e-6
        if any(abs(v['joint_target_rad']-reference(v['trace_us']))>command_time_tolerance for v in rows):
            raise ValueError('Joint command differs from declared trajectory')
        reference_meta=fields(one('RUN_REFERENCE '))
        if any(abs(v['joint_measured_rad']-reference_meta['sensor_direction']*(v['angle_rad']-reference_meta['start_angle_rad'])/8)>1e-6 for v in rows):
            raise ValueError('Virtual joint angle scaling mismatch')
        holds=[]
        for first,last,goal in ((1000000,1300000,direction*.5*degree),(2400000,2701000,0)):
            segment=[v for v in rows if first<=v['trace_us']<=last]
            p={'start_us':first,'samples':len(segment),'pass':False}
            if segment:
                errors=[abs(v['joint_measured_rad']-goal)/degree for v in segment]
                positions=[v['joint_measured_rad']/degree for v in segment]
                limited=statistics.mean(abs(v['iq_target_nominal_A'])>=profile['iq_limit_nominal_A']*.993 for v in segment)
                p.update(max_error_deg=max(errors),ripple_peak_to_peak_deg=max(positions)-min(positions),
                         current_limit_fraction=limited,mean_iq_A=statistics.mean(v['iq_filtered_nominal_A'] for v in segment))
                p['pass']=len(segment)>=28 and max(errors)<=.2 and max(positions)-min(positions)<=.2 and limited<=.1
            holds.append(p)
        peak=max(abs(v['joint_measured_rad'])/degree for v in rows)
        result.update(joint=joint,holds=holds,max_joint_displacement_deg=peak,
                      max_abs_joint_velocity_deg_s=max(abs(v['joint_velocity_rad_s'])/degree for v in rows))
        result['complete']=(stopped=='STOP: bench joint sequence complete' and guard['fault']==0 and
                            2700000<=times[-1]<2702000 and times[0]<1000 and
                            max(b-a for a,b in zip(times,times[1:]))<11000)
        result['pass']=bool(g1 and result['complete'] and peak<=.8 and all(p['pass'] for p in holds))
        return result
    complete=(stopped=='STOP: bench pulse sequence complete' and guard['fault']==0 and
              300000<=times[-1]<302000 and times[0]<8000 and
              max((b-a for a,b in zip(times,times[1:])),default=0)<2000)
    pulses=[]
    for onset,sign in ((40000,1),(110000,-1),(180000,1),(250000,-1)):
        segment=[v for v in rows if onset<=v['trace_us']<onset+20000 and abs(v['iq_target_nominal_A']-sign*amp)<1e-6]
        p={'nominal_onset_us':onset,'sign':sign,'samples':len(segment),'pass':False}
        if segment:
            actual=segment[0]['trace_us'];late=[v for v in segment if v['trace_us']>=onset+15000]
            reached=[a['trace_us']-actual for a,b in zip(segment,segment[1:]) if
                     sign*a['iq_filtered_nominal_A']>=.9*amp and sign*b['iq_filtered_nominal_A']>=.9*amp]
            mean=statistics.mean(v['iq_filtered_nominal_A'] for v in late) if late else None
            saturated=sum(int(v['bench_flags'])&1!=0 for v in segment)
            p.update(actual_onset_us=actual,t90_us=min(reached) if reached else None,
                     late_mean_iq_A=mean,late_mean_error_A=None if mean is None else sign*amp-mean,
                     vector_limited_samples=saturated,
                     slew_limited_samples=sum(int(v['bench_flags'])&2!=0 for v in segment))
            p['pass']=(len(segment)>=17 and onset<=actual<onset+1500 and reached and min(reached)<=10000
                       and len(late)>=3 and abs(sign*amp-mean)<=.03 and sign*mean>0 and saturated==0)
            p['pass']=bool(p['pass'])
        pulses.append(p)
    result.update(complete=complete,pulses=pulses)
    result['pass']=bool(g1 and complete and all(v['pass'] for v in pulses))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('log');p.add_argument('--output')
    args=p.parse_args();result=analyze(args.log);rendered=json.dumps(result,indent=2,ensure_ascii=False)
    if args.output:
        with open(args.output,'x',encoding='utf-8') as f:f.write(rendered+'\n')
    print(rendered)
