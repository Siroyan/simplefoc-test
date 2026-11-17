"""Validate a frozen DWT edge tail and summarize its unfiltered ABI intervals."""
import csv
import io
import math
import re
from edge_estimator import verify_edge_clock_check
from speed_range import verify_speed_guard_config,verify_velocity_stats_config

CONFIG='RUN_EDGE_CONFIG capacity=256 clock=DWT_elapsed origin=beginTrace storage=manual_edges raw=us,count,AB UART=after_stop'
ACTIVE='RUN_EDGE_ACTIVE window_us=10000 guard_LPF_s=0.0500 guard_limit_rad_s=5 voltage_limit_V=0.500 target_rad_s=1.000 mode=open_loop'
HIGH_ACTIVE='RUN_EDGE_ACTIVE window_us=2000 guard_LPF_s=0.0020 guard_limit_rad_s=8.5 voltage_limit_V=0.500 target_rad_s=6.283185 mode=open_loop'
HIGH_ESTIMATOR='ESTIMATOR observer=rolling_LS4ms min_sample_us=150 warmup_span_us=3000 guard=original_window2ms_LPF2ms guard_limit_rad_s=8.5 guard_profile=reference_envelope'
HIGH_REFERENCE='REFERENCE_RAMP duration_us=2000000 target_rad_s=6.283185 trace_target=final_goal origin=trace_start'
HIGH_DRIVE='RUN_EDGE_DRIVE speed_PID=off current_PI=off voltage_cap_V=.5 reference_ramp_us=2000000 running_UART=off'
FAST_REFERENCE=HIGH_REFERENCE.replace('2000000','800000')
FAST_DRIVE=HIGH_DRIVE.replace('2000000','800000')
BEGIN='RUN_EDGES_BEGIN: gates OFF; frozen manual storage; elapsed DWT microseconds'
OPEN='RUN OPEN: ramp speed to 1.000 rad/s at 0.500 V; stop after 3 seconds.'
ALIGN='ALIGN_PROFILE cycles=3 step_ms=4 voltage_V=.7 max_alignment_us=0'
HIGH_OPEN='RUN OPEN: ramp speed to 6.283185 rad/s at 0.500 V; stop after 3 seconds.'
STEPS=(0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0)


def verify_run_edge_config(line):
    if line!=CONFIG:raise RuntimeError('Raw edge capture configuration mismatch; stopping')


def verify_run_edge_active(line,high=False):
    if line!=(HIGH_ACTIVE if high else ACTIVE):raise RuntimeError('Raw observation drive/independent guard mismatch; stopping')


def verify_run_edge_observer(line):
    values=dict(re.findall(r'([A-Za-z_]+)=([^ ]+)',line))
    if any(values.get(k)!=v for k,v in {'observer_only':'1','max_age_us':'2000','deviation_delay_us':'20000'}.items()):
        raise RuntimeError('Raw observation requires the original ADC/VBUS guard; stopping')


def verify_run_edge_alignment(line):
    if line!=ALIGN:raise RuntimeError('Raw capture alignment differs from the established .7V profile; stopping')


def parse_run_edges(lines):
    configs=[l for l in lines if l.startswith('RUN_EDGE_CONFIG ')]
    if not configs:
        if any(l.startswith(('RUN_EDGE_ACTIVE ','RUN_EDGE_SUMMARY ','RUN_EDGES_BEGIN','RUN_EDGES_END')) for l in lines):
            raise ValueError('Raw edge records lack their configuration')
        return None
    active=[l for l in lines if l.startswith('RUN_EDGE_ACTIVE ')]
    checks=[l for l in lines if l.startswith('EDGE_CLOCK_CHECK ')]
    summaries=[l for l in lines if l.startswith('RUN_EDGE_SUMMARY ')]
    if len(configs)!=1 or len(active)!=1 or len(checks)!=1 or len(summaries)!=1 or lines.count(BEGIN)!=1 or lines.count('RUN_EDGES_END')!=1:
        raise ValueError('Unique complete raw-edge configuration, state, clock and dump required')
    high=active[0]==HIGH_ACTIVE
    verify_run_edge_config(configs[0]);verify_run_edge_active(active[0],high);verify_edge_clock_check(checks[0])
    alignments=[l for l in lines if l.startswith('ALIGN_PROFILE ')]
    if len(alignments)!=1:raise ValueError('One alignment profile required')
    verify_run_edge_alignment(alignments[0])
    if high:
        fast=FAST_REFERENCE in lines
        guards=[l for l in lines if l.startswith('SPEED_GUARD_CONFIG ')]
        stats=[l for l in lines if l.startswith('VELOCITY_STATS_CONFIG ')]
        if len(guards)!=1 or len(stats)!=1 or HIGH_ESTIMATOR not in lines or (FAST_REFERENCE if fast else HIGH_REFERENCE) not in lines or (FAST_DRIVE if fast else HIGH_DRIVE) not in lines:
            raise ValueError('Higher open-loop profile lacks its independent guard/reference/observer/drive configuration')
        verify_speed_guard_config(guards[0]);verify_velocity_stats_config(stats[0],800000 if fast else 2000000)
    observer=[l for l in lines if l.startswith('CURRENT_CONFIG ')]
    if len(observer)!=1:raise ValueError('Raw capture lacks protected ADC observation')
    verify_run_edge_observer(observer[0])
    if (HIGH_OPEN if high else OPEN) not in lines:raise ValueError('Raw observation is not the selected open-loop profile')
    start=lines.index(BEGIN);end=lines.index('RUN_EDGES_END')
    if start>=end or not any(l.startswith('STOP:') for l in lines[:start]):raise ValueError('Raw edges must be frozen and sent after stop')
    if lines[start+1]!='edge_us,signed_count,AB_state':raise ValueError('Raw edge CSV header mismatch')
    values={k:int(v) for k,v in re.findall(r'([A-Za-z_]+)=(-?\d+)',summaries[0])}
    if set(values)!={'total_edges','retained','origin_count','origin_AB','end_count','end_us'}:raise ValueError('Raw edge summary fields mismatch')
    if not 0<=values['total_edges']<=4294967295 or values['retained']!=min(256,values['total_edges']) or not 0<=values['origin_AB']<=3 or not 0<=values['end_us']<=3010000 or not all(-2147483648<=values[k]<=2147483647 for k in ('origin_count','end_count')):
        raise ValueError('Raw edge summary bounds mismatch')
    rows=list(csv.DictReader(io.StringIO('\n'.join(lines[start+1:end]))))
    if len(rows)!=values['retained']:raise ValueError('Raw edge tail was truncated')
    try:rows=[{k:int(v) for k,v in r.items()} for r in rows]
    except (ValueError,TypeError):raise ValueError('Raw edge rows must contain integers')
    if any(set(r)!={'edge_us','signed_count','AB_state'} or not 0<=r['edge_us']<=values['end_us'] or not -2147483648<=r['signed_count']<=2147483647 or not 0<=r['AB_state']<=3 for r in rows):
        raise ValueError('Raw edge row bounds mismatch')
    if rows and rows[-1]['signed_count']!=values['end_count']:raise ValueError('Frozen final count differs from last raw edge')
    if not rows and values['end_count']!=values['origin_count']:raise ValueError('Empty edge capture changed count')
    quality=next((l for l in lines if l.startswith('ABI_QUALITY_RUN ')),None)
    if quality is None:raise ValueError('Independent ABI run counters required')
    abi={k:float(v) for k,v in re.findall(r'([A-Za-z_]+)=(-?[\d.]+)',quality)}
    if abi.get('callbacks')!=values['total_edges'] or abi.get('count_delta')!=values['end_count']-values['origin_count']:
        raise ValueError('Raw edge totals and independent run counts disagree')
    previous={'edge_us':0,'signed_count':values['origin_count'],'AB_state':values['origin_AB']} if values['total_edges']<=256 else None
    invalid=duplicate=0;intervals=[];groups={str(s):[] for s in range(4)}
    for index,row in enumerate(rows):
        if previous is not None:
            changed=previous['AB_state']^row['AB_state'];step=STEPS[previous['AB_state']*4+row['AB_state']]
            invalid+=changed==3;duplicate+=changed==0
            if ((row['signed_count']-previous['signed_count'])&0xffffffff)!=(step&0xffffffff):raise ValueError('Raw AB state and signed count disagree')
            if index>0:
                dt=row['edge_us']-previous['edge_us']
                if dt<=0:raise ValueError('DWT edge timestamps must increase')
                intervals.append(dt);groups[str(previous['AB_state'])].append(dt)
        previous=row
    tick=math.tau/4000
    speed=None
    if len(rows)>1:
        span=rows[-1]['edge_us']-rows[0]['edge_us']
        mean=(rows[-1]['signed_count']-rows[0]['signed_count'])*tick*1e6/span
        error=sum(dt*(((b['signed_count']-a['signed_count'])*tick*1e6/dt)-mean)**2 for a,b,dt in zip(rows,rows[1:],intervals))/span
        speed={'span_s':span/1e6,'mean_encoder_rad_s':mean,'one_edge_RMS_about_mean_rad_s':math.sqrt(error)}
    return {'configuration':configs[0],'active':active[0],'clock_check':checks[0],'target_rad_s':6.283185 if high else 1,'summary':values,
            'tail_invalid_transitions':invalid,'tail_duplicate_callbacks':duplicate,
            'interval_us_range':[min(intervals),max(intervals)] if intervals else None,
            'mean_interval_us_by_previous_AB':{k:sum(v)/len(v) if v else None for k,v in groups.items()},
            'tail_speed':speed,'rows':rows,
            'note':'Only the retained 256 ISR observations, with the same ABI signal. IRQ latency is included. Interval noise and real shaft/angle ripple are not separated; no independent speed or angle calibration.'}
