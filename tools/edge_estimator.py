"""Require the explicitly selected edge-timing experiment and its stop summary."""
import math
import re

ESTIMATOR='ESTIMATOR controller=edge_period4_phase guard=original_window2ms_LPF2ms guard_limit_rad_s=8.5 guard_profile=reference_envelope'
CONFIG='EDGE_ESTIMATOR_CONFIG counts_per_turn=4000 period_intervals=4 phase_gain_s_inv=200 prediction_count_bound=1 sample_gap_limit_us=5000 no_edge_decay=gap_to_2gap isr=integer_only'
CONFIG_DWT=CONFIG+' clock=DWT_elapsed cycles_per_us=170'


def verify_edge_estimator_config(line,dwt=False):
    if line!=(CONFIG_DWT if dwt else CONFIG):
        raise RuntimeError('Edge estimator configuration mismatch; stopping')


def verify_edge_clock_check(line):
    prefix='EDGE_CLOCK_CHECK advancing=1 gates_off=1 cycles_per_us=170 '
    match=re.fullmatch(re.escape(prefix)+r'core_regressions=(\d+) maximum_core_backstep_us=(\d+)',line)
    if not match or (int(match[1])==0)!=(int(match[2])==0):
        raise RuntimeError('Disabled cycle-clock check missing or inconsistent; stopping')


def parse_edge_estimator(lines):
    estimators=[line for line in lines if line.startswith('ESTIMATOR controller=edge_')]
    configs=[line for line in lines if line.startswith('EDGE_ESTIMATOR_CONFIG ')]
    summaries=[line for line in lines if line.startswith('EDGE_ESTIMATOR_SUMMARY ')]
    if not estimators and not configs and not summaries:return None
    if estimators!=[ESTIMATOR] or len(configs)!=1 or len(summaries)!=1:
        raise ValueError('Complete unique edge-estimator metadata and summary required')
    dwt=configs[0]==CONFIG_DWT
    verify_edge_estimator_config(configs[0],dwt)
    clock_checks=[line for line in lines if line.startswith('EDGE_CLOCK_CHECK ')]
    faults=[line for line in lines if line.startswith('EDGE_CLOCK_FAULT ')]
    if dwt:
        if len(clock_checks)!=1 or len(faults)!=1:raise ValueError('Cycle-clock check and fault status required')
        verify_edge_clock_check(clock_checks[0])
        fault={key:int(value) for key,value in re.findall(r'([A-Za-z_]+)=(\d+)',faults[0])}
        if set(fault)!={'reason','now_us','edge_us','age_us','update_gap_us'} or fault['reason'] not in (0,1,2,3):raise ValueError('Malformed edge-clock fault status')
        if 'STOP: 3-second test complete' in lines and fault['reason']:raise ValueError('Completed run has an edge-clock fault')
    values={key:float(value) for key,value in re.findall(r'([A-Za-z_]+)=([^ ]+)',summaries[0])}
    if set(values)!={'invalid','samples','signed_count','span_us'} or not all(math.isfinite(v) and int(v)==v for v in values.values()):
        raise ValueError('Malformed edge-estimator summary')
    if values['invalid'] not in (0,1) or not 0<=values['samples']<=5 or not -2147483648<=values['signed_count']<=2147483647:
        raise ValueError('Invalid edge-estimator summary state')
    if (values['samples']<2 and values['span_us']!=0) or (values['samples']>=2 and not 0<values['span_us']<=2000000):
        raise ValueError('Edge-estimator span mismatch')
    if 'STOP: 3-second test complete' in lines and (values['invalid'] or values['samples']!=5):
        raise ValueError('Completed run lacks a valid full edge window')
    return {'configuration':configs[0],'clock_check':clock_checks[0] if clock_checks else None,
            'clock_fault':faults[0] if faults else None,**values}
